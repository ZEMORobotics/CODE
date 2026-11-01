/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM kretacka - brzina levog/desnog tocka preko CAN-a
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Lib/ADC.h"
#include "Lib/motor.h"
#include "Lib/motor2.h"
#include "Lib/comm.h"

#include "Lib/board_config.h"
#include "Lib/motors.h"
#include "Lib/encoders.h"
#include "Lib/odometry.h"
#include "Lib/PID.h"
#include "Lib/can_comm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
pid_gains_t wheel_gains;                   /* jedan skup Kp/Ki/Kd za oba tocka */
wheel_pid_t pid_left;
wheel_pid_t pid_right;

/* Menja ih TIM7 prekid, gledaju se iz debagera - volatile. */
volatile float meas_speed_left  = 0.0f;   /* rad/s, filtrirano */
volatile float meas_speed_right = 0.0f;
volatile float cmd_left  = 0.0f;          /* % maks. PWM-a, -100..+100 */
volatile float cmd_right = 0.0f;
volatile int sys = 0;





/* ===== TEST: vozi pravo, bez ROS2 i bez ijedne CAN poruke =====
 * 1 = test ukljucen, 0 = normalan rad preko CAN-a.
 * PAZNJA: robot krece SAM cim se ploca upali. */
#define TEST_DRIVE_ENABLE      0
#define TEST_DRIVE_SPEED_RADS  5.0f   /* oba tocka isto, ~0.20 m/s */
#define TEST_DRIVE_DELAY_MS    5000   /* koliko miruje pre polaska */
#define TEST_DRIVE_RUN_MS      3000   /* koliko vozi, pa stane zauvek */




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/* ===== TEST 3: cita enkoderske pinove kao obicne ulaze, bez tajmera ===== */
volatile uint8_t  enc1_a = 0, enc1_b = 0, enc2_a = 0, enc2_b = 0;
volatile uint32_t enc1_edges = 0, enc2_edges = 0;

static void test_enc_pins_init(void)
{
    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = ENC1_A_Pin; HAL_GPIO_Init(ENC1_A_GPIO_Port, &g);
    g.Pin = ENC1_B_Pin; HAL_GPIO_Init(ENC1_B_GPIO_Port, &g);
    g.Pin = ENC2_A_Pin; HAL_GPIO_Init(ENC2_A_GPIO_Port, &g);
    g.Pin = ENC2_B_Pin; HAL_GPIO_Init(ENC2_B_GPIO_Port, &g);
}

static void test_enc_pins_task(void)
{
    static uint8_t a1_prev = 0xFF, a2_prev = 0xFF;

    enc1_a = (HAL_GPIO_ReadPin(ENC1_A_GPIO_Port, ENC1_A_Pin) == GPIO_PIN_SET);
    enc1_b = (HAL_GPIO_ReadPin(ENC1_B_GPIO_Port, ENC1_B_Pin) == GPIO_PIN_SET);
    enc2_a = (HAL_GPIO_ReadPin(ENC2_A_GPIO_Port, ENC2_A_Pin) == GPIO_PIN_SET);
    enc2_b = (HAL_GPIO_ReadPin(ENC2_B_GPIO_Port, ENC2_B_Pin) == GPIO_PIN_SET);

    if (enc1_a != a1_prev) { a1_prev = enc1_a; enc1_edges++; }
    if (enc2_a != a2_prev) { a2_prev = enc2_a; enc2_edges++; }
}



/* ===========================================================================
 *  KONTROLNA PETLJA - TIM7 @ CONTROL_FREQ_HZ (500 Hz)
 *  enkoderi -> brzina tocka [rad/s] -> PID po tocku [%] -> PWM;
 *  svakih FEEDBACK_FREQ_HZ oznaci slanje feedback-a.
 * ===========================================================================*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM7)
        return;

    static uint16_t feedback_div = 0;
    static uint16_t alive_div = 0;
    sys++;

    odometry_update();                       /* razlika brojaca -> rad/s (+ filter) */
    meas_speed_left  = get_wheel_speed_left();
    meas_speed_right = get_wheel_speed_right();

    if (can_motors_on)
    {
        cmd_left  = wheel_pid_update(&pid_left,  can_ref_speed_left,  meas_speed_left);
        cmd_right = wheel_pid_update(&pid_right, can_ref_speed_right, meas_speed_right);
        PWM_SetCommand_Left(cmd_left);
        PWM_SetCommand_Right(cmd_right);
    }
    else
    {
        cmd_left = cmd_right = 0.0f;
        PWM_Stop();
    }

    if (++feedback_div >= (CONTROL_FREQ_HZ / FEEDBACK_FREQ_HZ))
    {
        feedback_div = 0;
        can_comm_feedback_due();
    }

    if (++alive_div >= (CONTROL_FREQ_HZ / 2))   /* 1 Hz treptanje = petlja ziva */
    {
        alive_div = 0;
        HAL_GPIO_TogglePin(ALIVE_GPIO_Port, ALIVE_Pin);
    }
}





#if TEST_DRIVE_ENABLE
/* Upisuje istu referencu u oba tocka. Regulator, odometrija, filteri i PWM
 * rade potpuno normalno - menja se samo odakle referenca dolazi. */
static void test_drive_task(void)
{
    static bool     started = false;
    static uint32_t t_start = 0;

    if (!started) {
        started = true;
        t_start = HAL_GetTick();
    }

    uint32_t dt = HAL_GetTick() - t_start;

    if (dt < TEST_DRIVE_DELAY_MS) {
        can_ref_speed_left  = 0.0f;                    /* miruj, skloni se */
        can_ref_speed_right = 0.0f;
    } else if (dt < (uint32_t)(TEST_DRIVE_DELAY_MS + TEST_DRIVE_RUN_MS)) {
        can_ref_speed_left  = TEST_DRIVE_SPEED_RADS;   /* vozi pravo */
        can_ref_speed_right = TEST_DRIVE_SPEED_RADS;
    } else {
        can_ref_speed_left  = 0.0f;                    /* gotovo */
        can_ref_speed_right = 0.0f;
    }
}
#endif




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_UART7_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */

  /* TIM7 na CONTROL_FREQ_HZ bez obzira sta je u CubeMX-u */
  __HAL_TIM_SET_PRESCALER(&htim7, TIM7_PRESCALER);
  __HAL_TIM_SET_AUTORELOAD(&htim7,
      (TIM7_INPUT_CLK_HZ / (TIM7_PRESCALER + 1)) / CONTROL_FREQ_HZ - 1);
  /* PSC/ARR su baferovani - primeni ih odmah i obrisi laznu update zastavicu */
  HAL_TIM_GenerateEvent(&htim7, TIM_EVENTSOURCE_UPDATE);
  __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);

  /* Jedan skup koeficijenata, oba regulatora pokazuju na njega */
  wheel_pid_gains_init(&wheel_gains, PID_DEFAULT_KP, PID_DEFAULT_KI, PID_DEFAULT_KD,
                       PID_D_LPF_FC_HZ, (float)CONTROL_FREQ_HZ);
  wheel_pid_init(&pid_left,  &wheel_gains, PID_COMMAND_LIMIT_PCT, PID_INTEGRATOR_LIMIT_PCT);
  wheel_pid_init(&pid_right, &wheel_gains, PID_COMMAND_LIMIT_PCT, PID_INTEGRATOR_LIMIT_PCT);

  /* motor_supply_voltage je za sada MOTOR_SUPPLY_VOLTAGE_DEFAULT (30 V -> 60 %).
   * Kad se doda ADC: PWM_SetSupplyVoltage(ADC_ReadVbat()) pre PWM_Init(). */
  PWM_Init();
  encoder_init();



//  test_enc_pins_init();   /* preuzima pinove od tajmera, cita ih kao ulaze */


  odometry_init();

  can_comm_init();
  can_comm_startup_flush();      /* isprazni RX, posalji nule, tek onda RX prekidi */

  HAL_TIM_Base_Start_IT(&htim7); /* od ovog trenutka radi PID */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      can_comm_task();   /* feedback 100 Hz, IR burst, timeout komande */
      test_enc_pins_task();


#if TEST_DRIVE_ENABLE
      test_drive_task(); /* mora POSLE can_comm_task() */
#endif

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
