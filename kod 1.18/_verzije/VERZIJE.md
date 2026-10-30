# Verzije koda

Svaki folder ovdje je **potpun snimak** izvornog koda u jednom trenutku.
Sadrži cio `Core`, `KOD eurobot.ioc` i linker skriptu.

Snimci su nezavisni od gita. Ne treba ti nijedna git komanda da se vratiš.

---

| verzija | datum | provjereno na robotu | šta je novo |
|---|---|---|---|
| **v1.0** | 30.10.2026. | da | prva ispravna osnova: rad/s, PID sa D članom, CAN ID-jevi 100–152, enkoderski režim se postavlja direktnim upisom u registre |
| **v1.1** | 30.10.2026. | da | dijagnostika u `can_comm.c`: broji primljene poruke, greške magistrale, stanje bus-off |
| **v1.2** | 30.10.2026. | **ne, tek flešovano** | uključen prekid za FDCAN, prijem CAN poruka ranije uopšte nije radio |

Detaljan opis svake verzije je u njenom `OPIS.txt`.

---

## Kako se vraća

**Najlakše:** reci Claude-u *„vrati mi na verziju 1"* ili *„vrati na 1.1"*.

**Ručno:**

1. Kopiraj `_verzije/<verzija>/Core` preko `Core` u projektu.
2. Kopiraj `_verzije/<verzija>/KOD eurobot.ioc` preko istog fajla u korijenu.
3. U STM32CubeIDE: `Project` → `Clean`, pa `Build Project`.

---

## Pravila

- Nova verzija se pravi **prije** svake veće izmjene, ne poslije.
- Kolona „provjereno na robotu" se popunjava tek kad stvarno proradi.
- Broj poslije tačke raste za male dopune, broj prije tačke za veće promjene.
- Ovaj folder ne utiče na bildovanje. CubeIDE kompajlira samo `Core` i `Drivers`.
