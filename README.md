sources:
https://lastminuteengineers.com/how-rfid-works-rc522-arduino-tutorial /
https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf
wiring:

RC522 VCC → 3.3 V

RC522 GND → GND

RC522 SDA (or SS/CS) → PA4 (or any GPIO you'll use for CS)

RC522 SCK → PA5 (SPI1_SCK)

RC522 MOSI → PA7 (SPI1_MOSI)

RC522 MISO → PA6 (SPI1_MISO)

RC522 RST → any GPIO (e.g., PB0)
