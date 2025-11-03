/* rc522_stm32_driver.c
 * Minimal bare-metal RC522 (MFRC522) driver init for STM32F4xx
 * Provides:
 *   void RC522_GPIO_Init(void);
 *   void RC522_SPI_Init(void);
 *   void RC522_Init(void);
 *
 * Adapt pin choices (PA4..PA7, PB0) to your board if needed.
 * This file uses direct register access for STM32F4 series.
 */

#include "stm32f4xx.h"
#include <stdint.h>

/* --- Pin configuration (change if needed) ---
 * SPI1: PA5=SCK, PA6=MISO, PA7=MOSI
 * CS  : PA4 (GPIO output, active low)
 * RST : PB0 (optional hardware reset)
 */
#define RC522_CS_GPIO_PORT    GPIOA
#define RC522_CS_PIN          4
#define RC522_RST_GPIO_PORT   GPIOB
#define RC522_RST_PIN         0

/* --- MFRC522 registers / commands (small subset) --- */
#define CommandReg      0x01
#define CommIEnReg      0x02
#define CommIrqReg      0x04
#define FIFODataReg     0x09
#define FIFOLevelReg    0x0A
#define BitFramingReg   0x0D
#define ModeReg         0x11
#define TxControlReg    0x14
#define TModeReg        0x2A
#define TPrescalerReg   0x2B
#define TReloadRegL     0x2D
#define VersionReg      0x37

#define PCD_SOFTRESET   0x0F
#define PCD_TRANSCIEVE  0x0C

/* --- Low-level SPI helpers (used by init) --- */
static inline void RC522_CS_Select(void)  { RC522_CS_GPIO_PORT->BSRR = (1U << (RC522_CS_PIN + 16)); } // low
static inline void RC522_CS_Unselect(void){ RC522_CS_GPIO_PORT->BSRR = (1U << RC522_CS_PIN); } // high

/* Blocking 8-bit transfer on SPI1 */
static uint8_t SPI1_TransferByte(uint8_t tx) {
    while (!(SPI1->SR & SPI_SR_TXE)) ;
    *((__IO uint8_t *)&SPI1->DR) = tx;
    while (!(SPI1->SR & SPI_SR_RXNE)) ;
    return *((__IO uint8_t *)&SPI1->DR);
}

/* --- Register read/write for MFRC522 --- */
static uint8_t MFRC522_ReadReg(uint8_t addr) {
    uint8_t val;
    RC522_CS_Select();
    SPI1_TransferByte((addr << 1) | 0x80); // READ: address<<1 | 0x80
    val = SPI1_TransferByte(0x00);
    RC522_CS_Unselect();
    return val;
}

static void MFRC522_WriteReg(uint8_t addr, uint8_t val) {
    RC522_CS_Select();
    SPI1_TransferByte((addr << 1) & 0x7E); // WRITE: address<<1, LSB=0
    SPI1_TransferByte(val);
    RC522_CS_Unselect();
}

static void MFRC522_SetBitMask(uint8_t reg, uint8_t mask) {
    uint8_t tmp = MFRC522_ReadReg(reg);
    MFRC522_WriteReg(reg, tmp | mask);
}

static void MFRC522_ClearBitMask(uint8_t reg, uint8_t mask) {
    uint8_t tmp = MFRC522_ReadReg(reg);
    MFRC522_WriteReg(reg, tmp & (~mask));
}

/* --- Public functions requested by user --- */

/* Configure SPI pins (AF5) and CS/RST GPIOs. */
void RC522_GPIO_Init(void) {
    /* Enable GPIOA and GPIOB clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    /* PA5, PA6, PA7 -> AF (AF5 for SPI1) */
    // Clear moderators then set AF mode (10)
    GPIOA->MODER &= ~((3U << (5*2)) | (3U << (6*2)) | (3U << (7*2)));
    GPIOA->MODER |=  ((2U << (5*2)) | (2U << (6*2)) | (2U << (7*2)));

    // Set AF5 for PA5, PA6, PA7
    GPIOA->AFR[0] &= ~((0xFU << (5*4)) | (0xFU << (6*4)) | (0xFU << (7*4)));
    GPIOA->AFR[0] |=  ((5U << (5*4)) | (5U << (6*4)) | (5U << (7*4)));

    // Configure PA5/PA6/PA7 as high speed, push-pull, no pull-up/down
    GPIOA->OSPEEDR |= ((3U << (5*2)) | (3U << (6*2)) | (3U << (7*2)));
    GPIOA->PUPDR &= ~((3U << (5*2)) | (3U << (6*2)) | (3U << (7*2)));
    GPIOA->OTYPER &= ~((1U << 5) | (1U << 7)); // push-pull (MISO is input so OTYPER bit ignored)

    /* PA4 -> CS pin, general purpose output push-pull */
    GPIOA->MODER &= ~(3U << (RC522_CS_PIN * 2));
    GPIOA->MODER |=  (1U << (RC522_CS_PIN * 2)); // output
    GPIOA->OSPEEDR |= (3U << (RC522_CS_PIN * 2));
    GPIOA->PUPDR &= ~(3U << (RC522_CS_PIN * 2));
    GPIOA->OTYPER &= ~(1U << RC522_CS_PIN);
    // Set CS high (inactive)
    RC522_CS_Unselect();

    /* PB0 -> RST pin (optional). Configure as output and drive high */
    GPIOB->MODER &= ~(3U << (RC522_RST_PIN * 2));
    GPIOB->MODER |=  (1U << (RC522_RST_PIN * 2)); // output
    GPIOB->OSPEEDR |= (3U << (RC522_RST_PIN * 2));
    GPIOB->PUPDR &= ~(3U << (RC522_RST_PIN * 2));
    GPIOB->OTYPER &= ~(1U << RC522_RST_PIN);
    GPIOB->BSRR = (1U << RC522_RST_PIN); // RST = 1
}

/* Initialize SPI1 as master, CPOL=0, CPHA=0, 8-bit, fPCLK/16, SSM (software NSS) */
void RC522_SPI_Init(void) {
    /* Enable SPI1 clock */
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    /* Make sure SPI is disabled while configuring */
    SPI1->CR1 = 0;

    /* Master mode */
    SPI1->CR1 |= SPI_CR1_MSTR;
    /* Baud rate: fPCLK/16 (BR[2:0] = 011 -> BR0|BR1 set) */
    SPI1->CR1 |= (SPI_CR1_BR_0 | SPI_CR1_BR_1);
    /* 8-bit data frame: default; CR1 MSTR+SPE are sufficient for basic transfers */

    /* Software slave management, set SSI to 1 to avoid MODF */
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;

    /* CPOL = 0, CPHA = 0 (SPI mode 0) -> defaults to 0 so no change needed */

    /* Enable SPI peripheral */
    SPI1->CR1 |= SPI_CR1_SPE;
}

/* Reset RC522 (soft reset) and perform basic configuration */
void RC522_Init(void) {
    /* Ensure GPIO and SPI are initialized */
    RC522_GPIO_Init();
    RC522_SPI_Init();

    /* Optional: hardware reset pulse using RST pin if available */
    // Drive RST low briefly then high
    RC522_RST_GPIO_PORT->BSRR = (1U << (RC522_RST_PIN + 16)); // RST = 0
    for (volatile int i = 0; i < 10000; ++i) __NOP();
    RC522_RST_GPIO_PORT->BSRR = (1U << RC522_RST_PIN); // RST = 1

    /* Soft reset via CommandReg */
    MFRC522_WriteReg(CommandReg, PCD_SOFTRESET);
    // wait until PowerDown bit (bit4) cleared
    for (int i = 0; i < 1000; ++i) {
        uint8_t cmd = MFRC522_ReadReg(CommandReg);
        if ((cmd & (1<<4)) == 0) break;
    }

    /* Timer and modulation setup (common recommended values) */
    MFRC522_WriteReg(TModeReg, 0x8D);
    MFRC522_WriteReg(TPrescalerReg, 0x3E);
    MFRC522_WriteReg(TReloadRegL, 30);
    MFRC522_WriteReg(ModeReg, 0x3D); // CRC preset and other mode bits

    /* Turn antenna on (TxControlReg bits 0..1) */
    MFRC522_SetBitMask(TxControlReg, 0x03);

    /* Optional: read VersionReg to verify communication */
    uint8_t version = MFRC522_ReadReg(VersionReg);
    (void)version; // use for debug (e.g., print over UART)
}

/* End of file */
