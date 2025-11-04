/* rc522_stm32_driver.c - corrected, bare-metal, register-level */

#include "stm32f4xx.h"
#include <stdint.h>

/* Pin / port config */
#define RC522_CS_GPIO_PORT    GPIOA
#define RC522_CS_PIN          4
#define RC522_RST_GPIO_PORT   GPIOB
#define RC522_RST_PIN         0

/* MFRC522 registers (subset) */
#define CommandReg      0x01
#define CommIEnReg      0x02
#define CommIrqReg      0x04
#define DivIrqReg       0x05
#define ErrorReg        0x06
#define FIFODataReg     0x09
#define FIFOLevelReg    0x0A
#define ControlReg      0x0C
#define BitFramingReg   0x0D
#define Status2Reg      0x08
#define ModeReg         0x11
#define TxControlReg    0x14
#define TModeReg        0x2A
#define TPrescalerReg   0x2B
#define TReloadRegL     0x2D
#define VersionReg      0x37
#define CRCResultRegL   0x22
#define CRCResultRegM   0x21

/* Commands */
#define PCD_IDLE        0x00
#define PCD_TRANSCEIVE  0x0C
#define PCD_AUTHENT     0x0E
#define PCD_CALCCRC     0x03
#define PCD_SOFTRESET   0x0F

/* PICC commands */
#define PICC_REQIDL     0x26
#define PICC_REQALL     0x52
#define PICC_ANTICOLL   0x93
#define PICC_SELECTTAG  0x93
#define PICC_HALT       0x50
#define PICC_READ       0x30
#define PICC_WRITE      0xA0

/* Misc */
#define MAX_LEN         16

/* Return codes */
#define MI_OK           0
#define MI_NOTAGERR     1
#define MI_ERR          2

/* Function prototypes */
void RC522_Init(void);
uint8_t RC522_ReadReg(uint8_t addr);
void RC522_WriteReg(uint8_t addr, uint8_t val);
void RC522_SetBitMask(uint8_t reg, uint8_t mask);
void RC522_SetBitMask(uint8_t reg, uint8_t mask);
uint8_t RC522_Request(uint8_t reqMode, uint8_t *TagType);
uint8_t RC522_Anticoll(uint8_t *serNum);
uint8_t RC522_SelectTag(uint8_t *serNum);
uint8_t RC522_Auth(uint8_t authMode, uint8_t BlockAddr, uint8_t *key, uint8_t *serNum);
uint8_t RC522_Read(uint8_t blockAddr, uint8_t *recvData);
uint8_t RC522_Write(uint8_t blockAddr, uint8_t *writeData);
void RC522_Halt(void);
void RC522_GPIO_Init(void);
void RC522_SPI_Init(void);
uint8_t RC522_ToCard(uint8_t command, uint8_t *sendData, uint8_t sendLen,
                     uint8_t *backData, uint16_t *backLenBits);
void RC522_CalculateCRC(uint8_t *data, uint8_t length, uint8_t *result);
void RC522_WriteFIFO(uint8_t *data, uint8_t length);
void RC522_ReadFIFO(uint8_t *data, uint8_t length);

/* CS helpers */
void RC522_CS_Select(void)  { RC522_CS_GPIO_PORT->BSRR = (1U << (RC522_CS_PIN + 16)); } /* low */
void RC522_CS_Unselect(void){ RC522_CS_GPIO_PORT->BSRR = (1U << RC522_CS_PIN); }         /* high */

/* Configure SPI pins (AF5) and CS/RST GPIOs. */
void RC522_GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    /* PA5/PA6/PA7 -> AF5 (SPI1) */
    GPIOA->MODER &= ~((3U << (5*2)) | (3U << (6*2)) | (3U << (7*2)));
    GPIOA->MODER |=  ((2U << (5*2)) | (2U << (6*2)) | (2U << (7*2)));
    GPIOA->AFR[0] &= ~((0xFU << (5*4)) | (0xFU << (6*4)) | (0xFU << (7*4)));
    GPIOA->AFR[0] |=  ((5U << (5*4)) | (5U << (6*4)) | (5U << (7*4)));
    GPIOA->OSPEEDR |= ((3U << (5*2)) | (3U << (6*2)) | (3U << (7*2)));
    GPIOA->PUPDR &= ~((3U << (5*2)) | (3U << (6*2)) | (3U << (7*2)));
    GPIOA->OTYPER &= ~((1U << 5) | (1U << 7));

    /* PA4 -> CS as output */
    GPIOA->MODER &= ~(3U << (RC522_CS_PIN * 2));
    GPIOA->MODER |=  (1U << (RC522_CS_PIN * 2));
    GPIOA->OSPEEDR |= (3U << (RC522_CS_PIN * 2));
    GPIOA->PUPDR &= ~(3U << (RC522_CS_PIN * 2));
    GPIOA->OTYPER &= ~(1U << RC522_CS_PIN);
    RC522_CS_Unselect();

    /* PB0 -> RST as output */
    GPIOB->MODER &= ~(3U << (RC522_RST_PIN * 2));
    GPIOB->MODER |=  (1U << (RC522_RST_PIN * 2));
    GPIOB->OSPEEDR |= (3U << (RC522_RST_PIN * 2));
    GPIOB->PUPDR &= ~(3U << (RC522_RST_PIN * 2));
    GPIOB->OTYPER &= ~(1U << RC522_RST_PIN);
    GPIOB->BSRR = (1U << RC522_RST_PIN); /* RST = 1 */
}

/* Initialize SPI1 as master (basic) */
void RC522_SPI_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI1->CR1 = 0;
    SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1->CR1 |= SPI_CR1_SPE;
}

/* Reset RC522 (soft reset) and perform basic configuration */
void RC522_Init(void) {
    RC522_GPIO_Init();
    RC522_SPI_Init();

    /* Hardware reset pulse if RST pin is wired */
    RC522_RST_GPIO_PORT->BSRR = (1U << (RC522_RST_PIN + 16)); /* RST = 0 */
    for (volatile int i = 0; i < 10000; ++i) __NOP();
    RC522_RST_GPIO_PORT->BSRR = (1U << RC522_RST_PIN); /* RST = 1 */

    /* Soft reset via CommandReg */
    RC522_WriteReg(CommandReg, PCD_SOFTRESET);
    for (int i = 0; i < 1000; ++i) {
        uint8_t cmd = RC522_ReadReg(CommandReg);
        if ((cmd & (1<<4)) == 0) break;
    }

    RC522_WriteReg(TModeReg, 0x8D);
    RC522_WriteReg(TPrescalerReg, 0x3E);
    RC522_WriteReg(TReloadRegL, 30);
    RC522_WriteReg(ModeReg, 0x3D);

    RC522_SetBitMask(TxControlReg, 0x03);

    (void)RC522_ReadReg(VersionReg); /* optional read to verify comms */
}

/* Low-level SPI byte transfer (single definition) */
uint8_t SPI1_TransferByte(uint8_t tx) {
    while (!(SPI1->SR & SPI_SR_TXE));
    *((__IO uint8_t *)&SPI1->DR) = tx;
    while (!(SPI1->SR & SPI_SR_RXNE));
    return *((__IO uint8_t *)&SPI1->DR);
}

/* Register access */
uint8_t RC522_ReadReg(uint8_t addr) {
    uint8_t val;
    RC522_CS_Select();
    SPI1_TransferByte((addr << 1) | 0x80);
    val = SPI1_TransferByte(0x00);
    RC522_CS_Unselect();
    return val;
}

void RC522_WriteReg(uint8_t addr, uint8_t val) {
    RC522_CS_Select();
    SPI1_TransferByte((addr << 1) & 0x7E);
    SPI1_TransferByte(val);
    RC522_CS_Unselect();
}

void RC522_SetBitMask(uint8_t reg, uint8_t mask) {
    uint8_t tmp = RC522_ReadReg(reg);
    RC522_WriteReg(reg, tmp | mask);
}

void RC522_ClearBitMask(uint8_t reg, uint8_t mask) {
    uint8_t tmp = RC522_ReadReg(reg);
    RC522_WriteReg(reg, tmp & (~mask));
}

/* FIFO helpers */
void RC522_WriteFIFO(uint8_t *data, uint8_t length) {
    for (uint8_t i = 0; i < length; i++) RC522_WriteReg(FIFODataReg, data[i]);
}
void RC522_ReadFIFO(uint8_t *data, uint8_t length) {
    for (uint8_t i = 0; i < length; i++) data[i] = RC522_ReadReg(FIFODataReg);
}

/* CRC using chip */
void RC522_CalculateCRC(uint8_t *data, uint8_t length, uint8_t *result /*2 bytes*/) {
    RC522_ClearBitMask(DivIrqReg, 0x04);      /* clear CRCIRq */
    RC522_SetBitMask(FIFOLevelReg, 0x80);     /* clear FIFO */
    RC522_WriteFIFO(data, length);
    RC522_WriteReg(CommandReg, PCD_CALCCRC);

    uint16_t i = 5000;
    uint8_t n;
    do {
        n = RC522_ReadReg(DivIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x04));

    result[0] = RC522_ReadReg(CRCResultRegL);
    result[1] = RC522_ReadReg(CRCResultRegM);
}

/* Generic ToCard routine */
uint8_t RC522_ToCard(uint8_t command, uint8_t *sendData, uint8_t sendLen,
                     uint8_t *backData, uint16_t *backLenBits) {
    uint8_t status = MI_ERR;
    uint8_t irqEn = 0x00, waitIRq = 0x00, n = 0;
    uint8_t lastBits = 0;
    uint16_t i;

    if (command == PCD_AUTHENT) { irqEn = 0x12; waitIRq = 0x10; }
    else if (command == PCD_TRANSCEIVE) { irqEn = 0x77; waitIRq = 0x30; }

    RC522_WriteReg(CommIEnReg, irqEn | 0x80);
    RC522_ClearBitMask(CommIrqReg, 0x80);
    RC522_SetBitMask(FIFOLevelReg, 0x80); /* flush FIFO */

    for (uint8_t k = 0; k < sendLen; k++) RC522_WriteReg(FIFODataReg, sendData[k]);

    RC522_WriteReg(CommandReg, command);
    if (command == PCD_TRANSCEIVE) RC522_SetBitMask(BitFramingReg, 0x80);

    i = 2000;
    do {
        n = RC522_ReadReg(CommIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitIRq));

    RC522_ClearBitMask(BitFramingReg, 0x80);

    if (i != 0) {
        if (!(RC522_ReadReg(ErrorReg) & 0x1B)) {
            status = MI_OK;
            if (n & irqEn & 0x01) status = MI_NOTAGERR;

            if (command == PCD_TRANSCEIVE) {
                n = RC522_ReadReg(FIFOLevelReg);
                lastBits = RC522_ReadReg(ControlReg) & 0x07;
                if (lastBits) *backLenBits = (n - 1) * 8 + lastBits;
                else *backLenBits = n * 8;

                if (n == 0) n = 1;
                if (n > MAX_LEN) n = MAX_LEN;

                if (backData) {
                    for (uint8_t j = 0; j < n; j++) backData[j] = RC522_ReadReg(FIFODataReg);
                } else {
                    /* discard FIFO if caller doesn't want data */
                    for (uint8_t j = 0; j < n; j++) (void)RC522_ReadReg(FIFODataReg);
                }
            }
        } else status = MI_ERR;
    } else status = MI_NOTAGERR;

    return status;
}

/* RC522_Request - inline style (simple REQA/WUPA) */
uint8_t RC522_Request(uint8_t reqMode, uint8_t *TagType) {
    uint8_t status = MI_ERR;
    uint8_t irqEn = 0x77, waitIRq = 0x30;

    RC522_WriteReg(CommIEnReg, irqEn | 0x80);
    RC522_ClearBitMask(CommIrqReg, 0x80);
    RC522_SetBitMask(FIFOLevelReg, 0x80);

    uint8_t data[1] = { reqMode };
    RC522_WriteFIFO(data, 1);

    RC522_WriteReg(CommandReg, PCD_TRANSCEIVE);
    RC522_SetBitMask(BitFramingReg, 0x80);

    uint16_t i = 2000;
    uint8_t n;
    do {
        n = RC522_ReadReg(CommIrqReg);
        i--;
    } while ((i != 0) && !(n & waitIRq));

    RC522_ClearBitMask(BitFramingReg, 0x80);

    if (i != 0) {
        if (!(RC522_ReadReg(ErrorReg) & 0x1B)) {
            status = MI_OK;
            TagType[0] = RC522_ReadReg(FIFODataReg);
            TagType[1] = RC522_ReadReg(FIFODataReg);
        } else status = MI_ERR;
    } else status = MI_NOTAGERR;

    return status;
}

/* Anti-collision */
uint8_t RC522_Anticoll(uint8_t *serNum) {
    uint8_t status;
    uint8_t serNumCheck = 0;
    uint16_t backBits;
    uint8_t buffer[2] = { PICC_ANTICOLL, 0x20 };
    uint8_t backData[MAX_LEN];

    RC522_WriteReg(BitFramingReg, 0x00);
    status = RC522_ToCard(PCD_TRANSCEIVE, buffer, 2, backData, &backBits);

    if (status == MI_OK) {
        for (uint8_t i = 0; i < 4; i++) serNumCheck ^= backData[i];
        if (serNumCheck != backData[4]) return MI_ERR;
        for (uint8_t i = 0; i < 5; i++) serNum[i] = backData[i];
    }
    return status;
}

/* Select tag */
uint8_t RC522_SelectTag(uint8_t *serNum) {
    uint8_t buffer[9], crc[2], backData[3];
    uint16_t backBits;

    buffer[0] = PICC_SELECTTAG;
    buffer[1] = 0x70;
    for (uint8_t i = 0; i < 5; i++) buffer[i + 2] = serNum[i];

    RC522_CalculateCRC(buffer, 7, crc);
    buffer[7] = crc[0];
    buffer[8] = crc[1];

    uint8_t status = RC522_ToCard(PCD_TRANSCEIVE, buffer, 9, backData, &backBits);
    if ((status == MI_OK) && (backBits == 0x18)) return MI_OK;
    return MI_ERR;
}

/* Authenticate */
uint8_t RC522_Auth(uint8_t authMode, uint8_t BlockAddr, uint8_t *key, uint8_t *serNum) {
    uint8_t buffer[12];
    uint16_t recvBits;

    buffer[0] = authMode;
    buffer[1] = BlockAddr;
    for (uint8_t i = 0; i < 6; i++) buffer[i + 2] = key[i];
    for (uint8_t i = 0; i < 4; i++) buffer[i + 8] = serNum[i];

    uint8_t status = RC522_ToCard(PCD_AUTHENT, buffer, 12, NULL, &recvBits);
    if ((status != MI_OK) || !(RC522_ReadReg(Status2Reg) & 0x08)) {
        return MI_ERR;
    }
    return MI_OK;
}

/* Read block */
uint8_t RC522_Read(uint8_t blockAddr, uint8_t *recvData) {
    uint8_t buffer[4];
    uint16_t unLen;
    buffer[0] = PICC_READ;
    buffer[1] = blockAddr;
    RC522_CalculateCRC(buffer, 2, (uint8_t*)&buffer[2]); /* CRC into buffer[2..3] */

    uint8_t status = RC522_ToCard(PCD_TRANSCEIVE, buffer, 4, recvData, &unLen);
    if (status == MI_OK) return MI_OK;
    return MI_ERR;
}

/* Write block */
uint8_t RC522_Write(uint8_t blockAddr, uint8_t *writeData) {
    uint8_t buffer[4], ackBuf[4];
    uint16_t recvBits;

    buffer[0] = PICC_WRITE;
    buffer[1] = blockAddr;
    RC522_CalculateCRC(buffer, 2, (uint8_t*)&buffer[2]);

    uint8_t status = RC522_ToCard(PCD_TRANSCEIVE, buffer, 4, ackBuf, &recvBits);
    if ((status != MI_OK) || (recvBits != 4) || ((ackBuf[0] & 0x0F) != 0x0A)) return MI_ERR;

    /* prepare 18-byte payload: 16 bytes + 2 CRC */
    uint8_t txBuf[18];
    for (uint8_t i = 0; i < 16; i++) txBuf[i] = writeData[i];
    RC522_CalculateCRC(txBuf, 16, &txBuf[16]);

    status = RC522_ToCard(PCD_TRANSCEIVE, txBuf, 18, ackBuf, &recvBits);
    if ((status != MI_OK) || (recvBits != 4) || ((ackBuf[0] & 0x0F) != 0x0A)) return MI_ERR;
    return MI_OK;
}

/* Halt */
void RC522_Halt(void) {
    uint8_t buffer[4];
    uint16_t unLen;
    buffer[0] = PICC_HALT;
    buffer[1] = 0x00;
    RC522_CalculateCRC(buffer, 2, (uint8_t*)&buffer[2]);
    RC522_ToCard(PCD_TRANSCEIVE, buffer, 4, NULL, &unLen);
}

/* Example main loop */
int main(void) {
    uint8_t status;
    uint8_t tagType[2];
    uint8_t serNum[5];
    uint8_t data[16];
    uint8_t keyA[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    uint8_t writeBlock[16] = {'R','C','5','2','2',' ','T','E','S','T',' ','B','L','O','C','K'};

    RC522_Init();

    while (1) {
        status = RC522_Request(PICC_REQIDL, tagType);
        if (status == MI_OK) {
            status = RC522_Anticoll(serNum);
            if (status == MI_OK) {
                status = RC522_SelectTag(serNum);
                if (status == MI_OK) {
                    status = RC522_Auth(0x60, 8, keyA, serNum); /* Key A */
                    if (status == MI_OK) {
                        if (RC522_Read(8, data) == MI_OK) {
                            /* handle data */
                        }
                        RC522_Write(8, writeBlock); /* write 16 bytes */
                        RC522_Halt();
                    }
                }
            }
        }
        for (volatile int d = 0; d < 200000; ++d) __NOP();
    }
    return 0;
}
