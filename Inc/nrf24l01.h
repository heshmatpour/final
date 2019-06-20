#ifndef __nrf24l01
#define __nrf24l01
#include "stm32f4xx_hal.h"
typedef enum {
    NRF_CONFIG      = 0x00,
    NRF_EN_AA       = 0x01,
    NRF_EN_RXADDR   = 0x02,
    NRF_SETUP_AW    = 0x03,
    NRF_SETUP_RETR  = 0x04,
    NRF_RF_CH       = 0x05,
    NRF_RF_SETUP    = 0x06,
    NRF_STATUS      = 0x07,
    NRF_OBSERVE_TX  = 0x08,
    NRF_CD          = 0x09,
    NRF_RX_ADDR_P0  = 0x0A,
    NRF_RX_ADDR_P1  = 0x0B,
    NRF_RX_ADDR_P2  = 0x0C,
    NRF_RX_ADDR_P3  = 0x0D,
    NRF_RX_ADDR_P4  = 0x0E,
    NRF_RX_ADDR_P5  = 0x0F,
    NRF_TX_ADDR     = 0x10,
    NRF_RX_PW_P0    = 0x11,
    NRF_RX_PW_P1    = 0x12,
    NRF_RX_PW_P2    = 0x13,
    NRF_RX_PW_P3    = 0x14,
    NRF_RX_PW_P4    = 0x15,
    NRF_RX_PW_P5    = 0x16,
    NRF_FIFO_STATUS = 0x17,
    NRF_DYNPD       = 0x1C,
    NRF_FEATURE     = 0x1D
} NRF_REGISTER;


/* Commands */
typedef enum {
    NRF_CMD_R_REGISTER         = 0x00,
    NRF_CMD_W_REGISTER         = 0x20,
    NRF_CMD_R_RX_PAYLOAD       = 0x61,
    NRF_CMD_W_TX_PAYLOAD       = 0xA0,
    NRF_CMD_FLUSH_TX           = 0xE1,
    NRF_CMD_FLUSH_RX           = 0xE2,
    NRF_CMD_REUSE_TX_PL        = 0xE3,
    NRF_CMD_ACTIVATE           = 0x50,
    NRF_CMD_R_RX_PL_WID        = 0x60,
    NRF_CMD_W_ACK_PAYLOAD      = 0xA8,
    NRF_CMD_W_TX_PAYLOAD_NOACK = 0xB0,
    NRF_CMD_NOP                = 0xFF
} NRF_COMMAND;


typedef enum {
//    NRF_DATA_RATE_250KBPS = 1, for nrf24l01 not nrf24l01+
    NRF_DATA_RATE_1MBPS   = 0,
    NRF_DATA_RATE_2MBPS   = 2
} NRF_DATA_RATE;


typedef enum {
    NRF_TX_PWR_M18dBm = 0,
    NRF_TX_PWR_M12dBm = 1,
    NRF_TX_PWR_M6dBm  = 2,
    NRF_TX_PWR_0dBm   = 3
} NRF_TX_PWR;

typedef enum {
    NRF_CRC_WIDTH_1B = 0,
    NRF_CRC_WIDTH_2B = 1
} NRF_CRC_WIDTH;

typedef enum {
    NRF_ADDR_WIDTH_3 = 1,
    NRF_ADDR_WIDTH_4 = 2,
    NRF_ADDR_WIDTH_5 = 3
} NRF_ADDR_WIDTH;

typedef enum {
    NRF_STATE_RX = 1,
    NRF_STATE_TX = 0
} NRF_TXRX_STATE;

typedef enum { NRF_OK, NRF_ERROR, NRF_INVALID_ARGUMENT } NRF_RESULT;

typedef struct {

    NRF_DATA_RATE  data_rate;
    NRF_TX_PWR     tx_power;
    NRF_CRC_WIDTH  crc_width;
    NRF_ADDR_WIDTH addr_width;
		NRF_TXRX_STATE state;
	
	  uint8_t       payload_length;
    uint8_t       retransmit_count;
    uint8_t       retransmit_delay;
    uint8_t       rf_channel;
		uint8_t 			busy_flag;

    /* Usr interface, Rx/Tx Buffer */
    uint8_t* rx_buffer;
    uint8_t* tx_buffer;
	  const uint8_t* rx_address;
    const uint8_t* tx_address;
	
    SPI_HandleTypeDef* spi;
    uint32_t           spi_timeout;

    GPIO_TypeDef* csn_port;
    uint16_t      csn_pin;

    GPIO_TypeDef* ce_port;
    uint16_t      ce_pin;

    GPIO_TypeDef* irq_port;
    uint16_t      irq_pin;

} nrf24l01_dev;


//typedef struct {
//    nrf24l01_config config;

//    volatile uint8_t        tx_busy;
//    volatile NRF_RESULT     tx_result;
//    volatile uint8_t        rx_busy;
//    volatile NRF_TXRX_STATE state;

//} nrf24l01;

/* Initialization routine */
NRF_RESULT NRF_Init(nrf24l01_dev* dev);

/* EXTI Interrupt Handler */
void NRF_IRQ_Handler(nrf24l01_dev* dev);

/* Blocking Data Sending / Receiving FXs */
NRF_RESULT NRF_SendPacket(nrf24l01_dev* dev, uint8_t* data);
NRF_RESULT NRF_ReceivePacket(nrf24l01_dev* dev, uint8_t* data);

/* Non-Blocking Data Sending / Receiving FXs */
NRF_RESULT NRF_PushPacket(nrf24l01_dev* dev, uint8_t* data);
NRF_RESULT NRF_PullPacket(nrf24l01_dev* dev, uint8_t* data);

/* LOW LEVEL STUFF (you don't have to look in here...)*/
NRF_RESULT NRF_SendCommand(nrf24l01_dev* dev, uint8_t cmd, uint8_t* tx, uint8_t* rx, uint8_t len);
/* CMD */
NRF_RESULT NRF_ReadRegister(nrf24l01_dev* dev, uint8_t reg, uint8_t* data);
NRF_RESULT NRF_WriteRegister(nrf24l01_dev* dev, uint8_t reg, uint8_t* data);
NRF_RESULT NRF_ReadRXPayload(nrf24l01_dev* dev, uint8_t* data);
NRF_RESULT NRF_WriteTXPayload(nrf24l01_dev* dev, uint8_t* data);
NRF_RESULT NRF_FlushTX(nrf24l01_dev* dev);
NRF_RESULT NRF_FlushRX(nrf24l01_dev* dev);

/* RF_SETUP */
NRF_RESULT NRF_SetDataRate(nrf24l01_dev* dev, NRF_DATA_RATE rate);
NRF_RESULT NRF_SetTXPower(nrf24l01_dev* dev, NRF_TX_PWR pwr);
NRF_RESULT NRF_SetCCW(nrf24l01_dev* dev, uint8_t activate);

/* STATUS */
NRF_RESULT NRF_ClearInterrupts(nrf24l01_dev* dev);

/* RF_CH */
NRF_RESULT NRF_SetRFChannel(nrf24l01_dev* dev, uint8_t ch);

/* SETUP_RETR */
NRF_RESULT NRF_SetRetransmittionCount(nrf24l01_dev* dev, uint8_t count);
NRF_RESULT NRF_SetRetransmittionDelay(nrf24l01_dev* dev, uint8_t delay);

/* SETUP_AW */
NRF_RESULT NRF_SetAddressWidth(nrf24l01_dev* dev, NRF_ADDR_WIDTH width);

/* EN_RXADDR */
NRF_RESULT NRF_EnableRXPipe(nrf24l01_dev* dev, uint8_t pipe);

/* EN_AA */
NRF_RESULT NRF_EnableAutoAcknowledgement(nrf24l01_dev* dev, uint8_t pipe);

/* CONFIG */
NRF_RESULT NRF_EnableCRC(nrf24l01_dev* dev, uint8_t activate);
NRF_RESULT NRF_SetCRCWidth(nrf24l01_dev* dev, NRF_CRC_WIDTH width);
NRF_RESULT NRF_PowerUp(nrf24l01_dev* dev, uint8_t powerUp);
NRF_RESULT NRF_RXTXControl(nrf24l01_dev* dev, NRF_TXRX_STATE rx);
NRF_RESULT NRF_EnableRXDataReadyIRQ(nrf24l01_dev* dev, uint8_t activate);
NRF_RESULT NRF_EnableTXDataSentIRQ(nrf24l01_dev* dev, uint8_t activate);
NRF_RESULT NRF_EnableMaxRetransmitIRQ(nrf24l01_dev* dev, uint8_t activate);

/* RX_ADDR_P0 */
NRF_RESULT NRF_SetRXAddress_P0(nrf24l01_dev* dev, uint8_t* address); // 5bytes of address

/* TX_ADDR */
NRF_RESULT NRF_SetTXAddress(nrf24l01_dev* dev, uint8_t* address); // 5bytes of address

/* RX_PW_P0 */
NRF_RESULT NRF_SetRXPayloadWidth_P0(nrf24l01_dev* dev, uint8_t width);

/* FEATURE */




#endif
