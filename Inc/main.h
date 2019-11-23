/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
//PIN CONFIG
#define MISO 12
#define MOSI 11
#define CS 9
#define SCL 13

//CC1120 STATUS BYTES
#define MIDLE 0b0001000
#define MRX 0b0011000
#define MTX 0b0101000
#define RXERR 0b1101000
#define TXERR 0b1111000
#define MCAL 0b100000

//STROBE COMMAND ADDRESSES AND FIFO ADDRESSES
#define SRES 0x30 //chip reset
#define SRX 0x34 //chip receive mode
#define STX 0x35 //chip transmit mode
#define SIDLE 0x36 //chip idle
#define SFRX 0x3A //chip rx fifo flush (when in error state)
#define SFTX 0x3B //chip tx fifo flush
#define SNOP 0x3D //chip do nothing (for status checking)
#define FIFOW 0b00111111 //address for tx fifo
#define FIFOR 0b10111111 //address for rx fifo

//EXTENDED REGISTERS
#define IF_MIX_CFG 0x00
#define FREQOFF_CFG 0x01
#define TOC_CFG 0x02
#define MARC_SPARE 0x03
#define ECG_CFG 0x04
#define CFM_DATA_CFG 0x05
#define EXT_CTRL 0x06
#define RCCAL_FINE 0x07
#define RCCAL_COARSE 0x08
#define RCCAL_OFFSET 0x09
#define FREQOFF1 0x0A
#define FREQOFF0 0x0B
#define FREQ2 0x0C //freq configuration register. refer to data sheet for formula to be used
#define FREQ1 0x0D 
#define FREQ0 0x0E 
#define IF_ADC2 0x0F
#define IF_ADC1 0x10
#define IF_ADC0 0x11
#define FS_DIG1 0x12
#define FS_DIG0 0x13
#define FS_CAL3 0x14
#define FS_CAL2 0x15
#define FS_CAL1 0x16
#define FS_CAL0 0x17
#define FS_CHP 0x18
#define FS_DIVTWO 0x19
#define FS_DSM1 0x1A
#define FS_DSM0 0x1B
#define FS_DVC1 0x1C
#define FS_DVC0 0x1D
#define FS_LBI 0x1E
#define FS_PFD 0x1F
#define FS_PRE 0x20
#define FS_REG_DIV_CML 0x21
#define FS_SPARE 0x22
#define FS_VCO4 0x23
#define FS_VCO3 0x24
#define FS_VCO2 0x25
#define FS_VCO1 0x26
#define FS_VCO0 0x27
#define GBIAS6 0x28
#define GBIAS5 0x29
#define GBIAS4 0x2A
#define GBIAS3 0x2B
#define GBIAS2 0x2C
#define GBIAS1 0x2D
#define GBIAS0 0x2E
#define IFAMP 0x2F
#define LNA 0x30
#define RXMIX 0x31
#define XOSC5 0x32
#define XOSC4 0x33
#define XOSC3 0x34
#define XOSC2 0x35
#define XOSC1 0x36
#define XOSC0 0x37
#define ANALOG_SPARE 0x38
#define PA_CFG3 0x39
#define WOR_TIME1 0x64
#define WOR_TIME0 0x65
#define WOR_CAPTURE1 0x66
#define WOR_CAPTURE0 0x67
#define BIST 0x68
#define DCFILTOFFSET_I1 0x69
#define DCFILTOFFSET_I0 0x6A
#define DCFILTOFFSET_Q1 0x6B
#define DCFILTOFFSET_Q0 0x6C
#define IQIE_I1 0x6D
#define IQIE_I0 0x6E
#define IQIE_Q1 0x6F
#define IQIE_Q0 0x70
#define RSSI1 0x71
#define RSSI0 0x72
#define MARCSTATE 0x73
#define LQI_VAL 0x74
#define PQT_SYNC_ERR 0x75
#define DEM_STATUS 0x76
#define FREQOFF_EST1 0x77
#define FREQOFF_EST0 0x78
#define AGC_GAIN3 0x79
#define AGC_GAIN2 0x7A
#define AGC_GAIN1 0x7B
#define AGC_GAIN0 0x7C
#define CFM_RX_DATA_OUT 0x7D
#define CFM_TX_DATA_IN 0x7E
#define ASK_SOFT_RX_DATA 0x7F
#define RNDGEN 0x80
#define MAGN2 0x81
#define MAGN1 0x82
#define MAGN0 0x83
#define ANG1 0x84
#define ANG0 0x85
#define CHFILT_I2 0x86
#define CHFILT_I1 0x87
#define CHFILT_I0 0x88
#define CHFILT_Q2 0x89
#define CHFILT_Q1 0x8A
#define CHFILT_Q0 0x8B
#define GPIO_STATUS 0x8C
#define FSCAL_CTRL 0x8D
#define PHASE_ADJUST 0x8E
#define PARTNUMBER 0x8F
#define PARTVERSION 0x90
#define SERIAL_STATUS 0x91
#define MODEM_STATUS1 0x92
#define MODEM_STATUS0 0x93
#define MARC_STATUS1 0x94
#define MARC_STATUS0 0x95
#define PA_IFAMP_TEST 0x96
#define FSRF_TEST 0x97
#define PRE_TEST 0x98
#define PRE_OVR 0x99
#define ADC_TEST 0x9A
#define DVC_TEST 0x9B
#define ATEST 0x9C
#define ATEST_LVDS 0x9D
#define ATEST_MODE 0x9E
#define XOSC_TEST1 0x9F
#define XOSC_TEST0 0xA0
#define RXFIRST 0xD2
#define TXFIRST 0xD3
#define RXLAST 0xD4
#define TXLAST 0xD5
#define NUM_TXBYTES 0xD6
#define NUM_RXBYTES 0xD7
#define FIFO_NUM_TXBYTES 0xD8
#define FIFO_NUM_RXBYTES 0xD9 

//REGULAR REGISTERS
#define IOCFG3 0x00 //gpio3 pin cfg; analog transfer, invert output, output selection (default pkt_sync_rtxtx)
#define IOCFG2 0x01 //gpio2 pin cfg; '' (default pkt_crc_ok)
#define IOCFG1 0x02 //gpio1 pin cfg; '' (default HIGHZ, acts as SO when CSn is low)
#define IOCFG0 0x03 //gpio0 pin cfgl '' (default EXT OSC EN)
#define SYNC3 0x04 //sync3-0 is sync word
#define SYNC2 0x05
#define SYNC1 0x06
#define SYNC0 0x07
#define SYNC_CFG1 0x08 //sync word detection settings; preamble quality check, sync word threshold
#define SYNC_CFG0 0x09 //sync word length settings; sync word mode, sync word bit check
#define DEVIATION_M 0x10 //frequency deviation
#define MODCFG_DEV_E 0x0B
#define DCFILT_CFG 0x0C
#define PREAMBLE_CFG1 0x0D //preamble settings; number of preamble bites, preamble byte cfg
#define PREAMBLE_CFG0 0x0E //preamble settings; preamble detection enable, pqt timeout, soft pqt
#define FREQ_IF_CFG 0x0F //Frequency synthesizer cfg
#define IQIC 0x10
#define CHAN_BW 0x11
#define MDMCFG1 0x12
#define MDMCFG0 0x13
#define SYMBOL_RATE2 0x14
#define SYMBOL_RATE1 0x15
#define SYMBOL_RATE0 0x16
#define AGC_REF 0x17
#define AGC_CS_THR 0x18
#define AGC_GAIN_ADJUST 0x19
#define AGC_CFG3 0x1A
#define AGC_CFG2 0x1B
#define AGC_CFG1 0x1C
#define AGC_CFG0 0x1D
#define FIFO_CFG 0x1E //tx and rx fifo sizes
#define DEV_ADDR 0x1F
#define SETTLING_CFG 0x20 //frequency settling cfg
#define FS_CFG 0x21 //
#define WOR_CFG1 0x22
#define WOR_CFG0 0x23
#define WOR_EVENT0_MSB 0x24
#define WOR_EVENT0_LSB 0x25
#define PKT_CFG2 0x26
#define PKT_CFG1 0x27
#define PKT_CFG0 0x28 //packet length cfg, packet bits, UART mode, UART swap
#define RFEND_CFG1 0x29 //mode which radio enters after receiving good packet, syncword timeout, rx timeout qualifier
#define RFEND_CFG0 0x2A
#define PA_CFG2 0x2B
#define PA_CFG1 0x2C
#define PA_CFG0 0x2D
#define PKT_LEN 0x2E //packet length if chosen to be fixed in PKT_CFG0 


uint8_t bytei; //byte received from spi, mainly used for status check
uint8_t datai; //byte received from spi, contains data from issued command, such as value in a register

typedef struct{
  uint8_t addr;
  uint8_t dat;
} registerSetting_t;

uint8_t cc_tx_wr_reg (uint16_t addr, uint8_t data);

#define CC_EXT_ADD 0x2F00

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define GPIO_PIN_CS_Pin GPIO_PIN_15
#define GPIO_PIN_CS_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
