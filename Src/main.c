/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define SFSTXON                  0x31      /*  SFSTXON - Enable and calibrate frequency synthesizer. */
#define CC1120_TX_MAX_FRAME_LEN 255
#define CC1120_TX_FIFO_SIZE 128
#define CC1120_TXFIFO_THR 63
#define CC1120_TXFIFO_IRQ_THR (127 - CC1120_TXFIFO_THR)
#define CC1120_TXFIFO_AVAILABLE_BYTES (CC1120_TX_FIFO_SIZE - CC1120_TXFIFO_IRQ_THR + 2)
#define CC1120_FIXED_PKT_LEN 0x00
#define SINGLE_TXFIFO            0x3F      /*  TXFIFO  - Single access to Transmit FIFO */
#define BURST_TXFIFO             0x7F      /*  TXFIFO  - Burst access to Transmit FIFO  */
#define SINGLE_RXFIFO            0xBF      /*  RXFIFO  - Single access to Receive FIFO  */
#define BURST_RXFIFO             0xFF      /*  RXFIFO  - Burst access to Receive FIFO  */

#define LQI_CRC_OK_BM            0x80
#define LQI_EST_BM               0x7F

static uint8_t tx_frag_buf[2 + CC1120_TX_FIFO_SIZE];

typedef struct {
  uint8_t cw_on;
  uint32_t duration_ms;
} cw_pulse_t;

// TX register settings
// static const registerSetting_t TX_preferredSettings[] =
//   {
//     { IOCFG3, 0x06 },
//     { IOCFG2, 0x02 },
//     { IOCFG1, 0x40 },
//     { IOCFG0, 0x40 },
//     { SYNC3, 0x00 },
//     { SYNC2, 0x00 },
//     { SYNC1, 0x7A },
//     { SYNC0, 0x0E },
//     { SYNC_CFG1, 0x0B },
//     { SYNC_CFG0, 0x03 }, /* No SYNC word. It is generated on the MPU and handled manually */
//     { DCFILT_CFG, 0x1C },
//     { PREAMBLE_CFG1, 0x00 }, /* No preamble. It is generated on the MPU and handled manually */
//     { PREAMBLE_CFG0, 0x0A }, /* No preamble. It is generated on the MPU and handled manually */
//     { IQIC, 0xC6 },
//     { CHAN_BW, 0x08 },
//     { MDMCFG0, 0x05 },
//     { SYMBOL_RATE2, 0x01},
//     { AGC_REF, 0x20 },
//     { AGC_CS_THR, 0x19 },
//     { AGC_CFG1, 0xA9 },
//     { FIFO_CFG, CC1120_TXFIFO_THR },
//     { SETTLING_CFG, 0x0b },
//     { FS_CFG, 0x14 },
//     { PKT_CFG1, 0x00 },
//     { PKT_CFG0, CC1120_FIXED_PKT_LEN },
//     { PA_CFG2, 0x3F },  //4-PACFG2 0X26, 6dBm 0x2B 2dbm 0x22,8 DBM 2F
//   { PA_CFG0, 0x7D },
//   { PKT_LEN, 0xFF },
//   { IF_MIX_CFG, 0x00 },
//   { FREQOFF_CFG, 0x22 },
//   { FREQ2, 0x6C },
//   { FREQ1, 0xF1 },
//   { FREQ0, 0x2F },
//   { FS_DIG1, 0x00 },
//   { FS_DIG0, 0x5F },
//   { FS_CAL1, 0x40 },
//   { FS_CAL0, 0x0E },
//   { FS_DIVTWO, 0x03 },
//   { FS_DSM0, 0x33 },
//   { FS_DVC0, 0x17 },
//   { FS_PFD, 0x50 },
//   { FS_PRE, 0x6E },
//   { FS_REG_DIV_CML, 0x14 },
//   { FS_SPARE, 0xAC },
//   { FS_VCO4, 0x13 },
//   { FS_VCO1, 0xAC },
//   { FS_VCO0, 0xB4 },
//   { XOSC5, 0x0E },
//   { XOSC1, 0x03 },
//   { DCFILTOFFSET_I1, 0xF8 },
//   { DCFILTOFFSET_I0, 0x39 },
//   { DCFILTOFFSET_Q1, 0x0E },
//   { DCFILTOFFSET_Q0, 0x9B },
//   { CFM_DATA_CFG, 0x00 },
//   { IQIE_I1, 0xEF },
//   { IQIE_I0, 0xDE },
//   { IQIE_Q1, 0x02 },
//   { IQIE_Q0, 0x2F },
//   { AGC_GAIN1, 0x13 },
//   { SERIAL_STATUS, 0x10 },
//   { MODCFG_DEV_E, 0x0B } };
static const registerSetting_t TX_preferredSettings[]= 
{
  {IOCFG3,            0xB0},
  {IOCFG2,            0x08},
  {IOCFG1,            0xB0},
  {IOCFG0,            0x09},
  {SYNC_CFG1,         0x0B},
  {DCFILT_CFG,        0x1C},
  {PREAMBLE_CFG1,     0x00},
  {IQIC,              0xC6},
  {MDMCFG1,           0x06},
  {MDMCFG0,           0x05},
  {AGC_REF,           0x20},
  {AGC_CS_THR,        0x19},
  {AGC_CFG1,          0xA9},
  {AGC_CFG0,          0xCF},
  {FIFO_CFG,          0x00},
  {SETTLING_CFG,      0x03},
  {FS_CFG,            0x14},
  {PKT_CFG2,          0x05},
  {PKT_CFG1,          0x00},
  {PKT_CFG0,          0x20},
  {PA_CFG2,           0x6B},
  {IF_MIX_CFG,        0x00},
  {FREQOFF_CFG,       0x22},
  {FREQ2,             0x6C},
  {FREQ1,             0x80},
  {FS_DIG1,           0x00},
  {FS_DIG0,           0x5F},
  {FS_CAL1,           0x40},
  {FS_CAL0,           0x0E},
  {FS_DIVTWO,         0x03},
  {FS_DSM0,           0x33},
  {FS_DVC0,           0x17},
  {FS_PFD,            0x50},
  {FS_PRE,            0x6E},
  {FS_REG_DIV_CML,    0x14},
  {FS_SPARE,          0xAC},
  {FS_VCO0,           0xB4},
  {XOSC5,             0x0E},
  {XOSC1,             0x03},
  {PARTNUMBER,        0x48},
  {PARTVERSION,       0x21},
  {SERIAL_STATUS,     0x08},
  {MODEM_STATUS1,     0x10},
};

const registerSetting_t CW_preferredSettings[] =
  {
    { IOCFG3, 0xB0 },
    { IOCFG2, 0x08 },
    { IOCFG1, 0xB0 },
    { IOCFG0, 0x09 },
    { SYNC_CFG1, 0x0B },
    { DEVIATION_M, 0x26 },
    { MODCFG_DEV_E, 0x05 },
    { DCFILT_CFG, 0x13 },
    { PREAMBLE_CFG1, 0x00 },
    { PREAMBLE_CFG0, 0x33 },
    { IQIC, 0x00 },
    { CHAN_BW, 0x03 },
    { MDMCFG0, 0x04 },
    { AGC_REF, 0x30 },
    { AGC_CS_THR, 0xEC },
    { AGC_CFG3, 0xD1 },
    { AGC_CFG2, 0x3F },
    { AGC_CFG1, 0x32 },
    { AGC_CFG0, 0x9F },
    { FIFO_CFG, 0x00 },
    { FS_CFG, 0x14 },
    { PKT_CFG2, 0x06 },
    { PKT_CFG1, 0x00 },
    { PKT_CFG0, 0x40 },
    { PA_CFG2, 0x66 },
    { PA_CFG0, 0x56 },
    { IF_MIX_CFG, 0x00 },
    { FREQOFF_CFG, 0x00 },
    { TOC_CFG, 0x0A },
    { CFM_DATA_CFG, 0x01 },
    { FREQ2, 0x6C },
    { FREQ1, 0xF1 },
    { FREQ0, 0x2F },
    { FS_DIG1, 0x00 },
    { FS_DIG0, 0x5F },
    { FS_CAL1, 0x40 },
    { FS_CAL0, 0x0E },
    { FS_DIVTWO, 0x03 },
    { FS_DSM0, 0x33 },
    { FS_DVC0, 0x17 },
    { FS_PFD, 0x50 },
    { FS_PRE, 0x6E },
    { FS_REG_DIV_CML, 0x14 },
    { FS_SPARE, 0xAC },
    { FS_VCO0, 0xB4 },
    { XOSC5, 0x0E },
    { XOSC1, 0x03 },
};
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
CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * Executes a command at the TX CC1120
 * @param CMDStrobe the command code
 * @return the first byte of the SPI buffer. Can be used for error checking
 */
uint8_t
cc_tx_cmd (uint8_t CMDStrobe)
{

  uint8_t tx_buf;
  uint8_t rx_buf;

  tx_buf = CMDStrobe;

  /* chip select LOw */
  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  /* Send-receive 1 byte */
  HAL_SPI_TransmitReceive (&hspi2, &tx_buf, &rx_buf, sizeof(uint8_t), 5000);

  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

  /*
   * TODO: Return the whole RX buffer
   */
  return rx_buf;
}
/**
 * Writes a value to a register of the TX CC1120
 * @param addr the address of the register
 * @param data the data to be written
 * @return the first byte of the SPI buffer. Can be used for error checking
 */
uint8_t
cc_tx_wr_reg (uint16_t addr, uint8_t data)
{

  uint8_t aTxBuffer[4];
  uint8_t aRxBuffer[4] = { 0, 0, 0, 0 };
  uint8_t len = 0;

  if (addr >= CC_EXT_ADD) {
    len = 3;

    aTxBuffer[0] = 0x2F;
    aTxBuffer[1] = (uint8_t) (0x00FF & addr);
    aTxBuffer[2] = data;
  }
  else {
    len = 2;

    aTxBuffer[0] = (uint8_t) (0x00FF & addr);
    aTxBuffer[1] = data;
  }

  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive (&hspi2, (uint8_t *) aTxBuffer, (uint8_t *) aRxBuffer, len, 5000); //send and receive 3 bytes
  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

  return aRxBuffer[0];
}

/**
 * Performs the setup of the TX CC1120 registers
 */
void
tx_registerConfig ()
{
  unsigned char writeByte;
  unsigned i;
  // Reset radio
  cc_tx_cmd (SRES);

  // Write registers to radio
  for (i = 0; i < (sizeof(TX_preferredSettings) / sizeof(registerSetting_t));
      i++) {
    writeByte = TX_preferredSettings[i].dat;
    cc_tx_wr_reg (TX_preferredSettings[i].addr, writeByte);
  }
}

/**
 * Performs the setup of the TX CC1120 registers suitable for CW transmission
 */
void
tx_cw_registerConfig ()
{
  unsigned char writeByte;
  unsigned i;
  // Reset radio
  cc_tx_cmd (SRES);

  // Write registers to radio
  for (i = 0; i < (sizeof(CW_preferredSettings) / sizeof(registerSetting_t));
      i++) {
    writeByte = CW_preferredSettings[i].dat;
    cc_tx_wr_reg (CW_preferredSettings[i].addr, writeByte);
  }
}

/**
 * Reads a register from the TX CC1120
 * @param addr the desired register address
 * @param data memory to store the value of the register
 * @return the first byte of the SPI buffer. Can be used for error checking
 */
uint8_t
cc_tx_rd_reg (uint16_t addr, uint8_t *data)
{
  uint8_t temp_TxBuffer[4];
  uint8_t temp_RxBuffer[4] = { 0, 0, 0, 0 };
  uint8_t len = 0;

  if (addr >= CC_EXT_ADD) {
    len = 3;

    temp_TxBuffer[0] = 0xAF;
    temp_TxBuffer[1] = (uint8_t) (0x00FF & addr);
    temp_TxBuffer[2] = 0;
  }
  else {
    len = 2;
    /* bit masked for read function */
    addr |= 0x0080;
    temp_TxBuffer[0] = (uint8_t) (0x00FF & addr);
    temp_TxBuffer[1] = 0;
  }

  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive (&hspi2, (uint8_t *) temp_TxBuffer,
         (uint8_t *) temp_RxBuffer, len, 5000);
  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

  *data = temp_RxBuffer[len - 1];

  return temp_RxBuffer[0];
}

/**
 * Write to the TX FIFO \p len bytes using the SPI bus
 * @param data the input buffer containing the data
 * @param spi_rx_data the SPI buffer for the return bytes
 * @param len the number of bytes to be sent
 * @return 0 on success of HAL_StatusTypeDef appropriate error code
 */
void
cc_tx_spi_write_fifo(const uint8_t *data, uint8_t *spi_rx_data, size_t len)
{
  /* Write the Burst flag at the start of the buffer */
  tx_frag_buf[0] = BURST_TXFIFO;
  memcpy(tx_frag_buf + 1, data, len);

  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, tx_frag_buf, len + 1, 5000);
  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  return;
}

/**
 * Sets the register configuration for CW transmission
 */
static inline void
set_tx_cw_regs()
{
  tx_cw_registerConfig();
}

/**
 * Transmits data using CW
 * @param in an array containing the CW symbols that should be sent
 * @param len the length of the array
 * @return CW_OK on success of an appropriate negative number with the
 * appropriate error
 */
int32_t
cc_tx_cw(const cw_pulse_t *in, size_t len)
{
  size_t i;
  uint8_t t[4] = {0, 0, 0, 0};
  uint8_t t2[16] = {0, 0};

  /* Set the CC1120 into unmodulated continuous FM mode */
  set_tx_cw_regs();
  cc_tx_cmd (SFTX);

  /*At least one byte should be written at the FIFO */
  cc_tx_spi_write_fifo (t, t2, 4);

  /*
   * Switch on and off the carrier for the proper amount of time
   */
  for(i = 0; i < len; i++) {
    if(in[i].cw_on){
      cc_tx_cmd (STX);
      HAL_Delay(in[i].duration_ms);
      cc_tx_cmd (SIDLE);
    }
    else{
      HAL_Delay(in[i].duration_ms);
    }
  }
  cc_tx_cmd (SIDLE);
  HAL_Delay(10);

  return 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

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
  MX_CRC_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Should return 0x (read ID)
  uint8_t cc_id_tx = 65;
  cc_tx_rd_reg (0x2F8F, &cc_id_tx);
  HAL_Delay(1000);
  HAL_UART_Transmit(&huart2, &cc_id_tx, sizeof(cc_id_tx), HAL_MAX_DELAY);

  // Trying to read if autocal is on
  cc_tx_cmd(SRES);
  cc_id_tx = 65;
  cc_tx_rd_reg (SETTLING_CFG, &cc_id_tx);
  HAL_UART_Transmit(&huart2, &cc_id_tx, sizeof(cc_id_tx), HAL_MAX_DELAY);

  uint8_t value = 65;

  tx_cw_registerConfig();

  // Trying to read if autocal is on
  cc_id_tx = 65;
  cc_tx_rd_reg (SETTLING_CFG, &cc_id_tx);
  HAL_UART_Transmit(&huart2, &cc_id_tx, sizeof(cc_id_tx), HAL_MAX_DELAY);

  HAL_Delay(10);

  uint8_t buff[] = {1,0,1,0,1,0,1,0,1,0,
                    1,0,1,0,1,0,1,0,1,0,
                    1,0,1,0,1,0,1,0,1,0,
                    1,0,1,0,1,0,1,0,1,0,
                    1,0,1,0,1,0,1,0,1,0,
                    1,0,1,0,1,0,1,0,1,0
                    };
  
  cc_tx_spi_write_fifo(buff, &value, sizeof(buff));

  // cc_id_tx = 65;
  // cc_tx_rd_reg (0xbf, &cc_id_tx);
  // HAL_UART_Transmit(&huart2, &cc_id_tx, sizeof(cc_id_tx), HAL_MAX_DELAY);

  cc_tx_cmd(SFSTXON);
  value = cc_tx_cmd(SNOP);
  HAL_UART_Transmit(&huart2, &value, sizeof(value), 2000);

  cc_tx_cmd(STX);

  while(1){
    value = cc_tx_cmd(SNOP);
    HAL_UART_Transmit(&huart2, &value, sizeof(value), 2000);
    HAL_Delay(1000);
  }

  HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // HAL_Delay(5000);
    // cc_tx_spi_write_fifo(buff, &value, sizeof(buff));
    // cc_tx_cmd(STX);
    // HAL_GPIO_TogglePin(GPIOA, LD2_Pin);
    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_PIN_CS_GPIO_Port, GPIO_PIN_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 LD2_Pin PAPin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|LD2_Pin|GPIO_PIN_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
