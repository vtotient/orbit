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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

const registerSetting_t standardRegisters1[]={ //configure register settings here
  {IOCFG3,              0xB0}, //GPIO3 IO Pin Configuration
  {IOCFG2,              0x06}, //GPIO2 IO Pin Configuration
  {IOCFG1,              0xB0}, //GPIO1 IO Pin Configuration
  {IOCFG0,              0x40}, //GPIO0 IO Pin Configuration
  {SYNC3,               0x93}, //Sync Word Configuration [31:24]
  {SYNC2,               0x0B}, //Sync Word Configuration [23:16]
  {SYNC1,               0x51}, //Sync Word Configuration [15:8]
  {SYNC0,               0xDE}, //Sync Word Configuration [7:0]
  {SYNC_CFG1,           0x0B}, //Sync Word Detection Configuration Reg. 1
  {SYNC_CFG0,           0x17}, //Sync Word Length Configuration Reg. 0
  {DEVIATION_M,         0x06}, //Frequency Deviation Configuration
  {MODCFG_DEV_E,        0x03}, //Modulation Format and Frequency Deviation Configur..
  {DCFILT_CFG,          0x1C}, //Digital DC Removal Configuration
  {PREAMBLE_CFG1,       0x18}, //Preamble Length Configuration Reg. 1
  {PREAMBLE_CFG0,       0x2A}, //Preamble Detection Configuration Reg. 0
  {FREQ_IF_CFG,         0x40}, //RX Mixer Frequency Configuration
  {IQIC,                0xC6}, //Digital Image Channel Compensation Configuration
  {CHAN_BW,             0x08}, //Channel Filter Configuration
  {MDMCFG1,             0x46}, //General Modem Parameter Configuration Reg. 1
  {MDMCFG0,             0x05}, //General Modem Parameter Configuration Reg. 0
  {SYMBOL_RATE2,        0x43}, //Symbol Rate Configuration Exponent and Mantissa [1..
  {SYMBOL_RATE1,        0xA9}, //Symbol Rate Configuration Mantissa [15:8]
  {SYMBOL_RATE0,        0x2A}, //Symbol Rate Configuration Mantissa [7:0]
  {AGC_REF,             0x20}, //AGC Reference Level Configuration
  {AGC_CS_THR,          0x19}, //Carrier Sense Threshold Configuration
  {AGC_GAIN_ADJUST,     0x00}, //RSSI Offset Configuration
  {AGC_CFG3,            0x91}, //Automatic Gain Control Configuration Reg. 3
  {AGC_CFG2,            0x20}, //Automatic Gain Control Configuration Reg. 2
  {AGC_CFG1,            0xA9}, //Automatic Gain Control Configuration Reg. 1
  {AGC_CFG0,            0xCF}, //Automatic Gain Control Configuration Reg. 0
  {FIFO_CFG,            64}, //FIFO Configuration
  {DEV_ADDR,            0x00}, //Device Address Configuration
  {SETTLING_CFG,        0x03}, //Frequency Synthesizer Calibration and Settling Con..
  {FS_CFG,              0b00010100}, //Frequency Synthesizer Configuration
  {WOR_CFG1,            0x08}, //eWOR Configuration Reg. 1
  {WOR_CFG0,            0x21}, //eWOR Configuration Reg. 0
  {WOR_EVENT0_MSB,      0x00}, //Event 0 Configuration MSB
  {WOR_EVENT0_LSB,      0x00}, //Event 0 Configuration LSB
  {PKT_CFG2,            0x04}, //Packet Configuration Reg. 2
  {PKT_CFG1,            0x05}, //Packet Configuration Reg. 1
  {PKT_CFG0,            0x00}, //Packet Configuration Reg. 0
  {RFEND_CFG1,          0x0F}, //RFEND Configuration Reg. 1
  {RFEND_CFG0,          0x00}, //RFEND Configuration Reg. 0
  {PA_CFG2,             0x7F}, //Power Amplifier Configuration Reg. 2
  {PA_CFG1,             0x56}, //Power Amplifier Configuration Reg. 1
  {PA_CFG0,             0x7C}, //Power Amplifier Configuration Reg. 0
  {PKT_LEN,             127} //Packet Length Configuration
};

//==================================================

const registerSetting_t extendedRegisters1[]={ //configure extended register settings here
{IF_MIX_CFG,          0x00}, //IF Mix Configuration
  {FREQOFF_CFG,         0x22}, //Frequency Offset Correction Configuration
  {TOC_CFG,             0x0B}, //Timing Offset Correction Configuration
  {MARC_SPARE,          0x00}, //MARC Spare
  {ECG_CFG,             0x00}, //External Clock Frequency Configuration
  {CFM_DATA_CFG,        0x00}, //Custom frequency modulation enable
  {EXT_CTRL,            0x01}, //External Control Configuration
  {RCCAL_FINE,          0x00}, //RC Oscillator Calibration Fine
  {RCCAL_COARSE,        0x00}, //RC Oscillator Calibration Coarse
  {RCCAL_OFFSET,        0x00}, //RC Oscillator Calibration Clock Offset
  {FREQOFF1,            0x00}, //Frequency Offset MSB
  {FREQOFF0,            0x00}, //Frequency Offset LSB
  {FREQ2,               0x6C}, //Frequency Configuration [23:16]
  {FREQ1,               0x40}, //Frequency Configuration [15:8]
  {FREQ0,               0x00}, //Frequency Configuration [7:0]
  {IF_ADC2,             0x02}, //Analog to Digital Converter Configuration Reg. 2
  {IF_ADC1,             0xA6}, //Analog to Digital Converter Configuration Reg. 1
  {IF_ADC0,             0x04}, //Analog to Digital Converter Configuration Reg. 0
  {FS_DIG1,             0x00}, //Frequency Synthesizer Digital Reg. 1
  {FS_DIG0,             0x5F}, //Frequency Synthesizer Digital Reg. 0
  {FS_CAL3,             0x00}, //Frequency Synthesizer Calibration Reg. 3
  {FS_CAL2,             0x20}, //Frequency Synthesizer Calibration Reg. 2
  {FS_CAL1,             0x40}, //Frequency Synthesizer Calibration Reg. 1
  {FS_CAL0,             0x0E}, //Frequency Synthesizer Calibration Reg. 0
  {FS_CHP,              0x27}, //Frequency Synthesizer Charge Pump Configuration
  {FS_DIVTWO,           0x03}, //Frequency Synthesizer Divide by 2
  {FS_DSM1,             0x00}, //FS Digital Synthesizer Module Configuration Reg. 1
  {FS_DSM0,             0x33}, //FS Digital Synthesizer Module Configuration Reg. 0
  {FS_DVC1,             0xFF}, //Frequency Synthesizer Divider Chain Configuration ..
  {FS_DVC0,             0x17}, //Frequency Synthesizer Divider Chain Configuration ..
  {FS_LBI,              0x00}, //Frequency Synthesizer Local Bias Configuration
  {FS_PFD,              0x50}, //Frequency Synthesizer Phase Frequency Detector Con..
  {FS_PRE,              0x6E}, //Frequency Synthesizer Prescaler Configuration
  {FS_REG_DIV_CML,      0x14}, //Frequency Synthesizer Divider Regulator Configurat..
  {FS_SPARE,            0xAC}, //Frequency Synthesizer Spare
  {FS_VCO4,             0x13}, //FS Voltage Controlled Oscillator Configuration Reg..
  {FS_VCO3,             0x00}, //FS Voltage Controlled Oscillator Configuration Reg..
  {FS_VCO2,             0x4C}, //FS Voltage Controlled Oscillator Configuration Reg..
  {FS_VCO1,             0x9C}, //FS Voltage Controlled Oscillator Configuration Reg..
  {FS_VCO0,             0xB4}, //FS Voltage Controlled Oscillator Configuration Reg..
  {GBIAS6,              0x00}, //Global Bias Configuration Reg. 6
  {GBIAS5,              0x02}, //Global Bias Configuration Reg. 5
  {GBIAS4,              0x00}, //Global Bias Configuration Reg. 4
  {GBIAS3,              0x00}, //Global Bias Configuration Reg. 3
  {GBIAS2,              0x10}, //Global Bias Configuration Reg. 2
  {GBIAS1,              0x00}, //Global Bias Configuration Reg. 1
  {GBIAS0,              0x00}, //Global Bias Configuration Reg. 0
  {IFAMP,               0x01}, //Intermediate Frequency Amplifier Configuration
  {LNA,                 0x01}, //Low Noise Amplifier Configuration
  {RXMIX,               0x01}, //RX Mixer Configuration
  {XOSC5,               0x0E}, //Crystal Oscillator Configuration Reg. 5
  {XOSC4,               0xA0}, //Crystal Oscillator Configuration Reg. 4
  {XOSC3,               0x03}, //Crystal Oscillator Configuration Reg. 3
  {XOSC2,               0x04}, //Crystal Oscillator Configuration Reg. 2
  {XOSC1,               0x03}, //Crystal Oscillator Configuration Reg. 1
  {XOSC0,               0x00}, //Crystal Oscillator Configuration Reg. 0
  {ANALOG_SPARE,        0x00}, //Analog Spare
  {PA_CFG3,             0x00}, //Power Amplifier Configuration Reg. 3
  {WOR_TIME1,           0x00}, //eWOR Timer Counter Value MSB
  {WOR_TIME0,           0x00}, //eWOR Timer Counter Value LSB
  {WOR_CAPTURE1,        0x00}, //eWOR Timer Capture Value MSB
  {WOR_CAPTURE0,        0x00}, //eWOR Timer Capture Value LSB
  {BIST,                0x00}, //MARC Built-In Self-Test
  {DCFILTOFFSET_I1,     0xFC}, //DC Filter Offset I MSB
  {DCFILTOFFSET_I0,     0x66}, //DC Filter Offset I LSB
  {DCFILTOFFSET_Q1,     0x06}, //DC Filter Offset Q MSB
  {DCFILTOFFSET_Q0,     0xE6}, //DC Filter Offset Q LSB
  {IQIE_I1,             0xFD}, //IQ Imbalance Value I MSB
  {IQIE_I0,             0x15}, //IQ Imbalance Value I LSB
  {IQIE_Q1,             0x02}, //IQ Imbalance Value Q MSB
  {IQIE_Q0,             0xEF}, //IQ Imbalance Value Q LSB
  {RSSI1,               0xED}, //Received Signal Strength Indicator Reg. 1
  {RSSI0,               0x03}, //Received Signal Strength Indicator Reg.0
  {MARCSTATE,           0x33}, //MARC State
  {LQI_VAL,             0x80}, //Link Quality Indicator Value
  {PQT_SYNC_ERR,        0x9F}, //Preamble and Sync Word Error
  {DEM_STATUS,          0x01}, //Demodulator Status
  {FREQOFF_EST1,        0x00}, //Frequency Offset Estimate MSB
  {FREQOFF_EST0,        0x0E}, //Frequency Offset Estimate LSB
  {AGC_GAIN3,           0x27}, //Automatic Gain Control Reg. 3
  {AGC_GAIN2,           0xD1}, //Automatic Gain Control Reg. 2
  {AGC_GAIN1,           0x00}, //Automatic Gain Control Reg. 1
  {AGC_GAIN0,           0x3F}, //Automatic Gain Control Reg. 0
  {CFM_RX_DATA_OUT,     0x00}, //Custom Frequency Modulation RX Data
  {CFM_TX_DATA_IN,      0x00}, //Custom Frequency Modulation TX Data
  {ASK_SOFT_RX_DATA,    0x30}, //ASK Soft Decision Output
  {RNDGEN,              0x7F}, //Random Number Generator Value
  {MAGN2,               0x00}, //Signal Magnitude after CORDIC [16]
  {MAGN1,               0x00}, //Signal Magnitude after CORDIC [15:8]
  {MAGN0,               0x10}, //Signal Magnitude after CORDIC [7:0]
  {ANG1,                0x00}, //Signal Angular after CORDIC [9:8]
  {ANG0,                0xD5}, //Signal Angular after CORDIC [7:0]
  {CHFILT_I2,           0x0F}, //Channel Filter Data Real Part [18:16]
  {CHFILT_I1,           0xFF}, //Channel Filter Data Real Part [15:8]
  {CHFILT_I0,           0xEE}, //Channel Filter Data Real Part [7:0]
  {CHFILT_Q2,           0x00}, //Channel Filter Data Imaginary Part [18:16]
  {CHFILT_Q1,           0x00}, //Channel Filter Data Imaginary Part [15:8]
  {CHFILT_Q0,           0x11}, //Channel Filter Data Imaginary Part [7:0]
  {GPIO_STATUS,         0x00}, //General Purpose Input/Output Status
  {FSCAL_CTRL,          0x09}, //Frequency Synthesizer Calibration Control
  {PHASE_ADJUST,        0x00}, //Frequency Synthesizer Phase Adjust
  {PARTNUMBER,          0x48}, //Part Number
  {PARTVERSION,         0x21}, //Part Revision
  {SERIAL_STATUS,       0x00}, //Serial Status
  {MODEM_STATUS1,       0x11}, //Modem Status Reg. 1
  {MODEM_STATUS0,       0x00}, //Modem Status Reg. 0
  {MARC_STATUS1,        0x40}, //MARC Status Reg. 1
  {MARC_STATUS0,        0x00}, //MARC Status Reg. 0
  {PA_IFAMP_TEST,       0x00}, //Power Amplifier Intermediate Frequency Amplifier T..
  {FSRF_TEST,           0x00}, //Frequency Synthesizer Test
  {PRE_TEST,            0x00}, //Frequency Synthesizer Prescaler Test
  {PRE_OVR,             0x00}, //Frequency Synthesizer Prescaler Override
  {ADC_TEST,            0x00}, //Analog to Digital Converter Test
  {DVC_TEST,            0x0B}, //Digital Divider Chain Test
  {ATEST,               0x40}, //Analog Test
  {ATEST_LVDS,          0x00}, //Analog Test LVDS
  {ATEST_MODE,          0x00}, //Analog Test Mode
  {XOSC_TEST1,          0x3C}, //Crystal Oscillator Test Reg. 1
  {XOSC_TEST0,          0x00}, //Crystal Oscillator Test Reg. 0
  {RXFIRST,             0x00}, //RX FIFO Pointer First Entry
  {TXFIRST,             0x0A}, //TX FIFO Pointer First Entry
  {RXLAST,              0x00}, //RX FIFO Pointer Last Entry
  {TXLAST,              0x0E}, //TX FIFO Pointer Last Entry
  {NUM_TXBYTES,         0x04}, //TX FIFO Status
  {NUM_RXBYTES,         0x00}, //RX FIFO Status
  {FIFO_NUM_TXBYTES,    0x0F}, //TX FIFO Status
  {FIFO_NUM_RXBYTES,    0x00}, //RX FIFO Status
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
  uint8_t data1[] = {SRES , 0, SNOP, 0};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    char buff[] = "fail\r\n";
    char buff2[] = "pass\r\n";
    for (int i = 0;; i++)
      {
          int result = 0;
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
          HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)&i, (uint8_t*)&result, sizeof(i), HAL_MAX_DELAY);
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
          if (result != (i - 1))
          {
              asm("nop");
              HAL_UART_Transmit(&huart2, buff, sizeof(buff), HAL_MAX_DELAY);
          }
          else{
        	  HAL_UART_Transmit(&huart2, buff2, sizeof(buff2), HAL_MAX_DELAY);
          }
          HAL_Delay(10);
      }
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
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_LSB;
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
