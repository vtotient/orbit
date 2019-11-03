// @todo add license 

/**
 * @file cc1120_hal_tx.c Brief description 
 *
 * More detailed description 
 */

#pragma once
//#include "Inc/cc1120_hal_spi.h" // @todo add mk file support
#include "stm32f4xx_hal.h"
#include "stdint.h"
// @todo how to include SPI handles

/**
 * @brief Peek a cc1120 tx register 
 *
 * @param addr cc1120 register address
 * @param data pointer to memory location to store register value
 */
void cc1120_hal_txPeek(uint8_t addr, uint8_t *data)
{
	// Transmit addr and recieve register data
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t *) addr, data, 2, HAL_MAX_DELAY);
}

/**
 * @brief Poke a cc1120 tx register
 *
 * @param addr cc1120 register address
 * @param data pointer to data to write to cc1120 tx register
 */
void cc1120_hal_txPoke(uint8_t addr, uint8_t *data)
{
	// Write to spi
	HAL_SPI_Transmit(&hspi2, (uint8_t *) addr, data, sizeof(data), HAL_MAX_DELAY);
{
