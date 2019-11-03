// @todo add license

/**
 * @file cc1120_hal_init.c
 * @author vtotient
 * @date 23 October, 2019
 * @brief APIs to initalize the CC1120 in both rx and tx mode.
 *
 * Include APIs to burst write initial register configuration modes.
 * @todo fill this out
 */

#include "Inc/cc1120_hal_config.h"
#include "stdint.h"
// #include "Inc/cc1120_hal_init.h"

typedef struct{
    uint16_t address;
    uint8_t value;
} CC1120_RegisterSettings_t;

// LUT for initalizing CC1120 into tx mode with burst SPI write
static const CC1120_RegisterSettings_t CC1120_txInitRegisters[] =
{
    { IOCFG3, 0x06 },
    { IOCFG2, 0x02 },
    { IOCFG1, 0x40 },
    { IOCFG0, 0x40 },
    { SYNC3, 0x00 },
    { SYNC2, 0x00 },
    { SYNC1, 0x7A },
    { SYNC0, 0x0E },
    { SYNC_CFG1, 0x0B },
    { SYNC_CFG0, 0x03 }, /* No SYNC word. It is generated on the MPU and handled manually */
    { DCFILT_CFG, 0x1C },
    { PREAMBLE_CFG1, 0x00 }, /* No preamble. It is generated on the MPU and handled manually */
    { PREAMBLE_CFG0, 0x0A }, /* No preamble. It is generated on the MPU and handled manually */
    { IQIC, 0xC6 },
    { CHAN_BW, 0x08 },
    { MDMCFG0, 0x05 },
    { SYMBOL_RATE2, 0x73 },
    { AGC_REF, 0x20 },
    { AGC_CS_THR, 0x19 },
    { AGC_CFG1, 0xA9 },
    { FIFO_CFG, CC1120_TXFIFO_THR },
    { SETTLING_CFG, 0x03 },
    { FS_CFG, 0x14 },
    { PKT_CFG1, 0x00 },
    { PKT_CFG0, CC1120_FIXED_PKT_LEN },
    { PA_CFG2, 0x2F },  //4-PACFG2 0X26, 6dBm 0x2B 2dbm 0x22,8 DBM 2F
	{ PA_CFG0, 0x7D },
	{ PKT_LEN, 0xFF },
	{ IF_MIX_CFG, 0x00 },
	{ FREQOFF_CFG, 0x22 },
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
	{ FS_VCO4, 0x13 },
	{ FS_VCO1, 0xAC },
	{ FS_VCO0, 0xB4 },
	{ XOSC5, 0x0E },
	{ XOSC1, 0x03 },
	{ DCFILTOFFSET_I1, 0xF8 },
	{ DCFILTOFFSET_I0, 0x39 },
	{ DCFILTOFFSET_Q1, 0x0E },
	{ DCFILTOFFSET_Q0, 0x9B },
	{ CFM_DATA_CFG, 0x00 },
	{ IQIE_I1, 0xEF },
	{ IQIE_I0, 0xDE },
	{ IQIE_Q1, 0x02 },
	{ IQIE_Q0, 0x2F },
	{ AGC_GAIN1, 0x13 },
	{ SERIAL_STATUS, 0x10 },
	{ MODCFG_DEV_E, 0x0B } };

const CC120_RegisterSettings_t CC1120_cwRegisters[] =
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

#define HAL_CC1120_NUM_CW_REGS sizeof(CC1120_cwRegisters) / sizeof(CC1120_RegisterSettings_t)

/**
 * Initialize registers for CW mode
 */
void cc1120_hal_cwConfig()
{
	// @todo reset the radio

	// Write LUT to CC1120 
	for(int i = 0; i < HAL_CC1120_NUM_CW_REGS; i++)
	{
		//cc1120_hal_txPoke(CC1120_cwRegisters[i].addr, CC1120_cwRegisters[i].data);
	}
}
