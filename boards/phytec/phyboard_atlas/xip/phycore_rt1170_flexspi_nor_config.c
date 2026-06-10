/*
 * Copyright 2026 PHYTEC America, LLC
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "phycore_rt1170_flexspi_nor_config.h"

#if defined(CONFIG_NXP_IMXRT_BOOT_HEADER) && defined(CONFIG_BOOT_FLEXSPI_NOR)
__attribute__((section(".boot_hdr.conf"), used))

#define FLASH_DUMMY_CYCLES 0x06
#define FLASH_DUMMY_VALUE  0x06

const flexspi_nor_config_t qspi_flash_config = {
	.mem_config = {
		.tag = FLEXSPI_CFG_BLK_TAG,
		.version = FLEXSPI_CFG_BLK_VERSION,
		.read_sample_clk_src =
			FLEXSPI_READ_SAMPLE_CLK_LOOPBACK_FROM_DQS_PAD,
		.cs_hold_time = 3u,
		.cs_setup_time = 3u,
		/* Enable DDR mode, Wordaddassable, Safe configuration, Differential clock */
		.controller_misc_option = 0x10,
		.device_type = FLEXSPI_DEVICE_TYPE_SERIAL_NOR,
		.sflash_pad_type = SERIAL_FLASH_4_PADS,
		.serial_clk_freq = FLEXSPI_SERIAL_CLK_100MHZ,
		.sflash_a1_size = 16u * 1024u * 1024u,
		/* Enable flash configuration feature */
		.config_cmd_enable = 1u,
		.config_mode_type[0] = DEVICE_CONFIG_CMD_TYPE_GENERIC,
		/* Set configuration command sequences */
		.config_cmd_seqs[0] = {
			.seq_num = 1,
			.seq_id = 12,
			.reserved = 0,
		},
		/* Prepare setting value for Read Register in flash */
		.config_cmd_args[0] = (FLASH_DUMMY_VALUE << 3),
		.lookup_table = {
			/* Fast read quad mode - SDR */
			[4 * CMD_LUT_SEQ_IDX_READ + 0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD,
				0xEB, RADDR_SDR, FLEXSPI_4PAD, 0x18),
			[4 * CMD_LUT_SEQ_IDX_READ + 1] = FLEXSPI_LUT_SEQ(DUMMY_SDR, FLEXSPI_4PAD,
				FLASH_DUMMY_CYCLES, READ_SDR, FLEXSPI_4PAD,
				0x04),

			/* Read Status LUTs */
			[4 * CMD_LUT_SEQ_IDX_READSTATUS + 0] = FLEXSPI_LUT_SEQ(CMD_SDR,
				FLEXSPI_1PAD, 0x05, READ_SDR, FLEXSPI_1PAD, 0x04),

			/* Write Enable LUTs */
			[4 * CMD_LUT_SEQ_IDX_WRITEENABLE + 0] = FLEXSPI_LUT_SEQ(CMD_SDR,
				FLEXSPI_1PAD, 0x06, STOP, FLEXSPI_1PAD, 0x0),

			/* Erase Sector LUTs */
			[4 * CMD_LUT_SEQ_IDX_ERASESECTOR + 0] = FLEXSPI_LUT_SEQ(CMD_SDR,
				FLEXSPI_1PAD, 0x20, RADDR_SDR, FLEXSPI_1PAD, 0x18),

			/* Page Program LUTs */
			[4 * CMD_LUT_SEQ_IDX_WRITE + 0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD,
				0x02, RADDR_SDR, FLEXSPI_1PAD, 0x18),
			[4 * CMD_LUT_SEQ_IDX_WRITE + 1] = FLEXSPI_LUT_SEQ(WRITE_SDR, FLEXSPI_1PAD,
				0x04, STOP, FLEXSPI_1PAD, 0x0),

			/* Erase Chip LUTs */
			[4 * CMD_LUT_SEQ_IDX_CHIPERASE + 0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD,
				0x60, STOP, FLEXSPI_1PAD, 0x0),
		},
	},
	.page_size = 256u,
	.sector_size = 4u * 1024u,
	.ipcmd_serial_clk_freq = 0x1,
	.block_size = 64u * 1024u,
	.is_uniform_block_size = false,
};

#endif /* defined(CONFIG_NXP_IMXRT_BOOT_HEADER) && defined(CONFIG_BOOT_FLEXSPI_NOR) */
