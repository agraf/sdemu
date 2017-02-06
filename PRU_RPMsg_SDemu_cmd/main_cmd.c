/*
 * Copyright (C) 2017 Alexander Graf <agraf@suse.de>
 *
 * SPDX-License-Identifier:      GPL-2.0
 *
 * Initially based on remoteproc example which is
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the
 *	  distribution.
 *
 *	* Neither the name of Texas Instruments Incorporated nor the names of
 *	  its contributors may be used to endorse or promote products derived
 *	  from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <pru_cfg.h>
#include <pru_intc.h>
#include <rsc_types.h>
#include <pru_rpmsg.h>
#include "resource_table_0.h"
#include "hw_types.h"
#include "soc_AM335x.h"
#include "hw_control_AM335x.h"


volatile register uint32_t __R30;	/* OUT */
volatile register uint32_t __R31;	/* IN */

/* AM335x specific mux bit defines */

#define AM335X_SLEWCTRL_FAST            (0 << 6)
#define AM335X_SLEWCTRL_SLOW            (1 << 6)
#define AM335X_INPUT_EN                 (1 << 5)
#define AM335X_PULL_UP                  (1 << 4)
/* bit 3: 0 - enable, 1 - disable for pull enable */
#define AM335X_PULL_DISA                (1 << 3)

#define AM335X_PIN_OUTPUT               (0)
#define AM335X_PIN_OUTPUT_PULLUP        (AM335X_PULL_UP)
#define AM335X_PIN_INPUT                (AM335X_INPUT_EN | AM335X_PULL_DISA)
#define AM335X_PIN_INPUT_PULLUP         (AM335X_INPUT_EN | AM335X_PULL_UP)
#define AM335X_PIN_INPUT_PULLDOWN       (AM335X_INPUT_EN)

#define CMD_FROM_HOST				0x40
#define CMD_GO_IDLE_STATE			0
#define CMD_SEND_OP_CMD				1
#define CMD_ALL_SEND_CID			2
#define CMD_SEND_RELATIVE_ADDR		3
#define CMD_SEND_DSR				4
#define CMD_SWITCH_FUNCTION			6
#define CMD_SELECT_CARD				7
#define CMD_SEND_IF_COND			8
#define CMD_SEND_CSD				9
#define CMD_SEND_CID				10
#define CMD_READ_DAT_UNTIL_STOP		11
#define CMD_STOP_TRANSMISSION		12
#define CMD_SEND_STATUS				13
#define CMD_GO_INACTIVE_STATE		15
#define CMD_SET_BLOCKLEN			16
#define CMD_READ_SINGLE_BLOCK		17
#define CMD_READ_MULTIPLE_BLOCK		18
#define CMD_SET_BLOCK_COUNT			23
#define CMD_WRITE_SINGLE_BLOCK		24
#define CMD_WRITE_MULTIPLE_BLOCK	25
#define CMD_PROGRAM_CID				26
#define CMD_PROGRAM_CSD				27
#define CMD_SET_WRITE_PROT			28
#define CMD_CLR_WRITE_PROT			29
#define CMD_SEND_WRITE_PROT			30
#define CMD_ERASE_WR_BLK_START		32
#define CMD_ERASE_WR_BLK_END		33
#define CMD_ERASE					38
#define CMD_LOCK_UNLOCK				42
#define CMD_APP_CMD					55
#define CMD_GEN_CMD					56

#define ACMD_SET_BUS_WIDTH			6
#define ACMD_SD_STATUS				13
#define ACMD_SEND_NUM_WR_BLOCKS		22
#define ACMD_SET_WR_BLK_ERASE_COUNT	23
#define ACMD_SD_APP_OP_COND			41
#define ACMD_SET_CLR_CARD_DETECT	42
#define ACMD_SEND_SCR				51

/* Card Status bits */
#define OUT_OF_RANGE		(1 << 31)
#define ADDRESS_ERROR		(1 << 30)
#define BLOCK_LEN_ERROR		(1 << 29)
#define ERASE_SEQ_ERROR		(1 << 28)
#define ERASE_PARAM			(1 << 27)
#define WP_VIOLATION		(1 << 26)
#define CARD_IS_LOCKED		(1 << 25)
#define LOCK_UNLOCK_FAILED	(1 << 24)
#define COM_CRC_ERROR		(1 << 23)
#define ILLEGAL_COMMAND		(1 << 22)
#define CARD_ECC_FAILED		(1 << 21)
#define CC_ERROR			(1 << 20)
#define SD_ERROR			(1 << 19)
#define CID_CSD_OVERWRITE	(1 << 16)
#define WP_ERASE_SKIP		(1 << 15)
#define CARD_ECC_DISABLED	(1 << 15)
#define ERASE_RESET			(1 << 13)
#define CURRENT_STATE		(7 << 9)
#define READY_FOR_DATA		(1 << 8)
#define APP_CMD				(1 << 5)
#define AKE_SEQ_ERROR		(1 << 3)

enum SDCardStates {
	sd_inactive_state	= -1,
	sd_idle_state		= 0,
	sd_ready_state,
	sd_identification_state,
	sd_standby_state,
	sd_transfer_state,
	sd_sendingdata_state,
	sd_receivingdata_state,
	sd_programming_state,
	sd_disconnect_state,
};

#define CARD_STATUS_A				0x02004100
#define CARD_STATUS_B				0x00c01000
#define CARD_STATUS_C				0xfd39a008 /* XXX original has APP_CMD too */
#define CARD_STATUS_RESET			READY_FOR_DATA

static uint32_t card_status = CARD_STATUS_RESET;

/* Operation Conditions Register */
#define OCR_POWER_UP		(1U << 31)
#define OCR_CAPACITY		(1 << 30)	/* 1 = SDHC/SDXC */
#define OCR_UHS_II			(1 << 29)
#define OCR_V18_OK			(1 << 24)
#define OCR_V35_36			(1 << 23)
#define OCR_V34_35			(1 << 22)
#define OCR_V33_34			(1 << 21)
#define OCR_V32_33			(1 << 20)
#define OCR_V31_32			(1 << 19)
#define OCR_V30_31			(1 << 18)
#define OCR_V29_30			(1 << 17)
#define OCR_V28_29			(1 << 16)
#define OCR_V27_28			(1 << 15)
#define OCR_VOLTAGE_MASK		(OCR_V35_36 | \
					 OCR_V34_35 | \
					 OCR_V33_34 | \
					 OCR_V32_33 | \
					 OCR_V31_32 | \
					 OCR_V30_31 | \
					 OCR_V29_30 | \
					 OCR_V28_29 | \
					 OCR_V27_28)

static uint32_t ocr = OCR_VOLTAGE_MASK;

typedef struct cid {
	uint8_t mid;		/* Manufacturer ID */
	uint8_t oid[2];		/* OEM / Application ID */
	uint8_t pnm[5];		/* Product Name */
	uint8_t prv;		/* Product Revision */
	uint8_t psn[4];		/* Product Serial Number */
	uint8_t mdt[2];		/* Manufacturing Date */
	uint8_t res:1;		/* always 1 */
	uint8_t crc7:7;		/* crc7 */
} cid_t;

#define MDT_MONTH	7		/* July */
#define MDT_YEAR	16		/* 2016 */

cid_t cid = {
		.mid = 0xaa,
		.oid = { 'B', 'B' },
		.pnm = { 'B', 'e', 'a', 'g', 'l' },
		.prv = 0x01, /* Version 0.1 */
		.psn = { 1, 2, 3, 4 },
		.mdt = { (MDT_YEAR >> 4), ((MDT_YEAR & 0xf) << 4) | MDT_MONTH},
};

/* XXX not swapped yet */
typedef struct csd_v1 {
	uint8_t csd_structure;
	uint8_t taac;					/* Data Read Access-time-1 */
	uint8_t nsac;					/* Data Read Access-time-2 in CLK cycles (NSAC*100) */
	uint8_t tran_speed;				/* Max Data Transfer Rate */
	uint8_t ccc_hi;					/* Card Command Classes */
	uint8_t ccc_lo:4;				/* Card Command Classes */
	uint8_t read_bl_len:4;			/* Max read data block length */
	uint8_t read_bl_partial:1;		/* Partial blocks for read allowed */
	uint8_t write_blk_misalign:1;	/* Write block misalignment */
	uint8_t read_blk_misalign:1;	/* Read block misalignment */
	uint8_t dsr_imp:1;				/* dsr implemented */
	uint8_t res4:2;
	uint8_t c_size_hi;				/* Device Size */
	uint8_t c_size_lo:4;
	uint8_t cdd_r_curr_min:3;		/* Max read current @VDD min */
	uint8_t cdd_r_curr_max:3;		/* Max read current @VDD max */
	uint8_t cdd_w_curr_min:3;		/* Max write current @VDD min */
	uint8_t cdd_w_curr_max:3;		/* Max write current @VDD max */
	uint8_t c_size_mult:3;			/* Device size multiplier */
	uint8_t erase_blk_en:1;			/* Erase single block enable */
	uint8_t sector_size:7;			/* Erase sector size */
	uint8_t wp_grp_size:7;			/* Write protect group size */

	uint8_t wp_grp_enable:1;		/* Write protect group enable */
	uint8_t res3:2;
	uint8_t r2w_factor:3;			/* Write Speed Factor */

	uint8_t write_bl_len:4;			/* Max write data block length */
	uint8_t write_bl_partial:1;		/* Partial blocks for write allowed */
	uint8_t res2:5;

	uint8_t file_format_grp:1;		/* File Format Group */
	uint8_t copy:1;					/* Copy Flag */
	uint8_t perm_write_protect:1;	/* Permanent Write protection */
	uint8_t tmp_write_protect:1;	/* Temporary Write Protection */
	uint8_t file_format:2;			/* File format */
	uint8_t res1:2;

	uint8_t crc7;					/* crc7 */

} csd_v1_t;

/* Bit swapped for bitfields in LE */
typedef struct csd_v2 {
	uint8_t res6:6;
	uint8_t csd_structure:2;

	uint8_t taac;					/* Data Read Access-time */

	uint8_t nsac;					/* Data Read Access-time in CLK cycles (NSAC*100) */

	uint8_t tran_speed;				/* Max Data Transfer Rate */

	uint8_t ccc_hi;					/* Card Command Classes */

	uint8_t read_bl_len:4;			/* Max read data block length */
	uint8_t ccc_lo:4;				/* Card Command Classes */

	uint8_t res5_1:4;
	uint8_t dsr_imp:1;				/* dsr implemented */
	uint8_t read_blk_misalign:1;	/* Read block misalignment */
	uint8_t write_blk_misalign:1;	/* Write block misalignment */
	uint8_t read_bl_partial:1;		/* Partial blocks for read allowed */

	uint8_t c_size3:6;				/* Device Size */
	uint8_t res5_2:2;

	uint8_t c_size2;
	uint8_t c_size1;

	uint8_t sector_size_hi:6;			/* Erase sector size */
	uint8_t erase_blk_en:1;			/* Erase single block enable */
	uint8_t res4:1;

	uint8_t wp_grp_size:7;			/* Write protect group size */
	uint8_t sector_size_lo:1;			/* Erase sector size */

	uint8_t write_bl_len_hi:2;			/* Max write data block length */
	uint8_t r2w_factor:3;			/* Write Speed Factor */
	uint8_t res3:2;
	uint8_t wp_grp_enable:1;		/* Write protect group enable */

	uint8_t res2:5;
	uint8_t write_bl_partial:1;		/* Partial blocks for write allowed */
	uint8_t write_bl_len_lo:2;			/* Max write data block length */

	uint8_t res1:2;
	uint8_t file_format:2;			/* File format */
	uint8_t tmp_write_protect:1;	/* Temporary Write Protection */
	uint8_t perm_write_protect:1;	/* Permanent Write protection */
	uint8_t copy:1;					/* Copy Flag */
	uint8_t file_format_grp:1;		/* File Format Group */

	uint8_t res:1;					/* always 1 */
	uint8_t crc7:7;					/* crc7 */
} csd_v2_t;

#define SD_SIZE (8192ULL * 1024 * 1024)
#define SD_SECTORS(size) ((size / (512 * 1024)) - 1)

csd_v2_t csd = {
		.csd_structure = 0x1,	/* CSD Version 2.0 */
		.taac = 0x0e,			/* 1ms */
		.nsac = 0x00,			/* 0 clock cycles */
		.tran_speed = 0x32,		/* 25Mhz */
		.ccc_hi = 0x0,
		.ccc_lo = 0xf,			/* Class 0-3 */
		.read_bl_len = 9,		/* 512 byte blocks */
		.read_bl_partial = 0,
		.write_blk_misalign = 0,
		.read_blk_misalign = 0,
		.dsr_imp = 0,
		.c_size3 = (SD_SECTORS(SD_SIZE) >> 16) & 0x3f,
		.c_size2 = (SD_SECTORS(SD_SIZE) >> 8) & 0xff,
		.c_size1 = (SD_SECTORS(SD_SIZE) >> 0) & 0xff,
		.erase_blk_en = 1,
		.sector_size_hi = 0x3f,
		.sector_size_lo = 0x1,
		.wp_grp_size = 0x0,
		.wp_grp_enable = 0x0,
		.r2w_factor = 0x2,
		.write_bl_len_hi = (0x9 >> 2) & 0x3,
		.write_bl_len_lo = 0x9 & 0x3,
		.write_bl_partial = 0x0,
		.file_format_grp = 0x0,
		.copy = 0x0,
		.perm_write_protect = 0x0,
		.tmp_write_protect = 0x0,
		.file_format = 0x0,
};

/* Relative Card Address */
uint32_t rca;

/* PRU1 ICC */

//#define PRU_DATA __far __attribute__((cregister("PRU_DMEM_DATA", far), peripheral))
//#define PRU_FAST __far __attribute__((cregister("PRU_DMEM_FAST", far), peripheral))

enum pru1_mode {
	pru1_mode_idle,
	pru1_mode_read_4bit,
	pru1_mode_write_4bit,
	pru1_mode_read_1bit,
	pru1_mode_write_1bit,
	pru1_mode_set_recv,
	pru1_mode_set_send,
	pru1_mode_read_scr_4bit,
	pru1_mode_read_scr_1bit,
};

#pragma DATA_SECTION(requested_mode, ".pru1data");
#pragma RETAIN(requested_mode)
volatile enum pru1_mode requested_mode;

#pragma DATA_SECTION(active_mode, ".pru1data");
#pragma RETAIN(active_mode)
volatile enum pru1_mode active_mode;

#pragma DATA_SECTION(sector, ".pru1data");
#pragma RETAIN(sector)
volatile uint32_t sector;

#pragma DATA_SECTION(fast_cmd, ".fast");
#pragma RETAIN(fast_cmd)
volatile char fast_cmd;

#pragma DATA_SECTION(fast_arg1, ".fast");
#pragma RETAIN(fast_arg1)
volatile uint32_t fast_arg1;

#pragma DATA_SECTION(fast_arg2, ".fast");
#pragma RETAIN(fast_arg2)
volatile uint64_t fast_arg2;

#pragma DATA_SECTION(fast_arg3, ".fast");
#pragma RETAIN(fast_arg3)
volatile uint32_t fast_arg3;

#define SD_BUS_1BIT		0x1
#define SD_BUS_4BIT		0x4

//char * const dmem_fast = (void*)0x1f80;
uint8_t bus_width = SD_BUS_1BIT;

/* R30/R31 bit masks for I/O lines */
#define DAT2_MASK	(1 << 5)
#define DAT3_MASK	(1 << 3)
#define CMD_MASK	(1 << 1)
#define CLK_MASK	(1 << 0)
#define DAT0_MASK	(1 << 2)
#define DAT1_MASK	(1 << 7)


/* Host-0 Interrupt sets bit 30 in register R31 */
#define HOST_INT			((uint32_t) 1 << 30)

/* Host-1 Interrupt sets bit 31 in register R31 */
//#define COMPLETE_INT			((uint32_t) 1 << 31)

/* The PRU-ICSS system events used for RPMsg are defined in the Linux device tree
 * PRU0 uses system event 16 (To ARM) and 17 (From ARM)
 * PRU1 uses system event 18 (To ARM) and 19 (From ARM)
 */
#define TO_ARM_HOST			16
#define FROM_ARM_HOST			17

#define FAST_INT 0x1c

/*
#define IN_INT 0x1c
#define OUT_INT 0x1d
#define DONE_INT 0x1e
*/
#define INT_ENABLE (1 << 5)
#define INT_OFFSET 16

#define SDEMU_MSG_DBG					' '
#define SDEMU_MSG_SETPINS_CMD_IN		'1'
#define SDEMU_MSG_SETPINS_CMD_OUT		'2'
#define SDEMU_MSG_SETPINS_DAT_IN		'3'
#define SDEMU_MSG_SETPINS_DAT_OUT		'4'
#define SDEMU_MSG_SETPINS_SPI			'5'
#define SDEMU_MSG_SETPINS_RESET			'6'
#define SDEMU_MSG_QUERY_SD_INIT			'I'
#define SDEMU_MSG_SET_SIZE				'S'
#define SDEMU_MSG_PREAD_4BIT			'R'
#define SDEMU_MSG_PWRITE_4BIT			'W'
#define SDEMU_MSG_DONE					'\0'


/*
 * Using the name 'rpmsg-pru' will probe the rpmsg_pru driver found
 * at linux-x.y.z/drivers/rpmsg/rpmsg_pru.c
 */
#define CHAN_NAME			"rpmsg-sdemu"
#define CHAN_DESC			"CMD Channel"
#define CHAN_PORT			0

/*
 * Used to make sure the Linux drivers are ready for RPMsg communication
 * Found at linux-x.y.z/include/uapi/linux/virtio_config.h
 */
#define VIRTIO_CONFIG_S_DRIVER_OK	4

static char buf[512];
static char *bufp = buf;
int is_initialized = 0;

struct pru_rpmsg_transport transport;
uint16_t src, dst;

extern void send_buf_1bit_cmd(register uint8_t *buf, register uint32_t len);

static char *alloc_buf(int len)
{
	char *p = bufp;

	bufp += len;
	if (((unsigned long)bufp - (unsigned long)buf) > 510) {
		bufp = buf;
		return alloc_buf(len);
	}

	return p;
}

static void clear_arm_irq(void)
{
	CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;
}

/* Table for CRC-7 (polynomial x^7 + x^3 + 1) */
const unsigned char crc7_syndrome_table[256] = {
	0x00, 0x09, 0x12, 0x1b, 0x24, 0x2d, 0x36, 0x3f,
	0x48, 0x41, 0x5a, 0x53, 0x6c, 0x65, 0x7e, 0x77,
	0x19, 0x10, 0x0b, 0x02, 0x3d, 0x34, 0x2f, 0x26,
	0x51, 0x58, 0x43, 0x4a, 0x75, 0x7c, 0x67, 0x6e,
	0x32, 0x3b, 0x20, 0x29, 0x16, 0x1f, 0x04, 0x0d,
	0x7a, 0x73, 0x68, 0x61, 0x5e, 0x57, 0x4c, 0x45,
	0x2b, 0x22, 0x39, 0x30, 0x0f, 0x06, 0x1d, 0x14,
	0x63, 0x6a, 0x71, 0x78, 0x47, 0x4e, 0x55, 0x5c,
	0x64, 0x6d, 0x76, 0x7f, 0x40, 0x49, 0x52, 0x5b,
	0x2c, 0x25, 0x3e, 0x37, 0x08, 0x01, 0x1a, 0x13,
	0x7d, 0x74, 0x6f, 0x66, 0x59, 0x50, 0x4b, 0x42,
	0x35, 0x3c, 0x27, 0x2e, 0x11, 0x18, 0x03, 0x0a,
	0x56, 0x5f, 0x44, 0x4d, 0x72, 0x7b, 0x60, 0x69,
	0x1e, 0x17, 0x0c, 0x05, 0x3a, 0x33, 0x28, 0x21,
	0x4f, 0x46, 0x5d, 0x54, 0x6b, 0x62, 0x79, 0x70,
	0x07, 0x0e, 0x15, 0x1c, 0x23, 0x2a, 0x31, 0x38,
	0x41, 0x48, 0x53, 0x5a, 0x65, 0x6c, 0x77, 0x7e,
	0x09, 0x00, 0x1b, 0x12, 0x2d, 0x24, 0x3f, 0x36,
	0x58, 0x51, 0x4a, 0x43, 0x7c, 0x75, 0x6e, 0x67,
	0x10, 0x19, 0x02, 0x0b, 0x34, 0x3d, 0x26, 0x2f,
	0x73, 0x7a, 0x61, 0x68, 0x57, 0x5e, 0x45, 0x4c,
	0x3b, 0x32, 0x29, 0x20, 0x1f, 0x16, 0x0d, 0x04,
	0x6a, 0x63, 0x78, 0x71, 0x4e, 0x47, 0x5c, 0x55,
	0x22, 0x2b, 0x30, 0x39, 0x06, 0x0f, 0x14, 0x1d,
	0x25, 0x2c, 0x37, 0x3e, 0x01, 0x08, 0x13, 0x1a,
	0x6d, 0x64, 0x7f, 0x76, 0x49, 0x40, 0x5b, 0x52,
	0x3c, 0x35, 0x2e, 0x27, 0x18, 0x11, 0x0a, 0x03,
	0x74, 0x7d, 0x66, 0x6f, 0x50, 0x59, 0x42, 0x4b,
	0x17, 0x1e, 0x05, 0x0c, 0x33, 0x3a, 0x21, 0x28,
	0x5f, 0x56, 0x4d, 0x44, 0x7b, 0x72, 0x69, 0x60,
	0x0e, 0x07, 0x1c, 0x15, 0x2a, 0x23, 0x38, 0x31,
	0x46, 0x4f, 0x54, 0x5d, 0x62, 0x6b, 0x70, 0x79
};

static inline unsigned char crc7_byte(register unsigned char crc, register unsigned char data)
{
	return crc7_syndrome_table[(crc << 1) ^ data];
}

unsigned char crc7(register const unsigned char *buffer, register unsigned int len)
{
	unsigned char crc = 0;

	while (len--)
		crc = crc7_byte(crc, *buffer++);
	return crc;
}

unsigned char crc7_cmd(register uint8_t cmd, register uint32_t args)
{
	register unsigned char crc = 0;

	crc = crc7_byte(crc, cmd);
	crc = crc7_byte(crc, args >> 24);
	crc = crc7_byte(crc, args >> 16);
	crc = crc7_byte(crc, args >> 8);
	crc = crc7_byte(crc, args);

	return crc;
}

static void read_cmd(void);

enum send_mode {
	SEND_MODE,
	RECV_MODE,
	UNKNOWN_MODE,
};
static enum send_mode in_send_mode = UNKNOWN_MODE;

static void switch_to_recv(void)
{
	if (in_send_mode == RECV_MODE)
		return;

	/* Send ARM host message */
	fast_cmd = SDEMU_MSG_SETPINS_CMD_IN;
	__R31 = (INT_ENABLE | (FAST_INT - INT_OFFSET));

	/* Wait for msg to comlete */
	while (fast_cmd != SDEMU_MSG_DONE) ;

	in_send_mode = RECV_MODE;
}

static void switch_to_send(void)
{
	if (in_send_mode == SEND_MODE)
		return;

	/* Pull CMD line high, so that nobody gets the idea we're switched yet */
	__R30 |= CMD_MASK;

	/* Send ARM host message */
	fast_cmd = SDEMU_MSG_SETPINS_CMD_OUT;
	__R31 = (INT_ENABLE | (FAST_INT - INT_OFFSET));

	/* Wait for msg to comlete */
	while (fast_cmd != SDEMU_MSG_DONE) ;

	in_send_mode = SEND_MODE;
}

static void switch_to_data_recv(void)
{
	requested_mode = pru1_mode_set_recv;
	while (active_mode != pru1_mode_set_recv) ;
}

static void switch_to_data_send(void)
{
	requested_mode = pru1_mode_set_send;
	while (active_mode != pru1_mode_set_send) ;
}

static void enable_pins(void)
{
	/* Configure R30/R31 GPIOs to directly go to register values */
	CT_CFG.GPCFG0 = 0x0000;

	switch_to_recv();
}

static void calc_reg_crc7(uint8_t *reg)
{
	/* CRC7 is in the last byte */
	reg[15] = (crc7(reg, 15) << 1) | 1;
}

static void reset_sd(void)
{
	calc_reg_crc7((uint8_t *)&cid);
	calc_reg_crc7((uint8_t *)&csd);
}

static void print_int(const char *str, register int line_nr)
{
	int i, slen = strlen(str);
	char *b = alloc_buf(1 + slen + 9);
	char *nump;

	if (!is_initialized) return;

	b[0] = SDEMU_MSG_DBG;
	memcpy(&b[1], str, slen);
	nump = &b[1 + slen];

	for (i = 0; i < 8; i++) {
		int n = (line_nr >> (i * 4)) & 0xf;
		if (n > 0x9)
			nump[7-i] = 'a' + (n - 0xa);
		else
			nump[7-i] = '0' + n;
	}
	nump[i] = '\n';

	pru_rpmsg_send(&transport, dst, src, b, 9 + slen + 1);
}

static void query_sd_size(void)
{
	uint64_t size = 0;
	uint32_t sectors = 0;

	/* Send ARM host message */
	fast_cmd = SDEMU_MSG_SET_SIZE;
	fast_arg1 = (long)&size;
	__R31 = (INT_ENABLE | (FAST_INT - INT_OFFSET));

	/* Wait for msg to comlete */
	while (fast_cmd != SDEMU_MSG_DONE) ;

	print_int("SD size: ", size);
	sectors = SD_SECTORS(size);

	/* Put the new size into our CSD */
	csd.c_size3 = (sectors >> 16) & 0x3f;
	csd.c_size2 = (sectors >> 8) & 0xff;
	csd.c_size1 = (sectors >> 0) & 0xff;
	calc_reg_crc7((uint8_t *)&csd);
}

/*
 * main.c
 */
void main(void)
{
	volatile uint8_t *status;

	/* allow OCP master port access by the PRU so the PRU can read external memories */
	CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

	reset_sd();

	/* clear the status of the PRU-ICSS system event that the ARM will use to 'kick' us */
	clear_arm_irq();

	/* Make sure the Linux drivers are ready for RPMsg communication */
	status = &resourceTable.rpmsg_vdev.status;
	while (!(*status & VIRTIO_CONFIG_S_DRIVER_OK));

	/* Initialize the RPMsg transport structure */
	pru_rpmsg_init(&transport, &resourceTable.rpmsg_vring0, &resourceTable.rpmsg_vring1, TO_ARM_HOST, FROM_ARM_HOST);

	/* Create the RPMsg channel between the PRU and ARM user space using the transport structure. */
	while (pru_rpmsg_channel(RPMSG_NS_CREATE, &transport, CHAN_NAME, CHAN_DESC, CHAN_PORT) != PRU_RPMSG_SUCCESS) ;

	while (!is_initialized) {
		/* Check bit 30 of register R31 to see if the ARM has kicked us */
		if (__R31 & HOST_INT) {
			uint16_t len;

			/* Clear the event status */
			clear_arm_irq();
			/* Receive all available messages, multiple messages can be sent per kick */
			while (pru_rpmsg_receive(&transport, &src, &dst, buf, &len) == PRU_RPMSG_SUCCESS) {
				if (!is_initialized) {
					is_initialized = 1;

					print_int("Hello from the CMD PRU!", (long)&fast_cmd);

					/* Configure pinmux */
					enable_pins();

					/* Make sure we know how big the SD card is */
					query_sd_size();

					/* Before interpreting commands, wait for the reset sequence */
					int i;

					for (i = 0; i < 20; i++) {
						/* Wait for a tick (clk is down) */
						while (__R31 & CLK_MASK) ;
						while (!(__R31 & CLK_MASK)) ;
					}

				}
			}
		}
	}

	while (1) {
		read_cmd();
	}
}

static inline uint32_t read_cmd_bits(register uint32_t mask)
{
	register uint32_t c = 0;
	register uint32_t r;

	do {
		/* Wait for a tick (clk is up) */
		while (1) {
			r = __R31;
			if (r & CLK_MASK)
				break;
		}

		/* Read cmd bit */
		if (r & CMD_MASK)
			c |= mask;

		mask >>= 1;

		/* Wait until clk is down again (new bit is getting set) */
		while (__R31 & CLK_MASK) ;
	} while (mask != 0);

	return c;
}

static void send_48reply(register uint8_t cmd, register uint32_t args)
{
	register uint8_t crc7 = (crc7_cmd(cmd, args) << 1) | 1;
	uint8_t buf[6] = {
			cmd,
			args >> 24,
			args >> 16,
			args >> 8,
			args,
			crc7,
	};

	send_buf_1bit_cmd(buf, sizeof(buf));
}

static void reply_r1(register uint8_t cmd)
{
	switch_to_send();
	send_48reply(cmd & ~CMD_FROM_HOST, card_status);
	switch_to_recv();

	/* Clear the "clear on read" status bits */
	card_status &= ~CARD_STATUS_C;
}

static void reply_r2(uint8_t *reg)
{
	uint8_t buf[17] = {
			0x3f,
			reg[0], reg[1], reg[2], reg[3], reg[4], reg[5], reg[6], reg[7],
			reg[8], reg[9], reg[10], reg[11], reg[12], reg[13], reg[14], reg[15] | 1 /* End bit */,
	};

	switch_to_send();
	send_buf_1bit_cmd(buf, sizeof(buf));
	switch_to_recv();
}

static void reply_r3()
{
	uint8_t buf[6] = {
			0x3f,
			ocr >> 24, ocr >> 16, ocr >> 8, ocr,
			0xff,
	};

	switch_to_send();
	send_buf_1bit_cmd(buf, sizeof(buf));
	switch_to_recv();
}

static void reply_r6(uint8_t cmd)
{
	uint32_t arg = rca << 16;

	arg |= ((card_status >> 8) & 0xc000) |
			((card_status >> 6) & 0x2000) |
			(card_status & 0x1fff);

	switch_to_send();
	send_48reply(cmd & ~CMD_FROM_HOST, arg);
	switch_to_recv();

	/* Clear the "clear on read" status bits */
	card_status &= ~CARD_STATUS_C;
}

static void set_state(enum SDCardStates state)
{
	print_int("Setting card status to: ", state);
	card_status &= ~CURRENT_STATE;
	card_status |= (state << 9) & CURRENT_STATE;
	print_int("  -> ", card_status);
}

static enum SDCardStates get_state(void)
{
	return (enum SDCardStates)((card_status & CURRENT_STATE) >> 9);
}

static void read_acmd(register uint32_t args)
{
	register unsigned char curcmd;
	int should_send_scr;

	/* Wait for CMD to go down and CLK to go up -> start bit */
	while ((__R31 & (CMD_MASK | CLK_MASK)) != CLK_MASK) ;

	curcmd = read_cmd_bits(0x80);
	args = read_cmd_bits(0x80000000);
	read_cmd_bits(0x80); /* crc7 */

	print_int("[ in] ACMD: ", curcmd);

	switch (curcmd) {
	case ACMD_SD_APP_OP_COND | CMD_FROM_HOST:		/* ACMD41 */
		/*
		 * Some firmware is supposed to break when we power on immediately,
		 * better tell the host next time
		 */
		reply_r3();

		ocr |= OCR_POWER_UP; /* Power on */
		ocr |= OCR_CAPACITY; /* SDHC */
		set_state(sd_ready_state);
		break;
	case ACMD_SET_CLR_CARD_DETECT | CMD_FROM_HOST:	/* ACMD42 */
		/*
		 * XXX Implement Card Detect resistor properly (needs host communication)
		 */
		reply_r1(curcmd);
		break;
	case ACMD_SET_BUS_WIDTH | CMD_FROM_HOST:		/* ACMD6 */
		switch (args) {
		case 0:
			bus_width = SD_BUS_1BIT;
			break;
		case 2:
			bus_width = SD_BUS_4BIT;
			break;
		default:
			print_int("[set bus width] Illegal bus width: ", args);
			card_status |= ILLEGAL_COMMAND;
			break;
		}

		reply_r1(curcmd);
		break;
	case ACMD_SEND_SCR | CMD_FROM_HOST:	/* ACMD51 */
		switch (get_state()) {
		case sd_transfer_state:
			should_send_scr = 1;
			switch_to_data_send();
			break;
		default:
			should_send_scr = 0;
			card_status |= ILLEGAL_COMMAND;
			break;
		}

		reply_r1(curcmd);

		if (should_send_scr) {
			set_state(sd_sendingdata_state);

			/* Force PRU1 back to idle */
			requested_mode = pru1_mode_idle;
			while (active_mode != pru1_mode_idle) ;

			/* Then kick it to SCR read */
			if (bus_width == SD_BUS_1BIT)
				requested_mode = pru1_mode_read_scr_1bit;
			else
				requested_mode = pru1_mode_read_scr_4bit;
		}

		break;
	default:
		card_status |= ILLEGAL_COMMAND;
		reply_r1(curcmd);
		print_int("[ in] Unknown ACMD: ", curcmd);
		break;
	}

	card_status &= ~APP_CMD;
}

static void cmd_invalid_silent(register int curcmd, register int args)
{
	/* Invalid Command */
	set_state(sd_idle_state);

	card_status |= ILLEGAL_COMMAND;
	reply_r1(curcmd);
}

static void cmd_invalid(register int curcmd, register int args)
{
	/* Invalid Command */
	cmd_invalid_silent(curcmd, args);

	print_int("[ in] Unknown CMD: ", curcmd);
}

static void cmd_go_idle_state(register int curcmd, register int args)
{
	/* CMD0 */

	set_state(sd_idle_state);
	card_status &= ~CARD_STATUS_B;

	/* No reply */
}

static void cmd_send_cid(register int curcmd, register int args)
{
	/* CMD2, CMD10 */

	/* Send R2 response */
	reply_r2((uint8_t *)&cid);

	set_state(sd_identification_state);
	card_status &= ~CARD_STATUS_B;
}

static void cmd_send_relative_addr(register int curcmd, register int args)
{
	/* CMD3 */

	/* Allocate new unique address */
	rca += 0x4567;
	/* Send R6 response */
	reply_r6(curcmd);

	set_state(sd_standby_state);
	card_status &= ~CARD_STATUS_B;
}

static void cmd_select_card(register int curcmd, register int args)
{
	/* CMD7 */

#ifdef BE_BUSY
	/* Indicate business */
	__R30 &= ~DAT0_MASK;

	/* Send R1 response */
	switch_to_send();
	send_48reply(curcmd & ~CMD_FROM_HOST, card_status); // XXX only if matching
	/* Clear the "clear on read" status bits */
	card_status &= ~CARD_STATUS_C;

	set_state(sd_transfer_state);

	/* Play busy for 10 cycles */
	int i;

	for (i = 0; i < 10; i++) {
		/* Wait for a tick (clk is down) */
		while (__R31 & CLK_MASK) ;
		while (!(__R31 & CLK_MASK)) ;
	}

	/* Unbusy ourselves */
	__R30 |= DAT0_MASK;
	switch_to_recv();
#else
	reply_r1(curcmd); // XXX only if matching
	set_state(sd_transfer_state);
#endif
	card_status &= ~CARD_STATUS_B;
}

static void cmd_send_if_cond(register int curcmd, register int args)
{
	/* CMD8 */

	/* Calculate response */
	args = (args & 0xff) | 0x100; /* 3.3V and canary echo */

	/* Send R7 response */
	switch_to_send();
	send_48reply(CMD_SEND_IF_COND, args);
	switch_to_recv();

	card_status &= ~CARD_STATUS_B;
}

static void cmd_send_csd(register int curcmd, register int args)
{
	/* CMD9 */

	/* Send R2 response */
	reply_r2((uint8_t *)&csd);

	card_status &= ~CARD_STATUS_B;
}

static void cmd_stop_transmission(register int curcmd, register int args)
{
	/* CMD12 */

	switch (get_state()) {
	case sd_sendingdata_state:
	case sd_receivingdata_state:
		set_state(sd_transfer_state);
		break;
	default:
		break;
	}

	/* Send R1b response */
	reply_r1(curcmd);

	card_status &= ~CARD_STATUS_B;
}

static void cmd_send_status(register int curcmd, register int args)
{
	/* CMD13 */

	/* Send R1 response */
	reply_r1(curcmd);

	card_status &= ~CARD_STATUS_B;
}

static void cmd_set_blocklen(register int curcmd, register int args)
{
	/* CMD16 */

	if (args != 512) {
		print_int("[set blocklen] invalid len: ", args);
		card_status |= BLOCK_LEN_ERROR;
	}

	/* Send R1 response */
	reply_r1(curcmd);

	card_status &= ~CARD_STATUS_B;
}

static void cmd_read_multiple_block(register int curcmd, register int args)
{
	/* CMD18 */
	register uint32_t replyargs;

	uint64_t addr = (ocr & OCR_CAPACITY) ? ((uint64_t)args << 9) : args;

	switch_to_data_send();

	//card_status |= CARD_IS_LOCKED;

	replyargs = card_status;
	/* Send R1 response */
	reply_r1(curcmd);

	{
		uint8_t replycmd = curcmd & ~CMD_FROM_HOST;
		register uint8_t crc7 = (crc7_cmd(replycmd, replyargs) << 1) | 1;

		print_int("[reply cmd18] cmd: ",  replycmd);
		print_int("[reply cmd18] args: ",  replyargs);
		print_int("[reply cmd18] crc7: ",  crc7);
	}

	sector = addr;
	if (bus_width == SD_BUS_1BIT)
		requested_mode = pru1_mode_read_1bit;
	else
		requested_mode = pru1_mode_read_4bit;

	set_state(sd_sendingdata_state);

	/*
	print_int("[read] args: ", args);
	print_int("[read] addr: ", addr);
	*/

	card_status &= ~CARD_STATUS_B;
}

static void cmd_app_cmd(register int curcmd, register int args)
{
	/* CMD55 */

	/* Expect ACMD next */
	card_status |= APP_CMD;

	/* Send R1 response */
	reply_r1(curcmd);

	/* Read ACMD */
	read_acmd(args);

	card_status &= ~CARD_STATUS_B;
}

static void (*const cmd_table[0x40])(register int curcmd, register int args) = {
		[CMD_GO_IDLE_STATE]			= cmd_go_idle_state,		/* CMD0 */
		[CMD_SEND_OP_CMD]			= cmd_invalid,				/* CMD1 XXX */
		[CMD_ALL_SEND_CID]			= cmd_send_cid,				/* CMD2 */
		[CMD_SEND_RELATIVE_ADDR]	= cmd_send_relative_addr,	/* CMD3 */
		[CMD_SEND_DSR]				= cmd_invalid,				/* CMD4 XXX */
		[5]							= cmd_invalid_silent,		/* CMD5 SDIO*/
		[CMD_SWITCH_FUNCTION]		= cmd_invalid,				/* CMD6 XXX */
		[CMD_SELECT_CARD]			= cmd_select_card,			/* CMD7 */
		[CMD_SEND_IF_COND]			= cmd_send_if_cond,			/* CMD8 */
		[CMD_SEND_CSD]				= cmd_send_csd,				/* CMD9 */
		[CMD_SEND_CID]				= cmd_send_cid,				/* CMD10 */
		[CMD_READ_DAT_UNTIL_STOP]	= cmd_invalid,				/* CMD11 XXX */
		[CMD_STOP_TRANSMISSION]		= cmd_stop_transmission,	/* CMD12 */
		[CMD_SEND_STATUS]			= cmd_send_status,			/* CMD13 */
		[14]						= cmd_invalid,				/* CMD14 */
		[CMD_GO_INACTIVE_STATE]		= cmd_invalid,				/* CMD15 XXX */
		[CMD_SET_BLOCKLEN]			= cmd_set_blocklen,			/* CMD16 */
		[CMD_READ_SINGLE_BLOCK]		= cmd_invalid,				/* CMD17 XXX */
		[CMD_READ_MULTIPLE_BLOCK]	= cmd_read_multiple_block,	/* CMD18 */
		[19]						= cmd_invalid,				/* CMD19 */
		[20]						= cmd_invalid,				/* CMD20 */
		[21]						= cmd_invalid,				/* CMD21 */
		[22]						= cmd_invalid,				/* CMD22 */
		[CMD_SET_BLOCK_COUNT]		= cmd_invalid,				/* CMD23 XXX */
		[CMD_WRITE_SINGLE_BLOCK]	= cmd_invalid,				/* CMD24 XXX */
		[CMD_WRITE_MULTIPLE_BLOCK]	= cmd_invalid,				/* CMD25 XXX */
		[CMD_PROGRAM_CID]			= cmd_invalid,				/* CMD26 XXX */
		[CMD_PROGRAM_CSD]			= cmd_invalid,				/* CMD27 XXX */
		[CMD_SET_WRITE_PROT]		= cmd_invalid,				/* CMD28 XXX */
		[CMD_CLR_WRITE_PROT]		= cmd_invalid,				/* CMD29 XXX */
		[CMD_SEND_WRITE_PROT]		= cmd_invalid,				/* CMD30 XXX */
		[31]						= cmd_invalid,				/* CMD31 */
		[CMD_ERASE_WR_BLK_START]	= cmd_invalid,				/* CMD32 XXX */
		[CMD_ERASE_WR_BLK_END]		= cmd_invalid,				/* CMD33 XXX */
		[34]						= cmd_invalid,				/* CMD34 */
		[35]						= cmd_invalid,				/* CMD35 */
		[36]						= cmd_invalid,				/* CMD36 */
		[37]						= cmd_invalid,				/* CMD37 */
		[CMD_ERASE]					= cmd_invalid,				/* CMD38 XXX */
		[39]						= cmd_invalid,				/* CMD39 */
		[40]						= cmd_invalid,				/* CMD40 */
		[41]						= cmd_invalid,				/* CMD41 */
		[CMD_LOCK_UNLOCK]			= cmd_invalid,				/* CMD42 XXX */
		[43]						= cmd_invalid,				/* CMD43 */
		[44]						= cmd_invalid,				/* CMD44 */
		[45]						= cmd_invalid,				/* CMD45 */
		[46]						= cmd_invalid,				/* CMD46 */
		[47]						= cmd_invalid,				/* CMD47 */
		[48]						= cmd_invalid,				/* CMD48 */
		[49]						= cmd_invalid,				/* CMD49 */
		[50]						= cmd_invalid,				/* CMD50 */
		[51]						= cmd_invalid,				/* CMD51 */
		[52]						= cmd_invalid_silent,		/* CMD52 SDIO */
		[53]						= cmd_invalid_silent,		/* CMD53 SDIO */
		[54]						= cmd_invalid,				/* CMD54 */
		[CMD_APP_CMD]				= cmd_app_cmd,				/* CMD55 */
		[CMD_GEN_CMD]				= cmd_invalid,				/* CMD56 XXX */
		[57]						= cmd_invalid,				/* CMD57 */
		[58]						= cmd_invalid,				/* CMD58 */
		[59]						= cmd_invalid,				/* CMD59 */
		[60]						= cmd_invalid,				/* CMD60 */
		[61]						= cmd_invalid,				/* CMD61 */
		[62]						= cmd_invalid,				/* CMD62 */
		[63]						= cmd_invalid,				/* CMD63 */
};

static void read_cmd(void)
{
	register char curcmd;
	register int args;

	/* Wait for CMD to go down and CLK to go up -> start bit */
	while ((__R31 & (CMD_MASK | CLK_MASK)) != CLK_MASK) ;

	curcmd = read_cmd_bits(0x80);
	args = read_cmd_bits(0x80000000);
	read_cmd_bits(0x80); /* crc7 */

	/* Only interpret host commands */
	if ((curcmd & 0xc0) != 0x40) {
		print_int("[ illegal in] CMD: ", curcmd);
		return;
	}

//	print_int("[ in] CMD: ", curcmd);

	cmd_table[curcmd & 0x3f](curcmd, args);
}
