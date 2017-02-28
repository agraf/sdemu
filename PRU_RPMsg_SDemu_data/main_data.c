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
#include <pru_ctrl.h>
#include <pru_intc.h>
#include <rsc_types.h>
#include <pru_rpmsg.h>
#include "resource_table_0.h"
#include "hw_types.h"
#include "soc_AM335x.h"
#include "hw_control_AM335x.h"
#include "sdemu_icc.h"


volatile register uint32_t __R30;	/* OUT */
volatile register uint32_t __R31;	/* IN */

/* R30/R31 bit masks for I/O lines */
#define DAT0_MASK	(1 << 8)
#define DAT1_MASK	(1 << 9)
#define DAT2_MASK	(1 << 10)
#define DAT3_MASK	(1 << 11)
#define CLK_MASK	(1 << 16)

/* v1.1 additions */
#define DAT0_OUT_MASK	(1 << 0)
#define DAT1_OUT_MASK	(1 << 1)
#define DAT2_OUT_MASK	(1 << 2)
#define DAT3_OUT_MASK	(1 << 3)

/* Host-1 Interrupt sets bit 31 in register R31 */
#define HOST_INT			((uint32_t) 1 << 31)

/* The PRU-ICSS system events used for RPMsg are defined in the Linux device tree
 * PRU0 uses system event 16 (To ARM) and 17 (From ARM)
 * PRU1 uses system event 18 (To ARM) and 19 (From ARM)
 */
#define TO_ARM_HOST			18
#define FROM_ARM_HOST			19

#define INT_ENABLE (1 << 5)
#define INT_OFFSET 16

/*
 * Using the name 'rpmsg-pru' will probe the rpmsg_pru driver found
 * at linux-x.y.z/drivers/rpmsg/rpmsg_pru.c
 */
#define CHAN_NAME			"rpmsg-sdemu"
#define CHAN_DESC			"DATA Channel"
#define CHAN_PORT			1

/*
 * Used to make sure the Linux drivers are ready for RPMsg communication
 * Found at linux-x.y.z/include/uapi/linux/virtio_config.h
 */
#define VIRTIO_CONFIG_S_DRIVER_OK	4


//#define PRU_DATA __far __attribute__((cregister("PRU_DMEM_DATA", near)))

#pragma DATA_SECTION(requested_mode, ".pru1data");
#pragma RETAIN(requested_mode)
volatile enum pru1_mode requested_mode;

#pragma DATA_SECTION(active_mode, ".pru1data");
#pragma RETAIN(active_mode)
volatile enum pru1_mode active_mode;

#pragma DATA_SECTION(sector, ".pru1data");
#pragma RETAIN(sector)
volatile uint32_t sector;

struct sdemu_msg_sector current_sector;

static uint8_t buf[512];
static uint8_t *bufp = buf;
uint32_t is_initialized = 0;

struct pru_rpmsg_transport transport;
uint16_t src, dst;

/* SD Card Configuration Register */
typedef struct scr {
	uint8_t sd_spec:4;
	uint8_t scr_structure:4;

	uint8_t sd_bus_widths:4;
	uint8_t sd_security:3;
	uint8_t data_stat_after_erase:1;

	uint8_t res1:2;
	uint8_t sd_spec4:1;
	uint8_t ex_security:4;
	uint8_t sd_spec3:1;

	uint8_t cmd_support:4;
	uint8_t res2:4;

	uint8_t res3[4];

	uint8_t crc16_hi;
	uint8_t crc16_lo;
} scr_t;

#define SD_BUS_1BIT		0x1
#define SD_BUS_4BIT		0x4

scr_t scr = {
		.scr_structure = 0x0,		/* Version 1.0 */
		.sd_spec = 0x2,				/* Version 2.00 (SDHC) */
		.data_stat_after_erase = 0x1,
		.sd_security = 0x3,			/* SDHC */
		.sd_bus_widths = SD_BUS_1BIT, /* Single DAT line only for now */
};

uint8_t switch_mode[64 + 2] = {
		0x00,		/* Maximum current consumption */
		0x01,
		0x80,		/* Supported group 6 functions */
		0x01,
		0x80,		/* Supported group 5 functions */
		0x01,
		0x80,		/* Supported group 4 functions */
		0x01,
		0x80,		/* Supported group 3 functions */
		0x01,
		0x80,		/* Supported group 2 functions */
		0x01,
		0x80,		/* Supported group 1 functions */
		0x01,
};

/* SD Status Register */
typedef struct sd_status {
	uint8_t res1:6;
	uint8_t dat_bus_width:2;

	uint8_t res2;
	uint8_t sd_card_type[2];
	uint8_t size_of_protected_area[4];
	uint8_t speed_class;
	uint8_t performance_move;

	uint8_t res3:4;
	uint8_t au_size:4;

	uint8_t erase_size_hi;
	uint8_t erase_size_lo;

	uint8_t erase_offset:2;
	uint8_t erase_timeout:6;

	uint8_t uhs_au_size:4;
	uint8_t uhs_speed_grade:4;

	uint8_t res4[49];

	uint8_t crc16_hi;
	uint8_t crc16_lo;
} sd_status_t;

sd_status_t sd_status_1bit = {
		.dat_bus_width = 0, /* 1 bit */
		.sd_card_type = { 0x00, 0x00 },
		.speed_class = 0x04, /* Class 10 */
		.performance_move = 0, /* Sequential Write */
		.au_size = 1, /* 16 kb */
		.erase_size_lo = 1, /* 1 AU */
		.erase_timeout = 63, /* 63 sec */
		.erase_offset = 3, /* 3 sec */
};

sd_status_t sd_status_4bit = {
		.dat_bus_width = 2, /* 4 bit */
		.sd_card_type = { 0x00, 0x00 },
		.speed_class = 0x04, /* Class 10 */
		.performance_move = 0, /* Sequential Write */
		.au_size = 1, /* 16 kb */
		.erase_size_lo = 1, /* 1 AU */
		.erase_timeout = 63, /* 63 sec */
		.erase_offset = 3, /* 3 sec */
};

static uint16_t crc16(uint8_t *message, int nBytes,
        uint16_t remainder, uint16_t polynomial)
{
	int byte;
	uint8_t bit;

    for (byte = 0; byte < nBytes; ++byte) {
        remainder ^= (message[byte] << 8);
        for (bit = 8; bit > 0; --bit) {
            if (remainder & 0x8000) {
                remainder = (remainder << 1) ^ polynomial;
            } else {
                remainder = (remainder << 1);
            }
        }
    }
    return remainder;
}

static uint16_t crc16ccitt_xmodem(uint8_t *message, int nBytes) {
    return crc16(message, nBytes, 0x0000, 0x1021);
}

static void init_scr_crc16(void)
{
	uint16_t crc16 = crc16ccitt_xmodem((void*)&scr, sizeof(uint64_t));

	scr.crc16_hi = crc16 >> 8;
	scr.crc16_lo = crc16;
}

static void init_switch_mode_crc16(void)
{
	uint16_t crc16 = crc16ccitt_xmodem((void*)switch_mode, 64);

	switch_mode[64] = crc16 >> 8;
	switch_mode[65] = crc16;
}

static void init_sd_status_crc16(void)
{
	uint16_t crc16;

	crc16 = crc16ccitt_xmodem((void*)&sd_status_1bit, sizeof(sd_status_1bit));

	sd_status_1bit.crc16_hi = crc16 >> 8;
	sd_status_1bit.crc16_lo = crc16;

	crc16 = crc16ccitt_xmodem((void*)&sd_status_4bit, sizeof(sd_status_4bit));

	sd_status_4bit.crc16_hi = crc16 >> 8;
	sd_status_4bit.crc16_lo = crc16;
}

static uint8_t *alloc_buf(int len)
{
	uint8_t *p = bufp;

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

static void enable_pins(void)
{
	/* Configure R30/R31 GPIOs to directly go to register values */
	CT_CFG.GPCFG0 = 0x0000;
}

static void print_int(const char *str, register int line_nr)
{
	int i, slen = strlen(str);
	uint8_t *b = alloc_buf(1 + slen + 9);
	uint8_t *nump;

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

static void drain_rpmsg(void)
{
	/* Check bit 30 of register R31 to see if the ARM has kicked us */
	if (__R31 & HOST_INT) {
		uint16_t len;
		uint16_t tmp_src, tmp_dst;

		clear_arm_irq();
		while (pru_rpmsg_receive(&transport, &tmp_src, &tmp_dst, buf, &len) == PRU_RPMSG_SUCCESS) ;
	}
}

static void read_sector(void)
{
	uint16_t len;
	uint16_t tmp_src, tmp_dst;
	uint8_t *p = (void*)&current_sector;

	current_sector.cmd = SDEMU_MSG_READ_SECTOR;
	current_sector.offset = ((uint64_t)sector) * 512;

	drain_rpmsg();
	pru_rpmsg_send(&transport, dst, src, &current_sector, 16);

	/* Wait for and receive reply */
	while (1) {
		int r;

		/* We can read max (512-16) bytes in one transaction, so we split the transfer in 2. */
		r = pru_rpmsg_receive(&transport, &tmp_src, &tmp_dst, p, &len);
		if (r == PRU_RPMSG_SUCCESS)
			break;
	}
	clear_arm_irq();

	if (len != (512-16)) {
		print_int("Short 1st read: ", len);
	}

	p += len;

	/* Second half of the transfer */
	while (pru_rpmsg_receive(&transport, &tmp_src, &tmp_dst, p, &len) != PRU_RPMSG_SUCCESS) ;
	clear_arm_irq();

	if (len != (sizeof(current_sector) - (512-16))) {
		print_int("Short 2nd read: ", len);
	}
}

extern void send_buf_1bit_startend(register uint8_t *buf, register uint32_t len);
extern void send_buf_4bit_startend(register uint8_t *buf, register uint32_t len);

static void read_data_4bit(void)
{
	print_int("Reading Sector: ", sector);

	/* Read sector from host */
	read_sector();

	print_int("Sending Sector 4b: ", sector);

	send_buf_4bit_startend(current_sector.data, 514);
	sector++;
}

static void read_data_1bit(void)
{
	print_int("Reading Sector: ", sector);

	/* Read sector from host */
	read_sector();

	print_int("Sending Sector 1b: ", sector);

	send_buf_1bit_startend(current_sector.data, 514);
	sector++;
}

static void write_data_4bit(void)
{
	/* Receive data from the host */

}

static void send_scr_4bit(void)
{
	register uint8_t *b = (void*)&scr;

	print_int("Reading SCR[0]: ", *(uint32_t*)b);
	print_int("Reading SCR[1]: ", *(uint32_t*)(b+4));

	/* Emit data to the host */
	send_buf_4bit_startend(b, sizeof(scr));
}

static void send_scr_1bit(void)
{
	register uint8_t *b = (void*)&scr;

	print_int("Reading SCR[0]: ", *(uint32_t*)b);
	print_int("Reading SCR[1]: ", *(uint32_t*)(b+4));
	print_int("Reading SCR[2]: ", *(uint16_t*)(b+8));

	/* Emit data to the host */
	send_buf_1bit_startend(b, sizeof(scr));

	print_int("Finished sending SCR  ", 0);
}

static void send_switch(void)
{
	register uint8_t *b = (void*)&switch_mode;

	print_int("Reading switch[0]: ", *(uint32_t*)b);
	print_int("Reading switch[1]: ", *(uint32_t*)(b+4));
	print_int("Reading switch[2]: ", *(uint32_t*)(b+8));

	/* Emit data to the host */
	send_buf_1bit_startend(b, sizeof(switch_mode));

	print_int("Finished sending switch_mode  ", 0);
}

static void send_sd_status_4bit(void)
{
	register uint8_t *b = (void*)&sd_status_4bit;

	/* Emit data to the host */
	send_buf_4bit_startend(b, sizeof(sd_status_4bit));
}

static void send_sd_status_1bit(void)
{
	register uint8_t *b = (void*)&sd_status_1bit;

	/* Emit data to the host */
	send_buf_1bit_startend(b, sizeof(sd_status_1bit));
}
void return_to_dat_main(void);

/*
 * main.c
 */
void main(void)
{
	volatile uint8_t *status;

	active_mode = requested_mode = pru1_mode_idle;

	/* allow OCP master port access by the PRU so the PRU can read external memories */
	CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

	/* clear the status of the PRU-ICSS system event that the ARM will use to 'kick' us */
	clear_arm_irq();

	/* Initialize internal state */
	init_scr_crc16();
	init_switch_mode_crc16();
	init_sd_status_crc16();

	/* Make sure the Linux drivers are ready for RPMsg communication */
	status = &resourceTable.rpmsg_vdev.status;
	while (!(*status & VIRTIO_CONFIG_S_DRIVER_OK));

	/* Initialize the RPMsg transport structure */
	pru_rpmsg_init(&transport, &resourceTable.rpmsg_vring0, &resourceTable.rpmsg_vring1, TO_ARM_HOST, FROM_ARM_HOST);

	/* Create the RPMsg channel between the PRU and ARM user space using the transport structure. */
	while (pru_rpmsg_channel(RPMSG_NS_CREATE, &transport, CHAN_NAME, CHAN_DESC, CHAN_PORT) != PRU_RPMSG_SUCCESS) ;

	/* Set up return point for soft reset */
	PRU1_CTRL.CTRL_bit.PCTR_RST_VAL = (uintptr_t)(void*)return_to_dat_main;

	asm("  xout 10, &R0, 120");
	asm("  .global return_to_dat_main");
	asm("return_to_dat_main:");
	asm("  xin 10, &R0, 120");

	while (1) {
		/* Check bit 31 of register R31 to see if the ARM has kicked us */
		if (__R31 & HOST_INT) {
			uint16_t len;

			/* Clear the event status */
			clear_arm_irq();

			/* Receive all available messages, multiple messages can be sent per kick */
			while (pru_rpmsg_receive(&transport, &src, &dst, buf, &len) == PRU_RPMSG_SUCCESS) {
				if (!is_initialized) {
					is_initialized = 1;
					print_int("Hello from the DATA PRU!", 0);

					enable_pins();
				}
			}
		}

		if (is_initialized) {
			if (requested_mode != active_mode)
				print_int("New mode:", requested_mode);

			switch (requested_mode) {
			case pru1_mode_read_1bit:
				print_int("Requested Read sector 1b:", sector);

				active_mode = pru1_mode_read_1bit;
				read_data_1bit();
				__R30 = DAT0_MASK | DAT1_MASK | DAT2_MASK | DAT3_MASK |
						DAT0_OUT_MASK | DAT1_OUT_MASK | DAT2_OUT_MASK | DAT3_OUT_MASK;
				break;
			case pru1_mode_read_4bit:
				print_int("Requested Read sector 4b:", sector);

				active_mode = pru1_mode_read_4bit;
				read_data_4bit();
				__R30 = DAT0_MASK | DAT1_MASK | DAT2_MASK | DAT3_MASK |
						DAT0_OUT_MASK | DAT1_OUT_MASK | DAT2_OUT_MASK | DAT3_OUT_MASK;
				break;
			case pru1_mode_write_4bit:
				active_mode = pru1_mode_write_4bit;
				write_data_4bit();
				break;
			case pru1_mode_read_scr_4bit:
				if (active_mode != pru1_mode_read_scr_4bit)
					send_scr_4bit();

				__R30 = DAT0_MASK | DAT1_MASK | DAT2_MASK | DAT3_MASK |
						DAT0_OUT_MASK | DAT1_OUT_MASK | DAT2_OUT_MASK | DAT3_OUT_MASK;
				active_mode = pru1_mode_read_scr_4bit;
				break;
			case pru1_mode_read_scr_1bit:
				if (active_mode != pru1_mode_read_scr_1bit)
					send_scr_1bit();

				__R30 = DAT0_MASK | DAT1_MASK | DAT2_MASK | DAT3_MASK |
						DAT0_OUT_MASK | DAT1_OUT_MASK | DAT2_OUT_MASK | DAT3_OUT_MASK;
				active_mode = pru1_mode_read_scr_1bit;
				break;
			case pru1_mode_send_switch:
				if (active_mode != pru1_mode_send_switch)
					send_switch();

				__R30 = DAT0_MASK | DAT1_MASK | DAT2_MASK | DAT3_MASK |
						DAT0_OUT_MASK | DAT1_OUT_MASK | DAT2_OUT_MASK | DAT3_OUT_MASK;
				active_mode = pru1_mode_send_switch;
				break;
			case pru1_mode_read_sd_status_4bit:
				if (active_mode != pru1_mode_read_sd_status_4bit)
					send_sd_status_4bit();

				__R30 = DAT0_MASK | DAT1_MASK | DAT2_MASK | DAT3_MASK |
						DAT0_OUT_MASK | DAT1_OUT_MASK | DAT2_OUT_MASK | DAT3_OUT_MASK;
				active_mode = pru1_mode_read_sd_status_4bit;
				break;
			case pru1_mode_read_sd_status_1bit:
				if (active_mode != pru1_mode_read_sd_status_1bit)
					send_sd_status_1bit();

				__R30 = DAT0_MASK | DAT1_MASK | DAT2_MASK | DAT3_MASK |
						DAT0_OUT_MASK | DAT1_OUT_MASK | DAT2_OUT_MASK | DAT3_OUT_MASK;
				active_mode = pru1_mode_read_sd_status_1bit;
				break;
			case pru1_mode_idle:
				__R30 = DAT0_MASK | DAT1_MASK | DAT2_MASK | DAT3_MASK |
						DAT0_OUT_MASK | DAT1_OUT_MASK | DAT2_OUT_MASK | DAT3_OUT_MASK;
				active_mode = pru1_mode_idle;
				break;
			}
		}
	}
}
