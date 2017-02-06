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

/* R30/R31 bit masks for I/O lines */
#define DAT0_MASK	(1 << 8)
#define DAT1_MASK	(1 << 9)
#define DAT2_MASK	(1 << 10)
#define DAT3_MASK	(1 << 11)
#define CLK_MASK	(1 << 16)

/* Host-1 Interrupt sets bit 31 in register R31 */
#define HOST_INT			((uint32_t) 1 << 31)

/* The PRU-ICSS system events used for RPMsg are defined in the Linux device tree
 * PRU0 uses system event 16 (To ARM) and 17 (From ARM)
 * PRU1 uses system event 18 (To ARM) and 19 (From ARM)
 */
#define TO_ARM_HOST			18
#define FROM_ARM_HOST			19

#define FAST_INT 0x1d

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

//#define PRU_DATA __far __attribute__((cregister("PRU_DMEM_DATA", near)))
//#define PRU_FAST __far __attribute__((cregister("PRU_DMEM_FAST", near)))

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

uint8_t data_buf[512 + 4];

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
	fast_cmd = SDEMU_MSG_SETPINS_DAT_IN;
	__R31 = (INT_ENABLE | (FAST_INT - INT_OFFSET));

	/* Wait for msg to comlete */
	while (fast_cmd != SDEMU_MSG_DONE) ;

	in_send_mode = RECV_MODE;
}

static void switch_to_send(void)
{
	if (in_send_mode == SEND_MODE)
		return;

	/* Pull DATA lines high, so that nobody gets the idea we're switched yet */
	__R30 = DAT0_MASK | DAT1_MASK | DAT2_MASK | DAT3_MASK;

	/* Send ARM host message */
	fast_cmd = SDEMU_MSG_SETPINS_DAT_OUT;
	__R31 = (INT_ENABLE | (FAST_INT - INT_OFFSET));

	/* Wait for msg to comlete */
	while (fast_cmd != SDEMU_MSG_DONE) ;

	in_send_mode = SEND_MODE;
}

static void enable_pins(void)
{
	/* Configure R30/R31 GPIOs to directly go to register values */
	CT_CFG.GPCFG0 = 0x0000;

	switch_to_recv();
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

static void read_sector(void)
{
	uint8_t *buf = alloc_buf(17);
	uint64_t addr = (uint64_t)sector * 512;

	/* Send ARM host message */
	fast_cmd = SDEMU_MSG_PREAD_4BIT;
	fast_arg1 = (long)data_buf;
	fast_arg2 = (long)addr;
	fast_arg3 = 512;
	__R31 = (INT_ENABLE | (FAST_INT - INT_OFFSET));

	/* Wait for msg to comlete */
	while (fast_cmd != SDEMU_MSG_DONE) ;
}

static inline void send_4bit(register uint32_t b)
{
	/* Write data bits */

	/* Wait for a tick (clk is down) */
	while (__R31 & CLK_MASK) ;

	/* Set lines */
	__R30 = b << 8;

	/* Wait until clk is up again (so the host can read the bit) */
	while (!(__R31 & CLK_MASK)) ;
}

/* shift may go from 0 (1<<0) up to 7 (1<<7) */
static inline void send_1bit(register uint32_t b, register uint32_t shift)
{
	/* Wait for a tick (clk is down) */
	while (__R31 & CLK_MASK) ;

	/* Set lines */
	if (shift < 8)
		__R30 = b << (8 - shift);
	else
		__R30 = b >> (shift - 8);

	/* Wait until clk is up again (so the host can read the bit) */
	while (!(__R31 & CLK_MASK)) ;
}

static inline void wait_for_clk_low(void)
{
	/* Wait for a tick (clk is down) */
	while (__R31 & CLK_MASK) ;
}

static inline void wait_for_clk_high(void)
{
	/* Wait until clk is up again (so the host can read the bit) */
	while (!(__R31 & CLK_MASK)) ;
}

static inline void set_line_1bit(uint32_t b, uint32_t shift)
{
	/* Set lines */
	if (shift < 8)
		__R30 = b << (8 - shift);
	else
		__R30 = b >> (shift - 8);
}

#if 0
#pragma FUNC_CANNOT_INLINE(send_buf_1bit_startend)

static void send_buf_1bit_startend(register uint8_t *buf, register uint32_t len)
{
	uint32_t curbyte, nextbyte;

	nextbyte = *buf;

	/* Start Bit */
	wait_for_clk_low();
	set_line_1bit(0, 0);
	wait_for_clk_high();

	do {
		curbyte = nextbyte;

		/* Bit 0 - use gap to inc ptr */
		wait_for_clk_low();
		set_line_1bit(curbyte, 0);
		wait_for_clk_high();
		buf++;
		asm volatile("");

		/* Bit 1 - use gap to load next byte */
		wait_for_clk_low();
		set_line_1bit(curbyte, 1);
		wait_for_clk_high();
		nextbyte = *buf;
		asm volatile("");

		/* Bit 2 - use gap to dec len */
		wait_for_clk_low();
		set_line_1bit(curbyte, 2);
		wait_for_clk_high();
		len--;
		asm volatile("");

		/* Bit 3 - nop in gap */
		wait_for_clk_low();
		set_line_1bit(curbyte, 3);
		wait_for_clk_high();

		/* Bit 4 - nop in gap */
		wait_for_clk_low();
		set_line_1bit(curbyte, 4);
		wait_for_clk_high();

		/* Bit 5 - nop in gap */
		wait_for_clk_low();
		set_line_1bit(curbyte, 5);
		wait_for_clk_high();

		/* Bit 6 - nop in gap */
		wait_for_clk_low();
		set_line_1bit(curbyte, 6);
		wait_for_clk_high();

		/* Bit 7 - loop in gap */
		wait_for_clk_low();
		set_line_1bit(curbyte, 7);
		wait_for_clk_high();

	} while(len);

	/* End Bit */
	wait_for_clk_low();
	set_line_1bit(1, 0);
	wait_for_clk_high();
}
#else
extern void send_buf_1bit_startend(register uint8_t *buf, register uint32_t len);
#endif

static inline void send_8x1bit(register uint32_t byte)
{
	send_1bit(byte, 7);
	send_1bit(byte, 6);
	send_1bit(byte, 5);
	send_1bit(byte, 4);
	send_1bit(byte, 3);
	send_1bit(byte, 2);
	send_1bit(byte, 1);
	send_1bit(byte, 0);
}

static inline void send_16x1bit(register uint32_t byte)
{
	send_1bit(byte, 15);
	send_1bit(byte, 14);
	send_1bit(byte, 13);
	send_1bit(byte, 12);
	send_1bit(byte, 11);
	send_1bit(byte, 10);
	send_1bit(byte, 9);
	send_1bit(byte, 8);
	send_1bit(byte, 7);
	send_1bit(byte, 6);
	send_1bit(byte, 5);
	send_1bit(byte, 4);
	send_1bit(byte, 3);
	send_1bit(byte, 2);
	send_1bit(byte, 1);
	send_1bit(byte, 0);
}

static inline void send_32x1bit(register uint32_t byte)
{
	send_1bit(byte, 31);
	send_1bit(byte, 30);
	send_1bit(byte, 29);
	send_1bit(byte, 28);
	send_1bit(byte, 27);
	send_1bit(byte, 26);
	send_1bit(byte, 25);
	send_1bit(byte, 24);
	send_1bit(byte, 23);
	send_1bit(byte, 22);
	send_1bit(byte, 21);
	send_1bit(byte, 20);
	send_1bit(byte, 19);
	send_1bit(byte, 18);
	send_1bit(byte, 17);
	send_1bit(byte, 16);
	send_1bit(byte, 15);
	send_1bit(byte, 14);
	send_1bit(byte, 13);
	send_1bit(byte, 12);
	send_1bit(byte, 11);
	send_1bit(byte, 10);
	send_1bit(byte, 9);
	send_1bit(byte, 8);
	send_1bit(byte, 7);
	send_1bit(byte, 6);
	send_1bit(byte, 5);
	send_1bit(byte, 4);
	send_1bit(byte, 3);
	send_1bit(byte, 2);
	send_1bit(byte, 1);
	send_1bit(byte, 0);
}

static void read_data_4bit(void)
{
	register uint8_t *b = data_buf;
	register uint8_t *b_end = &data_buf[514];

	print_int("Reading Sector: ", sector);

	/* Emit data to the host */
	if (0) {
		read_sector();
	} else {
		int i;
		uint16_t crc16;

		/* XXX Incorrect length! Needs to be 512 bytes */
		for (i = 0; i < 512; i+=2) {
			b[i] = 0x12;
			b[i + 1] = 0x48;
		}

		crc16 = crc16ccitt_xmodem(b, 512);
		b[512] = crc16 >> 8;
		b[513] = crc16;
	}

	/*
	 * Wait for clock to come up, so we that get a full
	 * clock cycle for the next transmission
	 */
	while (!(__R31 & CLK_MASK)) ;

	/* Start Bit */
	send_4bit(0);

	/* Data */
	do {
		send_4bit(*b >> 4);
		send_4bit(*b);
		b++;
	} while(b != b_end);

	/* End Bit */
	send_4bit(0xf);
}

static void read_data_1bit(void)
{
	register uint32_t *b = (void*)data_buf;
	register uint32_t *b_end;
	register uint32_t crc16;
	register uint32_t i;

	print_int("Reading Sector: ", sector);

	/* Emit data to the host */
	if (0) {
		read_sector();
	} else {
		for (i = 0; i < (512 / 4); i+=4) {
			b[i] = 0xAAAAAAAA;
			b[i+1] = 0x0;
			b[i+2] = 0x1;
			b[i+3] = 0xFFFFFFFF;
		}

		crc16 = crc16ccitt_xmodem(data_buf, 512);
		data_buf[512] = crc16 >> 8;
		data_buf[513] = crc16;
	}

	asm volatile (" nop \n");
	asm volatile (" nop \n");
	asm volatile (" nop \n");
	asm volatile (" nop \n");

	send_buf_1bit_startend(data_buf, 514);
#if 0
	b_end = (void*)&data_buf[512];

	/* Start Bit */
	send_1bit(0, 0);

	/* Data */
	for (i = 0; i < (512 / 4); i++)
		send_32x1bit(b[i]);

	/* CRC16 */
	send_16x1bit(crc16);

	/* End Bit */
	send_1bit(1, 0);
#endif


	asm volatile (" nop \n");
	asm volatile (" nop \n");
	asm volatile (" nop \n");
	asm volatile (" nop \n");
}

static void write_data_4bit(void)
{
	/* Receive data from the host */

}

static void send_scr_4bit(void)
{
	register char *b = (void*)&scr;
	register char *b_end = b + sizeof(scr);

	print_int("Reading SCR[0]: ", *(uint32_t*)b);
	print_int("Reading SCR[1]: ", *(uint32_t*)(b+4));

	/* Emit data to the host */

	/*
	 * Wait for clock to come up, so we that get a full
	 * clock cycle for the next transmission
	 */
	while (!(__R31 & CLK_MASK)) ;

	/* Start Bit */
	send_4bit(0);

	/* Data */
	do {
		send_4bit(*b >> 4);
		send_4bit(*b);
		b++;
	} while(b != b_end);

	/* End - just in case */
	send_4bit(0xf);
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

	/* Make sure the Linux drivers are ready for RPMsg communication */
	status = &resourceTable.rpmsg_vdev.status;
	while (!(*status & VIRTIO_CONFIG_S_DRIVER_OK));

	/* Initialize the RPMsg transport structure */
	pru_rpmsg_init(&transport, &resourceTable.rpmsg_vring0, &resourceTable.rpmsg_vring1, TO_ARM_HOST, FROM_ARM_HOST);

	/* Create the RPMsg channel between the PRU and ARM user space using the transport structure. */
	while (pru_rpmsg_channel(RPMSG_NS_CREATE, &transport, CHAN_NAME, CHAN_DESC, CHAN_PORT) != PRU_RPMSG_SUCCESS) ;

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
				print_int("Requested Read sector:", sector);

				active_mode = pru1_mode_read_1bit;
				read_data_1bit();
				__R30 = DAT0_MASK | DAT1_MASK | DAT2_MASK | DAT3_MASK;
				break;
			case pru1_mode_read_4bit:
				print_int("Requested Read sector:", sector);

				active_mode = pru1_mode_read_4bit;
				read_data_4bit();
				__R30 = DAT0_MASK | DAT1_MASK | DAT2_MASK | DAT3_MASK;
				break;
			case pru1_mode_write_4bit:
				active_mode = pru1_mode_write_4bit;
				write_data_4bit();
				break;
			case pru1_mode_set_recv:
				if (active_mode != pru1_mode_set_recv)
					switch_to_recv();

				active_mode = pru1_mode_set_recv;
				break;
			case pru1_mode_set_send:
				if (active_mode != pru1_mode_set_send) {
					__R30 = DAT0_MASK | DAT1_MASK | DAT2_MASK | DAT3_MASK;
					switch_to_send();
				}

				active_mode = pru1_mode_set_send;
				break;
			case pru1_mode_read_scr_4bit:
				if (active_mode != pru1_mode_read_scr_4bit)
					send_scr_4bit();

				__R30 = DAT0_MASK | DAT1_MASK | DAT2_MASK | DAT3_MASK;
				active_mode = pru1_mode_read_scr_4bit;
				break;
			case pru1_mode_read_scr_1bit:
				if (active_mode != pru1_mode_read_scr_1bit)
					send_scr_1bit();

				__R30 = DAT0_MASK | DAT1_MASK | DAT2_MASK | DAT3_MASK;
				active_mode = pru1_mode_read_scr_1bit;
				break;
			case pru1_mode_idle:
				active_mode = pru1_mode_idle;
				break;
			}
		}
	}
}
