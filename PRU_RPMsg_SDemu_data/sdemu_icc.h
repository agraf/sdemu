/*
 * sdemu_icc.h
 *
 *  Created on: Feb 10, 2017
 *      Author: agraf
 */

#ifndef SDEMU_ICC_H_
#define SDEMU_ICC_H_


/* ICC between PRU0 and PRU1 */

enum pru1_mode {
	pru1_mode_idle,
	pru1_mode_read_4bit,
	pru1_mode_write_4bit,
	pru1_mode_read_1bit,
	pru1_mode_write_1bit,
	pru1_mode_read_scr_4bit,
	pru1_mode_read_scr_1bit,
	pru1_mode_send_switch_1bit,
	pru1_mode_send_switch_4bit,
	pru1_mode_read_sd_status_1bit,
	pru1_mode_read_sd_status_4bit,
	pru1_mode_read_num_wr_blocks_1bit,
	pru1_mode_read_num_wr_blocks_4bit,
};

/* ICC between PRU and ARM */

enum sdemu_msg_cmd {
	/* FAST commands */

	SDEMU_MSG_SETPINS_CMD_IN		= '1',
	SDEMU_MSG_SETPINS_CMD_OUT		= '2',
	SDEMU_MSG_SETPINS_DAT_IN		= '3',
	SDEMU_MSG_SETPINS_DAT_OUT		= '4',
	SDEMU_MSG_SETPINS_SPI			= '5',
	SDEMU_MSG_SETPINS_RESET			= '6',
	SDEMU_MSG_QUERY_SD_INIT			= 'I',
	SDEMU_MSG_SET_SIZE				= 'S',
	SDEMU_MSG_PREAD_4BIT			= 'R',
	SDEMU_MSG_PWRITE_4BIT			= 'W',
	SDEMU_MSG_DONE					= '\0',

	/* RPMSG commands */
	SDEMU_MSG_DBG					= ' ',
	SDEMU_MSG_GET_SIZE,
	SDEMU_MSG_READ_SECTOR,
	SDEMU_MSG_WRITE_SECTOR,
};

struct sdemu_msg_dbg {
    uint8_t cmd;
    uint8_t data[512];
};

struct sdemu_msg_get_size {
    uint8_t cmd;
    uint8_t pad[7];
    uint64_t size;
};

struct sdemu_msg_sector {
    uint8_t cmd;
    uint8_t is_4bit;
    uint8_t pad[6];
    uint64_t offset;
    uint8_t data[512 + (2 * 4)];
};

union sdemu_msg {
    struct sdemu_msg_get_size get_size;
    struct sdemu_msg_sector sector;
};

enum sdemu_hwversion {
	SDEMU_V1_0,
	SDEMU_V1_1,
};

#endif /* SDEMU_ICC_H_ */
