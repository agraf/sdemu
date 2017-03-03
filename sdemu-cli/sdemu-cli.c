/*
 * Copyright (C) 2017 Alexander Graf <agraf@suse.de>
 *
 * SPDX-License-Identifier:      GPL-2.0
 */

#define _LARGEFILE64_SOURCE     /* See feature_test_macros(7) */
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

enum fd_type {
    FD_TYPE_CMD = 0,
    FD_TYPE_DATA,
    FD_TYPE_MAX,
};
int fd_list[] = { 0, 0 };
uint64_t filesize = 0x1000; /* XXX */
int filefd;

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

static uint16_t crc16_one_datline(char *buf, int line)
{
    char datbuf[512 / 4] = { 0 };
    int i;

    for (i = 0; i < 512; i++) {
        datbuf[i/4] <<= 2;
        datbuf[i/4] |= (((buf[i] >> line) & 0x10)) ? 0x2 : 0x0;
        datbuf[i/4] |= ((buf[i] >> line) & 0x1);
    }

    return crc16ccitt_xmodem(datbuf, sizeof(datbuf));
}

static void set_bit(char *buf, int idx, int is_one)
{
    unsigned char bit = 1 << (0x7 - (idx & 0x7));

    if (is_one)
        buf[idx / 8] |= bit;
    else
        buf[idx / 8] &= ~bit;
}

static void read_sector(uint64_t offset, char *buf, int is_4bit)
{

    if (filefd) {
        int r;

        lseek64(filefd, offset, SEEK_SET);
        r = read(filefd, buf, 512);

        if (r != 512) {
            printf("Error: short read: %d (offset=%#llx)\n", r, (long long)offset);
            memset(buf, 0x0, 512);
        }
    } else {
        int i;

        for (i = 0; i < 512; i+=4) {
            buf[i+0] = 0xAA;
            buf[i+1] = 0x00;
            buf[i+2] = 0xFF;
            buf[i+3] = 0x88;
        }
    }

    if (is_4bit) {
        uint16_t crc16[4] = {
            crc16_one_datline(buf, 0),
            crc16_one_datline(buf, 1),
            crc16_one_datline(buf, 2),
            crc16_one_datline(buf, 3),
        };
        int i, j = 0;
        char *crc16_buf = &buf[512];

        for (i = 0; i < 16; i++) {
            set_bit(crc16_buf, j++, crc16[3] & (1 << (15 - i)));
            set_bit(crc16_buf, j++, crc16[2] & (1 << (15 - i)));
            set_bit(crc16_buf, j++, crc16[1] & (1 << (15 - i)));
            set_bit(crc16_buf, j++, crc16[0] & (1 << (15 - i)));
        }
    } else {
        uint16_t crc16 = crc16ccitt_xmodem(buf, 512);

        buf[512] = crc16 >> 8;
        buf[513] = crc16;
    }
}

enum sdemu_msg_cmd {
    SDEMU_MSG_DBG = ' ',
    SDEMU_MSG_GET_SIZE,
    SDEMU_MSG_READ_SECTOR,
    SDEMU_MSG_WRITE_SECTOR,
};

struct sdemu_msg_dbg {
    uint8_t cmd;
    uint8_t data[512-16-1];
} __attribute__((packed));

struct sdemu_msg_get_size {
    uint8_t cmd;
    uint8_t pad[7];
    uint64_t size;
} __attribute__((packed));

struct sdemu_msg_sector {
    uint8_t cmd;
    uint8_t is_4bit;
    uint8_t pad[6];
    uint64_t offset;
    /* 512 bytes of data, crc16 for each dat line */
    uint8_t data[512 + (2 * 4)];
} __attribute__((packed));

union sdemu_msg {
    struct sdemu_msg_get_size get_size;
    struct sdemu_msg_sector sector;
    struct sdemu_msg_dbg dbg;
};

static void write_sector(struct sdemu_msg_sector *sec)
{
    int i;
    printf("Write sector %#lx\n", sec->offset);
    printf("Data: ");
    for (i = 0; i < sizeof(sec->data); i++)
        printf("%02x", sec->data[i]);
    printf("\n");
}

static int handle_fd(int fd)
{
    const char *prustr = (fd == fd_list[0]) ? "cmd" : "data";
    union sdemu_msg msg = { 0 };
    int len;

    len = read(fd, &msg, sizeof(msg));
    switch (msg.get_size.cmd) {
    case SDEMU_MSG_GET_SIZE:
    {
        struct sdemu_msg_get_size sz = {
            .cmd = SDEMU_MSG_GET_SIZE,
            .size = filesize,
        };
        write(fd, &sz, sizeof(sz));
        break;
    }
    case SDEMU_MSG_READ_SECTOR:
    {
        struct sdemu_msg_sector sec = {
            .cmd = SDEMU_MSG_READ_SECTOR,
            .offset = msg.sector.offset,
        };
        read_sector(msg.sector.offset, sec.data, msg.sector.is_4bit);
        /* We have a transmit limit of (512-16) bytes. Split the transfer in 2 */
        write(fd, &sec, 512-16);
        write(fd, ((void*)&sec) + (512-16), sizeof(sec) - (512-16));
        break;
    }
    case SDEMU_MSG_WRITE_SECTOR:
        read(fd, ((void*)&msg) + (512-16), sizeof(struct sdemu_msg_sector) - (512-16));
        write_sector(&msg.sector);
        break;
    case SDEMU_MSG_DBG:
        printf("[%4s] %s", prustr, msg.dbg.data);
        break;
    default:
        printf("[INVALID %s] cmd=%hhx (%s)", prustr, msg.get_size.cmd, (char*)&msg);
        break;
    }
}

static int start_channel(int fd)
{
    write(fd, "foo\n", 4);
}

static int check_fd(int fd, int nfds)
{
    if (fd < 0) {
        printf("Error: %d (%s)\n", errno, strerror(errno));
    }

    return ((fd + 1) < nfds) ? nfds : (fd + 1);
}

int main(int argc, char **argv)
{
    int nfds = 0;
    int i;

    if (argc > 1) {
        filefd = open(argv[1], O_RDWR);
        if (filefd < 0) {
            printf("Failed to open \"%s\"\n", argv[1]);
            exit(1);
        }

        filesize = lseek64(filefd, 0, SEEK_END);
        lseek64(filefd, 0, SEEK_SET);
    } else {
        printf("No file name passed in, using test pattern for reads\n");
    }

    for (i = 0; i < FD_TYPE_MAX; i++) {
        char path[512];

        sprintf(path, "/dev/rpmsg_sdemu%d", i);
        fd_list[i] = open(path, O_RDWR);
        nfds = check_fd(fd_list[i], nfds);
        start_channel(fd_list[i]);
    }

    while (1) {
        int r;
        struct timeval timeout = { .tv_sec = 5 };
        fd_set rfds, xfds;

        FD_ZERO(&rfds);
        FD_ZERO(&xfds);
        for (i = 0; i < FD_TYPE_MAX; i++) {
            FD_SET(fd_list[i], &rfds);
            FD_SET(fd_list[i], &xfds);
        }

        r = select(nfds, &rfds, NULL, &xfds, &timeout);

        for (i = 0; i < FD_TYPE_MAX; i++) {
            int fd = fd_list[i];

            if (FD_ISSET(fd, &rfds)) {
                handle_fd(fd);
            }
            if (FD_ISSET(fd, &xfds)) {
                printf("Lost connection with the PRU!\n");
                exit(1);
            }
        }
    }
}
