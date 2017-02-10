/*
 * Copyright (C) 2017 Alexander Graf <agraf@suse.de>
 *
 * SPDX-License-Identifier:      GPL-2.0
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>

enum fd_type {
    FD_TYPE_CMD = 0,
    FD_TYPE_DATA,
    FD_TYPE_MAX,
};
int fd_list[] = { 0, 0 };

static int handle_fd(int fd)
{
    const char *prustr = (fd == fd_list[0]) ? "cmd" : "data";
    char buf[512] = { 0 };

    read(fd, buf, sizeof(buf));
    printf("[%s] %s", prustr, buf);
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

    for (i = 0; i < FD_TYPE_MAX; i++) {
        char path[512];

        sprintf(path, "/dev/rpmsg_sdemu%d", i);
        fd_list[i] = open(path, O_RDWR);
        nfds = check_fd(fd_list[i], nfds);
        start_channel(fd_list[i]);
    }

    while (1) {
        int r, i;
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
