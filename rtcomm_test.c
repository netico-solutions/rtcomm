#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdint.h>

#include "rtcomm.h"

int main(void)
{
        int                     fd;
        int                     buffer_size;
        int                     count;
        char                    version[20];
        char                    buffer[2000];

        fprintf(stdout, "RTCOMM drv test 1, ver:" __DATE__ " : " __TIME__ "\n");

        fd = open("/dev/" RTCOMM_NAME, O_RDONLY);

        if (!fd) {
                fprintf(stderr, "Failed to open driver: %d\n", errno);
        }

        if (ioctl(fd, RTCOMM_GET_VERSION, version) == -1) {
                fprintf(stderr, "RTCOMM_GET_VERSION failed: %d\n", errno);

                return (-1);
        } else {
                fprintf(stdout, "RTCOMM_GET_VERSION get: %s\n", version);
        }

        buffer_size = sizeof(buffer);

        if (ioctl(fd, RTCOMM_SET_SIZE, &buffer_size) == -1) {
                fprintf(stderr, "RTCOMM_SET_SIZE failed: %d\n", errno);

                return (-1);
        } else {
                fprintf(stdout, "RTCOMM_SET_SIZE set: %d\n", buffer_size);
        }

        if (ioctl(fd, RTCOMM_START)) {
                fprintf(stderr, "RTCOMM_START failed: %d\n", errno);

                return (-1);
        } else {
                fprintf(stdout, "RTCOMM_START success.\n");
        }
        count = 0;

        for (;;) {
                int             ret;

                ret = read(fd, buffer, buffer_size);

                if (ret != buffer_size) {
                        fprintf(stderr, "Failed to read %d bytes, error: %d\n",
                                        buffer_size, ret);

                        return (-1);
                } else {
                        fprintf(stdout, " %05d: read %d bytes\n", count++,
                                        buffer_size);
                                uint32_t idx;
                                uint32_t idxl;

                                for (idx = 0; idx < 64; idx += 8) {
                                        printf("\ti %05d: ", idx);
                                        for (idxl = 0; idxl < 8; idxl++) {
                                                printf("%x ",
                                                        buffer[idxl + idx]);
                                        }
                                        printf("\n");
                                }
                }
        }

        return (0);
}
