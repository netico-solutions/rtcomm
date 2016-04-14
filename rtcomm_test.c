#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "rtcomm.h"

int main(int argc, char * argv[])
{
        int                     fd;
        int                     buffer_size = 64;
        int                     max_size = 64;
        int                     count;
        char                    version[20];
        char *                  buffer;
        
        fprintf(stdout, "RTCOMM drv test 1, ver:" __DATE__ " : " __TIME__ "\n");

        switch (argc) {
                case 3: max_size = atoi(argv[2]);
                case 2:
                        buffer_size = atoi(argv[1]);
                        break;
                case 1:
                        break;
                default:
                        fprintf(stderr, "Usage rtcomm_test [buffer_size] [max_print_size]\n");

                        return (1);
        }
        fprintf(stdout, "Using buffer size %d bytes\n", buffer_size);
        fprintf(stdout, "Using max print size %d bytes\n", max_size);

        buffer = malloc(buffer_size);

        if (!buffer) {
                fprintf(stderr, "Couldn't allocate buffer of size %d\n", 
                                buffer_size);

                return (1);
        }

        sleep(1);

        fd = open("/dev/" RTCOMM_NAME, O_RDONLY);

        if (!fd) {
                fprintf(stderr, "Failed to open driver: %d\n", errno);
        }

        if (ioctl(fd, RTCOMM_GET_VERSION, version) == -1) {
                fprintf(stderr, "RTCOMM_GET_VERSION failed: %d\n", errno);

                return (1);
        } else {
                fprintf(stdout, "RTCOMM_GET_VERSION get: %s\n", version);
        }

        if (ioctl(fd, RTCOMM_SET_SIZE, &buffer_size) == -1) {
                fprintf(stderr, "RTCOMM_SET_SIZE failed: %d\n", errno);

                return (1);
        } else {
                fprintf(stdout, "RTCOMM_SET_SIZE set: %d\n", buffer_size);
        }

        if (ioctl(fd, RTCOMM_START)) {
                fprintf(stderr, "RTCOMM_START failed: %d\n", errno);

                return (1);
        } else {
                fprintf(stdout, "RTCOMM_START success.\n");
        }
        count = 0;

        for (;;) {
                int             ret;
                uint32_t        idx;
                uint32_t        to_idx;
                uint32_t        idxl;

                ret = read(fd, buffer, buffer_size);

                if (ret != buffer_size) {
                        fprintf(stderr, "Failed to read %d bytes, error: %d\n",
                                        buffer_size, ret);

                        return (-1);
                } 
                fprintf(stdout, " %05d: read %d bytes\n", count++, buffer_size);

                to_idx = buffer_size > max_size ? max_size : buffer_size;

                for (idx = 0; idx < to_idx; idx += 8) {
                        char            local_buffer[100];
                        uint32_t        str_idx;

                        sprintf(local_buffer, "\t %05d: ", idx);
                        str_idx = strlen(local_buffer);
                                        
                        for (idxl = 0; idxl < 8; idxl++) {
                                sprintf(&local_buffer[str_idx], "%02x ", 
                                                buffer[idxl + idx]);

                                str_idx = strlen(local_buffer);
                        }
                        puts(local_buffer);
                }
                
        }

        return (0);
}
