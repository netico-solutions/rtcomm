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

#define ARRAY_SIZE(array)    (sizeof(array)/sizeof(array[0]))

struct tests
{
    int                  (* validator)(const void *, size_t);
    const char *            text;
};

static int
validate_test_0(const void * data, size_t size);

static struct tests tests[] = {
    {
        .validator = validate_test_0,
        .text = "all elements of array are 1...n",
    },
};

static int
validate_test_0(const void * data, size_t size)
{
    uint32_t                    idx;
    const uint8_t *            _data = (const uint8_t *)data;

    for (idx = 0; idx < size; idx++) {

        if (_data[idx] != (uint8_t)(idx & 0xf)) {
            printf("data not valid at byte %d\n", idx);
            
            return (idx);
        }
    }

    return (-1);
}


static void
print_tests(void)
{
        uint32_t                idx;

        for (idx = 0; idx < ARRAY_SIZE(tests); idx++) {
                fprintf(stdout, "test %d: %s\n", idx, tests[idx].text);
        }
}


int main(int argc, char * argv[])
{
        int                     fd;
        int                     buffer_size = 64;
        int                     max_size = 64;
        int                     count;
        char                    version[20];
        char *                  buffer;
        int                     use_test;
        
        fprintf(stdout, "RTCOMM drv test 2, ver:" __DATE__ " : " __TIME__ "\n");

        use_test = -1;

        switch (argc) {
                case 4: 
                        if (argv[3][0] == 't') {
                            use_test = atoi(&argv[3][1]);

                            if (use_test < 0) {
                                use_test = ARRAY_SIZE(tests);
                            }
                        }
                case 3: max_size = atoi(argv[2]);
                case 2:
                        buffer_size = atoi(argv[1]);
                        break;
                case 1:
                        break;
                default:
                        fprintf(stderr, "Usage rtcomm_test [buffer_size] [max_print_size] t[n]\n");

                        return (1);
        }
        fprintf(stdout, "Using buffer size %d bytes\n", buffer_size);
        fprintf(stdout, "Using max print size %d bytes\n", max_size);

        if (use_test >= (int)ARRAY_SIZE(tests)) {
            fprintf(stderr, "Test index is out of range, valid is [0-%d]\n",
                (int)ARRAY_SIZE(tests) - 1);
            print_tests();

            return (1);
        }

        if (use_test != -1)
            fprintf(stdout, "Validating data using test %d\n", use_test);

        buffer = malloc(buffer_size);

        if (!buffer) {
                fprintf(stderr, "Couldn't allocate buffer of size %d\n", 
                                buffer_size);

                return (1);
        }

        sleep(1);

        fd = open("/dev/" RTCOMM_NAME, O_RDONLY);

        fprintf(stderr, "Open driver: %d\n", errno);

        if (!fd) {
                fprintf(stderr, "Failed to open driver: %d\n", errno);

                return (1);
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
        
        if (ioctl(fd, RTCOMM_INIT)) {
                fprintf(stderr, "RTCOMM_INIT failed: %d\n", errno);

                return (1);
        } else {
                fprintf(stdout, "RTCOMM_INIT success.\n");
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
                fprintf(stdout, "pass %05d: read %d bytes\n", count++, 
                        buffer_size);

                to_idx = buffer_size > max_size ? max_size : buffer_size;

                for (idx = 0; idx < to_idx; idx += 8) {
                        int             data_failed;
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

                        if (use_test != -1) {
                            data_failed = tests[use_test].validator(buffer, 
                                    buffer_size);

                            if (data_failed != -1) {
                                return (1);
                            }
                        }
                }
        }

        return (0);
}
