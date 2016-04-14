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

#define IPC_API_INPUTS_QUANTITY                                 14u
#define IPC_API_OUTPUTS_QUANTITY                                7u
#define IPC_API_ANALOG_INPUTS_QUANTITY                  8u
#define IPC_API_PWM_QUANTITY                                    2u
#define IPC_API_INPUTS_SAMPLING_FREQUENCY_Hz    100u
#define IPC_API_ANALOG_SAMPLING_FREQUENCY_Hz    20000u
#define IPC_API_MASTER_READ_FRAME_RATE_Hz               10u

#define IPC_API_IO_SAMPLES_PER_FRAME                                            \
            (IPC_API_INPUTS_SAMPLING_FREQUENCY_Hz / IPC_API_MASTER_READ_FRAME_RATE_Hz)

#define IPC_API_ANALOG_SAMPLES_PER_FRAME                                                                                \
                (IPC_API_ANALOG_SAMPLING_FREQUENCY_Hz / IPC_API_MASTER_READ_FRAME_RATE_Hz)
#define IPC_IO_ATTR_                            __attribute__((packed))

struct IPC_IO_ATTR_ ipc_rtc_time
{
        uint16_t                                                        year;
        uint8_t                                                         month;
        uint8_t                                                         day;
        uint8_t                                                         hour;
        uint8_t                                                         minute;
        uint8_t                                                         second;
};

struct IPC_IO_ATTR_ ipc_timestamp
{
                struct ipc_rtc_time                                     rtc_time;
                        uint16_t                                                        ms_time;
};


struct IPC_IO_ATTR_ ipc_io_status
{
        uint8_t                                                         inputs_status[IPC_API_INPUTS_QUANTITY];
        uint8_t                                                         outputs_status[IPC_API_OUTPUTS_QUANTITY];
};

struct IPC_IO_ATTR_ ipc_acquisition_frame
{
        uint32_t                frame_counter;
        struct ipc_timestamp    timestamp;
        uint8_t                 timestamp_validity;
        uint16_t                analog_data[IPC_API_ANALOG_SAMPLES_PER_FRAME][IPC_API_ANALOG_INPUTS_QUANTITY];
        uint8_t                 analog_data_validity;
        struct ipc_io_status    io_status[IPC_API_IO_SAMPLES_PER_FRAME];
        uint8_t                 io_status_validity;
};


int main(int argc, char * argv[])
{
        int                     fd;
        int                     buffer_size = sizeof(struct ipc_acquisition_frame);
        int                     count;
        char                    version[20];
        void *                  buffer;
        
        fprintf(stdout, "RTCOMM drv test 1, ver:" __DATE__ " : " __TIME__ "\n");

        fprintf(stdout, "Using buffer size %d bytes\n", buffer_size);

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
                fprintf(stderr, "RTCOMM_GET_VERSION get: %s\n", version);
        }

        if (ioctl(fd, RTCOMM_SET_SIZE, &buffer_size) == -1) {
                fprintf(stderr, "RTCOMM_SET_SIZE failed: %d\n", errno);

                return (1);
        } else {
                fprintf(stderr, "RTCOMM_SET_SIZE set: %d\n", buffer_size);
        }

        if (ioctl(fd, RTCOMM_START)) {
                fprintf(stderr, "RTCOMM_START failed: %d\n", errno);

                return (1);
        } else {
                fprintf(stderr, "RTCOMM_START success.\n");
        }
        count = 0;

        for (;;) {
                int             ret;
                uint32_t        idx;
                uint32_t        to_idx;
                uint32_t        idxl;
                struct ipc_acquisition_frame * frame = buffer;

                ret = read(fd, buffer, buffer_size);

                if (ret != buffer_size) {
                        fprintf(stderr, "Failed to read %d bytes, error: %d\n",
                                        buffer_size, ret);

                        return (-1);
                } 
                fprintf(stderr, "Got %d frame: %04u-%02u-%02u - %02u:%02u:%02u.%04u\n", 
                                frame->frame_counter,
                                frame->timestamp.rtc_time.year,
                                frame->timestamp.rtc_time.month,
                                frame->timestamp.rtc_time.day,
                                frame->timestamp.rtc_time.hour,
                                frame->timestamp.rtc_time.minute,
                                frame->timestamp.ms_time);

                for (idx = 0; idx < IPC_API_ANALOG_SAMPLES_PER_FRAME; idx++) {
                        char            local_buffer[200];
                        uint32_t        str_idx;

                        sprintf(local_buffer, "%u, %d, ", frame->frame_counter, idx);
                        str_idx = strlen(local_buffer);
                                        
                        for (idxl = 0; idxl < IPC_API_ANALOG_INPUTS_QUANTITY; idxl++) {
                                sprintf(&local_buffer[str_idx], "%d, ", 
                                                (int16_t)frame->analog_data[idx][idxl]);

                                str_idx = strlen(local_buffer);
                        }
                        puts(local_buffer);
                }
                
        }

        return (0);
}
