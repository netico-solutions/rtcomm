#include <stdio.h>
#include <stdint.h>


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

int main(void)
{
        char                    buffer[sizeof(struct ipc_acquisition_frame)];

        fprintf(stdout, "%d\n", sizeof(buffer));

        return (0);
}
