
#ifndef NETICO_RTCOMM_IOCTL_
#define NETICO_RTCOMM_IOCTL_


#define RTCOMM_NAME                     "rtcomm"

#define RTCOMM_IOC_MAGIC                'r'

#define RTCOMM_GET_VERSION              _IOW(RTCOMM_IOC_MAGIC, 200, char [20])

#define RTCOMM_SET_SIZE                 _IOR(RTCOMM_IOC_MAGIC, 100, int)

#define RTCOMM_GET_FIFO_PID             _IOW(RTCOMM_IOC_MAGIC, 103, signed long long)

#define RTCOMM_INIT                     _IO(RTCOMM_IOC_MAGIC, 1)

#define RTCOMM_START                    _IO(RTCOMM_IOC_MAGIC, 2)

#define RTCOMM_STOP                     _IO(RTCOMM_IOC_MAGIC, 3)

#define RTCOMM_TERM                     _IO(RTCOMM_IOC_MAGIC, 4)

#endif /* NETICO_RTCOMM_IOCTL_ */
