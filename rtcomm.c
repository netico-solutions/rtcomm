/*
 * Real-time communication driver
 *
 * Author: Nenad Radulovic <nenad.b.radulovic@gmail.com>
 *
 * Licensed under GPL-v2
 * ----------------------------------------------------------------------------
 *
 * Real-time communication driver
 *
 * This driver manages all communication between Linux application and real-time
 * processor.
 */

#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/semaphore.h>
#include <linux/completion.h>
#include <linux/miscdevice.h>
#include <linux/moduleparam.h>

#include "rtcomm.h"

#define RTCOMM_LOG_LEVEL                LOG_LEVEL_WRN
#define RTCOMM_BUILD_TIME               "12:00"
#define RTCOMM_BUILD_DATE               "2017-06-10"
#define RTCOMM_BUILD_VER                "v1.3"
#define RTCOMM_VERSION                  RTCOMM_BUILD_VER " - " RTCOMM_BUILD_DATE

#define LOG_LEVEL_ERR                   0
#define LOG_LEVEL_WRN                   1
#define LOG_LEVEL_NOT                   2
#define LOG_LEVEL_INF                   3
#define LOG_LEVEL_DBG                   4

#define WARN_SKIP_READ_RTFW_BUFF        0
#define WARN_SKIP_READ_FIFO_BUFF        1
#define WARN_SKIP_READ_RTFW_BUFF_MASK        (0x1u << WARN_SKIP_READ_RTFW_BUFF)
#define WARN_SKIP_READ_FIFO_BUFF_MASK        (0x1u << WARN_SKIP_READ_FIFO_BUFF)


#define RTCOMM_ERR(msg, ...)                                                    \
printk(KERN_ERR RTCOMM_NAME " error: " msg, ## __VA_ARGS__);    

#define RTCOMM_INF(msg, ...)                                                    \
        do {                                                                    \
                if (loglevel >= LOG_LEVEL_INF) {                                \
                        printk(KERN_INFO RTCOMM_NAME " info: " msg,             \
                                ## __VA_ARGS__);                                \
                }                                                               \
        } while (0)

#define RTCOMM_NOT(msg, ...)                                                    \
        do {                                                                    \
                if (loglevel >= LOG_LEVEL_NOT) {                                \
                        printk(KERN_NOTICE  RTCOMM_NAME ": " msg,               \
                                ## __VA_ARGS__);                                \
                }                                                               \
        } while (0)

#define RTCOMM_WRN(msg, ...)                                                    \
        do {                                                                    \
                if (loglevel >= LOG_LEVEL_WRN) {                                \
                        printk(KERN_WARNING RTCOMM_NAME " warning: " msg,       \
                                ## __VA_ARGS__);                                \
                }                                                               \
        } while (0)

#define RTCOMM_DBG(msg, ...)                                                    \
        do {                                                                    \
                if (loglevel >= LOG_LEVEL_DBG) {                                \
                        printk(KERN_DEFAULT RTCOMM_NAME " debug: " msg,         \
                                ## __VA_ARGS__);                                \
                }                                                               \
        } while (0)

struct fifo_buff;

struct rtcomm_config
{
        int                     notify_pin_id;
        int                     spi_bus_id;
        int                     spi_bus_speed;
        int                     buffer_size_bytes;
        int                     fifo_buffers;
};

struct rtcomm_state 
{
        struct spi_board_info   spi_board_info;
        struct spi_device *     spi;
        struct fifo_buff *      fifo_buff;
        struct task_struct *    thd_rtcomm_fifo;
        struct completion       isr_signal;
        struct completion       exit_signal;
        struct rtcomm_config    config;
        pid_t                   thd_rtcomm_fifo_pid;
        char                    notify_label[64];
        bool                    is_busy;
        bool                    is_initialized;
        bool                    is_running;
        bool                    is_read_pending;
        volatile bool           should_exit;

        struct rtcomm_perf
        {
                volatile long unsigned int skip_read_rtfw_buff;
                volatile long unsigned int skip_read_fifo_buff;
                volatile long unsigned int warned;
        }                       perf;              
};



struct fifo_buff
{
        void **                 storage;
        uint32_t                head;
        uint32_t                tail;
        uint32_t                free;
        uint32_t                size;
        uint32_t                package_size;
        struct mutex            mutex;
        struct semaphore        items;
        struct semaphore        spaces;
};

static int      rtcomm_open(struct inode * inode, struct file * fd);
static int      rtcomm_release(struct inode * inode, struct file * fd);
static ssize_t  rtcomm_read(struct file * fd, char __user *, size_t, loff_t *);
static long     rtcomm_ioctl(struct file *, unsigned int, unsigned long);

static irqreturn_t 
                trigger_notify_handler(int irq, void * p);

static struct fifo_buff * 
                fifo_buff_init(uint32_t size, uint32_t package_size);
static void     fifo_buff_term(struct fifo_buff * fifo_buff);
static void *   fifo_buff_create(struct fifo_buff * fifo_buff);
static void     fifo_buff_delete(struct fifo_buff * fifo_buff, void * storage);
static void *   fifo_buff_get(struct fifo_buff * fifo_buff);
static int      fifo_buff_put(struct fifo_buff * fifo_buff, void * storage);
static uint32_t fifo_buff_package_size(struct fifo_buff * fifo_buff);

static int      rtcomm_fifo(void * data);

/*--  Module parameters  -----------------------------------------------------*/
static int busid = -1;
module_param(busid, int, S_IRUGO);
MODULE_PARM_DESC(busid, "SPI bus ID");

static int busspeed = 1000000;
module_param(busspeed, int, S_IRUGO);
MODULE_PARM_DESC(busspeed, "SPI bus speed [Hz]");

static int notifyid = -1;
module_param(notifyid, int, S_IRUGO);
MODULE_PARM_DESC(notifyid, "notification GPIO pin ID");

static int loglevel = 4;
module_param(loglevel, int, S_IRUGO);
MODULE_PARM_DESC(loglevel, "log level [0 - 4]");

static int fifosize = 30;
module_param(fifosize, int, S_IRUGO);
MODULE_PARM_DESC(fifosize, "number of buffers in FIFO");

static const struct file_operations g_rtcomm_fops = 
{
        .owner          = THIS_MODULE,
        .open           = rtcomm_open,
        .release        = rtcomm_release,
        .read           = rtcomm_read,
        .unlocked_ioctl = rtcomm_ioctl
};

static struct miscdevice        g_rtcomm_miscdev = 
{
        MISC_DYNAMIC_MINOR, 
        RTCOMM_NAME,
        &g_rtcomm_fops
};

static struct rtcomm_state      g_state;
static struct rtcomm_config     g_pending_config;

/*--  Driver configuration  --------------------------------------------------*/

static void config_init_pending(void)
{
        g_pending_config.notify_pin_id          = notifyid;
        g_pending_config.spi_bus_id             = busid;
        g_pending_config.spi_bus_speed          = busspeed;
        g_pending_config.buffer_size_bytes      = 0;
        g_pending_config.fifo_buffers           = fifosize;
}

/*--  FIFO_BUFF  -------------------------------------------------------------*/

static struct fifo_buff * fifo_buff_init(uint32_t size, uint32_t package_size)
{
        struct fifo_buff * fifo_buff;

        fifo_buff = kmalloc(sizeof(struct fifo_buff), GFP_KERNEL);
        RTCOMM_DBG("init fifo_buff: %p, size: %d of %d bytes\n", fifo_buff, 
                size, package_size);

        if (!fifo_buff) {
                RTCOMM_ERR("failed to create fifo_buff\n");
                goto ERR_MALLOC_FIFO_BUFF;
        }
        fifo_buff->storage = kmalloc(sizeof(void *) * size, GFP_KERNEL);
        
        if (!fifo_buff->storage) {
                RTCOMM_ERR("failed to create fifo_buff pointer storage\n");
                goto ERR_MALLOC_STORAGE;
        }
        fifo_buff->head = 0;
        fifo_buff->tail = 0;
        fifo_buff->free = size;
        fifo_buff->size = size;
        fifo_buff->package_size = package_size;
        mutex_init(&fifo_buff->mutex);
        sema_init(&fifo_buff->items, 0);
        sema_init(&fifo_buff->spaces, size);
        
        return (fifo_buff);
ERR_MALLOC_STORAGE:
        kfree(fifo_buff);
ERR_MALLOC_FIFO_BUFF:
        return (NULL);
}

static void fifo_buff_term(struct fifo_buff * fifo_buff)
{
        kfree(fifo_buff->storage);
        kfree(fifo_buff);
}

static void * fifo_buff_create(struct fifo_buff * fifo_buff)
{
        return (kmalloc(fifo_buff->package_size, GFP_KERNEL));
}

static void fifo_buff_delete(struct fifo_buff * fifo_buff, void * storage)
{
        (void)fifo_buff;
        
        kfree(storage);
}

static void * fifo_buff_get(struct fifo_buff * fifo_buff)
{
        int status;
        void * item;

        status = down_interruptible(&fifo_buff->items);
        
        if (status == -EINTR) {
                RTCOMM_DBG("fifo_buff: get(): interrupted\n");
                goto ERR_EINTR_ITEMS;
        }
        status = mutex_lock_interruptible(&fifo_buff->mutex);
        
        if (status == -EINTR) {
                RTCOMM_DBG("fifo_buff: get(): interrupted\n");
                goto ERR_EINTR_MUTEX;
        }
        /* --- Get item --- */
        item = fifo_buff->storage[fifo_buff->tail++];
        
        if (fifo_buff->tail == fifo_buff->size) {
                fifo_buff->tail = 0i;
        }
        fifo_buff->free++;
        
        mutex_unlock(&fifo_buff->mutex);
        up(&fifo_buff->spaces);
        
        return (item);
ERR_EINTR_MUTEX:
ERR_EINTR_ITEMS:
        return (NULL);
}

static int fifo_buff_put(struct fifo_buff * fifo_buff, void * storage)
{
        int status;
        
        status = down_trylock(&fifo_buff->spaces);
        
        if (status != 0) {
                status = -ENOMEM;
                goto ERR_EINTR_SPACES;
        }
        status = mutex_lock_interruptible(&fifo_buff->mutex);
        
        if (status == -EINTR) {
                RTCOMM_DBG("fifo_buff: put(): interrupted\n");
                status = -EINTR;
                goto ERR_EINTR_MUTEX;
        }
        /* --- Put item --- */
        fifo_buff->storage[fifo_buff->head++] = storage;

        if (fifo_buff->head == fifo_buff->size) {
                fifo_buff->head = 0u;
        }
        fifo_buff->free--;
        
        mutex_unlock(&fifo_buff->mutex);
        up(&fifo_buff->items);
        
        return (0);
ERR_EINTR_MUTEX:
ERR_EINTR_SPACES:
        kfree(storage);
        
        return (status);
}

static uint32_t fifo_buff_package_size(struct fifo_buff * fifo_buff)
{
        return (fifo_buff->package_size);
}

/*--  Misc  ------------------------------------------------------------------*/

static struct rtcomm_state * state_from_fd(struct file * fd)
{
        return (fd->private_data);
}

static void state_to_fd(struct file * fd, struct rtcomm_state * state)
{
        fd->private_data = state;
}

static int init_sampling(struct rtcomm_state * state, 
                const struct rtcomm_config * config)
{
        int                     ret;
        struct spi_master *     master;
       
        if (state->is_initialized) {
                return (0);
        }
        RTCOMM_DBG("init_sampling()\n");
        
        state->config = *config;

        /* 
         * Get SPI bus ID
         */
        RTCOMM_DBG("get SPI bus ID\n");
        strncpy(&state->spi_board_info.modalias[0], RTCOMM_NAME, SPI_NAME_SIZE);
        state->spi_board_info.max_speed_hz = config->spi_bus_speed;
        state->spi_board_info.bus_num      = config->spi_bus_id;
        master = spi_busnum_to_master(config->spi_bus_id);

        if (!master) {
                RTCOMM_ERR("invalid SPI bus id: %d\n", config->spi_bus_id);
                ret = -ENODEV;

                goto FAIL_MASTER;
        }

        /*
         * Setup SPI device
         */
        RTCOMM_DBG("setup SPI device\n");
        state->spi = spi_new_device(master, &state->spi_board_info);

        if (!state->spi) {
                RTCOMM_ERR("could not create SPI device\n");
                ret = -ENODEV;

                goto FAIL_SPI_NEW;
        }
        state->spi->bits_per_word = 8;
        state->spi->mode          = SPI_MODE_1;
        state->spi->max_speed_hz  = config->spi_bus_speed;
        ret = spi_setup(state->spi);
            
        if (ret) {
                RTCOMM_ERR("could not setup SPI device: %d\n", ret);
                ret = -ENODEV;
                
                goto FAIL_SPI_SETUP;
        }

        /*
         * Get GPIO pin
         */
        RTCOMM_DBG("get GPIO pin\n");
        sprintf(&state->notify_label[0], RTCOMM_NAME "-notify");
        RTCOMM_INF("gpio name: %s\n", state->notify_label);
        ret = gpio_request_one(config->notify_pin_id, GPIOF_DIR_IN, 
                state->notify_label);
        
        if (ret) {
                RTCOMM_ERR("NOTIFY gpio %d request failed\n", 
                        config->notify_pin_id);
                ret = -ENODEV;
                
                goto FAIL_GPIO_REQUEST;
        }

        /*
         * Setup FIFO buffer
         */
        RTCOMM_DBG("setup FIFO buffer\n");
        state->fifo_buff = fifo_buff_init(config->fifo_buffers, 
                                          config->buffer_size_bytes + 
                                          sizeof(struct rtcomm_packet_header));
        
        if (!state->fifo_buff) {
                RTCOMM_ERR("fifo_buff_init() init failed.\n");
                ret = -ENOMEM;

                goto FAIL_CREATE_PPBUFF;
        }
        
        /*
         * Check GPIO IRQ
         */
        RTCOMM_DBG("check GPIO IRQ\n");
        ret = gpio_to_irq(config->notify_pin_id);
        
        if (ret < 0) {
                RTCOMM_ERR("NOTIFY gpio %d interrupt request mapping failed\n", 
                        config->notify_pin_id);
                ret = -ENODEV;
                
                goto FAIL_GPIO_ISR_MAP_REQUEST;
        }
        
        /*
         * Create producer thread
         */
        RTCOMM_DBG("create producer thread\n");
        state->should_exit = false;
        init_completion(&state->isr_signal);
        init_completion(&state->exit_signal);
        state->thd_rtcomm_fifo = kthread_create(rtcomm_fifo, state, 
                        "rtcomm_fifo");
                        
        if (IS_ERR(state->thd_rtcomm_fifo)) {
                RTCOMM_ERR("can't create thd_rtcomm_fifo\n");
                ret = -ENOMEM;
                
                goto FAIL_CREATE_THREAD;
        }
        state->is_initialized = true;
        wake_up_process(state->thd_rtcomm_fifo);
        
        return (0);
        
FAIL_CREATE_THREAD:
FAIL_GPIO_ISR_MAP_REQUEST:
        fifo_buff_term(state->fifo_buff);
FAIL_CREATE_PPBUFF:
        gpio_free(config->notify_pin_id);
FAIL_GPIO_REQUEST:
        spi_unregister_device(state->spi);
FAIL_SPI_SETUP:
FAIL_SPI_NEW:
FAIL_MASTER:

        return (ret);
}

static int term_sampling(struct rtcomm_state * state) 
{
        if (!state->is_initialized) {
                return (0);
        }
        RTCOMM_DBG("term_sampling()");
        state->is_initialized = false;        
        state->should_exit    = true;
        complete(&state->isr_signal);
        kthread_stop(state->thd_rtcomm_fifo);
        wait_for_completion(&state->exit_signal);
        fifo_buff_term(state->fifo_buff);
        gpio_free(state->config.notify_pin_id);
        spi_unregister_device(state->spi);
        RTCOMM_DBG("term_sampling() done");
        
        return (0);
}

static int start_sampling(struct rtcomm_state * state)
{
        int                     ret;
       
        if (!state->is_initialized) {
                RTCOMM_ERR("start_sampling(): sampling was not initialized");

                return (-EFAULT);
        }

        if (state->is_running) {
                RTCOMM_ERR("start_sampling(): sampling is busy");

                return (-EBUSY);
        }
        state->is_read_pending = false;
        RTCOMM_DBG("start_sampling()");
        memset(&state->perf, 0, sizeof(state->perf));
        
        ret = request_irq(gpio_to_irq(state->config.notify_pin_id), 
                        &trigger_notify_handler, IRQF_TRIGGER_RISING, 
                        state->notify_label, NULL);
                        
        if (ret) {
                RTCOMM_ERR("NOTIFY gpio %d interrupt request failed: %d\n", 
                        state->config.notify_pin_id, ret);
                ret = -ENODEV;
                
                goto FAIL_GPIO_ISR_REQUEST;
        }
        state->is_running = true;

        return (0);
        
FAIL_GPIO_ISR_REQUEST:
        
        return (ret);
}

static int stop_sampling(struct rtcomm_state * state)
{
        if (!state->is_running) {
                return (0);
        }
        RTCOMM_DBG("stop_sampling()");
        state->is_running = false;
        disable_irq(gpio_to_irq(state->config.notify_pin_id));
        free_irq(gpio_to_irq(state->config.notify_pin_id), NULL);
        
        return (0);
}

/*--  FIFO producer thread  --------------------------------------------------*/

static int rtcomm_fifo(void * data)
{
        struct rtcomm_state *   state = data;
        struct sched_param      sched_param;
        int                     retval;
        static struct spi_message message;
        static struct spi_transfer transfer;
        struct timeval          timeval;
        struct rtcomm_packet *  packet;
        
        RTCOMM_NOT("rtcomm_fifo(): %d:%d\n", current->group_leader->pid, 
                        current->pid);
        state->thd_rtcomm_fifo_pid = current->pid;
       
        memset(&sched_param, 0, sizeof(sched_param));
        sched_param.sched_priority = MAX_RT_PRIO - 1;
        retval = sched_setscheduler(current, SCHED_FIFO, &sched_param);
        RTCOMM_DBG("rtcomm_fifo(): setting max priority\n");
        
        if (retval) {
                RTCOMM_WRN("rtcomm_fifo(): couldn't set scheduler policy: %d\n",
                                retval);
        }
        spi_bus_lock(state->spi->master);
        
        for (;;) {
                clear_bit(WARN_SKIP_READ_RTFW_BUFF, &state->perf.warned);
                state->is_read_pending = false;
                wait_for_completion(&state->isr_signal);
                state->is_read_pending = true;

                if (state->should_exit) {
                        break;                        
                }
                do_gettimeofday(&timeval);
                packet = fifo_buff_create(state->fifo_buff);
                packet->header.tv_sec = timeval.tv_sec;
                packet->header.tv_msec = timeval.tv_usec / 1000;
                memset(&transfer, 0, sizeof(transfer));
                transfer.rx_buf = &packet->data[0];
                transfer.len    = state->config.buffer_size_bytes;
                spi_message_init(&message);
                spi_message_add_tail(&transfer, &message);
                spi_sync_locked(state->spi, &message);
                
                retval = fifo_buff_put(state->fifo_buff, packet);
                
                switch (retval) {
                        case 0: {
                                if (state->perf.warned & 
                                        WARN_SKIP_READ_FIFO_BUFF_MASK) {
                                        RTCOMM_WRN("reading of FIFO resumed, "
                                                "skipped %lu time(s)\n",
                                                state->perf.skip_read_fifo_buff);
                                }
                                state->perf.warned &= 
                                        ~WARN_SKIP_READ_FIFO_BUFF_MASK;
                                break;
                        }
                        case -ENOMEM: {
                                state->perf.skip_read_fifo_buff++;
                                
                                if (!(state->perf.warned &
                                        WARN_SKIP_READ_FIFO_BUFF_MASK)) {
                                        RTCOMM_WRN("reading of FIFO skip\n");
                                }
                                state->perf.warned |= 
                                        WARN_SKIP_READ_FIFO_BUFF_MASK;
                                break;
                        }
                        case -EINTR: {
                                break;
                        }
                        default: {
                                RTCOMM_WRN(
                                    "rtcomm_fifo(): unknown error in FIFO\n");
                                break;
                        }
                }
        }
        spi_bus_unlock(state->spi->master);
        complete(&state->exit_signal);
        RTCOMM_NOT("rtcomm_fifo(): exiting\n");
        do_exit(0);
        
        return (0);
}

/*--  Notify handler  --------------------------------------------------------*/

static irqreturn_t trigger_notify_handler(int irq, void * p)
{
        struct rtcomm_state *   state = &g_state;

        if (state->is_read_pending) {
                state->perf.skip_read_rtfw_buff++;
                
                if (!(state->perf.warned & WARN_SKIP_READ_RTFW_BUFF_MASK)) {
                        RTCOMM_ERR("reading of RT firmware skipped: %lu time(s)\n",
                                state->perf.skip_read_rtfw_buff);
                }
                state->perf.warned |= WARN_SKIP_READ_RTFW_BUFF_MASK;
        }
        complete(&state->isr_signal);

        return (IRQ_HANDLED);
}


/*--  fops  ------------------------------------------------------------------*/

static ssize_t rtcomm_read(struct file * fd, char __user * buff, 
                size_t byte_count, loff_t * off)
{
        struct rtcomm_state *   state = state_from_fd(fd);
        void *                  packet;
        ssize_t                 retval;

        /* NOTE:
         * Always set offset to zero. This driver does not utilize offset 
         * counter.
         */
        *off = 0;

        if (!state->is_initialized) {
                return (-ENODEV);
        }
        if (byte_count != fifo_buff_package_size(state->fifo_buff)) {
                return (-EINVAL);
        }
        packet = fifo_buff_get(state->fifo_buff);
        
        if (!packet) {
                RTCOMM_NOT("read(): aborted\n");
                retval = -ENOMEM;

                goto FAIL_GET_STORAGE;
        }
        
        if (copy_to_user(buff, packet, byte_count)) {
                RTCOMM_ERR("read(): failed to copy data to user-space\n");
                retval = -EFAULT;

                goto FAIL_COPY_TO_USER;
        }
        fifo_buff_delete(state->fifo_buff, packet);
        
        return (byte_count);
FAIL_COPY_TO_USER:
        fifo_buff_delete(state->fifo_buff, packet);
FAIL_GET_STORAGE:

        return (retval);
}

static int rtcomm_open(struct inode * inode, struct file * fd)
{
        struct rtcomm_state *   state = &g_state;
        
        RTCOMM_NOT("open(): %d:%d\n", current->group_leader->pid, current->pid);
        
        state_to_fd(fd, state);

        if (state->is_busy) {
                return (-EBUSY);
        }
        state->is_busy = true;

        return (0);
}

static int rtcomm_release(struct inode * inode, struct file * fd)
{
        struct rtcomm_state *   state = state_from_fd(fd);

        RTCOMM_NOT("close(): %d:%d\n", current->group_leader->pid, 
                        current->pid);

        stop_sampling(state);
        term_sampling(state);
        state->is_busy = false;

        return (0);
}

static long rtcomm_ioctl(struct file * fd, unsigned int cmd, unsigned long arg)
{
        long                    retval;
        struct rtcomm_state *   state  = state_from_fd(fd);
        int                     cgpid;
        int                     cpid;

        cgpid = current->group_leader->pid;
        cpid = current->pid;
        
        retval = 0;

        switch (cmd) {
                case RTCOMM_GET_VERSION: {
                        RTCOMM_NOT("ioctl(): %d:%d RTCOMM_GET_VERSION\n", cgpid, 
                                        cpid);
                        retval = copy_to_user((void __user *)arg, RTCOMM_VERSION, 
                                sizeof(RTCOMM_VERSION));

                        if (retval) {
                                retval = -EINVAL;
                                break;
                        }
                        break;
                }
                case RTCOMM_SET_SIZE: {
                        int     bytes;

                        RTCOMM_NOT("ioctl(): %d:%d RTCOMM_SET_SIZE\n", cgpid, 
                                        cpid);
                        retval = copy_from_user(&bytes, (void __user *)arg,
                                        sizeof(bytes));

                        if (retval) {
                                retval = -EINVAL;
                                break;
                        }
                        
                        if (bytes <= 0) {
                                retval = -EINVAL;
                                break;
                        }
                        g_pending_config.buffer_size_bytes = bytes;
                        break;
                }
                case RTCOMM_INIT: {
                        RTCOMM_NOT("ioctl(): %d:%d RTCOMM_INIT\n", cgpid, cpid);

                        retval = stop_sampling(state);

                        if (retval) {
                                break;
                        }
                        retval = term_sampling(state);

                        if (retval) {
                                break;
                        }
                        retval = init_sampling(state, &g_pending_config);

                        break;                                
                }
                case RTCOMM_START: {
                        RTCOMM_NOT("ioctl(): %d:%d RTCOMM_START\n", cgpid, cpid);
                        retval = start_sampling(state);

                        break;
                }
                case RTCOMM_STOP: {
                        RTCOMM_NOT("ioctl(): %d:%d RTCOMM_STOP\n", cgpid, cpid);
                        retval = stop_sampling(state);

                        break;
                }
                case RTCOMM_TERM: {
                        RTCOMM_NOT("ioctl(): %d:%d RTCOMM_TERM\n", cgpid, cpid);
                        retval = stop_sampling(state);

                        if (retval) {
                                break;
                        }
                        retval = term_sampling(state);

                        break;
                }
                case RTCOMM_GET_FIFO_PID: {
                        signed long long pid;

                        RTCOMM_NOT("ioctl(): %d:%d RTCOMM_GET_FIFO_PID\n", cgpid, 
                                        cpid);

                        if (!state->is_initialized) {
                                retval = - ENODEV;
                                break;
                        }
                        pid = state->thd_rtcomm_fifo_pid;

                        retval = copy_to_user((void __user *)arg, &pid,
                                        sizeof(pid));

                        if (retval) {
                                retval = -EINVAL;
                                break;
                        }
                        break;                        
                }
                default : {
                        RTCOMM_ERR("ioctl(): %d:%d unknown IOCTL call.\n", cgpid,
                                        cpid);

                        retval = -EINVAL;
                        break;
                }     
        }
                        
        return (retval);
}

/*--  Module initialization  -------------------------------------------------*/

static int __init rtcomm_init(void)
{
        int                     ret;
        
        RTCOMM_NOT("registering RTCOMM device driver " RTCOMM_BUILD_VER "\n");
        RTCOMM_NOT("BUILD: " RTCOMM_BUILD_DATE " : " RTCOMM_BUILD_TIME "\n");
        config_init_pending();
        
        ret = misc_register(&g_rtcomm_miscdev);
        
        if (ret) {
                RTCOMM_ERR("Failed to register misc device: %d\n", ret);

                goto FAIL_REGISTER_MISC;
        }
        
        return (ret);
FAIL_REGISTER_MISC:

        return (ret);
}

/*--  Module termination -----------------------------------------------------*/

static void __exit rtcomm_exit(void)
{
        RTCOMM_NOT("deregistering\n");

        misc_deregister(&g_rtcomm_miscdev);
}

/*--  Module setup -----------------------------------------------------------*/

module_init(rtcomm_init);
module_exit(rtcomm_exit);
MODULE_AUTHOR("Nenad Radulovic <nenad.b.radulovic@gmail.com>");
MODULE_DESCRIPTION("Real-time communication driver");
MODULE_LICENSE("GPL v2");


