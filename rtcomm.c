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

#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/moduleparam.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/fs.h>
#include <linux/completion.h>
#include <asm/uaccess.h>

#include "rtcomm.h"

#define RTCOMM_LOG_LEVEL                LOG_LEVEL_WRN
#define RTCOMM_BUILD_TIME               "12:00"
#define RTCOMM_BUILD_DATE               "2016-04-13"
#define RTCOMM_BUILD_VER                "v1.0"
#define RTCOMM_VERSION                  RTCOMM_BUILD_VER " - " RTCOMM_BUILD_DATE

#define LOG_LEVEL_ERR                   0
#define LOG_LEVEL_WRN                   1
#define LOG_LEVEL_NOT                   2
#define LOG_LEVEL_INF                   3
#define LOG_LEVEL_DBG                   4

#define RTCOMM_ERR(msg, ...)                                                    \
printk(KERN_ERR RTCOMM_NAME " error: " msg, ## __VA_ARGS__);    

#define RTCOMM_INF(msg, ...)                                                    \
        do {                                                                    \
                if (g_arg_log_level >= LOG_LEVEL_INF) {                         \
                        printk(KERN_INFO RTCOMM_NAME " info: " msg,             \
                                ## __VA_ARGS__);                                \
                }                                                               \
        } while (0)

#define RTCOMM_NOT(msg, ...)                                                    \
        do {                                                                    \
                if (g_arg_log_level >= LOG_LEVEL_NOT) {                         \
                        printk(KERN_NOTICE  RTCOMM_NAME ": " msg,               \
                                ## __VA_ARGS__);                                \
                }                                                               \
        } while (0)

#define RTCOMM_WRN(msg, ...)                                                    \
        do {                                                                    \
                if (g_arg_log_level >= LOG_LEVEL_WRN) {                         \
                        printk(KERN_WARNING RTCOMM_NAME " warning: " msg,       \
                                ## __VA_ARGS__);                                \
                }                                                               \
        } while (0)

#define RTCOMM_DBG(msg, ...)                                                    \
        do {                                                                    \
                if (g_arg_log_level >= LOG_LEVEL_DBG) {                         \
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
};



struct rtcomm_state 
{
        struct spi_board_info   spi_board_info;
        struct spi_device *     spi;
        struct fifo_buff *      fifo_buff;
        struct task_struct *    thd_rtcomm_fifo;
        struct completion       isr_signal;
        struct rtcomm_config    config;
        pid_t                   thd_rtcomm_fifo_pid;
        char                    notify_label[64];
        bool                    is_busy;
        bool                    is_initialized;
        bool                    is_running;
        bool                    should_exit;

        struct rtcomm_perf
        {
                uint32_t                skip_read_data_buff;
        }                       perf;              
};



struct fifo_buff
{
        struct completion       put;
        uint32_t                size;
        bool                    is_reading;
        /* NOTE:
         * Buffer pointers are kept for allocation / deallocation of data
         * buffers in case when FIFO is interrupted during pointer swapping in
         * which case consumer and producer pointers are invalid.
         */
        void *                  buffer_a;
        void *                  buffer_b;
        void *                  consumer;
        void *                  producer;
};

static int rtcomm_open(struct inode * inode, struct file * fd);
static int rtcomm_release(struct inode * inode, struct file * fd);
static ssize_t rtcomm_read(struct file * fd, char __user *, size_t, loff_t *);
static long rtcomm_ioctl(struct file *, unsigned int, unsigned long);

static irqreturn_t trigger_notify_handler(int irq, void * p);

static struct fifo_buff * fifo_buff_init(uint32_t size);
static void fifo_buff_term(struct fifo_buff * fifo_buff);
static void * fifo_buff_create(struct fifo_buff * fifo_buff);
static void fifo_buff_delete(struct fifo_buff * fifo_buff, void * storage);
static void fifo_buff_recycle(struct fifo_buff * fifo_buff, void * storage);
static void * fifo_buff_get(struct fifo_buff * fifo_buff);
static int fifo_buff_put(struct fifo_buff * fifo_buff, void * storage);
static uint32_t fifo_buff_size(struct fifo_buff * fifo_buff);

static int rtcomm_fifo(void * data);

/*--  Module parameters  -----------------------------------------------------*/
static int g_arg_bus_id = -1;
module_param(g_arg_bus_id, int, S_IRUGO);
MODULE_PARM_DESC(g_arg_bus_id, "SPI bus ID");

static int g_arg_notify_pin_id = -1;
module_param(g_arg_notify_pin_id, int, S_IRUGO);
MODULE_PARM_DESC(g_arg_notify_pin_id, "notification GPIO pin ID");

static int g_arg_buffer_size_bytes = -1;
module_param(g_arg_buffer_size_bytes, int, S_IRUGO);
MODULE_PARM_DESC(g_arg_buffer_size_bytes, "buffer size in bytes");

static int g_arg_log_level = -1;
module_param(g_arg_log_level, int, S_IRUGO);
MODULE_PARM_DESC(g_arg_log_level, "log level [0 - 4]");



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
        g_pending_config.notify_pin_id          = g_arg_notify_pin_id;
        g_pending_config.spi_bus_id             = g_arg_bus_id;
        g_pending_config.spi_bus_speed          = 10000000ul;
        g_pending_config.buffer_size_bytes      = g_arg_buffer_size_bytes;
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
                return (-EBUSY);
        }
        /* 
         * Get SPI bus ID
         */
        RTCOMM_DBG("get SPI bus ID");
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
        RTCOMM_DBG("setup SPI device");
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
        RTCOMM_DBG("get GPIO pin");
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
        RTCOMM_DBG("setup FIFO buffer");
        state->fifo_buff = fifo_buff_init(config->buffer_size_bytes);
        
        if (!state->fifo_buff) {
                RTCOMM_ERR("fifo_buff_init() init failed.\n");
                ret = -ENOMEM;

                goto FAIL_CREATE_PPBUFF;
        }
        
        /*
         * Check GPIO IRQ
         */
        RTCOMM_DBG("check GPIO IRQ");
        ret = gpio_to_irq(config->notify_pin_id);
        
        if (ret < 0) {
                RTCOMM_ERR("NOTIFY gpio %d interrupt request mapping failed\n", 
                        config->notify_pin_id);
                ret = -ENODEV;
                
                goto FAIL_GPIO_ISR_MAP_REQUEST;
        }
        
        /*
         * Create consumer thread
         */
        RTCOMM_DBG("create consumer thread");
        state->thd_rtcomm_fifo = kthread_create(rtcomm_fifo, state, 
                        "rtcomm_fifo");
                        
        if (IS_ERR(state->thd_rtcomm_fifo)) {
                RTCOMM_ERR("can't create thd_rtcomm_fifo");
                ret = -ENOMEM;
                
                goto FAIL_CREATE_THREAD;
        }
        init_completion(&state->isr_signal);
        state->config = *config;
        state->is_initialized = true;
        
        RTCOMM_DBG("initialization complete");

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
        if (state->is_initialized) {
                state->is_initialized = false;        
                fifo_buff_term(state->fifo_buff);
                gpio_free(state->config.notify_pin_id);
                spi_unregister_device(state->spi);
        }
        
        return (0);
}



static int start_sampling(struct rtcomm_state * state)
{
        int                     ret;
        
        RTCOMM_DBG("start_sampling()");
        
        memset(&state->perf, 0, sizeof(state->perf));
        state->should_exit = false;
        spi_bus_lock(state->spi->master);
        init_completion(&state->isr_signal);
        wake_up_process(state->thd_rtcomm_fifo);
        
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
        spi_bus_unlock(state->spi->master); 
        
        return (ret);
}



static int stop_sampling(struct rtcomm_state * state)
{
        if (state->is_running) {
                state->should_exit = true;
                RTCOMM_DBG("stop sampling()\n");
                disable_irq_nosync(gpio_to_irq(state->config.notify_pin_id));
                free_irq(gpio_to_irq(state->config.notify_pin_id), NULL);
                complete(&state->isr_signal);
                kthread_stop(state->thd_rtcomm_fifo);
                spi_bus_unlock(state->spi->master);
                state->is_running = false;
        }
        
        return (0);
}

/*--  FIFO consumer thread  --------------------------------------------------*/

#

static int rtcomm_fifo(void * data)
{
        struct rtcomm_state *   state = data;
        struct sched_param      sched_param;
        int                     retval;
        static struct spi_message      message;
        static struct spi_transfer     transfer;
        
        RTCOMM_NOT("rtcomm_fifo(): %d:%d\n", current->group_leader->pid, 
                        current->pid);
        state->thd_rtcomm_fifo_pid = current->pid;
       
        memset(&sched_param, 0, sizeof(sched_param));
        sched_param.sched_priority = MAX_RT_PRIO - 1;
        retval = sched_setscheduler(current, SCHED_FIFO, &sched_param);
        
        if (retval) {
                RTCOMM_WRN("rtcomm_fifo(): couldn't set scheduler policy: %d\n",
                                retval);
        }
        
        for (;;) {
                void *          storage;

                wait_for_completion(&state->isr_signal);
                RTCOMM_DBG("rtcomm_fifo(): got signal\n");

                if (state->should_exit) {
                        RTCOMM_NOT("rtcomm_fifo(): exiting\n");
                        do_exit(0);
                }
                storage = fifo_buff_create(state->fifo_buff);
                memset(&transfer, 0, sizeof(transfer));
                transfer.rx_buf = storage;
                transfer.len    = fifo_buff_size(state->fifo_buff);
                spi_message_init(&message);
                spi_message_add_tail(&transfer, &message);
                spi_sync_locked(state->spi, &message);

                if (fifo_buff_put(state->fifo_buff, storage)) {
                        state->perf.skip_read_data_buff++;
                        RTCOMM_ERR("data read buffer skipped: %d time(s)\n",
                                        state->perf.skip_read_data_buff);
                }
        }
        
        return (0);
}

/*--  PPBUF  -----------------------------------------------------------------*/



/* NOTE:
 * FIFO buffers are currently organized as ping-pong buffers which implies that
 * only two buffers are used. If a need should arise then we will make true
 * FIFO buffer.
 */
static struct fifo_buff * fifo_buff_init(uint32_t size)
{
        struct fifo_buff * fifo_buff;

        fifo_buff = kmalloc(sizeof(struct fifo_buff), GFP_KERNEL);
        RTCOMM_DBG("init fifo_buff: %p, size: %d\n", fifo_buff, size);

        if (!fifo_buff) {
                RTCOMM_ERR("failed to create fifo_buff\n");

                return (NULL);
        }
        init_completion(&fifo_buff->put);
        fifo_buff->size         = size;
        fifo_buff->buffer_a     = kmalloc(size, GFP_DMA | GFP_KERNEL);
        RTCOMM_DBG("storage fifo_buff A: %p\n", fifo_buff->buffer_a);

        if (!fifo_buff->buffer_a) {
                RTCOMM_ERR("failed to allocate fifo_buff A storage\n");
                kfree(fifo_buff);

                return (NULL);
        }
        fifo_buff->buffer_b     = kmalloc(size, GFP_DMA | GFP_KERNEL);
        RTCOMM_DBG("storage fifo_buff B: %p\n", fifo_buff->buffer_b);

        if (!fifo_buff->buffer_b) {
                RTCOMM_ERR("failed to allocate fifo_buff B storage\n");
                kfree(fifo_buff->buffer_a);
                kfree(fifo_buff);

                return (NULL);
        }
        fifo_buff->consumer = fifo_buff->buffer_a;
        fifo_buff->producer = fifo_buff->buffer_b;
        fifo_buff->is_reading = false;
        
        return (fifo_buff);
}



static void fifo_buff_term(struct fifo_buff * fifo_buff)
{
        kfree(fifo_buff->buffer_a);
        kfree(fifo_buff->buffer_b);
        kfree(fifo_buff);
}



static void * fifo_buff_create(struct fifo_buff * fifo_buff)
{
        return (fifo_buff->producer);
}



static void fifo_buff_delete(struct fifo_buff * fifo_buff, void * storage)
{
        fifo_buff_recycle(fifo_buff, storage);
        fifo_buff->is_reading = false;
}



static void fifo_buff_recycle(struct fifo_buff * fifo_buff, void * storage)
{
        /* NOTE:
         * Reinitialize the completion. This function is called at the end of
         * read file operation. By doing this call at the end we disregard all
         * completions that might happen during read operations.
         */
        reinit_completion(&fifo_buff->put);
}



static void * fifo_buff_get(struct fifo_buff * fifo_buff)
{
        if (wait_for_completion_interruptible(&fifo_buff->put) != 0) {
                return (NULL);
        }
        
        return (fifo_buff->consumer);
}



static int fifo_buff_put(struct fifo_buff * fifo_buff, void * storage)
{
        int                     retval = 1;

        if (!fifo_buff->is_reading) {
                void *          tmp;

                tmp = fifo_buff->producer;
                fifo_buff->producer = fifo_buff->consumer;
                fifo_buff->consumer = tmp;

                retval = 0;
        }
        fifo_buff->is_reading = true;
        complete(&fifo_buff->put);

        return (retval);
}



static uint32_t fifo_buff_size(struct fifo_buff * fifo_buff)
{
        return (fifo_buff->size);
}


/*--  Notify handler  --------------------------------------------------------*/



static irqreturn_t trigger_notify_handler(int irq, void * p)
{
        struct rtcomm_state *   state = &g_state;

        complete(&state->isr_signal);

        return (IRQ_HANDLED);
}


/*--  FOPS  ------------------------------------------------------------------*/



static ssize_t rtcomm_read(struct file * fd, char __user * buff, 
                size_t byte_count, loff_t * off)
{
        struct rtcomm_state *   state = state_from_fd(fd);
        void *                  storage;
        ssize_t                 retval;

        /* NOTE:
         * Always set offset to zero. This driver does not utilize offset 
         * counter.
         */
        *off = 0;

        if (!state->is_running) {
                return (-ENODEV);
        }
        storage = fifo_buff_get(state->fifo_buff);
        
        if (!storage) {
                RTCOMM_NOT("read(): aborted\n");
                retval = -ENOMEM;

                goto FAIL_GET_STORAGE;
        }
        
        if (copy_to_user(buff, storage, byte_count)) {
                RTCOMM_ERR("read(): failed to copy data to user-space\n");
                retval = -EFAULT;

                goto FAIL_COPY_TO_USER;
        }
        fifo_buff_delete(state->fifo_buff, storage);
        
        return (byte_count);
FAIL_COPY_TO_USER:
        fifo_buff_recycle(state->fifo_buff, storage);
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

        RTCOMM_NOT("ioctl(): %d:%d\n", current->group_leader->pid, current->pid);
        
        retval = 0;

        switch (cmd) {
                case RTCOMM_GET_VERSION: {
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
                case RTCOMM_START: {
                        retval = init_sampling(state, &g_pending_config);
                        
                        if (retval) {
                                break;
                        }
                        retval = start_sampling(state);
                        
                        if (retval) {
                                break;
                        }
                        break;
                }
                case RTCOMM_STOP: {
                        retval = stop_sampling(state);

                        if (retval) {
                                break;
                        }
                        retval = term_sampling(state);

                        if (retval) {
                                break;
                        }
                        break;
                }
                case RTCOMM_GET_FIFO_PID: {
                        signed long long pid;

                        if (!state->is_running) {
                                retval = - EINVAL;
                                break;
                        }
                        pid = state->thd_rtcomm_fifo_pid;

                        retval = copy_to_user(&pid, (void __user *)arg, 
                                        sizeof(pid));

                        if (retval) {
                                retval = -EINVAL;
                                break;
                        }
                        break;                        
                }
                default : {
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



static void __exit rtcomm_exit(void)
{
        RTCOMM_NOT("deregistering\n");

        misc_deregister(&g_rtcomm_miscdev);
}

module_init(rtcomm_init);
module_exit(rtcomm_exit);
MODULE_AUTHOR("Nenad Radulovic <nenad.b.radulovic@gmail.com>");
MODULE_DESCRIPTION("Real-time communication driver");
MODULE_LICENSE("GPL v2");


