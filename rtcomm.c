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
#include <asm/uaccess.h>

#define RTCOMM_NAME                     "rtcomm"
#define RTCOMM_LOG_LEVEL                LOG_LEVEL_WRN
#define RTCOMM_BUILD_TIME               "12:00"
#define RTCOMM_BUILD_DATE               "2016-04-05"
#define RTCOMM_BUILD_VER                "v1.0"

#define LOG_LEVEL_ERR                   0
#define LOG_LEVEL_WRN                   1
#define LOG_LEVEL_NOT                   2
#define LOG_LEVEL_INF                   3
#define LOG_LEVEL_DBG                   4

#define RTCOMM_ERR(msg, ...)                                                    \
        printk(KERN_ERR RTCOMM_NAME " error: " msg, ## __VA_ARGS__);    

#define RTCOMM_INF(msg, ...)                                                    \
        do {                                                                    \
                if (g_rtcomm_config.log_level >= LOG_LEVEL_INF) {               \
                        printk(KERN_INFO RTCOMM_NAME " info: " msg,             \
                                ## __VA_ARGS__);                                \
                }                                                               \
        } while (0)

#define RTCOMM_NOT(msg, ...)                                                    \
        do {                                                                    \
                if (g_rtcomm_config.log_level >= LOG_LEVEL_NOT) {               \
                        printk(KERN_NOTICE  RTCOMM_NAME ": " msg,               \
                                ## __VA_ARGS__);                                \
                }                                                               \
        } while (0)

#define RTCOMM_WRN(msg, ...)                                                    \
        do {                                                                    \
                if (g_rtcomm_config.log_level >= LOG_LEVEL_WRN) {               \
                        printk(KERN_WARNING RTCOMM_NAME " warning: " msg,       \
                                ## __VA_ARGS__);                                \
                }                                                               \
        } while (0)

#define RTCOMM_DBG(msg, ...)                                                    \
        do {                                                                    \
                if (g_rtcomm_config.log_level >= LOG_LEVEL_DBG) {               \
                        printk(KERN_DEFAULT RTCOMM_NAME " debug: " msg,         \
                                ## __VA_ARGS__);                                \
                }                                                               \
        } while (0)


struct fifo_buff;

struct rtcomm_state 
{
        struct spi_board_info   spi_board_info;
        struct spi_device *     spi;
        struct fifo_buff *      fifo_buff;
        struct task_struct *    fifo_consumer;
        wait_queue_head_t *     isr_wait;
        bool                    isr_has_signaled;
        char                    notify_label[64];
        bool                    is_spi_locked;
        bool                    is_isr_init;
        bool                    is_gpio_init;
        bool                    is_busy;
};

struct rtcomm_config
{
        int                     log_level;
        int                     notify_pin_id;
        int                     spi_bus_id;
        int                     spi_bus_speed;
        int                     buffer_size_bytes;
};

struct fifo_buff
{
        wait_queue_head_t       wait;
        uint32_t                size;
        uint32_t                count;
        void *                  buffer;
};

static int rtcomm_open(struct inode * inode, struct file * fd);
static int rtcomm_release(struct inode * inode, struct file * fd);
static ssize_t rtcomm_read(struct file * fd, char __user *, size_t, loff_t *);
static struct fifo_buff * fifo_buff_init(uint32_t size);
static void fifo_buff_term(struct fifo_buff * fifo_buff);
static void * fifo_buff_create(struct fifo_buff * fifo_buff);
static void fifo_buff_delete(struct fifo_buff * fifo_buff, void * storage);
static void * fifo_buff_get(struct fifo_buff * fifo_buff);
static uint32_t fifo_buff_size(struct fifo_buff * fifo_buff);



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
};

static struct miscdevice        g_rtcomm_miscdev = 
{
        MISC_DYNAMIC_MINOR, 
        RTCOMM_NAME,
        &g_rtcomm_fops
};

static struct rtcomm_state      g_rtcomm_state;
static struct rtcomm_config     g_rtcomm_config;


/*--  Driver configuration  --------------------------------------------------*/



static void init_configuration(struct rtcomm_config * config)
{
        config->log_level = g_arg_log_level;
        config->notify_pin_id = g_arg_notify_pin_id;
        config->spi_bus_id = g_arg_bus_id;
        config->spi_bus_speed = 20000000ul;
        config->buffer_size_bytes = g_arg_buffer_size_bytes;
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



/*--  FIFO consumer thread  --------------------------------------------------*/

static int thread_fifo_consumer(void * data)
{
        struct rtcomm_state *   state = data;
        struct spi_message      message;
        struct spi_transfer     transfer;

        while (!kthread_should_stop()) {
                void *          storage;

                if (wait_event_interruptible(&state->isr_wait, state->isr_has_signaled) != 0) {
                        continue;
                }
                storage = fifo_buff_create(state->fifo_buff);
                memset(&transfer, 0, sizeof(transfer));
                transfer.rx_buf = storage;
                transfer.len    = fifo_buff_size(state->fifo_buff);
                spi_message_init(&message);
                spi_message_add_tail(&transfer, &message);
                spi_sync_locked(state->spi, &message);
                fifo_buff_put(state->fifo_buff, storage);
        }
        return (0);
}

/*--  PPBUF  -----------------------------------------------------------------*/

static struct fifo_buff * fifo_buff_init(uint32_t size)
{
        struct fifo_buff * fifo_buff;

        fifo_buff = kmalloc(sizeof(struct fifo_buff), GFP_KERNEL);
        RTCOMM_DBG("init fifo_buff: %p, size: %d\n", fifo_buff, size);

        if (fifo_buff == NULL) {
                RTCOMM_ERR("failed to create fifo_buff\n");

                return (NULL);
        }
        fifo_buff->size  = size;
        fifo_buff->count = 0;

        fifo_buff->buffer = kmalloc(size, GFP_DMA | GFP_KERNEL);
        RTCOMM_DBG("storage fifo_buff: %p\n", fifo_buff->buffer);

        if (fifo_buff->buffer == NULL) {
                RTCOMM_ERR("failed to allocate fifo_buff storage\n");
                kfree(fifo_buff);

                return (NULL);
        }
        
        return (fifo_buff);
}



static void fifo_buff_term(struct fifo_buff * fifo_buff)
{
        RTCOMM_DBG("term fifo_buff: %p\n", fifo_buff);

        if (fifo_buff) {
                kfree(fifo_buff->buffer);
                kfree(fifo_buff);
        }
}



static void ppbuff_producer_done(struct fifo_buff * fifo_buff)
{
        RTCOMM_DBG("done fifo_buff\n");

        fifo_buff->count = 1;
        wake_up_interruptible(&fifo_buff->wait);
}



static void fifo_buff_delete(struct fifo_buff * fifo_buff)
{
        fifo_buff->count = 0;
}



static void * fifo_buff_get(struct fifo_buff * fifo_buff)
{
        if (wait_event_interruptible(fifo_buff->wait, fifo_buff->count != 0) == 0)
        {
                return (fifo_buff->buffer);
        } 
        else 
        {
                /*
                 * In case we are interrupted return NULL pointer
                 */
                return (NULL);
        }
}



static uint32_t fifo_buff_size(struct fifo_buff * fifo_buff)
{
        return (fifo_buff->size);
}



static irqreturn_t trigger_notify_handler(int irq, void * p)
{
        struct rtcomm_state *   state = &g_rtcomm_state;

        disable_irq_nosync(irq);
        ppbuff_producer_done(state->fifo_buff);
        enable_irq(irq);

        return (IRQ_HANDLED);
}



static int rtcomm_open(struct inode * inode, struct file * fd)
{
        struct rtcomm_state *   state = state_from_fd(fd);
        struct rtcomm_config *  config = &g_rtcomm_config;
        int                     ret;
        
        RTCOMM_NOT("open(): %d:%d\n", current->group_leader->pid, current->pid);
        
        state_to_fd(fd, &g_rtcomm_state);

        if (state->is_busy) {
                return (-EBUSY);
        }
        state->is_busy = true;
        
        state->spi->bits_per_word = 8;
        state->spi->mode          = SPI_MODE_1;
        state->spi->max_speed_hz  = config->spi_bus_speed;
        ret = spi_setup(state->spi);
            
        if (ret) {
                RTCOMM_ERR("spi_setup() request failed: %d\n", ret);
                
                return (ret);
        }
        spi_bus_lock(state->spi->master);
        state->is_spi_locked = true;
        state->fifo_buff = fifo_buff_init(config->buffer_size_bytes);
        
        if (!state->fifo_buff) {
                RTCOMM_ERR("fifo_buff_init() init failed.\n");
                ret = -1;

                goto FAIL_CREATE_PPBUFF;
        }
        ret = gpio_to_irq(config->notify_pin_id);
        
        if (ret < 0) {
                RTCOMM_ERR("NOTIFY gpio %d interrupt request mapping failed\n", 
                        config->notify_pin_id);
                
                goto FAIL_GPIO_ISR_MAP_REQUEST;
        }
        init_waitqueue_head(&state->isr_wait);
        ret = request_irq(
                        ret, 
                        &trigger_notify_handler, 
                        IRQF_TRIGGER_RISING, 
                        state->notify_label, 
                        NULL);
                        
        if (ret) {
                RTCOMM_ERR("NOTIFY gpio %d interrupt request failed: %d\n", 
                        config->notify_pin_id, ret);
                
                goto FAIL_GPIO_ISR_REQUEST;
        }
        state->is_isr_init = true;

        return (0);
FAIL_GPIO_ISR_REQUEST:        
FAIL_GPIO_ISR_MAP_REQUEST:
        fifo_buff_term(state->fifo_buff);
FAIL_CREATE_PPBUFF:
        state->is_busy = false;
        spi_bus_unlock(state->spi->master);
        state->is_spi_locked = false;
        
        return (ret);
}



static int rtcomm_release(struct inode * inode, struct file * fd)
{
        struct rtcomm_state *   state = state_from_fd(fd);
        struct rtcomm_config *  config = &g_rtcomm_config;

        RTCOMM_NOT("close(): %d:%d\n", current->group_leader->pid, 
                        current->pid);

        if (!state->is_busy) {
                return (-EINVAL);
        }
        disable_irq_nosync(gpio_to_irq(config->notify_pin_id));
        free_irq(gpio_to_irq(config->notify_pin_id), NULL);
        spi_bus_unlock(state->spi->master);
        fifo_buff_term(state->fifo_buff);
        state->is_spi_locked = false;
        state->is_isr_init   = false;
        state->is_busy       = false;

        return (0);
}



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

        RTCOMM_DBG("read: requested size %d\n", byte_count);

        if (byte_count != fifo_buff_size(state->fifo_buff)) {
                RTCOMM_ERR("read(): invalid byte_count: %d, expected: %d\n", 
                        byte_count, fifo_buff_size(state->fifo_buff));
                retval = -EINVAL;

                goto FAIL_INVALID_BYTE_COUNT;
        }
        storage = fifo_buff_get(state->fifo_buff);
        
        if (!storage) {
                RTCOMM_ERR("read(): failed to get data storage\n");
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
        fifo_buff_delete(state->fifo_buff, storage);
FAIL_GET_STORAGE:
FAIL_INVALID_BYTE_COUNT:

        return (retval);
}



static int __init rtcomm_init(void)
{
        struct rtcomm_state *   state = &g_rtcomm_state;
        struct rtcomm_config *  config = &g_rtcomm_config;
        int                     ret;
        struct spi_master *     master;
        
        
        RTCOMM_NOT("registering RTCOMM device driver " RTCOMM_BUILD_VER "\n");
        RTCOMM_NOT("BUILD: " RTCOMM_BUILD_DATE " : " RTCOMM_BUILD_TIME "\n");
        init_configuration(&g_rtcomm_config);

        strncpy(&state->spi_board_info.modalias[0], RTCOMM_NAME, SPI_NAME_SIZE);
        state->spi_board_info.max_speed_hz = config->spi_bus_speed;
        state->spi_board_info.bus_num      = config->spi_bus_id;
        master = spi_busnum_to_master(config->spi_bus_id);

        if (!master) {
                RTCOMM_ERR("invalid SPI bus id: %d\n", config->spi_bus_id);

                return (-ENODEV);
        }
        state->spi = spi_new_device(master, &state->spi_board_info);

        if (!state->spi) {
                RTCOMM_ERR("could not create SPI device\n");

                return (-ENODEV);
        }
        sprintf(&state->notify_label[0], RTCOMM_NAME "-notify");
        RTCOMM_INF("gpio name: %s\n", state->notify_label);
        ret = gpio_request_one(config->notify_pin_id, GPIOF_DIR_IN, 
                state->notify_label);
        
        if (ret) {
                RTCOMM_ERR("NOTIFY gpio %d request failed\n", 
                        config->notify_pin_id);

                goto FAIL_GPIO_REQUEST;
        }
        state->is_isr_init   = false;
        state->is_spi_locked = false;
        state->is_gpio_init  = true;
        state->is_busy       = false;
        
        ret = misc_register(&g_rtcomm_miscdev);
        
        return (ret);

FAIL_GPIO_REQUEST:
        spi_unregister_device(state->spi);

        return (ret);
}



static void __exit rtcomm_exit(void)
{
        struct rtcomm_state *   state = &g_rtcomm_state;
        struct rtcomm_config *  config = &g_rtcomm_config;

        RTCOMM_NOT("deregistering\n");

        if (state->is_isr_init) {
                disable_irq_nosync(gpio_to_irq(config->notify_pin_id));
                free_irq(gpio_to_irq(config->notify_pin_id), NULL);
        }
        
        if (state->is_gpio_init) {
                gpio_free(config->notify_pin_id);
        }
        
        if (state->spi) {
        
                if (state->is_spi_locked) {
                        spi_bus_unlock(state->spi->master);
                }
                spi_unregister_device(state->spi);
        }
        misc_deregister(&g_rtcomm_miscdev);
}

module_init(rtcomm_init);
module_exit(rtcomm_exit);
MODULE_AUTHOR("Nenad Radulovic <nenad.b.radulovic@gmail.com>");
MODULE_DESCRIPTION("Real-time communication driver");
MODULE_LICENSE("GPL v2");


