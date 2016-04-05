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
                if (g_log_level >= LOG_LEVEL_INF) {                             \
                        printk(KERN_INFO RTCOMM_NAME " info: " msg,             \
                                ## __VA_ARGS__);                                \
                }                                                               \
        } while (0)

#define RTCOMM_NOT(msg, ...)                                                    \
        do {                                                                    \
                if (g_log_level >= LOG_LEVEL_NOT) {                             \
                        printk(KERN_NOTICE  RTCOMM_NAME ": " msg,               \
                                ## __VA_ARGS__);                                \
                }                                                               \
        } while (0)

#define RTCOMM_WRN(msg, ...)                                                    \
        do {                                                                    \
                if (g_log_level >= LOG_LEVEL_WRN) {                             \
                        printk(KERN_WARNING RTCOMM_NAME " warning: " msg,       \
                                ## __VA_ARGS__);                                \
                }                                                               \
        } while (0)

#define RTCOMM_DBG(msg, ...)                                                    \
        do {                                                                    \
                if (g_log_level >= LOG_LEVEL_DBG) {                             \
                        printk(KERN_DEFAULT RTCOMM_NAME " debug: " msg,         \
                                ## __VA_ARGS__);                                \
                }                                                               \
        } while (0)


struct ppbuff;

struct rtcomm_state 
{
        struct spi_transfer     spi_transfer;
        struct spi_message      spi_message;
        struct spi_board_info   spi_board_info;
        struct spi_device *     spi;
        struct ppbuff *         ppbuff;
        char                    notify_label[64];
        bool                    is_spi_locked;
        bool                    is_isr_init;
        bool                    is_gpio_init;
        bool                    is_busy;
};

struct ppbuff
{
        wait_queue_head_t       wait;
        uint32_t                size;
        uint32_t                count;
        void *                  buffer;
};

static int rtcomm_open(struct inode * inode, struct file * fd);
static int rtcomm_release(struct inode * inode, struct file * fd);
static ssize_t rtcomm_read(struct file * fd, char __user *, size_t, loff_t *);
static struct ppbuff * ppbuff_init(uint32_t size);
static void ppbuff_term(struct ppbuff * ppbuff);
static void ppbuff_producer_done(struct ppbuff * ppbuff);
static void * ppbuff_get_consumer_storage(struct ppbuff * ppbuff);
static uint32_t ppbuff_size(struct ppbuff * ppbuff);

/*--  Module parameters  -----------------------------------------------------*/
static int g_bus_id = 1;
module_param(g_bus_id, int, S_IRUGO);
MODULE_PARM_DESC(g_bus_id, "SPI bus ID [1]");

static int g_notify = 55;
module_param(g_notify, int, S_IRUGO);
MODULE_PARM_DESC(g_notify, "notification GPIO pin [55]");

static int g_buff_size = 65536;
module_param(g_buff_size, int, S_IRUGO);
MODULE_PARM_DESC(g_buff_size, "buffer size [65536]");

static int g_log_level = LOG_LEVEL_NOT;
module_param(g_log_level, int, S_IRUGO);
MODULE_PARM_DESC(g_log_level, "log level [2]<4 - 0>");


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



/*--  PPBUF  -----------------------------------------------------------------*/

static struct ppbuff * ppbuff_init(uint32_t size)
{
        struct ppbuff * ppbuff;

        ppbuff = kmalloc(sizeof(struct ppbuff), GFP_KERNEL);
        RTCOMM_DBG("init PPBUFF: %p, size: %d\n", ppbuff, size);

        if (ppbuff == NULL) {
                RTCOMM_ERR("failed to create PPBUFF\n");

                return (NULL);
        }
        ppbuff->size  = size;
        ppbuff->count = 0;
        init_waitqueue_head(&ppbuff->wait);

        ppbuff->buffer = kmalloc(size, GFP_DMA | GFP_KERNEL);
        RTCOMM_DBG("storage PPBUFF: %p\n", ppbuff->buffer);

        if (ppbuff->buffer == NULL) {
                RTCOMM_ERR("failed to allocate PPBUFF storage\n");
                kfree(ppbuff);

                return (NULL);
        }
        
        return (ppbuff);
}



static void ppbuff_term(struct ppbuff * ppbuff)
{
        RTCOMM_DBG("term PPBUFF: %p\n", ppbuff);

        if (ppbuff) {
                kfree(ppbuff->buffer);
                kfree(ppbuff);
        }
}



static void ppbuff_producer_done(struct ppbuff * ppbuff)
{
        RTCOMM_DBG("done PPBUFF\n");

        ppbuff->count = 1;
        wake_up_interruptible(&ppbuff->wait);
}



static void ppbuff_consumer_done(struct ppbuff * ppbuff)
{
        ppbuff->count = 0;
}



static void * ppbuff_get_consumer_storage(struct ppbuff * ppbuff)
{
        if (wait_event_interruptible(ppbuff->wait, ppbuff->count != 0) == 0)
        {
                return (ppbuff->buffer);
        } 
        else 
        {
                /*
                 * In case we are interrupted return NULL pointer
                 */
                return (NULL);
        }
}



static uint32_t ppbuff_size(struct ppbuff * ppbuff)
{
        return (ppbuff->size);
}



static irqreturn_t trigger_notify_handler(int irq, void * p)
{
        disable_irq_nosync(irq);
        ppbuff_producer_done(g_rtcomm_state.ppbuff);
        enable_irq(irq);

        return (IRQ_HANDLED);
}



static int rtcomm_open(struct inode * inode, struct file * fd)
{
        int                     ret;
        
        RTCOMM_NOT("open(): %d:%d\n", current->group_leader->pid, current->pid);

        if (g_rtcomm_state.is_busy) {
                return (-EBUSY);
        }
        g_rtcomm_state.is_busy = true;
        fd->private_data = &g_rtcomm_state;
        
        g_rtcomm_state.spi->bits_per_word = 8;
        g_rtcomm_state.spi->mode          = SPI_MODE_1;
        g_rtcomm_state.spi->max_speed_hz  = 20000000ul;
        ret = spi_setup(g_rtcomm_state.spi);
            
        if (ret) {
                RTCOMM_ERR("spi_setup() request failed: %d\n", ret);
                
                return (ret);
        }
        spi_bus_lock(g_rtcomm_state.spi->master);
        g_rtcomm_state.is_spi_locked = true;
        g_rtcomm_state.ppbuff = ppbuff_init(g_buff_size);
        
        if (!g_rtcomm_state.ppbuff) {
                RTCOMM_ERR("ppbuff_init() init failed.\n");
                ret = -1;

                goto FAIL_CREATE_PPBUFF;
        }
        ret = gpio_to_irq(g_notify);
        
        if (ret < 0) {
                RTCOMM_ERR("NOTIFY gpio %d interrupt request mapping failed\n", g_notify);
                
                goto FAIL_GPIO_ISR_MAP_REQUEST;
        }
        ret = request_irq(
                        ret, 
                        &trigger_notify_handler, 
                        IRQF_TRIGGER_RISING, 
                        g_rtcomm_state.notify_label, 
                        NULL);
                        
        if (ret) {
                RTCOMM_ERR("NOTIFY gpio %d interrupt request failed: %d\n", g_notify, ret);
                
                goto FAIL_GPIO_ISR_REQUEST;
        }
        g_rtcomm_state.is_isr_init = true;

        return (0);
FAIL_GPIO_ISR_REQUEST:        
FAIL_GPIO_ISR_MAP_REQUEST:
        ppbuff_term(g_rtcomm_state.ppbuff);
FAIL_CREATE_PPBUFF:
        g_rtcomm_state.is_busy = false;
        spi_bus_unlock(g_rtcomm_state.spi->master);
        g_rtcomm_state.is_spi_locked = false;
        
        return (ret);
}



static int rtcomm_release(struct inode * inode, struct file * fd)
{
        RTCOMM_NOT("close(): %d:%d\n", current->group_leader->pid, 
                        current->pid);

        if (!g_rtcomm_state.is_busy) {
                return (-EINVAL);
        }
        disable_irq_nosync(gpio_to_irq(g_notify));
        free_irq(gpio_to_irq(g_notify), NULL);
        spi_bus_unlock(g_rtcomm_state.spi->master);
        ppbuff_term(g_rtcomm_state.ppbuff);
        g_rtcomm_state.is_spi_locked = false;
        g_rtcomm_state.is_isr_init   = false;
        g_rtcomm_state.is_busy       = false;

        return (0);
}



static ssize_t rtcomm_read(struct file * fd, char __user * buff, 
                size_t byte_count, loff_t * off)
{
        void *                  storage;

        /* NOTE:
         * Always set offset to zero. This driver does not utilize offset 
         * counter.
         */
        *off = 0;

        RTCOMM_DBG("read: requested size %d\n", byte_count);

        if (byte_count != ppbuff_size(g_rtcomm_state.ppbuff)) {
                RTCOMM_ERR("read(): invalid byte_count: %d, expected: %d\n", 
                        byte_count, ppbuff_size(g_rtcomm_state.ppbuff));
                
                return (-EINVAL);
        }
        RTCOMM_DBG("read: size %d\n", byte_count);
        storage = ppbuff_get_consumer_storage(g_rtcomm_state.ppbuff);
        
        if (!storage) {
                RTCOMM_ERR("read(): failed to get data storage\n");

                return (-ENOMEM);
        }
        memset(&g_rtcomm_state.spi_transfer, 0, sizeof(g_rtcomm_state.spi_transfer));
        g_rtcomm_state.spi_transfer.rx_buf = storage;
        g_rtcomm_state.spi_transfer.len    = byte_count;
        
        spi_message_init(&g_rtcomm_state.spi_message);
        spi_message_add_tail(&g_rtcomm_state.spi_transfer, &g_rtcomm_state.spi_message);
        spi_sync_locked(g_rtcomm_state.spi, &g_rtcomm_state.spi_message);
        RTCOMM_DBG("read copy: size: %d\n", byte_count);
        
        if (copy_to_user(buff, storage, byte_count)) {
            return (-EFAULT);
        }
        ppbuff_consumer_done(g_rtcomm_state.ppbuff);
        
        return (byte_count);
}



static int __init rtcomm_init(void)
{
        int                     ret;
        struct spi_master *     master;
        
        strncpy(&g_rtcomm_state.spi_board_info.modalias[0], RTCOMM_NAME, SPI_NAME_SIZE);
        g_rtcomm_state.spi_board_info.max_speed_hz = 40000000ul;
        g_rtcomm_state.spi_board_info.bus_num      = g_bus_id;
        
        RTCOMM_NOT("registering RTCOMM device driver " RTCOMM_BUILD_VER "\n");
        RTCOMM_NOT("BUILD: " RTCOMM_BUILD_DATE " : " RTCOMM_BUILD_TIME "\n");

        master = spi_busnum_to_master(g_bus_id);

        if (!master) {
                RTCOMM_ERR("invalid SPI bus id: %d\n", g_bus_id);

                return (-ENODEV);
        }
        g_rtcomm_state.spi = spi_new_device(master, &g_rtcomm_state.spi_board_info);

        if (!g_rtcomm_state.spi) {
                RTCOMM_ERR("could not create SPI device\n");

                return (-ENODEV);
        }
        sprintf(&g_rtcomm_state.notify_label[0], RTCOMM_NAME "-notify");
        RTCOMM_INF("gpio name: %s\n", g_rtcomm_state.notify_label);
        ret = gpio_request_one(g_notify, GPIOF_DIR_IN, g_rtcomm_state.notify_label);
        
        if (ret) {
                RTCOMM_ERR("NOTIFY gpio %d request failed\n", g_notify);

                goto FAIL_GPIO_REQUEST;
        }
        g_rtcomm_state.is_isr_init   = false;
        g_rtcomm_state.is_spi_locked = false;
        g_rtcomm_state.is_gpio_init  = true;
        g_rtcomm_state.is_busy       = false;
        
        ret = misc_register(&g_rtcomm_miscdev);
        
        return (ret);

FAIL_GPIO_REQUEST:
        spi_unregister_device(g_rtcomm_state.spi);

        return (ret);
}



static void __exit rtcomm_exit(void)
{
        RTCOMM_NOT("deregistering\n");

        if (g_rtcomm_state.is_isr_init) {
                disable_irq_nosync(gpio_to_irq(g_notify));
                free_irq(gpio_to_irq(g_notify), NULL);
        }
        
        if (g_rtcomm_state.is_gpio_init) {
                gpio_free(g_notify);
        }
        
        if (g_rtcomm_state.spi) {
        
                if (g_rtcomm_state.is_spi_locked) {
                        spi_bus_unlock(g_rtcomm_state.spi->master);
                }
                spi_unregister_device(g_rtcomm_state.spi);
        }
        misc_deregister(&g_rtcomm_miscdev);
}

module_init(rtcomm_init);
module_exit(rtcomm_exit);
MODULE_AUTHOR("Nenad Radulovic <nenad.b.radulovic@gmail.com>");
MODULE_DESCRIPTION("Real-time communication driver");
MODULE_LICENSE("GPL v2");


