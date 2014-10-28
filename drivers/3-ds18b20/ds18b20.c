

#include <linux/module.h>  
#include <linux/kernel.h>
#include <linux/init.h> 
#include <linux/fs.h>   
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/slab.h>             /* kmalloc/kfree */
#include <asm/uaccess.h>
#include <asm/io.h>  
#include <asm/irq.h>

MODULE_LICENSE("GPL");

#define RT5350_DS18B20_MAJOR                  0
#define RT5350_DS18B20_NAME                   "rt5350_ds18b20"

#define RT5350_SYSCTRL_BASE_ADDR            0x10000000
#define RT5350_SYSCTRL_GPIOMODE_OFS         0x0060
#define RT5350_GPIO_BASE_ADDR               0x10000600
#define RT5350_GPIO27_22_DATA_OFS           0x0070
#define RT5350_GPIO27_22_DIR_OFS            0x0074

#define GPIO27_22_DS18B20_BIT               4

#define INPUT_DIR                           0
#define OUTPUT_DIR                          1
#define DATA_LOW_LEVEL                      0
#define DATA_HIGH_LEVEL                     1

#define GPIOMODE_EPHY_BT_GPIO_MODE          14
#define EPHY_BT_GPIOMODE_BIT                0x1         // Referenced by RT5350_20101018_un.pdf p.29 [GPIOMODE: GPIO Purpose Select (offset: 0x0060)]

static int rt5350_ds18b20_major = RT5350_DS18B20_MAJOR;

struct rt5350_ds18b20_dev {
    const char *ds18b20_name;
    struct cdev cdev;
    struct class *ds18b20_cls;
    struct device *ds18b20_dev;
};

static struct rt5350_ds18b20_dev *rt5350_ds18b20_devp;

static volatile unsigned long *gpio_mode;
static volatile unsigned long *gpio27_22_dir;
static volatile unsigned long *gpio27_22_data;

#define DS18B20_LOW                         (*gpio27_22_data &= ~(1<<GPIO27_22_DS18B20_BIT))
#define DS18B20_HIGH                        (*gpio27_22_data |= (1<<GPIO27_22_DS18B20_BIT))
#define DS18B20_IN                          (*gpio27_22_dir &= ~(1<<GPIO27_22_DS18B20_BIT))
#define DS18B20_OUT                         (*gpio27_22_dir |= (1<<GPIO27_22_DS18B20_BIT))
#define DS18B20_STATUS                      ((*gpio27_22_data>>4) & 0x1)

/* Setup the rt5350 gpio's mode */
static inline void rt5350_gpio_mode_setup(int gpiobit, int mode)
{
    *gpio_mode |= (gpiobit<<mode);
}

#if 0
/* 
 * Setup the rt5350 gpio27_22's direction
 * params:
 *      level  = 0:    input
 *      level  = 1:    output
 *      regbit = control bit 
 */
static inline void rt5350_gpio27_22_dir_setup(int level, int regbit)
{
    if (level)
        *gpio27_22_dir |= (1<<regbit);
    else
        *gpio27_22_dir &= ~(1<<regbit);
}

static inline void rt5350_gpio27_22_data_level(int level, int regbit)
{
    if (level)
        *gpio27_22_data |= (1<<regbit);
    else
        *gpio27_22_data &= ~(1<<regbit);
}
#endif

/* ################################## ds18b20 ops ################################## */

/* Moter configs and operations */
static inline void rt5350_ds18b20_gpio_mode_setup(void)
{
    /* 
     *  GPIO#26  = Related to ds18b20
     */
    rt5350_gpio_mode_setup(EPHY_BT_GPIOMODE_BIT, GPIOMODE_EPHY_BT_GPIO_MODE);
}

#if 0
static void rt5350_ds18b20_gpio_dir_setup(void)
{
    /* Setup the ds18b20's GPIO#7,8,9,10,17,18 to output */
    rt5350_gpio27_22_dir_setup(OUTPUT_DIR, GPIO21_00_MOTOR_OUT1_GPIO_7_BIT);
}

static void rt5350_ds18b20_gpio_data_level(int level, int regbit)
{
    rt5350_gpio27_22_data_level(level, regbit);
}
#endif

/* ################################## ds18b20 ops ################################## */

static void writeBit(unsigned char cmdBit)
{    
    DS18B20_OUT;
    DS18B20_HIGH;
    DS18B20_LOW;
    udelay(10);
    
    if (cmdBit) {
        DS18B20_HIGH;
    }else {
        DS18B20_LOW;
    }

    udelay(60);         // hold the level 60 us
    DS18B20_HIGH;
}

static unsigned char readBit(void)
{
    unsigned char tmp;

    /* host side */    
    DS18B20_OUT;                // set the bus direction to OUTPUT
    DS18B20_HIGH;               // the initial level is HIGH
    DS18B20_LOW;                // set to LOW in order to tell the host is gonna operate the ds18b20
    udelay(5);
    DS18B20_HIGH;               // release the bus so that the ds18b20 can get the bus to do something
    udelay(10);

    /* ds18b20 side */
    DS18B20_IN;                 // set to INPUT in order to let the ds18b20 to read the status & data bit
    tmp = DS18B20_STATUS;       // ds18b20 read the status
    udelay(60);

    DS18B20_OUT;                // set to OUTPUT in order to let the ds18b20 to output the status    
    DS18B20_HIGH;               // release the bus
    
    return tmp;
}

static unsigned char readByte(void)
{
    char ch;
    unsigned char tmp = 0;

    for (ch = 0; ch < 8; ch++) {
        tmp >>= 1;          // Read LSB
        if (readBit())
            tmp |= 0x80;
    }

    return tmp;
}

/* 
 *  Reset progress
 *  Return 0 if reset successfully, otherwise failure
 */
static unsigned char reset_ds18b20(void)
{
    unsigned char ret = 0;

    /* host side */
    DS18B20_OUT;                // set the bus direction to OUTPUT
    DS18B20_HIGH;               // the initial level is HIGH
    DS18B20_LOW;                // set to LOW in order to tell the host is gonna operate the ds18b20
    udelay(600);
    DS18B20_HIGH;               // release the bus so that the ds18b20 can get the bus to do something
    udelay(80);

    /* ds18b20 side */
    DS18B20_IN;                 // set to INPUT in order to let the ds18b20 to read the status
    ret = DS18B20_STATUS;       // ds18b20 read the status
    DS18B20_OUT;                // set to OUTPUT in order to let the ds18b20 to output the status
    udelay(300);
    DS18B20_HIGH;               // release the bus

    return ret;
}

static void send_ds18b20_cmd(unsigned char cmd)
{
    char ch = 0;

    while (ch++ < 8) {
        writeBit(cmd & 0x1);
        cmd >>= 1;
    }
}

static unsigned int readTemp(void)
{
    unsigned char msByte = 0, lsByte = 0;
    unsigned int temp;

    /* Reset ds18b20 */
    if (reset_ds18b20()) {
        printk("Reset ds18b20 failure!\n");
        return 0;
    }

    /* Send cmd to ds18b20 */
    send_ds18b20_cmd(0xCC);         // Skip ROM    
    send_ds18b20_cmd(0x44);         // Start temperature conversion

    if (reset_ds18b20()) {
        printk("Reset ds18b20 failure!\n");
        return 0;
    }
    
    send_ds18b20_cmd(0xCC);         // Skip ROM    
    send_ds18b20_cmd(0xBE);         // Read scratchpad
    lsByte = readByte();            // Read the 0st byte in the scratchpad
    msByte = readByte();            // Read the 1st byte in the scratchpad
    temp = msByte;
    temp <<= 8;
    temp |= lsByte;
    
    return temp;
}

static int rt5350_ds18b20_open(struct inode *inode, struct file *filp)
{
    /* rt5350 ds18b20 gpio mode setup */
    rt5350_ds18b20_gpio_mode_setup();

    /* rt5350 ds18b20 gpio direction setup */
    DS18B20_OUT;
    //rt5350_ds18b20_gpio_dir_setup();
    
    return 0;
}

static ssize_t rt5350_ds18b20_read(struct file *filp, char __user *buf, 
                                size_t size, loff_t *ppos)
{
    unsigned int temp;
    
    temp = readTemp();

    if (copy_to_user(buf, &temp, sizeof(temp)))
        return -EFAULT;
    
    return sizeof(temp);
}

static struct file_operations rt5350_ds18b20_fops = {
    .owner              = THIS_MODULE,
    .open               = rt5350_ds18b20_open,
    .read               = rt5350_ds18b20_read,
};

static void rt5350_ds18b20_setup_cdev(struct rt5350_ds18b20_dev *dev, 
        int minor)
{
    int error;
    dev_t devno = MKDEV(rt5350_ds18b20_major, minor);
    
    /* Initializing cdev */
    cdev_init(&dev->cdev, &rt5350_ds18b20_fops);
    dev->cdev.owner = THIS_MODULE;

    /* Adding cdev */
    error = cdev_add(&dev->cdev, devno, 1);

    if (error) {
        printk(KERN_NOTICE "[KERNEL(%s)]Error %d adding leds", __FUNCTION__, error);
    }
}

/* Mapping the rt5350 gpio registers */
static void rt5350_ds18b20_hw_init(void)
{
    gpio_mode       = (volatile unsigned long *)ioremap(RT5350_SYSCTRL_BASE_ADDR + RT5350_SYSCTRL_GPIOMODE_OFS, 4);
    gpio27_22_data  = (volatile unsigned long *)ioremap(RT5350_GPIO_BASE_ADDR + RT5350_GPIO27_22_DATA_OFS, 4);
    gpio27_22_dir   = (volatile unsigned long *)ioremap(RT5350_GPIO_BASE_ADDR + RT5350_GPIO27_22_DIR_OFS, 4);
}

static void rt5350_ds18b20_hw_exit(void)
{
    iounmap(gpio27_22_dir);
    iounmap(gpio27_22_data);
    iounmap(gpio_mode);
}

static int __init ds18b20_drv_init(void)
{
	int ret = 0;
    dev_t devno = MKDEV(rt5350_ds18b20_major, 0);

    /* Allocating rt5350_ds18b20_dev structure dynamically */
    rt5350_ds18b20_devp = kmalloc(sizeof(struct rt5350_ds18b20_dev), GFP_KERNEL);
    if (!rt5350_ds18b20_devp) {
        return -ENOMEM;
    }

    memset(rt5350_ds18b20_devp, 0, sizeof(struct rt5350_ds18b20_dev));

    rt5350_ds18b20_devp->ds18b20_name = RT5350_DS18B20_NAME;

    /* Register char devices region */
    if (rt5350_ds18b20_major) {
        ret = register_chrdev_region(devno, 1, rt5350_ds18b20_devp->ds18b20_name);
    }else {
        /* Allocating major number dynamically */
        ret = alloc_chrdev_region(&devno, 0, 1, rt5350_ds18b20_devp->ds18b20_name);
        rt5350_ds18b20_major = MAJOR(devno);
    }

    if (ret < 0)
        return ret;

    
    /* Helper function to initialize and add cdev structure */
    rt5350_ds18b20_setup_cdev(rt5350_ds18b20_devp, 0);

    /* mdev - automatically create the device node */
    rt5350_ds18b20_devp->ds18b20_cls = class_create(THIS_MODULE, rt5350_ds18b20_devp->ds18b20_name);
    if (IS_ERR(rt5350_ds18b20_devp->ds18b20_cls))
        return PTR_ERR(rt5350_ds18b20_devp->ds18b20_cls);

    rt5350_ds18b20_devp->ds18b20_dev = device_create(rt5350_ds18b20_devp->ds18b20_cls, NULL, devno, NULL, rt5350_ds18b20_devp->ds18b20_name);    
	if (IS_ERR(rt5350_ds18b20_devp->ds18b20_dev)) {
        class_destroy(rt5350_ds18b20_devp->ds18b20_cls);
        cdev_del(&rt5350_ds18b20_devp->cdev);
        unregister_chrdev_region(devno, 1);
        kfree(rt5350_ds18b20_devp);
		return PTR_ERR(rt5350_ds18b20_devp->ds18b20_dev);
	}

    /* hardware related initialization */
    rt5350_ds18b20_hw_init();

	printk(RT5350_DS18B20_NAME" is initialized!!\n");
    
    return ret;
}
module_init(ds18b20_drv_init);

static void __exit ds18b20_drv_exit(void)
{
    rt5350_ds18b20_hw_exit();
    device_destroy(rt5350_ds18b20_devp->ds18b20_cls, MKDEV(rt5350_ds18b20_major, 0));
    class_destroy(rt5350_ds18b20_devp->ds18b20_cls);
    cdev_del(&rt5350_ds18b20_devp->cdev);
    unregister_chrdev_region(MKDEV(rt5350_ds18b20_major, 0), 1);
    kfree(rt5350_ds18b20_devp);    
	printk(RT5350_DS18B20_NAME" is leaving away!!\n");
}
module_exit(ds18b20_drv_exit);

