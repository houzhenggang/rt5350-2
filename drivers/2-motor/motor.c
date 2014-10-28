

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

#define RT5350_MOTOR_MAJOR                  0
#define RT5350_MOTOR_NAME                   "rt5350_motor"

#define RT5350_SYSCTRL_BASE_ADDR            0x10000000
#define RT5350_SYSCTRL_GPIOMODE_OFS         0x0060
#define RT5350_GPIO_BASE_ADDR               0x10000600
#define RT5350_GPIO21_00_DATA_OFS           0x0020
#define RT5350_GPIO21_00_DIR_OFS            0x0024
#define RT5350_GPIO27_22_DATA_OFS           0x0070
#define RT5350_GPIO27_22_DIR_OFS            0x0074

#define GPIO21_00_MOTOR_OUT1_GPIO_7_BIT     7
#define GPIO21_00_MOTOR_OUT2_GPIO_8_BIT     8
#define GPIO21_00_MOTOR_OUT3_GPIO_9_BIT     9
#define GPIO21_00_MOTOR_OUT4_GPIO_10_BIT    10
#define GPIO21_00_MOTOR_EN_A_GPIO_17_BIT    17
#define GPIO21_00_MOTOR_EN_B_GPIO_18_BIT    18
#define GPIO27_22_BUZZER_BIT                3

#define INPUT_DIR                           0
#define OUTPUT_DIR                          1
#define DATA_LOW_LEVEL                      0
#define DATA_HIGH_LEVEL                     1

#define GPIOMODE_EPHY_BT_GPIO_MODE          14
#define GPIOMODE_UARTF_SHARE_MODE           2
#define GPIOMODE_JTAG_GPIO_MODE             6
#define EPHY_BT_GPIOMODE_BIT                0x1         // Referenced by RT5350_20101018_un.pdf p.29 [GPIOMODE: GPIO Purpose Select (offset: 0x0060)]
#define UARTF_SHARE_GPIOMODE_BIT            0x7         // Referenced by RT5350_20101018_un.pdf p.11 [UARTF pin share scheme]
#define JTAG_GPIOMODE_BIT                   0x1         // Referenced by RT5350_20101018_un.pdf p.29 [GPIOMODE: GPIO Purpose Select (offset: 0x0060)]

#define WIFI_VEHICLE_MOVE_FORWARD           0
#define WIFI_VEHICLE_MOVE_BACKWARD          1
#define WIFI_VEHICLE_MOVE_LEFT              2
#define WIFI_VEHICLE_MOVE_RIGHT             3
#define WIFI_VEHICLE_STOP                   4
#define WIFI_VEHICLE_BUZZER_ON              5
#define WIFI_VEHICLE_BUZZER_OFF             6

static int rt5350_motor_major = RT5350_MOTOR_MAJOR;

struct rt5350_motor_dev {
    const char *motor_name;
    struct cdev cdev;
    struct class *motor_cls;
    struct device *motor_dev;
};

static struct rt5350_motor_dev *rt5350_motor_devp;

static volatile unsigned long *gpio_mode;
static volatile unsigned long *gpio21_00_dir;
static volatile unsigned long *gpio21_00_data;
static volatile unsigned long *gpio27_22_dir;
static volatile unsigned long *gpio27_22_data;

/* Setup the rt5350 gpio's mode */
static void rt5350_gpio_mode_setup(int gpiobit, int mode)
{
    *gpio_mode |= (gpiobit<<mode);
}

/* 
 * Setup the rt5350 gpio21_00's direction
 * params:
 *      level  = 0:    input
 *      level  = 1:    output
 *      regbit = control bit 
 */
static void rt5350_gpio21_00_dir_setup(int level, int regbit)
{
    if (level)
        *gpio21_00_dir |= (1<<regbit);
    else
        *gpio21_00_dir &= ~(1<<regbit);
}

/* 
 * Setup the rt5350 gpio27_22's direction
 * params:
 *      level  = 0:    input
 *      level  = 1:    output
 *      regbit = control bit 
 */
static void rt5350_gpio27_22_dir_setup(int level, int regbit)
{
    if (level)
        *gpio27_22_dir |= (1<<regbit);
    else
        *gpio27_22_dir &= ~(1<<regbit);
}

static void rt5350_gpio21_00_data_level(int level, int regbit)
{
    if (level)
        *gpio21_00_data |= (1<<regbit);
    else
        *gpio21_00_data &= ~(1<<regbit);
}

static void rt5350_gpio27_22_data_level(int level, int regbit)
{
    if (level)
        *gpio27_22_data |= (1<<regbit);
    else
        *gpio27_22_data &= ~(1<<regbit);
}

/* ################################## buzzer ops ################################## */
static void rt5350_buzzer_gpio_mode_setup(void)
{    
    rt5350_gpio_mode_setup(EPHY_BT_GPIOMODE_BIT, GPIOMODE_EPHY_BT_GPIO_MODE);
}

static void rt5350_buzzer_gpio_dir_setup(void)
{
    /* Setup the buzzer to output */
    rt5350_gpio27_22_dir_setup(OUTPUT_DIR, GPIO27_22_BUZZER_BIT);
}

/* Since buzzer is controlled by gpio#25, we use the gpio27_22 */
/* Disable buzzer */
static void rt5350_buzzer_off(void)
{
    rt5350_gpio27_22_data_level(DATA_LOW_LEVEL, GPIO27_22_BUZZER_BIT);
}

/* Enable buzzer */
static void rt5350_buzzer_on(void)
{
    rt5350_gpio27_22_data_level(DATA_HIGH_LEVEL, GPIO27_22_BUZZER_BIT);
}
/* ################################## buzzer ops ################################## */

/* ################################## motor ops ################################## */

/* Moter configs and operations */
static void rt5350_motor_gpio_mode_setup(void)
{
    /* 
     *  GPIO#7  = Control OUT1
     *  GPIO#8  = Control OUT2
     *  GPIO#9  = Control OUT3
     *  GPIO#10 = Control OUT4
     */
    rt5350_gpio_mode_setup(UARTF_SHARE_GPIOMODE_BIT, GPIOMODE_UARTF_SHARE_MODE);

    /* 
     *  GPIO#17  = Control EN_A
     *  GPIO#18  = Control EN_B
     */
    rt5350_gpio_mode_setup(JTAG_GPIOMODE_BIT, GPIOMODE_JTAG_GPIO_MODE);
}

static void rt5350_motor_gpio_dir_setup(void)
{
    /* Setup the motor's GPIO#7,8,9,10,17,18 to output */
    rt5350_gpio21_00_dir_setup(OUTPUT_DIR, GPIO21_00_MOTOR_OUT1_GPIO_7_BIT);
    rt5350_gpio21_00_dir_setup(OUTPUT_DIR, GPIO21_00_MOTOR_OUT2_GPIO_8_BIT);
    rt5350_gpio21_00_dir_setup(OUTPUT_DIR, GPIO21_00_MOTOR_OUT3_GPIO_9_BIT);
    rt5350_gpio21_00_dir_setup(OUTPUT_DIR, GPIO21_00_MOTOR_OUT4_GPIO_10_BIT);
    rt5350_gpio21_00_dir_setup(OUTPUT_DIR, GPIO21_00_MOTOR_EN_A_GPIO_17_BIT);
    rt5350_gpio21_00_dir_setup(OUTPUT_DIR, GPIO21_00_MOTOR_EN_B_GPIO_18_BIT);
}

static void rt5350_motor_gpio_data_level(int level, int regbit)
{
    rt5350_gpio21_00_data_level(level, regbit);
}

/*
 *  desc: In order to move the vehicle forward, we do the following:
 *      OUT1 = low level
 *      OUT2 = high level
 *      OUT3 = low level
 *      OUT4 = high level
 */
static void rt5350_motor_move_forward(void)
{
    rt5350_gpio21_00_data_level(DATA_LOW_LEVEL, GPIO21_00_MOTOR_OUT1_GPIO_7_BIT);       // OUT1 = 0
    rt5350_gpio21_00_data_level(DATA_HIGH_LEVEL, GPIO21_00_MOTOR_OUT2_GPIO_8_BIT);      // OUT2 = 1
    rt5350_gpio21_00_data_level(DATA_LOW_LEVEL, GPIO21_00_MOTOR_OUT3_GPIO_9_BIT);       // OUT3 = 0
    rt5350_gpio21_00_data_level(DATA_HIGH_LEVEL, GPIO21_00_MOTOR_OUT4_GPIO_10_BIT);     // OUT4 = 1
}

/*
 *  desc: In order to move the vehicle backward, we do the following:
 *      OUT1 = high level
 *      OUT2 = low level
 *      OUT3 = high level
 *      OUT4 = low level
 */
static void rt5350_motor_move_backward(void)
{
    rt5350_gpio21_00_data_level(DATA_HIGH_LEVEL, GPIO21_00_MOTOR_OUT1_GPIO_7_BIT);      // OUT1 = 1
    rt5350_gpio21_00_data_level(DATA_LOW_LEVEL, GPIO21_00_MOTOR_OUT2_GPIO_8_BIT);       // OUT2 = 0
    rt5350_gpio21_00_data_level(DATA_HIGH_LEVEL, GPIO21_00_MOTOR_OUT3_GPIO_9_BIT);      // OUT3 = 1
    rt5350_gpio21_00_data_level(DATA_LOW_LEVEL, GPIO21_00_MOTOR_OUT4_GPIO_10_BIT);      // OUT4 = 0
}

/*
 *  desc: In order to move the vehicle left, we do the following:
 *      OUT1 = low level
 *      OUT2 = high level
 *      OUT3 = high level
 *      OUT4 = high level
 *  note: OUT1, OUT2 control the right tire
 *        OUT3, OUT4 control the left tire
 *        so to control the vehicle move left, we simply set the out1 to low, out2 to high, 
 *        and keep the out3, out4 both to low or high, here, we set the out3, out4 to high
 *        because we wanna light the left LED up based on the schematic diagram.
 */
static void rt5350_motor_move_left(void)
{
    rt5350_gpio21_00_data_level(DATA_LOW_LEVEL, GPIO21_00_MOTOR_OUT1_GPIO_7_BIT);       // OUT1 = 0
    rt5350_gpio21_00_data_level(DATA_HIGH_LEVEL, GPIO21_00_MOTOR_OUT2_GPIO_8_BIT);      // OUT2 = 1
    rt5350_gpio21_00_data_level(DATA_HIGH_LEVEL, GPIO21_00_MOTOR_OUT3_GPIO_9_BIT);      // OUT3 = 1
    rt5350_gpio21_00_data_level(DATA_HIGH_LEVEL, GPIO21_00_MOTOR_OUT4_GPIO_10_BIT);     // OUT4 = 1
}

static void rt5350_motor_move_right(void)
{
    rt5350_gpio21_00_data_level(DATA_HIGH_LEVEL, GPIO21_00_MOTOR_OUT1_GPIO_7_BIT);      // OUT1 = 1
    rt5350_gpio21_00_data_level(DATA_HIGH_LEVEL, GPIO21_00_MOTOR_OUT2_GPIO_8_BIT);      // OUT2 = 1
    rt5350_gpio21_00_data_level(DATA_LOW_LEVEL, GPIO21_00_MOTOR_OUT3_GPIO_9_BIT);       // OUT3 = 0
    rt5350_gpio21_00_data_level(DATA_HIGH_LEVEL, GPIO21_00_MOTOR_OUT4_GPIO_10_BIT);     // OUT4 = 1
}

static void rt5350_motor_stop_ledoff(void)
{
    rt5350_gpio21_00_data_level(DATA_LOW_LEVEL, GPIO21_00_MOTOR_OUT1_GPIO_7_BIT);       // out1 = 0
    rt5350_gpio21_00_data_level(DATA_LOW_LEVEL, GPIO21_00_MOTOR_OUT2_GPIO_8_BIT);       // out2 = 0
    rt5350_gpio21_00_data_level(DATA_LOW_LEVEL, GPIO21_00_MOTOR_OUT3_GPIO_9_BIT);       // out3 = 0
    rt5350_gpio21_00_data_level(DATA_LOW_LEVEL, GPIO21_00_MOTOR_OUT4_GPIO_10_BIT);      // out4 = 0
}

static void rt5350_motor_stop_ledon(void)
{
    rt5350_gpio21_00_data_level(DATA_HIGH_LEVEL, GPIO21_00_MOTOR_OUT1_GPIO_7_BIT);      // out1 = 0
    rt5350_gpio21_00_data_level(DATA_HIGH_LEVEL, GPIO21_00_MOTOR_OUT2_GPIO_8_BIT);      // out2 = 0
    rt5350_gpio21_00_data_level(DATA_HIGH_LEVEL, GPIO21_00_MOTOR_OUT3_GPIO_9_BIT);      // out3 = 0
    rt5350_gpio21_00_data_level(DATA_HIGH_LEVEL, GPIO21_00_MOTOR_OUT4_GPIO_10_BIT);     // out4 = 0
}

/* ################################## motor ops ################################## */

static int rt5350_motor_open(struct inode *inode, struct file *filp)
{
    /* rt5350 motor gpio mode setup */
    rt5350_motor_gpio_mode_setup();

    /* rt5350 motor gpio direction setup */
    rt5350_motor_gpio_dir_setup();
    
    return 0;
}

/* Based on the write api, the system allows to control the motor speed by using the duty ratio */
static ssize_t rt5350_motor_write(struct file *filp, const char __user *buf, 
                                    size_t size, loff_t *ppos)
{
    char val;

    if (copy_from_user(&val, buf, 1))
		return -EFAULT;

    /* if the user passing 1 to the kernel, which the system will set the motor to high level, otherwise, to low level */
    if (val & 0x1) {
        rt5350_motor_gpio_data_level(DATA_HIGH_LEVEL, GPIO21_00_MOTOR_EN_A_GPIO_17_BIT);
        rt5350_motor_gpio_data_level(DATA_HIGH_LEVEL, GPIO21_00_MOTOR_EN_B_GPIO_18_BIT);
    }else {
        rt5350_motor_gpio_data_level(DATA_LOW_LEVEL, GPIO21_00_MOTOR_EN_A_GPIO_17_BIT);
        rt5350_motor_gpio_data_level(DATA_LOW_LEVEL, GPIO21_00_MOTOR_EN_B_GPIO_18_BIT);
    }
    
    return 1;
}

static long rt5350_motor_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)    
{
    switch (cmd) {
    case WIFI_VEHICLE_MOVE_FORWARD:
        rt5350_motor_stop_ledoff();
        rt5350_motor_move_forward();        /* led all off */
        break;

    case WIFI_VEHICLE_MOVE_BACKWARD:
        rt5350_motor_stop_ledoff();
        rt5350_motor_move_backward();       /* led all on */
        break;

    case WIFI_VEHICLE_MOVE_LEFT:
        rt5350_motor_stop_ledoff();
        rt5350_motor_move_left();           /* left led on */
        mdelay(20);
        rt5350_motor_stop_ledon();
        break;

    case WIFI_VEHICLE_MOVE_RIGHT:
        rt5350_motor_stop_ledoff();
        rt5350_motor_move_right();          /* right led on */
        mdelay(20);
        rt5350_motor_stop_ledon();
        break;
       
    case WIFI_VEHICLE_STOP:
        rt5350_motor_stop_ledoff();         /* rt5350_motor_stop_ledon is fine as well */
        break;

    case WIFI_VEHICLE_BUZZER_ON:
        rt5350_buzzer_on();
        break;

    case WIFI_VEHICLE_BUZZER_OFF:
        rt5350_buzzer_off();
        break;

    default:
        break;
    }
    
    return 0;
}

static struct file_operations rt5350_motor_fops = {
    .owner              = THIS_MODULE,
    .open               = rt5350_motor_open,
    .write              = rt5350_motor_write,
    .unlocked_ioctl     = rt5350_motor_ioctl,
};

static void rt5350_motor_setup_cdev(struct rt5350_motor_dev *dev, 
        int minor)
{
    int error;
    dev_t devno = MKDEV(rt5350_motor_major, minor);
    
    /* Initializing cdev */
    cdev_init(&dev->cdev, &rt5350_motor_fops);
    dev->cdev.owner = THIS_MODULE;

    /* Adding cdev */
    error = cdev_add(&dev->cdev, devno, 1);

    if (error) {
        printk(KERN_NOTICE "[KERNEL(%s)]Error %d adding leds", __FUNCTION__, error);
    }
}

/* Mapping the rt5350 gpio registers */
static void rt5350_motor_hw_init(void)
{
    gpio_mode       = (volatile unsigned long *)ioremap(RT5350_SYSCTRL_BASE_ADDR + RT5350_SYSCTRL_GPIOMODE_OFS, 4);
    gpio21_00_data  = (volatile unsigned long *)ioremap(RT5350_GPIO_BASE_ADDR + RT5350_GPIO21_00_DATA_OFS, 4);
    gpio21_00_dir   = (volatile unsigned long *)ioremap(RT5350_GPIO_BASE_ADDR + RT5350_GPIO21_00_DIR_OFS, 4);
    gpio27_22_data  = (volatile unsigned long *)ioremap(RT5350_GPIO_BASE_ADDR + RT5350_GPIO27_22_DATA_OFS, 4);
    gpio27_22_dir   = (volatile unsigned long *)ioremap(RT5350_GPIO_BASE_ADDR + RT5350_GPIO27_22_DIR_OFS, 4);
}

static void rt5350_motor_hw_exit(void)
{
    iounmap(gpio27_22_dir);
    iounmap(gpio27_22_data);
    iounmap(gpio21_00_dir);
    iounmap(gpio21_00_data);
    iounmap(gpio_mode);
}

static int __init motor_drv_init(void)
{
	int ret = 0;
    dev_t devno = MKDEV(rt5350_motor_major, 0);

    /* Allocating rt5350_motor_dev structure dynamically */
    rt5350_motor_devp = kmalloc(sizeof(struct rt5350_motor_dev), GFP_KERNEL);
    if (!rt5350_motor_devp) {
        return -ENOMEM;
    }

    memset(rt5350_motor_devp, 0, sizeof(struct rt5350_motor_dev));

    rt5350_motor_devp->motor_name = RT5350_MOTOR_NAME;

    /* Register char devices region */
    if (rt5350_motor_major) {
        ret = register_chrdev_region(devno, 1, rt5350_motor_devp->motor_name);
    }else {
        /* Allocating major number dynamically */
        ret = alloc_chrdev_region(&devno, 0, 1, rt5350_motor_devp->motor_name);
        rt5350_motor_major = MAJOR(devno);
    }

    if (ret < 0)
        return ret;

    
    /* Helper function to initialize and add cdev structure */
    rt5350_motor_setup_cdev(rt5350_motor_devp, 0);

    /* mdev - automatically create the device node */
    rt5350_motor_devp->motor_cls = class_create(THIS_MODULE, rt5350_motor_devp->motor_name);
    if (IS_ERR(rt5350_motor_devp->motor_cls))
        return PTR_ERR(rt5350_motor_devp->motor_cls);

    rt5350_motor_devp->motor_dev = device_create(rt5350_motor_devp->motor_cls, NULL, devno, NULL, rt5350_motor_devp->motor_name);    
	if (IS_ERR(rt5350_motor_devp->motor_dev)) {
        class_destroy(rt5350_motor_devp->motor_cls);
        cdev_del(&rt5350_motor_devp->cdev);
        unregister_chrdev_region(devno, 1);
        kfree(rt5350_motor_devp);
		return PTR_ERR(rt5350_motor_devp->motor_dev);
	}

    /* Leds hardware related initialization */
    rt5350_motor_hw_init();

    /* Setup the gpio mode */
    rt5350_buzzer_gpio_mode_setup();

    /* Setup the buzzer gpio direction to output */
    rt5350_buzzer_gpio_dir_setup();
    
    /* Disable buzzer */
    rt5350_buzzer_off();

	printk(RT5350_MOTOR_NAME" is initialized!!\n");
    
    return ret;
}
module_init(motor_drv_init);

static void __exit motor_drv_exit(void)
{
    rt5350_motor_hw_exit();
    device_destroy(rt5350_motor_devp->motor_cls, MKDEV(rt5350_motor_major, 0));
    class_destroy(rt5350_motor_devp->motor_cls);
    cdev_del(&rt5350_motor_devp->cdev);
    unregister_chrdev_region(MKDEV(rt5350_motor_major, 0), 1);
    kfree(rt5350_motor_devp);    
	printk(RT5350_MOTOR_NAME" is left away!!\n");
}
module_exit(motor_drv_exit);

