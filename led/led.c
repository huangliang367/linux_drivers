#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#define LED_CNT     1
#define LED_NAME    "led"
#define LED_OFF     0
#define LED_ON      1

static void __iomem *IMX6U_CCM_CCGR1;
static void __iomem *SW_MUX_GPIO1_IO03;
static void __iomem *SW_PAD_GPIO1_IO03;
static void __iomem *GPIO1_DR;
static void __iomem *GPIO1_GDIR;

struct led_dev {
    dev_t devid;
    struct cdev cdev;
    struct class *class;
    struct device *device;
    int major;
    int minor;
    struct device_node *nd;
};

struct led_dev led_device;

void led_switch(u8 stat)
{
    u32 val = 0;

    if (stat == LED_ON) {
        val = readl(GPIO1_DR);
        val &= ~(1 << 3);
        writel(val, GPIO1_DR);
    } else if (stat == LED_OFF) {
        val = readl(GPIO1_DR);
        val |= (1 << 3);
        writel(val, GPIO1_DR);
    }
}

static int led_open(struct inode *inode, struct file *filp)
{
    filp->private_data = &led_device;
    return 0;
}

static ssize_t led_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt)
{
    return 0;
}

static ssize_t led_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *offt)
{
    int ret_value;
    unsigned char data_buf;
    unsigned char led_stat;

    ret_value = copy_from_user(&data_buf, buf, cnt);
    if (ret_value < 0) {
        printk("kernel write failed!\r\n");
        return -EFAULT;
    }

    led_stat = data_buf;

    if (led_stat == LED_ON) {
        led_switch(LED_ON);
    } else if (led_stat == LED_OFF) {
        led_switch(LED_OFF);
    }

    return 0;
}

static int led_release(struct inode *inode, struct file *filp)
{
    return 0;
}

static struct file_operations led_fops = {
    .owner = THIS_MODULE,
    .open = led_open,
    .read = led_read,
    .write = led_write,
    .release = led_release,
};

static int __init led_init(void)
{
    u32 val = 0;
    int ret = 0;
    u32 regdata[14];
    const char *str;
    struct property *proper;

    led_device.nd = of_find_node_by_path("/led");
    if (NULL == led_device.nd) {
        printk("led node does not exist!\r\n");
        return -EINVAL;
    } else {
        printk("Find node 'led'!\r\n");
    }

    proper = of_find_property(led_device.nd, "compatible", NULL);
    if (NULL == proper) {
        printk("compatible property find failed!\r\n");
    } else {
        printk("compatile = %s\r\n", (char *)proper->value);
    }

    ret = of_property_read_string(led_device.nd, "status", &str);
    if (ret < 0) {
        printk("status read failed!\r\n");
    } else {
        printk("status = %s\r\n", str);
    }

    ret = of_property_read_u32_array(led_device.nd, "reg", regdata, 10);
    if (ret < 0) {
        printk("reg property read failed!\r\n");
    } else {
        u8 i = 0;
        printk("reg data:\r\n");
        for (i = 0; i < 10; i++)
            printk("%#X ", regdata[i]);
        printk("\r\n");
    }

    IMX6U_CCM_CCGR1 = of_iomap(led_device.nd, 0);
    SW_MUX_GPIO1_IO03 = of_iomap(led_device.nd, 1);
    SW_PAD_GPIO1_IO03 = of_iomap(led_device.nd, 2);
    GPIO1_DR = of_iomap(led_device.nd, 3);
    GPIO1_GDIR = of_iomap(led_device.nd, 4);

    val = readl(IMX6U_CCM_CCGR1);
    val &= ~(3 << 26);
    val |= (3 << 26);
    writel(val, IMX6U_CCM_CCGR1);

    writel(5, SW_MUX_GPIO1_IO03);

    writel(0x10B0, SW_PAD_GPIO1_IO03);

    val = readl(GPIO1_GDIR);
    val &= ~(1 << 3);
    val |= (1 << 3);
    writel(val, GPIO1_GDIR);

    val = readl(GPIO1_DR);
    val |= (1 << 3);
    writel(val, GPIO1_DR);

    if (led_device.major) {
        led_device.devid = MKDEV(led_device.major, 0);
        register_chrdev_region(led_device.devid, LED_CNT, LED_NAME);
    } else {
        alloc_chrdev_region(&led_device.devid, 0, LED_CNT, LED_NAME);
        led_device.major = MAJOR(led_device.devid);
        led_device.minor = MINOR(led_device.devid);
    }
    printk("led major = %d, minor = %d\r\n", led_device.major, led_device.minor);

    led_device.cdev.owner = THIS_MODULE;
    cdev_init(&led_device.cdev, &led_fops);

    cdev_add(&led_device.cdev, led_device.devid, LED_CNT);

    led_device.class = class_create(THIS_MODULE, LED_NAME);
    if (IS_ERR(led_device.class))
        return PTR_ERR(led_device.class);

    led_device.device = device_create(led_device.class, NULL, led_device.devid, NULL, LED_NAME);
    if (IS_ERR(led_device.device))
        return PTR_ERR(led_device.device);

    return 0;
}

static void __exit led_exit(void)
{
    iounmap(IMX6U_CCM_CCGR1);
    iounmap(SW_MUX_GPIO1_IO03);
    iounmap(SW_PAD_GPIO1_IO03);
    iounmap(GPIO1_DR);
    iounmap(GPIO1_GDIR);
}

module_init(led_init);
module_exit(led_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("HuangLiang");