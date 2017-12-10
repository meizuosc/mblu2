#ifdef WIN32
#include "win_test.h"
#include "stdio.h"
#else
#ifdef CONFIG_COMPAT

#include <linux/fs.h>
#include <linux/compat.h>

#endif

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <mach/upmu_sw.h>
#endif
#ifdef CONFIG_COMPAT
#include <linux/fs.h>
#include <linux/compat.h>
#endif

/******************************************************************************
 * Definition
******************************************************************************/

/* device name and major number */
#define FLASH_TORCH_DEVNAME            "flash_torch"

/******************************************************************************
 * Debug configuration
******************************************************************************/
#ifdef WIN32
#define logI(fmt, ...)    {printf(fmt, __VA_ARGS__); printf("\n");}
#define logE(fmt, ...)    {printf("merror: %d ", __LINE__); printf(fmt, __VA_ARGS__); printf("\n");}
#else
	#define PFX "[FLASH_TORCH]"
	#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
	#define PK_DBG_FUNC(fmt, arg...)    printk(KERN_INFO PFX "%s: " fmt, __FUNCTION__ ,##arg)

	#define PK_WARN(fmt, arg...)        printk(KERN_WARNING PFX "%s: " fmt, __FUNCTION__ ,##arg)
	#define PK_NOTICE(fmt, arg...)      printk(KERN_NOTICE PFX "%s: " fmt, __FUNCTION__ ,##arg)
	#define PK_INFO(fmt, arg...)        printk(KERN_INFO PFX "%s: " fmt, __FUNCTION__ ,##arg)
	#define PK_TRC_FUNC(f)              printk(PFX "<%s>\n", __FUNCTION__);
	#define PK_TRC_VERBOSE(fmt, arg...) printk(PFX fmt, ##arg)

	#define DEBUG_KD_STROBE
	#ifdef DEBUG_KD_STROBE
	#define logI PK_DBG_FUNC
	#define logE(fmt, arg...)         printk(KERN_ERR PFX "%s: " fmt, __FUNCTION__ ,##arg)
	#else
	#define logI(a,...)
	#define logE(a,...)
	#endif
#endif
extern void FL_Torch(int level);
extern int FL_Disable(void);
extern int FL_Init(void);
//========================================================================
//========================================================================
//========================================================================
/* Kernel interface */
static int flash_torch_open(struct inode *inode, struct file *file)
{
    FL_Init();
    FL_Torch(3);
	return 0;
}

static int flash_torch_close(struct inode *inode, struct file *file)
{
	FL_Disable();
	return 0;
}
ssize_t flash_torch_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	char torch_level[2];
	int level;	
	if (count >= sizeof(torch_level))
		count = sizeof(torch_level)-1;

	if (copy_from_user(torch_level, buf, count))
		return -EFAULT;
	level = torch_level[0] - '0';
	logI("[flash_torch_write] level=%d\n",level);
	FL_Torch(level);
	return count;
}

static struct file_operations flash_torch_fops = {
    .owner      = THIS_MODULE,
    .open = flash_torch_open,
	.release = flash_torch_close,
	.write = flash_torch_write,
};

//========================================================================
// Driver interface
//========================================================================
struct flash_torch_data{
    spinlock_t lock;
    wait_queue_head_t read_wait;
    struct semaphore sem;
};
static struct class *flash_torch_class = NULL;
static struct device *flash_torch_device = NULL;
static struct flash_torch_data flash_torch_private;
static dev_t flash_torch_devno;
static struct cdev flash_torch_cdev;
//========================================================================
static int flash_torch_probe(struct platform_device *dev)
{
    int ret = 0, err = 0;

	logI("[flash_torch_probe] start ~");
    ret = alloc_chrdev_region(&flash_torch_devno, 0, 1, FLASH_TORCH_DEVNAME);
    if (ret) {
        logE("[flash_torch_probe] alloc_chrdev_region fail: %d ~", ret);
        goto flash_torch_probe_error;
    }
    cdev_init(&flash_torch_cdev, &flash_torch_fops);
    flash_torch_cdev.owner = THIS_MODULE;
    err = cdev_add(&flash_torch_cdev, flash_torch_devno, 1);
    if (err) {
        logE("[flash_torch_probe] cdev_add fail: %d ~", err);
        goto flash_torch_probe_error;
    }

    flash_torch_class = class_create(THIS_MODULE, "flashtorch");
    if (IS_ERR(flash_torch_class)) {
        logE("[flash_torch_probe] Unable to create class, err = %d ~", (int)PTR_ERR(flash_torch_class));
        goto flash_torch_probe_error;
    }

    flash_torch_device = device_create(flash_torch_class, NULL, flash_torch_devno, NULL, FLASH_TORCH_DEVNAME);
    if(NULL == flash_torch_device){
        logE("[flash_torch_probe] device_create fail ~");
        goto flash_torch_probe_error;
    }
    //initialize members
    spin_lock_init(&flash_torch_private.lock);
    init_waitqueue_head(&flash_torch_private.read_wait);
    sema_init(&flash_torch_private.sem, 1);

    logI("[flash_torch_probe] Done ~");
    return 0;

flash_torch_probe_error:
    if (err == 0)
        cdev_del(&flash_torch_cdev);
    if (ret == 0)
        unregister_chrdev_region(flash_torch_devno, 1);
    return -1;
}

static int flash_torch_remove(struct platform_device *dev)
{

    logI("[flash_torch_probe] start\n");

    cdev_del(&flash_torch_cdev);
    unregister_chrdev_region(flash_torch_devno, 1);
    device_destroy(flash_torch_class, flash_torch_devno);
    class_destroy(flash_torch_class);

    logI("[flash_torch_probe] Done ~");
    return 0;
}
static void flash_torch_shutdown(struct platform_device *dev)
{

    logI("[flashtorch_shutdown] start\n");
	FL_Disable();    
    logI("[flashtorch_shutdown] Done ~");
}
static struct platform_driver flash_torch_platform_driver =
{
    .probe      = flash_torch_probe,
    .remove     = flash_torch_remove,
    .shutdown   = flash_torch_shutdown,
    .driver     = {
        .name = FLASH_TORCH_DEVNAME,
		.owner	= THIS_MODULE,
    },
};

static struct platform_device flash_torch_platform_device = {
    .name = FLASH_TORCH_DEVNAME,
    .id = 0,
    .dev = {
    }
};

static int __init flash_torch_init(void)
{
    int ret = 0;
    logI("[flash_torch_probe] start ~");

	ret = platform_device_register (&flash_torch_platform_device);
	if (ret) {
        logE("[flash_torch_probe] platform_device_register fail ~");
        return ret;
	}

    ret = platform_driver_register(&flash_torch_platform_driver);
	if(ret){
		logE("[flash_torch_probe] platform_driver_register fail ~");
		return ret;
	}	
	logI("[flash_torch_probe] done! ~");
    return ret;
}

static void __exit flash_torch_exit(void)
{
    logI("[flash_torch_probe] start ~");
    platform_driver_unregister(&flash_torch_platform_driver);
    logI("[flash_torch_probe] done! ~");
}

//========================================================
module_init(flash_torch_init);
module_exit(flash_torch_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("liukun <liukun@wind-mobi.com>");
MODULE_DESCRIPTION("Flash torch control Driver");
