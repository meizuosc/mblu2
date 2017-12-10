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
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <cust_gpio_usage.h>
#include <cust_i2c.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
#include <linux/mutex.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#endif

#include <linux/i2c.h>
#include <linux/leds.h>
int led_vendor_id=0;
static int g_duty=-1;
static int g_timeOutTimeMs=0;
#define STROBE_DEVICE_ID 0xC6
/* liukun@wind-mobi.com 20141231 begin */
#define I2C_STROBE_SKY81294_7_BIT_ADDR 0x37
#define I2C_STROBE_SKY81294 2
/* liukun@wind-mobi.com 20141231 end */

#define TAG_NAME "leds_sky81294.c"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        xlog_printk(ANDROID_LOG_WARNING, TAG_NAME, KERN_WARNING  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_NOTICE  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        xlog_printk(ANDROID_LOG_INFO   , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME,  "<%s>\n", __FUNCTION__);
#define PK_TRC_VERBOSE(fmt, arg...) xlog_printk(ANDROID_LOG_VERBOSE, TAG_NAME,  fmt, ##arg)
#define PK_ERROR(fmt, arg...)       xlog_printk(ANDROID_LOG_ERROR  , TAG_NAME, KERN_ERR "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif

static struct work_struct workTimeOut;

static struct i2c_client *SKY81294_i2c_client = NULL;

struct SKY81294_platform_data {
	u8 torch_pin_enable;    // 1:  TX1/TORCH pin isa hardware TORCH enable
	u8 pam_sync_pin_enable; // 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input
	u8 thermal_comp_mode_enable;// 1: LEDI/NTC pin in Thermal Comparator Mode
	u8 strobe_pin_disable;  // 1 : STROBE Input disabled
	u8 vout_mode_enable;  // 1 : Voltage Out Mode enable
};

struct SKY81294_chip_data {
	struct i2c_client *client;
	struct SKY81294_platform_data *pdata;
	struct mutex lock;
	u8 last_flag;
	u8 no_pdata;
};

static int SKY81294_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret=0;
	struct SKY81294_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret =  i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		PK_ERR("failed writting at 0x%02x\n", reg);
	return ret;
}

static int SKY81294_read_reg(struct i2c_client *client, u8 reg)
{
	int val=0;
	struct SKY81294_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val =  i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);


	return val;
}
static int SKY81294_chip_init(struct SKY81294_chip_data *chip)
{
    s32 regVal6 = 0;
	regVal6 = SKY81294_read_reg(SKY81294_i2c_client, 0x06);
	if (regVal6 < 0) 
	{
	 PK_DBG("SKY81294 i2c bus read error\n");
	 return -1;
	}
    else
    {
     led_vendor_id = 0x8129;
    }
	return 0;
}
int is_ic_sky81294(void)
{
    return led_vendor_id;
}
/* liukun@wind-mobi.com 20150509 begin */
void SKY81294_torch_set_level(int level)
{
	  char buf[2];
	  PK_DBG("SKY81294 %s,%d,%d,level",__func__,__LINE__,level);
	  buf[0]=0x02;
	  buf[1]=level;  
	  SKY81294_write_reg(SKY81294_i2c_client, buf[0], buf[1]);
	  buf[0]=0x03;
	  buf[1]=0x01;
	  SKY81294_write_reg(SKY81294_i2c_client, buf[0], buf[1]);	  
}
/* liukun@wind-mobi.com 20150509 end */
void SKY81294_torch_mode(int g_duty)
{
      char buf[2];
      PK_DBG("SKY81294 %s,%d,%d,torch",__func__,__LINE__,g_duty);
      buf[0]=0x02;
      buf[1]=0x07;//205 MA    
  	  SKY81294_write_reg(SKY81294_i2c_client, buf[0], buf[1]);
      buf[0]=0x03;
      buf[1]=0x01;
  	  SKY81294_write_reg(SKY81294_i2c_client, buf[0], buf[1]);    
}
void SKY81294_flash_mode(int g_duty)
{
      char buf[2];
      PK_DBG("SKY81294 %s,%d,%d,flash",__func__,__LINE__,g_duty);
      buf[0]=0x00;
      buf[1]=g_duty;
      SKY81294_write_reg(SKY81294_i2c_client, buf[0], buf[1]);
      buf[0]=0x03;
      buf[1]=0x02;
      SKY81294_write_reg(SKY81294_i2c_client, buf[0], buf[1]);   
}
void SKY81294_shutdown_mode(void)
{
      char buf[2];
      buf[0]=0x03;
      buf[1]=0x00;
      SKY81294_write_reg(SKY81294_i2c_client, buf[0], buf[1]);    
}
static int SKY81294_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct SKY81294_chip_data *chip;
	struct SKY81294_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	PK_DBG("SKY81294_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		printk(KERN_ERR  "SKY81294 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct SKY81294_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if(pdata == NULL){ //values are set to Zero.
		PK_ERR("SKY81294 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct SKY81294_platform_data),GFP_KERNEL);
		chip->pdata  = pdata;
		chip->no_pdata = 1;
	}
	SKY81294_i2c_client = client;
	chip->pdata  = pdata;
	if(SKY81294_chip_init(chip)<0)
		goto err_chip_init;

	PK_DBG("SKY81294 Initializing is done \n");

	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	PK_ERR("SKY81294 probe is failed \n");
	return -ENODEV;
}

static int SKY81294_remove(struct i2c_client *client)
{
	struct SKY81294_chip_data *chip = i2c_get_clientdata(client);

    if(chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}

#define SKY81294_NAME "leds-SKY81294"
static const struct i2c_device_id SKY81294_id[] = {
	{SKY81294_NAME, 0},
	{}
};

static struct i2c_driver SKY81294_i2c_driver = {
	.driver = {
		.name  = SKY81294_NAME,
	},
	.probe	= SKY81294_probe,
	.remove   = SKY81294_remove,
	.id_table = SKY81294_id,
};

struct SKY81294_platform_data SKY81294_pdata = {0, 0, 0, 0, 0};
static struct i2c_board_info __initdata i2c_SKY81294={ I2C_BOARD_INFO(SKY81294_NAME, I2C_STROBE_SKY81294_7_BIT_ADDR), \
													.platform_data = &SKY81294_pdata,};

static int __init SKY81294_init(void)
{
	printk("SKY81294_init\n");
	i2c_register_board_info(I2C_STROBE_SKY81294, &i2c_SKY81294, 1);
	return i2c_add_driver(&SKY81294_i2c_driver);
}

static void __exit SKY81294_exit(void)
{
	i2c_del_driver(&SKY81294_i2c_driver);
}

module_init(SKY81294_init);
module_exit(SKY81294_exit);

MODULE_DESCRIPTION("Flash driver for SKY81294");
MODULE_AUTHOR("<liukun@wind-mobi.com>");
MODULE_LICENSE("GPL v2");
