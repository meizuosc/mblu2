/**************************************************************************
*  AW9201_ts_1Button.c
* 
*  Create Date :
* 
*  Modify Date : 
*
*  Create by   : AWINIC Technology CO., LTD
*
*  Version     : 1.0 , 2014/12/ 26
**************************************************************************/
//////////////////////////////////////////////////////////////
//  
//  APPLICATION DEFINE :
//
//                   Mobile -         HOME     
//                   AW9201 -       S1
//
//////////////////////////////////////////////////////////////

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/earlysuspend.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>

#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/gameport.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>

#include <mach/mt_gpio.h>
#include "cust_gpio_usage.h"
#include <cust_eint.h>
#include <mach/mt_gpt.h>
#include <linux/wakelock.h>
#include <mach/eint.h>

#define TS_DEBUG_MSG 			0

//////////////////////////////////////
// IO PIN DEFINE
//////////////////////////////////////
#define AW9201_EINT_NUM     CUST_EINT_CP128_EINT_NUM
#define AW9201_EINT_PIN     GPIO_CP281_EINT_PIN

#define AW9201_ts_NAME	   	       	"AW9201_ts"
#define AW9201_ts_I2C_ADDR		     0x45
#define AW9201_ts_I2C_BUS		     2

#define AW9201 
#define AW9201_EINT_SUPPORT   // use interrupt mode


#define ABS(x,y)         ((x>y)?(x-y):(y-x))


/*oujiacheng@wind-mobi.com 2015.5.19 add begin*/
#define AW_AUTO_CALI
#ifdef AW_AUTO_CALI
/*oujiacheng@wind-mobi.com 2015.7.10 add begin*/
#ifdef CONFIG_MTK_TOUCHKEY_AW9201_COMPATIBLE
#define CALI_NUM	8
#else 
#define CALI_NUM	2
#endif
/*oujiacheng@wind-mobi.com 2015.7.10 add end*/
#define CALI_RAW_MIN	250
#define CALI_RAW_MAX	1750	

unsigned char cali_flag = 0;
unsigned char cali_num = 0;
unsigned char cali_cnt = 0;
unsigned char cali_used = 0;
unsigned char old_cali_dir;	//	0: no cali		1: ofr pos cali		2: ofr neg cali
unsigned char old_ofr_cfg;

unsigned int  rawdata_sum;
#endif
/*oujiacheng@wind-mobi.com 2015.5.19 add end*/



static int debug_level=0;

#if 1
#define TS_DBG printk
#endif



static int AW9201_create_sysfs(struct i2c_client *client);

struct AW9201_i2c_setup_data {
	unsigned i2c_bus;  //the same number as i2c->adap.nr in adapter probe function
	unsigned short i2c_address;
	int irq;
	char type[I2C_NAME_SIZE];
};

struct AW9201_ts_data {
	struct input_dev	*input_dev;
#ifdef AW9201_EINT_SUPPORT
	struct work_struct 	eint_work;
#else
	struct work_struct 	pen_event_work;
	struct workqueue_struct *ts_workqueue;
	struct timer_list touch_timer;
#endif
	struct early_suspend	early_suspend;
};

struct AW9201_ts_data *AW9201_ts;
struct wake_lock touchkey_wakelock;
//////////////////////////////////////////////////////
//
// for adb shell and APK debug
//
//////////////////////////////////////////////////////
static ssize_t AW9201_show_debug(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t AW9201_store_debug(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW9201_get_reg(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t AW9201_write_reg(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW9201_get_Base(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t AW9201_get_rawdata(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t AW9201_get_delta(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t AW9201_get_irqstate(struct device* cd,struct device_attribute *attr, char* buf);
	

static DEVICE_ATTR(debug, S_IWUGO | S_IRUGO, AW9201_show_debug, AW9201_store_debug);
static DEVICE_ATTR(getreg,  S_IWUGO | S_IRUGO, AW9201_get_reg,    AW9201_write_reg);
static DEVICE_ATTR(base,  S_IWUGO | S_IRUGO, AW9201_get_Base,    NULL);
static DEVICE_ATTR(rawdata,  S_IWUGO | S_IRUGO, AW9201_get_rawdata,    NULL);
static DEVICE_ATTR(delta,  S_IWUGO | S_IRUGO, AW9201_get_delta,    NULL);
static DEVICE_ATTR(getstate,  S_IWUGO | S_IRUGO, AW9201_get_irqstate,    NULL);
/* liukun@wind-mobi.com 20150116 begin */
static struct attribute *aw9201_attributes[] = {
	&dev_attr_debug.attr,
	&dev_attr_getreg.attr,
	&dev_attr_base.attr,
	&dev_attr_rawdata.attr,
	&dev_attr_delta.attr,
	&dev_attr_getstate.attr,
	NULL
};
static struct attribute_group aw9201_attribute_group[] = {
	{.attrs = aw9201_attributes },
};
/* liukun@wind-mobi.com 20150116 end */

//////////////////////////////////////////////////////
//
// Touch process variable
//
//////////////////////////////////////////////////////

static unsigned char suspend_flag = 0 ; //0: normal; 1: sleep

static int WorkMode             = 1 ; //1: sleep, 2: normal

//////////////////////////////////////////////////////

static struct i2c_client *this_client;
static struct AW9201_i2c_setup_data AW9201_ts_setup={AW9201_ts_I2C_BUS, AW9201_ts_I2C_ADDR, 0, AW9201_ts_NAME};


#if 0
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif
#if 1//def MT6582
extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void);
#endif

//////////////////////////////////////////////////////
//
// PDN power control
//
//////////////////////////////////////////////////////
#if 0
static void AW9201_ts_pwron(void)
{
    mt_set_gpio_mode(AW9201_PDN,GPIO_MODE_00);  
    mt_set_gpio_dir(AW9201_PDN,GPIO_DIR_OUT);
    mt_set_gpio_out(AW9201_PDN,GPIO_OUT_ONE);
    msleep(20);
}

static void AW9201_ts_pwroff(void)
{
	    mt_set_gpio_out(AW9201_PDN,GPIO_OUT_ZERO);  
}
#endif

static void AW9201_ts_config_pins(void)
{
	//AW9201_ts_pwron();   
	msleep(10); //wait for stable
}


//////////////////////////////////////////////////////
//
// i2c write and read
//
//////////////////////////////////////////////////////

static unsigned int I2C_write_reg(unsigned char addr, unsigned char reg_data)
{

	   int ret,i;
	u8 wdbuf[512] = {0};

	wdbuf[0] = addr;

	wdbuf[1] = reg_data;

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= wdbuf,
		},
	};

	ret = i2c_transfer(this_client->adapter, msgs, 1);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

    return ret;

}


static unsigned char I2C_read_reg(unsigned char addr)
{

	unsigned char ret,i;
	u8 rdbuf[512] = {0};

	rdbuf[0] = addr;
	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rdbuf,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= rdbuf,
		},
	};

	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

    return rdbuf[0];
	
}

/* oujiacheng@wind-mobi.com 2015.5.19 add begin
 * AW92xx Auto Calibration
*/
#ifdef AW_AUTO_CALI
/*oujiacheng@wind-mobi.com 2015.7.10 add begin*/
#ifndef CONFIG_MTK_TOUCHKEY_AW9201_COMPATIBLE
unsigned char AW92xx_Auto_Cali()
{
	unsigned char i;
	unsigned char cali_dir;
	unsigned char ofr_cfg;
	unsigned char rawdata[2];

	if(cali_num == 0)
	{
		ofr_cfg = I2C_read_reg(0x07);
	}
	else
	{
		ofr_cfg = old_ofr_cfg;
	}
	
	rawdata[0] = I2C_read_reg(0x20);
	rawdata[1] = I2C_read_reg(0x21);

	rawdata_sum = (cali_cnt==0)? (0) : (rawdata_sum + (rawdata[0]<<8) | rawdata[1]);

	if(cali_cnt==4)
	{
		if((rawdata_sum>>2) < CALI_RAW_MIN)
		{
			if((ofr_cfg&0x1F) == 0x00)
			{
				cali_dir = 0;
			}
			else
			{
				cali_dir = 2;
				ofr_cfg = ofr_cfg -1;
			}
		}
		else if ((rawdata_sum>>2) > CALI_RAW_MAX)
		{
			if((ofr_cfg&0x1F) == 0x1F)
			{
				cali_dir = 0;
			}
			else
			{
				cali_dir = 1;
				ofr_cfg = ofr_cfg + 1;
			}
		}
		else
		{
			cali_dir = 0;
		}

		if(cali_num > 0)
		{
			if(cali_dir != old_cali_dir)
			{
				cali_dir = 0;
				ofr_cfg = old_ofr_cfg;
			}
		}
		
		cali_flag = 0;
		if(cali_dir != 0)
		{
			cali_flag = 1;
		}
		

		if((cali_flag==0) && (cali_num==0))
		{
			cali_used = 0;
		}
		else
		{
			cali_used = 1;
		}
		
		if(cali_flag == 0)
		{
			cali_num = 0;
			cali_cnt = 0;
			return 0;
		}
		

		I2C_write_reg(0x07, ofr_cfg);
		I2C_write_reg(0x01, 0x00);
		I2C_write_reg(0x01, 0x02);

		if(cali_num == (CALI_NUM -1))	// no calibration
		{
			cali_flag = 0;
			cali_num = 0;
			cali_cnt = 0;
			
			return 0;
		}
		
		old_cali_dir = cali_dir;
		old_ofr_cfg = ofr_cfg;
	
		cali_num ++;
	}

	if(cali_cnt < 4)
	{
		cali_cnt ++;	
	}
	else
	{
		cali_cnt = 0;
	}
	
	return 1;
}
#else
unsigned char AW92xx_Auto_Cali()
{
	unsigned char i;
	unsigned char cali_dir;
	unsigned char ofr_cfg;
	unsigned char rawdata[2];

	if(cali_num == 0)
	{
		ofr_cfg = I2C_read_reg(0x07);
	}
	else
	{
		ofr_cfg = old_ofr_cfg;
	}
	
	rawdata[0] = I2C_read_reg(0x20);
	rawdata[1] = I2C_read_reg(0x21);

	rawdata_sum = (cali_cnt==0)? (0) : (rawdata_sum + (rawdata[0]<<8) | rawdata[1]);

	if(cali_cnt==2)
	{
		if((rawdata_sum>>1) < CALI_RAW_MIN)
		{
			if((ofr_cfg&0x1F) == 0x00)
			{
				cali_dir = 0;
			}
			else
			{
				cali_dir = 2;
				ofr_cfg = ofr_cfg -1;
			}
		}
		else if ((rawdata_sum>>1) > CALI_RAW_MAX)
		{
			if((ofr_cfg&0x1F) == 0x1F)
			{
				cali_dir = 0;
			}
			else
			{
				cali_dir = 1;
				ofr_cfg = ofr_cfg + 1;
			}
		}
		else
		{
			cali_dir = 0;
		}

		if(cali_num > 0)
		{
			if(cali_dir != old_cali_dir)
			{
				cali_dir = 0;
				ofr_cfg = old_ofr_cfg;
			}
		}
		
		cali_flag = 0;
		if(cali_dir != 0)
		{
			cali_flag = 1;
		}
		

		if((cali_flag==0) && (cali_num==0))
		{
			cali_used = 0;
		}
		else
		{
			cali_used = 1;
		}
		
		if(cali_flag == 0)
		{
			cali_num = 0;
			cali_cnt = 0;
			return 0;
		}
		

		I2C_write_reg(0x07, ofr_cfg);
		I2C_write_reg(0x01, 0x00);
		I2C_write_reg(0x01, 0x02);

		if(cali_num == (CALI_NUM -1))	// no calibration
		{
			cali_flag = 0;
			cali_num = 0;
			cali_cnt = 0;
			
			return 0;
		}
		
		old_cali_dir = cali_dir;
		old_ofr_cfg = ofr_cfg;
	
		cali_num ++;
	}

	if(cali_cnt < 2)
	{
		cali_cnt ++;	
	}
	else
	{
		cali_cnt = 0;
	}
	
	return 1;
}
#endif
/*oujiacheng@wind-mobi.com 2015.7.10 add end*/
#endif
/*oujiacheng@wind-mobi.com 2015.5.19 add end*/
//////////////////////////////////////////////////////////////////////
// AW9201 initial register @ mobile active
//////////////////////////////////////////////////////////////////////
/*oujiacheng@wind-mobi.com 2015.7.10 add begin*/
#ifdef CONFIG_MTK_TOUCHKEY_AW9201_COMPATIBLE
static void AW_NormalMode(void)
{
	unsigned int i;
	unsigned int gpio_id1=0;
	mt_set_gpio_mode(GPIO99| 0x80000000,GPIO_MODE_00);    
	mt_set_gpio_dir(GPIO99 | 0x80000000, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO99| 0x80000000, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO99| 0x80000000,GPIO_PULL_DOWN);
	gpio_id1 =  mt_get_gpio_in(GPIO99 | 0x80000000);  // get signal
	TS_DBG("AW9201 gpio_id1=%d\n",gpio_id1);
	I2C_write_reg(0x00, 0x55);  // reset chip
	I2C_write_reg(0x01, 0x00);  // GCR (disable chip)
	/*oujiacheng@wind-mobi.com 2015.5.19 modify begin*/	
	if(gpio_id1 == 1)
	{
	I2C_write_reg(0x03, 0xB9);  // Set thr 
	I2C_write_reg(0x04, 0x07);  // Set thr 
	I2C_write_reg(0x05, 0x06);  // Clr thr
	I2C_write_reg(0x06, 0x41);  // Sensitivity /// 0x32 -> 0x31 scan timer
	I2C_write_reg(0x07, 0x13);  // Offset
	}
	else
	{
	I2C_write_reg(0x04, 0x05);  // Set thr 
	I2C_write_reg(0x05, 0x04);  // Clr thr
	I2C_write_reg(0x06, 0x31);  // Sensitivity /// 0x32 -> 0x31 scan timer
	I2C_write_reg(0x07, 0x12);  // Offset
	}
	I2C_write_reg(0x08, 0x40);  // Detect time //oujiacheng@wind-mobi.com 2015.5.27    0x00->0x40: touchkey release when long-press 15s 
	I2C_write_reg(0x0B, 0x08);  // BLDTH
#ifdef AW_AUTO_CALI	
	I2C_write_reg(0x0D, 0x10);  // Frame Interrupt 
#endif
	I2C_write_reg(0x0E, 0x01);  // ACFG1
	/*oujiacheng@wind-mobi.com 2015.5.19 modify end*/
	I2C_write_reg(0x0F, 0x08);  // ACFG2
	I2C_write_reg(0x2B, 0x4D);  // DSP  0x4C -> 0x4D  
#ifdef AW_AUTO_CALI	
	I2C_write_reg(0x01, 0x02);  // GCR //oujiacheng@wind-mobi.com 2015.5.19 add 
#else
	I2C_write_reg(0x01, 0x06);  // GCR
#endif	

 	WorkMode = 2;
	printk("AW9201 enter Normal mode\n");

}
#else
static void AW_NormalMode(void)
{
	unsigned int i;
	I2C_write_reg(0x00, 0x55);  // reset chip
	I2C_write_reg(0x01, 0x00);  // GCR (disable chip)
	/*oujiacheng@wind-mobi.com 2015.5.19 modify begin*/
	I2C_write_reg(0x04, 0x05);  // Set thr  //oujiacheng@wind-mobi.com 20150909 revert  0x04->5,0x05->4
	I2C_write_reg(0x05, 0x04);  // Clr thr
	I2C_write_reg(0x06, 0x31);  // Sensitivity /// 0x32 -> 0x31 scan timer
	I2C_write_reg(0x07, 0x12);  // Offset
	I2C_write_reg(0x08, 0x40);  // Detect time //oujiacheng@wind-mobi.com 2015.5.27    0x00->0x40: touchkey release when long-press 15s 
	I2C_write_reg(0x0B, 0x08);  // BLDTH
#ifdef AW_AUTO_CALI	
	I2C_write_reg(0x0D, 0x10);  // Frame Interrupt 
#endif
	I2C_write_reg(0x0E, 0x01);  // ACFG1
	/*oujiacheng@wind-mobi.com 2015.5.19 modify end*/
	I2C_write_reg(0x0F, 0x08);  // ACFG2
	I2C_write_reg(0x2B, 0x4D);  // DSP  0x4C -> 0x4D  
#ifdef AW_AUTO_CALI	
	I2C_write_reg(0x01, 0x02);  // GCR //oujiacheng@wind-mobi.com 2015.5.19 add 
#else
	I2C_write_reg(0x01, 0x06);  // GCR
#endif	

 	WorkMode = 2;
	printk("AW9201 enter Normal mode\n");

}
#endif
/*oujiacheng@wind-mobi.com 2015.7.10 add end*/

//////////////////////////////////////////////////////////////////////
// AW9201 initial register @ mobile sleep
//////////////////////////////////////////////////////////////////////
static void AW_SleepMode(void)
{
	unsigned int i;
	I2C_write_reg(0x00, 0x55);  // reset chip
	I2C_write_reg(0x01, 0x00);  // disable chip

	WorkMode = 1;
	printk("AW9201 enter Sleep mode\n");
}



/////////////////////////////////////////////
// report HOME-BTN single click 
/////////////////////////////////////////////
static void AW_center_press(void)
{
	TS_DBG("AW9201 center press \n");
	input_report_key(AW9201_ts->input_dev, KEY_FINGERPRINT, 1);
	input_sync(AW9201_ts->input_dev);


}
/////////////////////////////////////////////
// report HOME-BTN single click 
/////////////////////////////////////////////
static void AW_center_release(void)
{
	
	input_report_key(AW9201_ts->input_dev, KEY_FINGERPRINT, 0);
	input_sync(AW9201_ts->input_dev);
	TS_DBG("AW9201 center release \n");

}
////////////////////////////////////////////////////
//
// Function : Cap-touch main program @ mobile sleep 
//            wake up after double-click/right_slip/left_slip
//
////////////////////////////////////////////////////
static void AW_SleepMode_Proc(void)
{
	unsigned char buf;

	
	if(debug_level == 0)
	{
		buf=I2C_read_reg(0x02);			//read gesture interupt status
		if(buf & 0x08)
		{
			if(buf & 0x02)
			{
				AW_center_press();
			}
			else
			{
				AW_center_release();
			}		
		}
	}
}

////////////////////////////////////////////////////
//
// Function : Cap-touch main pragram @ mobile normal state
//            press/release
//
////////////////////////////////////////////////////
static int DOWN_STATUS=0; //xuecheng@wind-mobi.com add for touchkey 20160615
static void AW_NormalMode_Proc(void)
{
	unsigned char buf;
/*oujiacheng@wind-mobi.com 2015.5.19 add begin*/
#ifdef AW_AUTO_CALI	
	if(cali_flag)
	{
		AW92xx_Auto_Cali();
		if(cali_flag == 0)
		{	
			I2C_write_reg(0x0D,0x00);	// Frame Interrupt			
			I2C_write_reg(0x01,0x00);	// disable chip
			I2C_write_reg(0x01,0x06);	// enable chip
		}
		return ;
	}
#endif
/*oujiacheng@wind-mobi.com 2015.5.19 add end*/
	if(debug_level == 0)
	{
		buf=I2C_read_reg(0x02);			//read gesture interupt status
		printk("aw9201 status 0x02 = %x\n",buf);
		if(buf & 0x08)
		{
			if(buf & 0x02)
			{
			    DOWN_STATUS=1;//xuecheng@wind-mobi.com add for touchkey 20160615
				AW_center_press();
			}
			else
			{
			    DOWN_STATUS=0;//xuecheng@wind-mobi.com add for touchkey 20160615
				AW_center_release();
			}		
		}
	}


}

#ifdef AW9201_EINT_SUPPORT

static int AW9201_ts_clear_intr(struct i2c_client *client) 
{
	int res;

#ifdef AW_AUTO_CALI	
	I2C_read_reg(0x2D);			// clear Frame Interrupt //oujiacheng@wind-mobi.com 2015.5.19 add 
#endif
	res = I2C_read_reg(0x02);
	if(res < 0)
	{
		goto EXIT_ERR;
	}
	else
	{
		res = 0;
	}

	return res;

EXIT_ERR:
	TS_DBG("AW9201_ts_clear_intr fail\n");
	return 1;
}

////////////////////////////////////////////////////
//
// Function : Interrupt sub-program
//            work in AW_SleepMode_Proc() or 
//            AW_NormalMode_Proc()
//
////////////////////////////////////////////////////
static void AW9201_ts_eint_work(struct work_struct *work)
{
	//printk("AW9201 Eint work \n");
	//if(debug_level == 0)
	{
		switch(WorkMode)
		{
			case 1:
			AW_SleepMode_Proc();
			break;
			
			case 2:
			AW_NormalMode_Proc();
			break;
			
			default:
			break;
		}
	
	}
	AW9201_ts_clear_intr(this_client);
	mt_eint_unmask(AW9201_EINT_NUM); 
}


void AW9201_ts_eint_func(void)
{	
	if(AW9201_ts == NULL)
	{
		return;
	}

	schedule_work(&AW9201_ts->eint_work);

}

int AW9201_ts_setup_eint(void)
{
	mt_set_gpio_mode(AW9201_EINT_PIN, GPIO_CP281_EINT_PIN_M_GPIO);
	mt_set_gpio_dir(AW9201_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(AW9201_EINT_PIN, 1);
	mt_set_gpio_pull_enable(AW9201_EINT_PIN, GPIO_PULL_UP);
	
    /*mt65xx_eint_set_sens(AW9201_EINT_NUM, 1);//level
    mt65xx_eint_set_polarity(AW9201_EINT_NUM, CUST_EINT_POLARITY_LOW);
    mt65xx_eint_set_hw_debounce(AW9201_EINT_NUM, 0);
    mt65xx_eint_registration(AW9201_EINT_NUM, 0, CUST_EINT_POLARITY_LOW, AW9201_ts_eint_func, 0);
    mt65xx_eint_unmask(AW9201_EINT_NUM);*/
  
    mt_eint_set_hw_debounce(AW9201_EINT_NUM, 0);
    mt_eint_registration(AW9201_EINT_NUM, EINTF_TRIGGER_LOW, AW9201_ts_eint_func, 0);
    mt_eint_unmask(AW9201_EINT_NUM);

    return 0;
}
#else

static void AW9201_ts_work(struct work_struct *work)
{
	switch(WorkMode)
	{
		case 1:
		AW_SleepMode_Proc();
		break;
		
		case 2:
		AW_NormalMode_Proc();
		break;
		
		default:
		break;
	}
	
}

void AW9201_tpd_polling(unsigned long unuse)
{
    struct AW9201_ts_data *data = i2c_get_clientdata(this_client);

    if (!work_pending(&data->pen_event_work)) 
	{
    	queue_work(data->ts_workqueue, &data->pen_event_work);
    }
	data->touch_timer.expires = jiffies + HZ/FRAME_RATE;
	add_timer(&data->touch_timer);
}
#endif

////////////////////////////////////////////////////
//
// Function : AW9201 initial @ mobile goto sleep mode
//            enter SleepMode
//
////////////////////////////////////////////////////
static void AW9201_ts_suspend(struct early_suspend *handler)
{
   
    
	struct AW9201_ts_data *data = i2c_get_clientdata(this_client);

		if(WorkMode != 1)
		{
		//xuecheng@wind-mobi.com add for touchkey 20160615
			if(DOWN_STATUS==1)
			{
		     AW_center_release();
			 DOWN_STATUS=0;
			}
		//xuecheng@wind-mobi.com add for touchkey 20160615		
			AW_SleepMode();
			suspend_flag = 1;		
		}
	   	printk("==AW9201_ts_suspend=\n");
#ifndef AW9201_EINT_SUPPORT
		del_timer(&data->touch_timer);
#endif

}

////////////////////////////////////////////////////
//
// Function : AW9201 initial @ mobile wake up
//            enter NormalMode 
//
////////////////////////////////////////////////////
static void AW9201_ts_resume(struct early_suspend *handler)
{	
	struct AW9201_ts_data *data = i2c_get_clientdata(this_client);

	if(WorkMode != 2)
	{
		AW_NormalMode();
		suspend_flag = 0;
/*oujiacheng@wind-mobi.com 2015.5.19 add begin*/		
#ifdef AW_AUTO_CALI
		cali_flag = 1;
		cali_num = 0;
		cali_cnt = 0;
#endif	
/*oujiacheng@wind-mobi.com 2015.5.19 add end*/		
	}
	printk("AW9201 WAKE UP!!!");
#ifndef AW9201_EINT_SUPPORT
	data->touch_timer.expires = jiffies + HZ*5;
	add_timer(&data->touch_timer);
#endif
}

////////////////////////////////////////////////////
//
// Function : AW9201 initial @ mobile power on
//            enter NormalMode directly
//
////////////////////////////////////////////////////
static int AW9201_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	struct input_dev *input_dev;
	int err = 0;
	unsigned char reg_value,reg_value1; 
	
	TS_DBG("==AW9201_ts_probe=\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	TS_DBG("==kzalloc=");
	AW9201_ts = kzalloc(sizeof(*AW9201_ts), GFP_KERNEL);
	if (!AW9201_ts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	AW9201_ts_config_pins();

	client->addr = 0x45;								// chip  I2C address
	client->timing= 400;
	this_client = client;
	i2c_set_clientdata(client, AW9201_ts);
	//sc8810_i2c_set_clk(2,500000);

	TS_DBG("I2C addr=%x", client->addr);
	
	reg_value = I2C_read_reg(0x00);				//read chip ID
    TS_DBG("AW9201 chip ID = 0x%4x", reg_value);

	if(reg_value != 0x33)
	{
		err = -ENODEV;
		goto exit_create_singlethread;
	}

	
#ifdef AW9201_EINT_SUPPORT
		INIT_WORK(&AW9201_ts->eint_work, AW9201_ts_eint_work);
#else
		INIT_WORK(&AW9201_ts->pen_event_work, AW9201_ts_work);
	
		AW9201_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
		if (!AW9201_ts->ts_workqueue) {
			err = -ESRCH;
			goto exit_create_singlethread;
		}
#endif

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	
	AW9201_ts->input_dev = input_dev;


	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_SYN, input_dev->evbit);
	
	__set_bit(KEY_FINGERPRINT, input_dev->keybit);
	__set_bit(KEY_MENU, input_dev->keybit);
	__set_bit(KEY_BACK, input_dev->keybit);

	input_dev->name		= AW9201_ts_NAME;		//dev_name(&client->dev)
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"AW9201_ts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

TS_DBG("==register_early_suspend =");
	AW9201_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
	AW9201_ts->early_suspend.suspend = AW9201_ts_suspend;
	AW9201_ts->early_suspend.resume	= AW9201_ts_resume;
	register_early_suspend(&AW9201_ts->early_suspend);
	

    msleep(50);
    
	/* liukun@wind-mobi.com 20150116 begin */    
	sysfs_create_group(&input_dev->dev.kobj,
            &aw9201_attribute_group);
	/* liukun@wind-mobi.com 20150116 end */    
	WorkMode = 2;
	AW_NormalMode();
/*oujiacheng@wind-mobi.com 2015.5.19 add begin*/
#ifdef AW_AUTO_CALI
	cali_flag = 1;
	cali_num = 0;
	cali_cnt = 0;
#endif	
/*oujiacheng@wind-mobi.com 2015.5.19 add end*/
	reg_value1 = I2C_read_reg(0x01);
 	printk("AW9201 GCR = 0x%2x", reg_value1);

#ifdef AW9201_EINT_SUPPORT
   AW9201_ts_setup_eint();
#else
    AW9201_ts->touch_timer.function = AW9201_tpd_polling;
	AW9201_ts->touch_timer.data = 0;
	init_timer(&AW9201_ts->touch_timer);
	AW9201_ts->touch_timer.expires = jiffies + HZ*5;
	add_timer(&AW9201_ts->touch_timer);	
#endif

	TS_DBG("==probe over =\n");
    return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	//free_irq(client->irq, AW9201_ts);
#ifdef AW9201_EINT_SUPPORT
	cancel_work_sync(&AW9201_ts->eint_work);
#else
	cancel_work_sync(&AW9201_ts->pen_event_work);
	destroy_workqueue(AW9201_ts->ts_workqueue);
#endif
exit_create_singlethread:
	TS_DBG("==singlethread error =\n");
	i2c_set_clientdata(client, NULL);
	kfree(AW9201_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	//sprd_free_gpio_irq(AW9201_ts_setup.irq);
	return err;
}
/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static int  AW9201_ts_remove(struct i2c_client *client)
{

	struct AW9201_ts_data *AW9201_ts = i2c_get_clientdata(client);

	TS_DBG("==AW9201_ts_remove=\n");
	
	unregister_early_suspend(&AW9201_ts->early_suspend);
	input_unregister_device(AW9201_ts->input_dev);

	//cancel_work_sync(&AW9201_ts->pen_event_work);
	//destroy_workqueue(AW9201_ts->ts_workqueue);
	
	kfree(AW9201_ts);
	
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id AW9201_ts_id[] = {
	{ AW9201_ts_NAME, 0 },{ }
};


MODULE_DEVICE_TABLE(i2c, AW9201_ts_id);

static struct i2c_board_info __initdata AW9201_i2c_led[] = {
	{
		I2C_BOARD_INFO(AW9201_ts_NAME, AW9201_ts_I2C_ADDR)	/* 0x2c */
	}
};

static struct i2c_driver AW9201_ts_driver = {
	.probe		= AW9201_ts_probe,
	.remove		= AW9201_ts_remove,
	.id_table	= AW9201_ts_id,
	//.suspend    = AW9201_ts_suspend,
	//.resume     = AW9201_ts_resume,
	.driver	= {
		.name	= AW9201_ts_NAME,
		.owner	= THIS_MODULE,
	},
};
static int AW_nvram_read(char *filename, char *buf, ssize_t len, int offset)
{	
    struct file *fd;
    //ssize_t ret;
    int retLen = -1;
    
    mm_segment_t old_fs = get_fs();
    set_fs(KERNEL_DS);
    
    fd = filp_open(filename, O_RDONLY, 0);
    
    if(IS_ERR(fd)) {
        printk("[AW9201][nvram_read] : failed to open!!\n");
        return -1;
    }
    do{
        if ((fd->f_op == NULL) || (fd->f_op->read == NULL))
    		{
            printk("[AW9201][nvram_read] : file can not be read!!\n");
            break;
    		} 
    		
        if (fd->f_pos != offset) {
            if (fd->f_op->llseek) {
        		    if(fd->f_op->llseek(fd, offset, 0) != offset) {
						printk("[AW9201][nvram_read] : failed to seek!!\n");
					    break;
        		    }
        	  } else {
        		    fd->f_pos = offset;
        	  }
        }    		
        
    		retLen = fd->f_op->read(fd,
    									  buf,
    									  len,
    									  &fd->f_pos);			
    		
    }while(false);
    
    filp_close(fd, NULL);
    
    set_fs(old_fs);
    
    return retLen;
}

static int AW_nvram_write(char *filename, char *buf, ssize_t len, int offset)
{	
    struct file *fd;
    //ssize_t ret;
    int retLen = -1;
        
    mm_segment_t old_fs = get_fs();
    set_fs(KERNEL_DS);
    
    fd = filp_open(filename, O_WRONLY|O_CREAT, 0666);
    
    if(IS_ERR(fd)) {
        printk("[AW9201][nvram_write] : failed to open!!\n");
        return -1;
    }
    do{
        if ((fd->f_op == NULL) || (fd->f_op->write == NULL))
    		{
            printk("[AW9201][nvram_write] : file can not be write!!\n");
            break;
    		} /* End of if */
    		
        if (fd->f_pos != offset) {
            if (fd->f_op->llseek) {
        	    if(fd->f_op->llseek(fd, offset, 0) != offset) {
				    printk("[AW9201][nvram_write] : failed to seek!!\n");
                    break;
                }
            } else {
                fd->f_pos = offset;
            }
        }       		
        
        retLen = fd->f_op->write(fd,
                                 buf,
                                 len,
                                 &fd->f_pos);			
    		
    }while(false);
    
    filp_close(fd, NULL);
    
    set_fs(old_fs);
    
    return retLen;
}
#if 0

//////////////////////////////////////////////////////////////////////
/* 	MIN(x1,x2,x3): min val in x1,x2,x3                              */
//////////////////////////////////////////////////////////////////////
unsigned int MIN(unsigned int x1, unsigned int x2, unsigned int x3)
{
	unsigned int min_val;
	
	if(x1>=x2) 
	{
		if(x2>=x3)
			min_val = x3;
		else
			min_val = x2;
	}
	else      
	{
		if(x1>=x3)
			min_val = x3;
		else
			min_val = x1;
	}

	return min_val;
}
#endif



static ssize_t AW9201_show_debug(struct device* cd,struct device_attribute *attr, char* buf)
{
	ssize_t ret = 0;
	
	sprintf(buf, "AW9201 Debug %d\n",debug_level);
	
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t AW9201_store_debug(struct device* cd, struct device_attribute *attr,
		       const char* buf, size_t len)
{
	unsigned long on_off = simple_strtoul(buf, NULL, 10);
	debug_level = on_off;

	TS_DBG("%s: debug_level=%d\n",__func__, debug_level);
	
	return len;
}



static ssize_t AW9201_get_reg(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned char reg_val[1];
	ssize_t len = 0;
	u8 i;
	mt_eint_mask(AW9201_EINT_NUM);
	for(i=1;i<0x2D;i++)
	{
		reg_val[0] = I2C_read_reg(i);
		len += snprintf(buf+len, PAGE_SIZE-len, "reg%2X = 0x%2X, ", i,reg_val[0]);
	}
	mt_eint_unmask(AW9201_EINT_NUM);
	return len;

}

static ssize_t AW9201_write_reg(struct device* cd, struct device_attribute *attr,
		       const char* buf, size_t len)
{

	unsigned int databuf[2];
mt_eint_mask(AW9201_EINT_NUM);
	if(2 == sscanf(buf,"%x %x",&databuf[0], &databuf[1]))
	{
		I2C_write_reg((u8)databuf[0],(u8)databuf[1]);
	}
mt_eint_unmask(AW9201_EINT_NUM);
	return len;
}

static ssize_t AW9201_get_Base(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned int baseline;
	ssize_t len = 0;
	mt_eint_mask(AW9201_EINT_NUM);
	len += snprintf(buf+len, PAGE_SIZE-len, "baseline: \n");

	buf[0]=I2C_read_reg(0x26);
	buf[1]=I2C_read_reg(0x27);
	baseline = (unsigned int)((buf[0]<<8) | buf[1]);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",baseline);
	
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	mt_eint_unmask(AW9201_EINT_NUM);
	return len;
}

static ssize_t AW9201_get_rawdata(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned int rawdata;
	ssize_t len = 0;
	mt_eint_mask(AW9201_EINT_NUM);
	len += snprintf(buf+len, PAGE_SIZE-len, "rawdata: \n");

	buf[0]=I2C_read_reg(0x20);
	buf[1]=I2C_read_reg(0x21);
	rawdata = (unsigned int)((buf[0]<<8) | buf[1]);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",rawdata);
	
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	mt_eint_unmask(AW9201_EINT_NUM);
	return len;
}

static ssize_t AW9201_get_delta(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned int delta;
	ssize_t len = 0;
	mt_eint_mask(AW9201_EINT_NUM);
	len += snprintf(buf+len, PAGE_SIZE-len, "delta: \n");

	buf[0]=I2C_read_reg(0x24);
	buf[1]=I2C_read_reg(0x25);
	delta = (unsigned int)((buf[0]<<8) | buf[1]);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",delta);

	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	mt_eint_unmask(AW9201_EINT_NUM);
	return len;
}

static ssize_t AW9201_get_irqstate(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned char touch,key;

	ssize_t len = 0;
	mt_eint_mask(AW9201_EINT_NUM);
	len += snprintf(buf+len, PAGE_SIZE-len, "touch: \n");

	touch=I2C_read_reg(0x02);
	if((touch&0x2) == 0x2) key=1; else key=0;
	len += snprintf(buf+len, PAGE_SIZE-len, "%d, ",key);
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	
	mt_eint_unmask(AW9201_EINT_NUM);
	return len;
}


static int AW9201_create_sysfs(struct i2c_client *client)
{
	int err;
	struct device *dev = &(client->dev);

	TS_DBG("%s", __func__);
	
	err = device_create_file(dev, &dev_attr_debug);
	err = device_create_file(dev, &dev_attr_getreg);
	err = device_create_file(dev, &dev_attr_base);
	err = device_create_file(dev, &dev_attr_rawdata);
	err = device_create_file(dev, &dev_attr_delta);
	err = device_create_file(dev, &dev_attr_getstate);
	return err;
}
/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static int __init AW9201_ts_init(void)
{
	int ret;
	printk("==AW9201_ts_init==\n");

    i2c_register_board_info(AW9201_ts_I2C_BUS, AW9201_i2c_led, 1);

    msleep(50);

	ret = i2c_add_driver(&AW9201_ts_driver);
	return ret;
}

/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static void __exit AW9201_ts_exit(void)
{
	TS_DBG("==AW9201_ts_exit==\n");
	i2c_del_driver(&AW9201_ts_driver);
}




module_init(AW9201_ts_init);
module_exit(AW9201_ts_exit);

MODULE_AUTHOR("<lijunjiea@AWINIC.com>");
MODULE_DESCRIPTION("AWINIC AW9201 Touch driver");
MODULE_LICENSE("GPL");
