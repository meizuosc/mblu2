//liqiang@wind-mobi.com created this file 20140821 begin
/* drivers/hwmon/mt6516/amit/epl2182.c - EPL2182 ALS/PS driver
 *
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
//#include <cust_alsps.h>
#include <linux/hwmsen_helper.h>
#include "epl2182.h"

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#include <linux/earlysuspend.h>
#include <linux/wakelock.h>
#include <linux/sched.h>

#include <alsps.h>
#include <epl_cust_alsps.h>
#ifdef CUSTOM_KERNEL_SENSORHUB
#include <SCP_sensorHub.h>
#endif
/******************************************************************************
 * extern functions
*******************************************************************************/
extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void);
void epl2182_restart_polling(void);
/******************************************************************************
 * configuration
*******************************************************************************/

//dixiaobing@wind-mobi.com add begin 
//#define DYN_ENABLE			0
#define SOFTWARE_INTERRUPT_ENABLE 1
#define ELAN_WRITE_CALI  1
#if defined(WIND_DEF_PRO_A110L) //Only for A110L 
// TODO: change ps/als integrationtime
int PS_INTT = 4; //default setting
int ALS_INTT = 7;
// TODO: change delay time
#define PS_DELAY 			55
#define ALS_DELAY 			55
// TODO: parameters for lux equation y = ax + b
#define LUX_PER_COUNT		400              // 1100 = 1.1 * 1000
#if DYN_ENABLE
#define DYN_H_OFFSET 	 	300//400
#define DYN_L_OFFSET		200//300
#define DYN_CONDITION		7500
#endif

#else
// For Other project 
// TODO: change ps/als integrationtime
int PS_INTT = 4; //default setting
int ALS_INTT = 7;
// TODO: change delay time
#define PS_DELAY 			55
#define ALS_DELAY 			55
// TODO: parameters for lux equation y = ax + b
#define LUX_PER_COUNT		400              // 1100 = 1.1 * 1000
#if DYN_ENABLE
//lichengmin@wind-mobi.com 20141001 begin 
#if defined(WIND_DEF_PRO_A310F) //Only for A310L 
#define DYN_H_OFFSET 	 	500//400
#define DYN_L_OFFSET		350//300
#else
#define DYN_H_OFFSET 	 	300//400
#define DYN_L_OFFSET		200//300
#endif
//lichengmin@wind-mobi.com 20141001 end 
#define DYN_CONDITION		7500
#endif

#endif
//dixiaobing@wind-mobi.com add end 

#ifdef ELAN_WRITE_CALI
typedef struct _epl_ps_als_factory
{
    bool cal_file_exist;
    bool cal_finished;
    u16 ps_cal_h;
    u16 ps_cal_l;
    char s1[16];
    char s2[16];
    u16 cal_ps_raw;
};
#endif


#define PS_DRIVE EPL_DRIVE_120MA
#define TXBYTES 				2
#define RXBYTES 				2
#define PACKAGE_SIZE 			2
#define I2C_RETRY_COUNT 		3
#define IPI_WAIT_RSP_TIMEOUT    (HZ/10)     //100ms

static DEFINE_MUTEX(epl2182_mutex);

//liqiang@wind-mobi.com 20140924 add begin 
static struct wake_lock ps_lock; //1011

#if CHECK_CT_ENABLE
#define CT_H_TH  600
#define CT_L_TH  300

typedef struct _wind_check_ct 
{
	u16 ct_low_threshold;
	u16 ct_high_threshold;
	u16 ct_min_val;
	u16 check_enable;
}wind_check_ct_des;

static wind_check_ct_des wind_check_ct;
#endif 
//liqiang@wind-mobi.com 20140924 add end

typedef struct _epl_raw_data
{
    u8 raw_bytes[PACKAGE_SIZE];
    int ps_raw;
    u16 ps_state;
	
#if defined(DYN_ENABLE) || defined(ELAN_WRITE_CALI) 
	u16 ps_condition;
	u16 ps_min_raw;
	u16 ps_sta;
	int ps_dyn_high;
	int ps_dyn_low;
	bool ps_dny_ini_lock;
#endif
    u16 ps_int_state;
    u16 als_ch0_raw;
    u16 als_ch1_raw;
	u16 als_lux;
	bool ps_suspend_flag;
#ifdef ELAN_WRITE_CALI
    struct _epl_ps_als_factory ps_als_factory;
#endif
} epl_raw_data;

#ifdef ELAN_WRITE_CALI
#define PS_CAL_FILE_PATH	"/data/misc/sensor/ps-cali.txt"//"/data/data/com.eminent.ps.calibration/xtalk_cal"  //PS Calbration file path
static int PS_h_offset = 410;//3000;
static int PS_l_offset = 360;//2000;
static int PS_MAX_XTALK = 2000;//50000;
static int ps_enable_value = 0;
#endif

#define EPL2182_DEV_NAME     "EPL2182"


/*----------------------------------------------------------------------------*/

#define APS_TAG                  "[ALS/PS] "
#if 0
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_DEBUG APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_DEBUG fmt, ##args)
#else
#define APS_FUN(f)
#define APS_ERR(fmt, args...)
#define APS_LOG(fmt, args...)
#define APS_DBG(fmt, args...)
#endif
#define FTM_CUST_ALSPS "/data/epl2182"

#define POWER_NONE_MACRO MT65XX_POWER_NONE

static struct i2c_client *epl2182_i2c_client = NULL;

#if defined(DYN_ENABLE) || defined(ELAN_WRITE_CALI)
static DEFINE_MUTEX(sensor_mutex);
#endif

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id epl2182_i2c_id[] = {{"EPL2182",0},{}};
static struct i2c_board_info __initdata i2c_EPL2182= { I2C_BOARD_INFO("EPL2182", (0X92>>1))};

/*----------------------------------------------------------------------------*/
static int epl2182_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int epl2182_i2c_remove(struct i2c_client *client);
static int epl2182_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);

static int alsps_local_init(void);
static int alsps_remove(void);
/*----------------------------------------------------------------------------*/
static int epl2182_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int epl2182_i2c_resume(struct i2c_client *client);
#ifndef CUSTOM_KERNEL_SENSORHUB
static void epl2182_eint_func(void);
#endif
static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd);
static int set_psensor_threshold(struct i2c_client *client);

static struct epl2182_priv *g_epl2182_ptr = NULL;
static int isInterrupt = false;

static void polling_do_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(polling_work, polling_do_work);
#ifndef CUSTOM_KERNEL_SENSORHUB
static long long int_top_time = 0;
static int int_flag = 0;
#endif


/*----------------------------------------------------------------------------*/
typedef enum
{
    CMC_TRC_ALS_DATA = 0x0001,
    CMC_TRC_PS_DATA = 0X0002,
    CMC_TRC_EINT    = 0x0004,
    CMC_TRC_IOCTL   = 0x0008,
    CMC_TRC_I2C     = 0x0010,
    CMC_TRC_CVT_ALS = 0x0020,
    CMC_TRC_CVT_PS  = 0x0040,
    CMC_TRC_DEBUG   = 0x0800,
} CMC_TRC;

/*----------------------------------------------------------------------------*/
typedef enum
{
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;

/*----------------------------------------------------------------------------*/
struct epl2182_i2c_addr      /*define a series of i2c slave address*/
{
    u8  write_addr;
    u8  ps_thd;     /*PS INT threshold*/
};

/*----------------------------------------------------------------------------*/
struct epl2182_priv
{
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct work_struct  eint_work;
	struct work_struct data_work;
	 struct workqueue_struct *epl_wq;
#ifdef CUSTOM_KERNEL_SENSORHUB
    struct work_struct init_done_work;
#endif
    /*i2c address group*/
    struct epl2182_i2c_addr  addr;

    int enable_pflag;
    int enable_lflag;

    /*misc*/
    atomic_t    trace;
    atomic_t    i2c_retry;
    atomic_t    als_suspend;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;

    /*data*/
    u16         als;
    u16         ps;
    u16		lux_per_count;
    bool   		als_enable;    /*record current als status*/
    bool    	ps_enable;     /*record current ps status*/
    ulong       enable;         /*record HAL enalbe status*/
    ulong       pending_intr;   /*pending interrupt*/
    //ulong        first_read;   // record first read ps and als

    /*data*/
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];
	int			ps_cali;

	atomic_t	ps_thd_val_high;	 /*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val_low; 	/*the cmd value can't be read, stored in ram*/
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif
};
/*dixiaobing@wind-mobi.com 20150525 start*/
#ifdef CONFIG_SENSOR_NON_WAKE_UP
int als_ps_touch =0;
extern int non_wakeup_ps;
int non_wakeup_ps_suspend =0;
int non_wakeup_ps_flags =0;
#endif
/*dixiaobing@wind-mobi.com 20150525 end*/

/*dixiaobing@wind-mobi.com 20150617 start*/
#ifdef ELAN_WRITE_CALI
#include <linux/meizu.h>
static int ps_calibbias_value = 0;
static struct meizu_classdev *psensor = NULL;
static void meizu_ps_node_init(void);
static void meizu_ps_node_uninit(void);
#endif
/*dixiaobing@wind-mobi.com 20150617 end*/

/*----------------------------------------------------------------------------*/
static struct i2c_driver epl2182_i2c_driver =
{
    .probe      = epl2182_i2c_probe,
    .remove     = epl2182_i2c_remove,
    .detect     = epl2182_i2c_detect,
    .suspend    = epl2182_i2c_suspend,
    .resume     = epl2182_i2c_resume,
    .id_table   = epl2182_i2c_id,
    .driver = {
        .name           = EPL2182_DEV_NAME,
    },
};


static struct epl2182_priv *epl2182_obj = NULL;
static epl_raw_data	gRawData;

static int alsps_init_flag =-1; // 0<==>OK -1 <==> fail

static struct alsps_init_info epl2182_init_info = {
		.name = EPL2182_DEV_NAME,
		.init = alsps_local_init,
		.uninit = alsps_remove,

};

static DECLARE_WAIT_QUEUE_HEAD(wait_rsp_wq);

static atomic_t wait_rsp_flag = ATOMIC_INIT(0);

//static struct wake_lock als_lock; /* Bob.chen add for if ps run, the system forbid to goto sleep mode. */

/*
//====================I2C write operation===============//
//regaddr: ELAN epl2182 Register Address.
//bytecount: How many bytes to be written to epl2182 register via i2c bus.
//txbyte: I2C bus transmit byte(s). Single byte(0X01) transmit only slave address.
//data: setting value.
//
// Example: If you want to write single byte to 0x1D register address, show below
//	      elan_epl2182_I2C_Write(client,0x1D,0x01,0X02,0xff);
//
*/
static int elan_epl2182_I2C_Write(struct i2c_client *client, uint8_t regaddr, uint8_t bytecount, uint8_t txbyte, uint8_t data)
{
    uint8_t buffer[2];
    int ret = 0;
    int retry;

    //APS_ERR("[ELAN epl2182] %s\n", __func__); //wenggaojian@wind-mobi.com 20150702
	mutex_lock(&epl2182_mutex);
    buffer[0] = (regaddr<<3) | bytecount ;
    buffer[1] = data;


    //APS_ERR("---elan_epl2182_I2C_Write register (0x%x) buffer data (%x) (%x)---\n",regaddr,buffer[0],buffer[1]);  //wenggaojian@wind-mobi.com 20150617
    //APS_ERR("wenggaojian %s\n", __func__); //wenggaojian@wind-mobi.com 20150702
    for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
    {
        ret = i2c_master_send(client, buffer, txbyte);
        if (ret >= 0)
        {
            break;
        }

        APS_ERR("epl2182 i2c write error,TXBYTES %d\r\n",ret);
        mdelay(10);
    }

    if(retry>=I2C_RETRY_COUNT)
    {
    	mutex_unlock(&epl2182_mutex);
        APS_ERR(KERN_ERR "[ELAN epl2182 error] %s i2c write retry over %d\n",__func__, I2C_RETRY_COUNT);
        return -EINVAL;
    }
	mutex_unlock(&epl2182_mutex);
    return ret;
}




/*
//====================I2C read operation===============//
*/
static int elan_epl2182_I2C_Read(struct i2c_client *client, uint8_t regaddr, uint8_t bytecount, uint8_t rxbyte, uint8_t *data)
{
    uint8_t buffer[RXBYTES];
    int ret = 0, i =0;
    int retry;

    //APS_DBG("[ELAN epl2182] %s\n", __func__);
	mutex_lock(&epl2182_mutex);
	buffer[0] = (regaddr<<3) | bytecount ;

    for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
    {
    	ret = hwmsen_read_block(client, buffer[0], buffer, rxbyte);
        if (ret >= 0)
            break;

        APS_ERR("epl2182 i2c read error,RXBYTES %d\r\n",ret);
        mdelay(10);
    }

    if(retry>=I2C_RETRY_COUNT)
    {
        APS_ERR(KERN_ERR "[ELAN epl2182 error] %s i2c read retry over %d\n",__func__, I2C_RETRY_COUNT);
		mutex_unlock(&epl2182_mutex);
        return -EINVAL;
    }

    for(i=0; i<PACKAGE_SIZE; i++)
        *data++ = buffer[i];
	mutex_unlock(&epl2182_mutex);
    //APS_DBG("----elan_epl2182_I2C_Read Receive data from (0x%x):byte1 (%x) byte2 (%x)-----\n",regaddr, buffer[0], buffer[1]);

    return ret;
}

#ifdef ELAN_WRITE_CALI
static int write_factory_calibration(struct epl2182_priv *epl_data, char* ps_data, int ps_cal_len)
{
    struct file *fp_cal;

	mm_segment_t fs;
	loff_t pos;

	APS_FUN();
    pos = 0;

	fp_cal = filp_open(PS_CAL_FILE_PATH, O_CREAT|O_RDWR|O_TRUNC, 0777);
	if (IS_ERR(fp_cal))
	{
		APS_ERR("[ELAN]create file error\n");
		return -1;
	}

    fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_write(fp_cal, ps_data, ps_cal_len, &pos);

    filp_close(fp_cal, NULL);

	set_fs(fs);

	return 0;
}

static bool read_factory_calibration(struct epl2182_priv *epl_data)
{
	struct i2c_client *client = epl_data->client;
	struct file *fp;
	mm_segment_t fs;
	loff_t pos;
	char buffer[100]= {0};
	if(gRawData.ps_als_factory.cal_file_exist == 1)
	{
		fp = filp_open(PS_CAL_FILE_PATH, O_RDWR, /*S_IRUSR*/0777);

		if (IS_ERR(fp))
		{
			APS_ERR("NO PS calibration file(%d)\n", (int)IS_ERR(fp));
			gRawData.ps_als_factory.cal_file_exist =  0;
			return -EINVAL;
		}
		else
		{
		    int ps_hthr = 0, ps_lthr = 0;
			pos = 0;
			fs = get_fs();
			set_fs(KERNEL_DS);
			vfs_read(fp, buffer, sizeof(buffer), &pos);
			filp_close(fp, NULL);

			sscanf(buffer, "%d,%d", &ps_hthr, &ps_lthr);
			gRawData.ps_als_factory.ps_cal_h = ps_hthr;
			gRawData.ps_als_factory.ps_cal_l = ps_lthr;
			set_fs(fs);

			epl_data->hw->ps_threshold_high = gRawData.ps_als_factory.ps_cal_h;
		    epl_data->hw->ps_threshold_low = gRawData.ps_als_factory.ps_cal_l;

		    //atomic_set(&epl_data->ps_thd_val_high, epl_data->hw->ps_threshold_high);
	        //atomic_set(&epl_data->ps_thd_val_low, epl_data->hw->ps_threshold_low);
		}

	//	gRawData.ps_als_factory.cal_finished = 0;
	}
	return 0;
}

static int elan_epl2182_psensor_enable(struct epl2182_priv *epl_data, int enable);

static int elan_run_calibration(struct epl2182_priv *epl_data)
{

    struct epl2182_priv *obj = epl_data;
    u16 ch1;
    u32 ch1_all=0;
    int count = 1;
    int i;
    uint8_t read_data[2];
    int ps_hthr=0, ps_lthr=0;
    int ps_cal_len = 0;
    char ps_calibration[20];
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;

    APS_FUN();

    if(!epl_data)
    {
        APS_ERR("epl2182_obj is null!!\n");
        return -EINVAL;
    }

    if(PS_MAX_XTALK < 0)
    {
        APS_ERR("Failed: PS_MAX_XTALK < 0 \r\n");
        return -EINVAL;
    }

    if(enable_ps == 1)
    {
        //set_bit(CMC_BIT_PS, &obj->enable);
        epl2182_restart_polling();
        msleep(ALS_DELAY+2*PS_DELAY+50);
    }else
    {
        return -EINVAL;
    }

    for(i=0; i<count; i++)
    {
        u16 ps_cali_raw;
        msleep(PS_DELAY);


        ps_cali_raw = gRawData.ps_raw;
	    APS_LOG("[%s]: gRawData.ps_raw=%d \r\n", __func__, gRawData.ps_raw);

		ch1_all = ch1_all+ ps_cali_raw;
    }

    ch1 = (u16)ch1_all/count;
    gRawData.ps_als_factory.cal_ps_raw = ch1;
    if(ch1 > PS_MAX_XTALK)
    {
        APS_ERR("Failed: ch1 > max_xtalk(%d) \r\n", ch1);
        return -EINVAL;
    }
    else if(ch1 <= 0)
    {
        APS_ERR("Failed: ch1 = 0\r\n");
        return -EINVAL;
    }

    ps_hthr = ch1 + PS_h_offset;
    ps_lthr = ch1 + PS_l_offset;

    ps_cal_len = sprintf(ps_calibration, "%d,%d", ps_hthr, ps_lthr);

    if(write_factory_calibration(obj, ps_calibration, ps_cal_len) < 0)
    {
        APS_ERR("[%s] create file error \n", __func__);
        return -EINVAL;
    }

    gRawData.ps_als_factory.cal_file_exist = 1;
    gRawData.ps_als_factory.cal_finished = 1;

    gRawData.ps_als_factory.ps_cal_h = ps_hthr;
    gRawData.ps_als_factory.ps_cal_l = ps_lthr;
    epl_data->hw->ps_threshold_high = ps_hthr;
    epl_data->hw->ps_threshold_low = ps_lthr;
    //atomic_set(&epl_data->ps_thd_val_high, epl_data->hw->ps_threshold_high);
    //atomic_set(&epl_data->ps_thd_val_low, epl_data->hw->ps_threshold_low);

    set_psensor_intr_threshold(epl_data->hw->ps_threshold_low,epl_data->hw->ps_threshold_high);

	APS_LOG("[%s]: ch1 = %d\n", __func__, ch1);

	return ch1;
}

#endif


#if DYN_ENABLE
static void dyn_ps_cal(struct epl2182_priv *epl_data)
{		
	if((gRawData.ps_raw < gRawData.ps_min_raw)
	&& (gRawData.ps_sta != 1)
	&& (gRawData.ps_condition <= DYN_CONDITION))  
	{
		gRawData.ps_min_raw = gRawData.ps_raw;
		gRawData.ps_dyn_low = gRawData.ps_raw + DYN_L_OFFSET;
		gRawData.ps_dyn_high = gRawData.ps_raw + DYN_H_OFFSET;
		APS_LOG("dyn ps raw = %d, min = %d, condition = %d\n", gRawData.ps_raw, gRawData.ps_min_raw, gRawData.ps_condition);
		
		set_psensor_intr_threshold(gRawData.ps_dyn_low, gRawData.ps_dyn_high);
		APS_LOG("dyn k thre_l = %d, thre_h = %d\n",gRawData.ps_dyn_low, gRawData.ps_dyn_high);		
	}
}
#endif

static int elan_epl2182_psensor_enable(struct epl2182_priv *epl_data, int enable)
{
	APS_FUN();
	int ret = 0;
	int err = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA req;
    int len;
#else
    uint8_t regdata;
	uint8_t read_data[2];
	int ps_state;
    struct i2c_client *client = epl_data->client;
#endif

    APS_LOG("[ELAN epl2182] %s enable = %d\n", __func__, enable);

#ifdef CUSTOM_KERNEL_SENSORHUB
    req.activate_req.sensorType = ID_PROXIMITY;
    req.activate_req.action = SENSOR_HUB_ACTIVATE;
    req.activate_req.enable = enable;
    len = sizeof(req.activate_req);
    ret = SCP_sensorHub_req_send(&req, &len, 1);

    //Handle delay control in SCP side.

    atomic_set(&wait_rsp_flag, 1);
    if (0 == wait_event_interruptible_timeout(wait_rsp_wq, atomic_read(&wait_rsp_flag) == 0, IPI_WAIT_RSP_TIMEOUT))
    {
        APS_ERR("Wait IPI response timeout!\n");
    }
    else
    {
        //gRawData.ps_state has been updated in eint_work.
    }
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
    epl_data->enable_pflag = enable;
    ret = elan_epl2182_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,EPL_INT_DISABLE | EPL_DRIVE_120MA);

    if(enable)
    {
        //wake_lock(&ps_lock);	
        regdata = EPL_SENSING_2_TIME | EPL_PS_MODE | EPL_M_GAIN ;
		//regdata = EPL_SENSING_2_TIME | EPL_PS_MODE | EPL_L_GAIN ; 2014-9-11
        regdata = regdata | (isInterrupt ? EPL_C_SENSING_MODE : EPL_S_SENSING_MODE);
        ret = elan_epl2182_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

        regdata = PS_INTT<<4 | EPL_PST_1_TIME | EPL_12BIT_ADC;
        ret = elan_epl2182_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02,regdata);

        //set_psensor_intr_threshold(epl_data->hw ->ps_threshold_low,epl_data->hw ->ps_threshold_high);
#ifndef DYN_ENABLE

#ifdef ELAN_WRITE_CALI
    //   if(gRawData.ps_als_factory.cal_finished == 1)
          {
		  //  ret=read_factory_calibration(epl_data);
		            epl_data->hw->ps_threshold_high = gRawData.ps_als_factory.ps_cal_h;
		            epl_data->hw->ps_threshold_low = gRawData.ps_als_factory.ps_cal_l;
		  
                    gRawData.ps_als_factory.cal_finished = 0;
          }

         //printk("dixiaobing    [ELAN epl2182] cal_finished = %d\, cal_file_exist = %d\n",gRawData.ps_als_factory.cal_finished , gRawData.ps_als_factory.cal_file_exist);
        APS_LOG("[ELAN epl2182] %s cal_finished = %d\, cal_file_exist = %d\n", __func__, gRawData.ps_als_factory.cal_finished , gRawData.ps_als_factory.cal_file_exist);
		set_psensor_intr_threshold(epl_data->hw->ps_threshold_low,epl_data->hw->ps_threshold_high);
#else
		set_psensor_threshold(client);
#endif

#endif

        ret = elan_epl2182_I2C_Write(client,REG_7,W_SINGLE_BYTE,0X02,EPL_C_RESET);
        ret = elan_epl2182_I2C_Write(client,REG_7,W_SINGLE_BYTE,0x02,EPL_C_START_RUN);
		msleep(PS_DELAY);


		elan_epl2182_I2C_Read(client, REG_13, R_SINGLE_BYTE, 0x01, read_data);
		ps_state = !((read_data[0] & 0x04) >> 2);
		int_flag = ps_state;
		gRawData.ps_sta = ((read_data[0] & 0x02) >> 1);
#ifdef ELAN_WRITE_CALI
        elan_epl2182_I2C_Read(epl_data->client,REG_16,R_TWO_BYTE,0x02,read_data);//liqiang modified
        gRawData.ps_raw = (read_data[1]<<8) | read_data[0];
#endif
#if DYN_ENABLE
		elan_epl2182_I2C_Read(epl_data->client, REG_14, R_TWO_BYTE, 0x02, read_data);
		gRawData.ps_condition = ((read_data[1] << 8) | read_data[0]);

	    elan_epl2182_I2C_Read(epl_data->client,REG_16,R_TWO_BYTE,0x02,read_data);//liqiang modified
        gRawData.ps_raw = (read_data[1]<<8) | read_data[0];

		dyn_ps_cal(epl_data);
		
		//APS_LOG("dyn k ps raw = %d, condition = %d\n, ps_state = %d",	gRawData.ps_raw, gRawData.ps_condition, ps_state);
#endif

		if (isInterrupt)
		{
			if (gRawData.ps_state != ps_state)
			{
				gRawData.ps_state = ps_state;
				APS_LOG("epl2182 ps state = %d \n", gRawData.ps_state);
/*dixiaobing@wind-mobi.com 20150525 start*/
#ifdef CONFIG_SENSOR_NON_WAKE_UP
                               //printk("dixiaobing 11111111111 non_wakeup_ps=  %d, ps_state =%d\n",non_wakeup_ps,ps_state);
                               
                               if(ps_state)
                               	{
                                    als_ps_touch =0;
                                    non_wakeup_ps_flags=ps_state;
                               	}
                               else
                               	{
                                    als_ps_touch =1;
									non_wakeup_ps_flags=ps_state;
                               	}
                              
                                if(non_wakeup_ps)
                                {
						              if(non_wakeup_ps_suspend)
							          {
							                if((err = ps_report_interrupt_data(ps_state)))
							                {
							                    APS_ERR("epl2182 call ps_report_interrupt_data fail = %d\n", err);
							                 }
							           }else
							           {
							         	     if((err = ps_report_interrupt_data(ps_state)))
							                {
							                    APS_ERR("epl2182 call ps_report_interrupt_data fail = %d\n", err);
							                 }
		         	                    }
                                }else
                                {
                                   if ((err = ps_report_interrupt_data(ps_state)))
                                        APS_ERR("epl2182 call ps_report_interrupt_data fail = %d\n", err);
                                }
#else
                   if ((err = ps_report_interrupt_data(ps_state)))
					APS_ERR("epl2182 call ps_report_interrupt_data fail = %d\n", err);
#endif
/*dixiaobing@wind-mobi.com 20150525 end*/
				APS_LOG("epl2182 eint work soft gRawData.ps_state = %d\n", gRawData.ps_state);				
				elan_epl2182_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02, EPL_INT_ACTIVE_LOW | PS_DRIVE); //wenggaojian@wind-mobi.com 20150616


			}
			else
			{
				elan_epl2182_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02, EPL_INT_ACTIVE_LOW | PS_DRIVE);
			}
		}

	}
	else
	{
	 //wake_unlock(&ps_lock);
/*dixiaobing@wind-mobi.com 20150525 start*/
#ifdef CONFIG_SENSOR_NON_WAKE_UP
                //printk("dixiaobing2222222222222 non_wakeup_ps = %d\n",non_wakeup_ps);
              //  if(non_wakeup_ps)
                {
                   als_ps_touch = 0;
                }
#endif
/*dixiaobing@wind-mobi.com 20150525 end*/
		regdata = EPL_SENSING_2_TIME | EPL_PS_MODE | EPL_L_GAIN;
		regdata = regdata | EPL_S_SENSING_MODE;
		ret = elan_epl2182_I2C_Write(client, REG_0, W_SINGLE_BYTE, 0X02, regdata);
		ret = elan_epl2182_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02, EPL_INT_DISABLE | EPL_DRIVE_120MA);//yucong add
	}
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB


   if(ret<0)
    {
        APS_ERR("[ELAN epl2182 error]%s: ps enable %d fail\n",__func__,ret);
    }
    else
    {
        ret = 0;
    }

    return ret;
}


static int elan_epl2182_lsensor_enable(struct epl2182_priv *epl_data, int enable)
{
	APS_FUN();
    int ret = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA req;
    int len;
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
    uint8_t regdata;

    struct i2c_client *client = epl_data->client;
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

    //APS_LOG("[ELAN epl2182] %s enable = %d\n", __func__, enable);

    epl_data->enable_lflag = enable;

    if(enable)
    {
#ifdef CUSTOM_KERNEL_SENSORHUB
        req.activate_req.sensorType = ID_LIGHT;
        req.activate_req.action = SENSOR_HUB_ACTIVATE;
        req.activate_req.enable = enable;
        len = sizeof(req.activate_req);
        ret = SCP_sensorHub_req_send(&req, &len, 1);
        if (ret)
        {
            APS_ERR("SCP_sensorHub_req_send!\n");
        }
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
        regdata = EPL_INT_DISABLE;
        ret = elan_epl2182_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02, regdata);


        regdata = EPL_S_SENSING_MODE | EPL_SENSING_8_TIME | EPL_ALS_MODE | EPL_AUTO_GAIN;

        ret = elan_epl2182_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

        regdata = ALS_INTT<<4 | EPL_PST_1_TIME | EPL_10BIT_ADC;
        ret = elan_epl2182_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02,regdata);

        ret = elan_epl2182_I2C_Write(client,REG_10,W_SINGLE_BYTE,0X02,0x3e);
        ret = elan_epl2182_I2C_Write(client,REG_11,W_SINGLE_BYTE,0x02,0x3e);

        ret = elan_epl2182_I2C_Write(client,REG_7,W_SINGLE_BYTE,0X02,EPL_C_RESET);
        ret = elan_epl2182_I2C_Write(client,REG_7,W_SINGLE_BYTE,0x02,EPL_C_START_RUN);
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
		msleep(ALS_DELAY);
    }


    if(ret<0)
    {
        APS_ERR("[ELAN epl2182 error]%s: als_enable %d fail\n",__func__,ret);
    }
    else
    {
        ret = 0;
    }

    return ret;
}

//convert raw to lux
static int epl2182_get_als_value(struct epl2182_priv *obj, u16 als)
{
    int idx;
    int invalid = 0;
    int lux = 0;
#if 1
    //if(als < 15)
    //{
        //APS_DBG("epl2182 ALS: %05d => 0\n", als);
   //     return 0;
   // }

    lux = (als * obj->lux_per_count)/1000;

    for(idx = 0; idx < obj->als_level_num; idx++)
    {
        if(lux < obj->hw->als_level[idx])
        {
            break;
        }
    }

    if(idx >= obj->als_value_num)
    {
        APS_ERR("epl2182 exceed range\n");
        idx = obj->als_value_num - 1;
    }

    if(1 == atomic_read(&obj->als_deb_on))
    {
        unsigned long endt = atomic_read(&obj->als_deb_end);
        if(time_after(jiffies, endt))
        {
            atomic_set(&obj->als_deb_on, 0);
        }

        if(1 == atomic_read(&obj->als_deb_on))
        {
            invalid = 1;
        }
    }

    if(!invalid)
    {
		#if 1 //defined(MTK_AAL_SUPPORT)
        int level_high = obj->hw->als_level[idx];
    	int level_low = (idx > 0) ? obj->hw->als_level[idx-1] : 0;
        int level_diff = level_high - level_low;
		int value_high = obj->hw->als_value[idx];
        int value_low = (idx > 0) ? obj->hw->als_value[idx-1] : 0;
        int value_diff = value_high - value_low;
        int value = 0;

        if ((level_low >= level_high) || (value_low >= value_high))
            value = value_low;
        else
            value = (level_diff * value_low + (lux - level_low) * value_diff + ((level_diff + 1) >> 1)) / level_diff;

		//APS_DBG("ALS: %d [%d, %d] => %d [%d, %d] \n", als, level_low, level_high, value, value_low, value_high);
		return value;
		#endif
        //APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
        return obj->hw->als_value[idx];
    }
    else
    {
        APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);
        return -1;
    }
#else

     lux = (als * obj->lux_per_count)/1000;
    for(idx = 0; idx < obj->als_level_num; idx++)
    {   
        if(lux < obj->hw->als_level[idx])
        {
            break;
        }
    }

    if(idx >= obj->als_value_num)
    {
        APS_ERR("exceed range\n");
        idx = obj->als_value_num - 1;
    }

    if(!invalid)
    {
        gRawData.als_lux = obj->hw->als_value[idx];
        APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
        return obj->hw->als_value[idx];
    }
    else
    {
        APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);
        return gRawData.als_lux;
    }
#endif
}

static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd)
{
    int ret = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA data;
    EPL2182_CUST_DATA *pCustData;
    int len;

    //ps_cali would be add back in SCP side.
    low_thd -= epl2182_obj->ps_cali;
    high_thd -= epl2182_obj->ps_cali;

    data.set_cust_req.sensorType = ID_PROXIMITY;
    data.set_cust_req.action = SENSOR_HUB_SET_CUST;
    pCustData = (EPL2182_CUST_DATA *)(&data.set_cust_req.custData);

    pCustData->setPSThreshold.action = EPL2182_CUST_ACTION_SET_PS_THRESHODL;
    pCustData->setPSThreshold.threshold[0] = low_thd;
    pCustData->setPSThreshold.threshold[1] = high_thd;
    len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->setPSThreshold);

    ret = SCP_sensorHub_req_send(&data, &len, 1);
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
    struct epl2182_priv *epld = epl2182_obj;
    struct i2c_client *client = epld->client;
    uint8_t high_msb ,high_lsb, low_msb, low_lsb;

    APS_LOG("[%s]: low_thd = %d, high_thd = %d \n",__func__, low_thd, high_thd);

    high_msb = (uint8_t) (high_thd >> 8);
    high_lsb = (uint8_t) (high_thd & 0x00ff);
    low_msb  = (uint8_t) (low_thd >> 8);
    low_lsb  = (uint8_t) (low_thd & 0x00ff);

    elan_epl2182_I2C_Write(client,REG_2,W_SINGLE_BYTE,0x02,high_lsb);
    elan_epl2182_I2C_Write(client,REG_3,W_SINGLE_BYTE,0x02,high_msb);
    elan_epl2182_I2C_Write(client,REG_4,W_SINGLE_BYTE,0x02,low_lsb);
    elan_epl2182_I2C_Write(client,REG_5,W_SINGLE_BYTE,0x02,low_msb);
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

    return ret;
}



/*----------------------------------------------------------------------------*/
static void epl2182_dumpReg(struct i2c_client *client)
{
    APS_LOG("chip id REG 0x00 value = %8x\n", i2c_smbus_read_byte_data(client, 0x00));
    APS_LOG("chip id REG 0x01 value = %8x\n", i2c_smbus_read_byte_data(client, 0x08));
    APS_LOG("chip id REG 0x02 value = %8x\n", i2c_smbus_read_byte_data(client, 0x10));
    APS_LOG("chip id REG 0x03 value = %8x\n", i2c_smbus_read_byte_data(client, 0x18));
    APS_LOG("chip id REG 0x04 value = %8x\n", i2c_smbus_read_byte_data(client, 0x20));
    APS_LOG("chip id REG 0x05 value = %8x\n", i2c_smbus_read_byte_data(client, 0x28));
    APS_LOG("chip id REG 0x06 value = %8x\n", i2c_smbus_read_byte_data(client, 0x30));
    APS_LOG("chip id REG 0x07 value = %8x\n", i2c_smbus_read_byte_data(client, 0x38));
    APS_LOG("chip id REG 0x09 value = %8x\n", i2c_smbus_read_byte_data(client, 0x48));
    APS_LOG("chip id REG 0x0D value = %8x\n", i2c_smbus_read_byte_data(client, 0x68));
    APS_LOG("chip id REG 0x0E value = %8x\n", i2c_smbus_read_byte_data(client, 0x70));
    APS_LOG("chip id REG 0x0F value = %8x\n", i2c_smbus_read_byte_data(client, 0x71));
    APS_LOG("chip id REG 0x10 value = %8x\n", i2c_smbus_read_byte_data(client, 0x80));
    APS_LOG("chip id REG 0x11 value = %8x\n", i2c_smbus_read_byte_data(client, 0x88));
    APS_LOG("chip id REG 0x13 value = %8x\n", i2c_smbus_read_byte_data(client, 0x98));

}


/*----------------------------------------------------------------------------*/
int hw8k_init_device(struct i2c_client *client)
{
    APS_LOG("hw8k_init_device.........\r\n");

    epl2182_i2c_client=client;

    APS_LOG("epl2182 I2C Addr==[0x%x],line=%d\n",epl2182_i2c_client->addr,__LINE__);

    return 0;
}

/*----------------------------------------------------------------------------*/
int epl2182_get_addr(struct alsps_hw *hw, struct epl2182_i2c_addr *addr)
{
    if(!hw || !addr)
    {
        return -EFAULT;
    }
    addr->write_addr= hw->i2c_addr[0];
    return 0;
}


/*----------------------------------------------------------------------------*/
static void epl2182_power(struct alsps_hw *hw, unsigned int on)
{
#ifndef FPGA_EARLY_PORTING
    static unsigned int power_on = 0;

    //APS_LOG("power %s\n", on ? "on" : "off");

    if(hw->power_id != POWER_NONE_MACRO)
    {
        if(power_on == on)
        {
            APS_LOG("ignore power control: %d\n", on);
        }
        else if(on)
        {
            if(!hwPowerOn(hw->power_id, hw->power_vol, "EPL2182"))
            {
                APS_ERR("power on fails!!\n");
            }
        }
        else
        {
            if(!hwPowerDown(hw->power_id, "EPL2182"))
            {
                APS_ERR("power off fail!!\n");
            }
        }
    }
    power_on = on;
#endif //#ifndef FPGA_EARLY_PORTING
}

/*----------------------------------------------------------------------------*/

int epl2182_read_als(struct i2c_client *client, u16 *data)
{
#ifdef CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA reqData;
    EPL2182_CUST_DATA *pCustData;
    int len;

    reqData.set_cust_req.sensorType = ID_LIGHT;
    reqData.set_cust_req.action = SENSOR_HUB_SET_CUST;
    pCustData = (EPL2182_CUST_DATA *)(&reqData.set_cust_req.custData);

    pCustData->getALSRawData.action = EPL2182_CUST_ACTION_GET_ALS_RAW_DATA;
    len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->getALSRawData);

    SCP_sensorHub_req_send(&reqData, &len, 1);

    //gRawData.als_ch0_raw and gRawData.als_ch1_raw would be updated in eint_work.
    atomic_set(&wait_rsp_flag, 1);
    if (0 == wait_event_interruptible_timeout(wait_rsp_wq, atomic_read(&wait_rsp_flag) == 0, IPI_WAIT_RSP_TIMEOUT))
    {
        APS_ERR("Wait IPI response timeout!\n");
    }
    else
    {
        *data = gRawData.als_ch1_raw;
    }

#else //#ifdef CUSTOM_KERNEL_SENSORHUB
    struct epl2182_priv *obj = i2c_get_clientdata(client);
	uint8_t read_data[2];
    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

    elan_epl2182_I2C_Read(obj->client,REG_14,R_TWO_BYTE,0x02,read_data);
	APS_DBG("epl2182_read_als read REG_14 raw_bytes_high: 0x%x, raw_bytes_low: 0x%x\n",read_data[1],read_data[0]);
    gRawData.als_ch0_raw = (read_data[1]<<8) | read_data[0];
	APS_DBG("epl2182_read_als read channel0 data: 0x%x\n",gRawData.als_ch0_raw);
    elan_epl2182_I2C_Read(obj->client,REG_16,R_TWO_BYTE,0x02,read_data);
	APS_DBG("epl2182_read_als read REG_16 raw_bytes_high: 0x%x, raw_bytes_low: 0x%x\n",read_data[1],read_data[0]);
    gRawData.als_ch1_raw = (read_data[1]<<8) | read_data[0];
	APS_DBG("epl2182_read_als read channel1 data: 0x%x\n",gRawData.als_ch1_raw);
    *data =  gRawData.als_ch1_raw;

    APS_LOG("epl2182 read als raw data = %d\n", gRawData.als_ch1_raw);
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
    return 0;
}


/*----------------------------------------------------------------------------*/
long epl2182_read_ps(struct i2c_client *client, u16 *data)
{
    struct epl2182_priv *obj = i2c_get_clientdata(client);
#ifdef CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA reqData;
    EPL2182_CUST_DATA *pCustData;
    int len;

    reqData.set_cust_req.sensorType = ID_PROXIMITY;
    reqData.set_cust_req.action = SENSOR_HUB_SET_CUST;
    pCustData = (EPL2182_CUST_DATA *)(&reqData.set_cust_req.custData);

    pCustData->getPSRawData.action = EPL2182_CUST_ACTION_GET_PS_RAW_DATA;
    len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->getPSRawData);

    SCP_sensorHub_req_send(&reqData, &len, 1);

    atomic_set(&wait_rsp_flag, 1);
    if (0 == wait_event_interruptible_timeout(wait_rsp_wq, atomic_read(&wait_rsp_flag) == 0, IPI_WAIT_RSP_TIMEOUT))
    {
        APS_ERR("Wait IPI response timeout!\n");
    }
    else
    {
        //gRawData.ps_raw and gRawData.ps_state would be updated in eint_work.
    }

#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	uint8_t read_data[2];
    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

    //elan_epl2182_I2C_Read(client,REG_13,R_SINGLE_BYTE,0x01,read_data);
	//APS_DBG("epl2182_read_als read REG_13 raw_bytes: 0x%x\n",read_data[0]);
    //setting = read_data[0];
    //if((setting&(3<<4))!=0x10)
    //{
        //APS_ERR("epl2182 read ps data in wrong mode\n");
    //}

    //gRawData.ps_state= !((read_data[0]&0x04)>>2);
	//APS_LOG("epl2182 ps state = %d, %s\n", gRawData.ps_state, __func__);

	elan_epl2182_I2C_Read(obj->client, REG_16, R_TWO_BYTE, 0x02, read_data);
	APS_DBG("epl2182_read_ps read REG_16 raw_bytes_high: 0x%x, raw_bytes_low: 0x%x\n", read_data[1], read_data[0]);
	gRawData.ps_raw = (read_data[1] << 8) | read_data[0];
	elan_epl2182_I2C_Read(obj->client, REG_13, R_SINGLE_BYTE, 0x01, read_data);
	gRawData.ps_state = !((read_data[0] & 0x04) >> 2);

#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

#if 0
	if (gRawData.ps_raw < obj->ps_cali)
		*data = 0;
	else
		*data = gRawData.ps_raw - obj->ps_cali;
#else
	*data = gRawData.ps_raw;
#endif

    APS_LOG("epl2182 read ps raw data = %d\n", gRawData.ps_raw);
    APS_LOG("epl2182 read ps binary data = %d\n", gRawData.ps_state);

    return 0;
}


/*----------------------------------------------------------------------------*/
#ifdef CUSTOM_KERNEL_SENSORHUB
static void alsps_init_done_work(struct work_struct *work)
{
    struct epl2182_priv *obj = g_epl2182_ptr;
    EPL2182_CUST_DATA *p_cust_data;
    SCP_SENSOR_HUB_DATA data;
    int max_cust_data_size_per_packet;
    int i;
    uint sizeOfCustData;
    uint len;
    char *p = (char *)obj->hw;

    p_cust_data = (EPL2182_CUST_DATA *)data.set_cust_req.custData;

    data.set_cust_req.sensorType = ID_LIGHT;
    data.set_cust_req.action = SENSOR_HUB_SET_CUST;
    sizeOfCustData = sizeof(*(obj->hw));
    p_cust_data->setCust.action = EPL2182_CUST_ACTION_SET_CUST;
    max_cust_data_size_per_packet = sizeof(data.set_cust_req.custData) - offsetof(EPL2182_SET_CUST, data);

    for (i=0;sizeOfCustData>0;i++)
    {
        p_cust_data->setCust.part = i;
        if (sizeOfCustData > max_cust_data_size_per_packet)
        {
            len = max_cust_data_size_per_packet;
        }
        else
        {
            len = sizeOfCustData;
        }

        memcpy(p_cust_data->setCust.data, p, len);
        sizeOfCustData -= len;
        p += len;

        len += offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + offsetof(EPL2182_SET_CUST, data);
        SCP_sensorHub_req_send(&data, &len, 1);
    }
}
#endif
/*----------------------------------------------------------------------------*/
#ifndef CUSTOM_KERNEL_SENSORHUB
void epl2182_eint_func(void)
{
    struct epl2182_priv *obj = g_epl2182_ptr;

    int_top_time = sched_clock();

    if(!obj)
    {
        return;
    }

//#ifndef FPGA_EARLY_PORTING
    mt_eint_mask(CUST_EINT_ALS_NUM);
//#endif //#ifndef FPGA_EARLY_PORTING
    schedule_work(&obj->eint_work);
}
#else
static int alsps_irq_handler(void* data, uint len)
{
	struct epl2182_priv *obj = g_epl2182_ptr;
    SCP_SENSOR_HUB_DATA_P rsp = (SCP_SENSOR_HUB_DATA_P)data;

	if(!obj)
	{
		return -1;
	}

    switch(rsp->rsp.action)
    {
        case SENSOR_HUB_NOTIFY:
            switch(rsp->notify_rsp.event)
            {
                case SCP_INIT_DONE:
                    schedule_work(&obj->init_done_work);
                    break;
                case SCP_NOTIFY:
                    if (EPL2182_NOTIFY_PROXIMITY_CHANGE == rsp->notify_rsp.event)
                    {
                        gRawData.ps_state = rsp->notify_rsp.data[0];
                        schedule_work(&obj->eint_work);
                    }
                    else
                    {
                        APS_ERR("Unknow notify");
                    }
                    break;
                default:
                    APS_ERR("Error sensor hub notify");
                    break;
            }
            break;
        default:
            APS_ERR("Error sensor hub action");
            break;
    }

    return 0;
}
#endif//#ifdef CUSTOM_KERNEL_SENSORHUB

/*----------------------------------------------------------------------------*/
static void epl2182_eint_work(struct work_struct *work)
{
#ifdef CUSTOM_KERNEL_SENSORHUB
    int res = 0;

    res = ps_report_interrupt_data(gRawData.ps_state);
    if(res != 0)
    {
        APS_ERR("epl2182_eint_work err: %d\n", res);
    }
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	mutex_lock(&sensor_mutex);
    struct epl2182_priv *epld = g_epl2182_ptr;
    int err;
	uint8_t read_data[2];
	int flag;

    wake_lock_timeout(&ps_lock, 2*HZ);  // make sure system wakeup to do ISR //wenggaojian@wind-mobi.com 20150617

    if(epld->enable_pflag==0)
        goto exit;

	APS_ERR("epl2182 int top half time = %lld\n", int_top_time);

	elan_epl2182_I2C_Read(epld->client, REG_13, R_SINGLE_BYTE, 0x01, read_data);
	flag = !((read_data[0] & 0x04) >> 2);

	//if (flag != gRawData.ps_state){
		APS_LOG("epl2182 eint work hard gRawData.ps_state = %d, flag = %d, %s\n", gRawData.ps_state, flag, __func__);

	gRawData.ps_state = flag;//update ps state
	APS_LOG("interrupt hardward = %d\n", gRawData.ps_state);
/*dixiaobing@wind-mobi.com 20150525 start*/
#ifdef CONFIG_SENSOR_NON_WAKE_UP
       //printk("dixiaobing 3333333333333333 non_wakeup_ps=  %d, flag =%d\n",non_wakeup_ps,flag);
        
        
        if(flag)
        {
           als_ps_touch = 0;
		   non_wakeup_ps_flags=flag;
        }
        else
        {
           als_ps_touch = 1;
		   non_wakeup_ps_flags=flag;
         }
        
        if(non_wakeup_ps)
        {
            if(non_wakeup_ps_suspend)
         	{
                if((err = ps_report_interrupt_data(flag)))
                {
                    APS_ERR("epl2182 call ps_report_interrupt_data fail = %d\n", err);
                 }
         	 }else
         	 {
         	     if((err = ps_report_interrupt_data(flag)))
                {
                    APS_ERR("epl2182 call ps_report_interrupt_data fail = %d\n", err);
                 }
         	 }
        }else
        {
        //let up layer to know
            if((err = ps_report_interrupt_data(flag)))
            {  
                APS_ERR("epl2182 call ps_report_interrupt_data fail = %d\n", err);
            }
        }
#else
        //let up layer to know
        if((err = ps_report_interrupt_data(flag)))
        {
            APS_ERR("epl2182 call ps_report_interrupt_data fail = %d\n", err);
        }
#endif
/*dixiaobing@wind-mobi.com 20150525 end*/
	//APS_LOG("epl2182 xxxxx eint work\n");

	//}
	//else{
		//APS_LOG("epl2182 eint data won't update");
		//APS_LOG("epl2182 eint work gRawData.ps_state = %d, flag = %d, %s\n", gRawData.ps_state, flag, __func__);
	//}

exit:
	elan_epl2182_I2C_Write(epld->client, REG_9, W_SINGLE_BYTE, 0x02, EPL_INT_ACTIVE_LOW);
	elan_epl2182_I2C_Write(epld->client, REG_7, W_SINGLE_BYTE, 0x02, EPL_DATA_UNLOCK);
	mutex_unlock(&sensor_mutex);
	//APS_DBG("als disable eint unmask ps!\n");
//#ifndef FPGA_EARLY_PORTING
		mt_eint_unmask(CUST_EINT_ALS_NUM);
//#endif //#ifndef FPGA_EARLY_PORTING
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
}



/*----------------------------------------------------------------------------*/
int epl2182_setup_eint(struct i2c_client *client)
{
#ifdef CUSTOM_KERNEL_SENSORHUB
    int err = 0;

    err = SCP_sensorHub_rsp_registration(ID_PROXIMITY, alsps_irq_handler);

    return err;
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
    struct epl2182_priv *obj = i2c_get_clientdata(client);

    APS_LOG("epl2182_setup_eint\n");


    g_epl2182_ptr = obj;

    /*configure to GPIO function, external interrupt*/

#ifndef FPGA_EARLY_PORTING
    mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

	mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, epl2182_eint_func, 0);

	mt_eint_unmask(CUST_EINT_ALS_NUM);
#endif //#ifndef FPGA_EARLY_PORTING

    return 0;
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
}




/*----------------------------------------------------------------------------*/
static int epl2182_init_client(struct i2c_client *client)
{
#ifndef CUSTOM_KERNEL_SENSORHUB
    struct epl2182_priv *obj = i2c_get_clientdata(client);
#endif
    int err=0;

    APS_LOG("epl2182 [Agold spl] I2C Addr==[0x%x],line=%d\n",epl2182_i2c_client->addr,__LINE__);

    /*  interrupt mode */


    APS_FUN();

#ifdef CUSTOM_KERNEL_SENSORHUB
    epl2182_setup_eint(client);
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
    if(isInterrupt)
    {
#ifndef FPGA_EARLY_PORTING
        mt_eint_mask(CUST_EINT_ALS_NUM);
#endif //#ifndef FPGA_EARLY_PORTING

        if((err = epl2182_setup_eint(client)))
        {
            APS_ERR("setup eint: %d\n", err);
            return err;
        }
        APS_LOG("epl2182 interrupt setup\n");
    }
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB


    if((err = hw8k_init_device(client)) != 0)
    {
        APS_ERR("init dev: %d\n", err);
        return err;
    }

    return err;
}
#ifndef CUSTOM_KERNEL_SENSORHUB
static void epl2182_check_ps_data(struct work_struct *work)
{
	int flag;
	uint8_t read_data[2];
	int err = 0;
	struct epl2182_priv *epld = epl2182_obj;
    elan_epl2182_I2C_Read(epld->client,REG_13,R_SINGLE_BYTE,0x01,read_data);
    flag = !((read_data[0]&0x04)>>2);
	if(flag != int_flag){
		APS_ERR("epl2182 call hwmsen_get_interrupt_data fail = %d\n", err);
		goto exit;
	}
	else{
		//let up layer to know
		APS_LOG("epl2182 int_flag state = %d, %s\n", int_flag, __func__);
		if (0 != (err = ps_report_interrupt_data(int_flag)))
		{
			APS_ERR("epl2182 call ps_report_interrupt_data fail = %d\n", err);
			goto exit;
		}
	}
exit:
	return;
}
#endif
/*----------------------------------------------------------------------------*/
static ssize_t epl2182_show_reg(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = epl2182_obj->client;
    ssize_t len = 0;

    if(!epl2182_obj)
    {
        APS_ERR("epl2182_obj is null!!\n");
        return 0;
    }

    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x00 value = %8x\n", i2c_smbus_read_byte_data(client, 0x00));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x01 value = %8x\n", i2c_smbus_read_byte_data(client, 0x08));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x02 value = %8x\n", i2c_smbus_read_byte_data(client, 0x10));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x03 value = %8x\n", i2c_smbus_read_byte_data(client, 0x18));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x04 value = %8x\n", i2c_smbus_read_byte_data(client, 0x20));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x05 value = %8x\n", i2c_smbus_read_byte_data(client, 0x28));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x06 value = %8x\n", i2c_smbus_read_byte_data(client, 0x30));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x07 value = %8x\n", i2c_smbus_read_byte_data(client, 0x38));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x09 value = %8x\n", i2c_smbus_read_byte_data(client, 0x48));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0D value = %8x\n", i2c_smbus_read_byte_data(client, 0x68));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0E value = %8x\n", i2c_smbus_read_byte_data(client, 0x70));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0F value = %8x\n", i2c_smbus_read_byte_data(client, 0x71));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x10 value = %8x\n", i2c_smbus_read_byte_data(client, 0x80));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x11 value = %8x\n", i2c_smbus_read_byte_data(client, 0x88));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x13 value = %8x\n", i2c_smbus_read_byte_data(client, 0x98));

    return len;

}

/*----------------------------------------------------------------------------*/
static ssize_t epl2182_show_status(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct epl2182_priv *epld = epl2182_obj;
	uint8_t read_data[2];
    if(!epl2182_obj)
    {
        APS_ERR("epl2182_obj is null!!\n");
        return 0;
    }
    elan_epl2182_I2C_Write(epld->client,REG_7,W_SINGLE_BYTE,0x02,EPL_DATA_LOCK);

    elan_epl2182_I2C_Read(epld->client,REG_16,R_TWO_BYTE,0x02,read_data);
    gRawData.ps_raw = (read_data[1]<<8) | read_data[0];
    APS_LOG("ch1 raw_data = %d\n", gRawData.ps_raw);

    elan_epl2182_I2C_Write(epld->client,REG_7,W_SINGLE_BYTE,0x02,EPL_DATA_UNLOCK);
    len += snprintf(buf+len, PAGE_SIZE-len, "ch1 raw is %d\n",gRawData.ps_raw);
    return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl2182_store_als_int_time(struct device_driver *ddri, const char *buf, size_t count)
{
    if(!epl2182_obj)
    {
        APS_ERR("epl2182_obj is null!!\n");
        return 0;
    }

    sscanf(buf, "%d", &ALS_INTT);
    APS_LOG("als int time is %d\n", ALS_INTT);
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl2182_store_ps_int_time(struct device_driver *ddri, const char *buf, size_t count)
{
    if(!epl2182_obj)
    {
        APS_ERR("epl2182_obj is null!!\n");
        return 0;
    }
    sscanf(buf, "%d", &PS_INTT);
    APS_LOG("ps int time is %d\n", PS_INTT);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl2182_store_ps_interrupt(struct device_driver *ddri, const char *buf, size_t count)
{
	if (!epl2182_obj)
	{
		APS_ERR("epl2182_obj is null!!\n");
		return 0;
	}
	sscanf(buf, "%d", &isInterrupt);
	APS_LOG("ps int time is %d\n", isInterrupt);


	epl2182_restart_polling();
	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl2182_show_ps_threshold(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct epl2182_priv *obj = epl2182_obj;
	len += snprintf(buf + len, PAGE_SIZE - len, "gRawData.ps_dyn_high(H/L): %d/%d \r\n", gRawData.ps_dyn_high, gRawData.ps_dyn_low);
	return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl2182_store_ps_threshold(struct device_driver *ddri, const char *buf, size_t count)
{

#ifdef ELAN_WRITE_CALI
    struct epl2182_priv *obj = epl2182_obj;
    int ps_thd_l, ps_thd_h;

    sscanf(buf, "%d,%d", &ps_thd_h, &ps_thd_l);

    gRawData.ps_als_factory.ps_cal_h = ps_thd_h;
    gRawData.ps_als_factory.ps_cal_l = ps_thd_l;

    obj->hw->ps_threshold_high = gRawData.ps_als_factory.ps_cal_h;
    obj->hw->ps_threshold_low = gRawData.ps_als_factory.ps_cal_l;
#else
    sscanf(buf, "%d,%d", &gRawData.ps_dyn_high, &gRawData.ps_dyn_low);
	set_psensor_intr_threshold(gRawData.ps_dyn_low, gRawData.ps_dyn_high);
#endif
	return count;
}

#ifdef ELAN_WRITE_CALI
int ps_cali_flag = 0;
static ssize_t epl2182_show_ps_enable(struct device_driver *ddri, char *buf)
{
    // return sprintf(buf, "ps_enable %d\n", ps_enable_value);
     struct epl2182_priv *obj = epl2182_obj;
   // int ps_enable = 0;
    u16 ch1;
    u32 ch1_all=0;
    int num = 5;
    int i;
    bool enable_ps=test_bit(CMC_BIT_PS, &obj->enable);

	wake_lock(&ps_lock);

        if(isInterrupt)
		{
			mt_eint_unmask(CUST_EINT_ALS_NUM);
			gRawData.ps_int_state = 2;
		}
		gRawData.ps_state  = -1;
           if(enable_ps)
           {
           }
           else
           {
		set_bit(CMC_BIT_PS, &obj->enable);
           }

		epl2182_restart_polling();
		msleep(ALS_DELAY+2*PS_DELAY+50);
        for(i=0; i<num; i++)
		{
			u16 ps_cali_raw;
			msleep(PS_DELAY);


			ps_cali_raw = gRawData.ps_raw;
			APS_LOG("[%s]: gRawData.ps_raw=%d \r\n", __func__, gRawData.ps_raw);

			ch1_all = ch1_all+ ps_cali_raw;
		}

		ch1 = (u16)ch1_all/num;

		

		if(ch1 > PS_MAX_XTALK)
		{
			APS_ERR("Failed: ch1 > max_xtalk(%d) \r\n", ch1);
			ps_cali_flag = 0;
			wake_unlock(&ps_lock);
                        if(enable_ps)
                        {

                        }
                        else
                        {
			  clear_bit(CMC_BIT_PS, &obj->enable);
                        }
                        gRawData.ps_als_factory.cal_ps_raw = -1;
			return sprintf(buf, "%d\n", -1);
		}
		else if(ch1 <= 0)
		{
			APS_ERR("Failed: ch1 = 0\r\n");
			ps_cali_flag = 0;
                        if(enable_ps)
                        {
 
                        }
                        else
                        {
			  clear_bit(CMC_BIT_PS, &obj->enable);
                        }
                        gRawData.ps_als_factory.cal_ps_raw = -1;
			wake_unlock(&ps_lock);
			return sprintf(buf, "%d\n", -1);
		}
		ps_cali_flag = 1;
		gRawData.ps_als_factory.cal_ps_raw = ch1+PS_h_offset;

		gRawData.ps_als_factory.ps_cal_h = ch1+PS_h_offset;
		gRawData.ps_als_factory.ps_cal_l = ch1+PS_l_offset;
		gRawData.ps_als_factory.cal_finished = 1;
                if(enable_ps)
                {
                }
                else
                {
		   clear_bit(CMC_BIT_PS, &obj->enable);
                }
		wake_unlock(&ps_lock);

    return sprintf(buf, "%d\n", gRawData.ps_als_factory.cal_ps_raw);
}
static ssize_t epl2182_store_ps_enable(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl2182_priv *obj = epl2182_obj;
   // int ps_enable = 0;
    u16 ch1;
    u32 ch1_all=0;
    int num = 5;
	int i;

    sscanf(buf, "%d", &ps_enable_value);
	wake_lock(&ps_lock);

    if(ps_enable_value)
    {
        if(isInterrupt)
		{
			mt_eint_unmask(CUST_EINT_ALS_NUM);
            //modify by chenlj2 at 2013/7/10
			gRawData.ps_int_state = 2;
        }
		gRawData.ps_state  = -1;

		set_bit(CMC_BIT_PS, &obj->enable);

		epl2182_restart_polling();
		msleep(ALS_DELAY+2*PS_DELAY+50);
        for(i=0; i<num; i++)
		{
			u16 ps_cali_raw;
			msleep(PS_DELAY);


			ps_cali_raw = gRawData.ps_raw;
			APS_LOG("[%s]: gRawData.ps_raw=%d \r\n", __func__, gRawData.ps_raw);

			ch1_all = ch1_all+ ps_cali_raw;
		}

		ch1 = (u16)ch1_all/num;

		

		if(ch1 > PS_MAX_XTALK)
		{
			APS_ERR("Failed: ch1 > max_xtalk(%d) \r\n", ch1);
			ps_cali_flag = 0;
			wake_unlock(&ps_lock);
			clear_bit(CMC_BIT_PS, &obj->enable);
			return -EINVAL;
		}
		else if(ch1 <= 0)
		{
			APS_ERR("Failed: ch1 = 0\r\n");
			ps_cali_flag = 0;
			clear_bit(CMC_BIT_PS, &obj->enable);
			wake_unlock(&ps_lock);
			return -EINVAL;
		}
		ps_cali_flag = 1;
		gRawData.ps_als_factory.cal_ps_raw = ch1+PS_h_offset;

		gRawData.ps_als_factory.ps_cal_h = ch1+PS_h_offset;
		gRawData.ps_als_factory.ps_cal_l = ch1+PS_l_offset;
		gRawData.ps_als_factory.cal_finished = 1;
		clear_bit(CMC_BIT_PS, &obj->enable);
		wake_unlock(&ps_lock);
    }
    else
    {
		clear_bit(CMC_BIT_PS, &obj->enable);
    	wake_unlock(&ps_lock);
    }
    return count;
}



static ssize_t epl2182_show_ps_cal_raw(struct device_driver *ddri, char *buf)
{
  APS_FUN();

    struct epl2182_priv *obj = epl2182_obj;
    u16 ch1;
    u32 ch1_all=0;
    int count =1;
    int i;
    uint8_t read_data[2];
    ssize_t len = 0;

    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;

    if(!epl2182_obj)
    {
        APS_ERR("epl2182_obj is null!!\n");
        return 0;
    }


    if(enable_ps == 0 || enable_als == 0)
    {
        set_bit(CMC_BIT_ALS, &obj->enable);
        set_bit(CMC_BIT_PS, &obj->enable);
        epl2182_restart_polling();
        msleep(ALS_DELAY+2*PS_DELAY+30+50);
    }

    for(i=0; i<count; i++)
    {
        //elan_epl2182_psensor_enable(obj, 1);
        msleep(PS_DELAY);
        APS_LOG("epl2182_show_ps_cal_raw: gRawData.ps_raw=%d \r\n", gRawData.ps_raw);


		ch1_all = ch1_all+ gRawData.ps_raw;

    }

    ch1 = (u16)ch1_all/count;
	APS_LOG("epl2182_show_ps_cal_raw =  %d\n", ch1);

    len += snprintf(buf+len, PAGE_SIZE-len, "%d \r\n", ch1);
        clear_bit(CMC_BIT_PS, &obj->enable);
	return len;
}

static ssize_t epl2182_store_ps_w_calfile(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl2182_priv *obj = epl2182_obj;
    int ps_hthr=0, ps_lthr=0;
    int ps_cal_len = 0;
    char ps_calibration[20];
	APS_FUN();

	if(!epl2182_obj)
    {
        APS_ERR("epl2182_obj is null!!\n");
        return 0;
    }
    sscanf(buf, "%d,%d", &ps_hthr, &ps_lthr);

    ps_cal_len = sprintf(ps_calibration, "%d,%d", ps_hthr, ps_lthr);

    write_factory_calibration(obj, ps_calibration, ps_cal_len);
	return count;
}

static ssize_t epl2182_show_ps_run_cali(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl2182_priv *obj = epl2182_obj;
	ssize_t len = 0;
    int ret;

    APS_FUN();

    ret = elan_run_calibration(obj);

    len += snprintf(buf+len, PAGE_SIZE-len, "ret = %d,%d,%d,%d\r\n", ret, gRawData.ps_als_factory.cal_ps_raw, gRawData.ps_als_factory.ps_cal_l, gRawData.ps_als_factory.ps_cal_h);

	return len;
}

static ssize_t epl2182_show_ps_cali_data(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl2182_priv *obj = epl2182_obj;
	ssize_t len = 0;
    int ret;
	int ps_thr=0;

    APS_FUN();
	    ps_thr = gRawData.ps_als_factory.cal_ps_raw;

    len += sprintf(buf,"%d\n",ps_thr);

	return len;
}

static ssize_t epl2182_store_ps_cali_data(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl2182_priv *obj = epl2182_obj;
    int ps_thr=0;
	APS_FUN();	
	
	if(!epl2182_obj)
    {
        APS_ERR("epl2182_obj is null!!\n");
        return 0;
    }
    sscanf(buf, "%d", &ps_thr);

//printk("[darren-ep12182] buf = %s,ps_thr = %d\n",buf,ps_thr);
	
	gRawData.ps_als_factory.cal_ps_raw = ps_thr;
    gRawData.ps_als_factory.ps_cal_h = gRawData.ps_als_factory.cal_ps_raw;
    gRawData.ps_als_factory.ps_cal_l = gRawData.ps_als_factory.cal_ps_raw-PS_h_offset+PS_l_offset;

	return count;
}

#endif

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(status, S_IWUSR | S_IRUGO, epl2182_show_status, NULL);
static DRIVER_ATTR(reg, S_IWUSR | S_IRUGO, epl2182_show_reg, NULL);
static DRIVER_ATTR(als_int_time, S_IWUSR | S_IRUGO, NULL, epl2182_store_als_int_time);
static DRIVER_ATTR(ps_int_time, S_IWUSR | S_IRUGO, NULL, epl2182_store_ps_int_time);
static DRIVER_ATTR(ps_interrupt, S_IWUSR | S_IRUGO, NULL, epl2182_store_ps_interrupt);
static DRIVER_ATTR(ps_threshold, S_IWUSR | S_IRUGO, epl2182_show_ps_threshold, epl2182_store_ps_threshold);
#ifdef ELAN_WRITE_CALI
static DRIVER_ATTR(ps_enable, 			    S_IROTH | S_IWOTH, epl2182_show_ps_enable, 	 NULL/*epl2182_store_ps_enable*/);
static DRIVER_ATTR(ps_cal_raw, 			    S_IROTH | S_IWOTH, epl2182_show_ps_cal_raw, 	  		NULL);
static DRIVER_ATTR(ps_calfile,				S_IROTH | S_IWOTH, NULL,			epl2182_store_ps_w_calfile);
static DRIVER_ATTR(ps_run_cali,				S_IROTH | S_IWOTH, epl2182_show_ps_run_cali, NULL);
static DRIVER_ATTR(ps_cali_data,				S_IROTH | S_IWOTH, epl2182_show_ps_cali_data, epl2182_store_ps_cali_data);
#endif
/*----------------------------------------------------------------------------*/
static struct driver_attribute * epl2182_attr_list[] =
{
	&driver_attr_status,
	&driver_attr_reg,
	&driver_attr_als_int_time,
	&driver_attr_ps_int_time,
	&driver_attr_ps_interrupt,
	&driver_attr_ps_threshold,
#ifdef ELAN_WRITE_CALI
    &driver_attr_ps_enable,
    &driver_attr_ps_cal_raw,
    &driver_attr_ps_calfile,
    &driver_attr_ps_run_cali,
    &driver_attr_ps_cali_data,
#endif
};

/*----------------------------------------------------------------------------*/
static int epl2182_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(epl2182_attr_list)/sizeof(epl2182_attr_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if((err = driver_create_file(driver, epl2182_attr_list[idx])))
        {
            APS_ERR("driver_create_file (%s) = %d\n", epl2182_attr_list[idx]->attr.name, err);
            break;
        }
    }
    return err;
}



/*----------------------------------------------------------------------------*/
static int epl2182_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(epl2182_attr_list)/sizeof(epl2182_attr_list[0]));

    if (!driver)
        return -EINVAL;

    for (idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, epl2182_attr_list[idx]);
    }

    return err;
}



/******************************************************************************
 * Function Configuration
******************************************************************************/
static int epl2182_open(struct inode *inode, struct file *file)
{
    file->private_data = epl2182_i2c_client;

    APS_FUN();

    if (!file->private_data)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }

    return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int epl2182_release(struct inode *inode, struct file *file)
{
    APS_FUN();
    file->private_data = NULL;
    return 0;
}

/*----------------------------------------------------------------------------*/



static int set_psensor_threshold(struct i2c_client *client)
{
	struct epl2182_priv *obj = i2c_get_clientdata(client);
	int databuf[2];
	int res = 0;
	databuf[0] = atomic_read(&obj->ps_thd_val_low);
	databuf[1] = atomic_read(&obj->ps_thd_val_high);//threshold value need to confirm

#ifndef DYN_ENABLE
	res = set_psensor_intr_threshold(databuf[0], databuf[1]);
#endif

	return res;
}

void epl2182_restart_polling(void)
{
    struct epl2182_priv *epld = epl2182_obj;

    cancel_delayed_work(&polling_work);

    schedule_delayed_work(&polling_work, msecs_to_jiffies(50)); //liqiang modified
}

static void polling_do_work(struct work_struct *work)
{
    struct epl2182_priv *epld = epl2182_obj;
    struct i2c_client *client = epld->client;

    bool enable_als=test_bit(CMC_BIT_ALS, &epld->enable);
    bool enable_ps=test_bit(CMC_BIT_PS, &epld->enable);

	APS_LOG("als / ps enable: %d / %d \n", enable_als, enable_ps);

    cancel_delayed_work(&polling_work);
#if 0
    if((enable_als && enable_ps) || (enable_ps && isInterrupt == 0) || (enable_als && enable_ps == false)){
        queue_delayed_work(epld->epl_wq, &polling_work,msecs_to_jiffies(PS_DELAY * 2 + ALS_DELAY));
        //queue_delayed_work(epld->epl_wq, &polling_work,msecs_to_jiffies(350));
    }
#else
    queue_delayed_work(epld->epl_wq, &polling_work,msecs_to_jiffies(PS_DELAY * 2 + ALS_DELAY + 30));
    //queue_delayed_work(epld->epl_wq, &polling_work,msecs_to_jiffies(350));
#endif

	//0816
	if (enable_als && atomic_read(&epld->als_suspend) == 0 && test_bit(CMC_BIT_PS, &epld->pending_intr) == 0)
	{
		mutex_lock(&sensor_mutex);
		elan_epl2182_lsensor_enable(epld, 1);
		epl2182_read_als(client, &gRawData.als_ch1_raw);
		mutex_unlock(&sensor_mutex);
		gRawData.als_lux = gRawData.als_ch1_raw*LUX_PER_COUNT / 1000;
	}
	//0816
	if (enable_ps && atomic_read(&epld->ps_suspend) == 0 && test_bit(CMC_BIT_PS, &epld->pending_intr) == 0)
	{
		mutex_lock(&sensor_mutex);
		elan_epl2182_psensor_enable(epld, 1);
		mutex_unlock(&sensor_mutex);
		if (isInterrupt)
		{
		}
		else
		{
			mutex_lock(&sensor_mutex);
			epl2182_read_ps(epl2182_obj->client, &epl2182_obj->ps);
			mutex_unlock(&sensor_mutex);
			}
    }

	if(gRawData.ps_suspend_flag)
	{
		cancel_delayed_work(&polling_work);
		return;
	}

	if (!enable_als &&  !enable_ps)
	{
		cancel_delayed_work(&polling_work);
		APS_LOG("disable sensor\n");
		elan_epl2182_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02, EPL_INT_DISABLE);
		elan_epl2182_I2C_Write(client, REG_0, W_SINGLE_BYTE, 0X02, EPL_S_SENSING_MODE);
	 //09-25 add by Charles  add for suspend -resume TP closing Issue
            }

}

/*----------------------------------------------------------------------------*/
static long epl2182_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct i2c_client *client = (struct i2c_client*)file->private_data;
    struct epl2182_priv *obj = i2c_get_clientdata(client);
    int err = 0;
    void __user *ptr = (void __user*) arg;
    int dat;
    uint32_t enable;
    bool enable_als=test_bit(CMC_BIT_ALS, &obj->enable);
    bool enable_ps=test_bit(CMC_BIT_PS, &obj->enable);      

    
	int ps_result;
	int ps_cali;
	int threshold[2];
#ifdef CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA data;
    EPL2182_CUST_DATA *pCustData;
    int len;

    data.set_cust_req.sensorType = ID_PROXIMITY;
    data.set_cust_req.action = SENSOR_HUB_SET_CUST;
    pCustData = (EPL2182_CUST_DATA *)(&data.set_cust_req.custData);
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

    APS_LOG("---epl2182_ioctll- cmd = %x........\r\n", cmd);

    switch (cmd)
    {
        case ALSPS_SET_PS_MODE:
            if(copy_from_user(&enable, ptr, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
			

            if(enable)
            {
#if 0
                if(isInterrupt)
                {
                    if((err = elan_epl2182_psensor_enable(obj, 1))!=0)
                    {
                        APS_ERR("enable ps fail: %d\n", err);
                        return -1;
                    }
                }
#endif
                set_bit(CMC_BIT_PS, &obj->enable);

            }
            else
            {
#if 0
                if(isInterrupt)
                {
                    if((err = elan_epl2182_psensor_enable(obj, 0))!=0)
                    {
                        APS_ERR("disable ps fail: %d\n", err);
                        return -1;
                    }
                }
#endif
                clear_bit(CMC_BIT_PS, &obj->enable);
            }
            epl2182_restart_polling();
            break;


        case ALSPS_GET_PS_MODE:
            enable=test_bit(CMC_BIT_PS, &obj->enable);
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;


        case ALSPS_GET_PS_DATA:
#if 0
            if((err = elan_epl2182_psensor_enable(obj, 1))!=0)
            {
                APS_ERR("enable ps fail: %d\n", err);
                return -1;
            }
            epl2182_read_ps(obj->client, &obj->ps);
#else
            if(enable_ps == 0)
            {
                set_bit(CMC_BIT_PS, &obj->enable);
                epl2182_restart_polling();
            }
#endif
           dat = gRawData.ps_state;                          

            APS_LOG("ioctl ps state value = %d \n", dat);

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;


        case ALSPS_GET_PS_RAW_DATA:
#if 0
            if((err = elan_epl2182_psensor_enable(obj, 1))!=0)
            {
                APS_ERR("enable ps fail: %d\n", err);
                return -1;
		    }
            epl2182_read_ps(obj->client, &obj->ps);
            dat = obj->ps;
#else
            if(enable_ps == 0)
            {
                set_bit(CMC_BIT_PS, &obj->enable);
                epl2182_restart_polling();
            }
	

           dat = gRawData.ps_raw;
#endif
            APS_LOG("ioctl ps raw value = %d \n", dat);
            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;


        case ALSPS_SET_ALS_MODE:
            if(copy_from_user(&enable, ptr, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            if(enable)
            {
                set_bit(CMC_BIT_ALS, &obj->enable);
            }
            else
            {
                clear_bit(CMC_BIT_ALS, &obj->enable);
            }
            break;



        case ALSPS_GET_ALS_MODE:
            enable=test_bit(CMC_BIT_ALS, &obj->enable);
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;



        case ALSPS_GET_ALS_DATA:
#if 0
            if((err = elan_epl2182_lsensor_enable(obj, 1))!=0)
            {
                APS_ERR("disable als fail: %d\n", err);
                return -1;
            }

            epl2182_read_als(obj->client, &obj->als);
            dat = epl2182_get_als_value(obj, obj->als);
#else
            if(enable_als == 0)
            {
                set_bit(CMC_BIT_ALS, &obj->enable);
                epl2182_restart_polling();
            }

            dat = epl2182_get_als_value(obj, gRawData.als_ch1_raw);
#endif
            APS_LOG("ioctl get als data = %d, gRawData.als_lux=%d, ch1=%d\n", dat, gRawData.als_lux, gRawData.als_ch1_raw);

#if 0
            if(obj->enable_pflag && isInterrupt)
            {
                if((err = elan_epl2182_psensor_enable(obj, 1))!=0)
                {
                    APS_ERR("disable ps fail: %d\n", err);
                    return -1;
                }
            }
#endif
            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;


        case ALSPS_GET_ALS_RAW_DATA:
#if 0
            if((err = elan_epl2182_lsensor_enable(obj, 1))!=0)
            {
                APS_ERR("disable als fail: %d\n", err);
                return -1;
            }

            epl2182_read_als(obj->client, &obj->als);
            dat = obj->als;
#else
            if(enable_als == 0)
            {
                set_bit(CMC_BIT_ALS, &obj->enable);
                epl2182_restart_polling();
             }
			
            dat = gRawData.als_ch1_raw;
#endif
            APS_DBG("ioctl get als raw data = %d\n", dat);

#if 0
            if(obj->enable_pflag && isInterrupt)
            {
                if((err = elan_epl2182_psensor_enable(obj, 1))!=0)
                {
                    APS_ERR("disable ps fail: %d\n", err);
                    return -1;
                }
            }
#endif
            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;
			/*----------------------------------for factory mode test---------------------------------------*/
						case ALSPS_GET_PS_TEST_RESULT:
							if((err = epl2182_read_ps(obj->client, &obj->ps)))
							{
								goto err_out;
							}
							if(obj->ps > atomic_read(&obj->ps_thd_val_high))
								{
									ps_result = 0;
								}
							else	ps_result = 1;

							if(copy_to_user(ptr, &ps_result, sizeof(ps_result)))
							{
								err = -EFAULT;
								goto err_out;
							}
							break;


						case ALSPS_IOCTL_CLR_CALI:
							if(copy_from_user(&dat, ptr, sizeof(dat)))
							{
								err = -EFAULT;
								goto err_out;
							}
							if(dat == 0)
								obj->ps_cali = 0;

#ifdef CUSTOM_KERNEL_SENSORHUB
                            pCustData->clearCali.action = EPL2182_CUST_ACTION_CLR_CALI;
                            len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->clearCali);

                            err = SCP_sensorHub_req_send(&data, &len, 1);
#endif

							break;

						case ALSPS_IOCTL_GET_CALI:
							ps_cali = obj->ps_cali ;
							APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
							if(copy_to_user(ptr, &ps_cali, sizeof(ps_cali)))
							{
								err = -EFAULT;
								goto err_out;
							}
							break;

						case ALSPS_IOCTL_SET_CALI:
							if(copy_from_user(&ps_cali, ptr, sizeof(ps_cali)))
							{
								err = -EFAULT;
								goto err_out;
							}

							obj->ps_cali = ps_cali;

#ifdef CUSTOM_KERNEL_SENSORHUB
                            pCustData->setCali.action = EPL2182_CUST_ACTION_SET_CALI;
                            pCustData->setCali.cali = ps_cali;
                            len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->setCali);

                            err = SCP_sensorHub_req_send(&data, &len, 1);
#endif

							APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
							break;

						case ALSPS_SET_PS_THRESHOLD:
							if(copy_from_user(threshold, ptr, sizeof(threshold)))
							{
								err = -EFAULT;
								goto err_out;
							}
							APS_ERR("%s set threshold high: 0x%x, low: 0x%x\n", __func__, threshold[0],threshold[1]);
							atomic_set(&obj->ps_thd_val_high,  (threshold[0]+obj->ps_cali));
							atomic_set(&obj->ps_thd_val_low,  (threshold[1]+obj->ps_cali));//need to confirm

		set_psensor_threshold(obj->client);

		break;

						case ALSPS_GET_PS_THRESHOLD_HIGH:
							APS_ERR("%s get threshold high before cali: 0x%x\n", __func__, atomic_read(&obj->ps_thd_val_high));
							threshold[0] = atomic_read(&obj->ps_thd_val_high) - obj->ps_cali;
							APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
							APS_ERR("%s get threshold high: 0x%x\n", __func__, threshold[0]);
							if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
							{
								err = -EFAULT;
								goto err_out;
							}
							break;

						case ALSPS_GET_PS_THRESHOLD_LOW:
							APS_ERR("%s get threshold low before cali: 0x%x\n", __func__, atomic_read(&obj->ps_thd_val_low));
							threshold[0] = atomic_read(&obj->ps_thd_val_low) - obj->ps_cali;
							APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
							APS_ERR("%s get threshold low: 0x%x\n", __func__, threshold[0]);
							if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
							{
								err = -EFAULT;
								goto err_out;
							}
							break;
						/*------------------------------------------------------------------------------------------*/



        default:
            APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
            err = -ENOIOCTLCMD;
            break;
    }

err_out:
    return err;
}


/*----------------------------------------------------------------------------*/
static struct file_operations epl2182_fops =
{
    .owner = THIS_MODULE,
    .open = epl2182_open,
    .release = epl2182_release,
    .unlocked_ioctl = epl2182_unlocked_ioctl,
};


/*----------------------------------------------------------------------------*/
static struct miscdevice epl2182_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "als_ps",
    .fops = &epl2182_fops,
};


/*----------------------------------------------------------------------------*/
static int epl2182_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
    //struct epl2182_priv *obj = i2c_get_clientdata(client);
    int err = 0;
    APS_FUN();
	//epl2182_power(epl2182_obj->hw, 1);
#if 0
    if(msg.event == PM_EVENT_SUSPEND)
    {
        if(!obj)
        {
            APS_ERR("null pointer!!\n");
            return -EINVAL;
        }

        atomic_set(&obj->als_suspend, 1);
        if((err = elan_epl2182_lsensor_enable(obj, 0))!=0)
        {
            APS_ERR("disable als: %d\n", err);
            return err;
        }

        atomic_set(&obj->ps_suspend, 1);
        if((err = elan_epl2182_psensor_enable(obj, 0))!=0)
        {
            APS_ERR("disable ps:  %d\n", err);
            return err;
        }

        epl2182_power(obj->hw, 0);
    }
#endif

    return err;

}



/*----------------------------------------------------------------------------*/
static int epl2182_i2c_resume(struct i2c_client *client)
{
   // struct epl2182_priv *obj = i2c_get_clientdata(client);//liqiang
    int err = 0;
    APS_FUN();
#if 0//liqiang 2014
    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }

    epl2182_power(obj->hw, 1);

    msleep(50);

    if(err = epl2182_init_client(client))
    {
        APS_ERR("initialize client fail!!\n");
        return err;
    }

    atomic_set(&obj->als_suspend, 0);
    if(test_bit(CMC_BIT_ALS, &obj->enable))
    {
        if((err = elan_epl2182_lsensor_enable(obj, 1))!=0)
        {
            APS_ERR("enable als fail: %d\n", err);
        }
    }
    atomic_set(&obj->ps_suspend, 0);
    if(test_bit(CMC_BIT_PS,  &obj->enable))
    {
        if((err = elan_epl2182_psensor_enable(obj, 1))!=0)
        {
            APS_ERR("enable ps fail: %d\n", err);
        }
    }


    if(obj->hw->polling_mode_ps == 0)
        epl2182_setup_eint(client);
#endif
    return err;
}



/*----------------------------------------------------------------------------*/
static void epl2182_early_suspend(struct early_suspend *h)
{
    /*early_suspend is only applied for ALS*/
    struct epl2182_priv *obj = container_of(h, struct epl2182_priv, early_drv);
    int err;
    APS_FUN();

    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return;
    }

    atomic_set(&obj->als_suspend, 1);
	gRawData.ps_dny_ini_lock = true;
/*dixiaobing@wind-mobi.com 20150603 start*/
#ifdef CONFIG_SENSOR_NON_WAKE_UP
	non_wakeup_ps_suspend = 1;
#endif
    gRawData.ps_suspend_flag = true;         //wenggaojian@wind-mobi.com 20150617
    msleep(PS_DELAY * 2 + ALS_DELAY + 30);   //wait queue stop  //wenggaojian@wind-mobi.com 20150617


/*dixiaobing@wind-mobi.com 20150603 end*/
	/*
    if(isInterrupt)
	{
		 gRawData.ps_suspend_flag = true;
	}
	*/
	/*
	if(test_bit(CMC_BIT_PS, &obj->enable) == 0)
	{
		atomic_set(&obj->ps_suspend, 1);
		msleep(PS_DELAY * 2 + ALS_DELAY + 30);		
		elan_epl2182_lsensor_enable(obj, 1);//confirm INT pin doesn't keep in Low voltage with active als mode ONCE and switch to idle mode		
    	}
    	*/
}



/*----------------------------------------------------------------------------*/
static void epl2182_late_resume(struct early_suspend *h)
{
    /*late_resume is only applied for ALS*/
    struct epl2182_priv *obj = container_of(h, struct epl2182_priv, early_drv);
    int err;
    APS_FUN();

    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return;
    }

    atomic_set(&obj->als_suspend, 0);
/*dixiaobing@wind-mobi.com 20150603 start*/
#ifdef CONFIG_SENSOR_NON_WAKE_UP
	non_wakeup_ps_suspend = 0;
#endif
/*dixiaobing@wind-mobi.com 20150603 end*/



    atomic_set(&obj->ps_suspend, 0);
	gRawData.ps_dny_ini_lock = false;
	/*
    if(isInterrupt)
		gRawData.ps_suspend_flag = false;
	*/
	
/*dixiaobing@wind-mobi.com 20150604 start*/
#if 0//def CONFIG_SENSOR_NON_WAKE_UP
         //printk("dixiaobing   4444444444  non_wakeup_ps_flags =%d\n",non_wakeup_ps_flags);
	if((err = ps_report_interrupt_data(non_wakeup_ps_flags)))
	{
		APS_ERR("dixiaobing epl2182 call ps_report_interrupt_data fail = %d\n", err);
     }
#endif
/*dixiaobing@wind-mobi.com 20150604 end*/

    gRawData.ps_suspend_flag = false;         //wenggaojian@wind-mobi.com 20150617

	epl2182_restart_polling();
}

/*--------------------------------------------------------------------------------*/
static int als_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int als_enable_nodata(int en)
{
	int res = 0;
	if(!epl2182_obj)
	{
		APS_ERR("epl2182_obj is null!!\n");
		return -1;
	}
	APS_LOG("epl2182_obj als enable value = %d\n", en);

	if(en)
	{
		set_bit(CMC_BIT_ALS, &epl2182_obj->enable);
	    epl2182_restart_polling();
	}
	else
	{
		clear_bit(CMC_BIT_ALS, &epl2182_obj->enable);
	}
	return 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
    if(en)
	{
		if((res = elan_epl2182_lsensor_enable(epl2182_obj, 1)))
		{
			APS_ERR("enable als fail: %d\n", res);
			return -1;
		}
		set_bit(CMC_BIT_ALS, &epl2182_obj->enable);
	}
	else
	{
		if((res = elan_epl2182_lsensor_enable(epl2182_obj, 0)))
		{
			APS_ERR("disable als fail: %d\n", res);
			return -1;
		}
		clear_bit(CMC_BIT_ALS, &epl2182_obj->enable);
	}
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
    if(en)
    {
        set_bit(CMC_BIT_ALS, &epl2182_obj->enable);
#ifndef FPGA_EARLY_PORTING
        mt_eint_mask(CUST_EINT_ALS_NUM);
#endif //#ifndef FPGA_EARLY_PORTING
        //APS_DBG("enable als mask ps!\n");
    }
    else
    {
#ifndef FPGA_EARLY_PORTING
        mt_eint_unmask(CUST_EINT_ALS_NUM);
#endif //#ifndef FPGA_EARLY_PORTING
        //APS_DBG("disable als unmask ps!\n");
        clear_bit(CMC_BIT_ALS, &epl2182_obj->enable);
        if(epl2182_obj->enable_pflag && isInterrupt)
        {
            if((res = elan_epl2182_psensor_enable(epl2182_obj, 1))!=0)
            {
                APS_ERR("enable ps fail: %d\n", res);
                return -1;
            }
        }
    }
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

	if(res){
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;
}
/*--------------------------------------------------------------------------------*/
static int als_set_delay(u64 ns)
{
	return 0;
}
/*--------------------------------------------------------------------------------*/
static int als_get_data(int* value, int* status)
{
	int err = 0;

    APS_FUN();
	if(!epl2182_obj)
	{
		APS_ERR("epl2182_obj is null!!\n");
		return -1;
	}

#ifdef CUSTOM_KERNEL_SENSORHUB
	if((err = epl2182_read_als(epl2182_obj->client, &epl2182_obj->als)))
	{
		err = -1;
	}
	else
	{
		*value = epl2182_get_als_value(epl2182_obj, epl2182_obj->als);
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}
#else //#ifdef CUSTOM_KERNEL_SENSORHUB

#if 0

    if(0 == atomic_read(&epl2182_obj->als_suspend)){
        if((err = elan_epl2182_lsensor_enable(epl2182_obj, 1))!=0)
        {
            APS_ERR("enable als fail: %d\n", err);
            return -1;
        }
        epl2182_read_als(epl2182_obj->client, &epl2182_obj->als);


       *value = epl2182_get_als_value(epl2182_obj, epl2182_obj->als);
       //*value =epl2182_obj->als* epl2182_obj->lux_per_count/1000;

        *status = SENSOR_STATUS_ACCURACY_MEDIUM;
        //APS_LOG("get als data->values[0] = %d\n", sensor_data->values[0]);
	}
	else{
        APS_LOG("epl2182 sensor in suspend!\n");
        return -1;
    }
/*
    if(epl2182_obj->enable_pflag && isInterrupt)
    {
        if((err = elan_epl2182_psensor_enable(epl2182_obj, 1))!=0)
        {
            APS_ERR("enable ps fail: %d\n", err);
            return -1;
        }
    }
*/
#else

	    //*value =gRawData.als_lux;
	    *value = epl2182_get_als_value(epl2182_obj, gRawData.als_ch1_raw);
        *status = SENSOR_STATUS_ACCURACY_MEDIUM;
        APS_LOG("[%s]: gRawData.als_lux=%d, gRawData.als_ch1_raw=%d", __func__, gRawData.als_lux, gRawData.als_ch1_raw);
#endif

#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

	return err;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int ps_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int ps_enable_nodata(int en)
{
	int res = 0;
	if(!epl2182_obj)
	{
		APS_ERR("epl2182_obj is null!!\n");
		return -1;
	}
	APS_LOG("epl2182_obj ps enable value = %d\n", en);

    if(en)
    {
	/*dixiaobing@wind-mobi.com 20150625 start*/
               clear_bit(CMC_BIT_PS, &epl2182_obj->enable);    
               msleep(PS_DELAY * 2 + ALS_DELAY + 40);
   /*dixiaobing@wind-mobi.com 20150625 end*/

		//wake_lock(&ps_lock);         //wenggaojian@wind-mobi.com 20150617
        if(isInterrupt)
		{
			mt_eint_unmask(CUST_EINT_ALS_NUM);
            //modify by chenlj2 at 2013/7/10
			gRawData.ps_int_state = 2;
        }
		gRawData.ps_state  = -1;
#if DYN_ENABLE
		//set_psensor_intr_threshold(0xff00,0xff00);
		if(!gRawData.ps_dny_ini_lock)
		gRawData.ps_min_raw=0xffff;
#endif
		//queue_delayed_work(epl2182_obj->epl_wq, &polling_work,msecs_to_jiffies(5));
        set_bit(CMC_BIT_PS, &epl2182_obj->enable);
		epl2182_restart_polling();
    }
    else
    {
	/*dixiaobing@wind-mobi.com 20150525 start*/
#ifdef CONFIG_SENSOR_NON_WAKE_UP
             //  if(non_wakeup_ps)
                  als_ps_touch = 0;
#endif
	/*dixiaobing@wind-mobi.com 20150525 end*/
		clear_bit(CMC_BIT_PS, &epl2182_obj->enable);
    	//wake_unlock(&ps_lock);          //wenggaojian@wind-mobi.com 20150617
    }
	return 0;

#ifdef CUSTOM_KERNEL_SENSORHUB
    if(en)
	{
		if((res = elan_epl2182_psensor_enable(epl2182_obj, 1)))
		{
			APS_ERR("enable ps fail: %d\n", res);
			return -1;
		}
		set_bit(CMC_BIT_PS, &epl2182_obj->enable);
	}
	else
	{
		if((res = elan_epl2182_psensor_enable(epl2182_obj, 0)))
		{
			APS_ERR("disable ps fail: %d\n", res);
			return -1;
		}
		clear_bit(CMC_BIT_PS, &epl2182_obj->enable);
	}
#else //#ifdef CUSTOM_KERNEL_SENSORHUB

    if(en)
    {
        if(isInterrupt)
        {
            if((res = elan_epl2182_psensor_enable(epl2182_obj, 1))!=0)
            {
                APS_ERR("enable ps fail: %d\n", res);
                return -1;
            }
        }
        set_bit(CMC_BIT_PS, &epl2182_obj->enable);
    }
    else
    {
        if(isInterrupt)
        {
            if((res = elan_epl2182_psensor_enable(epl2182_obj, 0))!=0)
            {
                APS_ERR("disable ps fail: %d\n", res);
                return -1;
            }
        }
        clear_bit(CMC_BIT_PS, &epl2182_obj->enable);
    }

#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

	if(res){
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}

	return 0;

}
/*--------------------------------------------------------------------------------*/
static int ps_set_delay(u64 ns)
{
	return 0;
}
/*--------------------------------------------------------------------------------*/
static int ps_get_data(int* value, int* status)
{
    int err = 0;
    APS_FUN();

    if(!epl2182_obj)
	{
		APS_ERR("epl2182_obj is null!!\n");
		return -1;
	}

#ifdef CUSTOM_KERNEL_SENSORHUB
    if((err = epl2182_read_ps(epl2182_obj->client, &epl2182_obj->ps)))
    {
        err = -1;
    }
    else
    {
        *value = gRawData.ps_state;
        *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    }
#else //#ifdef CUSTOM_KERNEL_SENSORHUB

#if 0
    if((err = elan_epl2182_psensor_enable(epl2182_obj, 1))!=0)
    {
        APS_ERR("enable ps fail: %d\n", err);
        return -1;
    }

    epl2182_read_ps(epl2182_obj->client, &epl2182_obj->ps);
#else

    APS_LOG("[%s]: gRawData.ps_state=%d, gRawData.ps_raw=%d \r\n", __func__, gRawData.ps_state, gRawData.ps_raw);

    *value = gRawData.ps_state;
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;

#endif

#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

	return err;
}
/*----------------------------------------------------------------------------*/

static int epl2182_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    strcpy(info->type, EPL2182_DEV_NAME);
    return 0;
}


/*----------------------------------------------------------------------------*/
static int epl2182_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct epl2182_priv *obj;
    struct als_control_path als_ctl={0};
	struct als_data_path als_data={0};
	struct ps_control_path ps_ctl={0};
	struct ps_data_path ps_data={0};
    int err = 0;
	int temp_value = 0;
	
    APS_FUN();

    epl2182_dumpReg(client);

	temp_value = i2c_smbus_read_byte_data(client, 0x98);

	printk("[darren-als] elan sensor temp_value = 0x%x\n",temp_value);
	
	if(temp_value != 0x68)
	{
		printk("[darren-als] elan sensor is fail\n");
		err = -ENOTSUPP;
		goto exit;
	}

    if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
    {
        err = -ENOMEM;
        goto exit;
    }

    memset(obj, 0, sizeof(*obj));

    epl2182_obj = obj;
    obj->hw = get_cust_alsps_hw_epl2182();

    epl2182_get_addr(obj->hw, &obj->addr);

    epl2182_obj->als_level_num = sizeof(epl2182_obj->hw->als_level)/sizeof(epl2182_obj->hw->als_level[0]);
    epl2182_obj->als_value_num = sizeof(epl2182_obj->hw->als_value)/sizeof(epl2182_obj->hw->als_value[0]);
    BUG_ON(sizeof(epl2182_obj->als_level) != sizeof(epl2182_obj->hw->als_level));
    memcpy(epl2182_obj->als_level, epl2182_obj->hw->als_level, sizeof(epl2182_obj->als_level));
    BUG_ON(sizeof(epl2182_obj->als_value) != sizeof(epl2182_obj->hw->als_value));
    memcpy(epl2182_obj->als_value, epl2182_obj->hw->als_value, sizeof(epl2182_obj->als_value));

    INIT_WORK(&obj->eint_work, epl2182_eint_work);
	obj->epl_wq = create_singlethread_workqueue("elan_sensor_wq");

#ifdef CUSTOM_KERNEL_SENSORHUB
    INIT_WORK(&obj->init_done_work, alsps_init_done_work);
#else
    INIT_WORK(&obj->data_work, epl2182_check_ps_data);
#endif

    init_waitqueue_head(&wait_rsp_wq);
	
	mutex_init(&sensor_mutex);

            gRawData.ps_suspend_flag = false;
    obj->client = client;
#ifdef FPGA_EARLY_PORTING
    obj->client->timing = 100;
#else
    obj->client->timing = 400;
#endif
    obj->client->timing = 400;

    i2c_set_clientdata(client, obj);
	wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "ps wakelock"); //1011
    atomic_set(&obj->als_debounce, 2000);
    atomic_set(&obj->als_deb_on, 0);
    atomic_set(&obj->als_deb_end, 0);
    atomic_set(&obj->ps_debounce, 1000);
    atomic_set(&obj->ps_deb_on, 0);
    atomic_set(&obj->ps_deb_end, 0);
    atomic_set(&obj->ps_mask, 0);
    atomic_set(&obj->trace, 0x00);
    atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->ps_thd_val_high, obj->hw ->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low, obj->hw ->ps_threshold_low);

	obj->ps_cali = 0;
    obj->ps_enable = 0;
    obj->als_enable = 0;
    obj->lux_per_count = LUX_PER_COUNT;
    obj->enable = 0;
    obj->pending_intr = 0;

	gRawData.ps_state = -1;
	gRawData.ps_dny_ini_lock = false;

    atomic_set(&obj->i2c_retry, 3);

    epl2182_i2c_client = client;

/******zhangaifeng@wind-mobi.com***********test begain*************/
//elan_epl2182_I2C_Write(client,REG_0,W_SINGLE_BYTE,0x02, EPL_S_SENSING_MODE);
//elan_epl2182_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,EPL_INT_DISABLE);
    err =elan_epl2182_I2C_Write(client,REG_0,W_SINGLE_BYTE,0x02, EPL_S_SENSING_MODE);
    if(err <0)
    {
		goto exit_init_failed;
	}
    //if((err <elan_epl2182_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,EPL_INT_DISABLE)))
    err <elan_epl2182_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,EPL_INT_DISABLE);
    if(err <0)
    {
		goto exit_init_failed;
	}
/*************zhangaifeng@wind-mobi.com*****test end******************/


    if((err = epl2182_init_client(client)))
    {
        goto exit_init_failed;
    }


    if((err = misc_register(&epl2182_device)))
    {
        APS_ERR("epl2182_device register failed\n");
        goto exit_misc_device_register_failed;
    }

    if((err = epl2182_create_attr(&epl2182_init_info.platform_diver_addr->driver)))
    {
        APS_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }

  //  if( obj->hw->polling_mode_ps == 1)
   // {
	//	isInterrupt = false;
   // }
   // else
   // {
        isInterrupt=true;
  //  }

    als_ctl.open_report_data= als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay  = als_set_delay;
	als_ctl.is_report_input_direct = false;
#ifdef CUSTOM_KERNEL_SENSORHUB
	als_ctl.is_support_batch = true;
#else
    als_ctl.is_support_batch = false;
#endif

	err = als_register_control_path(&als_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}


	ps_ctl.open_report_data= ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay  = ps_set_delay;
	ps_ctl.is_report_input_direct = false;
	ps_ctl.is_polling_mode = false;
#ifdef CUSTOM_KERNEL_SENSORHUB
	ps_ctl.is_support_batch = true;
#else
    ps_ctl.is_support_batch = false;
#endif

	err = ps_register_control_path(&ps_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}
	/*dixiaobing@wind-mobi.com 20150617 start*/
#ifdef ELAN_WRITE_CALI
     meizu_ps_node_init();
#endif
	/*dixiaobing@wind-mobi.com 20150617 end*/
#ifdef ELAN_WRITE_CALI
    gRawData.ps_als_factory.ps_cal_h = 350;
    gRawData.ps_als_factory.ps_cal_l = 300;
    gRawData.ps_als_factory.cal_file_exist = 1;
    gRawData.ps_als_factory.cal_finished = 1;
#endif


#ifndef FPGA_EARLY_PORTING
#if defined(CONFIG_HAS_EARLYSUSPEND)
    obj->early_drv.level    = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2,
    obj->early_drv.suspend  = epl2182_early_suspend,
    obj->early_drv.resume   = epl2182_late_resume,
    register_early_suspend(&obj->early_drv);
#endif
#endif

//liqiang
#if CHECK_CT_ENABLE
	wind_check_ct.ct_high_threshold = CT_H_TH;
	wind_check_ct.ct_low_threshold = CT_L_TH;
#endif
//liqiang

    if(isInterrupt)
        epl2182_setup_eint(client);

    alsps_init_flag = 0;
    APS_LOG("%s: OK\n", __func__);
    return 0;

exit_create_attr_failed:
exit_sensor_obj_attach_fail:
    misc_deregister(&epl2182_device);
exit_misc_device_register_failed:
exit_init_failed:
    alsps_init_flag = -1;
    kfree(obj);
exit:
    epl2182_i2c_client = NULL;
    APS_ERR("%s: err = %d\n", __func__, err);
    alsps_init_flag = -1;
    return err;



}



/*----------------------------------------------------------------------------*/
static int epl2182_i2c_remove(struct i2c_client *client)
{
    int err;

    if((err = epl2182_delete_attr(&epl2182_init_info.platform_diver_addr->driver)))
    {
        APS_ERR("epl2182_delete_attr fail: %d\n", err);
    }

    if((err = misc_deregister(&epl2182_device)))
    {
        APS_ERR("misc_deregister fail: %d\n", err);
    }

    epl2182_i2c_client = NULL;
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));
	
/*dixiaobing@wind-mobi.com 20150617 start*/
#ifdef ELAN_WRITE_CALI
    meizu_ps_node_uninit();
#endif
/*dixiaobing@wind-mobi.com 20150617 end*/
    return 0;
}



/*----------------------------------------------------------------------------*/
static int alsps_local_init(void)
{
    struct alsps_hw *hw = get_cust_alsps_hw_epl2182();
	printk("[darren-als] fwq loccal init  alsps_init_flag = %d\n",alsps_init_flag);

	epl2182_power(hw, 1);
	if(i2c_add_driver(&epl2182_i2c_driver))
	{
		APS_ERR("add driver error\n");
		printk("[darren-als]add driver epl2182 error\n");
		return -1;
	}
	if(-1 == alsps_init_flag)
	{
	   return -1;
	}
	printk("[darren-als]  alsps_init_flag = %d\n",alsps_init_flag);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int alsps_remove()
{
    struct alsps_hw *hw = get_cust_alsps_hw_epl2182();
    APS_FUN();
    epl2182_power(hw, 0);

    APS_ERR("EPL2182 remove \n");
    i2c_del_driver(&epl2182_i2c_driver);
    return 0;
}
/*----------------------------------------------------------------------------*/
static int __init epl2182_init(void)
{
	struct alsps_hw *hw = get_cust_alsps_hw_epl2182();
	APS_LOG("%s: i2c_number=%d\n", __func__,hw->i2c_num);
	i2c_register_board_info(hw->i2c_num, &i2c_EPL2182, 1);
    alsps_driver_add(&epl2182_init_info);
    return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit epl2182_exit(void)
{
    APS_FUN();
}
/*dixiaobing@wind-mobi.com 20150617 start*/
#ifdef ELAN_WRITE_CALI
static ssize_t meizu_ps_calibration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	// return sprintf(buf, "ps_enable %d\n", ps_enable_value);
		 struct epl2182_priv *obj = epl2182_obj;
	   // int ps_enable = 0;
		u16 ch1;
		u32 ch1_all=0;
		int num = 5;
		int i;
		bool enable_ps=test_bit(CMC_BIT_PS, &obj->enable);
	
		wake_lock(&ps_lock);
	
			if(isInterrupt)
			{
				mt_eint_unmask(CUST_EINT_ALS_NUM);
				gRawData.ps_int_state = 2;
			}
			gRawData.ps_state  = -1;
			   if(enable_ps)
			   {
			   }
			   else
			   {
			set_bit(CMC_BIT_PS, &obj->enable);
			   }
	
			epl2182_restart_polling();
			msleep(ALS_DELAY+2*PS_DELAY+50);
			for(i=0; i<num; i++)
			{
				u16 ps_cali_raw;
				msleep(PS_DELAY);
	
	
				ps_cali_raw = gRawData.ps_raw;
				APS_LOG("[%s]: gRawData.ps_raw=%d \r\n", __func__, gRawData.ps_raw);
	
				ch1_all = ch1_all+ ps_cali_raw;
			}
	
			ch1 = (u16)ch1_all/num;
	
			
	
			if(ch1 > PS_MAX_XTALK)
			{
				APS_ERR("Failed: ch1 > max_xtalk(%d) \r\n", ch1);
				ps_cali_flag = 0;
				wake_unlock(&ps_lock);
							if(enable_ps)
							{
	
							}
							else
							{
				  clear_bit(CMC_BIT_PS, &obj->enable);
							}
							gRawData.ps_als_factory.cal_ps_raw = -1;
				return sprintf(buf, "%d\n", -1);
			}
			else if(ch1 <= 0)
			{
				APS_ERR("Failed: ch1 = 0\r\n");
				ps_cali_flag = 0;
							if(enable_ps)
							{
	 
							}
							else
							{
				  clear_bit(CMC_BIT_PS, &obj->enable);
							}
							gRawData.ps_als_factory.cal_ps_raw = -1;
				wake_unlock(&ps_lock);
				return sprintf(buf, "%d\n", -1);
			}
			ps_cali_flag = 1;
			gRawData.ps_als_factory.cal_ps_raw = ch1+PS_h_offset;
	        ps_calibbias_value =gRawData.ps_als_factory.cal_ps_raw;
			gRawData.ps_als_factory.ps_cal_h = ch1+PS_h_offset;
			gRawData.ps_als_factory.ps_cal_l = ch1+PS_l_offset;
			gRawData.ps_als_factory.cal_finished = 1;
					if(enable_ps)
					{
					}
					else
					{
			   clear_bit(CMC_BIT_PS, &obj->enable);
					}
			wake_unlock(&ps_lock);
	
		return sprintf(buf, "%d\n", 1); 
}


static ssize_t meizu_ps_calibration_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
    // return sprintf(buf, "ps_enable %d\n", ps_enable_value);
     struct epl2182_priv *obj = epl2182_obj;
   // int ps_enable = 0;
    u16 ch1;
    u32 ch1_all=0;
    int num = 5;
    int i;
	int ps_calibration_value = 0;
    bool enable_ps=test_bit(CMC_BIT_PS, &obj->enable);
    sscanf(buf, "%d\n",&ps_calibration_value);
if(ps_calibration_value)
{
	wake_lock(&ps_lock);
        ps_calibration_value = 0;
        if(isInterrupt)
		{
			mt_eint_unmask(CUST_EINT_ALS_NUM);
			gRawData.ps_int_state = 2;
		}
		gRawData.ps_state  = -1;
           if(enable_ps)
           {
           }
           else
           {
		set_bit(CMC_BIT_PS, &obj->enable);
           }

		epl2182_restart_polling();
		msleep(ALS_DELAY+2*PS_DELAY+50);
        for(i=0; i<num; i++)
		{
			u16 ps_cali_raw;
			msleep(PS_DELAY);


			ps_cali_raw = gRawData.ps_raw;
			APS_LOG("[%s]: gRawData.ps_raw=%d \r\n", __func__, gRawData.ps_raw);

			ch1_all = ch1_all+ ps_cali_raw;
		}

		ch1 = (u16)ch1_all/num;

		

		if(ch1 > PS_MAX_XTALK)
		{
			APS_ERR("Failed: ch1 > max_xtalk(%d) \r\n", ch1);
			ps_cali_flag = 0;
			wake_unlock(&ps_lock);
                        if(enable_ps)
                        {

                        }
                        else
                        {
			  clear_bit(CMC_BIT_PS, &obj->enable);
                        }
                        gRawData.ps_als_factory.cal_ps_raw = -1;
			 return -1;
		}
		else if(ch1 <= 0)
		{
			APS_ERR("Failed: ch1 = 0\r\n");
			ps_cali_flag = 0;
                        if(enable_ps)
                        {
 
                        }
                        else
                        {
			  clear_bit(CMC_BIT_PS, &obj->enable);
                        }
                        gRawData.ps_als_factory.cal_ps_raw = -1;
			wake_unlock(&ps_lock);
			 return -1;
		}
		ps_cali_flag = 1;
		gRawData.ps_als_factory.cal_ps_raw = ch1+PS_h_offset;
        ps_calibbias_value =gRawData.ps_als_factory.cal_ps_raw;
		gRawData.ps_als_factory.ps_cal_h = ch1+PS_h_offset;
		gRawData.ps_als_factory.ps_cal_l = ch1+PS_l_offset;
		gRawData.ps_als_factory.cal_finished = 1;
                if(enable_ps)
                {
                }
                else
                {
		   clear_bit(CMC_BIT_PS, &obj->enable);
                }
		wake_unlock(&ps_lock);
 }     	
 
     return 1;
}

static ssize_t meizu_ps_calibbias_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
     return sprintf(buf, "%d\n", ps_calibbias_value);
}

static ssize_t meizu_ps_offset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
     int ps_offset_value=0;
     sscanf(buf, "%d\n",&ps_offset_value );
	 gRawData.ps_als_factory.cal_ps_raw = ps_offset_value;
     gRawData.ps_als_factory.ps_cal_h = gRawData.ps_als_factory.cal_ps_raw;
     gRawData.ps_als_factory.ps_cal_l = gRawData.ps_als_factory.cal_ps_raw-PS_h_offset+PS_l_offset;
	 return size;
}

static ssize_t meizu_ps_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
     return sprintf(buf, "%d\n", gRawData.ps_als_factory.cal_ps_raw);
}

static DEVICE_ATTR(ps_calibration, 0664, meizu_ps_calibration_show, meizu_ps_calibration_store);
static DEVICE_ATTR(ps_calibbias, 0664, meizu_ps_calibbias_show, NULL);
static DEVICE_ATTR(ps_offset, 0664, meizu_ps_offset_show,meizu_ps_offset_store);

static void meizu_ps_node_init(void)
{
     int ret ;
     
     psensor = kzalloc(sizeof(struct meizu_classdev), GFP_KERNEL);
	 
	 if (!psensor) {
         APS_ERR("[psensor kzalloc fail!\n");
		 return;
  	}

     psensor->name = "ps";
     ret = meizu_classdev_register(NULL, psensor);
	
	 if(ret)
	 {
	     APS_ERR("[psensor meizu_classdev_register fail!\n");
	     return;
	 }
     ret = device_create_file(psensor->dev, &dev_attr_ps_calibration);
	 if(ret)
	 {
	     APS_ERR("[psensor device_create_file ps_calibration fail!\n");
	 }
     ret = device_create_file(psensor->dev, &dev_attr_ps_calibbias);
	 if(ret)
	 {
	     APS_ERR("[psensor device_create_file ps_calibbias fail!\n");
	 }
     ret = device_create_file(psensor->dev, &dev_attr_ps_offset);
	 if(ret)
	 {
	     APS_ERR("[psensor device_create_file ps_offset fail!\n");
	 }
     return;
}

static void meizu_ps_node_uninit(void)
{
     meizu_classdev_unregister(psensor);
     kfree(psensor);
     psensor = NULL;
	 
	 return;
}
#endif
/*dixiaobing@wind-mobi.com 20150617 end*/

/*----------------------------------------------------------------------------*/
module_init(epl2182_init);
module_exit(epl2182_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("yucong.xiong@mediatek.com");
MODULE_DESCRIPTION("EPL2182 ALSPS driver");
MODULE_LICENSE("GPL");
//liqiang@wind-mobi.com created this file 20140821 end





