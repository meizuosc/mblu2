/* add by yudengwu 2015-01-30 *
 *  this file function is display all devices name
 */

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
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/seq_file.h>
#include <asm/io.h>
#include <asm/uaccess.h>

#include <mach/mtk_nand.h>



#include <linux/proc_fs.h>


#include "lcm_drv.h"
#include "../../../input/touchscreen/mediatek/tpd.h"


#define HARDWARE_INFO_VERSION	0x02


char *gyro_name  = NULL;

static char  color_type[5] = {0};
static char  sw_version_name[11] = {0};
static char  lock_state[2] = {0};
extern LCM_DRIVER  *lcm_drv_name; 

extern char *main_camera;
extern char *sub_camera;
extern char *alsps_name;
extern char *gsensor_name;
extern char *msensor_name;
extern struct tpd_device  *tpd;
extern struct tpd_driver_t *g_tpd_drv;
extern char *saved_command_line;
/****************************************************************************** 
 * Function Configuration
******************************************************************************/


/****************************************************************************** 
 * Debug configuration

******************************************************************************/
static unsigned int func_lv_mask = 0;
#define FUNC_LV_MODULE         BIT(0)  /* module, platform driver interface */

#define FUNC_EXIT(lv)           do { if ((lv) & func_lv_mask) printk("<< %s():%d\n", __func__, __LINE__); } while (0)

static ssize_t show_version(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
	
    ret_value = sprintf(buf, "Version:    :0x%x\n", HARDWARE_INFO_VERSION); 	

    return ret_value;
}

static ssize_t show_lcm(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;

    if(lcm_drv_name->name)
	
    	ret_value = sprintf(buf, "lcd name    :%s\n", lcm_drv_name->name); 	
    else
    	ret_value = sprintf(buf, "lcd  not found\n"); 	

    return ret_value;
}

static ssize_t show_ctp(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    int count = 0;

    if(g_tpd_drv->tpd_device_name)

    count += sprintf(buf + count, "ctp driver  :%s\n", g_tpd_drv->tpd_device_name);

    else

    count += sprintf(buf + count, "ctp id read fail\n");

	ret_value = count;
	
    return ret_value;
}

static ssize_t show_main_camera(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;

   
    if(main_camera)
    	ret_value = sprintf(buf , "main camera :%s\n", main_camera);
    else
        ret_value = sprintf(buf , "main camera :can not  get camera id\n");
	
	
    return ret_value;
}

static ssize_t show_sub_camera(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    int count = 0;

    if(sub_camera)
    	ret_value = sprintf(buf , "sub camera  :%s\n", sub_camera);
    else
        ret_value = sprintf(buf , "sub camera  :can not  get camera id\n");
	
    return ret_value;
}

static ssize_t show_alsps(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
	
    if(alsps_name)
        ret_value = sprintf(buf, "AlSPS name  :%s\n",alsps_name); 	
    else
        ret_value = sprintf(buf, "AlSPS name  :Not found\n");     

    return ret_value;
}

static ssize_t show_gsensor(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;

    if(gsensor_name)
        ret_value = sprintf(buf, "GSensor name:%s\n",gsensor_name); 	
    else
        ret_value = sprintf(buf, "GSensor name:Not found\n");    
   
    return ret_value;
}

static ssize_t show_msensor(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    
  #if defined (CONFIG_CUSTOM_KERNEL_MAGNETOMETER)
	
    if(msensor_name)
        ret_value = sprintf(buf, "MSensor name:%s\n",msensor_name); 	
    else
        ret_value = sprintf(buf, "MSensor name:Not found\n"); 
    #else

    ret_value = sprintf(buf, "MSensor name:not support msensor\n"); 

    #endif

    return ret_value;
}

static ssize_t show_gyro(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;

#if defined(CONFIG_CUSTOM_KERNEL_GYROSCOPE)
    if(gyro_name)
        ret_value = sprintf(buf, "Gyro  name  :%s\n",gyro_name); 	
    else
        ret_value = sprintf(buf, "Gyro  name  :Not found\n"); 
    #else
    ret_value = sprintf(buf, "Gyro  name  :Not support Gyro\n"); 	
#endif
    return ret_value;
}

static ssize_t show_wifi(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    ret_value = sprintf(buf, "wifi name   :MT6625L\n"); 	
    return ret_value;
}
static ssize_t show_bt(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    ret_value = sprintf(buf, "bt name     :MT6625L\n"); 	
    return ret_value;
}
static ssize_t show_gps(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    ret_value = sprintf(buf, "GPS name    :MT6625L\n"); 	
    return ret_value;
}
static ssize_t show_fm(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    ret_value = sprintf(buf, "FM name     :MT6625L\n"); 	
    return ret_value;
}

static DEVICE_ATTR(99_version, 0644, show_version, NULL);
static DEVICE_ATTR(00_lcm, 0644, show_lcm, NULL);
static DEVICE_ATTR(01_ctp, 0644, show_ctp, NULL);
static DEVICE_ATTR(02_main_camera, 0644, show_main_camera, NULL);
static DEVICE_ATTR(03_sub_camera, 0644, show_sub_camera, NULL);
static DEVICE_ATTR(05_gsensor, 0644, show_gsensor, NULL);
static DEVICE_ATTR(06_msensor, 0644, show_msensor, NULL);
static DEVICE_ATTR(08_gyro, 0644, show_gyro, NULL);
static DEVICE_ATTR(07_alsps, 0644, show_alsps, NULL);
static DEVICE_ATTR(09_wifi, 0644, show_wifi, NULL);
static DEVICE_ATTR(10_bt, 0644, show_bt, NULL);
static DEVICE_ATTR(11_gps, 0644, show_gps, NULL);
static DEVICE_ATTR(12_fm, 0644, show_fm, NULL);


///////////////////////////////////////////////////////////////////////////////////////////
//// platform_driver API 
///////////////////////////////////////////////////////////////////////////////////////////
static int HardwareInfo_driver_probe(struct platform_device *dev)	
{	
	int ret_device_file = 0;

    printk("** HardwareInfo_driver_probe!! **\n" );
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_00_lcm)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_01_ctp)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_02_main_camera)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_03_sub_camera)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_05_gsensor)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_06_msensor)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_08_gyro)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_07_alsps)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_09_wifi)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_10_bt)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_11_gps)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_12_fm)) != 0) goto exit_error;

	if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_99_version)) != 0) goto exit_error;   
    

exit_error:	
    return ret_device_file;
}

static int HardwareInfo_driver_remove(struct platform_device *dev)
{
	printk("** HardwareInfo_drvier_remove!! **");

    device_remove_file(&(dev->dev), &dev_attr_00_lcm);
    device_remove_file(&(dev->dev), &dev_attr_01_ctp);
    device_remove_file(&(dev->dev), &dev_attr_02_main_camera);
    device_remove_file(&(dev->dev), &dev_attr_03_sub_camera);
    
//    device_remove_file(&(dev->dev), &dev_attr_04_flash);
    device_remove_file(&(dev->dev), &dev_attr_05_gsensor);
    device_remove_file(&(dev->dev), &dev_attr_06_msensor);
    device_remove_file(&(dev->dev), &dev_attr_08_gyro);
    device_remove_file(&(dev->dev), &dev_attr_07_alsps);
    device_remove_file(&(dev->dev), &dev_attr_09_wifi);
    device_remove_file(&(dev->dev), &dev_attr_10_bt);
    device_remove_file(&(dev->dev), &dev_attr_11_gps);
    device_remove_file(&(dev->dev), &dev_attr_12_fm);

    device_remove_file(&(dev->dev), &dev_attr_99_version);
    return 0;
}





static struct platform_driver HardwareInfo_driver = {
    .probe		= HardwareInfo_driver_probe,
    .remove     = HardwareInfo_driver_remove,
    .driver     = {
        .name = "HardwareInfo",
    },
};

static struct platform_device HardwareInfo_device = {
    .name   = "HardwareInfo",
    .id	    = -1,
};

static char *_copy_from_user_for_proc(const char __user *buffer, size_t count)
{
    char *buf = (char *)__get_free_page(GFP_USER);

    if (!buf)
        return NULL;

    if (count >= PAGE_SIZE)
        goto out;

    if (copy_from_user(buf, buffer, count))
        goto out;

    buf[count] = '\0';

    return buf;

out:
    free_page((unsigned long)buf);

    return NULL;
}

static void init_proinfo_value_from_cmdline(){
    int size = strlen(saved_command_line)+1;
    char *strs = (char *)kmalloc(size, GFP_KERNEL);
    if(strs == NULL){
        return;
    }
    memcpy(strs, saved_command_line, size);
    char *token;
    char delim[] = "= ";
    for(token = strsep(&strs, delim); token != NULL; token = strsep(&strs, delim)){
        if(!strcmp(token, "androidboot.colortype")){
            strcpy(color_type, strsep(&strs, delim));
        }else if(!strcmp(token, "androidboot.sw_version")){
            strcpy(sw_version_name, strsep(&strs, delim));
        }else if(!strcmp(token, "androidboot.lstate")){
            strcpy(lock_state, strsep(&strs, delim));
        }
    }
    kfree(strs);
}

static int sw_version_proc_show(struct seq_file *m, void *v)
{
    seq_printf(m, "%s\n", sw_version_name);
    return 0;
}

static int rtx_proc_show(struct seq_file *m, void *v)
{
    seq_printf(m, "%s\n", lock_state);
    return 0;
}

static int colortype_proc_show(struct seq_file *m, void *v)
{
    seq_printf(m, "%s\n", color_type);
    return 0;
}

static int sec_proc_show(struct seq_file *m, void *v)
{
#ifdef CONFIG_MTK_SECURITY_SW_SUPPORT
    /*if(1 == g_lk_info.sec_policy)
    {
        seq_printf(m, "%s\n", "sw_security");
    }
    else
    {*/
        extern int sec_schip_enabled(void);
        seq_printf(m, "%s\n", sec_schip_enabled()?"hw_security":"Non-Secure Chip");
    //}
#else
    seq_printf(m, "%s\n", "NS");
#endif
    return 0;
}

#define PROC_FOPS_RW(name)							\
    static int name ## _proc_open(struct inode *inode, struct file *file)	\
{									\
    return single_open(file, name ## _proc_show, PDE_DATA(inode));	\
}									\
static const struct file_operations name ## _proc_fops = {		\
    .owner          = THIS_MODULE,					\
    .open           = name ## _proc_open,				\
    .read           = seq_read,					\
    .llseek         = seq_lseek,					\
    .release        = single_release,				\
    .write          = name ## _proc_write,				\
}


#define PROC_FOPS_RO(name)							\
    static int name ## _proc_open(struct inode *inode, struct file *file)	\
{									\
    return single_open(file, name ## _proc_show, PDE_DATA(inode));	\
}									\
static const struct file_operations name ## _proc_fops = {		\
    .owner          = THIS_MODULE,					\
    .open           = name ## _proc_open,				\
    .read           = seq_read,					\
    .llseek         = seq_lseek,					\
    .release        = single_release,				\
}


PROC_FOPS_RO(colortype); // <-XXX
PROC_FOPS_RO(sw_version);
PROC_FOPS_RO(rtx);
PROC_FOPS_RO(sec);

#define PROC_ENTRY(name)	{__stringify(name), &name ## _proc_fops}

static int _colortype_proc_create_procfs(void)
{
    struct proc_dir_entry *dir = NULL;
    //struct proc_dir_entry *cpu_dir = NULL; 
    int i; //, j;

    struct pentry {
        const char *name;
        const struct file_operations *fops;
    };

    const struct pentry entries[] = {
        PROC_ENTRY(colortype),  
		PROC_ENTRY(sw_version),
		PROC_ENTRY(rtx),
		PROC_ENTRY(sec),
    };

    dir = proc_mkdir("lk_info", NULL);

    if (!dir) {
        printk("fail to create /proc/lk_info @ %s()\n", __func__);
        return -ENOMEM;
    }
    init_proinfo_value_from_cmdline();
	 for (i = 0; i < ARRAY_SIZE(entries); i++) {
        if (!proc_create(entries[i].name, S_IRUGO | S_IWUSR | S_IWGRP, dir, entries[i].fops))
            printk("%s(), create /proc/cpufreq/%s failed\n", __func__, entries[i].name);
	 	}
        
    return 0;
}



static int __init HardwareInfo_mod_init(void)
{
    int ret = 0;


    ret = platform_device_register(&HardwareInfo_device);
    if (ret) {
        printk("**HardwareInfo_mod_init  Unable to driver register(%d)\n", ret);
        goto  fail_2;
    }
    

    ret = platform_driver_register(&HardwareInfo_driver);
    if (ret) {
        printk("**HardwareInfo_mod_init  Unable to driver register(%d)\n", ret);
        goto  fail_1;
    }

	if(_colortype_proc_create_procfs())
		{
			printk("**_colortype_proc_create_procfs  Unable to driver register\n");
        		goto  out;
		}

    goto ok_result;

    
fail_1:
	platform_driver_unregister(&HardwareInfo_driver);
fail_2:
	platform_device_unregister(&HardwareInfo_device);
out:
   	FUNC_EXIT(FUNC_LV_MODULE);
ok_result:

    return ret;
}


/*****************************************************************************/
static void __exit HardwareInfo_mod_exit(void)
{
    platform_driver_unregister(&HardwareInfo_driver);
	platform_device_unregister(&HardwareInfo_device);
	 FUNC_EXIT(FUNC_LV_MODULE);
}
/*****************************************************************************/
module_init(HardwareInfo_mod_init);
module_exit(HardwareInfo_mod_exit);
/*****************************************************************************/
MODULE_AUTHOR(" <yudengwu@wind-mobi.com>");
MODULE_DESCRIPTION("MT6735 Hareware Info driver");
MODULE_LICENSE("GPL");



