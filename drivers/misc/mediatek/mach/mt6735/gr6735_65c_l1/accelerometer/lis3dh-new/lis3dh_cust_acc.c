#include <linux/types.h>
#include <cust_acc.h>
#include <mach/mt_pm_ldo.h>


/*---------------------------------------------------------------------------*/
static struct acc_hw cust_acc_hw = {
    .i2c_num = 2,
//guohongjin@wind-mobi.com 20140808 begin
    .direction = 0,//0 7  6  4  2 3  1
//guohongjin@wind-mobi.com 20140808 end
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
    .firlen = 0, //old value 16                /*!< don't enable low pass fileter */
};
/*---------------------------------------------------------------------------*/
struct acc_hw* lis3dh_get_cust_acc_hw(void) 
{
    return &cust_acc_hw;
}
