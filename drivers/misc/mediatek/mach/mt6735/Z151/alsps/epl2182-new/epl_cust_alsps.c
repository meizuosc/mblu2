#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
//#include <cust_alsps.h>
#include <epl_cust_alsps.h>

static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 2,
    .polling_mode_ps =0,
    .polling_mode_als =1,   
    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    //.i2c_addr   = {0x0C, 0x48, 0x78, 0x00},
 //   .als_level  = { 0,  1,  1,   7,  15,  15,  100, 1000, 2000,  3000,  6000, 10000, 14000, 18000, 20000},
  //  .als_value  = {40, 40, 90,  90, 160, 160,  225,  320,  640,  1280,  1280,  2600,  2600, 2600,  10240, 10240},
    .als_level  = { 0, 8, 100, 180, 270,  650, 1200, 1800, 3000,  4000,  5000, 7000, 9000, 15000, 20000},
    .als_value  = {0,0,150, 340, 430, 570,  700,  800,  950,  1500,  3000,  5000, 7000, 9000, 10240, 10240},
    .ps_threshold_high = 53,
    .ps_threshold_low = 46,
};
struct alsps_hw *get_cust_alsps_hw_epl2182(void) {
    return &cust_alsps_hw;
}   

