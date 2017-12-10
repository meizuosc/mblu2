//guohongjin@wind-mobi.com 20140813 end
/* BEGIN PN: , Added by guohongjin, 2014.08.13*/
#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif

#ifdef BUILD_LK
#include <platform/gpio_const.h>
#include <platform/mt_gpio.h>
#include <platform/upmu_common.h>
#else
#include <mach/gpio_const.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <linux/string.h>
#endif


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define LCM_DSI_CMD_MODE									0
#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)

#define LCM_ID_ILI7802                                                              (0x7802)


#define REGFLAG_DELAY             								0xFC
#define REGFLAG_END_OF_TABLE      							0xFD   // END OF REGISTERS MARKER

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------


static LCM_UTIL_FUNCS lcm_util;

#define __SAME_IC_COMPATIBLE__

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define MDELAY(n) 											(lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting1[] = {
	//CCMON
	/*
	Note :

	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...
 
	Setting ending by predefined flag
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/
#if 0
	{0xFF,3,{0x98,0x81,0x07}},
//GIP_1
	{0x03,1,{0x21}},
	{0x04,1,{0x06}},
	{0x05,1,{0x00}},
	{0x06,1,{0x00}},
	{0x07,1,{0x00}},
	{0x08,1,{0x04}},
	{0x09,1,{0x00}},
	{0x0a,1,{0x04}},
	{0x0b,1,{0x03}},
	{0x0c,1,{0x03}},
	{0x0d,1,{0x00}},
	{0x0e,1,{0x00}},
	{0x0f,1,{0x00}},
	{0x10,1,{0x40}}, 
	{0x11,1,{0x04}},
	{0x12,1,{0x0B}}, 
	{0x13,1,{0x00}},
	{0x14,1,{0x00}},
	{0x15,1,{0x00}},
	{0x16,1,{0x04}},
	{0x17,1,{0x04}},
	{0x18,1,{0x00}},
	{0x19,1,{0x00}},
	{0x1a,1,{0x00}},
	{0x1b,1,{0x54}},
	{0x1c,1,{0xBB}},
	{0x1d,1,{0x3C}},
	{0x1e,1,{0x04}},
	{0x1f,1,{0x00}},
	{0x20,1,{0x00}},
	{0x21,1,{0x00}},
	{0x22,1,{0x00}},
	{0x23,1,{0xC0}},
	{0x24,1,{0x50}},
	{0x25,1,{0x00}},
	{0x26,1,{0x08}}, 
	{0x27,1,{0x05}},
//GIP_2          ,
	{0x30,1,{0x01}},
	{0x31,1,{0x23}},
	{0x32,1,{0x45}},
	{0x33,1,{0x67}},
	{0x34,1,{0x89}},
	{0x35,1,{0xab}},
	{0x36,1,{0x01}},
	{0x37,1,{0x23}},
	{0x38,1,{0x45}},
	{0x39,1,{0x67}},
	{0x3a,1,{0x89}},
	{0x3b,1,{0xab}},
	{0x3c,1,{0xcd}},
	{0x3d,1,{0xef}},
//GIP_3          ,
	{0x50,1,{0x01}},
	{0x51,1,{0x14}},
	{0x52,1,{0x15}},
	{0x53,1,{0x0C}},
	{0x54,1,{0x0D}},
	{0x55,1,{0x0E}},
	{0x56,1,{0x0F}},
	{0x57,1,{0x10}},
	{0x58,1,{0x11}},
	{0x59,1,{0x08}}, 
	{0x5a,1,{0x02}}, 
	{0x5b,1,{0x0A}},
	{0x5c,1,{0x02}},
	{0x5d,1,{0x02}},
	{0x5e,1,{0x02}},
	{0x5f,1,{0x02}},
	{0x60,1,{0x02}},
	{0x61,1,{0x02}},
	{0x62,1,{0x02}},
	{0x63,1,{0x02}},
	{0x64,1,{0x06}},
	{0x65,1,{0x02}},
	{0x66,1,{0x02}},
	{0x67,1,{0x14}},
	{0x68,1,{0x15}},
	{0x69,1,{0x11}},
	{0x6a,1,{0x10}},
	{0x6b,1,{0x0F}},
	{0x6c,1,{0x0E}},
	{0x6d,1,{0x0D}},
	{0x6e,1,{0x0C}},
	{0x6f,1,{0x06}}, 
	{0x70,1,{0x02}}, 
	{0x71,1,{0x0A}},
	{0x72,1,{0x02}},
	{0x73,1,{0x02}},
	{0x74,1,{0x02}},
	{0x75,1,{0x02}},
	{0x76,1,{0x02}},
	{0x77,1,{0x02}},
	{0x78,1,{0x02}},
	{0x79,1,{0x02}},
	{0x7a,1,{0x08}},
	{0x7b,1,{0x02}},
	{0x7c,1,{0x02}},
//CMD_Page 8
	{0xFF,3,{0x98,0x81,0x08}},
	{0x76,1,{0xD4}},               
	{0x78,1,{0x02}},               
	{0x74,1,{0x3b}},               
	{0x8E,1,{0x15}},               
	{0x40,1,{0x01}},
	{0x7B,1,{0x04}},
	{0x72,1,{0x25}}, 
	{0x87,1,{0x3a}},
	{0x6c,1,{0x05}},
	{0x49,1,{0x10}}, 
	{0x2D,1,{0x80}}, 
	{0x32,1,{0x05}},
	{0x3C,1,{0x05}},
	{0xE4,1,{0xE0}},
//CMD_Page 1
	{0xFF,3,{0x98,0x81,0x01}},
	{0x22,1,{0x0A}},               
	{0x31,1,{0x00}}, 
	{0x53,1,{0x7D}},              
	{0x55,1,{0x88}},               
	{0x50,1,{0xA3}},                 //99  4.5   a6                       
	{0x51,1,{0xA3}},                 //aa                      
	{0x60,1,{0x30}},               //SDT
	{0xA0,1,{0x00}},               //08               //VP255 Gamma P
	{0xA1,1,{0x1C}},              //VP251
	{0xA2,1,{0x2B}},               //VP247
	{0xA3,1,{0x12}},               //VP243
	{0xA4,1,{0x15}},               //VP239
	{0xA5,1,{0x29}},               //VP231
	{0xA6,1,{0x1D}},               //VP219
	{0xA7,1,{0x1F}},               //VP203
	{0xA8,1,{0x88}},               //VP175
	{0xA9,1,{0x1B}},               //VP144
	{0xAA,1,{0x28}},               //VP111
	{0xAB,1,{0x71}},               //VP80
	{0xAC,1,{0x1B}},               //VP52
	{0xAD,1,{0x19}},               //VP36
	{0xAE,1,{0x4C}},               //VP24
	{0xAF,1,{0x21}},               //VP16
	{0xB0,1,{0x28}},               //VP12
	{0xB1,1,{0x4A}},               //VP8
	{0xB2,1,{0x59}},               //VP4
	{0xB3,1,{0x3f}},              //2C               //VP0
	{0xC0,1,{0x00}},               //08               //VN255 GAMMA N
	{0xC1,1,{0x1C}},               //VN251
	{0xC2,1,{0x2B}},               //VN247
	{0xC3,1,{0x12}},               //VN243
	{0xC4,1,{0x15}},               //VN239
	{0xC5,1,{0x29}},               //VN231
	{0xC6,1,{0x1D}},               //VN219
	{0xC7,1,{0x1F}},               //VN203
	{0xC8,1,{0x88}},               //VN175
	{0xC9,1,{0x1B}},               //VN144
	{0xCA,1,{0x28}},               //VN111
	{0xCB,1,{0x71}},               //VN80
	{0xCC,1,{0x1B}},               //VN52
	{0xCD,1,{0x19}},               //VN36
	{0xCE,1,{0x4C}},               //VN24
	{0xCF,1,{0x21}},               //VN16
	{0xD0,1,{0x28}},               //VN12
	{0xD1,1,{0x4A}},               //VN8
	{0xD2,1,{0x59}},               //VN4
	{0xD3,1,{0x3f}},             //2C               //VN0
/*****   CMD Page 0   *****/
	{0xFF,3,{0x98,0x81,0x00}},
	{0x36,1,{0x03}},

	{0x11,	1,		{0x00}},       //Sleep out
       {REGFLAG_DELAY, 120, {}},		//120
	{0x29,	1,		{0x00}},	
	{REGFLAG_DELAY, 20, {}},	   //100
	{REGFLAG_END_OF_TABLE, 0x00, {}}
#else
//[znw]

{0xFF,5,{0xFF,0x78,0x02,0x00,0x07}},
{0xF1,1,{0x01}},
{0xF5,1,{0x80}},


{0xFF,5,{0xFF,0x78,0x02,0x00,0x06}},
{0x00,1,{0x21}},
{0x01,1,{0x01}},
{0x02,1,{0x80}},
{0x03,1,{0x00}},
{0x04,1,{0x00}},
{0x05,1,{0x00}},
{0x06,1,{0x80}},
{0x07,1,{0x03}},
{0x08,1,{0x00}},
{0x09,1,{0x10}},
{0x0A,1,{0x00}},
{0x0B,1,{0x00}},
{0x0C,1,{0x05}},	     // 06
{0x0D,1,{0x05}},	     // 06
{0x0E,1,{0x00}},
{0x0F,1,{0x00}},
{0x10,1,{0x10}},	     // 10
{0x11,1,{0x04}},
{0x12,1,{0x00}},
{0x13,1,{0x00}},
{0x14,1,{0x00}},
{0x15,1,{0xC0}},
{0x16,1,{0x08}},
{0x17,1,{0x00}},
{0x18,1,{0x00}},
{0x19,1,{0x04}},	     // 06
{0x1A,1,{0x00}},
{0x1B,1,{0x00}},
{0x1C,1,{0x00}},
{0x1D,1,{0x00}},
{0x1E,1,{0x00}},
{0x20,1,{0x00}},
{0x21,1,{0x23}},
{0x22,1,{0x45}},
{0x23,1,{0x67}},
{0x24,1,{0x00}},
{0x25,1,{0x11}},
{0x26,1,{0x45}},
{0x27,1,{0x67}},
{0x30,1,{0x00}},
{0x31,1,{0x0A}},
{0x32,1,{0x0B}},
{0x33,1,{0x03}},
{0x34,1,{0x03}},
{0x35,1,{0x03}},
{0x36,1,{0x03}},
{0x37,1,{0x03}},
{0x38,1,{0x03}},
{0x39,1,{0x03}},
{0x3A,1,{0x03}},
{0x3B,1,{0x03}},
{0x3C,1,{0x03}},
{0x3D,1,{0x03}},
{0x3E,1,{0x03}},
{0x3F,1,{0x03}},
{0x40,1,{0x03}},
{0x41,1,{0x03}},
{0x42,1,{0x03}},
{0x43,1,{0x01}},
{0x44,1,{0x06}},
{0x45,1,{0x03}},
{0x46,1,{0x15}},
{0x47,1,{0x14}},
{0x48,1,{0x13}},
{0x49,1,{0x03}},
{0x4A,1,{0x03}},
{0x4B,1,{0x03}},
{0x4C,1,{0x03}},
{0x4D,1,{0x03}},
{0x4E,1,{0x03}},
{0x4F,1,{0x03}},
{0x50,1,{0x03}},
{0x51,1,{0x03}},
{0x60,1,{0x00}},
{0x61,1,{0x0B}},
{0x62,1,{0x0A}},
{0x63,1,{0x03}},
{0x64,1,{0x03}},
{0x65,1,{0x03}},
{0x66,1,{0x03}},
{0x67,1,{0x03}},
{0x68,1,{0x03}},
{0x69,1,{0x03}},
{0x6A,1,{0x03}},
{0x6B,1,{0x03}},
{0x6C,1,{0x03}},
{0x6D,1,{0x03}},
{0x6E,1,{0x03}},
{0x6F,1,{0x03}},
{0x70,1,{0x03}},
{0x71,1,{0x03}},
{0x72,1,{0x03}},
{0x73,1,{0x01}},
{0x74,1,{0x06}},
{0x75,1,{0x03}},
{0x76,1,{0x15}},
{0x77,1,{0x14}},
{0x78,1,{0x13}},
{0x79,1,{0x03}},
{0x7A,1,{0x03}},
{0x7B,1,{0x03}},
{0x7C,1,{0x03}},
{0x7D,1,{0x03}},
{0x7E,1,{0x03}},
{0x7F,1,{0x03}},
{0x80,1,{0x03}},
{0x81,1,{0x03}},
{0xA0,1,{0x0E}},
{0xA2,1,{0x08}},
{0xA3,1,{0x17}},
{0xA7,1,{0x00}},


{0xFF,5,{0xFF,0x78,0x02,0x00,0x01}},
{0x22,1,{0x03}},
{0x30,1,{0x02}},
{0x31,1,{0x00}},            // inversion
{0x36,1,{0x01}},        
{0x74,1,{0x00}},
{0x75,1,{0x00}},
{0x42,1,{0x00}},	     //   VGH=VSP+VCI=3*VCI VGL=VCL-VCI=2*VCL
{0x43,1,{0x8B}},	     // CLAMP VGH 
{0x44,1,{0x8F}}, 	     // CLAMP VGL=-7.064
{0x46,1,{0x1A}},
{0x40,1,{0x12}},
{0x41,1,{0xF0}},
{0x45,1,{0x15}},
{0x50,1,{0xc2}},	     // gvddp
{0x51,1,{0xc2}},	     // gvddn
{0x64,1,{0x00}},		//EQT OFF
{0x65,1,{0x0f}},		//
{0x53,1,{0x90}},		//vcom
	//Generic_Short_Write_1P(0x56,0x11);
	
	
{0xFF,5,{0xFF,0x78,0x02,0x00,0x02}},
{0x00,1,{0x00}},
{0x01,1,{0x8c}},	//255
{0x02,1,{0x00}},
{0x03,1,{0xd4}},//254
{0x04,1,{0x01}},
{0x05,1,{0x09}},//252
{0x06,1,{0x01}},
{0x07,1,{0x1a}},//250
{0x08,1,{0x01}},
{0x09,1,{0x30}},//248
{0x0A,1,{0x01}},
{0x0B,1,{0x3f}},//246
{0x0C,1,{0x01}},
{0x0D,1,{0x50}},//244
{0x0E,1,{0x01}},
{0x0F,1,{0x5b}},//242
{0x10,1,{0x01}},
{0x11,1,{0x69}},//240
{0x12,1,{0x01}},
{0x13,1,{0x91}},//232
{0x14,1,{0x01}},
{0x15,1,{0xb0}},//224
{0x16,1,{0x01}},
{0x17,1,{0xe1}},//208
{0x18,1,{0x02}},
{0x19,1,{0x0a}},//192
{0x1A,1,{0x02}},
{0x1B,1,{0x4a}},//160
{0x1C,1,{0x02}},
{0x1D,1,{0x79}},//128
{0x1E,1,{0x02}},
{0x1F,1,{0x7b}},//127
{0x20,1,{0x02}},
{0x21,1,{0xaa}},//95
{0x22,1,{0x02}},
{0x23,1,{0xde}},//63
{0x24,1,{0x02}},
{0x25,1,{0xfe}},//47
{0x26,1,{0x03}},
{0x27,1,{0x26}},//31
{0x28,1,{0x03}},
{0x29,1,{0x40}},//23
{0x2A,1,{0x03}},
{0x2B,1,{0x65}},//15
{0x2C,1,{0x03}},
{0x2D,1,{0x6C}},//13
{0x2E,1,{0x03}},
{0x2F,1,{0x78}},//11
{0x30,1,{0x03}},
{0x31,1,{0x83}},//9
{0x32,1,{0x03}},
{0x33,1,{0x92}},//7
{0x34,1,{0x03}},
{0x35,1,{0xa1}},//5
{0x36,1,{0x03}},
{0x37,1,{0xb2}},//3
{0x38,1,{0x03}},
{0x39,1,{0xb8}},//1
{0x3A,1,{0x03}},
{0x3B,1,{0xC2}},//0
{0x3C,1,{0x00}},
{0x3D,1,{0x8c}},//255
{0x3E,1,{0x00}},
{0x3F,1,{0xd4}},//254
{0x40,1,{0x01}},
{0x41,1,{0x09}},//252
{0x42,1,{0x01}},
{0x43,1,{0x1a}},//250
{0x44,1,{0x01}},
{0x45,1,{0x30}},//248
{0x46,1,{0x01}},
{0x47,1,{0x3f}},//246
{0x48,1,{0x01}},
{0x49,1,{0x50}},//244
{0x4A,1,{0x01}},
{0x4B,1,{0x5b}},//242
{0x4C,1,{0x01}},
{0x4D,1,{0x69}},//240
{0x4E,1,{0x01}},
{0x4F,1,{0x91}},//232
{0x50,1,{0x01}},
{0x51,1,{0xb0}},//224
{0x52,1,{0x01}},
{0x53,1,{0xe1}},//208
{0x54,1,{0x02}},
{0x55,1,{0x0a}},//192
{0x56,1,{0x02}},
{0x57,1,{0x4a}},//160
{0x58,1,{0x02}},
{0x59,1,{0x79}},//128
{0x5A,1,{0x02}},
{0x5B,1,{0x7b}},//127
{0x5C,1,{0x02}},
{0x5D,1,{0xaa}},//95
{0x5E,1,{0x02}},
{0x5F,1,{0xde}},//63
{0x60,1,{0x02}},
{0x61,1,{0xfe}},//47
{0x62,1,{0x03}},
{0x63,1,{0x26}},//31
{0x64,1,{0x03}},
{0x65,1,{0x40}},//23
{0x66,1,{0x03}},
{0x67,1,{0x65}},//15
{0x68,1,{0x03}},
{0x69,1,{0x6C}},//13
{0x6A,1,{0x03}},
{0x6B,1,{0x78}},//11
{0x6C,1,{0x03}},
{0x6D,1,{0x83}},//9
{0x6E,1,{0x03}},
{0x6F,1,{0x92}},//7
{0x70,1,{0x03}},
{0x71,1,{0xa1}},//5
{0x72,1,{0x03}},
{0x73,1,{0xb2}},//3
{0x74,1,{0x03}},
{0x75,1,{0xb8}},//1
{0x76,1,{0x03}},
{0x77,1,{0xC2}},//0
{0x78,1,{0x01}},
{0x79,1,{0x01}},


{0xFF,5,{0xFF,0x78,0x02,0x00,0x03}},
{0x00,1,{0x00}},
{0x01,1,{0x8c}},	//255
{0x02,1,{0x00}},
{0x03,1,{0xd4}},//254
{0x04,1,{0x01}},
{0x05,1,{0x09}},//252
{0x06,1,{0x01}},
{0x07,1,{0x1a}},//250
{0x08,1,{0x01}},
{0x09,1,{0x30}},//248
{0x0A,1,{0x01}},
{0x0B,1,{0x3f}},//246
{0x0C,1,{0x01}},
{0x0D,1,{0x50}},//244
{0x0E,1,{0x01}},
{0x0F,1,{0x5b}},//242
{0x10,1,{0x01}},
{0x11,1,{0x69}},//240
{0x12,1,{0x01}},
{0x13,1,{0x91}},//232
{0x14,1,{0x01}},
{0x15,1,{0xb0}},//224
{0x16,1,{0x01}},
{0x17,1,{0xe1}},//208
{0x18,1,{0x02}},
{0x19,1,{0x0a}},//192
{0x1A,1,{0x02}},
{0x1B,1,{0x4a}},//160
{0x1C,1,{0x02}},
{0x1D,1,{0x79}},//128
{0x1E,1,{0x02}},
{0x1F,1,{0x7b}},//127
{0x20,1,{0x02}},
{0x21,1,{0xaa}},//95
{0x22,1,{0x02}},
{0x23,1,{0xde}},//63
{0x24,1,{0x02}},
{0x25,1,{0xfe}},//47
{0x26,1,{0x03}},
{0x27,1,{0x26}},//31
{0x28,1,{0x03}},
{0x29,1,{0x40}},//23
{0x2A,1,{0x03}},
{0x2B,1,{0x65}},//15
{0x2C,1,{0x03}},
{0x2D,1,{0x6C}},//13
{0x2E,1,{0x03}},
{0x2F,1,{0x78}},//11
{0x30,1,{0x03}},
{0x31,1,{0x83}},//9
{0x32,1,{0x03}},
{0x33,1,{0x92}},//7
{0x34,1,{0x03}},
{0x35,1,{0xa1}},//5
{0x36,1,{0x03}},
{0x37,1,{0xb2}},//3
{0x38,1,{0x03}},
{0x39,1,{0xb8}},//1
{0x3A,1,{0x03}},
{0x3B,1,{0xC2}},//0
{0x3C,1,{0x00}},
{0x3D,1,{0x8c}},//255
{0x3E,1,{0x00}},
{0x3F,1,{0xd4}},//254
{0x40,1,{0x01}},
{0x41,1,{0x09}},//252
{0x42,1,{0x01}},
{0x43,1,{0x1a}},//250
{0x44,1,{0x01}},
{0x45,1,{0x30}},//248
{0x46,1,{0x01}},
{0x47,1,{0x3f}},//246
{0x48,1,{0x01}},
{0x49,1,{0x50}},//244
{0x4A,1,{0x01}},
{0x4B,1,{0x5b}},//242
{0x4C,1,{0x01}},
{0x4D,1,{0x69}},//240
{0x4E,1,{0x01}},
{0x4F,1,{0x91}},//232
{0x50,1,{0x01}},
{0x51,1,{0xb0}},//224
{0x52,1,{0x01}},
{0x53,1,{0xe1}},//208
{0x54,1,{0x02}},
{0x55,1,{0x0a}},//192
{0x56,1,{0x02}},
{0x57,1,{0x4a}},//160
{0x58,1,{0x02}},
{0x59,1,{0x79}},//128
{0x5A,1,{0x02}},
{0x5B,1,{0x7b}},//127
{0x5C,1,{0x02}},
{0x5D,1,{0xaa}},//95
{0x5E,1,{0x02}},
{0x5F,1,{0xde}},//63
{0x60,1,{0x02}},
{0x61,1,{0xfe}},//47
{0x62,1,{0x03}},
{0x63,1,{0x26}},//31
{0x64,1,{0x03}},
{0x65,1,{0x40}},//23
{0x66,1,{0x03}},
{0x67,1,{0x65}},//15
{0x68,1,{0x03}},
{0x69,1,{0x6C}},//13
{0x6A,1,{0x03}},
{0x6B,1,{0x78}},//11
{0x6C,1,{0x03}},
{0x6D,1,{0x83}},//9
{0x6E,1,{0x03}},
{0x6F,1,{0x92}},//7
{0x70,1,{0x03}},
{0x71,1,{0xa1}},//5
{0x72,1,{0x03}},
{0x73,1,{0xb2}},//3
{0x74,1,{0x03}},
{0x75,1,{0xb8}},//1
{0x76,1,{0x03}},
{0x77,1,{0xC2}},//0
{0x78,1,{0x01}},
{0x79,1,{0x01}},


{0xFF,5,{0xFF,0x78,0x02,0x00,0x04}},
{0x00,1,{0x00}},
{0x01,1,{0x8c}},	//255
{0x02,1,{0x00}},
{0x03,1,{0xd4}},//254
{0x04,1,{0x01}},
{0x05,1,{0x09}},//252
{0x06,1,{0x01}},
{0x07,1,{0x1a}},//250
{0x08,1,{0x01}},
{0x09,1,{0x30}},//248
{0x0A,1,{0x01}},
{0x0B,1,{0x3f}},//246
{0x0C,1,{0x01}},
{0x0D,1,{0x50}},//244
{0x0E,1,{0x01}},
{0x0F,1,{0x5b}},//242
{0x10,1,{0x01}},
{0x11,1,{0x69}},//240
{0x12,1,{0x01}},
{0x13,1,{0x91}},//232
{0x14,1,{0x01}},
{0x15,1,{0xb0}},//224
{0x16,1,{0x01}},
{0x17,1,{0xe1}},//208
{0x18,1,{0x02}},
{0x19,1,{0x0a}},//192
{0x1A,1,{0x02}},
{0x1B,1,{0x4a}},//160
{0x1C,1,{0x02}},
{0x1D,1,{0x79}},//128
{0x1E,1,{0x02}},
{0x1F,1,{0x7b}},//127
{0x20,1,{0x02}},
{0x21,1,{0xaa}},//95
{0x22,1,{0x02}},
{0x23,1,{0xde}},//63
{0x24,1,{0x02}},
{0x25,1,{0xfe}},//47
{0x26,1,{0x03}},
{0x27,1,{0x26}},//31
{0x28,1,{0x03}},
{0x29,1,{0x40}},//23
{0x2A,1,{0x03}},
{0x2B,1,{0x65}},//15
{0x2C,1,{0x03}},
{0x2D,1,{0x6C}},//13
{0x2E,1,{0x03}},
{0x2F,1,{0x78}},//11
{0x30,1,{0x03}},
{0x31,1,{0x83}},//9
{0x32,1,{0x03}},
{0x33,1,{0x92}},//7
{0x34,1,{0x03}},
{0x35,1,{0xa1}},//5
{0x36,1,{0x03}},
{0x37,1,{0xb2}},//3
{0x38,1,{0x03}},
{0x39,1,{0xb8}},//1
{0x3A,1,{0x03}},
{0x3B,1,{0xC2}},//0
{0x3C,1,{0x00}},
{0x3D,1,{0x8c}},//255
{0x3E,1,{0x00}},
{0x3F,1,{0xd4}},//254
{0x40,1,{0x01}},
{0x41,1,{0x09}},//252
{0x42,1,{0x01}},
{0x43,1,{0x1a}},//250
{0x44,1,{0x01}},
{0x45,1,{0x30}},//248
{0x46,1,{0x01}},
{0x47,1,{0x3f}},//246
{0x48,1,{0x01}},
{0x49,1,{0x50}},//244
{0x4A,1,{0x01}},
{0x4B,1,{0x5b}},//242
{0x4C,1,{0x01}},
{0x4D,1,{0x69}},//240
{0x4E,1,{0x01}},
{0x4F,1,{0x91}},//232
{0x50,1,{0x01}},
{0x51,1,{0xb0}},//224
{0x52,1,{0x01}},
{0x53,1,{0xe1}},//208
{0x54,1,{0x02}},
{0x55,1,{0x0a}},//192
{0x56,1,{0x02}},
{0x57,1,{0x4a}},//160
{0x58,1,{0x02}},
{0x59,1,{0x79}},//128
{0x5A,1,{0x02}},
{0x5B,1,{0x7b}},//127
{0x5C,1,{0x02}},
{0x5D,1,{0xaa}},//95
{0x5E,1,{0x02}},
{0x5F,1,{0xde}},//63
{0x60,1,{0x02}},
{0x61,1,{0xfe}},//47
{0x62,1,{0x03}},
{0x63,1,{0x26}},//31
{0x64,1,{0x03}},
{0x65,1,{0x40}},//23
{0x66,1,{0x03}},
{0x67,1,{0x65}},//15
{0x68,1,{0x03}},
{0x69,1,{0x6C}},//13
{0x6A,1,{0x03}},
{0x6B,1,{0x78}},//11
{0x6C,1,{0x03}},
{0x6D,1,{0x83}},//9
{0x6E,1,{0x03}},
{0x6F,1,{0x92}},//7
{0x70,1,{0x03}},
{0x71,1,{0xa1}},//5
{0x72,1,{0x03}},
{0x73,1,{0xb2}},//3
{0x74,1,{0x03}},
{0x75,1,{0xb8}},//1
{0x76,1,{0x03}},
{0x77,1,{0xC2}},//0
{0x78,1,{0x01}},
{0x79,1,{0x01}},
	
	
//Generic_Long_Write_5P(0xFF,0xFF,0x78,0x02,0x00,0x05);
//Generic_Short_Write_1P(0x00,0x04);	     // PWM Frequency
	
{0xFF,5,{0xFF,0x78,0x02,0x00,0x00}},
//Generic_Short_Write_1P(0x51,0x80);	     // PWM duty max
//Generic_Short_Write_1P(0x53,0x2C);	     // CABC ON
//Generic_Short_Write_1P(0x55,0x01);	     // mode select    
	
{0x36,1,{0x0a}}, 
{0x11,	1,		{0x00}},       
{REGFLAG_DELAY, 120, {}},		
{0x29,	1,		{0x00}},	        
{REGFLAG_DELAY, 20, {}},	      
{REGFLAG_END_OF_TABLE, 0x00, {}}
#endif
};
static struct LCM_setting_table lcm_sleep_out_setting[] = {
    //Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
    {0x29, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
	//zhangaifeng@wind-mobi.com begin
//    {0xFF,	3,		{0x98,0x81,0x01}},
 //   {0x58, 1, {0x01}},
//   {REGFLAG_DELAY, 20, {}},
		//zhangaifeng@wind-mobi.com end
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;

            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{

		memset(params, 0, sizeof(LCM_PARAMS));
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		params->dbi.te_mode				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		//LCM_DBI_TE_MODE_DISABLED;
		//LCM_DBI_TE_MODE_VSYNC_ONLY;  
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING; 
		/////////////////////   
		//if(params->dsi.lcm_int_te_monitor)  
		//params->dsi.vertical_frontporch *=2;  
		//params->dsi.lcm_ext_te_monitor= 0;//TRUE;
		//zhounengwen@wind-mobi.com begin
		//params->dsi.noncont_clock= TRUE;//FALSE;   
		//params->dsi.noncont_clock_period=2;
		params->dsi.cont_clock=1;    //modify  yudengwu
		//zhounengwen@wind-mobi.com end
		////////////////////          
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;  
		// DSI    /* Command mode setting */  
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;      
		//The following defined the fomat for data coming from LCD engine.  
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;   
		params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST; 
		params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB;    
		params->dsi.data_format.format	  = LCM_DSI_FORMAT_RGB888;       
		// Video mode setting		   
		params->dsi.intermediat_buffer_num = 2;  
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;  
		params->dsi.packet_size=256;    
		// params->dsi.word_count=480*3;	
		//DSI CMD mode need set these two bellow params, different to 6577   
		// params->dsi.vertical_active_line=800;   
		params->dsi.vertical_sync_active				= 6; //4   
		params->dsi.vertical_backporch				       = 20;  //14  
		params->dsi.vertical_frontporch				       = 14;  //16  
		params->dsi.vertical_active_line				       = FRAME_HEIGHT;     
		params->dsi.horizontal_sync_active				= 60;   //4
		params->dsi.horizontal_backporch				= 100;  //60  
		params->dsi.horizontal_frontporch				= 100;    //60
		params->dsi.horizontal_blanking_pixel				= 60;   
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;  
		
	//	params->dsi.pll_div1=1;		   
	//	params->dsi.pll_div2=1;		   
	//	params->dsi.fbk_div =28;//28	
//liqiang@wind-mobi.com 20140820 beign
// To fix boot error
		params->dsi.PLL_CLOCK = 225;	   // 245;

	//	params->dsi.ss

//		params->dsi.CLK_TRAIL = 17;
			
//liqiang@wind-mobi.com 20140820 end
		params->dsi.esd_check_enable = 1;
                params->dsi.customization_esd_check_enable = 1;
                params->dsi.lcm_esd_check_table[0].cmd                  = 0x0a;
                params->dsi.lcm_esd_check_table[0].count                = 1;
                params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
}
	//zhangaifeng@wind-mobi.com begin
//#ifndef BUILD_LK

#if 0
#include <linux/kernel.h>
#include <linux/module.h>  
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
//#include <linux/jiffies.h>
#include <linux/uaccess.h>
//#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
/***************************************************************************** 
 * Define
 *****************************************************************************/
//#ifndef FPGA_EARLY_PORTING
#define TPS_I2C_BUSNUM  0//for I2C channel 0
#define I2C_ID_NAME "ili9881"
#define TPS_ADDR 0x3E
/***************************************************************************** 
 * GLobal Variable
 *****************************************************************************/
static struct i2c_board_info __initdata ili9881_board_info = {I2C_BOARD_INFO(I2C_ID_NAME, TPS_ADDR)};
static struct i2c_client *ili9881_i2c_client = NULL;


/***************************************************************************** 
 * Function Prototype
 *****************************************************************************/ 
static int ili9881_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int ili9881_remove(struct i2c_client *client);
/***************************************************************************** 
 * Data Structure
 *****************************************************************************/

 struct ili9881_dev	{	
	struct i2c_client	*client;
	
};

static const struct i2c_device_id ili9881_id[] = {
	{ I2C_ID_NAME, 0 },
	{ }
};

//#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36))
//static struct i2c_client_address_data addr_data = { .forces = forces,};
//#endif
static struct i2c_driver ili9881_iic_driver = {
	.id_table	= ili9881_id,
	.probe		= ili9881_probe,
	.remove		= ili9881_remove,
	//.detect		= mt6605_detect,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "ili9881",
	},
 
};
/***************************************************************************** 
 * Extern Area
 *****************************************************************************/ 
 
 

/***************************************************************************** 
 * Function
 *****************************************************************************/ 
static int ili9881_probe(struct i2c_client *client, const struct i2c_device_id *id)
{  
	printk( "ili9881_iic_probe\n");
	printk("TPS: info==>name=%s addr=0x%x\n",client->name,client->addr);
	ili9881_i2c_client  = client;		
	return 0;      
}


static int ili9881_remove(struct i2c_client *client)
{  	
  printk( "ili9881_remove\n");
  ili9881_i2c_client = NULL;
   i2c_unregister_device(client);
  return 0;
}


 static int ili9881_write_bytes(unsigned char addr, unsigned char value)
{	
	int ret = 0;
	struct i2c_client *client = ili9881_i2c_client;
	char write_data[2]={0};	
	write_data[0]= addr;
	write_data[1] = value;
    ret=i2c_master_send(client, write_data, 2);
	if(ret<0)
	printk("ili9881 write data fail !!\n");	
	return ret ;
}



/*
 * module load/unload record keeping
 */

static int __init ili9881_iic_init(void)
{

   printk( "ili9881_iic_init\n");
   i2c_register_board_info(TPS_I2C_BUSNUM, &ili9881_board_info, 1);
   printk( "ili9881_iic_init2\n");
   i2c_add_driver(&ili9881_iic_driver);
   printk( "ili9881_iic_init success\n");	
   return 0;
}

static void __exit ili9881_iic_exit(void)
{
  printk( "ili9881_iic_exit\n");
  i2c_del_driver(&ili9881_iic_driver);  
}


module_init(ili9881_iic_init);
module_exit(ili9881_iic_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK ili9881 I2C Driver");
MODULE_LICENSE("GPL"); 
//#endif
#endif

#if 0

#define ili9881_I2C_ID	I2C0
static struct mt_i2c_t ili9881_i2c;
#define ili9881_SLAVE_ADDR_WRITE   0x7c
#define ili9881_SLAVE_ADDR_READ    0x7d
#include <platform/mt_i2c.h>
/*
kal_uint32 ili9881_write_byte(kal_uint8 addr, kal_uint8 value)
{
    kal_uint32 ret_code = I2C_OK;
    kal_uint8 write_data[2];
    kal_uint16 len;

    write_data[0]= addr;
    write_data[1] = value;

    ili9881_i2c.id = ili9881_I2C_ID;
    // Since i2c will left shift 1 bit, we need to set ili9881 I2C address to >>1 
    ili9881_i2c.addr = (ili9881_SLAVE_ADDR_WRITE >> 1);
    ili9881_i2c.mode = ST_MODE;
    ili9881_i2c.speed = 100;
    len = 2;

    ret_code = i2c_write(&ili9881_i2c, write_data, len);

    if(I2C_OK != ret_code)
        dprintf(INFO, "%s: i2c_write: ret_code: %d\n", __func__, ret_code);

    return ret_code;
}
*/
kal_uint32 ili9881_read_byte (kal_uint8 addr, kal_uint8 *dataBuffer) 
{
    kal_uint32 ret_code = I2C_OK;
    kal_uint16 len;
    *dataBuffer = addr;

    ili9881_i2c.id = ili9881_I2C_ID;
    /* Since i2c will left shift 1 bit, we need to set ili9881 I2C address to >>1 */
    ili9881_i2c.addr = (ili9881_SLAVE_ADDR_READ >> 1);
    ili9881_i2c.mode = ST_MODE;
    ili9881_i2c.speed = 100;
    len = 1;

    ret_code = i2c_write_read(&ili9881_i2c, dataBuffer, len, len);

    if(I2C_OK != ret_code)
        dprintf(INFO, "%s: i2c_read: ret_code: %d\n", __func__, ret_code);

    return ret_code;
}
#endif

#if 0
//#ifndef FPGA_EARLY_PORTING
#define ili9881_SLAVE_ADDR_WRITE  0x7C  
static struct mt_i2c_t ili9881_i2c;

static int ili9881_write_byte(kal_uint8 addr, kal_uint8 value)
{
    kal_uint32 ret_code = I2C_OK;
    kal_uint8 write_data[2];
    kal_uint16 len;

    write_data[0]= addr;
    write_data[1] = value;

    ili9881_i2c.id = 0;//I2C2;
    /* Since i2c will left shift 1 bit, we need to set FAN5405 I2C address to >>1 */
    ili9881_i2c.addr = (ili9881_SLAVE_ADDR_WRITE >> 1);
    ili9881_i2c.mode = ST_MODE;
    ili9881_i2c.speed = 100;
    len = 2;

    ret_code = i2c_write(&ili9881_i2c, write_data, len);
    //printf("%s: i2c_write: ret_code: %d\n", __func__, ret_code);

    return ret_code;
}

#else
  
//	extern int mt8193_i2c_write(u16 addr, u32 data);
//	extern int mt8193_i2c_read(u16 addr, u32 *data);
	
//	#define ili9881_write_byte(add, data)  mt8193_i2c_write(add, data)
	//#define ili9881_read_byte(add)  mt8193_i2c_read(add)
  
//#endif
#endif

	//zhangaifeng@wind-mobi.com end
static void lcm_init(void)
{
	/*
             MDELAY(50);
             SET_RESET_PIN(1);
		MDELAY(10);
		SET_RESET_PIN(0);
		MDELAY(20); 
		SET_RESET_PIN(1);
		MDELAY(120); 
*/
			//AVDD power on
			//zhangaifeng@wind-mobi.com begin
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret=0;
	cmd=0x03;
	data=0x0;
			//zhangaifeng@wind-mobi.com end

	printk("[darren-ili7802]-------lcm_init\n");
			
	mt_set_gpio_mode(GPIO_LCM_PWR_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM_PWR_EN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ONE);
			MDELAY(10);

		//zhangaifeng@wind-mobi.com begin
	#ifdef BUILD_LK
		  //  ili9881_write_byte(0x03,0x01);
#endif
/*
#ifdef BUILD_LK
	ret=ili9881_write_byte(cmd,data);
    if(ret)    	
    dprintf(0, "[LK]ili9881----tps6132----cmd=%0x--i2c write error----\n",cmd);    	
	else
	dprintf(0, "[LK]ili9881----tps6132----cmd=%0x--i2c write success----\n",cmd);    		
#else
	ret=ili9881_write_bytes(cmd,data);
	if(ret<0)
	printk("[KERNEL]ili9881----tps6132---cmd=%0x-- i2c write error-----\n",cmd);
	else
	printk("[KERNEL]ili9881----tps6132---cmd=%0x-- i2c write success-----\n",cmd);
#endif
	//zhangaifeng@wind-mobi.com end
*/
	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
	
	      push_table(lcm_initialization_setting1, sizeof(lcm_initialization_setting1) / sizeof(struct LCM_setting_table), 1); 

}

static void lcm_suspend(void) 
{

	// when phone sleep , config output low, disable backlight drv chip  
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	
	mt_set_gpio_mode(GPIO_LCM_PWR_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM_PWR_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ZERO);
	MDELAY(10);

}

// zhuhuadong@wind-mobi.com begin 
/*#ifndef BUILD_LK
extern void mms_resume_enable(void);
void custom_tp_resume_adjust(void)
{
	#if CONFIG_MTK_MELFAS_MMS400
	mms_resume_enable();
	#endif
}
#endif //#ifndef BUILD_LK
// zhuhuadong@wind-mobi.com end*/

static void lcm_resume(void)
{
	lcm_init();
	// zhuhuadong@wind-mobi.com begin
	#ifndef BUILD_LK
	//custom_tp_resume_adjust();
	#endif
	// zhuhuadong@wind-mobi.com end
}

static unsigned int lcm_compare_id(void)
{
	unsigned int id=0,id1=0,id2=0;
	unsigned char buffer[3];
	unsigned int array[16];  

		mt_set_gpio_mode(GPIO_LCM_PWR_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM_PWR_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ONE);
	MDELAY(10);

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	
	SET_RESET_PIN(1);
	MDELAY(120); 


    array[0]=0x00063902;
    array[1]=0x0278FFFF;
    array[2]=0x00000100;
    dsi_set_cmdq(array, 3, 1);
    MDELAY(10); 
    
    array[0] = 0x00023700;// read id return two byte,version and id
    dsi_set_cmdq(array, 1, 1);
    MDELAY(10); 
    
    read_reg_v2(0x00, buffer, 1);
    id1 = buffer[0]; 
    
    read_reg_v2(0x01, buffer, 1);
    id2 = buffer[1];  
    
    id2 = buffer[1]; //we test buffer 1
	id = (id1 << 8) | id2;

    #ifdef BUILD_LK
		printf("[znw]%s, LK ili7802 debug: ili7802 id = %x,%x,%x\n", __func__, id,id1,id2);
    #else
		printk("[znw]%s, kernel ili7802 horse debug: ili7802 id = 0x%08x\n", __func__, id);
    #endif
//return 1;
    if(id == LCM_ID_ILI7802)
    	return 1;
    else
        return 0; 
}
static int err_count = 0;
static unsigned int lcm_esd_check(void)
{
  #ifndef BUILD_LK
    unsigned char buffer[8] = {0};

    unsigned int array[4];
    int i =0;

	if(lcm_esd_test)
	{
		lcm_esd_test = FALSE;
		return TRUE;
	}

    array[0] = 0x00013700;    

    dsi_set_cmdq(array, 1,1);

    read_reg_v2(0x0A, buffer,8);

	printk( "ili7802 lcm_esd_check: buffer[0] = %d,buffer[1] = %d,buffer[2] = %d,buffer[3] = %d,buffer[4] = %d,buffer[5] = %d,buffer[6] = %d,buffer[7] = %d\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7]);

    if((buffer[0] != 0x9C))/*LCD work status error,need re-initalize*/

    {

        printk( "ili7802 lcm_esd_check buffer[0] = %d\n",buffer[0]);

        return TRUE;

    }

    else

    {

#if 0
        if(buffer[3] != 0x02) //error data type is 0x02

        {
		//  is not 02, 
             err_count = 0;

        }

        else

        {
		// is 02, so ,
             if((buffer[4] == 0x40) || (buffer[5] == 0x80))
             {
			 // buffer[4] is not 0, || (huo),buffer[5] is 0x80.
			   err_count = 0;
             }
             else

             {
			// is  0,0x80,  
			   err_count++;
             }             

             if(err_count >=2 )
             {
			
                 err_count = 0;

                 printk( "ili9881 lcm_esd_check buffer[4] = %d , buffer[5] = %d\n",buffer[4],buffer[5]);

                 return TRUE;

             }

        }
#endif
        return FALSE;

    }
#endif
	
}
static unsigned int lcm_esd_recover(void)
{
    #ifndef BUILD_LK
    printk( "ili7802 lcm_esd_recover\n");
    #endif
	lcm_init();
//	lcm_resume();

	return TRUE;
}
LCM_DRIVER ili7802_hd720_dsi_vdo_yassy_lcm_drv =
{
    .name           	= "ili7802_hd720_dsi_vdo_yassy",
    .set_util_funcs 	= lcm_set_util_funcs,
    .get_params     	= lcm_get_params,
    .init           	= lcm_init,
    .suspend        	= lcm_suspend,
    .resume         	= lcm_resume,
    .compare_id     	= lcm_compare_id,
	.esd_check = lcm_esd_check,
	.esd_recover = lcm_esd_recover,
};
//late_initcall(lcm_init);
/* END PN: , Added by guohongjin, 2014.08.13*/
//guohongjin@wind-mobi.com 20140813 end
