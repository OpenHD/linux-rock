// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony IMX477 cameras.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd
 *
 * Based on Sony imx219 camera driver
 * Copyright (C) 2019-2020 Raspberry Pi (Trading) Ltd
 */
#include <asm/unaligned.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <linux/rk-camera-module.h>
#include <linux/version.h>

#define DRIVER_VERSION		KERNEL_VERSION(0, 0x0, 0x2)
#define IMX477_NAME			"imx477p"

static int debug = 1;
module_param(debug, int, 0644);

#define IMX477_REG_VALUE_08BIT		1
#define IMX477_REG_VALUE_16BIT		2

/* Chip ID */
#define IMX477_REG_CHIP_ID		0x0016
#define IMX477_CHIP_ID			0x0477

#define IMX477_REG_MODE_SELECT		0x0100
#define IMX477_MODE_STANDBY			0x00
#define IMX477_MODE_STREAMING		0x01

#define IMX477_REG_ORIENTATION		0x101

#define IMX477_XCLK_FREQ			24000000

#define IMX477_DEFAULT_LINK_FREQ	114000000

/* Pixel rate is fixed at 840MHz for all the modes */
#define IMX477_PIXEL_RATE			614400000

/* V_TIMING internal */
#define IMX477_REG_FRAME_LENGTH		0x0340
#define IMX477_FRAME_LENGTH_MAX		0xffdc

/* H_TIMING internal */
#define IMX477_REG_LINE_LENGTH		0x0342
#define IMX477_LINE_LENGTH_MAX		0xfff0

/* Long exposure multiplier */
#define IMX477_LONG_EXP_SHIFT_MAX	7
#define IMX477_LONG_EXP_SHIFT_REG	0x3100

/* Exposure control */
#define IMX477_REG_EXPOSURE			0x0202
#define IMX477_EXPOSURE_OFFSET		22
#define IMX477_EXPOSURE_MIN			4
#define IMX477_EXPOSURE_STEP		1
#define IMX477_EXPOSURE_DEFAULT		0x640
#define IMX477_EXPOSURE_MAX			(IMX477_FRAME_LENGTH_MAX - \
					 				IMX477_EXPOSURE_OFFSET)

/* Analog gain control */
#define IMX477_REG_ANALOG_GAIN		0x0204
#define IMX477_ANA_GAIN_MIN			0
#define IMX477_ANA_GAIN_MAX			978
#define IMX477_ANA_GAIN_STEP		1
#define IMX477_ANA_GAIN_DEFAULT		0x0

/* Digital gain control */
#define IMX477_REG_DIGITAL_GAIN		0x020e
#define IMX477_DGTL_GAIN_MIN		0x0100
#define IMX477_DGTL_GAIN_MAX		0xffff
#define IMX477_DGTL_GAIN_DEFAULT	0x0100
#define IMX477_DGTL_GAIN_STEP		1

/* Test Pattern Control */
#define IMX477_REG_TEST_PATTERN			0x0600
#define IMX477_TEST_PATTERN_DISABLE		0
#define IMX477_TEST_PATTERN_SOLID_COLOR	1
#define IMX477_TEST_PATTERN_COLOR_BARS	2
#define IMX477_TEST_PATTERN_GREY_COLOR	3
#define IMX477_TEST_PATTERN_PN9			4

/* Test pattern colour components */
#define IMX477_REG_TEST_PATTERN_R		0x0602
#define IMX477_REG_TEST_PATTERN_GR		0x0604
#define IMX477_REG_TEST_PATTERN_B		0x0606
#define IMX477_REG_TEST_PATTERN_GB		0x0608
#define IMX477_TEST_PATTERN_COLOUR_MIN	0
#define IMX477_TEST_PATTERN_COLOUR_MAX	0x0fff
#define IMX477_TEST_PATTERN_COLOUR_STEP	1
#define IMX477_TEST_PATTERN_R_DEFAULT	IMX477_TEST_PATTERN_COLOUR_MAX
#define IMX477_TEST_PATTERN_GR_DEFAULT	0
#define IMX477_TEST_PATTERN_B_DEFAULT	0
#define IMX477_TEST_PATTERN_GB_DEFAULT	0

/* Trigger mode */
#define IMX477_REG_MC_MODE			0x3f0b
#define IMX477_REG_MS_SEL			0x3041
#define IMX477_REG_XVS_IO_CTRL		0x3040
#define IMX477_REG_EXTOUT_EN		0x4b81

/* IMX477 native and active pixel array size. */
#define IMX477_NATIVE_WIDTH			4072U
#define IMX477_NATIVE_HEIGHT		3176U
#define IMX477_PIXEL_ARRAY_LEFT		8U
#define IMX477_PIXEL_ARRAY_TOP		16U
#define IMX477_PIXEL_ARRAY_WIDTH	4056U
#define IMX477_PIXEL_ARRAY_HEIGHT	3040U

#define IMX477P 1

struct imx477_reg {
	u16 address;
	u8 val;
};

struct imx477_reg_list {
	unsigned int num_of_regs;
	const struct imx477_reg *regs;
};

/* Mode : resolution and related config&values */
struct imx477_mode {

	u32 bus_fmt;

	/* Frame width */
	unsigned int width;

	/* Frame height */
	unsigned int height;

	/* H-timing in pixels */
	unsigned int line_length_pix;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* Highest possible framerate. */
	struct v4l2_fract max_fps;

	/* Default framerate. */
	struct v4l2_fract timeperframe_default;

	/* Default register values */
	struct imx477_reg_list reg_list;
};

//#if (IMX477P==0)
static const struct imx477_reg mode_common_regs[] = {
	{0X0103, 0X01},
	{0x0136, 0x18},
	{0x0137, 0x00},
	//Global Setting	
	{0x38A8, 0x1F},
	{0x38A9, 0xFF},
	{0x38AA, 0x1F},
	{0x38AB, 0xFF},
	{0x55D4, 0x00},
	{0x55D5, 0x00},
	{0x55D6, 0x07},
	{0x55D7, 0xFF},
	{0x55E8, 0x07},
	{0x55E9, 0xFF},
	{0x55EA, 0x00},
	{0x55EB, 0x00},
	{0x575C, 0x07},
	{0x575D, 0xFF},
	{0x575E, 0x00},
	{0x575F, 0x00},
	{0x5764, 0x00},
	{0x5765, 0x00},
	{0x5766, 0x07},
	{0x5767, 0xFF},
	{0x5974, 0x04},
	{0x5975, 0x01},
	{0x5F10, 0x09},
	{0x5F11, 0x92},
	{0x5F12, 0x32},
	{0x5F13, 0x72},
	{0x5F14, 0x16},
	{0x5F15, 0xBA},
	{0x5F17, 0x13},
	{0x5F18, 0x24},
	{0x5F19, 0x60},
	{0x5F1A, 0xE3},
	{0x5F1B, 0xAD},
	{0x5F1C, 0x74},
	{0x5F2D, 0x25},
	{0x5F5C, 0xD0},
	{0x6A22, 0x00},
	{0x6A23, 0x1D},
	{0x7BA8, 0x00},
	{0x7BA9, 0x00},
	{0x886B, 0x00},
	{0x9002, 0x0A},
	{0x9004, 0x1A},
	{0x9214, 0x93},
	{0x9215, 0x69},
	{0x9216, 0x93},
	{0x9217, 0x6B},
	{0x9218, 0x93},
	{0x9219, 0x6D},
	{0x921A, 0x57},
	{0x921B, 0x58},
	{0x921C, 0x57},
	{0x921D, 0x59},
	{0x921E, 0x57},
	{0x921F, 0x5A},
	{0x9220, 0x57},
	{0x9221, 0x5B},
	{0x9222, 0x93},
	{0x9223, 0x02},
	{0x9224, 0x93},
	{0x9225, 0x03},
	{0x9226, 0x93},
	{0x9227, 0x04},
	{0x9228, 0x93},
	{0x9229, 0x05},
	{0x922A, 0x98},
	{0x922B, 0x21},
	{0x922C, 0xB2},
	{0x922D, 0xDB},
	{0x922E, 0xB2},
	{0x922F, 0xDC},
	{0x9230, 0xB2},
	{0x9231, 0xDD},
	{0x9232, 0xB2},
	{0x9233, 0xE1},
	{0x9234, 0xB2},
	{0x9235, 0xE2},
	{0x9236, 0xB2},
	{0x9237, 0xE3},
	{0x9238, 0xB7},
	{0x9239, 0xB9},
	{0x923A, 0xB7},
	{0x923B, 0xBB},
	{0x923C, 0xB7},
	{0x923D, 0xBC},
	{0x923E, 0xB7},
	{0x923F, 0xC5},
	{0x9240, 0xB7},
	{0x9241, 0xC7},
	{0x9242, 0xB7},
	{0x9243, 0xC9},
	{0x9244, 0x98},
	{0x9245, 0x56},
	{0x9246, 0x98},
	{0x9247, 0x55},
	{0x9380, 0x00},
	{0x9381, 0x62},
	{0x9382, 0x00},
	{0x9383, 0x56},
	{0x9384, 0x00},
	{0x9385, 0x52},
	{0x9388, 0x00},
	{0x9389, 0x55},
	{0x938A, 0x00},
	{0x938B, 0x55},
	{0x938C, 0x00},
	{0x938D, 0x41},
	{0x5078, 0x01},
		
	//2Lane	
	//Full resolution 4056x3040 RAW10 12fps	@100MHz
	//H: 4056	
	//V: 3040	
	//MIPI setting	
		
	{0x0112, 0x0A},
	{0x0113, 0x0A},
	{0x0114, 0x01},
};

/* 12 mpix 10fps */
static const struct imx477_reg mode_4056x3040_regs[] = {
	//Frame Horizontal Clock Count	
	
	{0x0342, 0x49},
	{0x0343, 0xa8},
	{0x0350, 0x00},
		
	//Frame Vertical Clock Count	
		
	{0x0340, 0x0C},
	{0x0341, 0x1E},
	{0x3210, 0x00},
		
		
	//Visible Size	
		
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x0F},
	{0x0349, 0xD7},
	{0x034A, 0x0B},
	{0x034B, 0xDF},
		
		
	//Mode Setting	
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x00},
	{0x0901, 0x11},
	{0x0902, 0x00},
	{0x3140, 0x02},
		
		
	//Digital Crop & Scaling	
		
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040A, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x0F},
	{0x040D, 0xD8},
	{0x040E, 0x0B},
	{0x040F, 0xE0},
		
		
	//Output Crop	
	{0x034C, 0x0F},
	{0x034D, 0xD8},
	{0x034E, 0x0B},
	{0x034F, 0xE0},

	//114MHz
	{0x0301, 0x05},  //VT_PIX_CLK_DIV
	{0x0303, 0x02},  //VTSYCK_DIV
	{0x0305, 0x04},  //VT_PRE_PLL_DIV
	{0x0306, 0x01},  //VT_PLL_MULTIPLER
	{0x0307, 0x00},
	{0x0309, 0x08},  //OP_PIX_CLK_DIV
	{0x030b, 0x02},  //OPSYCK_DIV
	{0x030d, 0x02},  //OP_PRE_PLL_DIV
	{0x030e, 0x00},  //OP_PLL_MULTIPLER
	{0x030f, 0X98},  //0x96
	{0x0310, 0x01},
	{0x0820, 0x20},
	{0x0821, 0xD0},
	{0x0822, 0x00},
	{0x0823, 0x00},
		
		
	//Output Data Select Setting	
		
	{0x3E20, 0x01},
	{0x3E37, 0x00},
		
		
	//PowerSave Setting	
		
	{0x3F50, 0x00},
	{0x3F56, 0x00},
	{0x3F57, 0x82},
		
		
	//Other Setting	
		
		
	//Integration Time Setting	
		
	{0x0202, 0x0C},
	{0x0203, 0x08},
		
		
	//Gain Setting	
		
	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x020E, 0x01},
	{0x020F, 0x00},
	{0x0210, 0x01},
	{0x0211, 0x00},
	{0x0212, 0x01},
	{0x0213, 0x00},
	{0x0214, 0x01},
	{0x0215, 0x00},

	{0x0100, 0x01},
};
static const struct imx477_reg mode_3840x2160_regs[] = {

//Frame Horizontal Clock Count	
{0x0342, 0x34},
{0x0343, 0x80},

{0x0350, 0x00},
	
//Frame Vertical Clock Count	
	
{0x0340, 0x08},
{0x0341, 0xED},
{0x3210, 0x00},
	
	
//Visible Size	
	
{0x0344, 0x00},
{0x0345, 0x6C},
{0x0346, 0x01},
{0x0347, 0xB8},
{0x0348, 0x0F},
{0x0349, 0x6B},
{0x034A, 0x0A},
{0x034B, 0x27},
	
	
//Mode Setting	
	
{0x0220, 0x00},
{0x0221, 0x11},
{0x0381, 0x01},
{0x0383, 0x01},
{0x0385, 0x01},
{0x0387, 0x01},
{0x0900, 0x00},
{0x0901, 0x11},
{0x0902, 0x00},
{0x3140, 0x02},
	
//Digital Crop & Scaling	
	
{0x0401, 0x00},
{0x0404, 0x00},
{0x0405, 0x10},
{0x0408, 0x00},
{0x0409, 0x00},
{0x040A, 0x00},
{0x040B, 0x00},
{0x040C, 0x0F},
{0x040D, 0x00},
{0x040E, 0x08},
{0x040F, 0x70},
	
	
//Output Crop	
	
{0x034C, 0x0F},
{0x034D, 0x00},
{0x034E, 0x08},
{0x034F, 0x70},

//114MHz
{0x0301, 0x05},  //VT_PIX_CLK_DIV
{0x0303, 0x02},  //VTSYCK_DIV
{0x0305, 0x04},  //VT_PRE_PLL_DIV
{0x0306, 0x01},  //VT_PLL_MULTIPLER
{0x0307, 0x00},
{0x0309, 0x08},  //OP_PIX_CLK_DIV
{0x030b, 0x02},  //OPSYCK_DIV
{0x030d, 0x02},  //OP_PRE_PLL_DIV
{0x030e, 0x00},  //OP_PLL_MULTIPLER
{0x030f, 0X98},  //0x96
{0x0310, 0x01},
{0x0820, 0x20},
{0x0821, 0xD0},
{0x0822, 0x00},
{0x0823, 0x00},


	
//Output Data Select Setting	
	
{0x3E20, 0x01},
{0x3E37, 0x00},
	
	
//PowerSave Setting	
	
{0x3F50, 0x00},
{0x3F56, 0x00},
{0x3F57, 0x82},
	
	
//Other Setting	

	
//Integration Time Setting	
	
{0x0202, 0x08},
{0x0203, 0xEC},
	
	
//Gain Setting	
	
{0x0204, 0x00},
{0x0205, 0x00},
{0x020E, 0x01},
{0x020F, 0x00},
{0x0210, 0x01},
{0x0211, 0x00},
{0x0212, 0x01},
{0x0213, 0x00},
{0x0214, 0x01},
{0x0215, 0x00},

{0x0100, 0x01},

};

static const struct imx477_reg mode_1920x1080_regs[] = {
	//Frame Horizontal Clock Count  
		
	{0x0342, 0x20},  //0B
	{0x0343, 0x70},
	{0x0350, 0x00},
		
		
	//Frame Vertical Clock Count    
		
	{0x0340, 0x04},  //06
	{0x0341, 0xD0},
	{0x3210, 0x00},
		
		
	//Visible Size	
		
	{0x0344, 0x00},
	{0x0345, 0x60},
	{0x0346, 0x01},
	{0x0347, 0xB8},
	{0x0348, 0x0F},
	{0x0349, 0xCB},
	{0x034A, 0x0B},
	{0x034B, 0xDF},
		
		
	//Mode Setting	
		
	{0x00E3, 0x00},
	{0x00E4, 0x00},
	{0x00E5, 0x01},
	{0x00FC, 0x0A},
	{0x00FD, 0x0A},
	{0x00FE, 0x0A},
	{0x00FF, 0x0A},
	{0xE013, 0x00},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x22},
	{0x0902, 0x02},
	{0x3140, 0x02},
	{0x3241, 0x11},
	{0x3250, 0x03},
	{0x3E10, 0x00},
	{0x3E11, 0x00},
	{0x3F0D, 0x00},
	{0x3F42, 0x00},
	{0x3F43, 0x00},
		
		
	//Digital Crop & Scaling	
		
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040A, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x07},
	{0x040D, 0x80},
	{0x040E, 0x04},
	{0x040F, 0x38},
		
		
	//Output Crop	
		
	{0x034C, 0x07},
	{0x034D, 0x80},
	{0x034E, 0x04},
	{0x034F, 0x38},

	//114MHz
	{0x0301, 0x05},  //VT_PIX_CLK_DIV
	{0x0303, 0x02},  //VTSYCK_DIV
	{0x0305, 0x04},  //VT_PRE_PLL_DIV
	{0x0306, 0x01},  //VT_PLL_MULTIPLER
	{0x0307, 0x00},
	{0x0309, 0x08},  //OP_PIX_CLK_DIV
	{0x030b, 0x02},  //OPSYCK_DIV
	{0x030d, 0x02},  //OP_PRE_PLL_DIV
	{0x030e, 0x00},  //OP_PLL_MULTIPLER
	{0x030f, 0X98},  //0x96
	{0x0310, 0x01},
	{0x0820, 0x20},
	{0x0821, 0xD0},
	{0x0822, 0x00},
	{0x0823, 0x00},
	//Output Data Select Setting	
		
	{0x3E20, 0x01},
	{0x3E37, 0x00},
};
	
	
/* 2x2 binned. 40fps */
static const struct imx477_reg mode_2028x1520_regs[] = {
	{0x0342, 0x31},
	{0x0343, 0xc4},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x0f},
	{0x0349, 0xd7},
	{0x034a, 0x0b},
	{0x034b, 0xdf},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x12},
	{0x0902, 0x02},
	{0x3140, 0x02},
	{0x3c00, 0x00},
	{0x3c01, 0x03},
	{0x3c02, 0xa2},
	{0x3f0d, 0x01},
	{0x5748, 0x07},
	{0x5749, 0xff},
	{0x574a, 0x00},
	{0x574b, 0x00},
	{0x7b53, 0x01},
	{0x9369, 0x73},
	{0x936b, 0x64},
	{0x936d, 0x5f},
	{0x9304, 0x00},
	{0x9305, 0x00},
	{0x9e9a, 0x2f},
	{0x9e9b, 0x2f},
	{0x9e9c, 0x2f},
	{0x9e9d, 0x00},
	{0x9e9e, 0x00},
	{0x9e9f, 0x00},
	{0xa2a9, 0x60},
	{0xa2b7, 0x00},
	{0x0401, 0x01},
	{0x0404, 0x00},
	{0x0405, 0x20},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x0f},
	{0x040d, 0xd8},
	{0x040e, 0x0b},
	{0x040f, 0xe0},
	{0x034c, 0x07},
	{0x034d, 0xec},
	{0x034e, 0x05},
	{0x034f, 0xf0},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x04},
	{0x0306, 0x01},
	{0x0307, 0x5e},
	{0x0309, 0x0c},
	{0x030b, 0x02},
	{0x030d, 0x02},
	{0x030e, 0x00},
	{0x030f, 0x96},
	{0x0310, 0x01},
	{0x0820, 0x07},
	{0x0821, 0x08},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x080a, 0x00},
	{0x080b, 0x7f},
	{0x080c, 0x00},
	{0x080d, 0x4f},
	{0x080e, 0x00},
	{0x080f, 0x77},
	{0x0810, 0x00},
	{0x0811, 0x5f},
	{0x0812, 0x00},
	{0x0813, 0x57},
	{0x0814, 0x00},
	{0x0815, 0x4f},
	{0x0816, 0x01},
	{0x0817, 0x27},
	{0x0818, 0x00},
	{0x0819, 0x3f},
	{0xe04c, 0x00},
	{0xe04d, 0x7f},
	{0xe04e, 0x00},
	{0xe04f, 0x1f},
	{0x3e20, 0x01},
	{0x3e37, 0x00},
	{0x3f50, 0x00},
	{0x3f56, 0x01},
	{0x3f57, 0x6c},
};

/* 1080p cropped mode */
static const struct imx477_reg mode_2028x1080_regs[] = {
	{0x0342, 0x31},
	{0x0343, 0xc4},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x01},
	{0x0347, 0xb8},
	{0x0348, 0x0f},
	{0x0349, 0xd7},
	{0x034a, 0x0a},
	{0x034b, 0x27},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x12},
	{0x0902, 0x02},
	{0x3140, 0x02},
	{0x3c00, 0x00},
	{0x3c01, 0x03},
	{0x3c02, 0xa2},
	{0x3f0d, 0x01},
	{0x5748, 0x07},
	{0x5749, 0xff},
	{0x574a, 0x00},
	{0x574b, 0x00},
	{0x7b53, 0x01},
	{0x9369, 0x73},
	{0x936b, 0x64},
	{0x936d, 0x5f},
	{0x9304, 0x00},
	{0x9305, 0x00},
	{0x9e9a, 0x2f},
	{0x9e9b, 0x2f},
	{0x9e9c, 0x2f},
	{0x9e9d, 0x00},
	{0x9e9e, 0x00},
	{0x9e9f, 0x00},
	{0xa2a9, 0x60},
	{0xa2b7, 0x00},
	{0x0401, 0x01},
	{0x0404, 0x00},
	{0x0405, 0x20},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x0f},
	{0x040d, 0xd8},
	{0x040e, 0x04},
	{0x040f, 0x38},
	{0x034c, 0x07},
	{0x034d, 0xec},
	{0x034e, 0x04},
	{0x034f, 0x38},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x04},
	{0x0306, 0x01},
	{0x0307, 0x5e},
	{0x0309, 0x0c},
	{0x030b, 0x02},
	{0x030d, 0x02},
	{0x030e, 0x00},
	{0x030f, 0x96},
	{0x0310, 0x01},
	{0x0820, 0x07},
	{0x0821, 0x08},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x080a, 0x00},
	{0x080b, 0x7f},
	{0x080c, 0x00},
	{0x080d, 0x4f},
	{0x080e, 0x00},
	{0x080f, 0x77},
	{0x0810, 0x00},
	{0x0811, 0x5f},
	{0x0812, 0x00},
	{0x0813, 0x57},
	{0x0814, 0x00},
	{0x0815, 0x4f},
	{0x0816, 0x01},
	{0x0817, 0x27},
	{0x0818, 0x00},
	{0x0819, 0x3f},
	{0xe04c, 0x00},
	{0xe04d, 0x7f},
	{0xe04e, 0x00},
	{0xe04f, 0x1f},
	{0x3e20, 0x01},
	{0x3e37, 0x00},
	{0x3f50, 0x00},
	{0x3f56, 0x01},
	{0x3f57, 0x6c},
};

/* 4x4 binned. 120fps */
static const struct imx477_reg mode_1332x990_regs[] = {
	{0x420b, 0x01},
	{0x990c, 0x00},
	{0x990d, 0x08},
	{0x9956, 0x8c},
	{0x9957, 0x64},
	{0x9958, 0x50},
	{0x9a48, 0x06},
	{0x9a49, 0x06},
	{0x9a4a, 0x06},
	{0x9a4b, 0x06},
	{0x9a4c, 0x06},
	{0x9a4d, 0x06},
	{0x0112, 0x0C},
	{0x0113, 0x0C},
	{0x0114, 0x01},
	{0x0342, 0x1F},
	{0x0343, 0x08},
	{0x0340, 0x04},
	{0x0341, 0x18},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x02},
	{0x0347, 0x10},
	{0x0348, 0x0f},
	{0x0349, 0xd7},
	{0x034a, 0x09},
	{0x034b, 0xcf},
	{0x00e3, 0x00},
	{0x00e4, 0x00},
	{0x00fc, 0x0a},
	{0x00fd, 0x0a},
	{0x00fe, 0x0a},
	{0x00ff, 0x0a},
	{0xe013, 0x00},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x22},
	{0x0902, 0x02},
	{0x3140, 0x02},
	{0x3c00, 0x00},
	{0x3c01, 0x01},
	{0x3c02, 0x9c},
	{0x3f0d, 0x00},
	{0x5748, 0x00},
	{0x5749, 0x00},
	{0x574a, 0x00},
	{0x574b, 0xa4},
	{0x7b75, 0x0e},
	{0x7b76, 0x09},
	{0x7b77, 0x08},
	{0x7b78, 0x06},
	{0x7b79, 0x34},
	{0x7b53, 0x00},
	{0x9369, 0x73},
	{0x936b, 0x64},
	{0x936d, 0x5f},
	{0x9304, 0x03},
	{0x9305, 0x80},
	{0x9e9a, 0x2f},
	{0x9e9b, 0x2f},
	{0x9e9c, 0x2f},
	{0x9e9d, 0x00},
	{0x9e9e, 0x00},
	{0x9e9f, 0x00},
	{0xa2a9, 0x27},
	{0xa2b7, 0x03},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x0408, 0x01},
	{0x0409, 0x5c},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x05},
	{0x040d, 0x34},
	{0x040e, 0x03},
	{0x040f, 0xde},
	{0x034c, 0x05},
	{0x034d, 0x34},
	{0x034e, 0x03},
	{0x034f, 0xde},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x02},
	{0x0306, 0x00},
	{0x0307, 0xaf},
	{0x0309, 0x0a},
	{0x030b, 0x02},
	{0x030d, 0x02},
	{0x030e, 0x00},
	{0x030f, 0x96},
	{0x0310, 0x01},
	{0x0820, 0x07},
	{0x0821, 0x08},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x080a, 0x00},
	{0x080b, 0x7f},
	{0x080c, 0x00},
	{0x080d, 0x4f},
	{0x080e, 0x00},
	{0x080f, 0x77},
	{0x0810, 0x00},
	{0x0811, 0x5f},
	{0x0812, 0x00},
	{0x0813, 0x57},
	{0x0814, 0x00},
	{0x0815, 0x4f},
	{0x0816, 0x01},
	{0x0817, 0x27},
	{0x0818, 0x00},
	{0x0819, 0x3f},
	{0xe04c, 0x00},
	{0xe04d, 0x5f},
	{0xe04e, 0x00},
	{0xe04f, 0x1f},
	{0x3e20, 0x01},
	{0x3e37, 0x00},
	{0x3f50, 0x00},
	{0x3f56, 0x00},
	{0x3f57, 0xbf},
};

static const struct imx477_mode supported_modes[] = {
	
	{
		.bus_fmt = MEDIA_BUS_FMT_SRGGB10_1X10,
		/* 12MPix 10fps mode */ 
		.width = 4056,
		.height = 3040,
		.line_length_pix = 0x49a8,
		.max_fps = {
			.numerator = 100,
			.denominator = 1000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 1000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_4056x3040_regs),
			.regs = mode_4056x3040_regs,
		},
	},
	{
		.bus_fmt = MEDIA_BUS_FMT_SRGGB10_1X10,
		/* 4K 20fps mode */ 
		.width = 3840,
		.height = 2160,
		.line_length_pix = 0x3480,
		.max_fps = {
			.numerator = 100,
			.denominator = 2000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 2000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_3840x2160_regs),
			.regs = mode_3840x2160_regs,
		},
	},
	{
		.bus_fmt = MEDIA_BUS_FMT_SRGGB10_1X10,
		/* 1080p 50fps cropped mode */ 
		.width = 1920,
		.height = 1080,
		.line_length_pix = 0x2070,

		.max_fps = {
			.numerator = 100,
			.denominator = 6000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 6000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1920x1080_regs),
			.regs = mode_1920x1080_regs,
		},
	}
};

static const s64 link_freq_menu_items[] = {
	IMX477_DEFAULT_LINK_FREQ,
};

static const char * const imx477_test_pattern_menu[] = {
	"Disabled",
	"Color Bars",
	"Solid Color",
	"Grey Color Bars",
	"PN9"
};

static const int imx477_test_pattern_val[] = {
	IMX477_TEST_PATTERN_DISABLE,
	IMX477_TEST_PATTERN_COLOR_BARS,
	IMX477_TEST_PATTERN_SOLID_COLOR,
	IMX477_TEST_PATTERN_GREY_COLOR,
	IMX477_TEST_PATTERN_PN9,
};

/* regulator supplies */
static const char * const imx477_supply_name[] = {
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.05V) supply */
	"VDDL",  /* IF (1.8V) supply */
};

#define IMX477_NUM_SUPPLIES ARRAY_SIZE(imx477_supply_name)

/*
 * Initialisation delay between XCLR low->high and the moment when the sensor
 * can start capture (i.e. can leave software standby), given by T7 in the
 * datasheet is 8ms.  This does include I2C setup time as well.
 *
 * Note, that delay between XCLR low->high and reading the CCI ID register (T6
 * in the datasheet) is much smaller - 600us.
 */
#define IMX477_XCLR_MIN_DELAY_US	8000
#define IMX477_XCLR_DELAY_RANGE_US	1000

struct imx477_compatible_data {
	unsigned int chip_id;
	struct imx477_reg_list extra_regs;
};

struct imx477 {
	struct i2c_client	*client;
	struct clk *xclk;
	struct gpio_desc *reset_gpio;

	struct regulator_bulk_data supplies[IMX477_NUM_SUPPLIES];

	struct v4l2_subdev subdev;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl 	*exposure;
	struct v4l2_ctrl	*anal_a_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*pixel_rate;
	struct v4l2_ctrl 	*vflip;
	struct v4l2_ctrl 	*hflip;
	struct v4l2_ctrl	*link_freq;
	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;
	/* Streaming on/off */
	bool streaming;
	bool power_on;

	/* Current mode */
	const struct imx477_mode *cur_mode;
	/*module*/
	u32 		module_index;
	u32			cfg_num;
	const char *module_facing;
	const char *module_name;
	const char *len_name;
	unsigned int fmt_code;

	u32 xclk_freq;

	/* Rewrite common registers on stream on? */
	bool common_regs_written;

	/* Current long exposure factor in use. Set through V4L2_CID_VBLANK */
	unsigned int long_exp_shift;

	/* Any extra information related to different compatible sensors */
	const struct imx477_compatible_data *compatible_data;

};

static inline struct imx477 *to_imx477(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx477, subdev);
}

/* Read registers up to 2 at a time */
static int imx477_read_reg(struct imx477 *imx477, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = imx477->client;
	struct i2c_msg msgs[2];
	u8 addr_buf[2] = { reg >> 8, reg & 0xff };
	u8 data_buf[4] = { 0, };
	int ret;

	if (len > 4)
		return -EINVAL;

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_be32(data_buf);

	return 0;
}

/* Write registers up to 2 at a time */
static int imx477_write_reg(struct imx477 *imx477, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = imx477->client;
	u8 buf[6];

	if (len > 4)
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << (8 * (4 - len)), buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

/* Write a list of registers */
static int imx477_write_regs(struct imx477 *imx477,
			     const struct imx477_reg *regs, u32 len)
{
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = imx477_write_reg(imx477, regs[i].address, 1, regs[i].val);
		if (ret) {
			dev_err_ratelimited(&imx477->client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    regs[i].address, ret);

			return ret;
		}
	}

	return 0;
}

static int imx477_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx477 *imx477 = to_imx477(sd);
	struct v4l2_mbus_framefmt *try_fmt_img =
			v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct imx477_mode *def_mode = &supported_modes[0];

	mutex_lock(&imx477->mutex);

	/* Initialize try_fmt for the image pad */
	try_fmt_img->width = def_mode->width;
	try_fmt_img->height = def_mode->height;
	try_fmt_img->code = def_mode->bus_fmt;
	try_fmt_img->field = V4L2_FIELD_NONE;

	mutex_unlock(&imx477->mutex);

	return 0;
}

static void imx477_adjust_exposure_range(struct imx477 *imx477)
{
	int exposure_max, exposure_def;

	/* Honour the VBLANK limits when setting exposure. */
	exposure_max = imx477->cur_mode->height + imx477->vblank->val -
		       IMX477_EXPOSURE_OFFSET;
	exposure_def = min(exposure_max, imx477->exposure->val);
	__v4l2_ctrl_modify_range(imx477->exposure, imx477->exposure->minimum,
				 exposure_max, imx477->exposure->step,
				 exposure_def);
}

static int imx477_set_frame_length(struct imx477 *imx477, unsigned int val)
{
	int ret = 0;

	imx477->long_exp_shift = 0;

	while (val > IMX477_FRAME_LENGTH_MAX) {
		imx477->long_exp_shift++;
		val >>= 1;
	}

	ret = imx477_write_reg(imx477, IMX477_REG_FRAME_LENGTH,
			       IMX477_REG_VALUE_16BIT, val);
	if (ret)
		return ret;

	return imx477_write_reg(imx477, IMX477_LONG_EXP_SHIFT_REG,
				IMX477_REG_VALUE_08BIT, imx477->long_exp_shift);
}

static int imx477_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx477 *imx477 =
		container_of(ctrl->handler, struct imx477, ctrl_handler);
	struct i2c_client *client = imx477->client;
	int ret = 0;

	/*
	 * The VBLANK control may change the limits of usable exposure, so check
	 * and adjust if necessary.
	 */
	if (ctrl->id == V4L2_CID_VBLANK)
		imx477_adjust_exposure_range(imx477);

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	// if (!pm_runtime_get_if_in_use(&client->dev))
	// 	return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx477_write_reg(imx477, IMX477_REG_ANALOG_GAIN,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		 dev_info(&client->dev, "%s: set gain = val(%d)\n", __func__,  ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = imx477_write_reg(imx477, IMX477_REG_EXPOSURE,
				       IMX477_REG_VALUE_16BIT, ctrl->val >>
							imx477->long_exp_shift);
		dev_info(&client->dev, "%s: set exposure = val(%d)\n", __func__,  ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = imx477_write_reg(imx477, IMX477_REG_DIGITAL_GAIN,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = imx477_write_reg(imx477, IMX477_REG_TEST_PATTERN,
				       IMX477_REG_VALUE_16BIT,
				       imx477_test_pattern_val[ctrl->val]);
		break;
	case V4L2_CID_TEST_PATTERN_RED:
		ret = imx477_write_reg(imx477, IMX477_REG_TEST_PATTERN_R,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENR:
		ret = imx477_write_reg(imx477, IMX477_REG_TEST_PATTERN_GR,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_BLUE:
		ret = imx477_write_reg(imx477, IMX477_REG_TEST_PATTERN_B,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENB:
		ret = imx477_write_reg(imx477, IMX477_REG_TEST_PATTERN_GB,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = imx477_write_reg(imx477, IMX477_REG_ORIENTATION, 1,
				       imx477->hflip->val |
				       imx477->vflip->val << 1);
		break;
	case V4L2_CID_VBLANK:
		ret = imx477_set_frame_length(imx477,
					      imx477->cur_mode->height + ctrl->val);
		break;
	case V4L2_CID_HBLANK:
		ret = imx477_write_reg(imx477, IMX477_REG_LINE_LENGTH, 2,
				       imx477->cur_mode->width + ctrl->val);
		break;
	default:
		dev_info(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx477_ctrl_ops = {
	.s_ctrl = imx477_set_ctrl,
};

static int imx477_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx477 *imx477 = to_imx477(sd);

		if (code->index >= imx477->cfg_num)
		return -EINVAL;

	code->code = supported_modes[code->index].bus_fmt;

	return 0;
}

static int imx477_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx477 *imx477 = to_imx477(sd);

	v4l2_dbg(1, debug, sd, "%s: code = (0x%X), index = (%d)\n",
		 __func__, fse->code, fse->index);


		if (fse->index >= imx477->cfg_num)
			return -EINVAL;

		if (fse->code != supported_modes[fse->index].bus_fmt)
			return -EINVAL;

		fse->min_width = supported_modes[fse->index].width;
		fse->max_width = supported_modes[fse->index].width;
		fse->min_height = supported_modes[fse->index].height;
		fse->max_height = supported_modes[fse->index].height;

	return 0;
}

static int imx477_get_fmt(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_format *fmt)
{
	struct imx477 *imx477 = to_imx477(sd);
	const struct imx477_mode *mode = imx477->cur_mode;

	mutex_lock(&imx477->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&imx477->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = mode->bus_fmt;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	// v4l2_dbg(1, debug, sd, "%s: width: (%d) height: (%d) code: (0x%X)\n",
	// 	 __func__, fmt->format.width, fmt->format.height, fmt->format.code);
	mutex_unlock(&imx477->mutex);
	return 0;
}

static
unsigned int imx477_get_frame_length(const struct imx477_mode *mode,
				     const struct v4l2_fract *timeperframe)
{
	u64 frame_length;

	frame_length = (u64)timeperframe->numerator * IMX477_PIXEL_RATE;
	do_div(frame_length,
	       (u64)timeperframe->denominator * mode->line_length_pix);

	if (WARN_ON(frame_length > IMX477_FRAME_LENGTH_MAX))
		frame_length = IMX477_FRAME_LENGTH_MAX;

	return max_t(unsigned int, frame_length, mode->height);
}

static void imx477_set_framing_limits(struct imx477 *imx477)
{
	unsigned int frm_length_min, frm_length_default, hblank_min;
	const struct imx477_mode *mode = imx477->cur_mode;

	frm_length_min = imx477_get_frame_length(mode, &mode->max_fps);
	frm_length_default =
		     imx477_get_frame_length(mode, &mode->timeperframe_default);

	/* Default to no long exposure multiplier. */
	imx477->long_exp_shift = 0;

	/* Update limits and set FPS to default */
	__v4l2_ctrl_modify_range(imx477->vblank, frm_length_min - mode->height,
				 ((1 << IMX477_LONG_EXP_SHIFT_MAX) *
					IMX477_FRAME_LENGTH_MAX) - mode->height,
				 1, frm_length_default - mode->height);

	/* Setting this will adjust the exposure limits as well. */
	__v4l2_ctrl_s_ctrl(imx477->vblank, frm_length_default - mode->height);

	hblank_min = mode->line_length_pix - mode->width;
	__v4l2_ctrl_modify_range(imx477->hblank, hblank_min,
				 IMX477_LINE_LENGTH_MAX, 1, hblank_min);
	__v4l2_ctrl_s_ctrl(imx477->hblank, hblank_min);
	__v4l2_ctrl_s_ctrl_int64(imx477->pixel_rate,
					 IMX477_PIXEL_RATE);
}

static int imx477_get_reso_dist(const struct imx477_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct imx477_mode *
imx477_find_best_fit(struct imx477 *imx477, struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < imx477->cfg_num; i++) {
		dist = imx477_get_reso_dist(&supported_modes[i], framefmt);
		if ((cur_best_fit_dist == -1 || dist < cur_best_fit_dist) &&
			supported_modes[i].bus_fmt == framefmt->code) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}
	dev_info(&imx477->client->dev, "%s: cur_best_fit(%d)",
		 __func__, cur_best_fit);

	return &supported_modes[cur_best_fit];
}

static int imx477_set_fmt(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_format *fmt)
{
	struct imx477 *imx477 = to_imx477(sd);
	const struct imx477_mode *mode;

	mutex_lock(&imx477->mutex);

	mode = imx477_find_best_fit(imx477, fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		 fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&imx477->mutex);
		return -ENOTTY;
#endif
	} else if (imx477->cur_mode != mode) {
		imx477->cur_mode = mode;
		dev_dbg(&imx477->client->dev, "set fmt: cur_mode: %dx%d\n",
		mode->width, mode->height);
		imx477_set_framing_limits(imx477);
	}
	// dev_info(&imx477->client->dev, "%s: mode->mipi_freq_idx(%d)",
	// 	 __func__, mode->mipi_freq_idx);
	mutex_unlock(&imx477->mutex);

	return 0;
}

static int im477_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct imx477 *imx477 = to_imx477(sd);
	const struct imx477_mode *mode = imx477->cur_mode;

	mutex_lock(&imx477->mutex);
	fi->interval = mode->timeperframe_default;
	mutex_unlock(&imx477->mutex);

	return 0;
}

#define CROP_START(SRC, DST) (((SRC) - (DST)) / 2 / 4 * 4)
#define DST_WIDTH_4048 4048
#define DST_HEIGHT_3040 3040
#define DST_WIDTH_1920 1920
#define DST_HEIGHT_1080 1080

// static int imx477_get_selection(struct v4l2_subdev *sd,
// 				struct v4l2_subdev_pad_config *cfg,
// 				struct v4l2_subdev_selection *sel)
// {
// 	if(sel->target == V4L2_SEL_TGT_CROP_BOUNDS) {
// 		if (imx477->cur_mode->width == 3864) {
// 		} else if (imx477->cur_mode->width == 1944) {
// 		} else {
// 		}
// 		return 0;
// 	}

// 	return -EINVAL;
// }

/* Start streaming */
static int __imx477_start_streaming(struct imx477 *imx477)
{
	const struct imx477_reg_list *reg_list;
	const struct imx477_reg_list *extra_regs;
	int ret;

	if (!imx477->common_regs_written) {
		ret = imx477_write_regs(imx477, mode_common_regs,
					ARRAY_SIZE(mode_common_regs));
		if (!ret) {
			extra_regs = &imx477->compatible_data->extra_regs;
			ret = imx477_write_regs(imx477,	extra_regs->regs,
						extra_regs->num_of_regs);
		}

		if (ret) {
			dev_err(&imx477->client->dev, "%s failed to set common settings\n",
				__func__);
			return ret;
		}
		imx477->common_regs_written = true;
	}

	/* Apply default values of current mode */
	reg_list = &imx477->cur_mode->reg_list;
	ret = imx477_write_regs(imx477, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&imx477->client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}
	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(imx477->subdev.ctrl_handler);
	if (ret)
		return ret;

	/* set stream on register */
	return imx477_write_reg(imx477, IMX477_REG_MODE_SELECT,
				IMX477_REG_VALUE_08BIT, IMX477_MODE_STREAMING);
}

/* Stop streaming */
static void __imx477_stop_streaming(struct imx477 *imx477)
{
	struct i2c_client *client = imx477->client;
	int ret;

	/* set stream off register */
	ret = imx477_write_reg(imx477, IMX477_REG_MODE_SELECT,
			       IMX477_REG_VALUE_08BIT, IMX477_MODE_STANDBY);
	if (ret)
		dev_err(&client->dev, "%s failed to set stream\n", __func__);
}

static int imx477_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx477 *imx477 = to_imx477(sd);
	struct i2c_client *client = imx477->client;
	int ret = 0;

	mutex_lock(&imx477->mutex);
	if (imx477->streaming == enable) {
		mutex_unlock(&imx477->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto err_unlock;
		}

		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = __imx477_start_streaming(imx477);
		if (ret)
			goto err_rpm_put;
	} else {
		__imx477_stop_streaming(imx477);
		pm_runtime_put(&client->dev);
	}

	imx477->streaming = enable;

	/* vflip and hflip cannot change during streaming */
	__v4l2_ctrl_grab(imx477->vflip, enable);
	__v4l2_ctrl_grab(imx477->hflip, enable);

	mutex_unlock(&imx477->mutex);

	return ret;

err_rpm_put:
	pm_runtime_put(&client->dev);
err_unlock:
	mutex_unlock(&imx477->mutex);

	return ret;
}


static int imx477_s_power(struct v4l2_subdev *sd, int on)
{
	struct imx477 *imx477 = to_imx477(sd);
	struct i2c_client *client = imx477->client;
	int ret = 0;

	mutex_lock(&imx477->mutex);

	if (imx477->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}
		imx477->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		imx477->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&imx477->mutex);

	return ret;
}

/* Power/clock management functions */
static int __imx477_power_on(struct imx477 *imx477)
{
	int ret;

	ret = regulator_bulk_enable(IMX477_NUM_SUPPLIES,
				    imx477->supplies);
	if (ret) {
		dev_err(&imx477->client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(imx477->xclk);
	if (ret) {
		dev_err(&imx477->client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}

	gpiod_set_value_cansleep(imx477->reset_gpio, 1);
	usleep_range(IMX477_XCLR_MIN_DELAY_US,
		     IMX477_XCLR_MIN_DELAY_US + IMX477_XCLR_DELAY_RANGE_US);

	return 0;

reg_off:
	regulator_bulk_disable(IMX477_NUM_SUPPLIES, imx477->supplies);
	return ret;
}

static int __imx477_power_off(struct imx477 *imx477)
{
	if (!IS_ERR(imx477->reset_gpio))
		gpiod_direction_output(imx477->reset_gpio, 1);
	clk_disable_unprepare(imx477->xclk);

	regulator_bulk_disable(IMX477_NUM_SUPPLIES, imx477->supplies);

	/* Force reprogramming of the common registers when powered up again. */
	imx477->common_regs_written = false;

	return 0;
}

static int __maybe_unused imx477_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx477 *imx477 = to_imx477(sd);

	__imx477_power_off(imx477);

	return 0;
}

static int __maybe_unused imx477_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx477 *imx477 = to_imx477(sd);
	int ret;

	ret = __imx477_power_on(imx477);
	if (ret)
		goto error;

	return 0;

error:
	__imx477_stop_streaming(imx477);
	imx477->streaming = 0;
	return ret;
}

static int imx477_configure_regulators(struct imx477 *imx477)
{
	unsigned int i;

	for (i = 0; i < IMX477_NUM_SUPPLIES; i++)
		imx477->supplies[i].supply = imx477_supply_name[i];

	return devm_regulator_bulk_get(&imx477->client->dev,
				       IMX477_NUM_SUPPLIES,
				       imx477->supplies);
}

/* Verify chip ID */
static int imx477_identify_module(struct imx477 *imx477, u32 expected_id)
{
	int ret;
	u32 val;

	ret = imx477_read_reg(imx477, IMX477_REG_CHIP_ID,
			      IMX477_REG_VALUE_16BIT, &val);
	if (ret) {
		dev_err(&imx477->client->dev, "failed to read chip id %x, with error %d\n",
			expected_id, ret);
		return ret;
	}

	if (val != expected_id) {
		dev_err(&imx477->client->dev, "chip id mismatch: %x!=%x\n",
			expected_id, val);
		return -EIO;
	}

	dev_info(&imx477->client->dev, "Device found is imx%x\n", val);

	return 0;
}

static int imx477_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				struct v4l2_mbus_config *config)
{
	u32 val = 0;

	val = 1 << 1/*(imx477->lanes - 1)*/| V4L2_MBUS_CSI2_CHANNEL_0 |
	      V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
		val |= V4L2_MBUS_CSI2_CHANNEL_1;
	config->type = V4L2_MBUS_CSI2_DPHY;
	config->flags = val;

	return 0;
}

static void imx477_get_module_inf(struct imx477 *imx477,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, IMX477_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, imx477->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, imx477->len_name, sizeof(inf->base.lens));

	v4l2_dbg(1, debug, &imx477->subdev,"%s: get_module_inf:%s, %s, %s.\n", __func__,
		inf->base.sensor, inf->base.module, inf->base.lens);
}

static long imx477_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct imx477 *imx477 = to_imx477(sd);
	long ret = 0;
	switch (cmd)
	{
	case RKMODULE_GET_MODULE_INFO:
		imx477_get_module_inf(imx477, (struct rkmodule_inf *)arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}

#ifdef CONFIG_COMPAT
static long imx477_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	long ret;
	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = imx477_ioctl(sd, cmd, inf);
		if (!ret) {
			if (copy_to_user(up, inf, sizeof(*inf))) {
				kfree(inf);
				return -EFAULT;
			}
		}
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
		if (!cfg) {
			ret = -ENOMEM;
			return ret;
		}

		if (copy_from_user(cfg, up, sizeof(*cfg))) {
			kfree(cfg);
			return -EFAULT;
		}
		ret = imx477_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;

}
#endif

static const struct v4l2_subdev_core_ops imx477_core_ops = {
	.s_power = imx477_s_power,
	.ioctl = imx477_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = imx477_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops imx477_video_ops = {
	.s_stream = imx477_s_stream,
	.g_frame_interval = im477_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops imx477_pad_ops = {
	.enum_mbus_code = imx477_enum_mbus_code,
	.enum_frame_size = imx477_enum_frame_size,
	.get_fmt = imx477_get_fmt,
	.set_fmt = imx477_set_fmt,
	// .get_selection = imx477_get_selection,
	.set_mbus_config = imx477_g_mbus_config,
};

static const struct v4l2_subdev_ops imx477_subdev_ops = {
	.core = &imx477_core_ops,
	.video = &imx477_video_ops,
	.pad = &imx477_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx477_internal_ops = {
	.open = imx477_open,
};

/* Initialize control handlers */
static int imx477_init_controls(struct imx477 *imx477)
{
	const struct imx477_mode *mode;
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct v4l2_fwnode_device_properties props;
	unsigned int i;
	int ret;

	ctrl_hdlr = &imx477->ctrl_handler;
	mode = imx477->cur_mode;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 16);
	if (ret)
		return ret;

	ctrl_hdlr->lock = &imx477->mutex;

	/* freq */
	imx477->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr,NULL, V4L2_CID_LINK_FREQ,
			       0, 0, link_freq_menu_items);
	v4l2_ctrl_s_ctrl(imx477->link_freq,0);


	/* By default, PIXEL_RATE is read only */
	imx477->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       IMX477_PIXEL_RATE,
					       IMX477_PIXEL_RATE, 1,
					       IMX477_PIXEL_RATE);
	imx477->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					   V4L2_CID_VBLANK, 0, 0xffff, 1, 0);
	imx477->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					   V4L2_CID_HBLANK, 0, 0xffff, 1, 0);

	imx477->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX477_EXPOSURE_MIN,
					     IMX477_EXPOSURE_MAX,
					     IMX477_EXPOSURE_STEP,
					     IMX477_EXPOSURE_DEFAULT);

	imx477->anal_a_gain = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX477_ANA_GAIN_MIN, IMX477_ANA_GAIN_MAX,
			  IMX477_ANA_GAIN_STEP, IMX477_ANA_GAIN_DEFAULT);

	imx477->digi_gain = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX477_DGTL_GAIN_MIN, IMX477_DGTL_GAIN_MAX,
			  IMX477_DGTL_GAIN_STEP, IMX477_DGTL_GAIN_DEFAULT);

	imx477->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (imx477->hflip)
		imx477->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	imx477->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (imx477->vflip)
		imx477->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &imx477_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx477_test_pattern_menu) - 1,
				     0, 0, imx477_test_pattern_menu);
	for (i = 0; i < 4; i++) {
		/*
		 * The assumption is that
		 * V4L2_CID_TEST_PATTERN_GREENR == V4L2_CID_TEST_PATTERN_RED + 1
		 * V4L2_CID_TEST_PATTERN_BLUE   == V4L2_CID_TEST_PATTERN_RED + 2
		 * V4L2_CID_TEST_PATTERN_GREENB == V4L2_CID_TEST_PATTERN_RED + 3
		 */
		v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
				  V4L2_CID_TEST_PATTERN_RED + i,
				  IMX477_TEST_PATTERN_COLOUR_MIN,
				  IMX477_TEST_PATTERN_COLOUR_MAX,
				  IMX477_TEST_PATTERN_COLOUR_STEP,
				  IMX477_TEST_PATTERN_COLOUR_MAX);
		/* The "Solid color" pattern is white by default */
	}

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&imx477->client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&imx477->client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx477_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	/* Setup exposure and frame/line length limits. */
	imx477_set_framing_limits(imx477);

	imx477->subdev.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	mutex_destroy(&imx477->mutex);

	return ret;
}

static int imx477_check_hwcfg(struct device *dev)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	int ret = -EINVAL;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg)) {
		dev_err(dev, "could not parse endpoint\n");
		goto error_out;
	}

	/* Check the number of MIPI CSI2 data lanes */
	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 2) {
		dev_err(dev, "only 2 data lanes are currently supported\n");
		goto error_out;
	}

	/* Check the link frequency set in device tree */
	if (!ep_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		goto error_out;
	}

	ret = 0;

error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);

	return ret;
}


static const struct imx477_compatible_data imx477_compatible = {
	.chip_id = IMX477_CHIP_ID,
	.extra_regs = {
		.num_of_regs = 0,
		.regs = NULL
	}
};

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id imx477_of_match[] = {
	{ .compatible = "arducam,imx477p" , .data = &imx477_compatible },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, imx477_of_match);
#endif

static const struct i2c_device_id imx477_match_id[] = {
	{"imx477p",0},
	{},
};

static int imx477_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct imx477 *imx477;
	const struct of_device_id *match;
	struct v4l2_subdev *sd;

	char facing[2];
	int ret;
	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	imx477 = devm_kzalloc(dev, sizeof(struct imx477), GFP_KERNEL);
	if (!imx477)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &imx477->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &imx477->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &imx477->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &imx477->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	imx477->client = client;
	imx477->cur_mode = &supported_modes[0];
	imx477->cfg_num = ARRAY_SIZE(supported_modes);

	match = of_match_device(imx477_of_match, dev);
	if (!match)
		return -ENODEV;
	imx477->compatible_data =
		(const struct imx477_compatible_data *)match->data;

	/* Check the hardware configuration in device tree */
	if (imx477_check_hwcfg(dev))
		return -EINVAL;

	/* Get system clock (xclk) */
	imx477->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(imx477->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(imx477->xclk);
	}

	imx477->xclk_freq = clk_get_rate(imx477->xclk);
	if (imx477->xclk_freq != IMX477_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			imx477->xclk_freq);
		return -EINVAL;
	}

	/* Request optional enable pin */
	imx477->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_ASIS);
	if (IS_ERR(imx477->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	ret = imx477_configure_regulators(imx477);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	mutex_init(&imx477->mutex);

	sd = &imx477->subdev;
	v4l2_i2c_subdev_init(sd, client, &imx477_subdev_ops);
	ret = imx477_init_controls(imx477);
	if (ret)
		goto error_destroy_mutex;
	/*
	 * The sensor must be powered for imx477_identify_m()
	 * to be able to read the CHIP_ID register
	 */
	ret = __imx477_power_on(imx477);
	if (ret)
		goto error_handler_free;

	ret = imx477_identify_module(imx477, imx477->compatible_data->chip_id);
	if (ret)
		goto error_power_off;
			

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	/* Initialize subdev */
	sd->internal_ops = &imx477_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	/* Initialize source pads */
	imx477->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &imx477->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_power_off;
	}
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(imx477->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), 
		 "m%02d_%s_%s %s", imx477->module_index, facing,
		 IMX477_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_clean_entity;
	}

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

error_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
error_power_off:
	__imx477_power_off(imx477);
error_handler_free:
	v4l2_ctrl_handler_free(&imx477->ctrl_handler);
error_destroy_mutex:
	mutex_destroy(&imx477->mutex);

	return ret;
}

static int imx477_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx477 *imx477 = to_imx477(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&imx477->ctrl_handler);
	mutex_destroy(&imx477->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__imx477_power_off(imx477);
	pm_runtime_set_suspended(&client->dev);
    return 0;
}

static const struct dev_pm_ops imx477_pm_ops = {
	// SET_SYSTEM_SLEEP_PM_OPS(imx477_suspend, imx477_resume)
	SET_RUNTIME_PM_OPS(imx477_suspend, imx477_resume, NULL)
};

static struct i2c_driver imx477_i2c_driver = {
	.driver = {
		.name = IMX477_NAME,
		.pm = &imx477_pm_ops,
		.of_match_table	= of_match_ptr(imx477_of_match),
	},
	.probe = imx477_probe,
	.remove = imx477_remove,
	.id_table = imx477_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&imx477_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&imx477_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("Sony IMX477 sensor driver");
MODULE_LICENSE("GPL v2");