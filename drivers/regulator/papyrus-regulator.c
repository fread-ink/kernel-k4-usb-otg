/*
 * Copyright (C) 2010-2011 Amazon.com, Inc. All Rights Reserved.
 * Manish Lachwani (lachwani@amazon.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/papyrus.h>
#include <linux/gpio.h>
#include <asm/div64.h>

/*
 * PMIC Register Addresses
 */
enum {
    REG_PAPYRUS_EXT_TEMP = 0x0,
    REG_PAPYRUS_ENABLE,
    REG_PAPYRUS_CONFIG,
    REG_PAPYRUS_DVR = 0x4,
    REG_PAPYRUS_INT_TEMP = 0x5,
    REG_PAPYRUS_STATUS,
    REG_PAPYRUS_PRODUCT_ID,
    REG_PAPYRUS_FAULT,  /*0x0A*/
    REG_PAPYRUS_HVINP,
    REG_PAPYRUS_PRGM_CTRL,
    REG_PAPYRUS_TMST_CONFIG = 0x0C,
    REG_PAPYRUS_TIMING1,
    REG_PAPYRUS_TIMING2,
    REG_PAPYRUS_PG_GOOD = 0x0f,
    REG_PAPYRUS_PRODUCT_REV = 0x10,
    REG_PAPYRUS_TIMING4,
    REG_PAPYRUS_TIMING5,
    REG_PAPYRUS_TIMING6,
    REG_PAPYRUS_TIMING7,
    REG_PAPYRUS_TIMING8,
};
#define PAPYRUS_REG_NUM        21
#define PAPYRUS_MAX_REGISTER   0xFF

#define PAPYRUS_ENABLE_REGS		0xbf
#define PAPYRUS_STANDBY_REGS	0x7f	

#define PAPYRUS_TEMP_THRESHOLD	60000	/* 60 seconds */
#define PAPYRUS_VCOM_FACTOR	10.78

#define PAPYRUS_DISPLAY_SETTLE_TIME 15 /* 15 ms */
#define PAPYRUS_SUSPEND_DISPLAY_SETTLE_TIME	30 /* 30ms */
#define PAPYRUS_MAX_ENABLE_RETRY 2 /* Number of retries to enable */

/*
 * Bitfield macros that use rely on bitfield width/shift information.
 */
#define BITFMASK(field) (((1U << (field ## _WID)) - 1) << (field ## _LSH))
#define BITFVAL(field, val) ((val) << (field ## _LSH))
#define BITFEXT(var, bit) ((var & BITFMASK(bit)) >> (bit ## _LSH))

/*
 * Shift and width values for each register bitfield
 */
#define EXT_TEMP_LSH    7
#define EXT_TEMP_WID    9

#define THERMAL_SHUTDOWN_LSH    0
#define THERMAL_SHUTDOWN_WID    1

#define INT_TEMP_LSH    7
#define INT_TEMP_WID    9

#define STAT_BUSY_LSH   0
#define STAT_BUSY_WID   1
#define STAT_OPEN_LSH   1
#define STAT_OPEN_WID   1
#define STAT_SHRT_LSH   2
#define STAT_SHRT_WID   1

#define PROD_REV_LSH    0
#define PROD_REV_WID    8

#define PROD_ID_LSH     0
#define PROD_ID_WID     8

#define DVR_LSH         0
#define DVR_WID         8

#define ENABLE_LSH      7
#define ENABLE_WID      1

#define STANDBY_LSH	6
#define STANDBY_WID	1

#define VCOM_ENABLE_LSH 4
#define VCOM_ENABLE_WID 1

#define FAULT_FBPG_LSH      0
#define FAULT_FBPG_WID      1
#define FAULT_HVINP_LSH     1
#define FAULT_HVINP_WID     1
#define FAULT_HVINN_LSH     2
#define FAULT_HVINN_WID     1
#define FAULT_FBNG_LSH      3
#define FAULT_FBNG_WID      1
#define FAULT_HVINPSC_LSH   4
#define FAULT_HVINPSC_WID   1
#define FAULT_HVINNSC_LSH   5
#define FAULT_HVINNSC_WID   1
#define FAULT_OT_LSH        6
#define FAULT_OT_WID        1
#define FAULT_POK_LSH       7
#define FAULT_POK_WID       1

#define HVINP_LSH           0
#define HVINP_WID           4

#define CTRL_DVR_LSH        0
#define CTRL_DVR_WID        1
#define CTRL_TIMING_LSH     1
#define CTRL_TIMING_WID     1

#define TIMING1_LSH         0
#define TIMING1_WID         8
#define TIMING2_LSH         0
#define TIMING2_WID         8
#define TIMING3_LSH         0
#define TIMING3_WID         8
#define TIMING4_LSH         0
#define TIMING4_WID         8
#define TIMING5_LSH         0
#define TIMING5_WID         8
#define TIMING6_LSH         0
#define TIMING6_WID         8
#define TIMING7_LSH         0
#define TIMING7_WID         8
#define TIMING8_LSH         0
#define TIMING8_WID         8

/*
 * Regulator definitions
 *   *_MIN_uV  - minimum microvolt for regulator
 *   *_MAX_uV  - maximum microvolt for regulator
 *   *_STEP_uV - microvolts between regulator output levels
 *   *_MIN_VAL - minimum register field value for regulator
 *   *_MAX_VAL - maximum register field value for regulator
 */
#define PAPYRUS_HVINP_MIN_uV    5000000
#define PAPYRUS_HVINP_MAX_uV   20000000
#define PAPYRUS_HVINP_STEP_uV   1000000
#define PAPYRUS_HVINP_MIN_VAL         0
#define PAPYRUS_HVINP_MAX_VAL         1

#define PAPYRUS_HVINN_MIN_uV    5000000
#define PAPYRUS_HVINN_MAX_uV   20000000
#define PAPYRUS_HVINN_STEP_uV   1000000
#define PAPYRUS_HVINN_MIN_VAL         0
#define PAPYRUS_HVINN_MAX_VAL         1

#define PAPYRUS_GVDD_MIN_uV    5000000
#define PAPYRUS_GVDD_MAX_uV   20000000
#define PAPYRUS_GVDD_STEP_uV   1000000
#define PAPYRUS_GVDD_MIN_VAL         0
#define PAPYRUS_GVDD_MAX_VAL         1

#define PAPYRUS_GVEE_MIN_uV    5000000
#define PAPYRUS_GVEE_MAX_uV   20000000
#define PAPYRUS_GVEE_STEP_uV   1000000
#define PAPYRUS_GVEE_MIN_VAL         0
#define PAPYRUS_GVEE_MAX_VAL         1

#define PAPYRUS_VCOM_MIN_uV             0
#define PAPYRUS_VCOM_MAX_uV             275000 
#define PAPYRUS_VCOM_STEP_uV            11000
#define PAPYRUS_VCOM_MIN_VAL            0
#define PAPYRUS_VCOM_MAX_VAL            255
#define PAPYRUS_VCOM_FUDGE_FACTOR       0
#define PAPYRUS_VCOM_VOLTAGE_DEFAULT    186000	/* PAPYRUS_TO_VCOM(186000/1000) -> -2V */

#define PAPYRUS_VNEG_MIN_uV    5000000
#define PAPYRUS_VNEG_MAX_uV   20000000
#define PAPYRUS_VNEG_STEP_uV   1000000
#define PAPYRUS_VNEG_MIN_VAL         0
#define PAPYRUS_VNEG_MAX_VAL         1

#define PAPYRUS_VPOS_MIN_uV    5000000
#define PAPYRUS_VPOS_MAX_uV   20000000
#define PAPYRUS_VPOS_STEP_uV   1000000
#define PAPYRUS_VPOS_MIN_VAL         0
#define PAPYRUS_VPOS_MAX_VAL         1

#define PAPYRUS_TMST_MIN_uV	5000000
#define PAPYRUS_TMST_MAX_uV	20000000
#define PAPYRUS_TSMT_STEP_uV	1000000
#define PAPYRUS_TMST_MIN_VAL	0
#define PAPYRUS_TMST_MAX_VAL	1
#define PAPYRUS_TEMP_LOGGING	29

int papyrus_vcom_cur_voltage = PAPYRUS_VCOM_VOLTAGE_DEFAULT;

static int papyrus_gpio_wakeup = 1;

extern int mxc_i2c_suspended;

static void papyrus_gettemp_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(papyrus_temp_work, papyrus_gettemp_work);

struct papyrus {
	/* chip revision */
	int rev;

	struct device *dev;

	/* Platform connection */
	struct i2c_client *i2c_client;

	/* Client devices */
	struct platform_device *pdev[PAPYRUS_REG_NUM];

	/* GPIOs */
	int gpio_pmic_pwrgood;
	int gpio_pmic_vcom_ctrl;
	int gpio_pmic_wakeup;
	int gpio_pmic_intr;

	bool vcom_setup;
	bool is_suspended;
	bool is_shutdown;
	
	int max_wait;
	int num_disp_enabled;
};

/*this is saved from the probe and used in gpio_wakeup sys entry only*/
static int gpio_pmic_wakeup_saved; 

DEFINE_MUTEX(papyrus_display_mutex);
DEFINE_MUTEX(papyrus_vcom_mutex);

struct papyrus *g_papyrus = NULL;

/*	276 input values    -> 256 output values

	A couple of real-world examples:
	VCOM is -1.69 (169) -> 156
	VCOM is -1.56 (156) -> 144
 */
static char vcom_to_papyrus_conversion[276] = 
{
/*    0    1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18      <-- -(VCOM / 100) (actual VCOM output from Papyrus)		  */
      0,   1,   1,   2,   3,   4,   5,   6,   7,   8,   9,  10,  11,  12,  13,  14,  14,  15,  16,  /*  --> PAPYRUS_VCOM (register VCOM value to set for Papyrus) */

/*   19   20   21   22   23   24   25   26   27   28   29   30   31   32   33   34   35   36   37   */
     17,  18,  19,  20,  21,  22,  23,  24,  25,  25,  26,  27,  28,  29,  30,  31,  32,  33,  34,

/*   38   39   40   41   42   43   44   45   46   47   48   49   50   51   52   53   54   55   56   */
     35,  36,  37,  37,  38,  39,  40,  41,  42,  43,  44,  45,  46,  47,  48,  49,  49,  50,  51,

/*   57   58   59   60   61   62   63   64   65   66   67   68   69   70   71   72   73   74   75   */
     52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63,  63,  64,  65,  66,  67,  68,  69,

/*   76   77   78   79   80   81   82   83   84   85   86   87   88   89   90   91   92   93   94   */
     70,  71,  72,  73,  74,  74,  75,  76,  77,  78,  79,  80,  81,  82,  83,  84,  85,  86,  87,

/*   95   96   97   98   99  100  101  102  103  104  105  106  107  108  109  110  111  112  113   */
     87,  88,  89,  90,  91,  92,  93,  94,  95,  96,  97,  98,  99, 100, 100, 101, 102, 103, 104,

/*  114  115  116  117  118  119  120  121  122  123  124  125  126  127  128  129  130  131  132   */
    105, 106, 107, 108, 109, 110, 111, 112, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122,

/*  133  134  135  136  137  138  139  140  141  142  143  144  145  146  147  148  149  150  151   */
    123, 124, 125, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 138, 139,

/*  152  153  154  155  156  157  158  159  160  161  162  163  164  165  166  167  168  169  170   */
    140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 151, 152, 153, 154, 155, 156, 157,

/*  171  172  173  174  175  176  177  178  179  180  181  182  183  184  185  186  187  188  189   */
    158, 159, 160, 161, 162, 163, 164, 165, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175,

/*  190  191  192  193  194  195  196  197  198  199  200  201  202  203  204  205  206  207  208   */
    176, 177, 178, 179, 180, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193,

/*  209  210  211  212  213  214  215  216  217  218  219  220  221  222  223  224  225  226  227   */
    194, 195, 196, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211,

/*  228  229  230  231  232  233  234  235  236  237  238  239  240  241  242  243  244  245  246   */
    211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 223, 224, 225, 226, 227, 228,

/*  247  248  249  250  251  252  253  254  255  256  257  258  259  260  261  262  263  264  265   */
    229, 230, 231, 232, 233, 234, 235, 236, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246,

/*  266  267  268  269  270  271  272  273  274  275                                                */
    247, 248, 249, 249, 250, 251, 252, 253, 254, 255
};
#define VCOM_TO_PAPYRUS(v) vcom_to_papyrus_conversion[(int)(v)]

/*	The Papyrus-to-VCOM conversion is more ambiguious because several VCOM-to-Papyrus entries are
	duplicates.  For example, if you program in say, -1.63V (163) or -1.64V (164), the
	VCOM-to-Papyrus value is 151 in both cases.
 */
static int papyrus_to_vcom_conversion[256] =
{
/*    0    1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18      <-- VCOM_PAPYRUS (register VCOM value from Papyrus)  */
      0,   2,   3,   4,   5,   6,   7,   8,   9,  10,  11,  12,  13,  14,  16,  17,  18,  19,  20,  /* --> -(VCOM/ 100) (actual VCOM output from Papyrus)   */                                                  

/*   19   20   21   22   23   24   25   26   27   28   29   30   31   32   33   34   35   36   37   */
     21,  22,  23,  24,  25,  26,  27,  29,  30,  31,  32,  33,  34,  35,  36,  37,  38,  39,  41,

/*   38   39   40   41   42   43   44   45   46   47   48   49   50   51   52   53   54   55   56   */
     42,  43,  44,  45,  46,  47,  48,  49,  50,  51,  52,  54,  55,  56,  57,  58,  59,  60,  61,

/*   57   58   59   60   61   62   63   64   65   66   67   68   69   70   71   72   73   74   75   */
     62,  63,  64,  65,  66,  67,  69,  70,  71,  72,  73,  74,  75,  76,  77,  78,  79,  81,  82,

/*   76   77   78   79   80   81   82   83   84   85   86   87   88   89   90   91   92   93   94   */
     83,  84,  85,  86,  87,  88,  89,  90,  91,  92,  93,  94,  96,  97,  98,  99, 100, 101, 102,

/*   95   96   97   98   99  100  101  102  103  104  105  106  107  108  109  110  111  112  113   */
    103, 104, 105, 106, 107, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 122, 123,

/*  114  115  116  117  118  119  120  121  122  123  124  125  126  127  128  129  130  131  132   */
    124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 136, 137, 138, 139, 140, 141, 142, 143,

/*  133  134  135  136  137  138  139  140  141  142  143  144  145  146  147  148  149  150  151   */
    144, 145, 146, 147, 148, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163,

/*  152  153  154  155  156  157  158  159  160  161  162  163  164  165  166  167  168  169  170   */
    165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 180, 181, 182, 183, 184,

/*  171  172  173  174  175  176  177  178  179  180  181  182  183  184  185  186  187  188  189   */
    185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 196, 197, 198, 199, 200, 201, 202, 203, 204,

/* 190  191  192  193  194  195  196  197  198  199  200  201  202  203  204  205  206  207  208    */
   205, 206, 207, 208, 209, 210, 211, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224,

/* 209  210  211  212  213  214  215  216  217  218  219  220  221  222  223  224  225  226  227    */
   225, 226, 227, 228, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 242, 243, 244, 245,

/* 228  229  230  231  232  233  234  235  236  237  238  239  240  241  242  243  244  245  246    */
   246, 247, 248, 249, 250, 251, 252, 253, 255, 256, 257, 258, 259, 260, 261, 262, 263, 264, 265,

/* 247  248  249  250  251  252  253  254  255                                                      */
   266, 267, 268, 270, 271, 272, 237, 274, 275
};
#define PAPYRUS_TO_VCOM(p) papyrus_to_vcom_conversion[(p)]

static int papyrus_vcom_set_voltage(struct regulator_dev *reg,
					int minuV, int uV)
{
	struct papyrus *papyrus = rdev_get_drvdata(reg);
	struct i2c_client *client = papyrus->i2c_client;
	unsigned int reg_val;
	int vcom_read;

	if ((uV < PAPYRUS_VCOM_MIN_uV) || (uV > PAPYRUS_VCOM_MAX_uV))
		return -EINVAL;

	reg_val = i2c_smbus_read_byte_data(client, REG_PAPYRUS_DVR);

	/*
	 * Only program VCOM if it is not set to the desired value.
	 * Programming VCOM excessively degrades ability to keep
	 * DVR register value persistent.
	 */
	vcom_read = PAPYRUS_TO_VCOM(reg_val) * 1000;
	if (vcom_read != uV) {
		reg_val = VCOM_TO_PAPYRUS(uV/1000);
		if (i2c_smbus_write_byte_data(client, REG_PAPYRUS_DVR, reg_val) < 0)
			return -1;
		papyrus_vcom_cur_voltage = uV;
	}

	return 0;
}

static int papyrus_vcom_get_voltage(struct regulator_dev *reg)
{
	struct papyrus *papyrus = rdev_get_drvdata(reg);
	struct i2c_client *client = papyrus->i2c_client;
	unsigned int reg_val;

	reg_val = i2c_smbus_read_byte_data(client, REG_PAPYRUS_DVR);
	return (PAPYRUS_TO_VCOM(reg_val) * 1000);
}

static int papyrus_vcom_enable(struct regulator_dev *reg)
{
	struct papyrus *papyrus = rdev_get_drvdata(reg);
	int ret = 0;

	mutex_lock(&papyrus_display_mutex);
	mutex_lock(&papyrus_vcom_mutex);
	if (!papyrus_gpio_wakeup || papyrus->is_shutdown)
		goto out;
	
	if (papyrus->is_suspended || !papyrus->num_disp_enabled)
	{
		printk(KERN_ERR "Attempting to enable VCOM while Papyrus disabled\n");
		ret = -1;
		goto out;
	}
		
	/*
	 * Check to see if we need to set the VCOM voltage.
	 * We can only change vcom voltage if we have been enabled.
	 * TEQ-1762 - Jack says we need to set VCOM on every power up
	 */
	if (papyrus_vcom_set_voltage(reg,
				papyrus_vcom_cur_voltage,
				papyrus_vcom_cur_voltage) < 0)
	{
		ret = -1;
		goto out;
	}

	/* Turn on the GPIO */
	gpio_set_value(papyrus->gpio_pmic_vcom_ctrl, 1);

out:	
	mutex_unlock(&papyrus_vcom_mutex);
	mutex_unlock(&papyrus_display_mutex);
	return ret;
}

static int papyrus_vcom_disable(struct regulator_dev *reg)
{
	struct papyrus *papyrus   = rdev_get_drvdata(reg);

	mutex_lock(&papyrus_display_mutex);
	mutex_lock(&papyrus_vcom_mutex);
	if (!papyrus_gpio_wakeup)
		goto out;

	/* Turn off the GPIO */
	gpio_set_value(papyrus->gpio_pmic_vcom_ctrl, 0);
 
out:
	mutex_unlock(&papyrus_vcom_mutex);
	mutex_unlock(&papyrus_display_mutex);
	return 0;
}

static int papyrus_tmst_enable(struct regulator_dev *reg)
{
	/* Papyrus already in standby */
	return 0;
}

static int papyrus_tmst_disable(struct regulator_dev *reg)
{
	/* Papyrus already in standby */
	return 0;
}

static int papyrus_tmst_get_temp(struct regulator_dev *reg)
{
	struct papyrus *papyrus = rdev_get_drvdata(reg);
	struct i2c_client *client = papyrus->i2c_client;
	unsigned int reg_val;
	int i = 0;

	i2c_smbus_write_byte_data(client, REG_PAPYRUS_TMST_CONFIG, 0x80);
	reg_val = i2c_smbus_read_byte_data(client, REG_PAPYRUS_TMST_CONFIG);

	for (i=0; i <= papyrus->max_wait; i++) {
		if (reg_val & 0x20)
			break;
		msleep(20);
		reg_val = i2c_smbus_read_byte_data(client, REG_PAPYRUS_TMST_CONFIG);
	}	

	if (i == papyrus->max_wait)
		return -ETIMEDOUT;

	reg_val = i2c_smbus_read_byte_data(client, REG_PAPYRUS_EXT_TEMP);
	return reg_val;
}	

static int papyrus_wait_power_good(struct papyrus *papyrus)
{
	int i;
	int reg_val = 0;
	struct i2c_client *client = papyrus->i2c_client;

	for (i = 0; i < papyrus->max_wait; i++) {
		if (gpio_get_value(papyrus->gpio_pmic_pwrgood))
			return 0;
		msleep(1);
	}

	printk(KERN_ERR "Papyrus timed out:0x%x\n", reg_val);
	return -ETIMEDOUT;
}

static int papyrus_wait_power_not_good(struct papyrus *papyrus)
{
	int i;
	for (i = 0; i < papyrus->max_wait; i++)
	{
		if (!gpio_get_value(papyrus->gpio_pmic_pwrgood))
			return 0;
		msleep(1); 
	}
	printk(KERN_ERR "Power not good timed out\n");
	return -ETIMEDOUT;
}


static int papyrus_display_is_enabled(struct regulator_dev *reg)
{
	struct papyrus *papyrus = rdev_get_drvdata(reg);
	return gpio_get_value(papyrus->gpio_pmic_pwrgood);
}

static void papyrus_sleep(void)
{
	struct papyrus *papyrus = g_papyrus;

	if (!papyrus || papyrus->is_suspended)
		return;

	/* Cancel temp work */
	//cancel_rearming_delayed_work(&papyrus_temp_work);
	
	/* Pull down the GPIO pin for sleep mode */
	/* VCOM will be cleared and will need to be reset */
	papyrus->vcom_setup = false;
	papyrus->is_suspended = true;
	papyrus->num_disp_enabled = 0;
	gpio_set_value(papyrus->gpio_pmic_vcom_ctrl, 0);
	gpio_set_value(papyrus->gpio_pmic_wakeup, 0);
	/* Technically we should be waiting 3 seconds here */
}

static int papyrus_enable(struct papyrus *papyrus)
{
	struct i2c_client *client = papyrus->i2c_client;
	int retry = PAPYRUS_MAX_ENABLE_RETRY;
	int ret = 0;

	if (!papyrus->num_disp_enabled)
	{
		while(retry)
		{
			if ((ret = i2c_smbus_write_byte_data(client, REG_PAPYRUS_ENABLE, PAPYRUS_ENABLE_REGS)) < 0)
			{
				retry--;
				msleep(5);
				continue;
			}
			break;
		}
	}
	if (!retry)
	{
		printk(KERN_ERR "Papyrus: I2C Send ENABLE failed! : %d\n", ret);
		return -1;
	}

	papyrus->num_disp_enabled++;
	return ret;
}

static int papyrus_wake(void)
{
	struct papyrus *papyrus = g_papyrus;
	int ret = 0;
	int retry = PAPYRUS_MAX_ENABLE_RETRY;

	if (!papyrus)
		return -1;

	while(retry)
	{
		/* Turn on Wakeup GPIO . Will be on till next suspend */
		gpio_set_value(papyrus->gpio_pmic_wakeup, 1);
		
		/* Wait for I2C to be ready */
		msleep(PAPYRUS_SUSPEND_DISPLAY_SETTLE_TIME);

		/* Papyrus enable success ? */
		if((ret = papyrus_enable(papyrus)) < 0)
			goto retry;

		/* Wait for PWRGOOD */
		ret = papyrus_wait_power_good(papyrus);
		if (!ret)
		{
			/* Success. We're done */
			papyrus->is_suspended = false;
			break;
		}
		else
			papyrus->num_disp_enabled--;

retry:
		printk(KERN_ERR "Retrying Papyrus enable\n");
		/* Failed. Reset and retry */
		papyrus->is_suspended = true;
		gpio_set_value(papyrus->gpio_pmic_wakeup, 0);
		
		retry--;
	}

	if (!retry)
		return ret;

	return 0;
}

static int papyrus_standby()
{
  struct papyrus *papyrus = g_papyrus;
  struct i2c_client *client = papyrus->i2c_client;
  int ret = 0;

  if (papyrus->is_suspended || !papyrus->num_disp_enabled)
    goto out;

  papyrus->num_disp_enabled--;
  if (papyrus->num_disp_enabled > 0)
    goto out;

	mutex_lock(&papyrus_vcom_mutex);
	ret = i2c_smbus_write_byte_data(client, REG_PAPYRUS_ENABLE, PAPYRUS_STANDBY_REGS);
	//papyrus_wait_power_not_good(papyrus);
	mutex_unlock(&papyrus_vcom_mutex);

out:
	return ret;
}

static int papyrus_display_enable(struct regulator_dev *reg)
{
	struct papyrus *papyrus = rdev_get_drvdata(reg);
	int ret = 0;

	mutex_lock(&papyrus_display_mutex);
	
	if (!papyrus_gpio_wakeup)
		goto out;

	ret = papyrus_wake();

out:
	mutex_unlock(&papyrus_display_mutex);
	return ret;
}

static int papyrus_display_disable(struct regulator_dev *reg)
{
	struct papyrus *papyrus = rdev_get_drvdata(reg);
	int ret = 0;

	mutex_lock(&papyrus_display_mutex);
	if (!papyrus_gpio_wakeup)
		goto out;
	// Set papyrus to standby
	ret = papyrus_standby();

out:
	mutex_unlock(&papyrus_display_mutex);
	return ret;
}

static int papyrus_suspend(struct i2c_client *client, pm_message_t state)
{
	struct papyrus *papyrus = i2c_get_clientdata(client);
	
	mutex_lock(&papyrus_display_mutex);
	mutex_lock(&papyrus_vcom_mutex);
	papyrus->is_shutdown = true;
	cancel_rearming_delayed_work(&papyrus_temp_work);
	i2c_smbus_write_byte_data(client, REG_PAPYRUS_ENABLE, PAPYRUS_STANDBY_REGS);
	papyrus_sleep();
	mutex_unlock(&papyrus_vcom_mutex);
	mutex_unlock(&papyrus_display_mutex);
	return 0;
}
	
static int papyrus_resume(struct i2c_client *client)
{
	struct papyrus *papyrus = i2c_get_clientdata(client);
	
	mutex_lock(&papyrus_display_mutex);
	papyrus->is_shutdown = false;
	mutex_unlock(&papyrus_display_mutex);
	/* Restart the temp collection worker now */
	schedule_delayed_work(&papyrus_temp_work, msecs_to_jiffies(0));	
	return 0;
}

/*
 * Regulator operations
 */
static struct regulator_ops papyrus_display_ops = {
	.enable = papyrus_display_enable,
	.disable = papyrus_display_disable,
	.is_enabled = papyrus_display_is_enabled,
};

static struct regulator_ops papyrus_gvdd_ops = {
};

static struct regulator_ops papyrus_gvee_ops = {
};

/*
static struct regulator_ops papyrus_hvinn_ops = {
};

static struct regulator_ops papyrus_hvinp_ops = {
	.enable = papyrus_hvinp_enable,
	.disable = papyrus_hvinp_disable,
	.get_voltage = papyrus_hvinp_get_voltage,
	.set_voltage = papyrus_hvinp_set_voltage,
};
*/

static struct regulator_ops papyrus_vcom_ops = {
	.enable = papyrus_vcom_enable,
	.disable = papyrus_vcom_disable,
	.get_voltage = papyrus_vcom_get_voltage,
	.set_voltage = papyrus_vcom_set_voltage,
};

static struct regulator_ops papyrus_vneg_ops = {
};

static struct regulator_ops papyrus_vpos_ops = {
};

static struct regulator_ops papyrus_tmst_ops = {
	.get_voltage = papyrus_tmst_get_temp,
	.enable = papyrus_tmst_enable,
	.disable = papyrus_tmst_disable,
};

/*
 * Regulator descriptors
 */
static struct regulator_desc papyrus_reg[PAPYRUS_NUM_REGULATORS] = {
{
	.name = "DISPLAY",
	.id = PAPYRUS_DISPLAY,
	.ops = &papyrus_display_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "GVDD",
	.id = PAPYRUS_GVDD,
	.ops = &papyrus_gvdd_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "GVEE",
	.id = PAPYRUS_GVEE,
	.ops = &papyrus_gvee_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "VCOM",
	.id = PAPYRUS_VCOM,
	.ops = &papyrus_vcom_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "VNEG",
	.id = PAPYRUS_VNEG,
	.ops = &papyrus_vneg_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "VPOS",
	.id = PAPYRUS_VPOS,
	.ops = &papyrus_vpos_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "TMST",
	.id = PAPYRUS_TMST,
	.ops = &papyrus_tmst_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
};

/*
 * Regulator init/probing/exit functions
 */
static int papyrus_regulator_probe(struct platform_device *pdev)
{
	struct regulator_dev *rdev;

	rdev = regulator_register(&papyrus_reg[pdev->id], &pdev->dev,
				  pdev->dev.platform_data,
				  dev_get_drvdata(&pdev->dev));

	if (IS_ERR(rdev)) {
		dev_err(&pdev->dev, "failed to register %s\n",
			papyrus_reg[pdev->id].name);
		return PTR_ERR(rdev);
	}

	return 0;
}

static int papyrus_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);
	regulator_unregister(rdev);
	return 0;
}

static struct platform_driver papyrus_regulator_driver = {
	.probe = papyrus_regulator_probe,
	.remove = papyrus_regulator_remove,
	.driver = {
		.name = "papyrus-reg",
	},
};

static int papyrus_register_regulator(struct papyrus *papyrus, int reg,
				     struct regulator_init_data *initdata)
{
	struct platform_device *pdev;
	int ret;

	struct i2c_client *client = papyrus->i2c_client;

	ret = i2c_smbus_read_byte_data(client, 0x10);

	/* If we can't find PMIC via I2C, we should not register regulators */
	if (ret < 0) {
		dev_err(papyrus->dev,
			"Papyrus PMIC not found!\n");
		return -ENXIO;
	}

	if (papyrus->pdev[reg])
		return -EBUSY;

	pdev = platform_device_alloc("papyrus-reg", reg);
	if (!pdev)
		return -ENOMEM;

	papyrus->pdev[reg] = pdev;

	initdata->driver_data = papyrus;

	pdev->dev.platform_data = initdata;
	pdev->dev.parent = papyrus->dev;
	platform_set_drvdata(pdev, papyrus);

	ret = platform_device_add(pdev);

	if (ret != 0) {
		dev_err(papyrus->dev,
		       "Failed to register regulator %d: %d\n",
			reg, ret);
		platform_device_del(pdev);
		papyrus->pdev[reg] = NULL;
	}

	return ret;
}

struct regulator *temp_regulator, *display_regulator, *vcom_regulator;
signed int papyrus_temp = 25;
EXPORT_SYMBOL(papyrus_temp);

static ssize_t show_papyrus_gpio_wakeup(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", papyrus_gpio_wakeup);
}

static ssize_t
store_papyrus_gpio_wakeup(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	volatile int wakeup = 0;
	
	if (sscanf(buf, "%d", &wakeup) > 0) {
		if(0 == wakeup)
		{
			mutex_lock(&papyrus_display_mutex);
			mutex_lock(&papyrus_vcom_mutex);
			papyrus_gpio_wakeup = wakeup;
			papyrus_sleep();
			mutex_unlock(&papyrus_vcom_mutex);
			mutex_unlock(&papyrus_display_mutex);
		}else if(1 == wakeup)
		{
			mutex_lock(&papyrus_display_mutex);
			mutex_lock(&papyrus_vcom_mutex);
			papyrus_gpio_wakeup = wakeup;
			papyrus_wake();
			mutex_unlock(&papyrus_vcom_mutex);
			mutex_unlock(&papyrus_display_mutex);
		}else
		{
			printk(KERN_ERR "invalid gpio_wakeup echo\n");
		}
		return strlen(buf);
	}else
	{
		printk(KERN_ERR "invalid gpio_wakeup echo\n");
	}

	return -EINVAL;
}
static DEVICE_ATTR(papyrus_gpio_wakeup, 0666, show_papyrus_gpio_wakeup, store_papyrus_gpio_wakeup);

static ssize_t show_papyrus_vcom_voltage(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = -EINVAL;
	volatile int volt = 0;

	ret = regulator_enable(display_regulator);
	if (ret < 0)
	{
		return sprintf(buf, "%d\n", ret);
	}
  
	ret = regulator_enable(vcom_regulator);
	if (ret < 0)
	{
		regulator_disable(display_regulator);
		return sprintf(buf, "%d\n", ret);
	}

	volt = regulator_get_voltage(vcom_regulator)/1000;
	volt = (volt * 2750) / 255;

	regulator_disable(vcom_regulator);
	regulator_disable(display_regulator);
	return sprintf(buf, "%d\n", volt);
}

static ssize_t
store_papyrus_vcom_voltage(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	volatile int volt = 0;
	
	if (sscanf(buf, "%d", &volt) > 0) {
		regulator_enable(display_regulator);
		regulator_enable(vcom_regulator);
		
		volt++;
		volt = (volt * 255) / 2750;
		volt = volt * 1000;

		if (regulator_set_voltage(vcom_regulator, volt, volt) < 0) {
			printk(KERN_ERR "Error setting Papyrus VCOM to %d\n", volt);
			return -EINVAL;
		}

		regulator_disable(vcom_regulator);		
		regulator_disable(display_regulator);

		return strlen(buf);
	}

	return -EINVAL;
}
static DEVICE_ATTR(papyrus_vcom_voltage, 0666, show_papyrus_vcom_voltage, store_papyrus_vcom_voltage);

static ssize_t
show_papyrus_temperature(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", papyrus_temp);
}
static DEVICE_ATTR(papyrus_temperature, 0444, show_papyrus_temperature, NULL);

static void papyrus_gettemp_work(struct work_struct *work)
{
	signed int temp = 0;

	/* i2c bus is suspended */
	if (mxc_i2c_suspended)
		return;

	if (!papyrus_gpio_wakeup)
		return;
	
	mutex_lock(&papyrus_display_mutex);
	if (g_papyrus->is_shutdown == true)
	{
		printk(KERN_INFO "Temperature: Papyrus is shutting down. Return\n");
		mutex_unlock(&papyrus_display_mutex);
		return;
	}

	papyrus_wake();
	temp = regulator_get_voltage(temp_regulator);
	papyrus_standby();
	

	if (temp >= 246)
		temp = (temp - 256);

	papyrus_temp = temp;

	if (papyrus_temp >= PAPYRUS_TEMP_LOGGING)
		printk(KERN_DEBUG "kernel: I papyrus:TempExceeded:temp=%d:\n", papyrus_temp);
	schedule_delayed_work(&papyrus_temp_work, msecs_to_jiffies(PAPYRUS_TEMP_THRESHOLD));
	mutex_unlock(&papyrus_display_mutex);
}

static int papyrus_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	int i;
	struct papyrus *papyrus;
	struct papyrus_platform_data *pdata = client->dev.platform_data;
	int ret = 0;

	if (!pdata || !pdata->regulator_init)
		return -ENODEV;

	/* Create the PMIC data structure */
	papyrus = kzalloc(sizeof(struct papyrus), GFP_KERNEL);
	if (papyrus == NULL) {
		kfree(client);
		return -ENOMEM;
	}
  
	/* Initialize the PMIC data structure */
	i2c_set_clientdata(client, papyrus);
	papyrus->dev = &client->dev;
	papyrus->i2c_client = client;

	papyrus->gpio_pmic_pwrgood = pdata->gpio_pmic_pwrgood;
	papyrus->gpio_pmic_vcom_ctrl = pdata->gpio_pmic_vcom_ctrl;
	papyrus->gpio_pmic_wakeup = pdata->gpio_pmic_wakeup;
	papyrus->gpio_pmic_intr = pdata->gpio_pmic_intr;

	papyrus->is_shutdown  = false;
	papyrus->is_suspended = true;
	papyrus->vcom_setup		= false; 

	gpio_pmic_wakeup_saved = papyrus->gpio_pmic_wakeup;

	ret = platform_driver_register(&papyrus_regulator_driver);
	if (ret < 0)
		goto err;

	for (i = 0; i <= PAPYRUS_TMST; i++) {
		ret = papyrus_register_regulator(papyrus, i, &pdata->regulator_init[i]);
		if (ret != 0) {
			dev_err(papyrus->dev, "Platform init() failed: %d\n",
			ret);
			goto err;
		}
	}

	papyrus->max_wait = 100;
 
	/* Initialize the PMIC device */
	dev_info(&client->dev, "PMIC PAPYRUS for eInk display\n");

	/* Get the temperature reg */
	temp_regulator = regulator_get(NULL, "TMST");
	if (IS_ERR(temp_regulator)) {
		printk(KERN_ERR "Could not get the TMST regulator\n");
	}

	display_regulator = regulator_get(NULL, "DISPLAY");
	if (IS_ERR(display_regulator)) {
		printk(KERN_ERR "Could not get the DISPLAY regulator\n");
	}

	vcom_regulator = regulator_get(NULL, "VCOM");
	if (IS_ERR(vcom_regulator)) {
		printk(KERN_ERR "Could not get the VCOM regulator\n");
	}

	if (device_create_file(&client->dev, &dev_attr_papyrus_temperature) < 0)
		printk(KERN_ERR "(%s): Could not create papyrus_temperature file\n", __FUNCTION__);

	if (device_create_file(&client->dev, &dev_attr_papyrus_vcom_voltage) < 0)
		printk(KERN_ERR "(%s): Could not create papyrus_vcom_voltage file\n", __FUNCTION__);

	if (device_create_file(&client->dev, &dev_attr_papyrus_gpio_wakeup) < 0)
		printk(KERN_ERR "(%s): Could not create papyrus_gpio_wakeup file\n", __FUNCTION__);

	g_papyrus = papyrus;
	mutex_init(&papyrus_vcom_mutex);
	mutex_init(&papyrus_display_mutex);
	
	schedule_delayed_work(&papyrus_temp_work, msecs_to_jiffies(0));	

	return ret;
err:
	kfree(papyrus);

	return ret;
}


static int papyrus_i2c_remove(struct i2c_client *i2c)
{
	struct papyrus *papyrus = i2c_get_clientdata(i2c);
	int i;

	mutex_lock(&papyrus_display_mutex);
	mutex_lock(&papyrus_vcom_mutex);
	papyrus->is_shutdown = true;
	cancel_rearming_delayed_work(&papyrus_temp_work);
	papyrus_sleep();
	mutex_unlock(&papyrus_vcom_mutex);
	mutex_unlock(&papyrus_display_mutex);

	for (i = 0; i < ARRAY_SIZE(papyrus->pdev); i++)
		platform_device_unregister(papyrus->pdev[i]);

	device_remove_file(&i2c->dev, &dev_attr_papyrus_temperature);
	device_remove_file(&i2c->dev, &dev_attr_papyrus_vcom_voltage);
	device_remove_file(&i2c->dev, &dev_attr_papyrus_gpio_wakeup);

	platform_driver_unregister(&papyrus_regulator_driver);
	regulator_put(temp_regulator);
	regulator_put(display_regulator);
	regulator_put(vcom_regulator);

	kfree(papyrus);

	return 0;
}

static const struct i2c_device_id papyrus_i2c_id[] = {
       { "papyrus", 0 },
       { }
};
MODULE_DEVICE_TABLE(i2c, papyrus_i2c_id);


static struct i2c_driver papyrus_i2c_driver = {
	.driver = {
		   .name = "papyrus",
		   .owner = THIS_MODULE,
	},
	.probe = papyrus_i2c_probe,
	.remove = papyrus_i2c_remove,
	.suspend = papyrus_suspend,
	.resume = papyrus_resume,
	.id_table = papyrus_i2c_id,
};

static int __init papyrus_init(void)
{
	return i2c_add_driver(&papyrus_i2c_driver);
}
module_init(papyrus_init);

static void __exit papyrus_exit(void)
{
	i2c_del_driver(&papyrus_i2c_driver);
}
module_exit(papyrus_exit);

/* Module information */
MODULE_DESCRIPTION("PAPYRUS regulator driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Manish Lachwani <lachwani@amazon.com>");
