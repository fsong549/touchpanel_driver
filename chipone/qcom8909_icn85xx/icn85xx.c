/*++

*Copyright (c) 2012-2022 ChipOne Technology (Beijing) Co., Ltd. All Rights Reserved.
*This PROPRIETARY SOFTWARE is the property of ChipOne Technology (Beijing) Co., Ltd.
*and may contains trade secrets and/or other confidential information of ChipOne
*Technology (Beijing) Co., Ltd. This file shall not be disclosed to any third party,
*in whole or in part, without prior written consent of ChipOne.
*THIS PROPRIETARY SOFTWARE & ANY RELATED DOCUMENTATION ARE PROVIDED AS IS,
*WITH ALL FAULTS, & WITHOUT WARRANTY OF ANY KIND. CHIPONE DISCLAIMS ALL EXPRESS OR
*IMPLIED WARRANTIES.

*File Name:    icn85xx.c
*Abstract:
		input driver.
*Author:	miaodefang
*Date :		2016.01.05
*Version:	1.0
*History :
		2016.01.05, V1.0 first version
--*/

#include "icn85xx.h"

#ifdef CONFIG_DEV_INFO
static int save_tp_info(void);
#endif
struct i2c_client *this_client;
void icn85xx_irq_disable(void);
void icn85xx_irq_enable(void);
#if (SUPPORT_SYSFS || SUPPORT_PROC_FS)
static short log_basedata[COL_NUM][ROW_NUM] = {{0, 0} };
static short log_rawdata[COL_NUM][ROW_NUM] = {{0, 0} };
static short log_diffdata[COL_NUM][ROW_NUM] = {{0, 0} };
static void icn85xx_log(char diff);
#endif/*(SUPPORT_SYSFS || SUPPORT_PROC_FS)*/

#if SUPPORT_SYSFS
static const struct attribute_group *icn_drv_grp[];
static short log_diff_combine_1010[COL_NUM][ROW_NUM] = {{0, 0} };
static short log_diff_combine_1100[COL_NUM][ROW_NUM] = {{0, 0} };
#endif/*SUPPORT_SYSFS*/
char firmware[] = {"icn85xx_firmware"};

//static char boot_mode = ICN85XX_WITH_FLASH_85;

#ifdef CONFIG_TOUCHSCREEN_ICN85XX_GESTURE
#define KEY_ICN_SENSOR 251
struct input_dev *icn_key_dev;
extern u8 gTGesture;

#define GESTURE_C                       0x12
#define GESTURE_O                       0x13
#define GESTURE_M                       0x14
#define GESTURE_D_TAP                   0x50

GESTURE_DATA structGestureData;
static void icn85xx_gesture_handler(u8 gesture_id);
#endif/*CONFIG_TOUCHSCREEN_ICN85XX_GESTURE*/
//static void icn85xx_ts_register_productinfo(struct icn85xx_ts_data *icn85xx_ts);

#if defined(CONFIG_TOUCHSCREEN_ICN85XX_FH)
extern int usb_flag;
static int charger_detect_enable =1;
static int device_is_available;

module_param_named(charger_detect_enable, charger_detect_enable, int, S_IRUGO | S_IWUSR);
static void icn85xx_charger_plugin(struct icn85xx_ts_data *icn85xx_ts, int plugin);
#endif /* CONFIG_TOUCHSCREEN_ICN85XX_FH */

static int debug_mask = 1;
static int esd_mask = 1;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR);
module_param_named(esd_mask, esd_mask, int, S_IRUGO | S_IWUSR);
#if TOUCH_VIRTUAL_KEYS
static int virtual_keys_debug = 1;
module_param_named(virtual_keys_debug, virtual_keys_debug, int, S_IRUGO | S_IWUSR);
#endif

#define icn85xx_trace(fmt, args...)	do {	if (debug_mask)	\
									printk("icn85xx_ts: "fmt, ##args);	\
								} while (0)

#define icn85xx_point_info(fmt, args...)	do {	if (debug_mask)	\
										printk("icn85xx_ts: "fmt, ##args);	\
									} while (0)

#define icn85xx_info(fmt, args...)	do {	if (debug_mask)	\
									printk("icn85xx_ts: "fmt, ##args);	\
							} while (0)

#if SUPPORT_SYSFS
/*  update cmd:
 *u:/mnt/aaa/bbb/ccc   or U:/mnt/aaa/bbb/ccc, update firmware
 *t or T, reset tp
 *r or R, printf rawdata
 *d or D, printf diffdata
 *c:100 or C:100, Consistence test
 *e:Vol:Max:Min or E:Vol:Max:Min, diffdata test
 *        you can get the diffdata test result, and you should send e/E cmd before this cmd
 *        result formate:   result:Max:Min=%02x:%08x:%08x
 *        when cmd e/E before this cmd and Max=Min=0, you can get Max/Min diffdata
 */

/*
   example 1: update firmware
   echo u:/mnt/sdcard/download/icn86xx.bin > icn_update
   cat icn_update

   example 2: Uniformity test
   echo c:100 > icn_update
   cat icn_update

   example 3: open short (dif) test
   echo e:3:500:200 > icn_update
   cat icn_update

 */

static ssize_t icn85xx_show_version(struct device_driver *drv, char *buf)
{
	ssize_t ret = 0;

	snprintf(buf, PAGE_SIZE, "0x%x\n", icn85xx_readVersion());
	ret = strlen(buf) + 1;

	return ret;
}

#define PARA_STATE_CMD             0X00
#define PARA_STATE_DATA1           0X01
#define PARA_STATE_DATA2           0X02
#define PARA_STATE_DATA3           0X03

#define UPDATE_CMD_NONE            0x00

static char update_cmd = UPDATE_CMD_NONE;
static int  update_data1;
static int  update_data2;/* MAX ODD&EVEN */
static int  update_data3;/* MIN */
static int  update_data4;/* MAX 1ST&3RD */
static int  update_data5;/* MIN */


static ssize_t icn85xx_show_update(struct device_driver *drv,
		char *buf)
{
	ssize_t ret = 0;
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(this_client);

	switch (update_cmd) {
	case 'u':
	case 'U':
		snprintf(buf, PAGE_SIZE, icn85xx_ts->fw_name);
		ret = strlen(buf) + 1;
		icn85xx_trace("firmware: %s\n", icn85xx_ts->fw_name);

		break;
	case 't':
	case 'T':
		icn85xx_trace("cmd t,T\n");

		break;
	case 'r':
	case 'R':
		icn85xx_trace("cmd r,R\n");

		break;
	case 'd':
	case 'D':
		icn85xx_trace("cmd d,D\n");

		break;
	case 'c':
	case 'C':
		icn85xx_trace("cmd c,C, %d, %d, %d\n", update_data1, update_data2, update_data3);
		snprintf(buf, PAGE_SIZE, "%02x:%08x:%08x\n", update_data1, update_data2, update_data3);
		ret = strlen(buf) + 1;
		break;
	case 'e':
	case 'E':
		icn85xx_trace("cmd e,E, %d, %d, %d\n", update_data1, update_data2, update_data3);
		snprintf(buf, PAGE_SIZE, "%02x:%08x:%08x\n", update_data1, update_data2, update_data3);
		ret = strlen(buf) + 1;
		break;
	default:
		icn85xx_trace("this conmand is unknow!!\n");
		break;
	}

	return ret;
}

/* open & short test macro\vars\ function */
#define PHYSICAL_MAX_NUM_ROW                      (42)
#define PHYSICAL_MAX_NUM_COL                      (30)
/*  icn86xx reg map */
#define  REG_SCAN_RXSEL0           (0x00040800 + 0x0000)
#define  REG_SCAN_RXSEL4           (0x00040800 + 0x0010)
#define  ANAREG6                   (0x00040100 + 0x0094)

/* hostcom addr map */
#define PANEL_PARA_OFFSET_RX_ORDER (0X8030)
#define BIT_5                      (0x00000020)
#define BIT_0                      (0x00000001)


short log_rawdata_temp[COL_NUM][ROW_NUM] = {{0, 0} };
static u8 rx_format_1010[PHYSICAL_MAX_NUM_COL] = {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0};
static u8 rx_format_0101[PHYSICAL_MAX_NUM_COL] = {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1};
static u8 rx_format_1100[PHYSICAL_MAX_NUM_COL] = {1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1};
static u8 rx_format_0011[PHYSICAL_MAX_NUM_COL] = {0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0};
/* static u8 rx_format_0000[PHYSICAL_MAX_NUM_COL] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; */

static u8 rx_order[PHYSICAL_MAX_NUM_COL] = {0};

/***********************************************************************************************
Name    :   icn85xx_read_multibyte
Input   :   addr
		pdata
		len
Output  :
function    :   read register of icn85xx, normal mode
***********************************************************************************************/
int icn85xx_read_multibyte(unsigned short addr, char *pdata, int len)
{
	int ret = -1;
	int i = 0;

	ret = icn85xx_i2c_rxdata(addr, pdata, len);
	icn85xx_trace("Addr: 0x%x, length: %d, ret: %d\n", addr, len, ret);
	for (i = 0; i < len; i++)
		icn85xx_trace(" %d,", *(pdata+i));
	icn85xx_trace("\n");

	return ret;
}

/***********************************************************************************************
Name    :   icn85xx_debug_read_byte
Input   :   addr
		pdata
Output  :
function    :   read register of icn85xx by debug interface, normal mode
***********************************************************************************************/
int icn85xx_debug_read_byte(unsigned int regAddr, char *pdata)
{
	int ret = -1;

	ret = icn85xx_i2c_txdata(0xf000, (char *)(&regAddr), 4);
	if (ret < 0) {
		icn85xx_error("debug read reg addr failed! ret: %d\n", ret);
		return -EPERM;
	}
	ret = icn85xx_i2c_rxdata(0xf004, pdata, 1);
	icn85xx_trace("read: 0x%x, 0x%x\n", regAddr, *pdata);
	if (ret < 0) {
		icn85xx_error("debug read reg value failed! ret: %d\n", ret);
		return -EPERM;
	}

	return ret;
}

/***********************************************************************************************
Name    :   icn85xx_debug_write_byte
Input   :   addr
		pdata
Output  :
function    :   write register of icn85xx by debug interface, normal mode
***********************************************************************************************/
int icn85xx_debug_write_byte(unsigned int addr, char *pdata)
{
	int ret = -1;

	ret = icn85xx_i2c_txdata(0xf000, (char *)(&addr), 4);
	if (ret < 0) {
		icn85xx_error("debug write reg addr failed! ret: %d\n", ret);
		return -EPERM;
	}
	ret = icn85xx_i2c_txdata(0xf004, pdata, 1);
	if (ret < 0) {
		icn85xx_error("debug write reg value failed! ret: %d\n", ret);
		return -EPERM;
	}

	return ret;
}

/***********************************************************************************************
Name    :   icn85xx_config_rx_as_fromatp
Input   :   addr
		pdata
Output  :
function    :   config rx base on fromat define.
***********************************************************************************************/
int icn85xx_config_rx_as_fromat (char *rx_format, char column)
{
	int ret = -1;
	int i = 0;
	int regAddr = 0;
	char regValue = 0;

	if ((NULL == rx_format) || (column > PHYSICAL_MAX_NUM_COL)) {
		icn85xx_error("Error:ptr Null or column len expand boundary!\n");
		return ret;
	}

	for (i = 0; i < column; i++) {
		if (1 == rx_format[i]) {/* set 1 */
			if (0 == rx_order[i] % 2) {/* group a */
				regAddr = REG_SCAN_RXSEL0 + (rx_order[i] >> 1);
				regValue = (BIT_5 | i);
				icn85xx_debug_write_byte (regAddr, &regValue);
			} else {/* group b */
				regAddr = REG_SCAN_RXSEL4 + (rx_order[i] >> 1);
				regValue = (BIT_5 | i);
				icn85xx_debug_write_byte (regAddr, &regValue);
			}
		} else if (0 == rx_format[i]) {/* set 0 */
			if (0 == rx_order[i]%2) {/* group a */
				regAddr = REG_SCAN_RXSEL0 + (rx_order[i] >> 1);
				regValue = ((~BIT_5) & i);
				icn85xx_debug_write_byte (regAddr, &regValue);
			} else {/* group b */
				regAddr = REG_SCAN_RXSEL4 + (rx_order[i] >> 1);
				regValue = ((~BIT_5) & i);
				icn85xx_debug_write_byte (regAddr, &regValue);
			}
		}
	}
	icn85xx_trace("after config:\n");
	for (i = 0; i < column; i++)
		icn85xx_debug_read_byte(REG_SCAN_RXSEL0 + (rx_order[i] >> 1), &regValue);
	icn85xx_trace("\n");

	return ret;
}

/***********************************************************************************************
Name    :   icn85xx_test_odd_even
Input   :    vol                 : tx voltage
					max_diff_ref   : max diff reference
					min_diff_ref    : min diff reference
					column           : column nums used
					columnMax     : the max rx num of the package
Output  :   err type
function    :   test TP
***********************************************************************************************/
static char icn85xx_test_odd_even (int vol, int max_diff_ref, int min_diff_ref,
							char row, char rowMax, char column, char columnMax)
{
	char vol_local = 0;
	int diffMax = 0;
	int diffMin = 100000;
	char ret = 0;
	int i, j;

	/* normal voltage: */
	icn85xx_write_reg(4, 0x10);/* set normal tx voltage */

	icn85xx_trace("normal scan RX 1010:\n");
	icn85xx_config_rx_as_fromat(rx_format_1010, column);

	for (i = 0; i < 4; i++)/* capture odd data */
		icn85xx_log(0);

	memcpy(&log_rawdata_temp[0][0], &log_rawdata[0][0], columnMax * rowMax * 2);

	icn85xx_trace("normal scan RX 0101:\n");
	icn85xx_config_rx_as_fromat(rx_format_0101, column);

	for (i = 0; i < 4; i++)/* capture even data */
		icn85xx_log(0);
	icn85xx_trace("combine even&odd data:\n");
	for (j = 0; j < column; j++) {
		if (0 == j%2)
			for (i = 0; i < row; i++)
				log_rawdata[i][j] = log_rawdata_temp[i][j];
	}

	for (i = 0; i < row; i++)
		icn85xx_rawdatadump(&log_rawdata[i][0], column, columnMax);
	memcpy(&log_basedata[0][0], &log_rawdata[0][0], columnMax * rowMax * 2);

	/* decrease the voltage */
	vol_local = vol & 0x0f;
	while (vol_local > 6) {
		icn85xx_write_reg(4, 0x10 | 6);
		vol_local -= 6;
	}
	if (vol_local)
		icn85xx_write_reg(4, 0x10 | vol_local);

	icn85xx_trace("low voltage scan RX 1010:\n");

	icn85xx_config_rx_as_fromat(rx_format_1010, column);

	for (i = 0; i < 4; i++)/* capture odd data */
		icn85xx_log(0);
	memcpy(&log_rawdata_temp[0][0], &log_rawdata[0][0], columnMax * rowMax * 2);

	icn85xx_trace("low voltage scan RX 0101:\n");
	icn85xx_config_rx_as_fromat(rx_format_0101, column);
	for (i = 0; i < 4; i++)/* capture even data */
		icn85xx_log(0);

	icn85xx_trace("low voltage combine even&odd data:\n");
	for (j = 0; j < column; j++)
		if (0 == j % 2)
			for (i = 0; i < row; i++)
				log_rawdata[i][j] = log_rawdata_temp[i][j];
	for (i = 0; i < row; i++)
		icn85xx_rawdatadump(&log_rawdata[i][0], column, columnMax);
	/* make odd&even diff to judge the result */

	update_data2 = max_diff_ref*120/100;
	update_data3 = min_diff_ref*80/100;

	for (i = 0; i < row; i++) {
		for (j = 0; j < column; j++) {
			log_diffdata[i][j] = log_basedata[i][j] - log_rawdata[i][j];
			if (log_diffdata[i][j] > diffMax)
				diffMax = log_diffdata[i][j];
			else if (log_diffdata[i][j] < diffMin)
				diffMin = log_diffdata[i][j];

			if ((update_data2 > 0) && (update_data3 > 0))/* make sure Max/Min > 0 */
				if ((log_diffdata[i][j] > update_data2) || (log_diffdata[i][j] < update_data3))
					ret = 1;
		}
		icn85xx_rawdatadump(&log_diffdata[i][0], column, columnMax);
	}

	memcpy(&log_diff_combine_1010[0][0], &log_diffdata[0][0], columnMax * rowMax * 2);

	update_data2 = diffMax;
	update_data3 = diffMin;

	icn85xx_trace("odd&even RX short test End!\n");

	return ret;
}


/***********************************************************************************************
Name    :   icn85xx_test_1st_3rd
Input   :    vol                 : tx voltage
		max_diff_ref   : max diff reference
		min_diff_ref    : min diff reference
		column           : column nums used
		columnMax     : the max rx num of the package
Output  :   err type
function    :   test TP
***********************************************************************************************/
static char icn85xx_test_1st_3rd (int vol, int max_diff_ref, int min_diff_ref,
						char row, char rowMax, char column, char columnMax)
{
	char vol_local = 0;
	int diffMax = 0;
	int diffMin = 100000;
	char ret = 0;
	char temp_data = 0;
	int i, j;

	icn85xx_trace("1st and 3rd RX short test start:\n");
	/* set normal tx voltage */
	icn85xx_write_reg(4, 0x10);

	icn85xx_trace("normal scan RX 1100:\n");
	icn85xx_config_rx_as_fromat(rx_format_1100, column);

	for (i = 0; i < 4; i++)
		icn85xx_log(0);
	memcpy(&log_rawdata_temp[0][0], &log_rawdata[0][0], columnMax * rowMax * 2);

	icn85xx_trace("normal scan RX 0011:\n");
	icn85xx_config_rx_as_fromat(rx_format_0011, column);

	/* capture even data */
	for (i = 0; i < 4; i++)
		icn85xx_log(0);

	icn85xx_trace("combine 1st&3rd data:\n");
	for (j = 0; j < column; j++) {
		temp_data = (j >> 1);
		if (0 == temp_data % 2)
			for (i = 0; i < row; i++)
				log_rawdata[i][j] = log_rawdata_temp[i][j];
	}

	for (i = 0; i < row; i++)
		icn85xx_rawdatadump(&log_rawdata[i][0], column, columnMax);

	memcpy(&log_basedata[0][0], &log_rawdata[0][0], columnMax * rowMax * 2);

	/* decrease the voltage */
	vol_local = vol & 0x0f;
	icn85xx_trace("-------vol: 0x%x\n", vol_local);
	while (vol_local > 6) {
		icn85xx_write_reg(4, 0x10 | 6);
		vol_local -= 6;
	}
	if (vol_local)
		icn85xx_write_reg(4, 0x10 | vol_local);
	icn85xx_trace("low voltage scan RX 1100:\n");

	icn85xx_config_rx_as_fromat(rx_format_1100, column);

	/* capture 1st and 2nd data */
	for (i = 0; i < 4; i++)
		icn85xx_log(0);

	memcpy(&log_rawdata_temp[0][0], &log_rawdata[0][0], columnMax * rowMax * 2);

	icn85xx_trace("low voltage scan RX 0011:\n");

	icn85xx_config_rx_as_fromat(rx_format_0011, column);
	/* capture 3rd and 4th data */
	for (i = 0; i < 4; i++)
		icn85xx_log(0);

	icn85xx_trace("low voltage combine 1st 2nd 3rd 4th data:\n");
	for (j = 0; j < column; j++) {
		temp_data = (j >> 1);
		if (0 == temp_data % 2)
			for (i = 0; i < row; i++)
				log_rawdata[i][j] = log_rawdata_temp[i][j];
	}
	for (i = 0; i < row; i++)
		icn85xx_rawdatadump(&log_rawdata[i][0], column, columnMax);

	/* make odd&even diff to judge the result */
	update_data4 = max_diff_ref * 120 / 100;
	update_data5 = min_diff_ref * 80 / 100;
	icn85xx_trace("-------diff data: %d, %d\n", update_data4, update_data5);
	for (i = 0; i < row; i++) {
		for (j = 0; j < column; j++) {
			log_diffdata[i][j] = log_basedata[i][j] - log_rawdata[i][j];
			if (log_diffdata[i][j] > diffMax)
				diffMax = log_diffdata[i][j];
			else if (log_diffdata[i][j] < diffMin)
				diffMin = log_diffdata[i][j];

			if ((update_data4 > 0) && (update_data5 > 0))
				if ((log_diffdata[i][j] > update_data4) || (log_diffdata[i][j] < update_data5))
					ret = 1;
		}
		icn85xx_rawdatadump(&log_diffdata[i][0], column, columnMax);
	}

	update_data4 = diffMax;
	update_data5 = diffMin;
	memcpy(&log_diff_combine_1100[0][0], &log_diffdata[0][0], columnMax * rowMax * 2);

	icn85xx_trace("1st and 3rd RX short test End!\n");
	return ret;
}

/***********************************************************************************************
Name    :   icn85xx_testTP
Input   :   0: ca, 1: diff test
Output  :   err type
function    :   test TP
***********************************************************************************************/
static int icn85xx_testTP(char type, int para1, int para2, int para3)
{
	char ret_odd_even = 0;
	char ret_1st_3rd = 0;
	char retvalue = 0;
	char row = 0;
	char column = 0;
	char rowMax = 0;
	char columnMax = 0;
	char ScanMode = 0;
	char ictype = 0x85;
	char regValue;
	int i = 0, j = 0;
	char bak_0x40b38 = 0;
	char bak_0x40195 = 0;
	char work_mode = 0;
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(this_client);

	/** - Set testing flag for disable ESD check */
	icn85xx_ts->icn_is_testing = true;

	/** - Disable IRQ */
	icn85xx_irq_disable();

	if ((icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_85)
			|| (icn85xx_ts->ictype == ICN85XX_WITH_FLASH_85)) {
		rowMax = 36;
		columnMax = 24;
		ictype = 85;
	} else if ((icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_86)
			|| (icn85xx_ts->ictype == ICN85XX_WITH_FLASH_86)) {
		rowMax = 42;
		columnMax = 30;
		ictype = 86;
	} else if ((icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_87)
			|| (icn85xx_ts->ictype == ICN85XX_WITH_FLASH_87)) {
		rowMax = 23;
		columnMax = 12;
		ictype = 87;
	}

	icn85xx_read_reg(0x8004, &row);
	icn85xx_read_reg(0x8005, &column);

	row--; /* remove the data of virtual key */
	icn85xx_ts->work_mode = 4; /* idle */
	if (type == 0) {
		memset(&log_basedata[0][0], 0, columnMax * rowMax * 2);
		icn85xx_log(0);
		for (i = 0; i < row; i++)
			for (j = 0; j < (column-1); j++)
				log_basedata[i][j] = log_rawdata[i][j+1] - log_rawdata[i][j];

		for (i = 0; i < (row - 1); i++) {
			for (j = 0; j < (column-1); j++) {
				log_rawdata[i][j]  = log_basedata[i+1][j] - log_basedata[i][j];
				if (abs(log_rawdata[i][j]) >  abs(para1))
					retvalue = 1;  /* error */
			}
			icn85xx_rawdatadump(&log_rawdata[i][0], column - 1, columnMax);
		}

	} else if (type == 1) {
		memset(&log_rawdata[0][0], 0, columnMax * rowMax * 2);
		if (ictype == 86) {
			icn85xx_trace("Config before:\n");
			icn85xx_debug_read_byte(ANAREG6 + 1, &regValue);
			icn85xx_debug_read_byte(0x040b38, &regValue);
			icn85xx_read_reg(0, &regValue);
			icn85xx_read_reg(0x8000 + 87, &regValue);

			icn85xx_read_reg(0x8000 + 87, &ScanMode);
			icn85xx_trace("old scan mode: %d\n", ScanMode);
			/* change scanmode to single mode */
			icn85xx_write_reg(0x8000 + 87, 0);
			/*read now work mode */
			icn85xx_read_reg(0, &work_mode);
			icn85xx_trace("old work mode: %d\n", work_mode);
			/* switch workmode to factory */
			icn85xx_write_reg(0, 1);
			mdelay(100);
			/* Pull down TX route */
			icn85xx_debug_read_byte(0x040b38, &bak_0x40b38);
			regValue = bak_0x40b38 & 0xfe;
			icn85xx_debug_write_byte(0x040b38, &regValue);
		}

		icn85xx_debug_read_byte((ANAREG6 + 1), &bak_0x40195);
		regValue = (bak_0x40195 | BIT_0);
		icn85xx_debug_write_byte((ANAREG6 + 1), &regValue);

		icn85xx_trace("RxOrder:\n");
		icn85xx_read_multibyte(PANEL_PARA_OFFSET_RX_ORDER, rx_order, column);

		icn85xx_trace("Config now:\n");
		icn85xx_debug_read_byte(ANAREG6 + 1, &regValue);
		icn85xx_debug_read_byte(0x040b38, &regValue);
		icn85xx_read_reg(0, &regValue);
		icn85xx_read_reg(0x8000 + 87, &regValue);

		/* open short scan */
		ret_odd_even = icn85xx_test_odd_even(para1, para2,
			para3, row, rowMax, column, columnMax);
		ret_1st_3rd = icn85xx_test_1st_3rd(para1, para2, para3,
			row, rowMax, column, columnMax);

		/* recover the config */
		icn85xx_write_reg(4, 0x10);
		if (ictype == 86) {
			icn85xx_write_reg(0x8000 + 87, ScanMode);
			icn85xx_write_reg(0, work_mode);
			mdelay(100);
			icn85xx_debug_write_byte(0x040b38, &bak_0x40b38);
			icn85xx_debug_write_byte((ANAREG6 + 1), &bak_0x40195);
		}
		icn85xx_trace("recover rx config:\n");
		for (i = 0; i < column; i++)
			icn85xx_debug_read_byte(REG_SCAN_RXSEL0 + (rx_order[i] >> 1), &regValue);
		icn85xx_trace("\n");
		icn85xx_debug_read_byte(ANAREG6 + 1, &regValue);
		icn85xx_debug_read_byte(0x040b38, &regValue);
		icn85xx_read_reg(0, &regValue);
		icn85xx_read_reg(0x8000 + 87, &regValue);

		icn85xx_trace("format 1010&0101 combination diff:\n");
		for (i = 0; i < row; i++)
			icn85xx_rawdatadump(&log_diff_combine_1010[i][0], column, columnMax);
		icn85xx_trace("format 1100&0011 combination diff:\n");

		for (i = 0; i < row; i++)
			icn85xx_rawdatadump(&log_diff_combine_1100[i][0], column, columnMax);
		icn85xx_trace(" Reference Max: %d, Min: %d\n", para2, para3);

		icn85xx_trace("1010&0101 TEST:\n");
		if (ret_odd_even)
			icn85xx_trace("FAIL!! ret:%d\n", ret_odd_even);
		else
			icn85xx_trace("PASS!! ret:%d\n", ret_odd_even);

		icn85xx_trace("RESULT MAX: %d, MIN: %d\n", update_data2, update_data3);

		icn85xx_trace("1100&0011 TEST:\n");
		if (ret_1st_3rd)
			icn85xx_trace("FAIL!! ret:%d\n", ret_1st_3rd);
		else
			icn85xx_trace("PASS!! ret:%d\n", ret_1st_3rd);

		icn85xx_trace("RESULT MAX: %d, MIN: %d\n", update_data4, update_data5);

		retvalue = (ret_odd_even | ret_1st_3rd);
	}
	icn85xx_ts->work_mode = 0;

	/** - Enable IRQ */
	icn85xx_irq_enable();

	/** Clear testing flag */
	icn85xx_ts->icn_is_testing = false;

	return retvalue;
}

static ssize_t icn85xx_store_update(struct device_driver *drv, const char *buf, size_t count)
{
	int i = 0, j = 0;
	char para_state = PARA_STATE_CMD;
	char cmd[16] = {0};
	char data1[128] = {0};
	char data2[16] = {0};
	char data3[16] = {0};
	char *p;
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(this_client);

	p = cmd;
	for (i = 0; i < count; i++) {
		if (buf[i] == ':') {
			if (PARA_STATE_CMD == para_state) {
				p[j] = '\0';
				para_state = PARA_STATE_DATA1;
				p = data1;
				j = 0;
			} else if (PARA_STATE_DATA1 == para_state) {
				p[j] = '\0';
				para_state = PARA_STATE_DATA2;
				p = data2;
				j = 0;
			} else if (PARA_STATE_DATA2 == para_state) {
				p[j] = '\0';
				para_state = PARA_STATE_DATA3;
				p = data3;
				j = 0;
			}

		} else {
			p[j++] =  buf[i];
		}
	}
	p[j] = '\0';

	update_cmd = cmd[0];
	switch (update_cmd) {
	case 'u':
	case 'U':
		icn85xx_trace("firmware: %s, %d\n", icn85xx_ts->fw_name, (int)strlen(icn85xx_ts->fw_name));
		for (i = 0; i < 40; i++)
			icn85xx_trace("0x%2x ", icn85xx_ts->fw_name[i]);
		icn85xx_trace("\n");
		memset(icn85xx_ts->fw_name, 0, 128);
		memcpy(icn85xx_ts->fw_name, data1, strlen(data1)-1);
		icn85xx_trace("firmware: %s, %d\n", icn85xx_ts->fw_name, (int)strlen(icn85xx_ts->fw_name));
		icn85xx_trace("Host FW Version : 0x%x\n", icn85xx_read_fw_Ver(icn85xx_ts->fw_name));
		icn85xx_trace("TP current version: 0x%x\n", icn85xx_readVersion());
		if (R_OK == icn85xx_fw_update(icn85xx_ts->fw_name))
			icn85xx_trace("update ok\n");
		else
			icn85xx_trace("update error\n");
		break;
	case 't':
	case 'T':
		icn85xx_trace("cmd t,T\n");
		icn85xx_ts_reset();
	break;
	case 'r':
	case 'R':
		icn85xx_trace("cmd r,R\n");
		icn85xx_log(0);
		break;
	case 'd':
	case 'D':
		icn85xx_trace("cmd d,D\n");
		icn85xx_log(1);
		break;
	case 'c':
	case 'C':
		update_data1 = simple_strtoul(data1, NULL, 10);
		icn85xx_trace("cmd c,C, %d\n", update_data1);
		update_data1 = icn85xx_testTP(0, update_data1, 0, 0);
		update_data2 = 0;
		update_data3 = 0;
		icn85xx_trace("cmd c,C, result: %d\n", update_data1);
		break;
	case 'e':
	case 'E':
		update_data1 = simple_strtoul(data1, NULL, 10);
		update_data2 = simple_strtoul(data2, NULL, 10);
		update_data3 = simple_strtoul(data3, NULL, 10);
		icn85xx_trace("cmd e,E, %d, %d, %d\n", update_data1, update_data2, update_data3);
		update_data1 = icn85xx_testTP(1, update_data1, update_data2, update_data3);
		if (update_data1 == 1)
			icn85xx_trace("Open&Short test fail: Find some short pins!!!\n");
		else
			icn85xx_trace("Open&Short test success!!!\n");
		break;
	default:
		icn85xx_trace("this conmand is unknow!!\n");
		break;
	}

	return count;
}

static DRIVER_ATTR(icn_fw_version, S_IRUGO | S_IWUSR, icn85xx_show_version, NULL);
static DRIVER_ATTR(icn_update, S_IRUGO | S_IWUSR, icn85xx_show_update, icn85xx_store_update);

static struct attribute *icn_drv_attr[] = {
	&driver_attr_icn_fw_version.attr,
	&driver_attr_icn_update.attr,
	NULL
};
static struct attribute_group icn_drv_attr_grp = {
	.attrs = icn_drv_attr,
};
static const struct attribute_group *icn_drv_grp[] = {
	&icn_drv_attr_grp,
	NULL
};

static ssize_t ts_fwupdate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}
#if 1
static ssize_t ts_fwupdate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(this_client);
	icn85xx_trace("firmware: %s, %d\n", icn85xx_ts->fw_name, (int)strlen(icn85xx_ts->fw_name));

	icn85xx_trace("firmware: %s, %d\n", icn85xx_ts->fw_name, (int)strlen(icn85xx_ts->fw_name));
	icn85xx_trace("Host FW Version : 0x%x\n", icn85xx_read_fw_Ver(icn85xx_ts->fw_name));
	icn85xx_trace("TP current version: 0x%x\n", icn85xx_readVersion());
	if (R_OK == icn85xx_fw_update(icn85xx_ts->fw_name)){
		icn85xx_trace("update ok\n");
#ifdef CONFIG_DEV_INFO
		save_tp_info();
#endif
	} else
		icn85xx_trace("update error\n");

	return count;
}
#else
static ssize_t ts_fwupdate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char path[256];

	memcpy(path, buf, min(count, sizeof(path) - 1));
	path[count] = '\0';

	printk("Update firmware: %s with version 0x%04x, current: 0x%04x\n",
		path, icn85xx_read_fw_Ver(path), icn85xx_readVersion());

	if (R_OK == icn85xx_fw_update(path))
		icn85xx_trace("update ok\n");
	else
		icn85xx_trace("update error\n");

	return count;
}
#endif
static DEVICE_ATTR(ts_fw_update, S_IRUGO | S_IWUSR, ts_fwupdate_show, ts_fwupdate_store);
#ifdef CONFIG_TOUCHSCREEN_ICN85XX_GESTURE
static ssize_t set_gesture_switch(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_gesture_switch(struct device *dev, struct device_attribute *attr, char *buf);
static DEVICE_ATTR(icn_gesture, S_IRUGO | S_IWUSR, get_gesture_switch, set_gesture_switch);
#endif

static struct attribute * icn_dev_attr[] = {
	&dev_attr_ts_fw_update.attr,
#ifdef CONFIG_TOUCHSCREEN_ICN85XX_GESTURE
	&dev_attr_icn_gesture.attr,
#endif
	NULL
};

static struct attribute_group icn_dev_attr_grp = {
	.attrs = icn_dev_attr,
};

int icn_create_sysfs(struct i2c_client * client)
{
	int err;
	
	err = sysfs_create_group(&client->dev.kobj, &icn_dev_attr_grp);
	if (0 != err) 
	{
		dev_err(&client->dev, "%s() - ERROR: sysfs_create_group() failed.\n", __func__);
		sysfs_remove_group(&client->dev.kobj, &icn_dev_attr_grp);
		return -EIO;
	} 
	else 
	{
		pr_info("icn:%s() - sysfs_create_group() succeeded.\n",__func__);
	}
	proc_symlink("tp_fw_upgrade", NULL, "/sys/bus/i2c/devices/3-0048/ts_fw_update");

	return err;
}

#endif/*SUPPORT_SYSFS*/

#if SUPPORT_PROC_FS
pack_head icn85xx_cmd_head;
static struct proc_dir_entry *icn85xx_proc_entry;
int  data_length = 0;
STRUCT_PANEL_PARA_H g_structPanelPara;

int icnt_pc_tool_i2c_rxdata(char dev_addr, char *rxdata, int length)
{
    int ret = -1;
    int retries = 0;
    struct i2c_msg msgs[] = {
        {
            .addr   = dev_addr,
            .flags  = I2C_M_RD,
            .len    = length,
            .buf    = rxdata,
        },
    };

    if (dev_addr == 0x78) {
        msgs[0].addr = this_client->addr;
    }
    while(retries < IIC_RETRY_NUM) {
        ret = i2c_transfer(this_client->adapter, msgs, 1);
        if(ret == 1)break;
        retries++;
    }
    if (retries >= IIC_RETRY_NUM) {
        icn85xx_error("%s i2c read error: %d\n", __func__, ret);
    }
    return ret;
}

int icnt_pc_tool_i2c_txdata(char dev_addr, char *txdata, int length)
{
    int ret = -1;
    int retries = 0;
    struct i2c_msg msgs[] = {
        {
            .addr   = dev_addr,
            .flags  = 0,
            .len    = length,
            .buf    = txdata,
        },
    };

    if (dev_addr == 0x78) {
        msgs[0].addr = this_client->addr;
    }
    while(retries < IIC_RETRY_NUM) {
        ret = i2c_transfer(this_client->adapter, msgs, 1);
        if(ret == 1)break;
        retries++;
    }
    if (retries >= IIC_RETRY_NUM) {
        icn85xx_error("%s i2c write error: %d\n", __func__, ret);
    }
    return ret;
}

static ssize_t icn85xx_tool_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	int ret = 0;
	int i;
	unsigned short addr;
	unsigned int prog_addr;
	char retvalue;
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(this_client);

	if (down_interruptible(&icn85xx_ts->sem))
		return -ENODEV;

	ret = copy_from_user(&icn85xx_cmd_head, buffer, CMD_HEAD_LENGTH);
	if (ret) {
		icn85xx_error("Tool copy write command header from user failed: %d\n", ret);
		goto write_out;
	} else
		ret = CMD_HEAD_LENGTH;

	if ((icn85xx_cmd_head.wr & 1) == 0) {
		icn85xx_trace("Tool write command %u prepare for read\n", icn85xx_cmd_head.wr);
		ret = CMD_HEAD_LENGTH + icn85xx_cmd_head.data_len;
		goto write_out;
	} else if (1 == icn85xx_cmd_head.wr) {
		ret = copy_from_user(&icn85xx_cmd_head.data[0], &buffer[CMD_HEAD_LENGTH], icn85xx_cmd_head.data_len);
		if (ret) {
			icn85xx_error("Tool copy panel param from user len: %u failed: %d\n",
				icn85xx_cmd_head.data_len, ret);
			goto write_out;
		}
		memcpy(&g_structPanelPara, &icn85xx_cmd_head.data[0], icn85xx_cmd_head.data_len);
		for (i = 0; i < icn85xx_cmd_head.data_len;) {
			int size = ((i + 64) > icn85xx_cmd_head.data_len) ? (icn85xx_cmd_head.data_len-i) : 64;
			ret = icn85xx_i2c_txdata(0x8000+i, &icn85xx_cmd_head.data[i], size);
			if (ret < 0) {
				icn85xx_error("Tool write panel param len: %u failed: %d\n",
					icn85xx_cmd_head.data_len, ret);
				goto write_out;
			}
			i = i + 64;
		}
		ret = icn85xx_cmd_head.data_len + CMD_HEAD_LENGTH;
		icn85xx_ts->work_mode = 4;
		icn85xx_trace("Tool switch to factory mode for update panel param\n");
		icn85xx_write_reg(0, 1);
		mdelay(100);
		icn85xx_trace("Tool switch back to normal mode for update panel param\n");
		icn85xx_write_reg(0, 0);
		mdelay(100);
		icn85xx_ts->work_mode = 0;
		goto write_out;
	} else if (3 == icn85xx_cmd_head.wr) {
		icn85xx_trace("Tool set firmware path: %.*s\n",
			(int)icn85xx_cmd_head.data_len, buffer + CMD_HEAD_LENGTH);

		ret = copy_from_user(&icn85xx_cmd_head.data[0], &buffer[CMD_HEAD_LENGTH], icn85xx_cmd_head.data_len);
		if (ret) {
			icn85xx_error("Tool copy firmware path: %.*s from user failed: %d\n",
				(int)icn85xx_cmd_head.data_len, buffer + CMD_HEAD_LENGTH, ret);
			goto write_out;
		}

		ret = icn85xx_cmd_head.data_len + CMD_HEAD_LENGTH;
		memset(icn85xx_ts->fw_name, 0, 128);
		memcpy(icn85xx_ts->fw_name, &icn85xx_cmd_head.data[0], icn85xx_cmd_head.data_len);
	} else if (5 == icn85xx_cmd_head.wr) {
		icn85xx_trace("Tool update firmware path: %s\n", icn85xx_ts->fw_name);
		icn85xx_update_status(1);

		if (!((icn85xx_ts->ictype == ICN85XX_WITH_FLASH_87) || (icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_87)))
			ret = icn85xx_fw_update(icn85xx_ts->fw_name);

		if (ret != R_OK)
			icn85xx_error("Tool update firmware path: %s failed: %d\n",
				icn85xx_ts->fw_name, ret);
	}
	else if (11 == icn85xx_cmd_head.wr) {
		icn85xx_ts->work_mode = icn85xx_cmd_head.flag;
		//structGestureData.u8Status = 0;

		ret = copy_from_user(&icn85xx_cmd_head.data[0], &buffer[CMD_HEAD_LENGTH], icn85xx_cmd_head.data_len);
		if (ret) {
			icn85xx_error("Tool copy 1-byte written to host comm from user failed: %d\n", ret);
			goto write_out;
		}
		addr = (icn85xx_cmd_head.addr[1]<<8) | icn85xx_cmd_head.addr[0];

		icn85xx_trace("Tool write host comm 1-byte addr: 0x%04x, value: 0x%02x\n",
			addr, icn85xx_cmd_head.data[0]);
		icn85xx_write_reg(addr, icn85xx_cmd_head.data[0]);
	}
	else if (13 == icn85xx_cmd_head.wr) {/* adc enable */
		icn85xx_trace("Tool enable adc\n");
		icn85xx_ts->work_mode = 4;
		mdelay(10);
		/* set col */
		icn85xx_write_reg(0x8000+offsetof(STRUCT_PANEL_PARA_H, u8ColNum), 1);
		/* u8RXOrder[0] = u8RXOrder[icn85xx_cmd_head.addr[0]]; */
		icn85xx_write_reg(0x8000+offsetof(STRUCT_PANEL_PARA_H, u8RXOrder[0]),
		g_structPanelPara.u8RXOrder[icn85xx_cmd_head.addr[0]]);
		/* set row */
		icn85xx_write_reg(0x8000+offsetof(STRUCT_PANEL_PARA_H, u8RowNum), 1);
		/* u8TXOrder[0] = u8TXOrder[icn85xx_cmd_head.addr[1]];    */
		icn85xx_write_reg(0x8000+offsetof(STRUCT_PANEL_PARA_H, u8TXOrder[0]),
		g_structPanelPara.u8TXOrder[icn85xx_cmd_head.addr[1]]);
		/* scan mode */
		icn85xx_write_reg(0x8000+offsetof(STRUCT_PANEL_PARA_H, u8ScanMode), 0);
		/* bit */
		icn85xx_write_reg(0x8000+offsetof(STRUCT_PANEL_PARA_H, u16BitFreq), 0xD0);
		icn85xx_write_reg(0x8000+offsetof(STRUCT_PANEL_PARA_H, u16BitFreq) + 1, 0x07);
		/* freq */
		icn85xx_write_reg(0x8000+offsetof(STRUCT_PANEL_PARA_H, u16FreqCycleNum[0]), 0x64);
		icn85xx_write_reg(0x8000+offsetof(STRUCT_PANEL_PARA_H, u16FreqCycleNum[0]) + 1, 0x00);
		/* pga */
		icn85xx_write_reg(0x8000+offsetof(STRUCT_PANEL_PARA_H, u8PgaGain), 0x0);
		/* config mode */
		icn85xx_write_reg(0, 0x2);
		mdelay(1);
		icn85xx_read_reg(2, &retvalue);
		icn85xx_trace("retvalue0: %d\n", retvalue);
		while (retvalue != 1) {
			icn85xx_trace("retvalue: %d\n", retvalue);
			mdelay(1);
			icn85xx_read_reg(2, &retvalue);
		}

		if (icn85xx_goto_progmode() != 0) {
			icn85xx_trace("icn85xx_goto_progmode() != 0 error\n");
			goto write_out;
		}
		icn85xx_prog_write_reg(0x040870, 1);
	} else if (15 == icn85xx_cmd_head.wr) {/* write hostcomm multibyte */
		ret = copy_from_user(&icn85xx_cmd_head.data[0], &buffer[CMD_HEAD_LENGTH], icn85xx_cmd_head.data_len);
		if (ret) {
			icn85xx_error("Tool copy multi-byte written to host comm from user failed: %d\n", ret);
			goto write_out;
		}
		addr = (icn85xx_cmd_head.addr[1]<<8) | icn85xx_cmd_head.addr[0];
		icn85xx_trace("Tool write multi-byte to host comm addr: 0x%04x len: %u data:\n%*ph\n",
			addr, icn85xx_cmd_head.data_len,
			icn85xx_cmd_head.data_len, icn85xx_cmd_head.data);

		ret = icn85xx_i2c_txdata(addr, &icn85xx_cmd_head.data[0], icn85xx_cmd_head.data_len);
		if (ret < 0) {
			icn85xx_error("Tool write multi-byte to host comm addr: 0x%04x len: %u failed: %d\n",
				addr, icn85xx_cmd_head.data_len, ret);
			goto write_out;
		}
	} else if (17 == icn85xx_cmd_head.wr) {/* write iic porgmode multibyte */
		ret = copy_from_user(&icn85xx_cmd_head.data[0], &buffer[CMD_HEAD_LENGTH], icn85xx_cmd_head.data_len);
		if (ret) {
			icn85xx_error("Tool copy multi-byte written to program mode from user failed: %d\n", ret);
			goto write_out;
		}
		prog_addr = (icn85xx_cmd_head.flag << 16) | (icn85xx_cmd_head.addr[1] << 8) | icn85xx_cmd_head.addr[0];
		icn85xx_goto_progmode();
		icn85xx_trace("Tool write multi-byte to program mode addr: 0x%06x len: %u data:\n%*ph\n",
			prog_addr, icn85xx_cmd_head.data_len,
			icn85xx_cmd_head.data_len, icn85xx_cmd_head.data);

		ret = icn85xx_prog_i2c_txdata(prog_addr, &icn85xx_cmd_head.data[0], icn85xx_cmd_head.data_len);
		if (ret < 0) {
			icn85xx_error("Tool write multi-byte to program mode addr: 0x%06x len: %u failed: %d\n",
				prog_addr, icn85xx_cmd_head.data_len, ret);
			goto write_out;
		}
	} else if (19 == icn85xx_cmd_head.wr){// write icnt iic by pc tool
        ret = copy_from_user(&icn85xx_cmd_head.data[0], &buffer[CMD_HEAD_LENGTH], icn85xx_cmd_head.data_len);
        if (ret) {
            icn85xx_error("Tool copy direct write data from user to dev_addr: 0x%02x len: %u failed: %d\n",
				icn85xx_cmd_head.addr[0], icn85xx_cmd_head.data_len, ret);
            goto write_out;
        }

		icn85xx_trace("Tool direct write to dev_addr: 0x%02x len: %u data:\n%*ph\n",
			icn85xx_cmd_head.addr[0], icn85xx_cmd_head.data_len,
			icn85xx_cmd_head.data_len, icn85xx_cmd_head.data);

         ret = icnt_pc_tool_i2c_txdata(icn85xx_cmd_head.addr[0], &icn85xx_cmd_head.data[0], icn85xx_cmd_head.data_len);
        if (ret < 0) {
            icn85xx_error("Tool direct write to dev_addr: 0x%02x len: %u failed: %d\n",
				icn85xx_cmd_head.addr[0], icn85xx_cmd_head.data_len, ret);
            goto write_out;
        }
	} else if ((icn85xx_cmd_head.wr & 1) != 0) {
		icn85xx_error("Tool write invalid command %u\n", icn85xx_cmd_head.wr);
	}

write_out:
	up(&icn85xx_ts->sem);

	return count;
}

static ssize_t icn85xx_tool_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
	int i, j;
	int ret = 0;
	char row, column, retvalue, max_column;
	unsigned short addr;
	unsigned int prog_addr;
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(this_client);

	if (down_interruptible(&icn85xx_ts->sem))
		return -EPERM;

	if (0 == icn85xx_cmd_head.wr) {
		icn85xx_trace("Tool read panel param len: %u\n", icn85xx_cmd_head.data_len);
		ret = icn85xx_i2c_rxdata(0x8000, (char *)(&g_structPanelPara), icn85xx_cmd_head.data_len);
		if (ret < 0)
			icn85xx_error("Tool read panel param failed: %d\n", ret);

		ret = copy_to_user(buffer, (void *)(&g_structPanelPara), icn85xx_cmd_head.data_len);
		if (ret) {
			icn85xx_error("Tool copy panel param to user failed: %d\n", ret);
			goto read_out;
		}
		goto read_out;

	} else if (2 == icn85xx_cmd_head.wr) {/* get update status */
		retvalue = icn85xx_get_status();
		icn85xx_trace("Tool read update status: %d\n", retvalue);
		ret = copy_to_user(buffer, (void *)(&retvalue), 1);
		if (ret) {
			icn85xx_error("Tool copy update status to user failed: %d\n", ret);
			goto read_out;
		}
	} else if (4 == icn85xx_cmd_head.wr) {/* read rawdata */
		icn85xx_trace("Tool read raw data\n");
		row = icn85xx_cmd_head.addr[1];
		column = icn85xx_cmd_head.addr[0];
		max_column = (icn85xx_cmd_head.flag == 0) ? (24) : (icn85xx_cmd_head.flag);
		/* scan tp rawdata */
		icn85xx_write_reg(4, 0x20);
		mdelay(1);
		for (i = 0; i < 1000; i++) {
			mdelay(1);
			icn85xx_read_reg(2, &retvalue);
			if (retvalue == 1)
				break;
		}

		for (i = 0; i < row; i++) {
			ret = icn85xx_i2c_rxdata(0x2000 + i*(max_column)*2, (char *) &log_rawdata[i][0], column*2);
			if (ret < 0)
				icn85xx_error("Tool read raw data from 0x%04x failed: %d\n", 0x2000 + i*(max_column)*2, ret);

			ret = copy_to_user(&buffer[column*2*i], (void *)(&log_rawdata[i][0]), column*2);
			if (ret) {
				icn85xx_error("Tool copy raw data to user failed: %d\n", ret);
				goto read_out;
			}
		}
		/* finish scan tp rawdata */
		icn85xx_write_reg(2, 0x0);
		icn85xx_write_reg(4, 0x21);
		goto read_out;
	} else if (6 == icn85xx_cmd_head.wr) {
		icn85xx_trace("Tool read diff data\n");
		row = icn85xx_cmd_head.addr[1];
		column = icn85xx_cmd_head.addr[0];
		max_column = (icn85xx_cmd_head.flag == 0) ? (24) : (icn85xx_cmd_head.flag);
		/* scan tp rawdata */
		icn85xx_write_reg(4, 0x20);
		mdelay(1);

		for (i = 0; i < 1000; i++) {
			mdelay(1);
			icn85xx_read_reg(2, &retvalue);
			if (retvalue == 1)
			break;
		}

		for (i = 0; i < row; i++) {
			ret = icn85xx_i2c_rxdata(0x3000 + (i + 1)*(max_column + 2) * 2 + 2, (char *)&log_diffdata[i][0], column * 2);
			if (ret < 0)
				icn85xx_error("Tool read diff data from 0x%04x failed: %d\n", 0x3000 + (i + 1)*(max_column + 2) * 2 + 2, ret);
			ret = copy_to_user(&buffer[column*2*i], (void *)(&log_diffdata[i][0]), column*2);
			if (ret) {
				icn85xx_error("Tool copy diff data to user failed: %d\n", ret);
				goto read_out;
			}
		}
		/* finish scan tp rawdata */
		icn85xx_write_reg(2, 0x0);
		icn85xx_write_reg(4, 0x21);

		goto read_out;
	} else if (8 == icn85xx_cmd_head.wr) {/* change TxVol, read diff */
		icn85xx_trace("Tool read diff data between change tx voltage\n");
		row = icn85xx_cmd_head.addr[1];
		column = icn85xx_cmd_head.addr[0];
		max_column = (icn85xx_cmd_head.flag == 0) ? (24) : (icn85xx_cmd_head.flag);
		/* scan tp rawdata */
		icn85xx_write_reg(4, 0x20);
		mdelay(1);
		for (i = 0; i < 1000; i++) {
			mdelay(1);
			icn85xx_read_reg(2, &retvalue);
			if (retvalue == 1)
				break;
		}

		for (i = 0; i < row; i++) {
			ret = icn85xx_i2c_rxdata(0x2000 + i*(max_column)*2, (char *) &log_rawdata[i][0], column*2);
			if (ret < 0)
				icn85xx_error("Tool read raw data before decrease voltage failed: %d\n", ret);
		}

		/* finish scan tp rawdata */
		icn85xx_write_reg(2, 0x0);
		icn85xx_write_reg(4, 0x21);

		icn85xx_write_reg(4, 0x12);

		/* scan tp rawdata */
		icn85xx_write_reg(4, 0x20);
		mdelay(1);
		for (i = 0; i < 1000; i++) {
			mdelay(1);
			icn85xx_read_reg(2, &retvalue);
			if (retvalue == 1)
				break;
		}

		for (i = 0; i < row; i++) {
			ret = icn85xx_i2c_rxdata(0x2000 + i * (max_column) * 2, (char *)&log_diffdata[i][0], column * 2);
			if (ret < 0)
				icn85xx_error("Tool read raw data after decrease voltage failed: %d\n", ret);
			for (j = 0; j < column; j++)
				log_basedata[i][j] = log_rawdata[i][j] - log_diffdata[i][j];
			ret = copy_to_user(&buffer[column*2*i], (void *)(&log_basedata[i][0]), column*2);
			if (ret) {
				icn85xx_error("Tool copy diff data between change tx voltage to user failed: %d\n", ret);
				goto read_out;
			}

		}
		/* finish scan tp rawdata */
		icn85xx_write_reg(2, 0x0);
		icn85xx_write_reg(4, 0x21);
		icn85xx_write_reg(4, 0x10);

		goto read_out;
	} else if (10 == icn85xx_cmd_head.wr) {
		icn85xx_trace("icn85xx_cmd_head_.wr == 10  \n");
		if (icn85xx_cmd_head.flag == 0)
			icn85xx_prog_write_reg(0x040874, 0);
		icn85xx_prog_read_page(2500 * icn85xx_cmd_head.flag,
			(char *)&log_diffdata[0][0], icn85xx_cmd_head.data_len);
		ret = copy_to_user(buffer, (void *)(&log_diffdata[0][0]), icn85xx_cmd_head.data_len);
		if (ret) {
			icn85xx_error("copy_to_user failed.\n");
			goto read_out;
		}

		if (icn85xx_cmd_head.flag == 9) {
			if ((icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_85)
				|| (icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_86)) {
				if (R_OK == icn85xx_fw_update(icn85xx_ts->fw_name)) {
					icn85xx_ts->code_loaded_flag = 1;
					icn85xx_trace("ICN85XX_WITHOUT_FLASH, reload code ok\n");
				} else {
					icn85xx_ts->code_loaded_flag = 0;
					icn85xx_trace("ICN85XX_WITHOUT_FLASH, reload code error\n");
				}
			} else {
				icn85xx_bootfrom_flash(icn85xx_ts->ictype);
				msleep(150);
			}
			icn85xx_ts->work_mode = 0;
		}
	} else if (12 == icn85xx_cmd_head.wr) {/* read hostcomm */
		addr = (icn85xx_cmd_head.addr[1]<<8) | icn85xx_cmd_head.addr[0];
		icn85xx_read_reg(addr, &retvalue);
		icn85xx_trace("Tool read host comm addr: 0x%04x, value: 0x%02x\n", addr, retvalue);
		ret = copy_to_user(buffer, (void *)(&retvalue), 1);
		if (ret) {
			icn85xx_error("Tool copy host comm read back data to user failed: %d\n", ret);
			goto read_out;
		}
	} else if (14 == icn85xx_cmd_head.wr) {/* read adc status */
		icn85xx_prog_read_reg(0x4085E, &retvalue);
		icn85xx_trace("Tool read adc status: 0x%02x\n", retvalue);
		ret = copy_to_user(buffer, (void *)(&retvalue), 1);
		if (ret) {
			icn85xx_error("Tool copy adc status to user failed: %d\n", ret);
			goto read_out;
		}
	}
#ifdef CONFIG_TOUCHSCREEN_ICN85XX_GESTURE
	else if (16 == icn85xx_cmd_head.wr) {/* read gesture data */
		icn85xx_trace("Tool read gesture data\n");
		ret = copy_to_user(buffer, (void *)(&structGestureData), sizeof(structGestureData));
		if (ret) {
			icn85xx_error("Tool copy gesture data to user failed: %d\n", ret);
			goto read_out;
		}

		if (structGestureData.u8Status == 1)
			structGestureData.u8Status = 0;
	}
#endif
	else if (18 == icn85xx_cmd_head.wr) {/* read hostcomm multibyte */
		addr = (icn85xx_cmd_head.addr[1] << 8) | icn85xx_cmd_head.addr[0];
		ret = icn85xx_i2c_rxdata(addr, &icn85xx_cmd_head.data[0], icn85xx_cmd_head.data_len);
		if (ret < 0) {
			icn85xx_error("Tool read from host comm addr: 0x%04x, len: %u failed: %d\n",
				addr, icn85xx_cmd_head.data_len, ret);
			goto read_out;
		}
		icn85xx_trace("Tool read from host comm addr: 0x%04x, len: %u data:\n%*ph\n",
			addr, icn85xx_cmd_head.data_len,
			icn85xx_cmd_head.data_len, icn85xx_cmd_head.data);
		ret = copy_to_user(buffer, &icn85xx_cmd_head.data[0], icn85xx_cmd_head.data_len);
		if (ret)
			icn85xx_error("Tool copy host comm multi-byte to user failed: %d\n", ret);

		goto read_out;

	} else if (20 == icn85xx_cmd_head.wr) {/* read iic porgmode multibyte */
		prog_addr = (icn85xx_cmd_head.flag << 16)
			| (icn85xx_cmd_head.addr[1] << 8) | icn85xx_cmd_head.addr[0];
		icn85xx_trace("Tool read from program mode addr: 0x%06x, len: %u\n",
			prog_addr, icn85xx_cmd_head.data_len);

		icn85xx_goto_progmode();

		ret = icn85xx_prog_i2c_rxdata(prog_addr, &icn85xx_cmd_head.data[0], icn85xx_cmd_head.data_len);
		if (ret < 0) {
			icn85xx_error("Tool read from program mode addr: 0x%06x, len: %u failed: %d\n",
				prog_addr, icn85xx_cmd_head.data_len, ret);
			goto read_out;
		}

		icn85xx_trace("Tool read from program mode addr: 0x%06x, len: %u data:\n%*ph\n",
			prog_addr, icn85xx_cmd_head.data_len,
			icn85xx_cmd_head.data_len, icn85xx_cmd_head.data);
		ret = copy_to_user(buffer, &icn85xx_cmd_head.data[0], icn85xx_cmd_head.data_len);
		if (ret) {
			icn85xx_error("Tool copy read from program mode to user failed: %d\n", ret);
			goto read_out;
		}
		icn85xx_bootfrom_sram();
		goto read_out;
	} else if (22 == icn85xx_cmd_head.wr) {/* read ictype */
		icn85xx_trace("Tool read ic type: 0x%02x\n", icn85xx_ts->ictype);
		ret = copy_to_user(buffer, (void *)(&icn85xx_ts->ictype), 1);
		if (ret) {
			icn85xx_error("Tool copy ic type to user failed: %d\n", ret);
			goto read_out;
		}
	} else if (24 == icn85xx_cmd_head.wr) {// read icnt iic by pc tool
        ret = icnt_pc_tool_i2c_rxdata(icn85xx_cmd_head.addr[0],
			&icn85xx_cmd_head.data[0], icn85xx_cmd_head.data_len);
        if (ret < 0) {
            icn85xx_error("Tool direct read dev_addr: 0x%02x, len: %u failed: %d\n",
				icn85xx_cmd_head.addr[0], icn85xx_cmd_head.data_len, ret);
            goto read_out;
        }

        icn85xx_trace("Tool direct read dev_addr: 0x%02x, len: %u data:\n%*ph\n",
			icn85xx_cmd_head.addr[0], icn85xx_cmd_head.data_len,
			icn85xx_cmd_head.data_len, icn85xx_cmd_head.data);

        ret = copy_to_user(&buffer[CMD_HEAD_LENGTH], &icn85xx_cmd_head.data[0], icn85xx_cmd_head.data_len);
		if (ret)
		{
		    icn85xx_error("Tool direct read copy to user failed: %d\n", ret);
		    goto read_out;
		}
	} else {
		icn85xx_error("Tool read invalid command %u\n", icn85xx_cmd_head.wr);
	}

read_out:
	up(&icn85xx_ts->sem);
//	icn85xx_trace("%s out: %d, icn85xx_cmd_head.data_len: %d\n\n",
	//	__func__, count, icn85xx_cmd_head.data_len);
	return icn85xx_cmd_head.data_len;
}

static const struct file_operations icn85xx_proc_fops = {
	.owner = THIS_MODULE,
	.read = icn85xx_tool_read,
	.write = icn85xx_tool_write,
};

void init_proc_node(void)
{
	int i;
	memset(&icn85xx_cmd_head, 0, sizeof(icn85xx_cmd_head));
	icn85xx_cmd_head.data = NULL;

	i = 5;
	while ((!icn85xx_cmd_head.data) && i) {
		icn85xx_cmd_head.data = kzalloc(i * DATA_LENGTH_UINT, GFP_KERNEL);
		if (NULL != icn85xx_cmd_head.data)
			break;
		i--;
	}
	if (i) {
		/* DATA_LENGTH = i * DATA_LENGTH_UINT + GTP_ADDR_LENGTH; */
		data_length = i * DATA_LENGTH_UINT;
		icn85xx_trace("alloc memory size:%d.\n", data_length);
	} else {
		icn85xx_error("alloc for memory failed.\n");
		return ;
	}

	icn85xx_proc_entry = proc_create(ICN85XX_ENTRY_NAME,
		0666, NULL, &icn85xx_proc_fops);
	if (icn85xx_proc_entry == NULL) {
		icn85xx_error("Couldn't create proc entry!\n");
		return ;
	} else {
		icn85xx_trace("Create proc entry success!\n");
	}

	return ;
}

void uninit_proc_node(void)
{
	kfree(icn85xx_cmd_head.data);
	icn85xx_cmd_head.data = NULL;
	remove_proc_entry(ICN85XX_ENTRY_NAME, NULL);
}
#endif/*SUPPORT_PROC_FS*/

#if (SUPPORT_SYSFS || SUPPORT_PROC_FS)
static void icn85xx_log(char diff)
{
	char row = 0;
	char column = 0;
	int i, j, ret;
	char retvalue = 0;
	char columnMax = 0;
	char rowMax = 0;
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(this_client);

	/** -Get physical number of row and column */
	if ((icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_85)
		|| (icn85xx_ts->ictype == ICN85XX_WITH_FLASH_85)) {
		rowMax = 36;
		columnMax = 24;
	} else if ((icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_86)
		|| (icn85xx_ts->ictype == ICN85XX_WITH_FLASH_86)) {
		rowMax = 42;
		columnMax = 30;
	} else if ((icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_87)
		|| (icn85xx_ts->ictype == ICN85XX_WITH_FLASH_87)) {
		rowMax = 23;
		columnMax = 12;
	}

	/** - Get number of current used rows and columns */
	icn85xx_read_reg(0x8004, &row);
	icn85xx_read_reg(0x8005, &column);

	/** - Send command to enable get raw/diff data */
	icn85xx_write_reg(4, 0x20);
	mdelay(1);
	/** - Polling until scan complete */
	for (i = 0; i < 1000; i++) {
		mdelay(1);
		icn85xx_read_reg(2, &retvalue);
		if (retvalue == 1)
			break;
	}
	if (i >= 1000) {
		icn85xx_error("*****************************\n"
					  "%s wait for data ready flag failed\n"
					  "*****************************\n",
						__func__);
	}

	if (diff == 0) {
		for (i = 0; i < row; i++) {
			ret = icn85xx_i2c_rxdata(0x2000 + i*columnMax*2, (char *)&log_rawdata[i][0], column*2);
			if (ret < 0)
				icn85xx_error("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
			/* icn85xx_rawdatadump(&log_rawdata[i][0], column, columnMax); */
		}
		/** Enlarge data of first & last row for H pattern type. */
		for (i = 0; i < column; i++) {
			log_rawdata[0][i] += (log_rawdata[0][i] / 2);
			log_rawdata[row - 2][i] += (log_rawdata[row - 2][i] / 2);
		}
		icn85xx_trace("\n");
		for (i = 0; i < row; i++) {
			icn85xx_rawdatadump(&log_rawdata[i][0], column, columnMax);
		}
	}
	if (diff == 1) {
		icn85xx_trace("\n");
		for (i = 0; i < row; i++) {
			ret = icn85xx_i2c_rxdata(0x3000 + (i+1)*(columnMax + 2) * 2 + 2, (char *)&log_diffdata[i][0], column*2);
			if (ret < 0)
				icn85xx_error("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
			icn85xx_rawdatadump(&log_diffdata[i][0], column, columnMax);
		}
	} else if (diff == 2) {
		icn85xx_trace("\n");
		for (i = 0; i < row; i++) {
			ret = icn85xx_i2c_rxdata(0x2000 + i*columnMax*2, (char *)&log_rawdata[i][0], column*2);
			if (ret < 0)
				icn85xx_error("%s read_data i2c_rxdata failed: %d\n", __func__, ret);

			if ((log_basedata[0][0] != 0) || (log_basedata[0][1] != 0))
				for (j = 0; j < column; j++)
					log_rawdata[i][j] = log_basedata[i][j] - log_rawdata[i][j];

			icn85xx_rawdatadump(&log_rawdata[i][0], column, columnMax);
		}
		if ((log_basedata[0][0] == 0) && (log_basedata[0][1] == 0))
			memcpy(&log_basedata[0][0], &log_rawdata[0][0], columnMax * rowMax * 2);
	}

	/** - Clear data ready flag */
	icn85xx_write_reg(2, 0x0);
	/** - Send command to disable get raw/diff data */
	icn85xx_write_reg(4, 0x21);
}
#endif/*(SUPPORT_SYSFS || SUPPORT_PROC_FS)*/

/***********************************************************************************************
Name    :   icn85xx_ts_reset
Input   :   void
Output  :   ret
function    : this function is used to reset tp, you should not delete it
***********************************************************************************************/
void icn85xx_ts_reset(void)
{
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(this_client);

	if (gpio_is_valid(icn85xx_ts->rst_gpio)) {
		gpio_set_value_cansleep(icn85xx_ts->rst_gpio, 0);
		msleep(30);
		gpio_set_value_cansleep(icn85xx_ts->rst_gpio, 1);
		msleep(80);
	}
}

void icn85xx_set_prog_addr(void)
{
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(this_client);
		gpio_direction_output(icn85xx_ts->irq_gpio, 0);
		gpio_set_value_cansleep(icn85xx_ts->irq_gpio, 0);
		msleep(20);
		icn85xx_ts_reset();
		gpio_set_value_cansleep(icn85xx_ts->irq_gpio, 1);
		msleep(20);
		gpio_direction_input(icn85xx_ts->irq_gpio);
		msleep(20);
}
/***********************************************************************************************
Name    :   icn85xx_irq_disable
Input   :   void
Output  :   ret
function    : this function is used to disable irq
***********************************************************************************************/
void icn85xx_irq_disable(void)
{
	unsigned long irqflags;
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(this_client);
	icn85xx_trace("icn85xx_ts: ** %s sussecc ---------------------------------------------**\n", __func__);

	spin_lock_irqsave(&icn85xx_ts->irq_lock, irqflags);
	if (!icn85xx_ts->irq_is_disable) {
		icn85xx_ts->irq_is_disable = 1;
		disable_irq_nosync(icn85xx_ts->irq);
	}
	spin_unlock_irqrestore(&icn85xx_ts->irq_lock, irqflags);
}

/***********************************************************************************************
Name    :   icn85xx_irq_enable
Input   :   void
Output  :   ret
function    : this function is used to enable irq
***********************************************************************************************/
void icn85xx_irq_enable(void)
{
	unsigned long irqflags = 0;
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(this_client);
	icn85xx_trace("icn85xx_ts: ** %s sussecc =============================================**\n", __func__);

	spin_lock_irqsave(&icn85xx_ts->irq_lock, irqflags);
	if (icn85xx_ts->irq_is_disable) {
		enable_irq(icn85xx_ts->irq);
		icn85xx_ts->irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&icn85xx_ts->irq_lock, irqflags);
}

/***********************************************************************************************
Name    :   icn85xx_prog_i2c_rxdata
Input   :   addr
		*rxdata
		length
Output  :   ret
function    : read data from icn85xx, prog mode
***********************************************************************************************/
int icn85xx_prog_i2c_rxdata(unsigned int addr, char *rxdata, int length)
{
	int ret = -1;
	int retries = 0;

	unsigned char tmp_buf[3];
	struct i2c_msg msgs[] = {
		{
			.addr   = ICN85XX_PROG_IIC_ADDR,/* this_client->addr, */
			.flags  = 0,
			.len    = 3,
			.buf    = tmp_buf,
		},
		{
			.addr   = ICN85XX_PROG_IIC_ADDR,/* this_client->addr, */
			.flags  = I2C_M_RD,
			.len    = length,
			.buf    = rxdata,
		},
	};

	tmp_buf[0] = (unsigned char)(addr>>16);
	tmp_buf[1] = (unsigned char)(addr>>8);
	tmp_buf[2] = (unsigned char)(addr);

	while (retries < IIC_RETRY_NUM) {
		ret = i2c_transfer(this_client->adapter, msgs, 2);
		if (ret == 2)
			break;
			retries++;
	}

	if (retries >= IIC_RETRY_NUM) {
		icn85xx_error("%s i2c read error: %d\n", __func__, ret);
		/* icn85xx_ts_reset(); */
	}

	return ret;
}
/***********************************************************************************************
Name    :   icn85xx_prog_i2c_txdata
Input   :   addr
		*rxdata
		length
Output  :   ret
function    : send data to icn85xx , prog mode
***********************************************************************************************/
int icn85xx_prog_i2c_txdata(unsigned int addr, char *txdata, int length)
{
	int ret = -1;
	char tmp_buf[128];
	int retries = 0;
	struct i2c_msg msg[] = {
		{
			.addr   = ICN85XX_PROG_IIC_ADDR,/* this_client->addr, */
			.flags  = 0,
			.len    = length + 3,
			.buf    = tmp_buf,
		},
	};

	if (length > 125) {
		icn85xx_error("%s too big datalen = %d!\n", __func__, length);
		return -EPERM;
	}
	tmp_buf[0] = (unsigned char)(addr>>16);
	tmp_buf[1] = (unsigned char)(addr>>8);
	tmp_buf[2] = (unsigned char)(addr);

	if (length != 0 && txdata != NULL)
		memcpy(&tmp_buf[3], txdata, length);

	while (retries < IIC_RETRY_NUM) {
		ret = i2c_transfer(this_client->adapter, msg, 1);
		if (ret == 1)
			break;
		retries++;
	}

	if (retries >= IIC_RETRY_NUM) {
		icn85xx_error("%s i2c write error: %d\n", __func__, ret);
		/* icn85xx_ts_reset(); */
	}

	return ret;
}
/***********************************************************************************************
Name    :   icn85xx_prog_write_reg
Input   :   addr -- address
		para -- parameter
Output  :
function    :   write register of icn85xx, prog mode
***********************************************************************************************/
int icn85xx_prog_write_reg(unsigned int addr, char para)
{
	char buf[3];
	int ret = -1;

	buf[0] = para;
	ret = icn85xx_prog_i2c_txdata(addr, buf, 1);
	if (ret < 0) {
		icn85xx_error("%s write reg failed! %#x ret: %d\n", __func__, buf[0], ret);
		return -EPERM;
	}

	return ret;
}

/***********************************************************************************************
Name    :   icn85xx_prog_read_reg
Input   :   addr
		pdata
Output  :
function    :   read register of icn85xx, prog mode
***********************************************************************************************/
int icn85xx_prog_read_reg(unsigned int addr, char *pdata)
{
	int ret = -1;

	ret = icn85xx_prog_i2c_rxdata(addr, pdata, 1);
	return ret;
}

int icn85xx_prog_read_page(unsigned int Addr, unsigned char *Buffer, unsigned int Length)
{
	int ret = 0;
	unsigned int StartAddr = Addr;
	while (Length) {
		if (Length > MAX_LENGTH_PER_TRANSFER) {
			ret = icn85xx_prog_i2c_rxdata(StartAddr, Buffer, MAX_LENGTH_PER_TRANSFER);
			Length -= MAX_LENGTH_PER_TRANSFER;
			Buffer += MAX_LENGTH_PER_TRANSFER;
			StartAddr += MAX_LENGTH_PER_TRANSFER;
		} else {
			ret = icn85xx_prog_i2c_rxdata(StartAddr, Buffer, Length);
			Length = 0;
			Buffer += Length;
			StartAddr += Length;
			break;
		}
		icn85xx_error("\n icn85xx_prog_read_page StartAddr:0x%x, length: %d\n",
			StartAddr, Length);
	}
	if (ret < 0)
		icn85xx_error("\n icn85xx_prog_read_page failed! StartAddr:  0x%x, ret: %d\n",
			StartAddr, ret);
	else
		icn85xx_trace("\n icn85xx_prog_read_page, StartAddr 0x%x, Length: %d\n",
			StartAddr, Length);

	return ret;
}

/***********************************************************************************************
Name    :   icn85xx_i2c_rxdata
Input   :   addr
		*rxdata
		length
Output  :   ret
function    : read data from icn85xx, normal mode
***********************************************************************************************/
int icn85xx_i2c_rxdata(unsigned short addr, char *rxdata, int length)
{
	int ret = -1;
	int retries = 0;

	unsigned char tmp_buf[2];
	struct i2c_msg msgs[] = {
		{
			.addr   = this_client->addr,
			.flags  = 0,
			.len    = 2,
			.buf    = tmp_buf,
		},
		{
			.addr   = this_client->addr,
			.flags  = I2C_M_RD,
			.len    = length,
			.buf    = rxdata,
		},
	};

	tmp_buf[0] = (unsigned char)(addr >> 8);
	tmp_buf[1] = (unsigned char)(addr);

	while (retries < IIC_RETRY_NUM) {
		ret = i2c_transfer(this_client->adapter, msgs, 2);
		if (ret == 2)
			break;
		retries++;
	}

	if (retries >= IIC_RETRY_NUM) {
		icn85xx_error("%s i2c read error: %d\n", __func__, ret);
		/* icn85xx_ts_reset(); */
	}

	return ret;
}
/***********************************************************************************************
Name    :   icn85xx_i2c_txdata
Input   :   addr
		*rxdata
		length
Output  :   ret
function    : send data to icn85xx , normal mode
***********************************************************************************************/
int icn85xx_i2c_txdata(unsigned short addr, char *txdata, int length) /*0626 song*/
{
	int ret = -1;
	unsigned char tmp_buf[128];
	int retries = 0;

	struct i2c_msg msg[] = {
		{
			.addr   = this_client->addr,
			.flags  = 0,
			.len    = length + 2,
			.buf    = tmp_buf,
		},
	};

	if (length > 125) {
		icn85xx_error("%s too big datalen = %d!\n", __func__, length);
		return -EPERM;
	}

	tmp_buf[0] = (unsigned char)(addr>>8);
	tmp_buf[1] = (unsigned char)(addr);

	if (length != 0 && txdata != NULL) {
		memcpy(&tmp_buf[2], txdata, length);
	}

	while (retries < IIC_RETRY_NUM) {
		ret = i2c_transfer(this_client->adapter, msg, 1);
		if (ret == 1)
			break;
		retries++;
	}

	if (retries >= IIC_RETRY_NUM) {
		icn85xx_error("%s i2c write error: %d\n", __func__, ret);
		icn85xx_ts_reset();
	}

	return ret;
}

/***********************************************************************************************
Name    :   icn85xx_write_reg
Input   :   addr -- address
		para -- parameter
Output  :
function    :   write register of icn85xx, normal mode
***********************************************************************************************/
int icn85xx_write_reg(unsigned short addr, char para)
{
	char buf[3];
	int ret = -1;

	buf[0] = para;
	ret = icn85xx_i2c_txdata(addr, buf, 1);
	if (ret < 0) {
		icn85xx_error("write reg failed! %#x ret: %d\n", buf[0], ret);
		return -EPERM;
	}

	return ret;
}


/***********************************************************************************************
Name    :   icn85xx_read_reg
Input   :   addr
		pdata
Output  :
function    :   read register of icn85xx, normal mode
***********************************************************************************************/
int icn85xx_read_reg(unsigned short addr, char *pdata)
{
	int ret = -1;

	ret = icn85xx_i2c_rxdata(addr, pdata, 1);
	return ret;
}

int icn87xx_prog_i2c_rxdata(unsigned int addr, char *rxdata, int length)
{
	int ret = -1;
	int retries = 0;

	unsigned char tmp_buf[2];
	struct i2c_msg msgs[] = {
		{
			.addr   = ICN87XX_PROG_IIC_ADDR,/*this_client->addr,*/
			.flags  = 0,
			.len    = 2,
			.buf    = tmp_buf,
		},
		{
			.addr   = ICN87XX_PROG_IIC_ADDR,/*this_client->addr,*/
			.flags  = I2C_M_RD,
			.len    = length,
			.buf    = rxdata,
		},
	};

	tmp_buf[0] = (unsigned char)(addr>>8);
	tmp_buf[1] = (unsigned char)(addr);

	while (retries < IIC_RETRY_NUM) {
		ret = i2c_transfer(this_client->adapter, msgs, 2);
		if (ret == 2)
			break;
		retries++;
	}

	if (retries >= IIC_RETRY_NUM)
		icn85xx_error("%s i2c read error: %d\n", __func__, ret);

	return ret;
}

int icn87xx_prog_i2c_txdata(unsigned int addr, char *txdata, int length)
{
	int ret = -1;
	char tmp_buf[128];
	int retries = 0;
	struct i2c_msg msg[] = {
		{
			.addr   = ICN85XX_PROG_IIC_ADDR,
			.flags  = 0,
			.len    = length + 2,
			.buf    = tmp_buf,
		},
	};

	if (length > 125) {
		icn85xx_error("%s too big datalen = %d!\n", __func__, length);
		return -EPERM;
	}

	tmp_buf[0] = (unsigned char)(addr>>8);
	tmp_buf[1] = (unsigned char)(addr);

	if (length != 0 && txdata != NULL)
		memcpy(&tmp_buf[2], txdata, length);

	while (retries < IIC_RETRY_NUM) {
		ret = i2c_transfer(this_client->adapter, msg, 1);
		if (ret == 1)
			break;
		retries++;
	}

	if (retries >= IIC_RETRY_NUM)
		icn85xx_error("%s i2c write error: %d\n", __func__, ret);

	return ret;
}




/***********************************************************************************************
Name    :   icn85xx_iic_test
Input   :   void
Output  :
function    : 0 success,
***********************************************************************************************/
static int icn85xx_iic_test(void)
{
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(this_client);
	int  ret = -1;
	char value = 0;
	char buf[3];
	int  retry = 0;
	int  flashid;

	icn85xx_ts->ictype = ICTYPE_UNKNOWN;
	while (retry++ < 3) {
		ret = icn85xx_read_reg(0xa, &value);
		icn85xx_error("<<<<sun11>>>> ret = %d \n", ret);
		if (ret > 0) {
			if (value == 0x85) {
				icn85xx_ts->ictype = ICN85XX_WITH_FLASH_85;
				setbootmode(ICN85XX_WITH_FLASH_85);
				return ret;
			} else if ((value == 0x86) || (value == 0x88)) {
				icn85xx_ts->ictype = ICN85XX_WITH_FLASH_86;
				setbootmode(ICN85XX_WITH_FLASH_86);
				return ret;
			} else if (value == 0x87) {
				icn85xx_ts->ictype = ICN85XX_WITH_FLASH_87;
				setbootmode(ICN85XX_WITH_FLASH_87);
				return ret;
			}
		}
		icn85xx_error("iic test error! retry = %d\n", retry);
		msleep(3);
	}
	icn85xx_goto_progmode();
	msleep(10);
	retry = 0;
	while (retry++ < 3) {
		buf[0] = 0;
		buf[1] = 0;
		buf[2] = 0;
		ret = icn85xx_prog_i2c_txdata(0x040000, buf, 3);
		if (ret < 0) {
			icn85xx_error("write reg failed! ret: %d\n", ret);
			return ret;
		}
		ret = icn85xx_prog_i2c_rxdata(0x040000, buf, 3);
		icn85xx_trace("icn85xx_check_progmod: %d, 0x%2x, 0x%2x, 0x%2x\n", ret, buf[0], buf[1], buf[2]);
		if (ret > 0) {
			if ((buf[2] == 0x85) && (buf[1] == 0x05)) {
				flashid = icn85xx_read_flashid();
				if ((MD25D40_ID1 == flashid) || (MD25D40_ID2 == flashid)
					|| (MD25D20_ID1 == flashid) || (MD25D20_ID2 == flashid)
					|| (GD25Q10_ID == flashid) || (MX25L512E_ID == flashid)
					|| (MD25D05_ID == flashid) || (MD25D10_ID == flashid)) {
					icn85xx_ts->ictype = ICN85XX_WITH_FLASH_85;
					setbootmode(ICN85XX_WITH_FLASH_85);
				} else {
					icn85xx_ts->ictype = ICN85XX_WITHOUT_FLASH_85;
					setbootmode(ICN85XX_WITHOUT_FLASH_85);
				}
				return ret;
			} else if (((buf[2] == 0x85) && (buf[1] == 0x0e)) || (buf[2] == 0x88)) {
				flashid = icn85xx_read_flashid();
				if ((MD25D40_ID1 == flashid) || (MD25D40_ID2 == flashid)
					|| (MD25D20_ID1 == flashid) || (MD25D20_ID1 == flashid)
					|| (GD25Q10_ID == flashid) || (MX25L512E_ID == flashid)
					|| (MD25D05_ID == flashid) || (MD25D10_ID == flashid)) {
					icn85xx_ts->ictype = ICN85XX_WITH_FLASH_86;
					setbootmode(ICN85XX_WITH_FLASH_86);
				} else {
					icn85xx_ts->ictype = ICN85XX_WITHOUT_FLASH_86;
					setbootmode(ICN85XX_WITHOUT_FLASH_86);
				}
				return ret;
			}
			else  //for ICNT87
      {                
          ret = icn87xx_prog_i2c_rxdata(0xf001, buf, 2);
          if(ret > 0)                    
          {
              if(buf[1] == 0x87)
              {                        
                  flashid = icn87xx_read_flashid();                        
                  printk("icnt87 flashid: 0x%x\n",flashid);
                  if(0x114051 == flashid)
                  {
                      icn85xx_ts->ictype = ICN85XX_WITH_FLASH_87;  
                      setbootmode(ICN85XX_WITH_FLASH_87);
                  }                        
                  else
                  {
                      icn85xx_ts->ictype = ICN85XX_WITHOUT_FLASH_87;  
                      setbootmode(ICN85XX_WITHOUT_FLASH_87);
                  }
                  return ret;
              }
					}
			}
		}
		icn85xx_error("iic2 test error! %d\n", retry);
		msleep(3);
	}
	icn85xx_error("<<<<sun12>>>>\n");

	return ret;
}
#if !CTP_REPORT_PROTOCOL

/***********************************************************************************************
Name    :   icn85xx_ts_release
Input   :   void
Output  :
function    : touch release
***********************************************************************************************/
static void icn85xx_ts_release(void)
{
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(this_client);
	icn85xx_info("==icn85xx_ts_release ==\n");
	//input_report_abs(icn85xx_ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	//input_report_abs(icn85xx_ts->input_dev, ABS_MT_PRESSURE, 0);

	input_report_abs(icn85xx_ts->input_dev, ABS_MT_TRACKING_ID, -1);//zhangbing@uniscope_drv 20161222 add for TP work abnomal at ffbm mode
	icn85xx_error("2017 release ABS_MT_TRACKING_ID: %d\n", ABS_MT_TRACKING_ID);
	input_report_key(icn85xx_ts->input_dev, BTN_TOUCH, 0);
	icn85xx_error("2017 release BTN_TOUCH: %d\n", BTN_TOUCH);
	
	input_sync(icn85xx_ts->input_dev);
}

/***********************************************************************************************
Name    :   icn85xx_report_value_A
Input   :   void
Output  :
function    : reprot touch ponit
***********************************************************************************************/
static void icn85xx_report_value_A(void)
{
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(this_client);
	char buf[POINT_NUM*POINT_SIZE+3] = {0};
	int ret = -1;
	int i;

#if TOUCH_VIRTUAL_KEYS
	unsigned char	key_value = 0;
	static unsigned char prev_key;
#endif
	icn85xx_info("==icn85xx_report_value_A ==\n");

	/*ret = icn85xx_i2c_rxdata(16, buf, POINT_NUM*POINT_SIZE+2);*/
	ret = icn85xx_i2c_rxdata(0x1000, buf, POINT_NUM*POINT_SIZE+2);
	if (ret < 0) {
		icn85xx_error("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ;
	}

#if TOUCH_VIRTUAL_KEYS
	key_value = buf[0];

	icn85xx_trace("key_value=%d\n", key_value);
	if (key_value & ICN_VIRTUAL_BUTTON_MENU) {
		input_report_key(icn85xx_ts->input_dev, KEY_MENU, 1);
		input_sync(icn85xx_ts->input_dev);
	} else if (prev_key & ICN_VIRTUAL_BUTTON_MENU) {
		input_report_key(icn85xx_ts->input_dev, KEY_MENU, 0);
		input_sync(icn85xx_ts->input_dev);
	}

	if (key_value & ICN_VIRTUAL_BUTTON_HOME) {
		input_report_key(icn85xx_ts->input_dev, KEY_HOMEPAGE, 1);
		input_sync(icn85xx_ts->input_dev);
	} else if (prev_key & ICN_VIRTUAL_BUTTON_HOME) {
		input_report_key(icn85xx_ts->input_dev, KEY_HOMEPAGE, 0);
		input_sync(icn85xx_ts->input_dev);
	}

	if (key_value & ICN_VIRTUAL_BUTTON_BACK) {
		input_report_key(icn85xx_ts->input_dev, KEY_BACK, 1);
		input_sync(icn85xx_ts->input_dev);
	} else if (prev_key & ICN_VIRTUAL_BUTTON_BACK) {
		input_report_key(icn85xx_ts->input_dev, KEY_BACK, 0);
		input_sync(icn85xx_ts->input_dev);
	}

	prev_key = key_value;
#endif

	icn85xx_ts->point_num = buf[1];
	if (icn85xx_ts->point_num == 0) {
		icn85xx_ts_release();
		return ;
	}
	for (i = 0; i < icn85xx_ts->point_num; i++) {
		if (buf[8 + POINT_SIZE*i]  != 4)
			break ;
	}

	if (i == icn85xx_ts->point_num) {
		icn85xx_ts_release();
		return ;
	}

	for (i = 0; i < icn85xx_ts->point_num; i++) {
		icn85xx_ts->point_info[i].u8ID = buf[2 + POINT_SIZE*i];
		icn85xx_ts->point_info[i].u16PosX = (buf[4 + POINT_SIZE*i]<<8) + buf[3 + POINT_SIZE*i];
		icn85xx_ts->point_info[i].u16PosY = (buf[6 + POINT_SIZE*i]<<8) + buf[5 + POINT_SIZE*i];
		icn85xx_ts->point_info[i].u8Pressure = 200;/*buf[7 + POINT_SIZE*i];*/
		icn85xx_ts->point_info[i].u8EventId = buf[8 + POINT_SIZE*i];

		if (1 == icn85xx_ts->revert_x_flag) {
			icn85xx_ts->point_info[i].u16PosX = icn85xx_ts->screen_max_x - icn85xx_ts->point_info[i].u16PosX;
		}
		if (1 == icn85xx_ts->revert_y_flag) {
			icn85xx_ts->point_info[i].u16PosY = icn85xx_ts->screen_max_y - icn85xx_ts->point_info[i].u16PosY;
		}

		icn85xx_info("u8ID %d\n", icn85xx_ts->point_info[i].u8ID);
		icn85xx_info("u16PosX %d\n", icn85xx_ts->point_info[i].u16PosX);
		icn85xx_info("u16PosY %d\n", icn85xx_ts->point_info[i].u16PosY);
		icn85xx_info("u8Pressure %d\n", icn85xx_ts->point_info[i].u8Pressure);
		icn85xx_info("u8EventId %d\n", icn85xx_ts->point_info[i].u8EventId);

		input_report_key(icn85xx_ts->input_dev, BTN_TOUCH, 1);
		icn85xx_error("2017 BTN_TOUCH: %d\n", BTN_TOUCH);
		input_report_abs(icn85xx_ts->input_dev, ABS_MT_POSITION_X, icn85xx_ts->point_info[i].u16PosX);
		icn85xx_error("2017 point_info[i].u16PosX: %d\n", icn85xx_ts->point_info[i].u16PosX);
		input_report_abs(icn85xx_ts->input_dev, ABS_MT_POSITION_Y, icn85xx_ts->point_info[i].u16PosY);
		icn85xx_error("2017 point_info[i].u16PosY: %d\n", icn85xx_ts->point_info[i].u16PosY);
		//input_report_abs(icn85xx_ts->input_dev, ABS_MT_TOUCH_MAJOR, 255);
		//input_report_abs(icn85xx_ts->input_dev, ABS_MT_WIDTH_MAJOR, 2);
		input_report_abs(icn85xx_ts->input_dev, ABS_MT_TOUCH_MAJOR, 0x16);
		input_report_abs(icn85xx_ts->input_dev, ABS_MT_WIDTH_MAJOR, 0x16);
		input_report_abs(icn85xx_ts->input_dev, ABS_MT_TRACKING_ID, icn85xx_ts->point_info[i].u8ID);
		icn85xx_error("2017 point_info[i].u8ID: %d\n", icn85xx_ts->point_info[i].u8ID);
		input_mt_sync(icn85xx_ts->input_dev);

		//input_report_abs(icn85xx_ts->input_dev, ABS_MT_TRACKING_ID, icn85xx_ts->point_info[i].u8ID);
		//input_report_abs(icn85xx_ts->input_dev, ABS_MT_WIDTH_MAJOR, 2);
		//input_report_abs(icn85xx_ts->input_dev, ABS_MT_TOUCH_MAJOR, 255);
		//input_report_abs(icn85xx_ts->input_dev, ABS_MT_POSITION_X, icn85xx_ts->point_info[i].u16PosY);
		//input_report_abs(icn85xx_ts->input_dev, ABS_MT_POSITION_Y, SCREEN_MAX_Y - icn85xx_ts->point_info[i].u16PosX);
		//input_report_abs(icn85xx_ts->input_dev, ABS_MT_PRESSURE, icn85xx_ts->point_info[i].u8Pressure);
		//input_mt_sync(icn85xx_ts->input_dev);
		icn85xx_point_info("point:id = %d %d ===x = %d,y = %d, press = %d ====\n",
			i, icn85xx_ts->point_info[i].u8ID, icn85xx_ts->point_info[i].u16PosX,
			icn85xx_ts->point_info[i].u16PosY, icn85xx_ts->point_info[i].u8Pressure);
	}

	input_sync(icn85xx_ts->input_dev);
}
#endif/*!CTP_REPORT_PROTOCOL*/
/***********************************************************************************************
Name    :   icn85xx_report_value_B
Input   :   void
Output  :
function    : reprot touch ponit
***********************************************************************************************/
#if CTP_REPORT_PROTOCOL
static void icn85xx_report_value_B(void)
{
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(this_client);
	char buf[POINT_NUM * POINT_SIZE + 3] = {0};
	static unsigned char finger_last[POINT_NUM + 1] = {0};
	unsigned char  finger_current[POINT_NUM + 1] = {0};
	unsigned int position = 0;
	int temp = 0;
	int ret = -1;

#if TOUCH_VIRTUAL_KEYS
	unsigned char	key_value = 0;
	static unsigned char prev_key;
#endif/*TOUCH_VIRTUAL_KEYS*/

	ret = icn85xx_i2c_rxdata(0x1000, buf, POINT_NUM * POINT_SIZE + 2);
	if (ret < 0) {
		icn85xx_error("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ;
	}

#if TOUCH_VIRTUAL_KEYS
	key_value = buf[0];
	if (virtual_keys_debug) {
			if (key_value & ICN_VIRTUAL_BUTTON_MENU) {
				input_report_key(icn85xx_ts->input_dev, KEY_MENU, 1);
				input_sync(icn85xx_ts->input_dev);
			} else if (prev_key & ICN_VIRTUAL_BUTTON_MENU) {
				input_report_key(icn85xx_ts->input_dev, KEY_MENU, 0);
				input_sync(icn85xx_ts->input_dev);
			}

		if (key_value & ICN_VIRTUAL_BUTTON_HOME) {
			input_report_key(icn85xx_ts->input_dev, KEY_HOMEPAGE, 1);
			input_sync(icn85xx_ts->input_dev);
		} else if (prev_key & ICN_VIRTUAL_BUTTON_HOME) {
			input_report_key(icn85xx_ts->input_dev, KEY_HOMEPAGE, 0);
			input_sync(icn85xx_ts->input_dev);
		}

		if (key_value & ICN_VIRTUAL_BUTTON_BACK) {
			input_report_key(icn85xx_ts->input_dev, KEY_BACK, 1);
			input_sync(icn85xx_ts->input_dev);
		} else if (prev_key & ICN_VIRTUAL_BUTTON_BACK) {
			input_report_key(icn85xx_ts->input_dev, KEY_BACK, 0);
			input_sync(icn85xx_ts->input_dev);
		}
	}
	prev_key = key_value;
#endif/*TOUCH_VIRTUAL_KEYS*/
  //printk("ICN x_flag=%d,x_flag=%d,max_x=%d,max_y=%d\n",icn85xx_ts->revert_x_flag,icn85xx_ts->revert_y_flag,icn85xx_ts->screen_max_x,icn85xx_ts->screen_max_y);
	icn85xx_ts->point_num = buf[1];
	if (icn85xx_ts->point_num > 0) {
		if (icn85xx_ts->point_num > 5)
			return;

		for (position = 0; position < icn85xx_ts->point_num; position++) {
			temp = buf[2 + POINT_SIZE*position] + 1;
			finger_current[temp] = 1;
			icn85xx_ts->point_info[temp].u8ID = buf[2 + POINT_SIZE*position];
			icn85xx_ts->point_info[temp].u16PosX = (buf[4 + POINT_SIZE*position]<<8) + buf[3 + POINT_SIZE*position];
			icn85xx_ts->point_info[temp].u16PosY = (buf[6 + POINT_SIZE*position]<<8) + buf[5 + POINT_SIZE*position];
			icn85xx_ts->point_info[temp].u8Pressure = buf[7 + POINT_SIZE*position];
			icn85xx_ts->point_info[temp].u8EventId = buf[8 + POINT_SIZE*position];
			printk("ICN befor Touch[%d]: event= %d, x= %d, y= %d, press= %d\n",
				icn85xx_ts->point_info[temp].u8ID,
				icn85xx_ts->point_info[temp].u8EventId,
				icn85xx_ts->point_info[temp].u16PosX,
				icn85xx_ts->point_info[temp].u16PosY,
				icn85xx_ts->point_info[temp].u8Pressure);
			if (icn85xx_ts->point_info[temp].u8EventId == 4)
				finger_current[temp] = 0;

			if (1 == icn85xx_ts->revert_x_flag)
				icn85xx_ts->point_info[temp].u16PosX = icn85xx_ts->screen_max_x
					- icn85xx_ts->point_info[temp].u16PosX;
			if (1 == icn85xx_ts->revert_y_flag)
				icn85xx_ts->point_info[temp].u16PosY = icn85xx_ts->screen_max_y
					- icn85xx_ts->point_info[temp].u16PosY;
			printk("ICN after Touch[%d]: event= %d, x= %d, y= %d, press= %d\n",
				icn85xx_ts->point_info[temp].u8ID,
				icn85xx_ts->point_info[temp].u8EventId,
				icn85xx_ts->point_info[temp].u16PosX,
				icn85xx_ts->point_info[temp].u16PosY,
				icn85xx_ts->point_info[temp].u8Pressure);
		}
	} else {
		for (position = 1; position < POINT_NUM + 1; position++)
			finger_current[position] = 0;
		//icn85xx_info("no touch\n");
	}

	for (position = 1; position < POINT_NUM + 1; position++) {
		if ((finger_current[position] == 0) && (finger_last[position] != 0)) {
			input_mt_slot(icn85xx_ts->input_dev, position-1);
			input_mt_report_slot_state(icn85xx_ts->input_dev, MT_TOOL_FINGER, false);
			icn85xx_point_info("one touch up: %d\n", position);
		} else if (finger_current[position]) {
			input_mt_slot(icn85xx_ts->input_dev, position-1);
			input_mt_report_slot_state(icn85xx_ts->input_dev, MT_TOOL_FINGER, true);
			//input_report_abs(icn85xx_ts->input_dev, ABS_MT_TOUCH_MAJOR, 1);
			//input_report_abs(icn85xx_ts->input_dev, ABS_MT_PRESSURE, icn85xx_ts->point_info[position].u8Pressure);
			input_report_abs(icn85xx_ts->input_dev, ABS_MT_POSITION_X, icn85xx_ts->point_info[position].u16PosX);
			input_report_abs(icn85xx_ts->input_dev, ABS_MT_POSITION_Y, icn85xx_ts->point_info[position].u16PosY);
		}
	}
	input_sync(icn85xx_ts->input_dev);

	for (position = 1; position < POINT_NUM + 1; position++)
		finger_last[position] = finger_current[position];
}
#endif/*CTP_REPORT_PROTOCOL*/

/***********************************************************************************************
Name    :   icn85xx_ts_pen_irq_work
Input   :   void
Output  :
function    : work_struct
***********************************************************************************************/
static void icn85xx_ts_pen_irq_work(struct work_struct *work)
{
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(this_client);

#if SUPPORT_PROC_FS
	if (down_interruptible(&icn85xx_ts->sem))
		return;
#endif/*SUPPORT_PROC_FS*/

	if (icn85xx_ts->work_mode == 0) {
#if CTP_REPORT_PROTOCOL
		icn85xx_report_value_B();
#else/*CTP_REPORT_PROTOCOL*/
		icn85xx_report_value_A();
#endif/*CTP_REPORT_PROTOCOL*/
		icn85xx_irq_enable();
	}
#if SUPPORT_PROC_FS
	else if (icn85xx_ts->work_mode == 1) {
		icn85xx_trace("log raw data\n");
		icn85xx_log(0);/* raw data */
	} else if (icn85xx_ts->work_mode == 2) {
		icn85xx_trace("log diff data\n");
		icn85xx_log(1);/* diff data */
	} else if (icn85xx_ts->work_mode == 3) {
		icn85xx_trace("raw2diff data\n");
		icn85xx_log(2);/* diff data */
	} else if (icn85xx_ts->work_mode == 4)/* idle */
		;
	else if (icn85xx_ts->work_mode == 5) {/* write para, reinit */
		icn85xx_trace("reinit tp\n");
		icn85xx_write_reg(0, 1);
		mdelay(100);
		icn85xx_write_reg(0, 0);
		icn85xx_ts->work_mode = 0;
	}
#endif/*SUPPORT_PROC_FS*/
#ifdef CONFIG_TOUCHSCREEN_ICN85XX_GESTURE
	else if ((icn85xx_ts->work_mode == 6) || (icn85xx_ts->work_mode == 7)) {/* gesture test mode */
		char buf[sizeof(structGestureData)] = {0};
		int ret = -1;

		if (!icn85xx_ts->gesture_switch)
			return ;
		ret = icn85xx_i2c_rxdata(0x7000, buf, 2);
		if (ret < 0) {
			icn85xx_error("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
			up(&icn85xx_ts->sem);
			return ;
		}
		icn85xx_gesture_handler(buf[0]);
		/* icn85xx_ts->work_mode = 0; */
#if 0
		structGestureData.u8Status = 1;
		structGestureData.u8Gesture = buf[0];
		structGestureData.u8GestureNum = buf[1];
		icn85xx_trace("structGestureData.u8Gesture: 0x%x\n", structGestureData.u8Gesture);
		icn85xx_trace("structGestureData.u8GestureNum: %d\n", structGestureData.u8GestureNum);

		ret = icn85xx_i2c_rxdata(0x7002, buf, structGestureData.u8GestureNum*6);
		if (ret < 0) {
			icn85xx_error("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
			return ;
		}

		for (i = 0; i < structGestureData.u8GestureNum; i++) {
			structGestureData.point_info[i].u16PosX = (buf[1 + 6*i]<<8) + buf[0 + 6*i];
			structGestureData.point_info[i].u16PosY = (buf[3 + 6*i]<<8) + buf[2 + 6*i];
			structGestureData.point_info[i].u8EventId = buf[5 + 6*i];
			icn85xx_trace("(%d, %d, %d)", structGestureData.point_info[i].u16PosX, structGestureData.point_info[i].u16PosY, structGestureData.point_info[i].u8EventId);
		}
		icn85xx_trace("\n");
#endif/*0*/
		if (icn85xx_ts->use_irq)
			icn85xx_irq_enable();
    }
#endif/*CONFIG_TOUCHSCREEN_ICN85XX_GESTURE*/

#if SUPPORT_PROC_FS
	up(&icn85xx_ts->sem);
#endif/*SUPPORT_PROC_FS*/
}
/***********************************************************************************************
Name    :   chipone_timer_func
Input   :   void
Output  :
function    : Timer interrupt service routine.
***********************************************************************************************/
static enum hrtimer_restart chipone_timer_func(struct hrtimer *timer)
{
	struct icn85xx_ts_data *icn85xx_ts = container_of(timer, struct icn85xx_ts_data, timer);
	queue_work(icn85xx_ts->ts_workqueue, &icn85xx_ts->pen_event_work);

	if (icn85xx_ts->use_irq == 1) {
		if ((icn85xx_ts->work_mode == 1) || (icn85xx_ts->work_mode == 2))
			hrtimer_start(&icn85xx_ts->timer, ktime_set(CTP_POLL_TIMER/1000,
				(CTP_POLL_TIMER%1000)*1000000), HRTIMER_MODE_REL);
	} else {
		hrtimer_start(&icn85xx_ts->timer, ktime_set(CTP_POLL_TIMER/1000,
			(CTP_POLL_TIMER%1000)*1000000), HRTIMER_MODE_REL);
	}
	return HRTIMER_NORESTART;
}
/***********************************************************************************************
Name    :   icn85xx_ts_interrupt
Input   :   void
Output  :
function    : interrupt service routine
***********************************************************************************************/
static irqreturn_t icn85xx_ts_interrupt(int irq, void *dev_id)
{
	struct icn85xx_ts_data *icn85xx_ts = dev_id;

	icn85xx_trace("icn85xx_ts TS Interrupt*********************mode<%d>******************************\n",icn85xx_ts->work_mode);
	if (icn85xx_ts->work_mode != 0 && icn85xx_ts->work_mode != 6)
		return IRQ_HANDLED;

	icn85xx_irq_disable();
	if (!work_pending(&icn85xx_ts->pen_event_work))
		queue_work(icn85xx_ts->ts_workqueue, &icn85xx_ts->pen_event_work);

	return IRQ_HANDLED;
}

#ifndef CONFIG_HAS_EARLYSUSPEND

#ifdef CONFIG_TOUCHSCREEN_ICN85XX_GESTURE

static void icn85xx_gesture_handler(u8 gesture_id)
{
	int i = 0;
	struct icn85xx_ts_data *ts = i2c_get_clientdata(this_client);
	icn85xx_error("%s: gesture_id = 0x%02X\n.", __func__, gesture_id);

	for (i = 0; i < ts->gesture_key->nbuttons; i++)
		if (gesture_id == ts->gesture_key->map[i])
			break;
	if (i >= ts->gesture_key->nbuttons) {
		icn85xx_error("%s: have no gesture to report.", __func__);
	} else {
		if ((ts->gesture_state) >> (i + 1) & 1) {
#if 0
			input_report_key(ts->input_dev, ts->gesture_map->map[i], 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, ts->gesture_map->map[i], 0);
			input_sync(ts->input_dev);
#else
			gTGesture = ts->gesture_map->map[i];
			input_report_key(icn_key_dev,KEY_ICN_SENSOR, 1);
			input_sync(icn_key_dev);
			input_report_key(icn_key_dev,KEY_ICN_SENSOR, 0);
			input_sync(icn_key_dev);
#endif
			icn85xx_info("%s: report gesture = %d \n", __func__, ts->gesture_map->map[i]);
		} else {
			icn85xx_error("%s: not open the gesture switch.", __func__);
		}
	}
}
#endif/*CONFIG_TOUCHSCREEN_ICN85XX_GESTURE*/

static void icn_release_all_finger(void)
{
#if CTP_REPORT_PROTOCOL
	int i;
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(this_client);
	for (i = 0; i < POINT_NUM; i++) {
		input_mt_slot(icn85xx_ts->input_dev, i);
		input_mt_report_slot_state(icn85xx_ts->input_dev, MT_TOOL_FINGER, false);
	}
	input_sync(icn85xx_ts->input_dev);
#else/*CTP_REPORT_PROTOCOL*/
	icn85xx_ts_release();
#endif/*CTP_REPORT_PROTOCOL*/
	icn85xx_trace("%s \n", __func__);
}

/***********************************************************************************************
Name    :   icn85xx_ts_suspend
Input   :   void
Output  :
function    : tp enter sleep mode
***********************************************************************************************/
static void icn85xx_ts_suspend(struct device *dev)
{
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(this_client);
	icn85xx_trace("%s \n", __func__);

	if (icn85xx_ts->icn_is_suspend) {
		icn85xx_trace("Icn85xx has already in suspend state,quit\n");
		return ;
	}
	if (icn85xx_ts->icn_is_update) {
		icn85xx_trace("Icn85xx is in update state,quit.\n");
		return ;
	}
	if (!icn85xx_ts->use_irq)
		hrtimer_cancel(&icn85xx_ts->timer);

#if SUPPORT_CHECK_ESD
	spin_lock(&icn85xx_ts->esd_lock);
	if (icn85xx_ts->esd_running) {
		icn85xx_ts->esd_running = 0;
		spin_unlock(&icn85xx_ts->esd_lock);
		icn85xx_trace("Esd cancelled");
		cancel_delayed_work_sync(&icn85xx_ts->icn85xx_esd_event_work);
	} else {
		spin_unlock(&icn85xx_ts->esd_lock);
	}
#endif/*SUPPORT_CHECK_ESD*/

#ifdef CONFIG_TOUCHSCREEN_ICN85XX_GESTURE
	icn85xx_ts->gesture_switch_pre = icn85xx_ts->gesture_switch;
	if (icn85xx_ts->gesture_switch_pre) {
		icn85xx_write_reg(ICN85XX_REG_PMODE, 0x40);
		icn85xx_ts->work_mode = 6;
	} else
#endif
	{
		icn85xx_irq_disable();
		icn85xx_write_reg(ICN85XX_REG_PMODE, PMODE_HIBERNATE);
	}

	icn_release_all_finger();

	icn85xx_ts->icn_is_suspend = 1;
	return ;
}

/***********************************************************************************************
Name    :   icn85xx_ts_resume
Input   :   void
Output  :
function    : wakeup tp or reset tp
***********************************************************************************************/
static void icn85xx_ts_resume(struct device *dev)
{
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(this_client);

#if SUPPORT_FW_UPDATE
	int ret = -1;
	int retry = 5;
	int need_update_fw = false;
	unsigned char value;
//	int icnt87_sram_crc = 0;
#endif/*SUPPORT_FW_UPDATE*/
	icn85xx_trace("%s begin.\n", __func__);
	if (!icn85xx_ts->icn_is_suspend) {
		icn85xx_trace("Icn85xx has already in resume state");
		return ;
	}

//	icn_release_all_finger();
#if SUPPORT_FW_UPDATE
	icn85xx_trace("icn85xx_ts_resume SUPPORT_FW_UPDATE\n");
	if (icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_85) {
			icn85xx_trace("icn85xx_ts_resume ICN85XX_WITHOUT_FLASH_85\n");
		while (retry-- && !need_update_fw) {
			icn85xx_ts_reset();
			icn85xx_bootfrom_sram();
			msleep(50);
			ret = icn85xx_read_reg(0xa, &value);
			if (ret > 0) {
				need_update_fw = false;
				break;
			}
		}
	} else if(icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_87)
    {
    	icn85xx_trace("icn85xx_ts_resume ICN85XX_WITHOUT_FLASH_87\n");
        while (retry-- && !need_update_fw) 
        {
            //icn85xx_ts_reset();
            icn85xx_set_prog_addr();
	     	//icn85xx_write_reg(ICN85xx_REG_PMODE, 0xff);

			// junfuzhang 20160913, check crc in sram before boot
            /*if(icnt87_sram_crc != icn87xx_calculate_crc(icnt87_sram_length)) // sram is broken
            {
    			icn85xx_trace("icn85xx_ts_resume,crc error!!!!!!!\n"); 
    			need_update_fw = true;
                break; 
			}*/
			// junfuzhang 20160913, adding end 
            icn87xx_boot_sram();
            msleep(80);
            ret = icn85xx_read_reg(0xa, &value);
            if (ret > 0) 
            { 
                need_update_fw = false;
                icn85xx_trace("icn85xx_ts_resume icn87xx_boot_sram ok\n");
                break; 
            }
        }
        /*if (retry <= 0) need_update_fw = true;

        if (need_update_fw) 
        {
        		icn85xx_trace("icn85xx_ts_resume need_update_fw\n");
            if(R_OK == icn87xx_fw_update(firmware))
            {
                icn85xx_ts->code_loaded_flag = 1;
                icn85xx_trace("ICN87XX_WITHOUT_FLASH, reload code ok\n"); 
            }
            else
            {
                icn85xx_ts->code_loaded_flag = 0;
                icn85xx_trace("ICN87XX_WITHOUT_FLASH, reload code error\n"); 
            }
        }  */         
    }
 	if (retry <= 0) need_update_fw = true;
	if (need_update_fw) {
		if(icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_87)
		{
					icn85xx_trace("icn85xx_ts_resume need_update_fw\n");
			if (R_OK == icn87xx_fw_update(icn85xx_ts->fw_name)) {
				icn85xx_ts->code_loaded_flag = 1;
				icn85xx_trace("ICN87XX_WITHOUT_FLASH_85, reload code ok\n");
			} else {
				icn85xx_ts->code_loaded_flag = 0;
				icn85xx_trace("ICN87XX_WITHOUT_FLASH_85, reload code error\n");
			}				
		}
		else
		{
				if (R_OK == icn85xx_fw_update(icn85xx_ts->fw_name)) {
					icn85xx_ts->code_loaded_flag = 1;
					icn85xx_trace("ICN85XX_WITHOUT_FLASH_85, reload code ok\n");
				} else {
					icn85xx_ts->code_loaded_flag = 0;
					icn85xx_trace("ICN85XX_WITHOUT_FLASH_85, reload code error\n");
				}
		}
	} else {
#ifdef CONFIG_TOUCHSCREEN_ICN85XX_GESTURE
		if (icn85xx_ts->gesture_switch_pre) {
			icn85xx_ts->work_mode = 0;
			icn85xx_ts_reset();
		} else
#endif/*CONFIG_TOUCHSCREEN_ICN85XX_GESTURE*/
		{
			if(icn85xx_ts->ictype != ICN85XX_WITHOUT_FLASH_87)
			{
				icn85xx_trace("ICN85XX_WITH_FLASH_85, reset\n");
				icn85xx_ts_reset();
				
			}
			
		}
	}
#else/*SUPPORT_FW_UPDATE*/
	icn85xx_ts_reset();
	icn85xx_irq_enable();
#endif/*SUPPORT_FW_UPDATE*/

	if (!icn85xx_ts->use_irq) {
		icn85xx_trace("icn85xx_ts_resume  hrtimer_start\n");
		hrtimer_start(&icn85xx_ts->timer, ktime_set(CTP_START_TIMER/1000, (CTP_START_TIMER%1000)*1000000), HRTIMER_MODE_REL);
	}
	else
		icn85xx_irq_enable();

#if SUPPORT_CHECK_ESD
	spin_lock(&icn85xx_ts->esd_lock);
	if (!icn85xx_ts->esd_running) {
		icn85xx_ts->esd_running = 1;
		spin_unlock(&icn85xx_ts->esd_lock);
		icn85xx_trace("Esd started");
		queue_delayed_work(icn85xx_ts->icn85xx_esd_workqueue,
			&icn85xx_ts->icn85xx_esd_event_work, icn85xx_ts->clk_tick_cnt);
	}
	else
		spin_lock(&icn85xx_ts->esd_lock);
#endif/*SUPPORT_CHECK_ESD*/

#if defined(CONFIG_TOUCHSCREEN_ICN85XX_FH)
	if(charger_detect_enable) {
		icn85xx_charger_plugin(icn85xx_ts, 
			icn85xx_ts->icn_is_charger_present);
	}
#endif /* CONFIG_TOUCHSCREEN_ICN85XX_FH */

	icn85xx_ts->icn_is_suspend = 0;

	return ;
}
#endif/*CONFIG_HAS_EARLYSUSPEND*/

#ifdef CONFIG_TOUCHSCREEN_ICN85XX_GESTURE
static int icn85xx_ts_gesture_suspend(struct device *dev)
{
	struct icn85xx_ts_data *icn85xx_ts = dev_get_drvdata(dev);

	if (!icn85xx_ts->use_irq)
		return 0;

	if (icn85xx_ts->gesture_switch_pre) {
		icn85xx_irq_disable();
		enable_irq_wake(icn85xx_ts->client->irq);
	}

	return 0;
}
static int icn85xx_ts_gesture_resume(struct device *dev)
{
	struct icn85xx_ts_data *icn85xx_ts = dev_get_drvdata(dev);

	if (!icn85xx_ts->use_irq)
		return 0;

	if (icn85xx_ts->gesture_switch_pre) {
		disable_irq_wake(icn85xx_ts->client->irq);
		icn85xx_irq_enable();
	}

	return 0;
}

static const struct dev_pm_ops icn85xx_ts_dev_pm_ops = {
	.suspend = icn85xx_ts_gesture_suspend,
	.resume  = icn85xx_ts_gesture_resume,
};
#endif/*CONFIG_TOUCHSCREEN_ICN85XX_GESTURE*/

/***********************************************************************************************
Name    :   icn85xx_request_io_port
Input   :   void
Output  :
function    : 0 success,
***********************************************************************************************/
static int icn85xx_request_io_port(struct i2c_client *client, struct icn85xx_ts_data *icn85xx_ts)
{
	int err = 0;

	struct device *dev = &(client->dev);
	struct device_node *np = dev->of_node;

	icn85xx_ts->rst_gpio = of_get_named_gpio_flags(np, "chipone,reset-gpio",
		0, &icn85xx_ts->reset_gpio_flags);

	icn85xx_ts->irq_gpio = of_get_named_gpio_flags(np, "chipone,irq-gpio",
		0, &icn85xx_ts->irq_gpio_flags);

	err = gpio_request(icn85xx_ts->irq_gpio, "TS_INT");
	if (err < 0) {
		icn85xx_error("Failed to request GPIO:%d, ERRNO:%d\n", (int)icn85xx_ts->irq, err);
		return err;
	}


	err = gpio_request(icn85xx_ts->rst_gpio, "TS_rest");
	if (err < 0) {
		icn85xx_error("Failed to request rest GPIO:%d, ERRNO:%d\n", (int)icn85xx_ts->rst, err);
		return err;
	}
	gpio_direction_output(icn85xx_ts->rst_gpio, 0);
	msleep(50);
	gpio_set_value_cansleep(icn85xx_ts->rst_gpio, 1);
	msleep(50);

	return err;
}

/***********************************************************************************************
Name    :   icn85xx_free_io_port
Input   :   void
Output  :
function    : 0 success,
***********************************************************************************************/
static int icn85xx_free_io_port(struct icn85xx_ts_data *icn85xx_ts)
{
	if (gpio_is_valid(icn85xx_ts->irq_gpio))
		gpio_free(icn85xx_ts->irq_gpio);
	if (gpio_is_valid(icn85xx_ts->rst_gpio))
		gpio_free(icn85xx_ts->rst_gpio);

	return 0;
}

/***********************************************************************************************
Name    :   icn85xx_request_irq
Input   :   void
Output  :
function    : 0 success,
***********************************************************************************************/
static int icn85xx_request_irq(struct icn85xx_ts_data *icn85xx_ts)
{
	int err = -1;

	
	//return err;

	/*err = gpio_request(icn85xx_ts->irq_gpio, "TS_INT");
	if (err < 0) {
		icn85xx_error("Failed to request GPIO:%d, ERRNO:%d\n", (int)icn85xx_ts->irq, err);
		return err;
	}*/
	gpio_direction_input(icn85xx_ts->irq_gpio);

	icn85xx_ts->irq  = gpio_to_irq(icn85xx_ts->irq_gpio);
	err = request_threaded_irq(icn85xx_ts->irq, NULL,
	icn85xx_ts_interrupt, IRQ_TYPE_EDGE_FALLING | IRQF_ONESHOT,
		"icn85xx_ts", icn85xx_ts);

	if (err < 0)  {
		icn85xx_error("icn85xx_ts_probe: request irq failed\n");
		return err;
	} else {
		icn85xx_irq_disable();
		icn85xx_ts->use_irq = 1;
	}

	return 0;
}


/***********************************************************************************************
Name    :   icn85xx_free_irq
Input   :   void
Output  :
function    : 0 success,
***********************************************************************************************/
static int icn85xx_free_irq(struct icn85xx_ts_data *icn85xx_ts)
{
	if (icn85xx_ts) {
		if (icn85xx_ts->use_irq)
			free_irq(icn85xx_ts->irq, icn85xx_ts);
		else
			hrtimer_cancel(&icn85xx_ts->timer);
	}

	return 0;
}

/***********************************************************************************************
Name    :   icn85xx_request_input_dev
Input   :   void
Output  :
function    : 0 success,
***********************************************************************************************/
static int icn85xx_request_input_dev(struct icn85xx_ts_data *icn85xx_ts)
{
	int ret = -1;
	struct input_dev *input_dev;
#if TOUCH_VIRTUAL_KEYS
//#ifdef CONFIG_TOUCHSCREEN_ICN85XX_GESTURE
	int index = 0;
#endif
	input_dev = input_allocate_device();
	if (!input_dev) {
		icn85xx_error("failed to allocate input device\n");
		return -ENOMEM;
	}

	icn85xx_ts->input_dev = input_dev;
	icn85xx_ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
#if CTP_REPORT_PROTOCOL
	__set_bit(INPUT_PROP_DIRECT, icn85xx_ts->input_dev->propbit);
	input_mt_init_slots(icn85xx_ts->input_dev, 5, 0);
#else/*CTP_REPORT_PROTOCOL*/
	//set_bit(ABS_MT_TOUCH_MAJOR, icn85xx_ts->input_dev->absbit);
	//set_bit(ABS_MT_POSITION_X, icn85xx_ts->input_dev->absbit);
	//set_bit(ABS_MT_POSITION_Y, icn85xx_ts->input_dev->absbit);
	//set_bit(ABS_MT_WIDTH_MAJOR, icn85xx_ts->input_dev->absbit);
	//set_bit(ABS_MT_PRESSURE, icn85xx_ts->input_dev->absbit);
	//set_bit(INPUT_PROP_DIRECT, icn85xx_ts->input_dev->propbit);
	icn85xx_ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif/*CTP_REPORT_PROTOCOL*/
	__set_bit(BTN_TOUCH,  icn85xx_ts->input_dev->keybit);

	__set_bit(INPUT_PROP_DIRECT, icn85xx_ts->input_dev->propbit);

	input_set_abs_params(icn85xx_ts->input_dev, ABS_MT_POSITION_X, 0, 640, 0, 0);
	input_set_abs_params(icn85xx_ts->input_dev, ABS_MT_POSITION_Y, 0, 1280, 0, 0);
	input_set_abs_params(icn85xx_ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(icn85xx_ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(icn85xx_ts->input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);
	//input_set_abs_params(icn85xx_ts->input_dev, ABS_MT_PRESSURE, 0, 10, 0, 0);

#if TOUCH_VIRTUAL_KEYS
	for (index = 0; index < icn85xx_ts->button_map->nbuttons; index++)
		input_set_capability(icn85xx_ts->input_dev,
			EV_KEY, icn85xx_ts->button_map->map[index]);
#endif/*TOUCH_VIRTUAL_KEYS*/

#ifdef CONFIG_TOUCHSCREEN_ICN85XX_GESTURE
#if 0
	for (index = 0; index < icn85xx_ts->gesture_map->nbuttons; index++)
		input_set_capability(icn85xx_ts->input_dev,
			EV_KEY, icn85xx_ts->gesture_map->map[index]);
#else
//enable gesture soft switch
	icn85xx_ts->gesture_state = 0xffff;
	icn85xx_ts->gesture_switch = true;

	icn_key_dev = input_allocate_device();
	if (!input_dev) {
		icn85xx_error("failed to allocate input device\n");
		return -ENOMEM;
	}
	__set_bit(EV_KEY,  icn_key_dev->evbit);
	__set_bit(KEY_ICN_SENSOR,  icn_key_dev->keybit);
	__set_bit(KEY_POWER,  icn_key_dev->keybit);
	icn_key_dev->id.bustype = BUS_HOST;
	icn_key_dev->name = "TP_ICN_GESTURE";
	if(input_register_device(icn_key_dev)) {
		icn85xx_error("icn_key_dev register : fail!\n");
	} else {
		icn85xx_error("icn_key_dev register : success!!\n");
	}
#endif
#endif/*CONFIG_TOUCHSCREEN_ICN85XX_GESTURE*/

	input_dev->name = "chipone-ts";
	ret = input_register_device(input_dev);
	if (ret) {
		icn85xx_error("Register %s input device failed\n", input_dev->name);
		input_free_device(input_dev);
		return -ENODEV;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	icn85xx_trace("==register_early_suspend =\n");
	icn85xx_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	icn85xx_ts->early_suspend.suspend = icn85xx_ts_suspend;
	icn85xx_ts->early_suspend.resume  = icn85xx_ts_resume;
	register_early_suspend(&icn85xx_ts->early_suspend);
#endif/*CONFIG_HAS_EARLYSUSPEND*/

	return 0;
}

#if SUPPORT_DELAYED_WORK
static void icn_delayedwork_fun(struct work_struct *icn_delayed_work)
{
#if SUPPORT_FW_UPDATE
	int retry;
#endif/*SUPPORT_FW_UPDATE*/
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(this_client);

#if SUPPORT_FW_UPDATE
	short fwVersion = 0;
	short curVersion = 0;
#endif/*SUPPORT_FW_UPDATE*/
	icn85xx_trace("%s begin.\n", __func__);

#if SUPPORT_FW_UPDATE
	fwVersion = icn85xx_read_fw_Ver(icn85xx_ts->fw_name);
	curVersion = icn85xx_readVersion();
	icn85xx_trace("fwVersion : 0x%x\n", fwVersion);
	icn85xx_trace("current version: 0x%x\n", curVersion);

#if FORCE_UPDATE_FW
	retry = 5;
	while (retry > 0) {
		if (R_OK == icn85xx_fw_update(icn85xx_ts->fw_name))
			break;
		retry--;
		icn85xx_error("icn85xx_fw_update failed.\n");
	}
#else/*FORCE_UPDATE_FW*/
	if (fwVersion > curVersion) {
		retry = 5;
		while (retry > 0) {
			if (R_OK == icn85xx_fw_update(icn85xx_ts->fw_name))
			break;
			retry--;
			icn85xx_error("icn85xx_fw_update failed.\n");
		}
	}
#endif/*FORCE_UPDATE_FW*/
#endif/*SUPPORT_FW_UPDATE*/

#ifdef CONFIG_DEV_INFO
	save_tp_info();
#endif
	icn85xx_irq_enable();
	icn85xx_trace("%s end.\n", __func__);
}
#endif/*SUPPORT_DELAYED_WORK*/

#if defined(CONFIG_FB)
#ifdef CONFIG_TINNO_DRV_OPTIMIZE
static void fb_notify_resume_work(struct work_struct *work)
{
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(this_client);
	icn85xx_ts_resume(&icn85xx_ts->client->dev);
}
#endif
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(this_client);
	icn85xx_trace("icn85xx_ts: ** %s 9 **\n", __func__);
#ifdef CONFIG_TINNO_DRV_OPTIMIZE
llll
	if (evdata && evdata->data &&
		icn85xx_ts && icn85xx_ts->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK && event == FB_EVENT_BLANK) {
			schedule_work(&icn85xx_ts->fb_notify_work);
		} else if (*blank == FB_BLANK_POWERDOWN && event == FB_EARLY_EVENT_BLANK) {
			flush_work(&icn85xx_ts->fb_notify_work);
			icn85xx_ts_suspend(&icn85xx_ts->client->dev);
		}
	}
#else
	icn85xx_trace("icn85xx_ts: ** %s 10 **\n", __func__);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
		icn85xx_ts && icn85xx_ts->client) {
		blank = evdata->data;
		icn85xx_trace("icn85xx_ts: ** %s 11 **blank %d\n", __func__ , *blank);

		if (*blank == FB_BLANK_UNBLANK) {
		icn85xx_trace("icn85xx_ts: ** %s 11 **\n", __func__);
			icn85xx_ts_resume(&icn85xx_ts->client->dev);
		} else if (*blank == FB_BLANK_POWERDOWN) {
			icn85xx_ts_suspend(&icn85xx_ts->client->dev);
		}
	}
#endif
	return 0;
}
#endif/*CONFIG_FB*/

#if SUPPORT_CHECK_ESD
static void icn85xx_esd_check_work(struct work_struct *work)
{
	int ret;
	u8 value;
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(this_client);
	int retry = 5;
	int need_update_fw = false;
	icn85xx_trace("icn85xx_esd_check_work enter\n");
	if (icn85xx_ts->icn_is_suspend ||icn85xx_ts->icn_is_update ||
		icn85xx_ts->icn_is_testing||(!esd_mask)) {
		icn85xx_trace("icn85xx Esd suspended or in APK update!");
		return;
	}
	icn85xx_irq_disable();
   if(icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_87)
    {
    		icn85xx_trace("icn85xx_esd_check_work enter_ICN85XX_WITHOUT_FLASH_87\n");
        while (retry-- && !need_update_fw) 
        {
            //icn85xx_ts_reset();
           // icn85xx_set_prog_addr();
           // icn87xx_boot_sram();
           // msleep(50);
            ret = icn85xx_read_reg(0xa, &value);
             if ((ret > 0) && (value == 0x87)) 
            { 
                need_update_fw = false;
                break; 
            }
        }
        if (retry <= 0) 
        	need_update_fw = true;

				retry = 3;
        while (retry-- && need_update_fw) 
        {
						icn85xx_set_prog_addr();
						icn87xx_boot_sram();
						msleep(80);
						ret = icn85xx_read_reg(0xa, &value);
						if ((ret > 0) && (value == 0x87)) 
						{ 
								icn85xx_trace("icn85xx_esd_check_work icn87xx_boot_sram success\n");
								need_update_fw =  false;
								break; 
						}   
        }  
        if (retry <= 0) 
				{
					if(R_OK == icn87xx_fw_update(firmware))
		            {
		                icn85xx_ts->code_loaded_flag = 1;
		                icn85xx_trace("ICN87XX_WITHOUT_FLASH, reload code ok\n"); 
		            }
		            else
		            {
		                icn85xx_ts->code_loaded_flag = 0;
		                icn85xx_trace("ICN87XX_WITHOUT_FLASH, reload code error\n"); 
		            }
				}          
    }
    else
	{
		ret = icn85xx_read_reg(0xa, &value);
		if (ret < 0 )
			icn85xx_ts_reset();
	}
	icn85xx_irq_enable();
	if (!icn85xx_ts->icn_is_suspend) {
		queue_delayed_work(icn85xx_ts->icn85xx_esd_workqueue,
			&icn85xx_ts->icn85xx_esd_event_work, icn85xx_ts->clk_tick_cnt);
	} else {
		icn85xx_trace("Esd suspended!");
	}
	return;
}
#endif/*SUPPORT_CHECK_ESD*/

static int icn85xx_power_init(struct icn85xx_ts_data *data, bool on)
{
	int rc;
	return 0;
	if (!on)
		goto pwr_deinit;

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		icn85xx_trace(
		"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, ICN85XX_VTG_MIN_UV,
			   ICN85XX_VTG_MAX_UV);
		if (rc) {
			icn85xx_trace(
			"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		icn85xx_trace(
		"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, ICN85XX_I2C_VTG_MIN_UV,
			   ICN85XX_I2C_VTG_MAX_UV);
		if (rc) {
			icn85xx_trace(
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
	reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, ICN85XX_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, ICN85XX_VTG_MAX_UV);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, ICN85XX_I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
	return 0;
}

static int icn85xx_power_on(struct icn85xx_ts_data *data, bool on)
{
	int rc;
	return 0;
	if (!on)
		goto power_off;

	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
		"Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
		"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}

	return rc;

power_off:
	rc = regulator_disable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
		"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
		"Regulator vcc_i2c disable failed rc=%d\n", rc);
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
		}
	}

	return rc;
}

static int icn85xx_ts_pinctrl_init(struct icn85xx_ts_data *icn85xx_data)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	icn85xx_data->ts_pinctrl = devm_pinctrl_get(&(icn85xx_data->client->dev));
	if (IS_ERR_OR_NULL(icn85xx_data->ts_pinctrl)) {
		dev_err(&icn85xx_data->client->dev,
		"Target does not use pinctrl\n");
		retval = PTR_ERR(icn85xx_data->ts_pinctrl);
		icn85xx_data->ts_pinctrl = NULL;
		return retval;
	}

	icn85xx_data->gpio_state_active
		= pinctrl_lookup_state(icn85xx_data->ts_pinctrl,
			"pmx_ts_active");
	if (IS_ERR_OR_NULL(icn85xx_data->gpio_state_active)) {
		dev_err(&icn85xx_data->client->dev,
		"Can not get ts default pinstate\n");
		retval = PTR_ERR(icn85xx_data->gpio_state_active);
		icn85xx_data->ts_pinctrl = NULL;
		return retval;
	}

	icn85xx_data->gpio_state_suspend
		= pinctrl_lookup_state(icn85xx_data->ts_pinctrl,
			"pmx_ts_suspend");
	if (IS_ERR_OR_NULL(icn85xx_data->gpio_state_suspend)) {
		dev_err(&icn85xx_data->client->dev,
		"Can not get ts sleep pinstate\n");
		retval = PTR_ERR(icn85xx_data->gpio_state_suspend);
		icn85xx_data->ts_pinctrl = NULL;
		return retval;
	}

	return 0;
}

static int icn85xx_ts_pinctrl_select(struct icn85xx_ts_data *icn85xx_data,
						bool on)
{
	struct pinctrl_state *pins_state;
	int ret;

	pins_state = on ? icn85xx_data->gpio_state_active
		: icn85xx_data->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(icn85xx_data->ts_pinctrl, pins_state);
		if (ret) {
			dev_err(&icn85xx_data->client->dev,
			"can not set %s pins\n",
			on ? "pmx_ts_active" : "pmx_ts_suspend");
			return ret;
		}
	} else {
		dev_err(&icn85xx_data->client->dev,
			"not a valid '%s' pinstate\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
	}

	return 0;
}

static int icn85xx_parse_dt(struct device *dev, struct icn85xx_ts_data *icn85xx_ts)
{
	int rc;
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 temp_val, num_buttons;
	u32 button_map[MAX_BUTTONS];
	u32 gesture_map[MAX_GESTURE];
	int i = 0;

	rc = of_property_read_u32(np, "chipone,screen_max_x", &temp_val);
	if (!rc)
		icn85xx_ts->screen_max_x = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "chipone,screen_max_y", &temp_val);
	if (!rc)
		icn85xx_ts->screen_max_y = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "chipone,short-test-limit-max", &temp_val);
	if (!rc)
		icn85xx_ts->short_test_limit_max = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "chipone,short-test-limit-min", &temp_val);
	if (!rc)
		icn85xx_ts->short_test_limit_min = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "chipone,short-test-vol", &temp_val);
	if (!rc)
		icn85xx_ts->short_test_vol = temp_val;
	else
		return rc;

	prop = of_find_property(np, "chipone,button-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_BUTTONS)
			return -EINVAL;
		icn85xx_ts->button_map = devm_kzalloc(dev,
			sizeof(*icn85xx_ts->button_map), GFP_KERNEL);
		if (!icn85xx_ts->button_map)
			return -ENOMEM;
		icn85xx_ts->button_map->map = devm_kzalloc(dev,
			sizeof(*icn85xx_ts->button_map->map) *
				MAX_BUTTONS, GFP_KERNEL);
		if (!icn85xx_ts->button_map->map)
			return -ENOMEM;
		rc = of_property_read_u32_array(np,
			"chipone,button-map", button_map,
				num_buttons);
		if (rc) {
			dev_err(dev, "Unable to read key codes\n");
			return rc;
		}
		for (i = 0; i < num_buttons; i++)
			icn85xx_ts->button_map->map[i] = button_map[i];
		icn85xx_ts->button_map->nbuttons = num_buttons;
	}

	prop = of_find_property(np, "chipone,gesture-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_GESTURE)
			return -EINVAL;
		icn85xx_ts->gesture_map = devm_kzalloc(dev,
			sizeof(*icn85xx_ts->gesture_map), GFP_KERNEL);
		if (!icn85xx_ts->gesture_map)
			return -ENOMEM;
		icn85xx_ts->gesture_map->map = devm_kzalloc(dev,
			sizeof(*icn85xx_ts->gesture_map->map) *
				MAX_GESTURE, GFP_KERNEL);
		if (!icn85xx_ts->gesture_map->map)
			return -ENOMEM;
		rc = of_property_read_u32_array(np,
			"chipone,gesture-map", gesture_map,
				num_buttons);
		if (rc) {
			dev_err(dev, "Unable to read gesture codes\n");
			return rc;
		}
		for (i = 0; i < num_buttons; i++)
			icn85xx_ts->gesture_map->map[i] = gesture_map[i];
		icn85xx_ts->gesture_map->nbuttons = num_buttons;
	}
	prop = of_find_property(np, "chipone,gesture-key", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_GESTURE)
		return -EINVAL;
		icn85xx_ts->gesture_key = devm_kzalloc(dev,
			sizeof(*icn85xx_ts->gesture_key), GFP_KERNEL);
		if (!icn85xx_ts->gesture_key)
			return -ENOMEM;
		icn85xx_ts->gesture_key->map = devm_kzalloc(dev,
			sizeof(*icn85xx_ts->gesture_key->map) *
				MAX_GESTURE, GFP_KERNEL);
		if (!icn85xx_ts->gesture_key->map)
			return -ENOMEM;
		rc = of_property_read_u32_array(np,
			"chipone,gesture-key", gesture_map,
				num_buttons);
		if (rc) {
			dev_err(dev, "Unable to read gesture codes\n");
			return rc;
		}
		for (i = 0; i < num_buttons; i++)
			icn85xx_ts->gesture_key->map[i] = gesture_map[i];
		icn85xx_ts->gesture_key->nbuttons = num_buttons;
	}

	return 0;
}

#ifdef CONFIG_TOUCHSCREEN_ICN85XX_GESTURE
static unsigned int asic_to_hex(unsigned char val)
{
	if ((val >= '0') && (val <= '9')) {
		val -= '0';
	} else if ((val >= 'a') && (val <= 'z')) {
		val = val - 'a' + 10;
	} else if ((val >= 'A') && (val <= 'Z')) {
		val = val - 'A' + 10;
	}
	return (unsigned int)val;
}

static ssize_t get_gesture_switch(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct icn85xx_ts_data *data = dev_get_drvdata(dev);

	if(data->gesture_switch)
		return 1;
	else
		return 0;
}

static ssize_t set_gesture_switch(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct icn85xx_ts_data *data = dev_get_drvdata(dev);
	unsigned char gesture[10], len;

	strlcpy(gesture, buf, sizeof(gesture));
	len = strlen(gesture);
	if (len > 0) {
		if ((gesture[len-1] == '\n') || (gesture[len-1] == '\0'))
			len--;
	}

	dev_info(&data->client->dev, "%s len: %d gtp_state: %d,%d,%d.\n",
		__func__, len, gesture[0], gesture[1], gesture[2]);
	if (len == 1) {
		if (gesture[0] == '1')
			data->gesture_state = 0xffff;
		else if (gesture[0] == '0')
			data->gesture_state = 0x0;
	} else if (len == 4) {
		data->gesture_state = asic_to_hex(gesture[0])*0x1000
			+ asic_to_hex(gesture[1]) * 0x100
			+ asic_to_hex(gesture[2]) * 0x10
			+ asic_to_hex(gesture[3]);
	} else {
		dev_info(&data->client->dev, "[set_gesture_switch]write wrong cmd.");
		return 0;
	}
	if (!data->gesture_state)
		data->gesture_switch = false;
	else
		data->gesture_switch = true;

	icn85xx_trace("%s is %x.\n", __func__ , data->gesture_state);

	return 0;
}

#endif/*CONFIG_TOUCHSCREEN_ICN85XX_GESTURE*/

#if defined(CONFIG_TOUCHSCREEN_ICN85XX_FH)
static void icn85xx_charger_plugin(struct icn85xx_ts_data *icn85xx_ts, int plugin)
{
	icn85xx_trace("%s: %d -> %d\n", __func__,
				icn85xx_ts->icn_is_charger_present, plugin);

	icn85xx_write_reg(ICN85XX_REG_PMODE, plugin ? 0x55 : 0x66);
}

int icn85xx_work_with_ac_usb_plugin(int plugin)
{
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(this_client);

	if (!device_is_available)
		return 0;
	
	icn85xx_trace("%s: %d -> %d\n", __func__, 
				icn85xx_ts->icn_is_charger_present, plugin);

	if ((!icn85xx_ts->icn_is_suspend) && charger_detect_enable) {
		icn85xx_charger_plugin(icn85xx_ts, plugin);
	} 

	icn85xx_ts->icn_is_charger_present = plugin;
	return 0;
}
EXPORT_SYMBOL_GPL(icn85xx_work_with_ac_usb_plugin);
#endif /* CONFIG_TOUCHSCREEN_ICN85XX_FH */

#ifdef CONFIG_DEV_INFO
static int save_tp_info()
{
	char buf[80];
	char *ic_name = "ICN86L8";
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(this_client);

	sprintf(buf, "HOLITECH-%s-%s--V%X", CONFIG_PRODUCT_NAME, ic_name, icn85xx_ts->ic_version);

	store_tp_info(buf);
	return 0;
}
#endif

/*int icn85xx_device_is_online()
{
	return device_is_online;
}*/
int device_is_online = 0;
EXPORT_SYMBOL(device_is_online);

static int icn85xx_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	short fwVersion = 0;
	short curVersion = 0;
	int retry;
	//char firmware[] = {"icn85xx_firmware"};
	struct icn85xx_ts_data *icn85xx_ts = NULL;

	icn85xx_trace("icn85xx_ts: ** %s enter **\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		icn85xx_error("I2C check functionality failed.\n");
		return -ENODEV;
	}

	icn85xx_ts = kzalloc(sizeof(*icn85xx_ts), GFP_KERNEL);
	if (!icn85xx_ts) {
		icn85xx_error("Alloc icn85xx_ts memory failed.\n");
		return -ENOMEM;
	}
	memset(icn85xx_ts, 0, sizeof(*icn85xx_ts));

	if (client->dev.of_node) {
		err = icn85xx_parse_dt(&client->dev, icn85xx_ts);
		if (err) {
			icn85xx_error("DT parsing failed\n");
			//return err;
		}
	}

	memcpy(icn85xx_ts->fw_name, firmware, sizeof(firmware)-1);
	this_client = client;
	this_client->addr = client->addr;
	/* 85xx addr 0625 */
//Fred, why change the slave address to 0x48?
	this_client->addr = 0x48;//0x90;//
	i2c_set_clientdata(client, icn85xx_ts);
	icn85xx_ts->work_mode = 0;
	spin_lock_init(&icn85xx_ts->irq_lock);

	icn85xx_ts->client = client;

	err = icn85xx_power_init(icn85xx_ts, true);//xx
	if (err) {
		dev_err(&client->dev, "power init failed");
		goto err_power_init;
	}
	err = icn85xx_power_on(icn85xx_ts, true);//xx
	if (err) {
		dev_err(&client->dev, "power init failed");
		goto err_power_on;
	}

	err = icn85xx_ts_pinctrl_init(icn85xx_ts);
	if (!err && icn85xx_ts->ts_pinctrl) {
		err = icn85xx_ts_pinctrl_select(icn85xx_ts, true);
		if (err < 0)
			goto err_pinctrl_init;
	}

	err = icn85xx_request_io_port(client, icn85xx_ts);
	if (err != 0) {
		icn85xx_error("icn85xx_request_io_port failed.\n");
		goto err_request_io_port;
	}
	icn85xx_error("<<<<sun1>>>>\n");

	err = icn85xx_iic_test();
	if (err < 0) {
		icn85xx_error("icn85xx_iic_test  failed.\n");
		device_is_online = 0;
		goto err_iic_test;
	}
	device_is_online = 1;
	icn85xx_error("<<<<sun2>>>>\n");
	
	if((icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_85) || (icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_86))
	{
		
#if SUPPORT_FW_UPDATE
		//icn85xx_set_fw(sizeof(icn85xx_fw), &icn85xx_fw[0]);
#endif    

		if(R_OK == icn85xx_fw_update(firmware))
		{
			icn85xx_ts->code_loaded_flag = 1;
			icn85xx_trace("ICN85XX_WITHOUT_FLASH, update ok\n"); 

		}
		else
		{
			icn85xx_ts->code_loaded_flag = 0;
			icn85xx_trace("ICN85XX_WITHOUT_FLASH, update error\n"); 
			return -1;
		}

	}
	else if((icn85xx_ts->ictype == ICN85XX_WITH_FLASH_85) || (icn85xx_ts->ictype == ICN85XX_WITH_FLASH_86))
	{
#if SUPPORT_FW_UPDATE
		//icn85xx_set_fw(sizeof(icn85xx_fw), &icn85xx_fw[0]);
		fwVersion = icn85xx_read_fw_Ver(firmware);
		curVersion = icn85xx_readVersion();
		icn85xx_trace("fwVersion : 0x%x\n", fwVersion);
		icn85xx_trace("current version: 0x%x\n", curVersion);

#if FORCE_UPDATE_FW
		retry = 5;
		while(retry > 0)
		{
			if(icn85xx_goto_progmode() != 0)
			{
				icn85xx_trace("icn85xx_goto_progmode() != 0 error\n");
				return -1; 
			} 
			icn85xx_read_flashid();
			icn85xx_trace("begin to update\n");
			if(R_OK == icn85xx_fw_update(firmware))
			{
				break;
			}
			retry--;
			icn85xx_trace("icn85xx_fw_update failed.\n");
		}
#else
		if(fwVersion > curVersion)
		{
			retry = 5;
			while(retry > 0)
			{
				if(R_OK == icn85xx_fw_update(firmware))
				{
					break;
				}
				retry--;
				icn85xx_trace("icn85xx_fw_update failed.\n");
			}
		}
#endif
#endif
	}
    else if(icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_87)
    {
        icn85xx_trace("icn85xx_update  87 without flash\n");
        
        #if SUPPORT_FW_UPDATE
            //icn85xx_set_fw(sizeof(icn85xx_fw), &icn85xx_fw[0]);
        #endif
        
        fwVersion = icn85xx_read_fw_Ver(firmware);
        curVersion = icn85xx_readVersion();
        icn85xx_trace("fwVersion : 0x%x\n", fwVersion); 
        icn85xx_trace("current version: 0x%x\n", curVersion);
  
        if(R_OK == icn87xx_fw_update(firmware))
        {
            icn85xx_ts->code_loaded_flag = 1;
            icn85xx_trace("ICN87XX_WITHOUT_FLASH, update default fw ok\n");
        }
        else
        {
            icn85xx_ts->code_loaded_flag = 0;
            icn85xx_trace("ICN87XX_WITHOUT_FLASH, update error\n"); 
        }
     
    }
    else if(icn85xx_ts->ictype == ICN85XX_WITH_FLASH_87)
    {
        icn85xx_trace("icn85xx_update 87 with flash\n");
           
        #if SUPPORT_FW_UPDATE
            //icn85xx_set_fw(sizeof(icn85xx_fw), &icn85xx_fw[0]);
        #endif
        
        fwVersion = icn85xx_read_fw_Ver(firmware);
        curVersion = icn85xx_readVersion();
        icn85xx_trace("fwVersion : 0x%x\n", fwVersion); 
        icn85xx_trace("current version: 0x%x\n", curVersion); 
             
        
        
        #if FORCE_UPDATE_FW        
            if(R_OK == icn87xx_fw_update(firmware))
            {
                icn85xx_ts->code_loaded_flag = 1;
                icn85xx_trace("ICN87XX_WITH_FLASH, update default fw ok\n");
            }
            else
            {
                icn85xx_ts->code_loaded_flag = 0;
                icn85xx_trace("ICN87XX_WITH_FLASH, update error\n"); 
            }    
         
        #else
            if(fwVersion > curVersion)
            {
                retry = 5;
                while(retry > 0)
                {
                    if(R_OK == icn87xx_fw_update(firmware))
                    {
                        break;
                    }
                    retry--;
                    icn85xx_trace("icn87xx_fw_update failed.\n");   
                }
            }
        #endif
    }


	err = icn85xx_readVersion();
	if (err < 0) {
		icn85xx_error("icn85xx read version failed.\n");
		if (icn85xx_goto_progmode() != 0) {
			icn85xx_error("icn85xx_goto_progmode error, exit!\n");
			goto err_iic_test;
		} else {
//			icn85xx_set_fw( sizeof(icn85xx_fw), &icn85xx_fw[0]);
			if(icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_87)
				err = icn87xx_fw_update(firmware);
			else
				err = icn85xx_fw_update(firmware);
			if (err != R_OK) {
				icn85xx_error("icn85xx update failed.\n");
				goto err_iic_test;
			} else {
				if (icn85xx_readVersion() < 0) {
					icn85xx_error("icn85xx read version failed.\n");
					goto err_iic_test;
				}
			}
		}
	}
#if  0
	err = icn85xx_get_pannel_id(icn85xx_ts);
	if (err < 0) {
		icn85xx_error("icn85xx get pannel id failed.\n");
		goto err_iic_test;
	}
#endif

#if  0
	if (icn85xx_ts->ictype == ICN85XX_WITHOUT_FLASH_85) {
		if (R_OK == icn85xx_fw_update(icn85xx_ts->fw_name)) {
			icn85xx_ts->code_loaded_flag = 1;
			icn85xx_trace("ICN85XX_WITHOUT_FLASH_85, update ok\n");
		} else {
			icn85xx_ts->code_loaded_flag = 0;
			icn85xx_trace("ICN85XX_WITHOUT_FLASH_85, update error\n");
			return -EINVAL;
		}
	}
#endif/*0*/

	INIT_WORK(&icn85xx_ts->pen_event_work, icn85xx_ts_pen_irq_work);
	icn85xx_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!icn85xx_ts->ts_workqueue) {
		icn85xx_error("create_singlethread_workqueue failed.\n");
		goto err_create_workqueue;
	}

	err = icn85xx_request_input_dev(icn85xx_ts);
	if (err < 0) {
		icn85xx_error("request input dev failed\n");
		goto err_request_input_dev;
	}

	err = icn85xx_request_irq(icn85xx_ts);
	if (err) {
		icn85xx_error("request irq error, use timer\n");
		icn85xx_ts->use_irq = 0;
		hrtimer_init(&icn85xx_ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		icn85xx_ts->timer.function = chipone_timer_func;
		hrtimer_start(&icn85xx_ts->timer, ktime_set(CTP_START_TIMER/1000,
			(CTP_START_TIMER % 1000) * 1000000), HRTIMER_MODE_REL);
	}

#if defined(CONFIG_FB)
#ifdef CONFIG_TINNO_DRV_OPTIMIZE
	INIT_WORK(&icn85xx_ts->fb_notify_work, fb_notify_resume_work);
#endif
	icn85xx_ts->fb_notif.notifier_call = fb_notifier_callback;
	err = fb_register_client(&icn85xx_ts->fb_notif);
	if (err)
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n", err);
#endif/*CONFIG_FB*/

#if SUPPORT_PROC_FS
	sema_init(&icn85xx_ts->sem, 1);
	init_proc_node();
	icn_create_sysfs(client);
#endif/*SUPPORT_PROC_FS*/

#if SUPPORT_CHECK_ESD
	INIT_DELAYED_WORK(&icn85xx_ts->icn85xx_esd_event_work, icn85xx_esd_check_work);
	icn85xx_ts->icn85xx_esd_workqueue = create_workqueue("icn85xx_esd");
	icn85xx_ts->clk_tick_cnt = 2* HZ;
	icn85xx_trace("Clock ticks for an esd cycle: %d", icn85xx_ts->clk_tick_cnt);
	spin_lock_init(&icn85xx_ts->esd_lock);
	spin_lock(&icn85xx_ts->esd_lock);
	icn85xx_ts->esd_running = 1;
	spin_unlock(&icn85xx_ts->esd_lock);
	icn85xx_trace("Esd started");
	queue_delayed_work(icn85xx_ts->icn85xx_esd_workqueue,
		&icn85xx_ts->icn85xx_esd_event_work, icn85xx_ts->clk_tick_cnt);
#endif/*SUPPORT_CHECK_ESD*/

#if SUPPORT_DELAYED_WORK
	INIT_DELAYED_WORK(&icn85xx_ts->icn_delayed_work, icn_delayedwork_fun);
	schedule_delayed_work(&icn85xx_ts->icn_delayed_work, msecs_to_jiffies(8000));
#else/*SUPPORT_DELAYED_WORK*/
	icn85xx_irq_enable();
#endif/*SUPPORT_DELAYED_WORK*/

#if defined(CONFIG_TOUCHSCREEN_ICN85XX_FH)
	device_is_available = 1;
	/* Init charger status */
	icn85xx_work_with_ac_usb_plugin(usb_flag);
#endif /* CONFIG_TOUCHSCREEN_ICN85XX_FH */
	icn85xx_trace("icn85xx_ts: ** %s sussecc **\n", __func__);
	icn85xx_error("<<<<sun_ending>>>>\n");

	return 0;

err_request_input_dev:
	destroy_workqueue(icn85xx_ts->ts_workqueue);
err_create_workqueue:
err_iic_test:
	if (gpio_is_valid(icn85xx_ts->rst_gpio))
		gpio_free(icn85xx_ts->rst_gpio);
	if (gpio_is_valid(icn85xx_ts->irq_gpio))
		gpio_free(icn85xx_ts->irq_gpio);
err_request_io_port:
	if (icn85xx_ts->ts_pinctrl) {
		icn85xx_ts_pinctrl_select(icn85xx_ts, false);
		devm_pinctrl_put(icn85xx_ts->ts_pinctrl);
		icn85xx_ts->ts_pinctrl = NULL;
	}
err_pinctrl_init:
	icn85xx_power_on(icn85xx_ts, false);
err_power_on:
	icn85xx_power_init(icn85xx_ts, false);
err_power_init:
	kfree(icn85xx_ts);
	return err;
}

static int icn85xx_ts_remove(struct i2c_client *client)
{
	struct icn85xx_ts_data *icn85xx_ts = i2c_get_clientdata(client);
	icn85xx_trace("==icn85xx_ts_remove=\n");
	icn85xx_irq_disable();
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&icn85xx_ts->early_suspend);
#endif/*CONFIG_HAS_EARLYSUSPEND*/

#if SUPPORT_PROC_FS
	uninit_proc_node();
#endif/*SUPPORT_PROC_FS*/

#if defined(CONFIG_FB)
	if (fb_unregister_client(&icn85xx_ts->fb_notif))
	dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#endif/*CONFIG_FB*/
	input_unregister_device(icn85xx_ts->input_dev);
	//input_free_device(icn85xx_ts->input_dev);
	cancel_work_sync(&icn85xx_ts->pen_event_work);
	destroy_workqueue(icn85xx_ts->ts_workqueue);
	icn85xx_ts_pinctrl_select(icn85xx_ts, false);
	icn85xx_free_irq(icn85xx_ts);
	icn85xx_free_io_port(icn85xx_ts);
	kfree(icn85xx_ts);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id icn85xx_ts_id[] = {
	{ CTP_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, icn85xx_ts_id);

static struct of_device_id icn85xx_match_table[] = {
	{ .compatible = "chipone,85xx",},
	{ },
};

static struct i2c_driver icn85xx_ts_driver = {
	//.class      = I2C_CLASS_HWMON,
	.probe      = icn85xx_ts_probe,
	.remove     = icn85xx_ts_remove,
	.id_table   = icn85xx_ts_id,
	.driver = {
		.name   = CTP_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = icn85xx_match_table,
#if SUPPORT_SYSFS
		.groups = icn_drv_grp,
#endif/*SUPPORT_SYSFS*/
#ifdef CONFIG_TOUCHSCREEN_ICN85XX_GESTURE
		.pm = &icn85xx_ts_dev_pm_ops,
#endif/*CONFIG_TOUCHSCREEN_ICN85XX_GESTURE*/
	},
};
static int __init icn85xx_ts_init(void)
{
	int ret = -1;

	icn85xx_trace("%s enter!\n", __func__);
	ret = i2c_add_driver(&icn85xx_ts_driver);
	icn85xx_trace("%s sussecc!\n", __func__);
	return ret;
}

static void __exit icn85xx_ts_exit(void)
{
	icn85xx_trace("==icn85xx_ts_exit==\n");
	i2c_del_driver(&icn85xx_ts_driver);
}

module_init(icn85xx_ts_init);
module_exit(icn85xx_ts_exit);

MODULE_AUTHOR("miaodefang <dfmiao@chipone.com>");
MODULE_DESCRIPTION("Chipone touchscreen driver");
MODULE_LICENSE("GPL");
