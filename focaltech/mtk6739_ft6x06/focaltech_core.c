/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/wakelock.h>
#include "include/tpd_ft5x0x_common.h"
#include "focaltech_core.h"
#include "tpd.h"
#include "base.h"

/* #define TIMER_DEBUG */
#ifdef TIMER_DEBUG
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#endif

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
#include <mach/md32_ipi.h>
#include <mach/md32_helper.h>
#endif
#include <linux/proc_fs.h>

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
enum DOZE_T {
	DOZE_DISABLED = 0,
	DOZE_ENABLED = 1,
	DOZE_WAKEUP = 2,
};
static DOZE_T doze_status = DOZE_DISABLED;
#endif

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
static s8 ftp_enter_doze(struct i2c_client *client);

enum TOUCH_IPI_CMD_T {
	/* SCP->AP */
	IPI_COMMAND_SA_GESTURE_TYPE,
	/* AP->SCP */
	IPI_COMMAND_AS_CUST_PARAMETER,
	IPI_COMMAND_AS_ENTER_DOZEMODE,
	IPI_COMMAND_AS_ENABLE_GESTURE,
	IPI_COMMAND_AS_GESTURE_SWITCH,
};

struct Touch_Cust_Setting {
	u32 i2c_num;
	u32 int_num;
	u32 io_int;
	u32 io_rst;
};

struct Touch_IPI_Packet {
	u32 cmd;
	union {
		u32 data;
		Touch_Cust_Setting tcs;
	} param;
};

/* static bool tpd_scp_doze_en = FALSE; */
static bool tpd_scp_doze_en = TRUE;
DEFINE_MUTEX(i2c_access);
#endif

#define TPD_SUPPORT_POINTS	5
#define WRITE_BUF_SIZE  1016
#define PROC_UPGRADE							0
#define PROC_READ_REGISTER						1
#define PROC_WRITE_REGISTER					    2
#define PROC_AUTOCLB							4
#define PROC_UPGRADE_INFO						5
#define PROC_WRITE_DATA						    6
#define PROC_READ_DATA							7
#define PROC_SET_TEST_FLAG						8
/*******************************************************************************
* 4.Static variables
*******************************************************************************/
struct i2c_client *i2c_client = NULL;
struct task_struct *thread_tpd = NULL;
struct i2c_client *fts_i2c_client 				= NULL;
struct input_dev *fts_input_dev				=NULL;
#ifdef TPD_AUTO_UPGRADE
static bool is_update = false;
#endif
#ifdef CONFIG_FT_AUTO_UPGRADE_SUPPORT
u8 *tpd_i2c_dma_va = NULL;
dma_addr_t tpd_i2c_dma_pa = 0;
#endif
static struct kobject *touchscreen_dir=NULL;
static struct kobject *virtual_dir=NULL;
static struct kobject *touchscreen_dev_dir=NULL;
static char *vendor_name=NULL;
static u8 ctp_fw_version;
static unsigned char proc_operate_mode 			= PROC_UPGRADE;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id);
static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
static void tpd_resume(struct device *h);
static void tpd_suspend(struct device *h);
static int tpd_flag;
static struct wake_lock ft_wake_lock;

#if TPD_PROXIMITY
#define APS_ERR(fmt,arg...)           	printk("<<proximity>> "fmt"\n",##arg)
#define TPD_PROXIMITY_DEBUG(fmt,arg...) printk("<<proximity>> "fmt"\n",##arg)
#define TPD_PROXIMITY_DMESG(fmt,arg...) printk("<<proximity>> "fmt"\n",##arg)
static volatile u8 tpd_proximity_flag 			= 0;
static volatile u8 tpd_proximity_suspend 		= 0;
static volatile u8 tpd_proximity_detect 		= 1;//0-->close ; 1--> far away
static volatile u8 tpd_proximity_detect_prev	= 0xff;//0-->close ; 1--> far away
#endif
unsigned int tpd_rst_gpio_number = 0;
unsigned int tpd_int_gpio_number = 1;
unsigned int touch_irq = 0;
u8 focaltech_halt=0;
#define TPD_OK 0
int fts_i2c_read2(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen);
/* Register define */
#define DEVICE_MODE	0x00
#define GEST_ID		0x01
#define TD_STATUS	0x02

#define TOUCH1_XH	0x03
#define TOUCH1_XL	0x04
#define TOUCH1_YH	0x05
#define TOUCH1_YL	0x06

#define TOUCH2_XH	0x09
#define TOUCH2_XL	0x0A
#define TOUCH2_YH	0x0B
#define TOUCH2_YL	0x0C

#define TOUCH3_XH	0x0F
#define TOUCH3_XL	0x10
#define TOUCH3_YH	0x11
#define TOUCH3_YL	0x12

#define TPD_RESET_ISSUE_WORKAROUND
#define TPD_MAX_RESET_COUNT	3

#ifdef TIMER_DEBUG

static struct timer_list test_timer;

static void timer_func(unsigned long data)
{
	tpd_flag = 1;
	wake_up_interruptible(&waiter);

	mod_timer(&test_timer, jiffies + 100*(1000/HZ));
}

static int init_test_timer(void)
{
	memset((void *)&test_timer, 0, sizeof(test_timer));
	test_timer.expires  = jiffies + 100*(1000/HZ);
	test_timer.function = timer_func;
	test_timer.data     = 0;
	init_timer(&test_timer);
	add_timer(&test_timer);
	return 0;
}
#endif


#if defined(CONFIG_TPD_ROTATE_90) || defined(CONFIG_TPD_ROTATE_270) || defined(CONFIG_TPD_ROTATE_180)
/*
static void tpd_swap_xy(int *x, int *y)
{
	int temp = 0;

	temp = *x;
	*x = *y;
	*y = temp;
}
*/
/*
static void tpd_rotate_90(int *x, int *y)
{
//	int temp;

	*x = TPD_RES_X + 1 - *x;

	*x = (*x * TPD_RES_Y) / TPD_RES_X;
	*y = (*y * TPD_RES_X) / TPD_RES_Y;

	tpd_swap_xy(x, y);
}
*/
static void tpd_rotate_180(int *x, int *y)
{
	*y = TPD_RES_Y + 1 - *y;
	*x = TPD_RES_X + 1 - *x;
}
/*
static void tpd_rotate_270(int *x, int *y)
{
//	int temp;

	*y = TPD_RES_Y + 1 - *y;

	*x = (*x * TPD_RES_Y) / TPD_RES_X;
	*y = (*y * TPD_RES_X) / TPD_RES_Y;

	tpd_swap_xy(x, y);
}
*/
#endif

struct touch_info {
	int y[TPD_SUPPORT_POINTS];
	int x[TPD_SUPPORT_POINTS];
	int p[TPD_SUPPORT_POINTS];
	int id[TPD_SUPPORT_POINTS];
	int count;
};

/*dma declare, allocate and release*/
#define __MSG_DMA_MODE__
#ifdef __MSG_DMA_MODE__
	u8 *g_dma_buff_va = NULL;
	dma_addr_t g_dma_buff_pa = 0;
#endif

#ifdef __MSG_DMA_MODE__

	static void msg_dma_alloct(void)
	{
	    if (NULL == g_dma_buff_va)
    		{
       		 tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
       		 g_dma_buff_va = (u8 *)dma_alloc_coherent(&tpd->dev->dev, 128, &g_dma_buff_pa, GFP_KERNEL);
    		}

	    	if(!g_dma_buff_va)
		{
	        	TPD_DMESG("[DMA][Error] Allocate DMA I2C Buffer failed!\n");
	    	}
	}
	static void msg_dma_release(void){
		if(g_dma_buff_va)
		{
	     		dma_free_coherent(NULL, 128, g_dma_buff_va, g_dma_buff_pa);
	        	g_dma_buff_va = NULL;
	        	g_dma_buff_pa = 0;
			TPD_DMESG("[DMA][release] Allocate DMA I2C Buffer release!\n");
	    	}
	}
#endif

static DEFINE_MUTEX(i2c_access);
static DEFINE_MUTEX(i2c_rw_access);

#if (defined(CONFIG_TPD_HAVE_CALIBRATION) && !defined(CONFIG_TPD_CUSTOM_CALIBRATION))
static int tpd_def_calmat_local_normal[8]  = TPD_CALIBRATION_MATRIX_ROTATION_NORMAL;
static int tpd_def_calmat_local_factory[8] = TPD_CALIBRATION_MATRIX_ROTATION_FACTORY;
#endif

static const struct i2c_device_id ft5x0x_tpd_id[] = {{"ft5x0x", 0}, {} };
static const struct of_device_id ft5x0x_dt_match[] = {
	{.compatible = "mediatek,cap_touch"},
	{},
};
MODULE_DEVICE_TABLE(of, ft5x0x_dt_match);

static struct i2c_driver tpd_i2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(ft5x0x_dt_match),
		.name = "ft5x0x",
	},
	.probe = tpd_probe,
	.remove = tpd_remove,
	.id_table = ft5x0x_tpd_id,
	.detect = tpd_i2c_detect,
};

static int of_get_ft5x0x_platform_data(struct device *dev)
{
	if (dev->of_node) {
		const struct of_device_id *match;

		match = of_match_device(of_match_ptr(ft5x0x_dt_match), dev);
		if (!match) {
			TPD_DMESG("Error: No device match found\n");
			return -ENODEV;
		}
	}

	TPD_DMESG("g_vproc_en_gpio_number %d\n", tpd_rst_gpio_number);
	TPD_DMESG("g_vproc_vsel_gpio_number %d\n", tpd_int_gpio_number);
	return 0;
}

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
static ssize_t show_scp_ctrl(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}
static ssize_t store_scp_ctrl(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	u32 cmd;
	Touch_IPI_Packet ipi_pkt;

	if (kstrtoul(buf, 10, &cmd)) {
		TPD_DEBUG("[SCP_CTRL]: Invalid values\n");
		return -EINVAL;
	}

	TPD_DEBUG("SCP_CTRL: Command=%d", cmd);
	switch (cmd) {
	case 1:
	    /* make touch in doze mode */
	    tpd_scp_wakeup_enable(TRUE);
	    tpd_suspend(NULL);
	    break;
	case 2:
	    tpd_resume(NULL);
	    break;
		/*case 3:
	    // emulate in-pocket on
	    ipi_pkt.cmd = IPI_COMMAND_AS_GESTURE_SWITCH,
	    ipi_pkt.param.data = 1;
		md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 0);
	    break;
	case 4:
	    // emulate in-pocket off
	    ipi_pkt.cmd = IPI_COMMAND_AS_GESTURE_SWITCH,
	    ipi_pkt.param.data = 0;
		md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 0);
	    break;*/
	case 5:
		{
				Touch_IPI_Packet ipi_pkt;

				ipi_pkt.cmd = IPI_COMMAND_AS_CUST_PARAMETER;
			    ipi_pkt.param.tcs.i2c_num = TPD_I2C_NUMBER;
			ipi_pkt.param.tcs.int_num = CUST_EINT_TOUCH_PANEL_NUM;
				ipi_pkt.param.tcs.io_int = tpd_int_gpio_number;
			ipi_pkt.param.tcs.io_rst = tpd_rst_gpio_number;
			if (md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 0) < 0)
				TPD_DEBUG("[TOUCH] IPI cmd failed (%d)\n", ipi_pkt.cmd);

			break;
		}
	default:
	    TPD_DEBUG("[SCP_CTRL] Unknown command");
	    break;
	}

	return size;
}
static DEVICE_ATTR(tpd_scp_ctrl, 0664, show_scp_ctrl, store_scp_ctrl);
#endif

static struct device_attribute *ft5x0x_attrs[] = {
#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
	&dev_attr_tpd_scp_ctrl,
#endif
};

static ssize_t mtk_ctp_firmware_vertion_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
  int ret;
  ret=sprintf(buf, "%s:%s:0x%02x:%x\n",vendor_name,"FT5446",ctp_fw_version,0);
  return ret;
}

static struct kobj_attribute ctp_firmware_vertion_attr = {
	.attr = {
		 .name = "firmware_version",                    
		 .mode = S_IRUGO,
		 },
	.show = &mtk_ctp_firmware_vertion_show,
};

static ssize_t mtk_ctp_firmware_update_store(struct kobject *kobj, struct kobj_attribute *attr,const char *buf, size_t count)
{
  unsigned char writebuf[WRITE_BUF_SIZE];
	int buflen = count;
	int writelen = 0;
	int ret = 0;
	
#if FT_ESD_PROTECT
    esd_switch(0);
	apk_debug_flag = 1;
//FTS_DBG("\n  zax v= %d \n",apk_debug_flag);
#endif
	if (copy_from_user(&writebuf, buf, buflen)) {
		dev_err(&fts_i2c_client->dev, "%s:copy from user error\n", __func__);
#if FT_ESD_PROTECT
	esd_switch(1);
    apk_debug_flag = 0;
#endif
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];

	switch (proc_operate_mode) {
	
	case PROC_UPGRADE:
		{
			char upgrade_file_path[128];
			memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
			sprintf(upgrade_file_path, "%s", writebuf + 1);
			upgrade_file_path[buflen-1] = '\0';
			TPD_DEBUG("%s\n", upgrade_file_path);
			//#if FT_ESD_PROTECT
			//	esd_switch(0);apk_debug_flag = 1;
			//#endif
			disable_irq(fts_i2c_client->irq);
			ret = fts_ctpm_fw_upgrade_with_app_file(fts_i2c_client, upgrade_file_path);
			enable_irq(fts_i2c_client->irq);
			if (ret < 0) {
				dev_err(&fts_i2c_client->dev, "%s:upgrade failed.\n", __func__);
				#if FT_ESD_PROTECT
					esd_switch(1);apk_debug_flag = 0;
				#endif
				return ret;
			}
		}
		break;

	case PROC_SET_TEST_FLAG:
		#if FT_ESD_PROTECT
		apk_debug_flag=writebuf[1];
		if(1==apk_debug_flag)
			esd_switch(0);
		else if(0==apk_debug_flag)
			esd_switch(1);
		FTS_DBG("\n zax flag=%d \n",apk_debug_flag);
		#endif
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
#if FT_ESD_PROTECT
					esd_switch(1);apk_debug_flag = 0;
				#endif
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
#if FT_ESD_PROTECT
					esd_switch(1);apk_debug_flag = 0;
				#endif
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_AUTOCLB:
		TPD_DEBUG("%s: autoclb\n", __func__);
		fts_ctpm_auto_clb(fts_i2c_client);
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = count - 1;
		if(writelen>0)
		{
			ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
			if (ret < 0) {
#if FT_ESD_PROTECT
					esd_switch(1);apk_debug_flag = 0;
				#endif
				dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
				return ret;
			}
		}
		break;
	default:
		break;
	}
	
	#if FT_ESD_PROTECT
		//FTS_DBG("\n  zax proc w 1 \n");		
esd_switch(1);apk_debug_flag = 0;
//FTS_DBG("\n  zax v= %d \n",apk_debug_flag);
			#endif
	return count; 
 
}

static struct kobj_attribute ctp_firmware_update_attr = {
	.attr = {
		 .name = "firmware_update",
		 .mode = S_IWUGO,
		 },
	.store = &mtk_ctp_firmware_update_store,

};

static ssize_t mtk_ctp_vendor_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
 int ret;
 ret=sprintf(buf,"%s\n",vendor_name); 
 return ret;
}

static struct kobj_attribute ctp_vendor_attr = {
	.attr = {
		 .name = "vendor",
		 .mode = S_IRUGO,
		 },
	.show = &mtk_ctp_vendor_show,
};

static struct attribute *mtk_properties_attrs[] = {
	&ctp_firmware_vertion_attr.attr,
	&ctp_vendor_attr.attr,
	&ctp_firmware_update_attr.attr,
	NULL
};

static struct attribute_group mtk_ctp_attr_group = {
	.attrs = mtk_properties_attrs,
};

static int create_ctp_node(void)
{
  int ret;
  virtual_dir = virtual_device_parent(NULL);
  if(!virtual_dir)
  {
   FTS_DBG("Get virtual dir failed\n");
   return -ENOMEM;
  }
  touchscreen_dir=kobject_create_and_add("touchscreen",virtual_dir);
  if(!touchscreen_dir)
  {
   FTS_DBG("Create touchscreen dir failed\n");
   return -ENOMEM;
  }
  touchscreen_dev_dir=kobject_create_and_add("touchscreen_dev",touchscreen_dir);
  if(!touchscreen_dev_dir)
  {
   FTS_DBG("Create touchscreen_dev dir failed\n");
   return -ENOMEM;
  }
  ret=sysfs_create_group(touchscreen_dev_dir, &mtk_ctp_attr_group);
  if(ret)
  {
    FTS_DBG("create mtk_ctp_firmware_vertion_attr_group error\n");
  } 
 
  return 0;
}

static void tpd_down(int x, int y, int p)
{
#if defined(CONFIG_TPD_ROTATE_90)
	tpd_rotate_90(&x, &y);
#elif defined(CONFIG_TPD_ROTATE_270)
	tpd_rotate_270(&x, &y);
#elif defined(CONFIG_TPD_ROTATE_180)
	tpd_rotate_180(&x, &y);
#endif

#ifdef TPD_SOLVE_CHARGING_ISSUE
	if (0 != x) {
#else
	{
#endif
        
		input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, p);
		TPD_DEBUG("%s x:%d y:%d p:%d\n", __func__, x, y, p);
		input_report_key(tpd->dev, BTN_TOUCH, 1);
		input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
		input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
		input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
		input_mt_sync(tpd->dev);
	
	}
}

static void tpd_up(int x, int y,int id)
{
#if defined(CONFIG_TPD_ROTATE_90)
	tpd_rotate_90(&x, &y);
#elif defined(CONFIG_TPD_ROTATE_270)
	tpd_rotate_270(&x, &y);
#elif defined(CONFIG_TPD_ROTATE_180)
	tpd_rotate_180(&x, &y);
#endif

#ifdef TPD_SOLVE_CHARGING_ISSUE
	if (0 != x) {
#else
	{
#endif
		TPD_DEBUG("%s x:%d y:%d\n", __func__, x, y);
		input_report_key(tpd->dev, BTN_TOUCH, 0);
		input_mt_sync(tpd->dev);
	}
}

static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
{
	int i = 0;
	char data[40] = {0};
	u8 report_rate = 0;
	u16 high_byte, low_byte;
	char writebuf[10]={0};

	writebuf[0]=0x00;
//	FTS_DBG("gavin tpd_touchinfo 1\n");
	fts_i2c_read2(i2c_client, writebuf,  1, data, 32);
	fts_read_reg(i2c_client, 0x88, &report_rate);

	if (report_rate < 8) {
		report_rate = 0x8;
		if ((fts_write_reg(i2c_client, 0x88, report_rate)) < 0)
			TPD_DMESG("I2C write report rate error, line: %d\n", __LINE__);
	}

	/* Device Mode[2:0] == 0 :Normal operating Mode*/
	if ((data[0] & 0x70) != 0)
		return false;

	memcpy(pinfo, cinfo, sizeof(struct touch_info));
	memset(cinfo, 0, sizeof(struct touch_info));
	for (i = 0; i < TPD_SUPPORT_POINTS; i++)
		cinfo->p[i] = 1;	/* Put up */

	/*get the number of the touch points*/
	cinfo->count = data[2] & 0x0f;

	TPD_DEBUG("Number of touch points = %d\n", cinfo->count);

	TPD_DEBUG("Procss raw data...\n");

	for (i = 0; i < cinfo->count; i++) {
		cinfo->p[i] = (data[3 + 6 * i] >> 6) & 0x0003; /* event flag */
		cinfo->id[i] = data[3+6*i+2]>>4; 						// touch id

		/*get the X coordinate, 2 bytes*/
		high_byte = data[3 + 6 * i];
		high_byte <<= 8;
		high_byte &= 0x0F00;

		low_byte = data[3 + 6 * i + 1];
		low_byte &= 0x00FF;
		cinfo->x[i] = high_byte | low_byte;

		/*get the Y coordinate, 2 bytes*/
		high_byte = data[3 + 6 * i + 2];
		high_byte <<= 8;
		high_byte &= 0x0F00;

		low_byte = data[3 + 6 * i + 3];
		low_byte &= 0x00FF;
		cinfo->y[i] = high_byte | low_byte;
	}

#ifdef CONFIG_TPD_HAVE_CALIBRATION
	for (i = 0; i < cinfo->count; i++) {
		tpd_calibrate_driver(&(cinfo->x[i]), &(cinfo->y[i]));
		TPD_DEBUG(" cinfo->x[%d] = %d, cinfo->y[%d] = %d, cinfo->p[%d] = %d\n", i,
		cinfo->x[i], i, cinfo->y[i], i, cinfo->p[i]);
	}
#endif

	return true;
};

#if TPD_PROXIMITY
static int tpd_enable_ps(int enable)
{
	u8 state, state2;

	if(focaltech_halt == 1)
		return 0;
		
	if(enable == tpd_proximity_flag)
		return 0;

	fts_read_reg(i2c_client, 0xB0, &state);
	FTS_DBG("[proxi_5206]read: 999 0xb0's value is 0x%02X\n", state);
	if (enable){
		state |= 0x01;
		tpd_proximity_flag = 1;
		tpd_proximity_detect = 1;
		tpd_proximity_detect_prev = 0xff;
		TPD_PROXIMITY_DEBUG("[proxi_5206]ps function is on\n");
		fts_write_reg(i2c_client, 0xB0, 0x01);
		wake_lock(&ft_wake_lock);
	}else{
		state &= 0x00;
		tpd_proximity_flag = 0;
		tpd_proximity_detect = 1;
		tpd_proximity_detect_prev = 0xff;
		TPD_PROXIMITY_DEBUG("[proxi_5206]ps function is off\n");
		fts_write_reg(i2c_client, 0xB0, 0x00);
		wake_unlock(&ft_wake_lock);
	}
	
	TPD_PROXIMITY_DEBUG("[proxi_5206]write: 0xB0's value is 0x%02X\n", state);
	fts_read_reg(i2c_client, 0xB0, &state2);

	return 0;
}

static int ft_prox_read(struct seq_file *m, void *v)
{
	char name[20];
	memset(name, 0, 20);
	seq_printf(m, "onoff:%x,status:%d\n", tpd_proximity_flag, tpd_proximity_detect);
	return 0;
};

static int proc_ft_prox_open(struct inode *inode, struct file *file)
{
    return single_open(file, ft_prox_read, NULL);
};

static  struct file_operations ft_prox_get_proc = {
    .owner = THIS_MODULE,
    .open  = proc_ft_prox_open,
    .read  = seq_read,
};

static int ft_prox_on_read(struct seq_file *m, void *v)
{
	tpd_enable_ps(1);
	return 0;
};

static int proc_ft_prox_on(struct inode *inode, struct file *file)
{
    return single_open(file, ft_prox_on_read, NULL);
};

static  struct file_operations ft_prox_on_proc = {
    .owner = THIS_MODULE,
    .open  = proc_ft_prox_on,
    .read  = seq_read,
};

static int ft_prox_off_read(struct seq_file *m, void *v)
{
	tpd_enable_ps(0);
	return 0;
};

static int proc_ft_prox_off(struct inode *inode, struct file *file)
{
    return single_open(file, ft_prox_off_read, NULL);
};

static  struct file_operations ft_prox_off_proc = {
    .owner = THIS_MODULE,
    .open  = proc_ft_prox_off,
    .read  = seq_read,
};
#endif

extern int mtk_i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num,
u32 ext_flag, u32 timing);

#if 1
int fts_i2c_read(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen)
{
  int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			pr_err("gavin 1 f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			pr_err("gavin 2 %s:i2c read error.\n", __func__);
	}
	return ret;
}

int fts_i2c_read2(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen)
{
  int ret;
	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = readlen,
		 .buf = readbuf,
		 },
	};
	ret = mtk_i2c_transfer(
		client->adapter,
		msg,
		2,
		client->flags,
		400000);

	if (ret < 0)
		pr_err("gavin 1 f%s: i2c read error.\n",__func__);

	return ret;
}

int fts_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error.\n", __func__);

	return ret;
}
#else
int fts_i2c_read(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen)
{
  int ret;
	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = readlen,
		 .buf = readbuf,
		 },
	};
	ret = mtk_i2c_transfer(
		client->adapter,
		msg,
		2,
		client->flags,
		400000);

	if (ret < 0)
		pr_err("gavin 1 f%s: i2c read error.\n",__func__);

	return ret;
}

int fts_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};
	ret = mtk_i2c_transfer(
		client->adapter,
		msg,
		1,
		client->flags,
		400000);

	if (ret < 0)
		pr_err("%s i2c write error.\n", __func__);

	return ret;
}
#endif
/************************************************************************
* Name: fts_write_reg
* Brief: write register
* Input: i2c info, reg address, reg value
* Output: no
* Return: fail <0
***********************************************************************/
int fts_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};

	buf[0] = regaddr;
	buf[1] = regvalue;

	return fts_i2c_write(client, buf, sizeof(buf));
}
/************************************************************************
* Name: fts_read_reg
* Brief: read register
* Input: i2c info, reg address, reg value
* Output: get reg value
* Return: fail <0
***********************************************************************/
int fts_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
	return fts_i2c_read(client, &regaddr, 1, regvalue, 1);
}

#if USB_CHARGE_DETECT
//extern int FG_charging_status ;
int FG_charging_status = 0;

int close_to_ps_flag_value = 1;	// 1: close ; 0: far away
int charging_flag = 0;
#endif
static int touch_event_handler(void *unused)
{
	int i = 0;
	#if FTS_GESTRUE_EN
	int ret = 0;
	u8 state = 0;
	#endif
#if TPD_PROXIMITY
	u8 state1 = 0;
	u8 proximity_status;
#endif
	#if USB_CHARGE_DETECT
	u8 data;
	#endif
	struct touch_info cinfo, pinfo, finfo;
	struct sched_param param = { .sched_priority = 4 };

	if (tpd_dts_data.use_tpd_button) {
		memset(&finfo, 0, sizeof(struct touch_info));
		for (i = 0; i < TPD_SUPPORT_POINTS; i++)
			finfo.p[i] = 1;
	}

	sched_setscheduler(current, SCHED_RR, &param);

	do {
		/*enable_irq(touch_irq);*/
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, tpd_flag != 0);

		tpd_flag = 0;

		set_current_state(TASK_RUNNING);

#if USB_CHARGE_DETECT
		if((FG_charging_status != 0) && (charging_flag == 0))
		{
			data = 0x1;
			charging_flag = 1;
			fts_write_reg(i2c_client, 0x8B, 0x01);  
		}
		else
		{
			if((FG_charging_status == 0) && (charging_flag == 1))
			{
				charging_flag = 0;
				data = 0x0;
				fts_write_reg(i2c_client, 0x8B, 0x00);  					
			}
		}
#endif
 
		#if FTS_GESTRUE_EN
			ret = fts_read_reg(fts_i2c_client, 0xd0,&state);
			if (ret<0)
			{
				FTS_DBG("[Focal][Touch] read value fail");
				//return ret;
			}
			//FTS_DBG("tpd fts_read_Gestruedata state=%d\n",state);
		     	if(state ==1)
		     	{
			        fts_read_Gestruedata();
			        continue;
		    	}
		#endif
#if TPD_PROXIMITY
				 if (tpd_proximity_flag == 1)
				 {
				 	 fts_read_reg(i2c_client, 0xB0, &state1);
					 TPD_PROXIMITY_DEBUG("proxi_5206 0xB0 state value is 1131 0x%02X\n", state1);

				 	 fts_read_reg(i2c_client, 0x01, &proximity_status);
					 TPD_PROXIMITY_DEBUG("proxi_5206 0x01 value is 1139 0x%02X\n", proximity_status);
					 if (proximity_status == 0xC0)
					 {
						 tpd_proximity_detect = 0;
					 }
					 else if(proximity_status == 0xE0)
					 {
						 tpd_proximity_detect = 1;
					 }
					 TPD_PROXIMITY_DEBUG("tpd_proximity_detect 1149 = %d\n", tpd_proximity_detect);
					 if(tpd_proximity_detect != tpd_proximity_detect_prev)
					 {
					 	if((tpd_proximity_detect_prev == 0xff)&&(tpd_proximity_detect == 1))
					 		{
					 			
					 		}
					 	else
					 		{
					 			if(tpd_proximity_detect == 0)
					 			{
									input_report_key(fts_input_dev, KEY_SLEEP, 1);
									input_sync(fts_input_dev);
									input_report_key(fts_input_dev, KEY_SLEEP, 0);
									input_sync(fts_input_dev);
								}
					 			else
					 			{
									input_report_key(fts_input_dev, KEY_WAKEUP, 1);
									input_sync(fts_input_dev);
									input_report_key(fts_input_dev, KEY_WAKEUP, 0);
									input_sync(fts_input_dev);
								}	
					 		}
						tpd_proximity_detect_prev = tpd_proximity_detect;
					 }
				 }
#endif
		//TPD_DEBUG("touch_event_handler start\n");

		if (tpd_touchinfo(&cinfo, &pinfo)) {
			if (tpd_dts_data.use_tpd_button) {
				if (cinfo.p[0] == 0)
					memcpy(&finfo, &cinfo, sizeof(struct touch_info));
			}

			if ((cinfo.y[0] >= TPD_RES_Y) && (pinfo.y[0] < TPD_RES_Y)
			&& ((pinfo.p[0] == 0) || (pinfo.p[0] == 2))) {
				TPD_DEBUG("Dummy release --->\n");
				tpd_up(pinfo.x[0], pinfo.y[0],pinfo.id[0]);
				input_sync(tpd->dev);
				continue;
			}


			if (cinfo.count > 0) {
				for (i = 0; i < cinfo.count; i++)
					tpd_down(cinfo.x[i], cinfo.y[i], cinfo.id[i]);
			} else {
#ifdef TPD_SOLVE_CHARGING_ISSUE
				tpd_up(1, 48);
#else
				tpd_up(cinfo.x[0], cinfo.y[0],cinfo.id[0]);
#endif

			}
			input_sync(tpd->dev);

		}
	} while (!kthread_should_stop());

	TPD_DEBUG("touch_event_handler exit\n");

	return 0;
}

static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, TPD_DEVICE);

	return 0;
}

static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id)
{
	//TPD_DEBUG("TPD interrupt has been triggered\n");
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
}

static int tpd_irq_registration(void)
{  
	struct device_node *node = NULL;
	int ret = 0;
	
	//node = of_find_compatible_node(NULL, NULL, "mediatek,cap_touch");
	node = of_find_matching_node(node, touch_of_match);
	if (node) {
		/*touch_irq = gpio_to_irq(tpd_int_gpio_number);*/
		touch_irq = irq_of_parse_and_map(node, 0);
		ret = request_irq(touch_irq, tpd_eint_interrupt_handler,
					IRQF_TRIGGER_NONE, TPD_DEVICE, NULL);
			if (ret > 0)
				TPD_DMESG("tpd request_irq IRQ LINE NOT AVAILABLE!.");
	} else {
		TPD_DMESG("[%s] tpd request_irq can not find touch eint device node!.", __func__);
	}
	return 0;
	
}

#if MTK_CTP_NODE 
#define CTP_PROC_FILE "tp_info"

static int ctp_proc_read_show (struct seq_file* m, void* data)
{
	char temp[40] = {0};
	sprintf(temp, "[Vendor]%s,[Fw]%x,[IC]FT5446\n","O-Film",4); 
	seq_printf(m, "%s\n", temp);
	return 0;
}

static int ctp_proc_open (struct inode* inode, struct file* file) 
{
    return single_open(file, ctp_proc_read_show, inode->i_private);
}

static const struct file_operations g_ctp_proc = 
{
    .open = ctp_proc_open,
    .read = seq_read,
};
#endif

static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int retval = TPD_OK;
	u8 report_rate = 0;
	int reset_count = 0;
	char data;
	int err = 0;

	i2c_client = client;
	fts_i2c_client = client;
	fts_input_dev=tpd->dev;
	if(i2c_client->addr != 0x38)
	{
		i2c_client->addr = 0x38;
		FTS_DBG("frank_zhonghua:i2c_client_FT->addr=%d\n",i2c_client->addr);
	}

	of_get_ft5x0x_platform_data(&client->dev);
	/* configure the gpio pins */

	TPD_DMESG("mtk_tpd: tpd_probe ft5x0x\n");


	retval = regulator_enable(tpd->reg);
	if (retval != 0)
		TPD_DMESG("Failed to enable reg-vgp6: %d\n", retval);

	/* set INT mode */

	tpd_gpio_as_int(tpd_int_gpio_number);

	msg_dma_alloct();

#ifdef CONFIG_FT_AUTO_UPGRADE_SUPPORT

    if (NULL == tpd_i2c_dma_va)
    {
        tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
        tpd_i2c_dma_va = (u8 *)dma_alloc_coherent(&tpd->dev->dev, 250, &tpd_i2c_dma_pa, GFP_KERNEL);
    }
    if (!tpd_i2c_dma_va)
		TPD_DMESG("TPD dma_alloc_coherent error!\n");
	else
		TPD_DMESG("TPD dma_alloc_coherent success!\n");
#endif

#if FTS_GESTRUE_EN
	fts_Gesture_init(tpd->dev);
#endif

reset_proc:
	/* Reset CTP */
	tpd_gpio_output(tpd_rst_gpio_number, 0);
	msleep(20);
	tpd_gpio_output(tpd_rst_gpio_number, 1);
	msleep(400);
	err = fts_read_reg(i2c_client, 0x00, &data);

	//TPD_DMESG("fts_i2c:err %d,data:%d\n", err,data);
	if(err< 0 || data!=0)// reg0 data running state is 0; other state is not 0
	{
		TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
#ifdef TPD_RESET_ISSUE_WORKAROUND
		if ( ++reset_count < TPD_MAX_RESET_COUNT )
		{
			goto reset_proc;
		}
#endif
		retval	= regulator_disable(tpd->reg); //disable regulator
		if(retval)
		{
			FTS_DBG("focaltech tpd_probe regulator_disable() failed!\n");
		}

		regulator_put(tpd->reg);
		msg_dma_release();
		gpio_free(tpd_rst_gpio_number);
		gpio_free(tpd_int_gpio_number);
		return -1;
	}

	tpd_irq_registration();
	msleep(100);
	tpd_load_status = 1;
/*#ifdef CONFIG_CUST_FTS_APK_DEBUG
	//ft_rw_iic_drv_init(client);

	//ft5x0x_create_sysfs(client);

	ft5x0x_create_apk_debug_channel(client);
#endif
*/   
    //device_create_file(tpd->tpd_dev, &tp_attr_foo); 
	#ifdef  MTK_CTP_NODE
    if((proc_create(CTP_PROC_FILE,0444,NULL,&g_ctp_proc))==NULL)
    {
      FTS_DBG("proc_create tp vertion node error\n");
	}
	#endif
	#ifdef  MTK_CTP_NODE
	create_ctp_node();
    #endif
	
	//touch_class = class_create(THIS_MODULE,"FT5446");
	#ifdef SYSFS_DEBUG
                fts_create_sysfs(fts_i2c_client);
	#endif
	//hidi2c_to_stdi2c(fts_i2c_client);
	fts_get_upgrade_array();
	#ifdef FTS_CTL_IIC
		 if (fts_rw_iic_drv_init(fts_i2c_client) < 0)
			 dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n", __func__);
	#endif

	#ifdef FTS_APK_DEBUG
		fts_create_apk_debug_channel(fts_i2c_client);
	#endif

	/* Set report rate 80Hz */
	report_rate = 0x8;
	if ((fts_write_reg(i2c_client, 0x88, report_rate)) < 0) {
		if ((fts_write_reg(i2c_client, 0x88, report_rate)) < 0)
			TPD_DMESG("I2C write report rate error, line: %d\n", __LINE__);
	}

	/* tpd_load_status = 1; */

	thread_tpd = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread_tpd)) {
		retval = PTR_ERR(thread_tpd);
		TPD_DMESG(TPD_DEVICE " failed to create kernel thread_tpd: %d\n", retval);
	}

	TPD_DMESG("Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");

#ifdef TIMER_DEBUG
	init_test_timer();
#endif

	{
		u8 ver;

		fts_read_reg(client, 0xA6, &ver);
		ctp_fw_version = ver;

		TPD_DMESG(TPD_DEVICE " fts_read_reg version : %d\n", ver);
        fts_read_reg(client, 0xA8, &ver);

		if(ver==0x51)
		{
         vendor_name="O-Film";
		}
		else
		{
         vendor_name="Rserve";
		}
		//FTS_DBG("vendor_name=%s fwvertion=%x\n",vendor_name,fwvertion);
	}

#if TPD_PROXIMITY
	proc_create("ft_prox_get_status", 0, NULL, &ft_prox_get_proc);
	proc_create("ft_prox_turnon", 0, NULL, &ft_prox_on_proc);
	proc_create("ft_prox_turnoff", 0, NULL, &ft_prox_off_proc);
	__set_bit(KEY_SLEEP, fts_input_dev->keybit);
	__set_bit(KEY_WAKEUP, fts_input_dev->keybit);
#endif

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
	int ret;

	ret = get_md32_semaphore(SEMAPHORE_TOUCH);
	if (ret < 0)
		pr_err("[TOUCH] HW semaphore reqiure timeout\n");
#endif

	wake_lock_init(&ft_wake_lock, WAKE_LOCK_SUSPEND, "ft6x0x wakelock");
	return 0;
}

static int tpd_remove(struct i2c_client *client)
{
	TPD_DEBUG("TPD removed\n");
#ifdef CONFIG_CUST_FTS_APK_DEBUG
	//ft_rw_iic_drv_exit();
#endif

#ifdef CONFIG_FT_AUTO_UPGRADE_SUPPORT
	if (tpd_i2c_dma_va) {
		dma_free_coherent(NULL, 4096, tpd_i2c_dma_va, tpd_i2c_dma_pa);
		tpd_i2c_dma_va = NULL;
		tpd_i2c_dma_pa = 0;
	}
#endif
	gpio_free(tpd_rst_gpio_number);
	gpio_free(tpd_int_gpio_number);

	return 0;
}

static int tpd_local_init(void)
{
	int retval;

	TPD_DMESG("Focaltech FT5x0x I2C Touchscreen Driver...\n");
	tpd->reg = regulator_get(tpd->tpd_dev, "vldo28");
	retval = regulator_set_voltage(tpd->reg, 2800000, 2800000);
	if (retval != 0) {
		TPD_DMESG("Failed to set reg-vgp6 voltage: %d\n", retval);
		return -1;
	}

	if (i2c_add_driver(&tpd_i2c_driver) != 0) {
		TPD_DMESG("unable to add i2c driver.\n");
		return -1;
	}
     /* tpd_load_status = 1; */
	if (tpd_dts_data.use_tpd_button) {
		tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local,tpd_dts_data.tpd_key_dim_local);
	}

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
	TPD_DO_WARP = 1;
	memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
	memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif

#if (defined(CONFIG_TPD_HAVE_CALIBRATION) && !defined(CONFIG_TPD_CUSTOM_CALIBRATION))

	memcpy(tpd_calmat, tpd_def_calmat_local_factory, 8 * 4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local_factory, 8 * 4);

	memcpy(tpd_calmat, tpd_def_calmat_local_normal, 8 * 4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local_normal, 8 * 4);

#endif

	TPD_DMESG("end %s, %d\n", __func__, __LINE__);
	tpd_type_cap = 1;

	return 0;
}

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
static s8 ftp_enter_doze(struct i2c_client *client)
{
	s8 ret = -1;
	s8 retry = 0;
	char gestrue_on = 0x01;
	char gestrue_data;
	int i;

	/* TPD_DEBUG("Entering doze mode..."); */
	pr_alert("Entering doze mode...");

	/* Enter gestrue recognition mode */
	ret = fts_write_reg(i2c_client, FT_GESTRUE_MODE_SWITCH_REG, gestrue_on);
	if (ret < 0) {
		/* TPD_DEBUG("Failed to enter Doze %d", retry); */
		pr_alert("Failed to enter Doze %d", retry);
		return ret;
	}
	msleep(30);

	for (i = 0; i < 10; i++) {
		fts_read_reg(i2c_client, FT_GESTRUE_MODE_SWITCH_REG, &gestrue_data);
		if (gestrue_data == 0x01) {
			doze_status = DOZE_ENABLED;
			/* TPD_DEBUG("FTP has been working in doze mode!"); */
			pr_alert("FTP has been working in doze mode!");
			break;
		}
		msleep(20);
		fts_write_reg(i2c_client, FT_GESTRUE_MODE_SWITCH_REG, gestrue_on);

	}

	return ret;
}
#endif

static void tpd_resume(struct device *h)
{
	int retval = TPD_OK;

	TPD_DEBUG("TPD wake up\n");

#if TPD_PROXIMITY
		if (tpd_proximity_suspend == 0)
		{
			return;
		}
		else
		{
			tpd_proximity_suspend = 0;
		}
#endif

	retval = regulator_enable(tpd->reg);
	if (retval != 0)
		TPD_DMESG("Failed to enable reg-vgp6: %d\n", retval);

	#if FTS_GESTRUE_EN
        fts_write_reg(i2c_client,0xD0,0x00);
        //fts_write_reg(i2c_client,0xD1,0x00);
       // fts_write_reg(i2c_client,0xD2,0x00);
	#endif

	tpd_gpio_output(tpd_rst_gpio_number, 0);
	msleep(20);
	tpd_gpio_output(tpd_rst_gpio_number, 1);
	msleep(200);



#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
	doze_status = DOZE_DISABLED;
	/* tpd_halt = 0; */
	int data;

	data = 0x00;

	fts_write_reg(i2c_client, FT_GESTRUE_MODE_SWITCH_REG, data);
#else
	enable_irq(touch_irq);
#endif
#if USB_CHARGE_DETECT
	if(FG_charging_status != 0)
	{
		charging_flag = 0;
	}
	else
	{
		charging_flag = 1;
	}
#endif

	focaltech_halt=0;

       tpd_up(0,0,0);
       input_sync(tpd->dev);
       TPD_DMESG("TPD wake up done\n");
}

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
void tpd_scp_wakeup_enable(bool en)
{
	tpd_scp_doze_en = en;
}

void tpd_enter_doze(void)
{

}
#endif

static void tpd_suspend(struct device *h)
{
	int retval = TPD_OK;
	static char data = 0x3;
	#if FTS_GESTRUE_EN
	int i = 0;
	u8 state = 0;
	#endif
	FTS_DBG("TPD enter sleep\n");
#if TPD_PROXIMITY
		if (tpd_proximity_flag == 1)
		{
			tpd_proximity_suspend = 0;
			return;
		}
		else
		{
			tpd_proximity_suspend = 1;
		}
#endif

	focaltech_halt=1;
	#if FTS_GESTRUE_EN
    {
	//disable_irq(touch_irq);
	msleep(100); 		
	if (fts_updateinfo_curr.CHIP_ID == 0x64)
		{
			fts_write_reg(i2c_client, 0xd0, 0x01);
			fts_write_reg(i2c_client, 0xd1, 0xff);
			fts_write_reg(i2c_client, 0xd2, 0xff);
			fts_write_reg(i2c_client, 0xd5, 0xff);
			fts_write_reg(i2c_client, 0xd6, 0xff);
			fts_write_reg(i2c_client, 0xd7, 0xff);
			fts_write_reg(i2c_client, 0xd8, 0xff);
		
		//	enable_irq(touch_irq);
			for(i = 0; i < 10; i++)
			{
				FTS_DBG("tpd_suspend4 %d",i);
			  	fts_read_reg(i2c_client, 0xd0, &state);

				if(state == 1)
				{
					TPD_DMESG("TPD gesture write 0x01\n");
						return;				
				}
				else
				{
					fts_write_reg(i2c_client, 0xd0, 0x01);
					fts_write_reg(i2c_client, 0xd1, 0xff);
		 			fts_write_reg(i2c_client, 0xd2, 0xff);
				    fts_write_reg(i2c_client, 0xd5, 0xff);
					fts_write_reg(i2c_client, 0xd6, 0xff);
					fts_write_reg(i2c_client, 0xd7, 0xff);
				  	fts_write_reg(i2c_client, 0xd8, 0xff);
        //	enable_irq(touch_irq);
					msleep(10);
				}
			}
			if(i >= 9)
			{
				TPD_DMESG("TPD gesture write 0x01 to d0 fail \n");
				return;
			}
    	}
	}
	#endif

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
	int sem_ret;
sdsd
	tpd_enter_doze();

	int ret;
	char gestrue_data;
	char gestrue_cmd = 0x03;
	static int scp_init_flag;

	/* TPD_DEBUG("[tpd_scp_doze]:init=%d en=%d", scp_init_flag, tpd_scp_doze_en); */

	mutex_lock(&i2c_access);

	sem_ret = release_md32_semaphore(SEMAPHORE_TOUCH);

	if (scp_init_flag == 0) {
		Touch_IPI_Packet ipi_pkt;

		ipi_pkt.cmd = IPI_COMMAND_AS_CUST_PARAMETER;
		ipi_pkt.param.tcs.i2c_num = TPD_I2C_NUMBER;
		ipi_pkt.param.tcs.int_num = CUST_EINT_TOUCH_PANEL_NUM;
		ipi_pkt.param.tcs.io_int = tpd_int_gpio_number;
		ipi_pkt.param.tcs.io_rst = tpd_rst_gpio_number;

		TPD_DEBUG("[TOUCH]SEND CUST command :%d ", IPI_COMMAND_AS_CUST_PARAMETER);

		ret = md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 0);
		if (ret < 0)
			TPD_DEBUG(" IPI cmd failed (%d)\n", ipi_pkt.cmd);

		msleep(20); /* delay added between continuous command */
		/* Workaround if suffer MD32 reset */
		/* scp_init_flag = 1; */
	}

	if (tpd_scp_doze_en) {
		TPD_DEBUG("[TOUCH]SEND ENABLE GES command :%d ", IPI_COMMAND_AS_ENABLE_GESTURE);
		ret = ftp_enter_doze(i2c_client);
		if (ret < 0) {
			TPD_DEBUG("FTP Enter Doze mode failed\n");
	  } else {
			int retry = 5;
	    {
				/* check doze mode */
				fts_read_reg(i2c_client, FT_GESTRUE_MODE_SWITCH_REG, &gestrue_data);
				TPD_DEBUG("========================>0x%x", gestrue_data);
	    }

	    msleep(20);
			Touch_IPI_Packet ipi_pkt = {.cmd = IPI_COMMAND_AS_ENABLE_GESTURE, .param.data = 1};

			do {
				if (md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 1) == DONE)
					break;
				msleep(20);
				TPD_DEBUG("==>retry=%d", retry);
			} while (retry--);

	    if (retry <= 0)
				TPD_DEBUG("############# md32_ipi_send failed retry=%d", retry);

			/*while(release_md32_semaphore(SEMAPHORE_TOUCH) <= 0) {
				//TPD_DEBUG("GTP release md32 sem failed\n");
				pr_alert("GTP release md32 sem failed\n");
			}*/

		}
		/* disable_irq(touch_irq); */
	}

	mutex_unlock(&i2c_access);
#else
	disable_irq(touch_irq);
	fts_write_reg(i2c_client, 0xA5, data);  /* TP enter sleep mode */

	retval = regulator_disable(tpd->reg);
	if (retval != 0)
		TPD_DMESG("Failed to disable reg-vgp6: %d\n", retval);

#endif

}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = "FT5x0x",
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
	.attrs = {
		.attr = ft5x0x_attrs,
		.num  = ARRAY_SIZE(ft5x0x_attrs),
	},
};

/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
	TPD_DMESG("MediaTek FT5x0x touch panel driver init\n");
	tpd_get_dts_info();
	if (tpd_driver_add(&tpd_device_driver) < 0)
		TPD_DMESG("add FT5x0x driver failed\n");

	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
	TPD_DMESG("MediaTek FT5x0x touch panel driver exit\n");
	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

