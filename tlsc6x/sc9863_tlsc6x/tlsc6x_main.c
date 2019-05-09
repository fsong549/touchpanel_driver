/*
 * This software is licensed under the terms of the GNU General Public 
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms. 
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * GNU General Public License for more details. 
 * * VERSION      	DATE			AUTHOR          Note
 * 
 */

#if ! defined(LINUX_VERSION_CODE)  
#include <linux/version.h>
#endif
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
//#include <soc/sprd/regulator.h>
#include <linux/input/mt.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>

#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/suspend.h>
#include <linux/irq.h>

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0))
#include <linux/wakelock.h>
#endif
#include <linux/hrtimer.h>
//#if(defined(CONFIG_I2C_SPRD) ||defined(CONFIG_I2C_SPRD_V1))
//#include <soc/sprd/i2c-sprd.h>
//#endif
#if defined(CONFIG_ADF)
#include <linux/notifier.h>
#include <video/adf_notifier.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#include <linux/miscdevice.h>
//dingyanlei Add , MMI read TP version , 2015.09.18
#include "tlsc6x_main.h"

#ifdef TP_GESTRUE
#include <linux/fs.h>
#include <linux/uaccess.h>
#endif
#include <linux/proc_fs.h>

#define	TOUCH_VIRTUAL_KEYS
#define	MULTI_PROTOCOL_TYPE_B	0
#define	TS_MAX_FINGER		2

struct tlsc6x_platform_data{
	u32 irq_gpio_number;
	u32 reset_gpio_number;
	const char *vdd_name;
	u32 virtualkeys[12];
	u32 TP_MAX_X;
	u32 TP_MAX_Y;
};


static struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int tpd_flag = 0;

#ifdef TP_PROXIMITY_SENSOR
#define LTR_IOCTL_MAGIC 0x1C
#define LTR_IOCTL_GET_PFLAG _IOR(LTR_IOCTL_MAGIC, 1, int)
#define LTR_IOCTL_GET_LFLAG _IOR(LTR_IOCTL_MAGIC, 2, int)
#define LTR_IOCTL_SET_PFLAG _IOW(LTR_IOCTL_MAGIC, 3, int)
#define LTR_IOCTL_SET_LFLAG _IOW(LTR_IOCTL_MAGIC, 4, int)
#define LTR_IOCTL_GET_DATA _IOW(LTR_IOCTL_MAGIC, 5, unsigned char)

static int PROXIMITY_SWITCH=0;
static int PROXIMITY_STATE=0;

static struct spinlock proximity_switch_lock;
static struct spinlock proximity_state_lock;
#endif

#define TS_NAME	   	       	"tlsc6x_ts"
unsigned char tlsc6x_chip_name[6][20]={"null", "tlsc6206a", "0x6306", "tlsc6206", "tlsc6324","tlsc6332"};

int g_is_telink_comp = 0;

static struct tlsc6x_data *g_tp_drvdata = NULL;
static struct i2c_client *this_client;

#ifdef TP_GESTRUE
static unsigned char gesture_enable = 0;
static unsigned char gesture_state = 0;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0))
static struct wake_lock gesture_timeout_wakelock;
#endif
#endif
			
#ifdef TP_GESTRUE
#define GESTURE_LEFT		0x20
#define GESTURE_RIGHT		0x21
#define GESTURE_UP		    0x22
#define GESTURE_DOWN		0x23
#define GESTURE_DOUBLECLICK	0x24
#define GESTURE_O		    0x30
#define GESTURE_W		    0x31
#define GESTURE_M		    0x32
#define GESTURE_E		    0x33
#define GESTURE_C		    0x34
#define GESTURE_S           0x46
#define GESTURE_V           0x54
#define GESTURE_Z           0x65
#define GESTURE_L           0x44

#endif

#if defined(CONFIG_ADF)
static int tlsc6x_suspend(void);
static int tlsc6x_resume(void);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void tlsc6x_ts_suspend(struct early_suspend *handler);
static void tlsc6x_ts_resume(struct early_suspend *handler);
#endif

struct ts_event {
	u16	x1;
	u16	y1;
	u16	x2;
	u16	y2;
	u16	x3;
	u16	y3;
	u16	x4;
	u16	y4;
	u16	x5;
	u16	y5;
	u16	pressure;
    u8  touch_point;
};

struct tlsc6x_data {
    struct input_dev *input_dev;
    struct input_dev *ps_input_dev;
    struct i2c_client	*client;
    struct ts_event	event;
#if defined(CONFIG_ADF)
    struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend	early_suspend;
#endif
	struct work_struct       resume_work;
    struct workqueue_struct *tp_resume_workqueue;
    int irq_gpio_number;
    int reset_gpio_number;
    int isVddAlone;
    struct regulator *reg_vdd;
    struct tlsc6x_platform_data	*platform_data;
};

#ifdef TOUCH_VIRTUAL_KEYS

static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct tlsc6x_data *data = i2c_get_clientdata(this_client);
	struct tlsc6x_platform_data *pdata = data->platform_data;
	return sprintf(buf,"%s:%s:%d:%d:%d:%d:%s:%s:%d:%d:%d:%d:%s:%s:%d:%d:%d:%d\n"
		,__stringify(EV_KEY), __stringify(KEY_APPSELECT),pdata ->virtualkeys[0],pdata ->virtualkeys[1],pdata ->virtualkeys[2],pdata ->virtualkeys[3]
		,__stringify(EV_KEY), __stringify(KEY_HOMEPAGE),pdata ->virtualkeys[4],pdata ->virtualkeys[5],pdata ->virtualkeys[6],pdata ->virtualkeys[7]
		,__stringify(EV_KEY), __stringify(KEY_BACK),pdata ->virtualkeys[8],pdata ->virtualkeys[9],pdata ->virtualkeys[10],pdata ->virtualkeys[11]);
}

static struct kobj_attribute virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys.tlsc6x_ts",
        .mode = S_IRUGO,
    },
    .show = &virtual_keys_show,
};

int tlsc6x_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
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
                .flags = 1,
                .len = readlen,
                .buf = readbuf,
            },
        };
        ret = i2c_transfer(client->adapter, msgs, 2);
        if (ret < 0){
            dev_err(&client->dev, "tlsc6x error:: i2c read error.\n");
        }
    } else {
        struct i2c_msg msgs[] = {
            {
                .addr = client->addr,
                .flags = 1,
                .len = readlen,
                .buf = readbuf,
                },
            };
            ret = i2c_transfer(client->adapter, msgs, 1);
            if (ret < 0){
                dev_err(&client->dev, "tlsc6x error:: i2c read error.\n");
            }
    }

    return ret;
}

int tlsc6x_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
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
    if (ret < 0){
         dev_err(&client->dev, "tlsc6x error:: i2c write error.\n");
    }

    return ret;

}

int tlsc6x_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
    unsigned char buf[2] = {0};
    buf[0] = regaddr;
    buf[1] = regvalue;

    return tlsc6x_i2c_Write(client, buf, sizeof(buf));
}


int tlsc6x_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
    return tlsc6x_i2c_Read(client, &regaddr, 1, regvalue, 1);
}
static struct attribute *properties_attrs[] = {
    &virtual_keys_attr.attr,
    NULL
};

static struct attribute_group properties_attr_group = {
    .attrs = properties_attrs,
};

static void tlsc6x_virtual_keys_init(void)
{
    int ret = 0;
    struct kobject *properties_kobj;

    pr_info("%s\n",__func__);

    properties_kobj = kobject_create_and_add("board_properties", NULL);
    if(properties_kobj){
        ret = sysfs_create_group(properties_kobj, &properties_attr_group);
    }
    if(!properties_kobj || ret){
        pr_err("failed to create board_properties\n");
    }
}

#endif




static void tlsc6x_clear_report_data(struct tlsc6x_data *drvdata)
{
    int i;

    for(i = 0; i<TS_MAX_FINGER; i++){
    #if MULTI_PROTOCOL_TYPE_B
        input_mt_slot(drvdata->input_dev, i);
        input_mt_report_slot_state(drvdata->input_dev, MT_TOOL_FINGER, false);
    #endif
    }

    input_report_key(drvdata->input_dev, BTN_TOUCH, 0);
    #if !MULTI_PROTOCOL_TYPE_B
    input_mt_sync(drvdata->input_dev);
    #endif
    input_sync(drvdata->input_dev);
}

static int tlsc6x_update_data(void)
{
    struct tlsc6x_data *data = i2c_get_clientdata(this_client);
    struct ts_event *event = &data->event;
    u8 buf[20] = {0};
    int ret = -1;
    int i;
    u16 x , y;
    u8 ft_pressure , ft_size;

    ret = tlsc6x_i2c_Read(this_client, buf, 1, buf, 18);
    if(ret < 0){
        pr_err("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
        return ret;
    }

    memset(event, 0, sizeof(struct ts_event));
    event->touch_point = buf[2] & 0x07;
#ifdef TP_PROXIMITY_SENSOR
    if(PROXIMITY_SWITCH){
        spin_lock(&proximity_state_lock);

        if (0xC0 == buf[1]){
            PROXIMITY_STATE = 1; // 1;  near
        }else if (0xE0 == buf[1]){
            PROXIMITY_STATE = 0; // 0;  far-away
        }/*else{
            tlsc6x_write_reg(this_client,0xb0, 0x01);
        }*/
        spin_unlock(&proximity_state_lock);
        input_report_abs(data->ps_input_dev, ABS_DISTANCE, PROXIMITY_STATE?0:1);
        input_report_key(data->ps_input_dev, BTN_TOUCH, 0);
    #if !MULTI_PROTOCOL_TYPE_B
        input_mt_sync(data->ps_input_dev);
    #endif
        input_sync(data->ps_input_dev);
    }
#endif
    for(i = 0; i < TS_MAX_FINGER; i++){
        if((buf[6*i+3] & 0xc0) == 0xc0){
            continue;
        }
        x = (s16)(buf[6*i+3] & 0x0F)<<8 | (s16)buf[6*i+4];	
        y = (s16)(buf[6*i+5] & 0x0F)<<8 | (s16)buf[6*i+6];
        ft_pressure = buf[6*i+7];
        if(ft_pressure > 127){
            ft_pressure = 127;
        }
        ft_size = (buf[6*i+8]>>4) & 0x0F;
        if((buf[6*i+3] & 0x40) == 0x0){
            #if MULTI_PROTOCOL_TYPE_B
                input_mt_slot(data->input_dev, buf[6*i+5]>>4);
                input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);
            #endif
                input_report_abs(data->input_dev, ABS_MT_POSITION_X, x);
                input_report_abs(data->input_dev, ABS_MT_POSITION_Y, y);
                input_report_abs(data->input_dev, ABS_MT_PRESSURE, 15);
                input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, ft_size);
                input_report_key(data->input_dev, BTN_TOUCH, 1);
		#if !MULTI_PROTOCOL_TYPE_B
                input_mt_sync(data->input_dev);
		#endif
        }else{
		#if MULTI_PROTOCOL_TYPE_B
                input_mt_slot(data->input_dev, buf[6*i+5]>>4);
                input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
		#endif
        }
    }
    if(0 == event->touch_point) {		
        tlsc6x_clear_report_data(data);
    }
    input_sync(data->input_dev);

    return 0;

}

#ifdef TP_GESTRUE

static u32 _gGestureWakeupValue[2] = {0};


#define GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG     0x0001    //0000 0000 0000 0001
#define GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG        0x0002    //0000 0000 0000 0010
#define GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG      0x0004    //0000 0000 0000 0100
#define GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG      0x0008    //0000 0000 0000 1000
#define GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG     0x0010    //0000 0000 0001 0000
#define GESTURE_WAKEUP_MODE_m_CHARACTER_FLAG      0x0020    //0000 0000 0010 0000
#define GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG      0x0040    //0000 0000 0100 0000
#define GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG      0x0080    //0000 0000 1000 0000
#define GESTURE_WAKEUP_MODE_e_CHARACTER_FLAG      0x0100    //0000 0001 0000 0000
#define GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG      0x0200    //0000 0010 0000 0000
#define GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG      0x0400    //0000 0100 0000 0000
#define GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG      0x0800    //0000 1000 0000 0000
#define GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG      0x1000    //0001 0000 0000 0000
static int check_gesture(int gesture_id)
{	
    int keycode = 0;
    struct tlsc6x_data *data = i2c_get_clientdata(this_client);
 
    printk("kaka gesture_id==0x%x\n ",gesture_id);
    switch(gesture_id){
        case GESTURE_LEFT:
          //  keycode = KEY_LEFT;
		_gGestureWakeupValue[0] = GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG;
		
        input_report_key(data->input_dev, KEY_POWER, 1);
        input_sync(data->input_dev);
        input_report_key(data->input_dev, KEY_POWER, 0);
        input_sync(data->input_dev);
            break;
        case GESTURE_RIGHT:
         //   keycode = KEY_RIGHT;
         _gGestureWakeupValue[0] = GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG;
		 
        input_report_key(data->input_dev, KEY_POWER, 1);
        input_sync(data->input_dev);
        input_report_key(data->input_dev, KEY_POWER, 0);
        input_sync(data->input_dev);
            break;
        case GESTURE_UP:
          //  keycode = KEY_UP;
          _gGestureWakeupValue[0] = GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG;
		  
        input_report_key(data->input_dev, KEY_POWER, 1);
        input_sync(data->input_dev);
        input_report_key(data->input_dev, KEY_POWER, 0);
        input_sync(data->input_dev);
            break;
        case GESTURE_DOWN:
          //  keycode = KEY_DOWN;
          _gGestureWakeupValue[0] = GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG;
		  
        input_report_key(data->input_dev, KEY_POWER, 1);
        input_sync(data->input_dev);
        input_report_key(data->input_dev, KEY_POWER, 0);
        input_sync(data->input_dev);
            break;
        case GESTURE_DOUBLECLICK:
        //    keycode = KEY_D;    //KEY_POWER;//
        _gGestureWakeupValue[0] = GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG;
		
        input_report_key(data->input_dev, KEY_POWER, 1);
        input_sync(data->input_dev);
        input_report_key(data->input_dev, KEY_POWER, 0);
        input_sync(data->input_dev);
            break;
        case GESTURE_O:
        //    keycode = KEY_O;
        _gGestureWakeupValue[0] = GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG;
		
        input_report_key(data->input_dev, KEY_POWER, 1);
        input_sync(data->input_dev);
        input_report_key(data->input_dev, KEY_POWER, 0);
        input_sync(data->input_dev);
            break;
        case GESTURE_W:
        //    keycode = KEY_W;
        _gGestureWakeupValue[0] = GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG;
		
        input_report_key(data->input_dev, KEY_POWER, 1);
        input_sync(data->input_dev);
        input_report_key(data->input_dev, KEY_POWER, 0);
        input_sync(data->input_dev);
            break;
        case GESTURE_M:
       //     keycode = KEY_M;
       _gGestureWakeupValue[0] = GESTURE_WAKEUP_MODE_m_CHARACTER_FLAG;
	   
        input_report_key(data->input_dev, KEY_POWER, 1);
        input_sync(data->input_dev);
        input_report_key(data->input_dev, KEY_POWER, 0);
        input_sync(data->input_dev);
            break;
        case GESTURE_E:
       //     keycode = KEY_E;
       _gGestureWakeupValue[0] = GESTURE_WAKEUP_MODE_e_CHARACTER_FLAG;
	   
        input_report_key(data->input_dev, KEY_POWER, 1);
        input_sync(data->input_dev);
        input_report_key(data->input_dev, KEY_POWER, 0);
        input_sync(data->input_dev);
            break;
        case GESTURE_C:
      //      keycode = KEY_C;
      _gGestureWakeupValue[0] = GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG;

        input_report_key(data->input_dev, KEY_POWER, 1);
        input_sync(data->input_dev);
        input_report_key(data->input_dev, KEY_POWER, 0);
        input_sync(data->input_dev);
            break;
        case GESTURE_S:
     //       keycode = KEY_S;
     _gGestureWakeupValue[0] = GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG;
	 
        input_report_key(data->input_dev, KEY_POWER, 1);
        input_sync(data->input_dev);
        input_report_key(data->input_dev, KEY_POWER, 0);
        input_sync(data->input_dev);
            break;
         case GESTURE_V:
     //       keycode = KEY_V;
     _gGestureWakeupValue[0] = GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG;
	 
        input_report_key(data->input_dev, KEY_POWER, 1);
        input_sync(data->input_dev);
        input_report_key(data->input_dev, KEY_POWER, 0);
        input_sync(data->input_dev);
            break;
        case GESTURE_Z:
    //        keycode = KEY_UP;
    _gGestureWakeupValue[0] = GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG;
	
        input_report_key(data->input_dev, KEY_POWER, 1);
        input_sync(data->input_dev);
        input_report_key(data->input_dev, KEY_POWER, 0);
        input_sync(data->input_dev);
            break;
    //    case GESTURE_L:
    //        keycode = KEY_L;
   // _gGestureWakeupValue[0] = GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG;
       //     break;
        default:
            break;
    }
    return keycode;
}
static int tlsc6x_read_Gestruedata(void)
{
    int ret = -1;
    int gestrue_id = 0;
    u8 buf[4] = {0xd3, 0xd3};
    
    ret = tlsc6x_i2c_Read(this_client, buf, 1, buf, 2);
    if(ret < 0){
        pr_err("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
        return ret;
    }
    if(buf[1] != 0){
        gestrue_id = 0x24;
    }else{
        gestrue_id = buf[0];
    }
    check_gesture(gestrue_id);;
    return 0;
}
#endif

#ifdef TLSC_ESD_HELPER_EN
static int tpd_esd_flag = 0;
static struct hrtimer tpd_esd_kthread_timer;
static DECLARE_WAIT_QUEUE_HEAD(tpd_esd_waiter);
#endif

static int touch_event_handler(void *unused)
{
    //struct sched_param param = { .sched_priority = 5 };

    //sched_setscheduler(current, SCHED_RR, &param);

    do{
        set_current_state(TASK_INTERRUPTIBLE);
        wait_event_interruptible(waiter, (0 != tpd_flag));
        tpd_flag = 0;
        set_current_state(TASK_RUNNING);
#ifdef TP_GESTRUE
        if(gesture_enable && gesture_state){
        #if (LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0))
            wake_lock_timeout(&gesture_timeout_wakelock, msecs_to_jiffies(2000));
        #endif 
        msleep(50);//wait for stable
            tlsc6x_read_Gestruedata();
            continue;
        }
#endif
        tlsc6x_update_data();

    } while (!kthread_should_stop());

    return 0;
}

static irqreturn_t tlsc6x_interrupt(int irq, void *dev_id)
{
    tpd_flag = 1;
    wake_up_interruptible(&waiter);
    return IRQ_HANDLED;
}

static void tlsc6x_tpd_reset(void)
{
    struct tlsc6x_platform_data *pdata = g_tp_drvdata->platform_data;

    gpio_direction_output(pdata->reset_gpio_number, 1);
    msleep(1);
    gpio_set_value(pdata->reset_gpio_number, 0);
    msleep(20);
    gpio_set_value(pdata->reset_gpio_number, 1);
    msleep(30);
}

static unsigned char suspend_flags = 0;

#if defined(CONFIG_ADF)
static int tlsc6x_suspend(void)
{
	int ret = -1;

    pr_info("==%s==\n", __FUNCTION__);

#ifdef TLSC_ESD_HELPER_EN
    hrtimer_cancel(&tpd_esd_kthread_timer);
#endif

#ifdef TP_PROXIMITY_SENSOR
    if(PROXIMITY_SWITCH){
        suspend_flags = 0;
        return 0;
    }
#endif

#ifdef TP_GESTRUE
    if(gesture_enable == 1){
        gesture_state = 0x01;
        suspend_flags =1;
        disable_irq_nosync(this_client->irq);
        tlsc6x_load_gesture_binlib();
        
        
        irq_set_irq_type(this_client->irq,IRQF_TRIGGER_HIGH | IRQF_NO_SUSPEND | IRQF_ONESHOT);
        //irq_set_irq_type(this_client->irq,IRQF_TRIGGER_LOW | IRQF_NO_SUSPEND | IRQF_ONESHOT);
        enable_irq(this_client->irq);
        if(enable_irq_wake(this_client->irq)){
            tlsc6x_write_reg(this_client, 0x00, 0x03);
            tlsc6x_write_reg(this_client, 0x00, 0x03);
        }else{
            tlsc6x_write_reg(this_client, 0x01, 0x03);
            tlsc6x_write_reg(this_client, 0x01, 0x03);
        }
        
        return 0;
    }
#endif

    disable_irq_nosync(this_client->irq);
    ret = tlsc6x_write_reg(this_client, 0xa5, 0x03);
    if(ret<0){
        printk("tlsc6x error::setup suspend fail!\n");
    }
    suspend_flags =1;
    tlsc6x_clear_report_data(g_tp_drvdata);
    return 0;
}
static int tlsc6x_resume(void)
{
	#ifdef TLSC_ESD_HELPER_EN
    hrtimer_start(&tpd_esd_kthread_timer,  ktime_set(3, 0), HRTIMER_MODE_REL);
#endif

#ifdef TP_PROXIMITY_SENSOR
    if(PROXIMITY_SWITCH && (suspend_flags == 0)){
        return 0;
    }
#endif

    queue_work(g_tp_drvdata->tp_resume_workqueue, &g_tp_drvdata->resume_work);
    return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void tlsc6x_ts_suspend(struct early_suspend *handler)
{
   int ret = -1;

    pr_info("==%s==\n", __FUNCTION__);

#ifdef TLSC_ESD_HELPER_EN
    hrtimer_cancel(&tpd_esd_kthread_timer);
#endif

#ifdef TP_PROXIMITY_SENSOR
    if(PROXIMITY_SWITCH){
        suspend_flags = 0;
        return;
    }
#endif

#ifdef TP_GESTRUE
    if(gesture_enable == 1){
        disable_irq_nosync(this_client->irq);
        tlsc6x_load_gesture_binlib();
        gesture_state = 0x01;
        enable_irq_wake(this_client->irq);
        irq_set_irq_type(this_client->irq,IRQF_TRIGGER_LOW | IRQF_NO_SUSPEND | IRQF_ONESHOT);
        enable_irq(this_client->irq);
        suspend_flags =1;
        return;
    }
#endif

    disable_irq_nosync(this_client->irq);
    ret = tlsc6x_write_reg(this_client, 0xa5, 0x03);
    if(ret<0){
        printk("tlsc6x error::setup suspend fail!\n");
    }
    suspend_flags =1;
    tlsc6x_clear_report_data(g_tp_drvdata);

return;
}

static void tlsc6x_ts_resume(struct early_suspend *handler)
{
#ifdef TLSC_ESD_HELPER_EN
    hrtimer_start(&tpd_esd_kthread_timer,  ktime_set(3, 0), HRTIMER_MODE_REL);
#endif

#ifdef TP_PROXIMITY_SENSOR
    if(PROXIMITY_SWITCH && (suspend_flags == 0)){
        return;
    }
#endif

    queue_work(g_tp_drvdata->tp_resume_workqueue, &g_tp_drvdata->resume_work);

}

#endif

static void tlsc6x_resume_work(struct work_struct *work)
{
    pr_info("==%s==\n", __FUNCTION__);

#ifdef TP_GESTRUE
    if(gesture_enable == 1){
        disable_irq_wake(this_client->irq);
        disable_irq_nosync(this_client->irq);
        irq_set_irq_type(this_client->irq, IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND);
        gesture_state = 0;
        tlsc6x_write_reg(this_client,0xD0,0x00);
    }
#endif


    tlsc6x_tpd_reset();

#ifdef TP_PROXIMITY_SENSOR
    if(PROXIMITY_SWITCH){
        msleep(100);//wait for stable
        tlsc6x_write_reg(this_client,0xb0, 0x01);
    }
#endif

    tlsc6x_clear_report_data(g_tp_drvdata);
	
    enable_irq(this_client->irq);

    suspend_flags = 0;
}


#if defined(CONFIG_ADF)
/*
 * touchscreen's suspend and resume state should rely on screen state,
 * as fb_notifier and early_suspend are all disabled on our platform,
 * we can only use adf_event now
 */
static int ts_adf_event_handler(struct notifier_block *nb, unsigned long action, void *data)
{

	struct adf_notifier_event *event = data;
	int adf_event_data;

	if (action != ADF_EVENT_BLANK)
		return NOTIFY_DONE;

	adf_event_data = *(int *)event->data;
	pr_info("receive adf event with adf_event_data=%d", adf_event_data);

	switch (adf_event_data) {
	case DRM_MODE_DPMS_ON:
		tlsc6x_resume();
		break;
	case DRM_MODE_DPMS_OFF:
		tlsc6x_suspend();
		break;
	default:
		pr_info("receive adf event with error data, adf_event_data=%d",
			adf_event_data);
		break;
	}

	return NOTIFY_OK;
}
#endif

static int tlsc6x_hw_init(struct tlsc6x_data *drvdata)
{
    struct regulator *reg_vdd;
    struct i2c_client *client = drvdata->client;
    struct tlsc6x_platform_data *pdata = drvdata->platform_data;
    int    ret;

    pr_info("%s [irq=%d];[rst=%d]\n",__func__,
		pdata->irq_gpio_number,pdata->reset_gpio_number);
    ret = gpio_request(pdata->irq_gpio_number, NULL);
    if(ret < 0){
        goto OUT;
    }
    ret = gpio_request(pdata->reset_gpio_number, NULL);
    if(ret < 0){
        goto OUT;
    }
    gpio_direction_output(pdata->reset_gpio_number, 1);
    gpio_direction_input(pdata->irq_gpio_number);


    //we got vdd_name in tlsc6x_parse_dt()
    reg_vdd = regulator_get(&client->dev, pdata->vdd_name);
    if (!WARN(IS_ERR(reg_vdd), "tlsc6x_hw_init regulator: failed to get %s.\n", pdata->vdd_name)) {
        //regulator_set_voltage(reg_vdd, 2800000, 2800000);
        //regulator_enable(reg_vdd);
        drvdata->reg_vdd = reg_vdd;
    }else{
        drvdata->reg_vdd = NULL;
    }

    ////msleep(100);    // add for power up sequence
    tlsc6x_tpd_reset();
    return 0;
OUT:

    return ret;
}


#ifdef CONFIG_OF
static struct tlsc6x_platform_data *tlsc6x_parse_dt(struct device *dev)
{
    struct tlsc6x_platform_data *pdata;
    struct device_node *np = dev->of_node;
    int ret;

    pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
    if (!pdata) {
        dev_err(dev, "Could not allocate struct tlsc6x_platform_data");
        return NULL;
    }
    pdata->reset_gpio_number = of_get_gpio(np, 0);
    if(pdata->reset_gpio_number < 0){
        dev_err(dev, "fail to get reset_gpio_number\n");
        goto fail;
    }
    pdata->irq_gpio_number = of_get_gpio(np, 1);
    if(pdata->reset_gpio_number < 0){
        dev_err(dev, "fail to get reset_gpio_number\n");
        goto fail;
    }
    ret = of_property_read_string(np, "vdd_name", &pdata->vdd_name);
    if(ret){
        dev_err(dev, "fail to get vdd_name\n");
        goto fail;
    }
    ret = of_property_read_u32_array(np, "virtualkeys", pdata->virtualkeys,12);
    if(ret){
        dev_err(dev, "fail to get virtualkeys\n");
        goto fail;
    }
    ret = of_property_read_u32(np, "TP_MAX_X", &pdata->TP_MAX_X);
    if(ret){
        dev_err(dev, "fail to get TP_MAX_X\n");
        goto fail;
    }
    ret = of_property_read_u32(np, "TP_MAX_Y", &pdata->TP_MAX_Y);
    if(ret){
        dev_err(dev, "fail to get TP_MAX_Y\n");
        goto fail;
    }

    return pdata;
fail:
    kfree(pdata);
    return NULL;
}
#endif

#ifdef TP_GESTRUE
static ssize_t tlsc6x_gesture_write(struct file *filp, const char __user * buff, size_t len, loff_t * off)
{
    s32 ret = 0;
    unsigned char temp;

    ret = copy_from_user(&temp, buff, 1);
    if (ret) {
        return -EPERM;
    }
    gesture_enable = temp == '1'?1:0;
	
    return len;
}

static const struct file_operations gesture_fops = {
    .owner = THIS_MODULE,
    .write = tlsc6x_gesture_write,
};
#endif

#if (defined TPD_AUTO_UPGRADE_PATH) || (defined TLSC_APK_DEBUG)

int auto_upd_busy = 0;
//0:sucess
//1: no file OR open fail
//2: wrong file size OR read error
//-1:op-fial
int tlsc6x_proc_cfg_update(u8 *dir, int behave)
{
    int ret = 1;
    u8 buf[256];
    u32 fileSize;
    loff_t pos = 0;
    u8 regval;
    mm_segment_t old_fs;
    static struct file *file = NULL;

    printk("tlsc6x proc-file:%s\n", dir);

    file = filp_open(dir, O_RDONLY, 0);
    if(IS_ERR(file)){
        printk("tlsc6x proc-file:open error!\n");
    }else{
        ret = 2;
        old_fs = get_fs();
        set_fs(KERNEL_DS);
        fileSize = file->f_op->llseek(file, 0, SEEK_END);
        printk("tlsc6x proc-file, size:%d\n", fileSize);
        if(204 == fileSize){
            tlsc6x_read_reg(this_client, 0x11,&regval);
            file->f_op->llseek(file, 0, SEEK_SET);
            if(204 == file->f_op->read(file, (char *)buf, 204, &file->f_pos)){
                printk("tlsc6x proc-file, read ok1!\n");
                tlsc6x_read_reg(this_client, 0x12,&regval);
                ret = 3;
            }else if(204 == vfs_read(file, buf, 204, &pos)){
                printk("tlsc6x proc-file, read ok2!\n");
                tlsc6x_read_reg(this_client, 0x13,&regval);
                ret = 3;
            }
            if(3 == ret){
                auto_upd_busy = 1;
                disable_irq(this_client->irq);
                msleep(1500);
                tlsc6x_read_reg(this_client, 0x14,&regval);
                ////wake_lock_timeout(&tlsc6x_wakelock, msecs_to_jiffies(2000));
                if(0 == behave){
                    ret = tlsx6x_update_running_cfg((u16*)buf);
                }else{
                    ret = tlsx6x_update_burn_cfg((u16*)buf);
                    tlsc6x_tpd_reset();
                }
                enable_irq(this_client->irq);
                auto_upd_busy = 0;
            }else{
                printk("tlsc6x proc-file, read error!\n");
            }
        }
        filp_close(file, NULL);
        set_fs(old_fs);
    }

    return ret;
}

#endif


#ifdef TLSC_APK_DEBUG    // MUST:(LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0))
unsigned char proc_out_len;
unsigned char proc_out_buf[256];
static struct proc_dir_entry *tlsc6x_proc_entry = NULL;
static ssize_t tlsc6x_proc_read(struct file *filp, char __user *page, size_t len, loff_t *pos)
{
    if(0 == proc_out_len){
        return -EFAULT;
    }

    if(copy_to_user(page,proc_out_buf,proc_out_len)){
        return -EFAULT;
    }

    return proc_out_len;
}

static ssize_t tlsc6x_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *ops)
{
    int buflen = len;
    unsigned char local_buf[256];

    if(buflen > 255){
        return -EFAULT;
    }
	
    if(copy_from_user(&local_buf, buff, buflen)){
        printk("%s:copy from user error\n", __func__);
        return -EFAULT;
    }

    // format:cmd+para+data0+data1+data2...
    switch(local_buf[0]){
    case 0: // cfg version
        proc_out_buf[0] = g_tlsc6x_cfg_ver;
        proc_out_buf[1] = g_tlsc6x_cfg_ver>>8;
        proc_out_buf[2] = g_tlsc6x_cfg_ver>>16;
        proc_out_buf[3] = g_tlsc6x_cfg_ver>>24;
        proc_out_len = 4;
        break;
    case 1:
        local_buf[buflen] = '\0';
        if(tlsc6x_proc_cfg_update(&local_buf[2], 0)){
            len = -EIO;
        }
        break;
    case 2:
        local_buf[buflen] = '\0';
        if(tlsc6x_proc_cfg_update(&local_buf[2], 1)){
            len = -EIO;
        }
        break;
    default:
        break;
    }

    return len;
}

static struct file_operations tlsc6x_proc_ops = {
    .owner = THIS_MODULE,
    .read = tlsc6x_proc_read,
    .write = tlsc6x_proc_write,
};
int tlsc6x_create_apk_debug_channel(struct i2c_client * client)
{
    tlsc6x_proc_entry = proc_create_data("tlsc6x-debug", S_IRUGO | S_IWUGO, NULL, &tlsc6x_proc_ops, (void *)client);
    if (NULL == tlsc6x_proc_entry) {
        dev_err(&client->dev, "tlsc6x error::create apk-debug entry!\n");
        return -ENOMEM;
    }
    return 0;
}

void tlsc6x_release_apk_debug_channel(void)
{
    if(tlsc6x_proc_entry){
        remove_proc_entry("tlsc6x-debug", NULL);
    }
}
#endif

#ifdef TLSC_ESD_HELPER_EN
static int esd_checker_handler(void *unused)
{
    u8 test_val;
    int ret = -1;
    ktime_t ktime;
    ////int retval = 0;

    do{
        wait_event_interruptible(tpd_esd_waiter, tpd_esd_flag != 0);
        tpd_esd_flag = 0;

        ktime = ktime_set(4, 0);
        hrtimer_start(&tpd_esd_kthread_timer, ktime, HRTIMER_MODE_REL);
#if (defined TPD_AUTO_UPGRADE_PATH) || (defined TLSC_APK_DEBUG)
        if(auto_upd_busy){
            continue;
        }
#endif	
        if(suspend_flags	){
            continue;
        }
        //mutex_lock(&i2c_access);
        ret = tlsc6x_read_reg(this_client, 0x0,&test_val);
        //mutex_unlock(&i2c_access);
        if(ret < 0){    //maybe confused by some noise,so retry is make sense.
            msleep(30);
            //mutex_lock(&i2c_access);
            ret = tlsc6x_read_reg(this_client, 0x0,&test_val);
            //mutex_unlock(&i2c_access);
            if(ret < 0){
                // re-power-on
                ////if(g_tp_drvdata->reg_vdd){
                ////    regulator_disable(g_tp_drvdata->reg_vdd);
                ////    msleep(50);
                ////    regulator_enable(g_tp_drvdata->reg_vdd);
                ////}
                tlsc6x_tpd_reset();
            }
        }
    }while(!kthread_should_stop());

    return 0;
}
	
enum hrtimer_restart tpd_esd_kthread_hrtimer_func(struct hrtimer *timer)
{
    tpd_esd_flag = 1;
    wake_up_interruptible(&tpd_esd_waiter);

    return HRTIMER_NORESTART;
}
#endif



#ifdef TP_PROXIMITY_SENSOR
static int TP_face_get_mode(void)
{
    return PROXIMITY_SWITCH;
}

static int TP_face_mode_state(void)
{
    return PROXIMITY_STATE;
}

static int TP_face_mode_switch(int on)
{
    spin_lock(&proximity_switch_lock);

    if(1 == on){
        PROXIMITY_SWITCH = 1;
        tlsc6x_write_reg(this_client,0xb0, 0x01);
    }else if(0 == on){
        PROXIMITY_SWITCH = 0;
        tlsc6x_write_reg(this_client,0xb0, 0x00);
    }else{
        spin_unlock(&proximity_switch_lock);
        return -EINVAL;
    }

//OUT:
    spin_unlock(&proximity_switch_lock);

    return 0;
}

static int tpd_ps_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int tpd_ps_release(struct inode *inode, struct file *file)
{
    return TP_face_mode_switch(0);
}

static long tpd_ps_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int flag;

    switch (cmd) {
        case LTR_IOCTL_SET_PFLAG:
            if (copy_from_user(&flag, argp, sizeof(flag))) {
                return -EFAULT;
            }
            if (flag < 0 || flag > 1) {
                return -EINVAL;
            }
            TP_face_mode_switch(flag);
            break;
        case LTR_IOCTL_GET_PFLAG:
            flag = PROXIMITY_STATE;
            if (copy_to_user(argp, &flag, sizeof(flag))) {
                return -EFAULT;
            }
            break;
	default:
            break;
    } 

    return 0;
}

static struct file_operations tpd_ps_fops = {
    .owner				= THIS_MODULE,
    .open				= tpd_ps_open,
    .release			= tpd_ps_release,
    .unlocked_ioctl		= tpd_ps_ioctl,
};
static struct miscdevice tpd_ps_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "tpd_proximity",
    .fops = &tpd_ps_fops,
};
#endif

static struct class *touchscreen_class;
struct device *tls6x_touchscreen_cmd_dev;

#ifdef TP_GESTRUE

ssize_t TLSC6XGestureWakeupValueShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    ssize_t value = 0;

    printk(" DrvMainGestureWakeupValueShow -> _gGestureWakeupValue[0] = %x, pBuf = %s\n", _gGestureWakeupValue[0], pBuf);
    value = (ssize_t)sprintf(pBuf, "%x", _gGestureWakeupValue[0]);
	_gGestureWakeupValue[0] = 0xFF;
	return value;
}

#if 0
ssize_t DrvMainGestureWakeupValueStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
   // u32 nLength, nWakeupMode;

   // DBG("*** %s() ***\n", __func__);

    if (pBuf != NULL)
    {
        sscanf(pBuf, "%x", &_gGestureWakeupValue[0]);
    }
	return 0;
}
#endif

static DEVICE_ATTR(gesture_wakeup_value, 0664, TLSC6XGestureWakeupValueShow, NULL);


ssize_t TLSC6XGestureWakeupModeShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
	ssize_t ret = 0;

	ret = (ssize_t)sprintf(pBuf, "%d", gesture_enable);

	return ret;
}

ssize_t TLSC6XGestureWakeupModeStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
	{
		if (strncmp(pBuf,"1", 1) == 0 )
		gesture_enable = 1;
	else if (strncmp(pBuf,"0",1) == 0 )
		gesture_enable = 0;

		return nSize;
	}


static DEVICE_ATTR(gesture_wakeup_mode, 0664, TLSC6XGestureWakeupModeShow, TLSC6XGestureWakeupModeStore);
#endif

static ssize_t TLSC6X_tp_version_show(struct device *dev,
	struct device_attribute *attr, char *buf)
	{
		int k;
		unsigned char tpver,csver;
		unsigned char business_name[20]="null";

		tpver = (g_tlsc6x_cfg_ver>>26)&0x3f;
		csver = (g_tlsc6x_cfg_ver>>9)&0x7f;
		if(0x03 == csver){
			strcpy(business_name, "yuye");	//
		}else if(0x26 == csver){
			strcpy(business_name, "zhenhai");	//
		}else if(0x18 == csver){
			strcpy(business_name, "yixing");	//
		}else{
			strcpy(business_name, "null");	//
		}

		k = (int)((g_tlsc6x_chip_code>>8)&0xf);
		if(k > 5){
			k = 0;
		}
		return sprintf(buf,__stringify(%s) ":" __stringify(%s) ":0x" __stringify(%02x) "\n", business_name, tlsc6x_chip_name[k], tpver);

	}

static DEVICE_ATTR(tp_version, 0444, TLSC6X_tp_version_show, NULL);

static int tlsc6x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err = 0;
#if defined(CONFIG_ADF)
	int ret;
#endif
    struct input_dev *input_dev;
    struct tlsc6x_platform_data *pdata = NULL;
#ifdef TP_PROXIMITY_SENSOR
    struct input_dev *ps_input_dev;
#endif

    #ifdef TP_GESTRUE
    struct proc_dir_entry *proc_entry = NULL;
    #endif

    pr_info("%s: probe !!!++\n",__func__);

    if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)){
        err = -ENODEV;
        goto exit_alloc_platform_data_failed;
    }

#ifdef CONFIG_OF    // NOTE:
    if(client->dev.of_node){
        pdata = tlsc6x_parse_dt(&client->dev);
        if(pdata){
            client->dev.platform_data = pdata;
        }
    }
#endif

    if(NULL == pdata){
        err = -ENOMEM;
        printk("%s: no platform data!!!\n",__func__);
        goto exit_alloc_platform_data_failed;
    }

    g_tp_drvdata = kzalloc(sizeof(*g_tp_drvdata), GFP_KERNEL);
    if(!g_tp_drvdata){
        err = -ENOMEM;
        goto exit_alloc_data_failed;
    }

    this_client = client;
    g_tp_drvdata->client = client;
    g_tp_drvdata->platform_data = pdata;

    err = tlsc6x_hw_init(g_tp_drvdata);
    if(err < 0){
        goto exit_gpio_request_failed;
    }

    i2c_set_clientdata(client, g_tp_drvdata);

    //#ifdef CONFIG_I2C_SPRD
    //sprd_i2c_ctl_chg_clk(client->adapter->nr, 400000);
    //#endif

    g_is_telink_comp = tlsc6x_tp_dect(client);
    if(g_is_telink_comp){
        #ifdef TLSC_FORCE_UPGRADE
        tlsc6x_data_crash_deal();
        tlsc6x_tpd_reset();
        #endif
        #ifdef TLSC_AUTO_UPGRADE
        tlsc6x_auto_upgrade_buidin();
        #endif
        tlsc6x_tpd_reset();
    }else{
        pr_err("tlsc6x:%s, no tlsc6x!\n", __func__);
        err = -ENODEV;
        goto exit_chip_check_failed;
    }

    client->irq = gpio_to_irq(pdata->irq_gpio_number);
	
    INIT_WORK(&g_tp_drvdata->resume_work, tlsc6x_resume_work);
    g_tp_drvdata->tp_resume_workqueue = create_singlethread_workqueue("tlsc6x_resume_work");
    if(!g_tp_drvdata->tp_resume_workqueue) {
        err = -ESRCH;
        goto exit_create_singlethread;
    }

    input_dev = input_allocate_device();
    if(!input_dev){
        err = -ENOMEM;
        dev_err(&client->dev, "tlsc6x error::failed to allocate input device\n");
        goto exit_input_dev_alloc_failed;
    }
    g_tp_drvdata->input_dev = input_dev;
    input_dev->name = "tlsc6x_dbg";//pClient->name;//"tlsc6x";//
    input_dev->phys = "I2C";
    input_dev->dev.parent = &client->dev;
    input_dev->id.bustype = BUS_I2C;
    #ifdef TP_PROXIMITY_SENSOR
    ps_input_dev = input_allocate_device();
    if(!ps_input_dev){
        err = -ENOMEM;
        dev_err(&client->dev, "tlsc6x error::failed to allocate ps-input device\n");
        goto exit_input_register_device_failed;
    }
    g_tp_drvdata->ps_input_dev = ps_input_dev;
    ps_input_dev->name = "alps_pxy";
    set_bit(EV_ABS, ps_input_dev->evbit);
    input_set_capability(ps_input_dev, EV_ABS, ABS_DISTANCE); 
    input_set_abs_params(ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);
    err = input_register_device(ps_input_dev);
    if (err) {
        dev_err(&client->dev,
            "failed to register input device: %s\n",
            dev_name(&client->dev));
        goto exit_input_register_device_failed;
    }

    err = misc_register(&tpd_ps_device);
    if(err){
        printk("%s: tlsc6x_ps_device register failed\n", __func__);
        goto exit_input_register_device_failed;
    }
    spin_lock_init(&proximity_switch_lock);
    spin_lock_init(&proximity_state_lock);
    #endif	

    __set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
    __set_bit(ABS_MT_POSITION_X, input_dev->absbit);
    __set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
    __set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
    __set_bit(KEY_APPSELECT,  input_dev->keybit);
    __set_bit(KEY_BACK,  input_dev->keybit);
    __set_bit(KEY_HOMEPAGE,  input_dev->keybit);
    set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
#ifdef TP_GESTRUE
    //__set_bit(KEY_LEFT,  input_dev->keybit);
    //__set_bit(KEY_RIGHT,  input_dev->keybit);
    //__set_bit(KEY_UP,  input_dev->keybit);
    //__set_bit(KEY_DOWN,  input_dev->keybit);
    //__set_bit(KEY_D,  input_dev->keybit);
    //__set_bit(KEY_O,  input_dev->keybit);
    //__set_bit(KEY_W,  input_dev->keybit);
    //__set_bit(KEY_M,  input_dev->keybit);
    //__set_bit(KEY_E,  input_dev->keybit);
    //__set_bit(KEY_C,  input_dev->keybit);
    //__set_bit(KEY_S,  input_dev->keybit);
    //__set_bit(KEY_V,  input_dev->keybit);
    //__set_bit(KEY_Z,  input_dev->keybit);
    __set_bit(KEY_POWER,  input_dev->keybit);

	input_set_capability(input_dev, EV_KEY, KEY_POWER);
	
#endif
    __set_bit(BTN_TOUCH, input_dev->keybit);

    #if MULTI_PROTOCOL_TYPE_B
    input_mt_init_slots(input_dev, TS_MAX_FINGER,0);
    #endif
    input_set_abs_params(input_dev,ABS_MT_POSITION_X, 0, pdata->TP_MAX_X, 0, 0);
    input_set_abs_params(input_dev,ABS_MT_POSITION_Y, 0, pdata->TP_MAX_Y, 0, 0);
    input_set_abs_params(input_dev,ABS_MT_TOUCH_MAJOR, 0, 15, 0, 0);
    input_set_abs_params(input_dev,ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
    input_set_abs_params(input_dev,ABS_MT_PRESSURE, 0, 127, 0, 0);

    set_bit(EV_ABS, input_dev->evbit);
    set_bit(EV_KEY, input_dev->evbit);

    err = input_register_device(input_dev);
    if(err){
        dev_err(&client->dev,"tlsc6x error::failed to register input device: %s\n",dev_name(&client->dev));
        goto exit_input_register_device_failed;
    }

    #ifdef TOUCH_VIRTUAL_KEYS
    tlsc6x_virtual_keys_init();
    #endif


    #ifdef TP_GESTRUE
    #if (LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0))
    wake_lock_init(&gesture_timeout_wakelock, WAKE_LOCK_SUSPEND, "gesture_timeout_wakelock");
    #endif
    proc_entry = proc_create("gesture_enable", 0644, NULL, &gesture_fops);
    #endif


    err = request_irq(client->irq, tlsc6x_interrupt,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND, client->name, g_tp_drvdata);
    if(err < 0){
        dev_err(&client->dev, "tlsc6x error::request irq failed %d\n",err);
        goto exit_irq_request_failed;
    }
	#if defined(CONFIG_ADF)
	g_tp_drvdata->fb_notif.notifier_call = ts_adf_event_handler;
    g_tp_drvdata->fb_notif.priority = 1000;
    ret = adf_register_client(&g_tp_drvdata->fb_notif);
    if (ret) {
        dev_err(&client->dev, "[FB]Unable to register fb_notifier: %d", ret);
    }
    #elif defined(CONFIG_HAS_EARLYSUSPEND)
	g_tp_drvdata->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	g_tp_drvdata->early_suspend.suspend = tlsc6x_ts_suspend;
	g_tp_drvdata->early_suspend.resume  = tlsc6x_ts_resume;
	register_early_suspend(&g_tp_drvdata->early_suspend);
    #endif

    #ifdef TLSC_APK_DEBUG
    tlsc6x_create_apk_debug_channel(client);
    #endif

    #ifdef TLSC_ESD_HELPER_EN
    {    // esd issue: i2c monitor thread
        ktime_t ktime = ktime_set(30, 0);
        hrtimer_init(&tpd_esd_kthread_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        tpd_esd_kthread_timer.function = tpd_esd_kthread_hrtimer_func;
        hrtimer_start(&tpd_esd_kthread_timer, ktime, HRTIMER_MODE_REL);	
        kthread_run(esd_checker_handler, 0, "tlsc6x_esd_helper");
    }
    #endif

    thread = kthread_run(touch_event_handler, 0, "focal-wait-queue");
    if (IS_ERR(thread)){
        err = PTR_ERR(thread);
        printk("failed to create kernel thread: %d\n", err);
    }


		touchscreen_class = class_create(THIS_MODULE, "touchscreen");
		if (IS_ERR_OR_NULL(touchscreen_class)) {
			printk("%s: create class error!\n", __func__);
			return -ENOMEM;
		}
		tls6x_touchscreen_cmd_dev = device_create(touchscreen_class, NULL, 0, NULL, "device");
			if (IS_ERR(tls6x_touchscreen_cmd_dev))
			{
				pr_err("Failed to create device(firmware_cmd_dev)!\n");
			}
				// version
		if (device_create_file(tls6x_touchscreen_cmd_dev, &dev_attr_tp_version) < 0)
		{
			pr_err("Failed to create device file(%s)!\n", dev_attr_tp_version.attr.name);
		}	
#ifdef TP_GESTRUE
			if (device_create_file(tls6x_touchscreen_cmd_dev, &dev_attr_gesture_wakeup_mode) < 0)
				pr_err("Failed to create device file(%s)!\n", dev_attr_gesture_wakeup_mode.attr.name);
			
			if (device_create_file(tls6x_touchscreen_cmd_dev, &dev_attr_gesture_wakeup_value) < 0)
				pr_err("Failed to create device file(%s)!\n", dev_attr_gesture_wakeup_value.attr.name);
#endif //

    return 0;

exit_irq_request_failed:
    input_unregister_device(input_dev);
exit_input_register_device_failed:
    input_free_device(input_dev);
    #ifdef TP_PROXIMITY_SENSOR
    if(ps_input_dev){
        input_free_device(ps_input_dev);
    }
    #endif
exit_input_dev_alloc_failed:
exit_create_singlethread:
exit_chip_check_failed:
    gpio_free(pdata->irq_gpio_number);
    gpio_free(pdata->reset_gpio_number);
exit_gpio_request_failed:
    kfree(g_tp_drvdata);
exit_alloc_data_failed:
    g_tp_drvdata = NULL;
    i2c_set_clientdata(client, g_tp_drvdata);
exit_alloc_platform_data_failed:
    return err;
}

static int tlsc6x_remove(struct i2c_client *client)
{
    struct tlsc6x_data *drvdata = i2c_get_clientdata(client);

    pr_info("%s ++!\n", __func__);

    #ifdef TP_GESTRUE
    #if (LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0))
    wake_lock_destroy(&gesture_timeout_wakelock);
    #endif
    #endif
	
    #ifdef TLSC_APK_DEBUG
    tlsc6x_release_apk_debug_channel();
    #endif

    #ifdef TLSC_ESD_HELPER_EN
    hrtimer_cancel(&tpd_esd_kthread_timer);
    #endif
	
	#if defined(CONFIG_ADF)
	adf_unregister_client(&g_tp_drvdata->fb_notif);
    #elif defined(CONFIG_HAS_EARLYSUSPEND)
    unregister_early_suspend(&drvdata->early_suspend);
    #endif

    free_irq(client->irq, drvdata);
    input_unregister_device(drvdata->input_dev);
    input_free_device(drvdata->input_dev);

    #ifdef CONFIG_HAS_EARLYSUSPEND
    cancel_work_sync(&drvdata->resume_work);
    destroy_workqueue(drvdata->tp_resume_workqueue);
    #endif
    kfree(drvdata);
    drvdata = NULL;
    i2c_set_clientdata(client, drvdata);

    return 0;
}

static const struct i2c_device_id tlsc6x_id[] = {
    { TS_NAME, 0 },{ }
};
MODULE_DEVICE_TABLE(i2c, tlsc6x_id);

static const struct of_device_id tlsc6x_of_match[] = {
    { .compatible = "tlsc6x,tlsc6x_ts", },
    { }
};
MODULE_DEVICE_TABLE(of, tlsc6x_of_match);
static struct i2c_driver tlsc6x_driver = {
    .probe		= tlsc6x_probe,
    .remove	= tlsc6x_remove,
    .id_table	= tlsc6x_id,
    .driver	= {
        .name	= TS_NAME,
        .owner	= THIS_MODULE,
        .of_match_table = tlsc6x_of_match,
    },
};

static int __init tlsc6x_init(void)
{
    printk("%s: ++\n",__func__);
    return i2c_add_driver(&tlsc6x_driver);
}

static void __exit tlsc6x_exit(void)
{
    i2c_del_driver(&tlsc6x_driver);
}

late_initcall(tlsc6x_init);
module_exit(tlsc6x_exit);


MODULE_DESCRIPTION("telink touchscreen driver");
MODULE_LICENSE("GPL");
