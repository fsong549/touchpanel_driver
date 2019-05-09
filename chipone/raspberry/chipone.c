#include <linux/delay.h>

#include <linux/property.h>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/kthread.h>
#include <linux/wait.h>

#define TOUCH_GPIO 18

const static unsigned short normal_address[] = {0x48, I2C_CLIENT_END};

static struct input_dev *ts_input;

static struct i2c_client * this_client = NULL;

static struct task_struct *tsk;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int tpd_flag = 0;

MODULE_LICENSE("Dual BSD/GPL");

struct sensor_chiponeic {
	struct mutex lock;
	struct i2c_client *client;
	struct completion wait;
	bool is_self_test;
	u16 x_axis;
	u16 y_axis;
	u16 z_axis;
	u8 sample;
	u8 out_rate;
	u8 mesura;
	u8 mode;
	u8 gain;
	wait_queue_head_t wq_irq;
};
static struct sensor_chiponeic *chiponeic;

static int i2c_read_bytes(struct i2c_client *client, u16 addr, uint8_t *buf, int len)
{
	int ret = -1;
	int retries = 0;
	unsigned char tmp_buf[2];

	struct i2c_msg msgs[] =
	{
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = tmp_buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	//tmp_buf[0] = addr;
	tmp_buf[0] = (unsigned char)(addr>>8);
	tmp_buf[1] = (unsigned char)(addr);

	while(retries < 5)
	{

		ret = i2c_transfer(client->adapter, msgs, 2);

		if(ret == 2)
			break;

		retries++;
	}
	return ret;
}

static int i2c_write_bytes(struct i2c_client *client,uint8_t *data,int len)
{
	struct i2c_msg msg;
	int ret=-1;

	msg.flags=!I2C_M_RD;
	msg.addr=client->addr;
	msg.len=len;
	msg.buf=data;

	ret=i2c_transfer(client->adapter,&msg, 1);
	return ret;
}

#define POINT_NUM  5
#define POINT_SIZE 7
static int thread_function(void *data)
{
	unsigned char point_data[POINT_NUM * POINT_SIZE + 3];
	unsigned short input_x = 0;
	unsigned short input_y = 0;
	unsigned short input_p = 0;
	static unsigned int status = 0;
	int ret = 0;
	unsigned char position = 0;
	struct sensor_chiponeic *chipone = data;

	do
	{
		wait_event(waiter,tpd_flag != 0);
		tpd_flag = 0;

		ret=i2c_read_bytes(chipone->client, 0x1000, point_data, sizeof(point_data)/sizeof(point_data[0]));
		if(ret <= 0){
			printk("Failed\n");
			return 0;
		}
		//point_data数组的前两位是要读取寄存器的起始地址，point_data[2]对应chiponeic芯片寄存器的0x0721寄存器内容
		if(point_data[1] > 0){
			status=1; //标记有触点触发
			input_y = (point_data[6 + POINT_SIZE*position]<<8) + point_data[5 + POINT_SIZE*position];
			input_x = (point_data[4 + POINT_SIZE*position]<<8) + point_data[3 + POINT_SIZE*position];
			input_p = point_data[7 + POINT_SIZE*position];
		}
		//如果没有触摸动作 并且上次有触摸 表明触摸松开
		else if(status){
			status = 0;
			//触摸点被释放
			input_mt_report_slot_state(ts_input, MT_TOOL_FINGER, false);
			input_mt_sync_frame(ts_input);
			input_sync(ts_input);
			input_x = input_y = input_p = 0;
		}

		printk("x=%d,y=%d,w=%d\n",input_x,input_y,input_p);

		if(status){
			if(input_x >=0 &&input_x<1024 &&input_y>=0&&input_y<600){
				//printk("x=%d,y=%d,w=%d\n",input_x,input_y,input_p);

				//触摸点被按下
				input_mt_report_slot_state(ts_input, MT_TOOL_FINGER, true);

				input_report_abs(ts_input, ABS_MT_POSITION_X, input_x);
				input_report_abs(ts_input, ABS_MT_POSITION_Y, 600-input_y);
				input_report_abs(ts_input, ABS_MT_TOUCH_MAJOR, input_p);

				input_mt_sync_frame(ts_input);
				input_sync(ts_input);
			}

		}
	}while(!kthread_should_stop());

	return 0;
}

static irqreturn_t chiponeic_thread(int irq , void * data)
{
	printk("chipone touch interrupts!!!\n");
	tpd_flag = 1;
	wake_up(&waiter);

	return IRQ_HANDLED;
}

/* echo addr value1 value2 value3 ... valueN > write_reg */
static ssize_t write_firmware_register_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return 0;
}
static DEVICE_ATTR(write_reg, S_IWUSR, NULL, write_firmware_register_store);

static ssize_t read_firmware_register_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}

/* echo addr size > read_reg */
static ssize_t read_firmware_register_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return 0;
}

static DEVICE_ATTR(read_reg, S_IWUSR | S_IRUSR,
		read_firmware_register_show, read_firmware_register_store);


static struct attribute *chipone_dev_firmware_atts[] = {
	&dev_attr_read_reg.attr,
	&dev_attr_write_reg.attr,
	NULL
};

static const struct attribute_group chipone_dev_firmware_attr_group = {
	.name  = "firmware",
	.attrs = chipone_dev_firmware_atts,
};



static ssize_t flash_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}
static DEVICE_ATTR(info, S_IRUGO, flash_info_show, NULL);

static ssize_t read_flash_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}

/* echo start_addr size [filepath] > read */
static ssize_t read_flash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return 0;
}
static DEVICE_ATTR(read, S_IWUSR | S_IRUGO, read_flash_show, read_flash_store);

/* echo addr size > erase */
static ssize_t erase_flash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return 0;
}
static DEVICE_ATTR(erase, S_IWUSR, NULL, erase_flash_store);

static struct attribute *cts_dev_flash_attrs[] = {
	&dev_attr_info.attr,
	&dev_attr_read.attr,
	&dev_attr_erase.attr,

	NULL
};

static const struct attribute_group chipone_dev_flash_attr_group = {
	.name  = "flash",
	.attrs = cts_dev_flash_attrs,
};


static const struct attribute_group *chipone_dev_attr_groups[] = {
	&chipone_dev_firmware_attr_group,
	&chipone_dev_flash_attr_group,
	NULL
};

void chipone_sysfs_add_device(struct device *dev)
{
	int ret,i;

	printk("chipone sysfs add device attr groups");


	for(i = 0; chipone_dev_attr_groups[i]; i++) {
		ret = sysfs_create_group(&dev->kobj, chipone_dev_attr_groups[i]);
		if (ret) {
			while (--i >= 0) {
				sysfs_remove_group(&dev->kobj, chipone_dev_attr_groups[i]);
			}
			break;
		}
	}

	if (ret) {
		printk("Add device attr failed %d", ret);
	}

	//return ret;
}

//#define CHIPONE_READ_BIN_FILE

static int chiponeic_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int retry, ret , err;
	//int ret;
	char test_data;
	//unsigned int read_data = 0;
	unsigned char read_data;

#ifdef CHIPONE_READ_BIN_FILE
	int file_size;
	//mm_segment_t fs;
	struct inode *inode = NULL;
	struct file *fp;
#endif

	//i2c_set_clientdata(client, chiponeic);

	printk("=================CHIPONEIC TOUCH BEGIN=================\n");
	printk("20190213 14:08 chipone_probe entry\n");

	test_data = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		dev_err(&client->dev, "Must have I2C_FUNC_I2C.\n");
		return -ENODEV;
	}

	chiponeic = devm_kzalloc(&client->dev, sizeof(struct sensor_chiponeic),
			GFP_KERNEL);
	if (!chiponeic)
		return -ENOMEM;
	i2c_set_clientdata(client, chiponeic);
	chiponeic->client = client;
#if 0
	//test iic communication
	for(retry=0;retry < 5; retry++)
	{
		msleep(100);

		ret = i2c_write_bytes(client, &test_data, 1);
		printk("chipone i2c write bytes RET is:%d\n",ret);
		if (ret > 0)
			break;
		dev_info(&client->dev, "GT811 I2C TEST FAILED!Please check the HARDWARE connect\n");
	}

	if(ret <= 0)
	{
		dev_err(&client->dev, "Warnning: I2C communication might be ERROR!\n");
		return -ENODEV;
	}

	//read ic version main
	ret = i2c_read_bytes(client, 0x000a, &read_data, 1);
	//printk("chipone i2c read bytes RET is:%d\n",ret);

	if(ret < 0)
	{
		printk("chipone Warnning: I2C communication READ ERROR!\n");
		return -ENODEV;
	}
	else
		printk("chiponeIC VERSION MAIN is:%x\n",read_data);
#endif
#if 0// chipone never uses!
	for(retry=0;retry < 5; retry++)
	{
		msleep(100);

		ret = i2c_write_bytes(client, &test_data, 1);
		if (ret > 0)
			break;
		dev_info(&client->dev, "GT811 I2C TEST FAILED!Please check the HARDWARE connect\n");
	}

	if(ret <= 0)
	{
		dev_err(&client->dev, "Warnning: I2C communication might be ERROR!\n");
		return -ENODEV;
	}

	//初始化chiponeic芯片
	for(retry = 0; retry != 5; ++ retry){
		ret = ts_init_panel(client);
		if(ret != 0){
			continue;
		}
		else{
			break;
		}
	}

	if(ret != 0){
		printk("GT811 Configue failed!\n");
		return -ENODEV;
	}
#endif

	this_client = client;

	//申请input设备空间
	ts_input = input_allocate_device();
	if(IS_ERR(ts_input)){
		printk("GT811 allocate ts input device failed!\n");
		return -ENOMEM;
	}

	ts_input->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	ts_input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts_input->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);
	/*
	   input_set_abs_params(ts_input, ABS_Y, 0, 1024, 0, 0);
	   input_set_abs_params(ts_input, ABS_X, 0, 600, 0, 0);
	   input_set_abs_params(ts_input, ABS_PRESSURE, 0, 255, 0, 0);
	   */
	__set_bit(INPUT_PROP_DIRECT, ts_input->propbit);
	__set_bit(EV_ABS, ts_input->evbit);

	input_mt_init_slots(ts_input, 1,0);
	input_set_abs_params(ts_input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts_input, ABS_MT_POSITION_X, 0, 1024, 0, 0);
	input_set_abs_params(ts_input, ABS_MT_POSITION_Y, 0, 600, 0, 0);

	ts_input->name = "chiponeic-ts";
	ts_input->phys = "input/ts";
	ts_input->id.bustype = BUS_I2C;
	ts_input->id.product = 0xBEEF;
	ts_input->id.vendor =0xDEAD;

	//注册输入设备
	ret = input_register_device(ts_input);
	if(ret < 0){
		printk("Unable register %s input device!\n", ts_input->name);
		input_free_device(ts_input);
		return -ENOMEM;
	}

	printk("chipone sysfs init");
	chipone_sysfs_add_device(&client->dev);


	//printk("chipone init waitqueue head;for test poll");
	//init_waitqueue_head(&chiponeic->wq_irq);	

	//irq request 
	printk("chipone irq request init");
	err = gpio_request_one(TOUCH_GPIO,GPIOF_IN,"touch irq gpio");	
	if(err)
		return err;
	enable_irq(gpio_to_irq(TOUCH_GPIO));
	err = devm_request_threaded_irq(&client->dev,gpio_to_irq(TOUCH_GPIO),NULL,chiponeic_thread,IRQF_ONESHOT|IRQF_TRIGGER_FALLING,"touch irq",chiponeic);
	if(err < 0){
		printk("chipone request irq fail!!!");
		return err;
	}

#ifdef CHIPONE_READ_BIN_FILE
	printk("=================CHIPONEIC READ BIN FILE=================\n");
	fp = filp_open("/home/pi/Desktop/liunx_test/chiponeic_icn/ICNT88xx.bin",O_RDONLY,0644);
	if(IS_ERR(fp))
	{
		printk("chipone open bin file!!!");
	}

	inode = fp->f_path.dentry->d_inode;
	file_size = inode->i_size;

	//fs = get_fs();
	//set_fs(KERNEL_DS);
	printk("chipone read bin file size:%d\n",file_size);
	filp_close(fp,NULL);
#endif

	printk("=================CHIPONEIC TOUCH END=================\n");

	printk("=================chiponr thread run init=================\n");
	tsk = kthread_run(thread_function, 0, "chipone_thread");
	if(IS_ERR(tsk))
	{
		printk("chipone create kthread failed!!!");
	}
	else
		printk("chipone create kthread success!!!");


	return 0;
}

static int chiponeic_remove(struct i2c_client *client)
{
	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts_input);
	//kfree(chiponeic);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id chiponeic_of_match[] = {
	{ .compatible = "chipone,icn8318", },
	{ }
};
MODULE_DEVICE_TABLE(of, chiponeic_of_match);
#endif


static const struct i2c_device_id chiponeic_id[] = {
	{"chiponeic", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, chiponeic_id);

static struct i2c_driver chiponeic_driver = {
	.driver = {
		.name = "chipone",
		.of_match_table = of_match_ptr(chiponeic_of_match),
	},
	.probe = chiponeic_probe,
	.remove = chiponeic_remove,
	.id_table = chiponeic_id,
};

module_i2c_driver(chiponeic_driver);

