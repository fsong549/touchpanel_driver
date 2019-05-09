#define LOG_TAG         "Plat"

#include "cts_config.h"
#include "cts_platform.h"
#include "cts_core.h"


extern struct chipone_ts_data *chipone_ts_data;


size_t cts_plat_get_max_i2c_xfer_size(struct cts_platform_data *pdata)
{
    return CFG_CTS_MAX_I2C_XFER_SIZE;
}

u8 *cts_plat_get_i2c_xfer_buf(struct cts_platform_data *pdata, 
    size_t xfer_size)
{
    return pdata->i2c_fifo_buf;
}

int cts_plat_i2c_write(struct cts_platform_data *pdata, u8 i2c_addr,
        const void *src, size_t len, int retry, int delay)
{
    int ret = 0, retries = 0;

    struct i2c_msg msg = {
        .flags    = 0,
        .addr    = i2c_addr,
        .buf    = (u8 *)src,
        .len    = len,
    };

    do {
        ret = i2c_transfer(pdata->i2c_client->adapter, &msg, 1);
        if (ret != 1) {
            if (ret >= 0) {
                ret = -EIO;
            }

            if (delay) {
                mdelay(delay);
            }
	    cts_dbg("Write data 0x04 0x40 failed,times = %d",retries);
            continue;
        } else {
	    cts_dbg("Write data 0x04 0x40 success,times = %d",retries);
            return 0;
        }
    } while (++retries < retry);

    return ret;
}

int cts_plat_i2c_read(struct cts_platform_data *pdata, u8 i2c_addr,
        const u8 *wbuf, size_t wlen, void *rbuf, size_t rlen,
        int retry, int delay)
{
    int num_msg, ret = 0, retries = 0;

    struct i2c_msg msgs[2] = {
        {
            .addr    = i2c_addr,
            .flags    = 0,
            .buf    = (u8 *)wbuf,
            .len    = wlen
        },
        {
            .addr    = i2c_addr,
            .flags    = I2C_M_RD,
            .buf    = (u8 *)rbuf,
            .len    = rlen
        }
    };

    if (wbuf == NULL || wlen == 0) {
        num_msg = 1;
    } else {
        num_msg = 2;
    }

    do {
        ret = i2c_transfer(pdata->i2c_client->adapter,
                msgs + ARRAY_SIZE(msgs) - num_msg, num_msg);

        if (ret != num_msg) {
            if (ret >= 0) {
                ret = -EIO;
            }

            if (delay) {
                mdelay(delay);
            }
            continue;
        } else {
            return 0;
        }
    } while (++retries < retry);

    return ret;
}

int cts_plat_is_i2c_online(struct cts_platform_data *pdata, u8 i2c_addr)
{
    u8 dummy_bytes[2] = {0x00, 0x00};
    int ret;

    ret = cts_plat_i2c_write(pdata, i2c_addr, dummy_bytes, sizeof(dummy_bytes), 5, 2);
    if (ret) {
        cts_err("!!! I2C addr 0x%02x is offline !!!", i2c_addr);
        return false;
    } else {
        cts_dbg("I2C addr 0x%02x is online", i2c_addr);
        return true;
    }
}

static void cts_plat_handle_irq(struct cts_platform_data *pdata)
{
    int ret;

    cts_dbg("Handle IRQ");

    cts_lock_device(pdata->cts_dev);
    ret = cts_irq_handler(pdata->cts_dev);
    if (ret) {
        cts_err("Device handle IRQ failed %d", ret);
    }
    cts_unlock_device(pdata->cts_dev);
}

static irqreturn_t cts_plat_irq_handler(int irq, void *dev_id)
{
    struct cts_platform_data *pdata;
#ifndef CONFIG_GENERIC_HARDIRQS
    struct chipone_ts_data *cts_data;
#endif /* CONFIG_GENERIC_HARDIRQS */

    cts_dbg("IRQ handler");

    pdata = (struct cts_platform_data *)dev_id;
    if (pdata == NULL) {
        cts_err("IRQ handler with NULL dev_id");
        return IRQ_NONE;
    }

#ifdef CONFIG_GENERIC_HARDIRQS
    cts_plat_handle_irq(pdata);
#else /* CONFIG_GENERIC_HARDIRQS */
    cts_data = container_of(pdata->cts_dev, struct chipone_ts_data, cts_dev);

    cts_plat_disable_irq(pdata);

    queue_work(cts_data->workqueue, &pdata->ts_irq_work);
#endif /* CONFIG_GENERIC_HARDIRQS */

    return IRQ_HANDLED;
}

#ifndef CONFIG_GENERIC_HARDIRQS
static void cts_plat_touch_dev_irq_work(struct work_struct *work)
{
    struct cts_platform_data *pdata =
        container_of(work, struct cts_platform_data, ts_irq_work);

    cts_dbg("IRQ work");

    cts_plat_handle_irq(pdata);

    cts_plat_enable_irq(pdata);
}
#endif /* CONFIG_GENERIC_HARDIRQS */

#ifdef CONFIG_CTS_OF
static int cts_plat_parse_dt(struct cts_platform_data *pdata,
		struct device_node *dev_node)
{
	int ret;

	cts_info("Parse device tree");

	pdata->int_gpio = of_get_named_gpio(dev_node, CFG_CTS_OF_INT_GPIO_NAME, 0);
	if (!gpio_is_valid(pdata->int_gpio)) {
		cts_err("Parse INT GPIO from dt failed %d", pdata->int_gpio);
		pdata->int_gpio = -1;
	}
	cts_info("  %-12s: %d", "int gpio", pdata->int_gpio);

    pdata->irq = gpio_to_irq(pdata->int_gpio);
    if (pdata->irq < 0) {
        cts_err("Parse irq failed %d", ret);
        return pdata->irq;
    }
	cts_info("  %-12s: %d", "irq num", pdata->irq);

#ifdef CFG_CTS_HAS_RESET_PIN
	pdata->rst_gpio = of_get_named_gpio(dev_node, CFG_CTS_OF_RST_GPIO_NAME, 0);
	if (!gpio_is_valid(pdata->rst_gpio)) {
		cts_err("Parse RST GPIO from dt failed %d", pdata->rst_gpio);
		pdata->rst_gpio = -1;
	}
	cts_info("  %-12s: %d", "rst gpio", pdata->rst_gpio);
#endif /* CFG_CTS_HAS_RESET_PIN */

	ret = of_property_read_u32(dev_node, CFG_CTS_OF_X_RESOLUTION_NAME,
			&pdata->res_x);
	if (ret) {
		cts_warn("Parse X resolution from dt failed %d", ret);
		//return ret;
	}
	cts_info("  %-12s: %d", "X resolution", pdata->res_x);

	ret = of_property_read_u32(dev_node, CFG_CTS_OF_Y_RESOLUTION_NAME,
			&pdata->res_y);
	if (ret) {
		cts_warn("Parse Y resolution from dt failed %d", ret);
		//return ret;
	}
	cts_info("  %-12s: %d", "Y resolution", pdata->res_y);

	return 0;
}
#endif /* CONFIG_CTS_OF */

int cts_init_platform_data(struct cts_platform_data *pdata,
        struct i2c_client *i2c_client)
{
	struct input_dev *input_dev;
    int ret;

    cts_info("Init");

#ifdef CONFIG_CTS_OF
    {
    	struct device *dev = &i2c_client->dev;
        ret = cts_plat_parse_dt(pdata, dev->of_node);
        if (ret) {
            cts_err("Parse dt failed %d", ret);
            return ret;
        }
    }
#endif /* CONFIG_CTS_OF */

    pdata->i2c_client = i2c_client;
    rt_mutex_init(&pdata->dev_lock);

    pdata->i2c_client->irq = pdata->irq;
    spin_lock_init(&pdata->irq_lock);

	input_dev = input_allocate_device();
	if (input_dev == NULL) {
		cts_err("Failed to allocate input device.");
		return -ENOMEM;
	}

	/** - Init input device */
	input_dev->name = CFG_CTS_DEVICE_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &pdata->i2c_client->dev;

    input_dev->evbit[0] =   BIT_MASK(EV_SYN) |
                            BIT_MASK(EV_KEY) |
                            BIT_MASK(EV_ABS);
#ifdef CFG_CTS_SWAP_XY
    input_set_abs_params(input_dev, ABS_MT_POSITION_X,
            0, pdata->res_y, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
            0, pdata->res_x, 0, 0);
#else /* CFG_CTS_SWAP_XY */
    input_set_abs_params(input_dev, ABS_MT_POSITION_X,
            0, pdata->res_x, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
            0, pdata->res_y, 0, 0);
#endif /* CFG_CTS_SWAP_XY */

    input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,0, 255, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TRACKING_ID,0, 255, 0, 0);

    input_set_capability(input_dev, EV_KEY, BTN_TOUCH);

#ifdef CONFIG_CTS_SLOTPROTOCOL
    input_mt_init_slots(input_dev, CFG_CTS_MAX_TOUCH_NUM,
        INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
#endif /* CONFIG_CTS_SLOTPROTOCOL */

    input_set_drvdata(input_dev, pdata);

    ret = input_register_device(input_dev);
    if (ret) {
        cts_err("Failed to register input device");
        return ret;
    }

    pdata->ts_input_dev = input_dev;

#if !defined(CONFIG_GENERIC_HARDIRQS)
    INIT_WORK(&pdata->ts_irq_work, cts_plat_touch_dev_irq_work);
#endif /* CONFIG_GENERIC_HARDIRQS */

#ifdef CONFIG_CTS_VIRTUALKEY
    {
	    u8 vkey_keymap[CFG_CTS_NUM_VKEY] = CFG_CTS_VKEY_KEYCODES;
        memcpy(pdata->vkey_keycodes, vkey_keymap, sizeof(vkey_keymap));
        pdata->vkey_num = CFG_CTS_NUM_VKEY;
    }
#endif /* CONFIG_CTS_VIRTUALKEY */

#ifdef CONFIG_CTS_GESTURE
    {
        u8 gesture_keymap[CFG_CTS_NUM_GESTURE][2] = CFG_CTS_GESTURE_KEYMAP;
        memcpy(pdata->gesture_keymap, gesture_keymap, sizeof(gesture_keymap));
        pdata->gesture_num = CFG_CTS_NUM_GESTURE;
    }
#endif /* CONFIG_CTS_GESTURE */

    return 0;
}

int cts_plat_request_resource(struct cts_platform_data *pdata)
{
    int ret;

    cts_info("Request resource");

    ret = gpio_request_one(pdata->int_gpio, GPIOF_IN,
        CFG_CTS_DEVICE_NAME "-int");
    if (ret) {
        cts_err("Request INT gpio (%d) failed %d", pdata->int_gpio, ret);
        goto err_out;
    }

#ifdef CFG_CTS_HAS_RESET_PIN
    ret = gpio_request_one(pdata->rst_gpio, GPIOF_OUT_INIT_HIGH,
        CFG_CTS_DEVICE_NAME "-rst");
    if (ret) {
        cts_err("Request RST gpio (%d) failed %d", pdata->rst_gpio, ret);
        goto err_free_int;
    }
#endif /* CFG_CTS_HAS_RESET_PIN */


    return 0;

#ifdef CONFIG_CTS_REGULATOR
err_free_rst:
#endif /* CONFIG_CTS_REGULATOR */
#ifdef CFG_CTS_HAS_RESET_PIN
    gpio_free(pdata->rst_gpio);
err_free_int:
#endif /* CFG_CTS_HAS_RESET_PIN */
    gpio_free(pdata->int_gpio);
err_out:
    return ret;
}

void cts_plat_free_resource(struct cts_platform_data *pdata)
{
    cts_info("Free resource");

    if (gpio_is_valid(pdata->int_gpio)) {
        gpio_free(pdata->int_gpio);
    }
#ifdef CFG_CTS_HAS_RESET_PIN
    if (gpio_is_valid(pdata->rst_gpio)) {
        gpio_free(pdata->rst_gpio);
    }
#endif /* CFG_CTS_HAS_RESET_PIN */
}

int cts_plat_request_irq(struct cts_platform_data *pdata)
{
    int ret;

    cts_info("Request IRQ");

#ifdef CONFIG_GENERIC_HARDIRQS
    /* Note:
     * If IRQ request succeed, IRQ will be enbled !!!
     */
    ret = request_threaded_irq(pdata->irq,
            NULL, cts_plat_irq_handler, IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND | IRQF_ONESHOT,
            pdata->i2c_client->dev.driver->name, pdata);
#else /* CONFIG_GENERIC_HARDIRQS */
    ret = request_irq(pdata->irq,
            cts_plat_irq_handler, IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND | IRQF_ONESHOT,
            pdata->i2c_client->dev.driver->name, pdata);
#endif /* CONFIG_GENERIC_HARDIRQS */
    if (ret) {
        cts_err("Request IRQ failed %d", ret);
        return ret;
    }

    cts_plat_disable_irq(pdata);

    return 0;
}

void cts_plat_free_irq(struct cts_platform_data *pdata)
{
    free_irq(pdata->irq, pdata);
}

int cts_plat_enable_irq(struct cts_platform_data *pdata)
{
    unsigned long irqflags;

    cts_dbg("Enable IRQ");

    if (pdata->irq > 0) {
        spin_lock_irqsave(&pdata->irq_lock, irqflags);
        if (pdata->irq_is_disable)/* && !cts_is_device_suspended(pdata->chip)) */{  
            cts_dbg("Real enable IRQ");
            enable_irq(pdata->irq);
            pdata->irq_is_disable = false;
        }
        spin_unlock_irqrestore(&pdata->irq_lock, irqflags);

        return 0;
    }

    return -ENODEV;
}

int cts_plat_disable_irq(struct cts_platform_data *pdata)
{
    unsigned long irqflags;

    cts_dbg("Disable IRQ");

    if (pdata->irq > 0) {
        spin_lock_irqsave(&pdata->irq_lock, irqflags);
        if (!pdata->irq_is_disable) {
            cts_dbg("Real disable IRQ");
            disable_irq_nosync(pdata->irq);
            pdata->irq_is_disable = true;
        }
        spin_unlock_irqrestore(&pdata->irq_lock, irqflags);

        return 0;
    }

    return -ENODEV;
}

#ifdef CFG_CTS_HAS_RESET_PIN
int cts_plat_reset_device(struct cts_platform_data *pdata)
{
    cts_info("Reset device");

	gpio_set_value(pdata->rst_gpio, 0);
    mdelay(1);
	gpio_set_value(pdata->rst_gpio, 1);
    mdelay(50);

    return 0;
}
#endif /* CFG_CTS_HAS_RESET_PIN */

int cts_plat_power_up_device(struct cts_platform_data *pdata)
{
    cts_info("Power up device");

    return 0;
}

int cts_plat_power_down_device(struct cts_platform_data *pdata)
{
    cts_info("Power down device");

    return 0;
}

int cts_plat_init_touch_device(struct cts_platform_data *pdata)
{
    cts_info("Init touch device");

    return 0;
}

void cts_plat_deinit_touch_device(struct cts_platform_data *pdata)
{
    cts_info("De-init touch device");

    if (pdata->ts_input_dev) {
#ifndef CONFIG_GENERIC_HARDIRQS
        if (work_pending(&pdata->ts_irq_work)) {
            cancel_work_sync(&pdata->ts_irq_work);
        }

        input_free_device(pdata->ts_input_dev);
        pdata->ts_input_dev = NULL;
#endif /* CONFIG_GENERIC_HARDIRQS */
    }
}

int cts_plat_process_touch_msg(struct cts_platform_data *pdata,
            struct cts_device_touch_msg *msgs, int num)
{
    struct input_dev *input_dev = pdata->ts_input_dev;
    int i;
    int contact = 0;

    cts_dbg("Process touch %d msgs", num);

    for (i = 0; i < num; i++) {
        u16 x, y;

        x = le16_to_cpu(msgs[i].x);
        y = le16_to_cpu(msgs[i].y);

#ifdef CFG_CTS_SWAP_XY
        swap(x,y);
#endif /* CFG_CTS_SWAP_XY */
#ifdef CFG_CTS_WRAP_X
        x = wrap(pdata->res_x,x);
#endif /* CFG_CTS_WRAP_X */
#ifdef CFG_CTS_WRAP_Y
        y = wrap(pdata->res_y,y);
#endif /* CFG_CTS_WRAP_Y */

        cts_dbg("  Process touch msg[%d]: id[%u] ev=%u x=%u y=%u p=%u",
            i, msgs[i].id, msgs[i].event, x, y, msgs[i].pressure);

#ifdef CONFIG_CTS_SLOTPROTOCOL
        input_mt_slot(input_dev, msgs[i].id);
        switch (msgs[i].event) {
            case CTS_DEVICE_TOUCH_EVENT_DOWN:
            case CTS_DEVICE_TOUCH_EVENT_MOVE:
            case CTS_DEVICE_TOUCH_EVENT_STAY:
                contact++;
                input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);
                input_report_abs(input_dev, ABS_MT_POSITION_X, x);
                input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
                input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, msgs[i].pressure);
                input_report_abs(input_dev, ABS_MT_PRESSURE, msgs[i].pressure);
                break;

            case CTS_DEVICE_TOUCH_EVENT_UP:
                input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
                break;

            default:
                cts_warn("Process touch msg with unknwon event %u id %u",
                    msgs[i].event, msgs[i].id);
                break;
        }
#else /* CONFIG_CTS_SLOTPROTOCOL */
        /**
         * If the driver reports one of BTN_TOUCH or ABS_PRESSURE
         * in addition to the ABS_MT events, the last SYN_MT_REPORT event
         * may be omitted. Otherwise, the last SYN_REPORT will be dropped
         * by the input core, resulting in no zero-contact event
         * reaching userland.
         */
        switch (msgs[i].event) {
            case CTS_CHIP_TOUCH_EVENT_DOWN:
            case CTS_CHIP_TOUCH_EVENT_MOVE:
            case CTS_CHIP_TOUCH_EVENT_STAY:
                input_report_abs(input_dev, ABS_MT_POSITION_X, x);
                input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
                input_report_abs(input_dev, ABS_MT_PRESSURE, msgs[i].pressure);
                input_mt_sync(input_dev);
                break;

            case CTS_CHIP_TOUCH_EVENT_UP:
                input_report_abs(input_dev, ABS_MT_PRESSURE, 0);
                input_report_key(input_dev, BTN_TOUCH, 0);
                input_mt_sync(input_dev);
            default:
                cts_warn("Process touch msg with unknwon event %u id %u",
                    msgs[i].event, msgs[i].id);
                break;
        }
#endif /* CONFIG_CTS_SLOTPROTOCOL */
    }

    input_report_key(input_dev, BTN_TOUCH, contact > 0);

    input_mt_sync_frame(input_dev);
    input_sync(input_dev);

    return 0;
}

int cts_plat_release_all_touch(struct cts_platform_data *pdata)
{
    struct input_dev *input_dev = pdata->ts_input_dev;

#if !defined(CONFIG_CTS_SLOTPROTOCOL)
    int id;
#endif /* CONFIG_CTS_SLOTPROTOCOL */

    cts_info("Release all touch");

#ifdef CONFIG_CTS_SLOTPROTOCOL
    input_mt_sync_frame(input_dev);
#else /* CONFIG_CTS_SLOTPROTOCOL */
    for (id = 0; id < CFG_CTS_MAX_TOUCH_NUM; id++) {
        input_report_key(input_dev, BTN_TOUCH, 0);
        input_report_abs(input_dev, ABS_MT_PRESSURE, 0);
        input_mt_sync(input_dev);
    }
#endif /* CONFIG_CTS_SLOTPROTOCOL */

    input_sync(input_dev);

    return 0;
}

#ifdef CONFIG_CTS_VIRTUALKEY
int cts_plat_init_vkey_device(struct cts_platform_data *pdata)
{
    int i;

    cts_info("Init VKey");

    pdata->vkey_state = 0;

    for (i = 0; i <  pdata->vkey_num; i++) {
        input_set_capability(pdata->ts_input_dev,
            EV_KEY, pdata->vkey_keycodes[i]);
    }

    return 0;
}

void cts_plat_deinit_vkey_device(struct cts_platform_data *pdata)
{
    cts_info("De-init VKey");

    pdata->vkey_state = 0;
}

int cts_plat_process_vkey(struct cts_platform_data *pdata, u8 vkey_state)
{
    u8  event;
    int i;

    event = pdata->vkey_state ^ vkey_state;

    cts_dbg("Process vkey state=0x%02x, event=0x%02x", vkey_state, event);

    for (i = 0; i < pdata->vkey_num; i++) {
        input_report_key(pdata->ts_input_dev,
                        pdata->vkey_keycodes[i], vkey_state & BIT(i) ? 1 : 0);
    }

    pdata->vkey_state = vkey_state;

    return 0;
}

int cts_plat_release_all_vkey(struct cts_platform_data *pdata)
{
    int i;

    cts_info("Release all vkeys");

    for (i = 0; i < pdata->vkey_num; i++) {
        if (pdata->vkey_state & BIT(i)) {
            input_report_key(pdata->ts_input_dev, pdata->vkey_keycodes[i], 0);
        }
    }

    pdata->vkey_state = 0;

    return 0;
}
#endif /* CONFIG_CTS_VIRTUALKEY */

#ifdef CONFIG_CTS_GESTURE

static ssize_t cts_gesture_write(struct file *filp,const char __user * buff, size_t len, loff_t * off)
{	
	s32 ret = 0;   
	unsigned char temp;	
	cts_info("enter cts_gesture_write");
	
	ret = copy_from_user(&temp, buff, 1);	
	if (ret) {		
		return -EPERM;	
	}   
		
	if (temp == '1') {
		cts_enable_gesture_wakeup(&chipone_ts_data->cts_dev);	
	}
	else {
		cts_disable_gesture_wakeup(&chipone_ts_data->cts_dev);	
	}	
	
	return len;
}
	
static const struct file_operations gesture_fops = {	
	.owner = THIS_MODULE,	
	.write = cts_gesture_write,
};

int cts_plat_enable_irq_wake(struct cts_platform_data *pdata)
{
    cts_info("Enable IRQ wake");

    if (pdata->irq > 0) {
        if (!pdata->irq_wake_enabled) {
            pdata->irq_wake_enabled = true;
            return enable_irq_wake(pdata->irq);
        }

        cts_warn("Enable irq wake while already disabled");
        return -EINVAL;
    }

    cts_warn("Enable irq wake while irq invalid %d", pdata->irq);
    return -ENODEV;
}

int cts_plat_disable_irq_wake(struct cts_platform_data *pdata)
{
    cts_info("Disable IRQ wake");

    if (pdata->irq > 0) {
        if (pdata->irq_wake_enabled) {
            pdata->irq_wake_enabled = false;
            return disable_irq_wake(pdata->irq);
        }

        cts_warn("Disable irq wake while already disabled");
        return -EINVAL;
    }

    cts_warn("Disable irq wake while irq invalid %d", pdata->irq);
    return -ENODEV;
}

int cts_plat_init_gesture(struct cts_platform_data *pdata)
{
    int i;
	//struct proc_dir_entry *proc_entry = NULL;

    cts_info("Init gesture");
	
    chipone_ts_data->procgs_entry = proc_create("gesture_enable", 0666, NULL, &gesture_fops);
    cts_disable_gesture_wakeup(&chipone_ts_data->cts_dev);
    // TODO: If system will issure enable/disable command, comment following line.
    //cts_enable_gesture_wakeup(pdata->cts_dev);

    for (i = 0; i < pdata->gesture_num; i ++) {
        input_set_capability(pdata->ts_input_dev, EV_KEY,
            pdata->gesture_keymap[i][1]);
    }

    return 0;
}

void cts_plat_deinit_gesture(struct cts_platform_data *pdata)
{
    cts_info("De-init gesture");
}

int cts_plat_process_gesture_info(struct cts_platform_data *pdata,
    struct cts_device_gesture_info *gesture_info)
{
    int i;

    cts_info("Process gesture, id=0x%02x", gesture_info->gesture_id);

#if defined(CFG_CTS_GESTURE_REPORT_KEY)
    for (i = 0; i < CFG_CTS_NUM_GESTURE; i++) {
        if (gesture_info->gesture_id == pdata->gesture_keymap[i][0]) {
            cts_info("Report key[%u]", pdata->gesture_keymap[i][1]);
            input_report_key(pdata->ts_input_dev,
                pdata->gesture_keymap[i][1], 1);
            input_sync(pdata->ts_input_dev);

            input_report_key(pdata->ts_input_dev,
                pdata->gesture_keymap[i][1], 0);
            input_sync(pdata->ts_input_dev);

            return 0;
        }
    }
#endif /* CFG_CTS_GESTURE_REPORT_KEY */

    cts_warn("Process unrecognized gesture id=%u",
        gesture_info->gesture_id);

    return -EINVAL;
}

#endif /* CONFIG_CTS_GESTURE */

