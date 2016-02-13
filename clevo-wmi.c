/*
 * CLEVO/Notebook WMI Hotkey Driver
 *
 * Copyright (C) 2016 Guillermo A. Amaral B. <g@maral.me>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/acpi.h>
#include <linux/dmi.h>
#include <linux/input.h>
#include <linux/input/sparse-keymap.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#define CLEVO_WMI_FILE "clevo"

#define CLEVO_WMI_EVENT_GUID  "ABBC0F6B-8EA1-11D1-00A0-C90629100000"
#define CLEVO_WMI_METHOD_GUID "ABBC0F6D-8EA1-11D1-00A0-C90629100000"

#define CLEVO_WMI_GCMD_EVNT 0x01
#define CLEVO_WMI_SCMD_HOTKEY_ENABLE 0x46

#define CLEVO_WMI_P65_GCMD_A2_POWER 0x05
#define CLEVO_WMI_P65_GCMD_WEBCAM_POWER 0x06
#define CLEVO_WMI_P65_GCMD_BLUETOOTH_POWER 0x07
#define CLEVO_WMI_P65_GCMD_TRACKPAD_POWER 0x09
#define CLEVO_WMI_P65_GCMD_A4_POWER 0x0A
#define CLEVO_WMI_P65_GCMD_A7_POWER 0x11
#define CLEVO_WMI_P65_GCMD_KB_BACKLIGHT 0x3D
#define CLEVO_WMI_P65_GCMD_VGA 0x54 /* 0x00 = DESCRETE, 0xFF = MSHYBRID */
#define CLEVO_WMI_P65_GCMD_EC 0x73

#define CLEVO_WMI_P65_SCMD_A2_POWER 0x20
#define CLEVO_WMI_P65_SCMD_BLUETOOTH_POWER 0x21
#define CLEVO_WMI_P65_SCMD_WEBCAM_POWER 0x22
#define CLEVO_WMI_P65_SCMD_KB_BACKLIGHT 0x27
#define CLEVO_WMI_P65_SCMD_TRACKPAD_POWER 0x2A
#define CLEVO_WMI_P65_SCMD_IGNR_AC 0x48
#define CLEVO_WMI_P65_SCMD_POWER_BTN 0x49
#define CLEVO_WMI_P65_SCMD_A4_POWER 0x4C
#define CLEVO_WMI_P65_SCMD_LCD_POWER 0x5E
#define CLEVO_WMI_P65_SCMD_FAN_CONTROL 0x65
#define CLEVO_WMI_P65_SCMD_PER_FAN_SPEED 0x68 /* Seperate fan speed values (set fans to manual) */
#define CLEVO_WMI_P65_SCMD_PER_FAN_AUTO 0x69 /* Reset fan speed control to automatic (0b1111) */
#define CLEVO_WMI_P65_SCMD_INDX 0x6B
#define CLEVO_WMI_P65_SCMD_AIRPLANE_LED 0x6C
#define CLEVO_WMI_P65_SCMD_EC 0x75
#define CLEVO_WMI_P65_SCMD_ACC 0x79

/*
 * Workaround for P65, AIRP seems to flip the wrong bits.
 */
#define CLEVO_WMI_P65_OFFSET_AIRP 0xD9

MODULE_AUTHOR("Guillermo A. Amaral B. <g@maral.me>");
MODULE_DESCRIPTION("CLEVO/Notebook Hotkey Driver.");
MODULE_LICENSE("GPL");
MODULE_ALIAS("wmi:" CLEVO_WMI_EVENT_GUID);

enum p65_accessory_cmd_t {
	P65_ACCESSORY_FAN_CONTROL   = 0x01,
	P65_ACCESSORY_FAN_SPEED     = 0x0E,
	P65_ACCESSORY_HEADPHONE_AMP = 0x0D,
	P65_ACCESSORY_MSHYBRID      = 0x0B,
	P65_ACCESSORY_WINDOWS_KEY   = 0x05
};

enum p65_keyboard_brightness_level_t {
	P65_KEYBOARD_BRIGHTNESS_OFF = 0,
	P65_KEYBOARD_BRIGHTNESS_20  = 1,
	P65_KEYBOARD_BRIGHTNESS_40  = 2,
	P65_KEYBOARD_BRIGHTNESS_60  = 3,
	P65_KEYBOARD_BRIGHTNESS_80  = 4,
	P65_KEYBOARD_BRIGHTNESS_MAX = 5
};

enum p65_fan_control_cmd_t {
	P65_FAN_CONTROL_NORMAL = 0,
	P65_FAN_CONTROL_FULL   = 1,
	P65_FAN_CONTROL_XTUF   = 2,
	_P65_FAN_CONTROL_MASK  = 0x03
};

enum p65_headphone_amp_cmd_t {
	P65_HEADPHONE_AMP_OFF      = 0,
	P65_HEADPHONE_AMP_NORMAL   = 1,
	P65_HEADPHONE_AMP_PRESET1  = 2,
	P65_HEADPHONE_AMP_PRESET2  = 3,
	P65_HEADPHONE_AMP_PRESET3  = 4, /* BOOST */
	P65_HEADPHONE_AMP_PRESET4  = 5,
	P65_HEADPHONE_AMP_RESET    = 6,
	_P65_HEADPHONE_AMP_DEFAULT = P65_HEADPHONE_AMP_PRESET3,
	_P65_HEADPHONE_AMP_MAX     = P65_HEADPHONE_AMP_RESET
};

static int _clevo_p65_deinit(struct platform_device *);
static int _clevo_p65_init(struct platform_device *);
static int _clevo_p65_resume(struct platform_device *);
static ssize_t _clevo_p65_show_attr(struct device *, struct device_attribute *, char *);
static ssize_t _clevo_p65_store_attr(struct device *, struct device_attribute *, const char *, size_t);
static enum led_brightness _clevo_p65_led_get(struct led_classdev *);
static void _clevo_p65_led_set(struct led_classdev *, enum led_brightness);
static void _clevo_p65_led_update(struct work_struct *);

/*
 * XXX: Model abstraction is still a work in progress.
 *
 * I only have one CLEVO laptop so it's gonna take a while to sort out the
 * common bits.
 */
static int __clevo_p65_airplane_led_set(bool);
static int __clevo_p65_fan_control_set(u8);
static int __clevo_p65_fan_speed_set(u8);
static int __clevo_p65_headphone_amp_set(u8);
static int __clevo_p65_keyboard_brightness_get(u8 *);
static int __clevo_p65_keyboard_brightness_set(u8);

static int _clevo_platform_match(const struct dmi_system_id *);
static int _clevo_wmi_bool_get(int, bool *);
static int _clevo_wmi_bool_set(int, bool);
static int _clevo_wmi_evaluate_method(u32, u32, u32 *);
static void _clevo_wmi_notify(u32, void *);
static enum led_brightness _clevo_led_get(struct led_classdev *);
static void _clevo_led_set(struct led_classdev *, enum led_brightness);
static void _clevo_led_update(struct work_struct *);

static void __exit clevo_wmi_exit(void);
static int __init_or_module clevo_wmi_init(void);
static int clevo_wmi_probe(struct platform_device *);
static int clevo_wmi_remove(struct platform_device *);
static int clevo_wmi_resume(struct platform_device *);

/****************************************************************************/

struct clevo_wmi_model_t
{
	int event_id;
	const struct key_entry *keymap;

	int (*deinit)(struct platform_device *pdev);
	int (*init)(struct platform_device *pdev);
	int (*resume)(struct platform_device *pdev);

	enum led_brightness (*led_get)(struct led_classdev *);
	void (*led_set)(struct led_classdev *, enum led_brightness);
	void (*led_update)(struct work_struct *);
};

struct clevo_t
{
	const struct clevo_wmi_model_t *model;
	struct platform_device *pdev;
	struct input_dev *idev;

	/* common leds */
	struct workqueue_struct *led_workqueue;
	struct led_classdev airp_led;
	struct work_struct airp_work;
};
static struct clevo_t s_clevo;

/*
 * TODO: Replace show and store functions with wrapper that calls model
 * specific versions.
 */
DEVICE_ATTR(bluetooth_power, S_IWUSR | S_IRUGO,
    _clevo_p65_show_attr, _clevo_p65_store_attr);
DEVICE_ATTR(headphone_amp, S_IWUSR | S_IRUGO,
    _clevo_p65_show_attr, _clevo_p65_store_attr);
DEVICE_ATTR(keyboard_backlight, S_IWUSR | S_IRUGO,
    _clevo_p65_show_attr, _clevo_p65_store_attr);
DEVICE_ATTR(trackpad_power, S_IWUSR | S_IRUGO,
    _clevo_p65_show_attr, _clevo_p65_store_attr);
DEVICE_ATTR(webcam_power, S_IWUSR | S_IRUGO,
    _clevo_p65_show_attr, _clevo_p65_store_attr);
DEVICE_ATTR(lcd_power, S_IWUSR, NULL, _clevo_p65_store_attr);

/* CLEVO P65 ****************************************************************/

struct clevo_p65_state_t
{
	int headphone_amp;
	bool airplane_led;
};

static const struct key_entry s_clevo_p65_keymap[] = {
	{ KE_KEY, 0x95, { KEY_PROG1 } },  /* Fn+ESC (Control Center) */
	{ KE_KEY, 0x7B, { KEY_PROG2 } },  /* Fn+Backspace (Flexikey) */
	{ KE_KEY, 0x7D, { KEY_RFKILL } }, /* Fn+2 */
	{ KE_KEY, 0x7E, { KEY_RFKILL } }, /* Fn+2 */
	{ KE_END, 0},
};

static struct clevo_wmi_model_t s_clevo_p65_model = {
	.event_id = 0xD0,
	.keymap = s_clevo_p65_keymap,

	.deinit = _clevo_p65_deinit,
	.init = _clevo_p65_init,
	.resume = _clevo_p65_resume,

	.led_set = _clevo_p65_led_set,
	.led_get = _clevo_p65_led_get,
	.led_update = _clevo_p65_led_update
};

static struct attribute * s_clevo_p65_attributes[] = {
	&dev_attr_bluetooth_power.attr,
	&dev_attr_headphone_amp.attr,
	&dev_attr_keyboard_backlight.attr,
	&dev_attr_lcd_power.attr,
	&dev_attr_trackpad_power.attr,
	&dev_attr_webcam_power.attr,
	NULL
};

static struct attribute_group s_clevo_attribute_group = {
	.attrs = s_clevo_p65_attributes
};

/****************************************************************************/

static struct platform_driver __initdata_or_module s_clevo_platform_driver = {
	.remove = clevo_wmi_remove,
	.resume = clevo_wmi_resume,
	.driver = {
		.name  = CLEVO_WMI_FILE,
		.owner = THIS_MODULE,
	}
};

static struct dmi_system_id __initdata_or_module clevo_dmi_table[] = {
	{
		.ident = "P65",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Notebook"),
			DMI_MATCH(DMI_PRODUCT_NAME, "P65_"),
		},
		.callback = _clevo_platform_match,
		.driver_data = &s_clevo_p65_model
	},
	{}
};
MODULE_DEVICE_TABLE(dmi, clevo_dmi_table);

/* CLEVO P65 ****************************************************************/

int
_clevo_p65_deinit(struct platform_device *pdev)
{
	struct clevo_p65_state_t *p65_state = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &s_clevo_attribute_group);

	kfree(p65_state);
	return 0;
}

int
_clevo_p65_init(struct platform_device *pdev)
{
	struct clevo_p65_state_t *p65_state;

	p65_state = kzalloc(sizeof(struct clevo_p65_state_t), GFP_KERNEL);
	p65_state->airplane_led = false;
	p65_state->headphone_amp = _P65_HEADPHONE_AMP_DEFAULT;
	platform_set_drvdata(pdev, p65_state);

	__clevo_p65_airplane_led_set(p65_state->airplane_led);
	__clevo_p65_headphone_amp_set(p65_state->headphone_amp);

	return sysfs_create_group(&pdev->dev.kobj, &s_clevo_attribute_group);
}

int
_clevo_p65_resume(struct platform_device *pdev)
{
	struct clevo_p65_state_t *p65_state = platform_get_drvdata(pdev);

	__clevo_p65_headphone_amp_set(p65_state->headphone_amp);

	return 0;
}

ssize_t
_clevo_p65_show_attr(struct device *dev,
    struct device_attribute *attr, char *buf)
{
	struct clevo_p65_state_t *p65_state = dev_get_drvdata(dev);
	bool tmpb;
	u8 tmp8;

	if (attr == &dev_attr_bluetooth_power) {
		if (_clevo_wmi_bool_get(CLEVO_WMI_P65_GCMD_BLUETOOTH_POWER,
		    &tmpb))
			return -EIO;

		return sprintf(buf, "%i", tmpb ? 1 : 0);
	}
	else if (attr == &dev_attr_headphone_amp) {
		return sprintf(buf, "%i", p65_state->headphone_amp);
	}
	else if (attr == &dev_attr_keyboard_backlight) {
		if (__clevo_p65_keyboard_brightness_get(&tmp8))
			return -EIO;

		return sprintf(buf, "%i", tmp8);
	}
	else if (attr == &dev_attr_trackpad_power) {
		if (_clevo_wmi_bool_get(CLEVO_WMI_P65_GCMD_TRACKPAD_POWER,
		    &tmpb))
			return -EIO;

		return sprintf(buf, "%i", tmpb ? 1 : 0);
	}
	else if (attr == &dev_attr_webcam_power) {
		if (_clevo_wmi_bool_get(CLEVO_WMI_P65_GCMD_WEBCAM_POWER,
		    &tmpb))
			return -EIO;

		return sprintf(buf, "%i", tmpb ? 1 : 0);
	}

	return 0;
}

ssize_t
_clevo_p65_store_attr(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
	struct clevo_p65_state_t *p65_state = dev_get_drvdata(dev);
	int val;

	if (!count) return 0;

	if (sscanf(buf, "%i", &val) != 1)
		return -EINVAL;

	if (attr == &dev_attr_bluetooth_power) {
		if (_clevo_wmi_bool_set(CLEVO_WMI_P65_SCMD_BLUETOOTH_POWER,
		    val == 1))
			return -EIO;
	}
	else if (attr == &dev_attr_headphone_amp) {
		if (__clevo_p65_headphone_amp_set(val))
			return -EIO;

		p65_state->headphone_amp = val;
	}
	else if (attr == &dev_attr_keyboard_backlight) {
		if (__clevo_p65_keyboard_brightness_set(val))
			return -EIO;
	}
	else if (attr == &dev_attr_lcd_power) {
		if (_clevo_wmi_bool_set(CLEVO_WMI_P65_SCMD_LCD_POWER,
		    val == 1))
			return -EIO;
	}
	else if (attr == &dev_attr_trackpad_power) {
		if (_clevo_wmi_bool_set(CLEVO_WMI_P65_SCMD_TRACKPAD_POWER,
		    val == 1))
			return -EIO;
	}
	else if (attr == &dev_attr_webcam_power) {
		if (_clevo_wmi_bool_set(CLEVO_WMI_P65_SCMD_WEBCAM_POWER,
		    val == 1))
			return -EIO;
	}

	return count;
}

enum led_brightness
_clevo_p65_led_get(struct led_classdev *led)
{
	struct clevo_p65_state_t *p65_state =
	    platform_get_drvdata(s_clevo.pdev);

	if (led == &s_clevo.airp_led)
		return p65_state->airplane_led ? LED_FULL : LED_OFF;

	return LED_OFF;
}

void
_clevo_p65_led_set(struct led_classdev *led, enum led_brightness value)
{
	struct clevo_p65_state_t *p65_state =
	    platform_get_drvdata(s_clevo.pdev);

	if (led == &s_clevo.airp_led) {
		p65_state->airplane_led = (value != LED_OFF);
		queue_work(s_clevo.led_workqueue, &s_clevo.airp_work);
	}
}

void
_clevo_p65_led_update(struct work_struct *work)
{
	struct clevo_p65_state_t *p65_state =
	    platform_get_drvdata(s_clevo.pdev);

	if (work == &s_clevo.airp_work) {
		/* inverted, rfkill trigger treats it as a WLAN-ON led. */
		__clevo_p65_airplane_led_set(!p65_state->airplane_led);
	}
}

int
__clevo_p65_airplane_led_set(bool on)
{
	u32 state = 0;

	/* This seems to pretty much do nothing.
	 *
	 * You can adjust the DSDT to update the correct offset to toggle the
	 * LED, but it's pretty much pointless. */
	_clevo_wmi_evaluate_method(CLEVO_WMI_P65_SCMD_AIRPLANE_LED, on ? 1 : 0, NULL);

	/*
	 * Update correct offset
	 */
	if (_clevo_wmi_evaluate_method(CLEVO_WMI_P65_SCMD_INDX,
	    CLEVO_WMI_P65_OFFSET_AIRP, NULL))
		return -EIO;

	if(_clevo_wmi_evaluate_method(CLEVO_WMI_P65_GCMD_EC, 0, &state))
		return -EIO;

	if (on) state |= (1 << 6);
	else state &= ~(1 << 6);

	if (_clevo_wmi_evaluate_method(CLEVO_WMI_P65_SCMD_EC, state, NULL))
		return -EIO;

	return 0;
}

int
__clevo_p65_fan_control_set(u8 ctrl)
{
	if (_clevo_wmi_evaluate_method(CLEVO_WMI_P65_SCMD_ACC,
	    (P65_ACCESSORY_FAN_CONTROL << 24)
	      | (ctrl & _P65_FAN_CONTROL_MASK),
	    NULL))
		return -EIO;

	return 0;
}

int
__clevo_p65_fan_speed_set(u8 speed)
{
	if (_clevo_wmi_evaluate_method(CLEVO_WMI_P65_SCMD_ACC,
	    (P65_ACCESSORY_FAN_SPEED << 24) | speed, NULL))
		return -EIO;

	return 0;
}

int
__clevo_p65_headphone_amp_set(u8 cmd)
{
	if (cmd > _P65_HEADPHONE_AMP_MAX)
		return -EINVAL;

	if (_clevo_wmi_evaluate_method(CLEVO_WMI_P65_SCMD_ACC,
	    (P65_ACCESSORY_HEADPHONE_AMP << 24) | cmd, NULL))
		return -EIO;

	return 0;
}

int
__clevo_p65_keyboard_brightness_get(u8 *level)
{
	u32 val;

	if (_clevo_wmi_evaluate_method(CLEVO_WMI_P65_GCMD_KB_BACKLIGHT,
	    0, &val))
		return -EIO;

	*level = val;

	return 0;
}

int
__clevo_p65_keyboard_brightness_set(u8 level)
{
	if (level > P65_KEYBOARD_BRIGHTNESS_MAX)
		return -EINVAL;

	if (_clevo_wmi_evaluate_method(CLEVO_WMI_P65_SCMD_KB_BACKLIGHT,
	    level, NULL))
		return -EIO;

	return 0;
}

/****************************************************************************/

int
_clevo_platform_match(const struct dmi_system_id *id)
{
	pr_info("CLEVO WMI detected a %s notebook.\n", id->ident);
	s_clevo.model = id->driver_data;
	return 1;
}

int
_clevo_wmi_bool_get(int code, bool *val)
{
	u32 tmp = 0;

	if (_clevo_wmi_evaluate_method(code, 0, &tmp))
		return -EIO;

	*val = tmp;

	return 0;
}

int
_clevo_wmi_bool_set(int code, bool on)
{
	if (_clevo_wmi_evaluate_method(code, on ? 1 : 0, NULL))
		return -EIO;

	return 0;
}

int
_clevo_wmi_evaluate_method(u32 method, u32 arg, u32 *ret)
{
	struct acpi_buffer input = { (acpi_size) sizeof(u32), &arg };
	struct acpi_buffer output = { ACPI_ALLOCATE_BUFFER, NULL };
	union acpi_object *obj;
	acpi_status status;
	u32 tmp = 0;

	status = wmi_evaluate_method(CLEVO_WMI_METHOD_GUID, 1,
	    method, &input, &output);
	if (ACPI_FAILURE(status))
		return -EIO;

	obj = (union acpi_object *) output.pointer;
	if (likely(obj)) {
		if (likely(obj->type == ACPI_TYPE_INTEGER))
			tmp = (u32) obj->integer.value;
		else if (obj->type == ACPI_TYPE_BUFFER)
			memcpy(&tmp, obj->buffer.pointer, 4);
	}

	if (ret) *ret = tmp;

	kfree(obj);

	return 0;
}

void
_clevo_wmi_notify(u32 value, void *context)
{
	acpi_status status;
	u32 code;

	if (s_clevo.model && s_clevo.model->event_id != value)
		return;

	code = 0;
	status = _clevo_wmi_evaluate_method(CLEVO_WMI_GCMD_EVNT, 0, &code);
	if (unlikely(status != AE_OK)) {
		pr_err("CLEVO WMI failed to receive code for event.\n");
		return;
	}

	if (!sparse_keymap_report_event(s_clevo.idev, code, 1, 1))
		pr_info("CLEVO WMI event '%x' went unhandled.\n", code);
}

enum led_brightness
_clevo_led_get(struct led_classdev *led)
{
	if (unlikely(!s_clevo.model || !s_clevo.model->led_get))
		return LED_OFF;

	return s_clevo.model->led_get(led);
}

void
_clevo_led_set(struct led_classdev *led, enum led_brightness value)
{
	if (unlikely(!s_clevo.model || !s_clevo.model->led_set))
		return;

	s_clevo.model->led_set(led, value);
}

void
_clevo_led_update(struct work_struct *work)
{
	if (unlikely(!s_clevo.model || !s_clevo.model->led_update))
		return;

	s_clevo.model->led_update(work);
}

/****************************************************************************/

int
clevo_wmi_init(void)
{
	int errno = 0;

	struct platform_device *platform_device;

	s_clevo.model = NULL;

	platform_device = platform_create_bundle(
	    &s_clevo_platform_driver,
	    clevo_wmi_probe,
	    NULL, 0, NULL, 0);
	if (IS_ERR(platform_device))
		return PTR_ERR(platform_device);

	s_clevo.pdev = platform_device;

	/* setup input device */

	s_clevo.idev = input_allocate_device();
	if (IS_ERR_OR_NULL(s_clevo.idev)) {
		errno = -ENODEV;
		goto error;
	}

	s_clevo.idev->name = "CLEVO/Notebook Hotkey Input";
	s_clevo.idev->phys = CLEVO_WMI_FILE "/input0";
	s_clevo.idev->id.bustype = BUS_HOST;
	s_clevo.idev->dev.parent = &platform_device->dev;

	set_bit(EV_KEY, s_clevo.idev->evbit);
	set_bit(KEY_PROG1, s_clevo.idev->keybit);
	set_bit(KEY_PROG2, s_clevo.idev->keybit);
	set_bit(KEY_RFKILL, s_clevo.idev->keybit);

	errno = input_register_device(s_clevo.idev);
	if (unlikely(errno)) goto error_idev;

	if (likely(s_clevo.model && s_clevo.model->keymap)) {
		errno = sparse_keymap_setup(s_clevo.idev,
		    s_clevo.model->keymap, NULL);
		if (unlikely(errno)) goto error_idev;
	}

	s_clevo.led_workqueue =
	    create_singlethread_workqueue("led_workqueue");
	if (IS_ERR_OR_NULL(s_clevo.led_workqueue)) {
		errno = -ENOMEM;
		goto error_keymap;
	}

	if (likely(s_clevo.model->led_get &&
	           s_clevo.model->led_set &&
	           s_clevo.model->led_update)) {
		INIT_WORK(&s_clevo.airp_work, _clevo_led_update);

		s_clevo.airp_led.name = "clevo:green:airplane_mode";
		s_clevo.airp_led.brightness_get = _clevo_led_get;
		s_clevo.airp_led.brightness_set = _clevo_led_set;
		s_clevo.airp_led.default_trigger = "rfkill1";
		s_clevo.airp_led.max_brightness = LED_FULL;

		errno = led_classdev_register(&s_clevo.pdev->dev, &s_clevo.airp_led);
		if (unlikely(errno)) goto error_led_workqueue;
	}

	return 0;

error_led_workqueue:
	destroy_workqueue(s_clevo.led_workqueue);

error_keymap:
	sparse_keymap_free(s_clevo.idev);

error_idev:
	input_free_device(s_clevo.idev);
	s_clevo.idev = NULL;

error:
	platform_device_unregister(platform_device);
	platform_driver_unregister(&s_clevo_platform_driver);
	s_clevo.pdev = NULL;

	return errno;
}

void
clevo_wmi_exit(void)
{
	if (likely(s_clevo.airp_led.dev))
		led_classdev_unregister(&s_clevo.airp_led);

	if (likely(s_clevo.led_workqueue))
		destroy_workqueue(s_clevo.led_workqueue);

	if (likely(s_clevo.model && s_clevo.model->keymap))
		sparse_keymap_free(s_clevo.idev);

	input_unregister_device(s_clevo.idev);
	input_free_device(s_clevo.idev);
	s_clevo.idev = NULL;

	platform_device_unregister(s_clevo.pdev);
	platform_driver_unregister(&s_clevo_platform_driver);
	s_clevo.pdev = NULL;
}

int
clevo_wmi_probe(struct platform_device *pdev)
{
	int ret;

	if (!wmi_has_guid(CLEVO_WMI_METHOD_GUID) &&
	    !wmi_has_guid(CLEVO_WMI_EVENT_GUID)) {
	    pr_err("CLEVO WMI required GUIDs where not found.\n");
	    return -ENODEV;
	}

	if (!dmi_check_system(clevo_dmi_table))
	    pr_warning("CLEVO WMI unknown laptop encountered.\n");

	ret = wmi_install_notify_handler(CLEVO_WMI_EVENT_GUID,
	    _clevo_wmi_notify, NULL);
	if (ACPI_FAILURE(ret)) {
		pr_err("CLEVO WMI was unable to install notification handler.\n");
		return -ENODEV;
	}

	_clevo_wmi_evaluate_method(CLEVO_WMI_SCMD_HOTKEY_ENABLE, 0, NULL);

	if (likely(s_clevo.model && s_clevo.model->init))
		return s_clevo.model->init(pdev);
	
	return 0;
}

int
clevo_wmi_remove(struct platform_device *pdev)
{
	wmi_remove_notify_handler(CLEVO_WMI_EVENT_GUID);

	if (likely(s_clevo.model && s_clevo.model->deinit))
		s_clevo.model->deinit(pdev);

	return 0;
}

int
clevo_wmi_resume(struct platform_device *pdev)
{
	_clevo_wmi_evaluate_method(CLEVO_WMI_SCMD_HOTKEY_ENABLE, 0, NULL);

	if (likely(s_clevo.model && s_clevo.model->resume))
		s_clevo.model->resume(pdev);

	return 0;
}

module_init(clevo_wmi_init);
module_exit(clevo_wmi_exit);
