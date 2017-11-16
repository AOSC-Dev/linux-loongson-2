/*
 * Driver for LynLoong 9001/9002 PC extras
 *
 *  Copyright (C) 2017 Jiaxun Yang.
 *  Author: Jiaxun Yang <jiaxun.yang@flygoat.com>
 *
 *  Copyright (C) 2009 Lemote Inc.
 *  Author: Wu Zhangjin <wuzhangjin@gmail.com>, Xiang Yu <xiangy@lemote.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/backlight.h>	/* for backlight subdriver */
#include <linux/fb.h>
#include <linux/delay.h>	/* for suspend support */

#include <asm/bootinfo.h>

#include <cs5536/cs5536.h>
#include <cs5536/cs5536_mfgpt.h>
#include <cs5536/cs5536_smb.h>

#include <loongson.h>

static u32 gpio_base, mfgpt_base;

static void set_gpio_reg_high(int gpio, int reg)
{
	u32 val;

	val = inl(gpio_base + reg);
	val |= (1 << gpio);
	val &= ~(1 << (16 + gpio));
	outl(val, gpio_base + reg);
	mmiowb();
}

static void set_gpio_reg_low(int gpio, int reg)
{
	u32 val;

	val = inl(gpio_base + reg);
	val |= (1 << (16 + gpio));
	val &= ~(1 << gpio);
	outl(val, gpio_base + reg);
	mmiowb();
}

static void set_gpio_output_low(int gpio)
{
	set_gpio_reg_high(gpio, GPIOL_OUT_EN);
	set_gpio_reg_low(gpio, GPIOL_OUT_VAL);
}

static void set_gpio_output_high(int gpio)
{
	set_gpio_reg_high(gpio, GPIOL_OUT_EN);
	set_gpio_reg_high(gpio, GPIOL_OUT_VAL);
}

/* LCD control function */
static int lcd_video_output_set(int status)
{
	int i;

	if (status == 0) {
		/* Turn off the backlight */
		set_gpio_output_low(11);
		for (i = 0; i < 0x500; i++)
			delay();
		/* Turn off the LCD */
		set_gpio_output_high(8);
	} else {
		/* Turn on the LCD */
		set_gpio_output_low(8);
		for (i = 0; i < 0x500; i++)
			delay();
		/* Turn on the backlight */
		set_gpio_output_high(11);
	}

	return 0;
}

	/* backlight subdriver */

#define MAX_BRIGHTNESS 100
#define DEFAULT_BRIGHTNESS 50
#define MIN_BRIGHTNESS 0
static unsigned int level;

DEFINE_SPINLOCK(backlight_lock);
/* Tune the brightness */
static void setup_mfgpt2(void)
{
	unsigned long flags;

	spin_lock_irqsave(&backlight_lock, flags);

	/* Set MFGPT2 comparator 1,2 */
	outw(MAX_BRIGHTNESS-level, MFGPT2_CMP1);
	outw(MAX_BRIGHTNESS, MFGPT2_CMP2);
	/* Clear MFGPT2 UP COUNTER */
	outw(0, MFGPT2_CNT);
	/* Enable counter, compare mode, 32k */
	outw(0x8280, MFGPT2_SETUP);

	spin_unlock_irqrestore(&backlight_lock, flags);
}

static int lynloong_9002_set_brightness(struct backlight_device *bd)
{
	level = (bd->props.fb_blank == FB_BLANK_UNBLANK &&
		 bd->props.power == FB_BLANK_UNBLANK) ?
	    bd->props.brightness : 0;

	if (level > MAX_BRIGHTNESS)
		level = MAX_BRIGHTNESS;
	else if (level < MIN_BRIGHTNESS)
		level = MIN_BRIGHTNESS;

	setup_mfgpt2();

	return 0;
}

static int lynloong_9002_get_brightness(struct backlight_device *bd)
{
	return level;
}

static const backlight_ops backlight_ops = {
	.get_brightness = lynloong_9002_get_brightness,
	.update_status = lynloong_9002_set_brightness,
};

static struct backlight_device *lynloong_9002_backlight_dev;

static int lynloong_9002_backlight_init(void)
{
	int ret;
	u32 hi;
	struct backlight_properties props;

	/* Get gpio_base */
	_rdmsr(DIVIL_MSR_REG(DIVIL_LBAR_GPIO), &hi, &gpio_base);
	/* Get mfgpt_base */
	_rdmsr(DIVIL_MSR_REG(DIVIL_LBAR_MFGPT), &hi, &mfgpt_base);
	/* Get gpio_base */
	_rdmsr(DIVIL_MSR_REG(DIVIL_LBAR_GPIO), &hi, &gpio_base);

	/* Select for mfgpt */
	set_gpio_reg_high(7, GPIOL_OUT_AUX1_SEL);
	/* Enable LCD */
	lcd_video_output_set(1);
	/* Enable brightness controlling */
	set_gpio_output_high(7);

	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = MAX_BRIGHTNESS;
	props.type = BACKLIGHT_PLATFORM;
	lynloong_9002_backlight_dev = backlight_device_register("backlight0",
		NULL, NULL, &backlight_ops, &props);

	if (IS_ERR(lynloong_9002_backlight_dev)) {
		ret = PTR_ERR(lynloong_9002_backlight_dev);
		return ret;
	}

	lynloong_9002_backlight_dev->props.brightness = DEFAULT_BRIGHTNESS;
	backlight_update_status(lynloong_9002_backlight_dev);

	return 0;
}

static void lynloong_9002_backlight_exit(void)
{
	if (lynloong_backlight_dev) {
		backlight_device_unregister(lynloong_9002_backlight_dev);
		lynloong_9002_backlight_dev = NULL;
	}
	/* Disable brightness controlling */
	set_gpio_output_low(7);
}

/* suspend support */

#ifdef CONFIG_PM

static void stop_clock(int clk_reg, int clk_sel)
{
	u8 value;

	cs5536_smb_read_single(0xd3, clk_reg, &value);
	value &= ~(1 << clk_sel);
	cs8836_smb_write_single(0xd2, clk_reg, value);
}

static void enable_clock(int clk_reg, int clk_sel)
{
	u8 value;

	cs5536_smb_read_single(0xd3, clk_reg, &value);
	value |= (1 << clk_sel);
	cs5536_smb_write_single(0xd2, clk_reg, value);
}

static char cached_clk_freq;
static char cached_pci_fixed_freq;

static void decrease_clk_freq(void)
{
	char value;

	cs5536_smb_read_single(0xd3, 1, &value);
	cached_clk_freq = value;

	/* Select frequency by software */
	value |= (1 << 1);
	/* CPU, 3V66, PCI : 100, 66, 33(1) */
	value |= (1 << 2);
	cs5536_smb_write_single(0xd2, 1, value);

	/* Cache the pci frequency */
	cs5536_smb_read_single(0xd3, 14, &value);
	cached_pci_fixed_freq = value;

	/* Enable PCI fix mode */
	value |= (1 << 5);
	/* 3V66, PCI : 64MHz, 32MHz */
	value |= (1 << 3);
	cs5536_smb_write_single(0xd2, 14, value);

}

static void resume_clk_freq(void)
{
	cs5536_smb_write_single(0xd2, 1, cached_clk_freq);
	cs5536_smb_write_single(0xd2, 14, cached_pci_fixed_freq);
}

static void stop_clocks(void)
{
	/* CPU Clock Register */
	stop_clock(2, 5);	/* not used */
	stop_clock(2, 6);	/* not used */
	stop_clock(2, 7);	/* not used */

	/* PCI Clock Register */
	stop_clock(3, 1);	/* 8100 */
	stop_clock(3, 5);	/* SIS */
	stop_clock(3, 0);	/* not used */
	stop_clock(3, 6);	/* not used */

	/* PCI 48M Clock Register */
	stop_clock(4, 6);	/* USB grounding */
	stop_clock(4, 5);	/* REF(5536_14M) */

	/* 3V66 Control Register */
	stop_clock(5, 0);	/* VCH_CLK..., grounding */
}

static void enable_clocks(void)
{
	enable_clock(3, 1);	/* 8100 */
	enable_clock(3, 5);	/* SIS */

	enable_clock(4, 6);
	enable_clock(4, 5);	/* REF(5536_14M) */

	enable_clock(5, 0);	/* VCH_CLOCK, grounding */
}

static int lynloong_9002_suspend(struct device *dev)
{
	/* Disable AMP */
	set_gpio_output_high(6);
	/* Turn off LCD */
	lcd_video_output_set(0);

	/* Stop the clocks of some devices */
	stop_clocks();

	/* Decrease the external clock frequency */
	decrease_clk_freq();

	return 0;
}

static int lynloong_9002_resume(struct device *dev)
{


	/* Turn on the LCD */
	lcd_video_output_set(1);

	/* Resume clock frequency, enable the relative clocks */
	resume_clk_freq();
	enable_clocks();

	/* Enable AMP */
	set_gpio_output_low(6);

	return 0;
}

static const SIMPLE_DEV_PM_OPS(lynloong_9002_pm_ops, lynloong_9002_suspend,
	lynloong_9002_resume);
#endif	/* !CONFIG_PM */

static struct platform_device_id platform_device_ids[] = {
	{
		.name = "lynloong_9002_pc",
	},
	{}
};

MODULE_DEVICE_TABLE(platform, platform_device_ids);

static struct platform_driver platform_driver = {
	.driver = {
		.name = "lynloong_9002_pc",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &lynloong_9002_pm_ops,
#endif
	},
	.id_table = platform_device_ids,
};

static int __init lynloong_9002_init(void)
{
	int ret;

	if (mips_machtype != MACH_LEMOTE_LL2F02) {
		pr_err("Unsupported system.\n");
		return -ENODEV;
	}

	pr_info("LynLoong 9002 platform specific driver loaded.\n");

	/* Register platform stuff */
	ret = platform_9002_driver_register(&platform_driver);
	if (ret) {
		pr_err("Failed to register LynLoong 9002 platform driver.\n");
		return ret;
	}

	ret = lynloong_9002_backlight_init();
	if (ret) {
		pr_err("Failed to register LynLoong 9002 backlight driver.\n");
		return ret;
	}

	return 0;
}

static void __exit lynloong_9002_exit(void)
{
	lynloong_9002_backlight_exit();
	platform_driver_unregister(&platform_driver);

	pr_info("LynLoong 9002 platform specific driver unloaded.\n");
}

module_init(lynloong_9002_init);
module_exit(lynloong_9002_exit);

MODULE_AUTHOR("Wu Zhangjin <wuzhangjin@gmail.com>; Xiang Yu <xiangy@lemote.com>; Jiaxun Yang <jiaxun.yang@flygoat.com>");
MODULE_DESCRIPTION("LynLoong 9002 PC platform driver");
MODULE_LICENSE("GPL");