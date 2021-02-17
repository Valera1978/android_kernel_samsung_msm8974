/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/qpnp/pwm.h>
#include <linux/clk.h>
#include <linux/spinlock_types.h>
#include <linux/kthread.h>
#include <asm/system.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/dma.h>

#include "mdss.h"
#include "mdss_edp.h"
#include "mdss_debug.h"
#include <linux/qpnp/pin.h>
#if defined(CONFIG_EDP_TCON_MDNIE)
#include "edp_tcon_mdnie.h"
#include <linux/ctype.h>
#include <asm/div64.h>
#endif

#define MAX_PWM_RESOLUTION 511
#define BIT_SHIFT 22

static int duty_level_table[256] = {
4, 4, 4, 8, 8, 
8, 12, 16, 16, 16, 
20, 20, 24, 24, 28, 
28, 32, 32, 36, 36, 
40, 40, 44, 44, 48, 
48, 52, 52, 56, 56, 
60, 60, 64, 64, 68, 
68, 72, 72, 76, 76, 
80, 80, 84, 84, 88, 
88, 92, 92, 96, 96, 
100, 100, 104, 104, 108, 
108, 112, 112, 116, 116, 
120, 120, 124, 124, 128, 
128, 132, 132, 136, 136, 
140, 140, 144, 144, 148, 
148, 152, 152, 156, 156, 
160, 160, 164, 164, 168, 
168, 172, 172, 176, 176, 
180, 180, 184, 184, 188, 
188, 192, 192, 196, 196, 
200, 200, 204, 204, 208, 
208, 212, 212, 216, 216, 
220, 220, 226, 226, 226, 
230, 234, 234, 238, 238, 
242, 242, 246, 246, 250, 
250, 254, 254, 258, 258, 
262, 262, 266, 266, 270, 
270, 274, 274, 278, 278, 
282, 282, 286, 286, 290, 
290, 294, 294, 298, 298, 
302, 302, 306, 306, 310, 
310, 314, 314, 318, 318, 
322, 322, 326, 326, 330, 
330, 334, 334, 338, 338, 
342, 342, 346, 346, 350, 
350, 354, 354, 358, 358, 
362, 362, 366, 366, 370, 
370, 374, 374, 378, 378, 
382, 382, 386, 386, 390, 
390, 394, 394, 398, 398, 
402, 402, 406, 406, 410, 
410, 414, 414, 418, 418, 
422, 422, 426, 426, 430, 
430, 434, 434, 438, 438, 
442, 442, 446, 446, 450, 
450, 454, 454, 458, 458, 
462, 462, 466, 466, 470, 
470, 474, 474, 478, 478, 
483, 483, 487, 487, 491, 
491, 495, 495, 499, 499, 
503, 503, 507, 507, 511, 
511, 
};

static int duty_ratio_table[256] = {
1, 1, 1, 1, 2, 
2, 2, 3, 3, 3, 
4, 4, 5, 5, 5, 
5, 6, 6, 7, 7, 
8, 8, 9, 9, 9, 
9, 10, 10, 11, 11, 
12, 12, 12, 12, 13, 
13, 14, 14, 15, 15, 
16, 16, 16, 16, 17, 
17, 18, 18, 19, 19, 
19, 19, 20, 20, 21, 
21, 22, 22, 22, 22, 
23, 23, 24, 24, 25, 
25, 26, 26, 26, 26, 
27, 27, 28, 28, 29, 
29, 29, 29, 30, 30, 
31, 31, 32, 32, 33, 
33, 33, 33, 34, 34, 
35, 35, 36, 36, 36, 
36, 37, 37, 38, 38, 
39, 39, 40, 40, 40, 
40, 41, 41, 42, 42, 
43, 43, 43, 43, 44, 
45, 46, 46, 46, 46, 
47, 47, 48, 48, 49, 
49, 50, 50, 50, 50, 
51, 51, 52, 52, 53, 
53, 54, 54, 54, 54, 
55, 55, 56, 56, 57, 
57, 57, 57, 58, 58, 
59, 59, 60, 60, 61, 
61, 61, 61, 62, 62, 
63, 63, 64, 64, 64, 
64, 65, 65, 66, 66, 
67, 67, 68, 68, 68, 
68, 69, 69, 70, 70, 
71, 71, 72, 72, 72, 
72, 73, 73, 74, 74, 
75, 75, 75, 75, 76, 
76, 77, 77, 78, 78, 
79, 79, 79, 79, 80, 
80, 81, 81, 82, 82, 
82, 82, 83, 83, 84, 
84, 85, 85, 86, 86, 
86, 86, 87, 87, 88, 
88, 89, 89, 90, 90, 
90, 90, 91, 91, 92, 
92, 93, 93, 93, 93, 
94, 94, 95, 95, 96, 
96, 97, 97, 97, 97, 
98, 98, 99, 99, 100, 
100, 
};

static int edp_power_state;
static int recovery_mode;

static struct completion edp_power_sync;
DEFINE_MUTEX(edp_power_state_chagne);
DEFINE_MUTEX(edp_event_state_chagne);

int get_edp_power_state(void)
{
	return edp_power_state;
}

static struct qpnp_pin_cfg  LCD_EN_PM_GPIO_WAKE =
{
	.mode = 1, /*QPNP_PIN_MODE_DIG_OUT*/
	.output_type = 0, /*QPNP_PIN_OUT_BUF_CMOS*/
	.invert = 0, /*QPNP_PIN_INVERT_DISABLE*/
	.pull = 5, /*QPNP_PIN_PULL_NO*/
	.vin_sel = 2,
	.out_strength = 3, /*QPNP_PIN_OUT_STRENGTH_HIGH*/
	.src_sel = 0, /*QPNP_PIN_SEL_FUNC_CONSTANT*/
	.master_en = 1,
};


static struct qpnp_pin_cfg  LCD_EN_PM_GPIO_SLEEP =
{
	.mode = 1, /*QPNP_PIN_MODE_DIG_OUT*/
	.output_type = 0, /*QPNP_PIN_OUT_BUF_CMOS*/
	.invert = 0, /*QPNP_PIN_INVERT_DISABLE*/
	.pull = 4, /*QPNP_PIN_PULL_DN*/
	.vin_sel = 2,
	.out_strength = 3, /*QPNP_PIN_OUT_STRENGTH_HIGH*/
	.src_sel = 0, /*QPNP_PIN_SEL_FUNC_CONSTANT*/
	.master_en = 1,
};

static struct qpnp_pin_cfg  LCD_PWM_PM_GPIO_WAKE =
{
	.mode = 1, /*QPNP_PIN_MODE_DIG_OUT*/
	.output_type = 0, /*QPNP_PIN_OUT_BUF_CMOS*/
	.invert = 0, /*QPNP_PIN_INVERT_DISABLE*/
	.pull = 5, /*QPNP_PIN_PULL_NO*/
	.vin_sel = 2,
	.out_strength = 3, /*QPNP_PIN_OUT_STRENGTH_HIGH*/
	.src_sel = 3, /*QPNP_PIN_SEL_FUNC_2*/
	.master_en = 1,
};

static struct qpnp_pin_cfg  LCD_PWM_PM_GPIO_SLEEP =
{
	.mode = 1, /*QPNP_PIN_MODE_DIG_OUT*/
	.output_type = 0, /*QPNP_PIN_OUT_BUF_CMOS*/
	.invert = 0, /*QPNP_PIN_INVERT_DISABLE*/
	.pull = 5, /*QPNP_PIN_PULL_NO*/
	.vin_sel = 2,
	.out_strength = 3, /*QPNP_PIN_OUT_STRENGTH_HIGH*/
	.src_sel = 0, /*QPNP_PIN_SEL_FUNC_CONSTANT*/
	.master_en = 1,
};

#if defined(CONFIG_EDP_ESD_FUNCTION)
static int edp_esd_power_state;
#endif

#define DEFAULT_BL_LEVEL 114 /* 140/640 us duty ratio */
#define MIN_BL_LEVEL 2

static void tcon_interanl_clock(void)
{
	pr_debug("%s not change tcon internal clock", __func__);
}

void edp_samsung_set_backlight(struct mdss_edp_drv_pdata *edp_drv, u32 bl_level)
{
	int ret;
	int bl_max;
	unsigned long long llpwm_period, ll_pwm_resolution;
	int duty_level = 0; /* 0~255 */
	int duty_period = 0;

	if (edp_drv->bl_pwm == NULL) {
		pr_err("%s: edp_drv->bl_pwm=NULL.\n", __func__);
		return;
	}

	if (bl_level < MIN_BL_LEVEL) {
		pr_err("%s : bl_level(%d) is too low.. set to MIN(3)\n", __func__, bl_level);
		bl_level = MIN_BL_LEVEL;
	}

	bl_max = edp_drv->panel_data.panel_info.bl_max;
	if (bl_level > bl_max)
		bl_level = bl_max;

	duty_level = duty_level_table[bl_level];

	if (edp_drv->duty_level == duty_level) {
		pr_err("%s : same duty level..(%d) do not pwm_config..\n", __func__, duty_level);
		return;
	}

	llpwm_period = edp_drv->pwm_period;
	llpwm_period <<=  BIT_SHIFT;
	llpwm_period *= duty_level;
	ll_pwm_resolution = MAX_PWM_RESOLUTION;
	do_div(llpwm_period, ll_pwm_resolution);
	duty_period = (llpwm_period >> BIT_SHIFT); 

	ret = pwm_config_us(edp_drv->bl_pwm, duty_period, edp_drv->pwm_period);
	if (ret) {
		pr_err("%s: pwm_config() failed err=%d.\n", __func__, ret);
		return;
	}

	ret = pwm_enable(edp_drv->bl_pwm);
	if (ret) {
		pr_err("%s: pwm_enable() failed err=%d\n", __func__, ret);
		return;
	}

	tcon_pwm_duty(duty_ratio_table[bl_level], 1);

#if defined(CONFIG_EDP_ESD_FUNCTION)
	edp_drv->current_bl = bl_level;
#endif
	edp_drv->duty_level = duty_level;

	pr_debug("%s bl_level : %d duty_level : %d duty_period : %d  duty_ratio : %d",
				__func__, bl_level, duty_level, duty_period,
				duty_ratio_table[bl_level]);
}

void set_backlight_first_kick_off(struct mdss_panel_data *pdata)
{
	static int first_kick_off;
	int i;

	if (first_kick_off)
		return;

	if (pdata == NULL)
		return ;

	edp_backlight_power_enable();

	for (i = 0; i < DEFAULT_BL_LEVEL; i+=2)
		mdss_edp_set_backlight(pdata, i);

	first_kick_off = 1;
}

int edp_samsung_init_pwm_gpios(struct mdss_edp_drv_pdata *edp_drv)
{
	int ret;

	edp_drv->gpio_panel_pwm = of_get_named_gpio(edp_drv->pdev->dev.of_node,
			"gpio-panel-pwm", 0);
	if (!gpio_is_valid(edp_drv->gpio_panel_pwm)) {
		pr_err("%s: gpio_panel_pwm=%d not specified\n", __func__,
				edp_drv->gpio_panel_pwm);
		goto edp_free_pwm;
	}

	ret = gpio_request(edp_drv->gpio_panel_pwm, "disp_pwm");
	if (ret) {
		pr_err("%s: Request reset gpio_panel_pwm failed, ret=%d\n",
				__func__, ret);
		goto edp_free_pwm;
	}

	return 0;

edp_free_pwm:
	pwm_free(edp_drv->bl_pwm);
	return -ENODEV;
}

void mdss_edp_fill_edid_data(struct mdss_edp_drv_pdata *edp_drv)
{
	struct edp_edid *edid = &edp_drv->edid;
	unsigned int res[2];

	edid->id_name[0] = 'A';
	edid->id_name[0] = 'U';
	edid->id_name[0] = 'O';
	edid->id_name[0] = 0;
	edid->id_product = 0x305D;
	edid->version = 1;
	edid->revision = 4;
	edid->ext_block_cnt = 0;
	edid->video_intf = 0x5;
	edid->color_depth = 8;
	edid->dpm = 0;
	edid->color_format = 0;

	edid->timing[0].pclk = 274000000;

	edid->timing[0].h_addressable = 2560;
	edid->timing[0].h_blank = 170;
	edid->timing[0].h_fporch = 48;
	edid->timing[0].h_sync_pulse = 32;

	edid->timing[0].v_addressable = 1600;
	edid->timing[0].v_blank = 73;
	edid->timing[0].v_fporch = 3;
	edid->timing[0].v_sync_pulse = 6;

	edid->timing[0].width_mm = 271;
	edid->timing[0].height_mm = 172;
	edid->timing[0].h_border = 0;
	edid->timing[0].v_border = 0;
	edid->timing[0].interlaced = 0;
	edid->timing[0].stereo = 0;
	edid->timing[0].sync_type = 1;
	edid->timing[0].sync_separate = 1;
	edid->timing[0].vsync_pol = 0;
	edid->timing[0].hsync_pol = 0;

	if (!of_property_read_u32_array(edp_drv->pdev->dev.of_node, "qcom,mdss-pan-size", res, 2)) {
		edid->timing[0].width_mm = res[0];
		edid->timing[0].height_mm = res[1];
	}
}

void mdss_edp_fill_dpcd_data(struct mdss_edp_drv_pdata *edp_drv)
{
	struct dpcd_cap *cap = &edp_drv->dpcd;

	cap->max_lane_count = 4;
	cap->max_link_rate = 10; /* FOR 2.7G  mdss_edp_clk_enable()*/
}

#if defined(CONFIG_EDP_ESD_FUNCTION)
static void edp_esd_work_func(struct work_struct *work)
{
	struct fb_info *info = registered_fb[0];
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	struct mdss_edp_drv_pdata *edp_drv = NULL;

	edp_drv = container_of(work, struct mdss_edp_drv_pdata, edp_esd_work);

	if (!edp_drv) {
		pr_err("%s: Invalid input data edp_drv", __func__);
		return ;
	}

	if (!mfd) {
		pr_err("%s: Invalid input data mfd", __func__);
		return ;
	}

	if (!edp_power_state) {
		pr_err("%s: edp_power_state is off", __func__);
		return ;
	}

	pr_debug("%s start", __func__);

	edp_drv->panel_data.event_handler(&edp_drv->panel_data, MDSS_EVENT_PANEL_OFF, NULL);
	edp_drv->panel_data.event_handler(&edp_drv->panel_data, MDSS_EVENT_UNBLANK, NULL);

	mdss_edp_set_backlight(&edp_drv->panel_data, edp_drv->current_bl);

	pr_debug("%s end", __func__);
}
#endif

void edp_samsung_edp_on_start(struct mdss_edp_drv_pdata *edp_drv)
{
        mutex_lock(&edp_power_state_chagne);
        INIT_COMPLETION(edp_power_sync);

	if (! edp_drv->cont_splash) {
		qpnp_pin_config(edp_drv->gpio_panel_pwm, &LCD_PWM_PM_GPIO_WAKE);
		qpnp_pin_config(edp_drv->gpio_panel_en, &LCD_EN_PM_GPIO_WAKE);
		mdss_edp_regulator_on(edp_drv);
	} else {
	}
}

void edp_samsung_edp_on_end(struct mdss_edp_drv_pdata *edp_drv)
{
	mutex_unlock(&edp_power_state_chagne);

	if (! edp_drv->cont_splash) {
		edp_write(edp_drv->base +0x304, 1);

		if (!wait_for_completion_timeout(&edp_power_sync, 3 * HZ)) {
			pr_err("%s: timeout error\n", __func__);
		}
#if defined(CONFIG_EDP_ESD_FUNCTION)
		edp_esd_power_state = 1;
#endif

	} else {
		edp_power_state = 1;
		tcon_i2c_slave_change(edp_drv);
		if (gpio_get_value(edp_drv->gpio_panel_hpd)) {
			tcon_interanl_clock();
		}
	}
}

void edp_samsung_edp_off_start(struct mdss_edp_drv_pdata *edp_drv)
{
	mutex_lock(&edp_power_state_chagne);
	edp_power_state = 0;
#if defined(CONFIG_EDP_ESD_FUNCTION)
	edp_esd_power_state = 0;
#endif
}

void edp_samsung_edp_off_end(struct mdss_edp_drv_pdata *edp_drv)
{
	mutex_unlock(&edp_power_state_chagne);
	edp_drv->duty_level = 0;

	qpnp_pin_config(edp_drv->gpio_panel_pwm, &LCD_PWM_PM_GPIO_SLEEP);
	qpnp_pin_config(edp_drv->gpio_panel_en, &LCD_EN_PM_GPIO_SLEEP);

	msleep(100); /* NDRA needs some delay after shutdown power */
}

static int count_recovery = 0;
void edp_samsung_event_link_train(struct mdss_edp_drv_pdata *ep)
{
	if (!edp_power_state) {
		msleep(120);
		if (gpio_get_value(ep->gpio_panel_hpd)) {
			pr_err("%s : hpd detected count_recovery = %d \n", __func__, count_recovery);
			msleep(230); /* NDRA LDI REQUIREMENT  350ms delay*/
			tcon_interanl_clock();
			mdss_edp_do_link_train(ep);
			edp_power_state = 1;
			edp_backlight_enable(ep);
			complete(&edp_power_sync);
		} else {
			count_recovery++;
			gpio_set_value(ep->gpio_panel_en, 0);
			msleep(100);
			gpio_set_value(ep->gpio_panel_en, 1);
			pr_err("%s : hpd is not detected, do gpio_panel_en reset = %d \n", __func__, count_recovery);
		}
	}
}

void edp_samsung_hpd_interrupt(struct mdss_edp_drv_pdata *ep)
{
	if (edp_esd_power_state)
		schedule_work(&ep->edp_esd_work);
	else
		edp_send_events(ep, EV_LINK_TRAIN);
}

int edp_samsung_event_handler(struct mdss_panel_data *pdata, int event, void *arg)
{
	int rc = 0;
	struct mdss_edp_drv_pdata *edp_drv = NULL;

	edp_drv = container_of(pdata, struct mdss_edp_drv_pdata,
					panel_data);

	if (!edp_drv) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&edp_event_state_chagne);

	pr_debug("%s: event=%d\n", __func__, event);

	switch (event) {
	case MDSS_EVENT_UNBLANK:
		rc = mdss_edp_on(pdata);
		break;
	case MDSS_EVENT_PANEL_OFF:
		rc = mdss_edp_off(pdata);
		break;
	default:
		pr_debug("%s : Unknown event (%d)\n", __func__, event);
		break;
	}

	mutex_unlock(&edp_event_state_chagne);

	return rc;
}

void edp_samsung_init(struct mdss_edp_drv_pdata *edp_drv)
{
	init_completion(&edp_power_sync);

#if defined(CONFIG_EDP_ESD_FUNCTION)
	INIT_WORK(&edp_drv->edp_esd_work, edp_esd_work_func);
#endif

#if defined(CONFIG_EDP_TCON_MDNIE)
	init_mdnie_class();
#endif
}

static int __init edp_current_boot_mode(char *mode)
{
	/*
	*	1 is recovery booting
	*	0 is normal booting
	*/

	if (strncmp(mode, "1", 1) == 0)
		recovery_mode = 1;
	else
		recovery_mode = 0;

	pr_debug("%s %s", __func__, recovery_mode ?
						"recovery" : "normal");
	return 1;
}
__setup("androidboot.boot_recovery=", edp_current_boot_mode);

