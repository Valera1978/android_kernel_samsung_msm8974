#ifndef __EDP_SAMSUNG_H__
#define __EDP_SAMSUNG_H__

extern void tcon_pwm_duty(int pwm_duty, int updata_from_backlight);
extern int config_i2c_lane(int enable);

void mdss_edp_set_backlight(struct mdss_panel_data *pdata, u32 bl_level);

void edp_samsung_set_backlight(struct mdss_edp_drv_pdata *edp_drv, u32 bl_level);
int edp_samsung_init_pwm_gpios(struct mdss_edp_drv_pdata *edp_drv);
void mdss_edp_fill_edid_data(struct mdss_edp_drv_pdata *edp_drv);
void mdss_edp_fill_dpcd_data(struct mdss_edp_drv_pdata *edp_drv);
void edp_samsung_event_link_train(struct mdss_edp_drv_pdata *ep);

void edp_samsung_init(struct mdss_edp_drv_pdata *edp_drv);

void edp_samsung_edp_on_start(struct mdss_edp_drv_pdata *edp_drv);
void edp_samsung_edp_on_end(struct mdss_edp_drv_pdata *edp_drv);
void edp_samsung_edp_off_start(struct mdss_edp_drv_pdata *edp_drv);
void edp_samsung_edp_off_end(struct mdss_edp_drv_pdata *edp_drv);

void edp_samsung_hpd_interrupt(struct mdss_edp_drv_pdata *edp_drv);

void set_backlight_first_kick_off(struct mdss_panel_data *pdata);
void edp_backlight_power_enable(void);
void edp_backlight_enable(struct mdss_edp_drv_pdata *ep);
void edp_backlight_disable(void);

int edp_samsung_event_handler(struct mdss_panel_data *pdata, int event, void *arg);

int mdss_edp_regulator_on(struct mdss_edp_drv_pdata *edp_drv);
int mdss_edp_on(struct mdss_panel_data *pdata);
int mdss_edp_off(struct mdss_panel_data *pdata);
void edp_send_events(struct mdss_edp_drv_pdata *ep, u32 events);
void mdss_edp_do_link_train(struct mdss_edp_drv_pdata *ep);

void tcon_i2c_slave_change(struct mdss_edp_drv_pdata *ep);
void restore_set_tcon(struct mdss_edp_drv_pdata *ep);
int edp_aux_write_buf(struct mdss_edp_drv_pdata *ep, u32 addr,
                                char *buf, int len, int i2c);
#endif
