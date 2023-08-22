#include "redbacklib.h"

rb_err_t rb_fcreate_linear(rb_fparam_linear_t *c)
{
    if ((c->m < -1) || (c->m > 1))
        return rb_OUT_OF_RANGE;
    c->t1 = abs( (c->y1 - c->y0) / c->m) + c->t0;
    c->m = (c->y1 >= c->y0) ? c->m : c->m*-1;
    return rb_OK;
}

rb_err_t rb_feval_linear(const rb_fparam_linear_t *c, const time_t t, float *y) {

    if (t < c->t0){
        return rb_OUT_OF_RANGE;
    }
    if (t >= c->t1){
        *y = c->y1;
        return rb_COMPLETE;
    }
    *y = c->m*(t-c->t0) + c->y0;
    return rb_OK;
}

int32_t rb_map_float_to_int(const int y0,const int y1,const float x0,const float x1, float x){
    float m = (y1 - y0)/(x1 - x0);
    return (int32_t)(m*x);
}

void set_motor_direction(bdc_motor_handle_t motor,float current_value)
{
    static rb_direction_t dir = DIRECTION_FORWARD;

    if((current_value < 0) && (dir != DIRECTION_REVERSE)) {
        ESP_ERROR_CHECK(bdc_motor_reverse(motor));
        dir = DIRECTION_REVERSE;
    }
    else if((current_value >= 0) && (dir != DIRECTION_FORWARD)) {
        ESP_ERROR_CHECK(bdc_motor_forward(motor));
        dir = DIRECTION_FORWARD;
    }
}

bdc_motor_handle_t new_motor(uint32_t pin_a, uint32_t pin_b)
{
    bdc_motor_handle_t motor = NULL; // Permenant memory
    bdc_motor_config_t motor_conf = {
        .pwm_freq_hz = MCPWM_FREQ_HZ,
        .pwma_gpio_num = pin_a,
        .pwmb_gpio_num = pin_b,
    };
    bdc_motor_mcpwm_config_t mcpwm_conf = {
        .group_id = 0,
        .resolution_hz = MCPWM_TIMER_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_conf, &mcpwm_conf, &motor));
    return motor;
}

pcnt_unit_handle_t new_encoder(uint32_t pin_a, uint32_t pin_b, int32_t max_pulse_count)
{
    pcnt_unit_handle_t encoder = NULL;
    pcnt_unit_config_t encoder_conf = {
        .flags.accum_count = true,
        .high_limit = max_pulse_count,
        .low_limit = -max_pulse_count,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&encoder_conf, &encoder));
    pcnt_glitch_filter_config_t fliter_conf = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(encoder,&fliter_conf));
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = pin_a,
        .level_gpio_num = pin_b,
    };
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = pin_b,
        .level_gpio_num = pin_a,
    };
    
    pcnt_channel_handle_t chan_a = NULL;
    pcnt_channel_handle_t chan_b = NULL;

    ESP_ERROR_CHECK(pcnt_new_channel(encoder,&chan_a_config,&chan_a));
    ESP_ERROR_CHECK(pcnt_new_channel(encoder,&chan_b_config,&chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_enable(encoder));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(encoder));
    ESP_ERROR_CHECK(pcnt_unit_start(encoder));

    return encoder;
}