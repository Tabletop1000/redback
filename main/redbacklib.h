/**
 * @file redbacklib.h
 * @brief set of functions for motor control
*/
#ifndef REDBACK_LIB_H
#define REDBACK_LIB_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"

#define MCPWM_FREQ_HZ 10000 // 10 kHz
#define MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10 MHz
#define MCPWM_DUTY_TICK_MAX       (MCPWM_TIMER_RESOLUTION_HZ / MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define BDC_PID_LOOP_PERIOD_MS        10   // calculate the motor speed every 10ms
#define BDC_PID_EXPECT_SPEED          400  // expected motor speed, in the pulses counted by the rotary encoder


typedef enum{
    rb_OK,
    rb_FAIL,
    rb_OUT_OF_RANGE,
    rb_COMPLETE
}rb_err_t;

typedef enum{
    IN_PROGRESS,
    COMPLETE,
    TIMEOUT
}rb_action_t;

typedef enum {
    DIRECTION_FORWARD,
    DIRECTION_REVERSE
}rb_direction_t;

// Linear function definition
typedef struct{
    float m;    // rate
    float y0;   // current value
    float y1;   // goal
    time_t t0;  // start time
    time_t t1;  // end time
} rb_fparam_linear_t;

typedef struct{
    pcnt_unit_handle_t encoder;
    bdc_motor_handle_t motor;
    pid_ctrl_block_handle_t pid;
}rb_leg_t;

/**
 * @brief Create a linear (y = mx + c) function
 * @param c function parameters
 * @returns rb_OK if successful, otherwise an out of range error
*/
rb_err_t rb_fcreate_linear(rb_fparam_linear_t *c);

/**
 @brief performs a linear interpolation between two points at a specific time
 @param c pointer to function parameters
 @param y pointer to the resulting y value
 @param t pointer to current time
*/
rb_err_t rb_feval_linear(const rb_fparam_linear_t *c, const time_t t, float *y);

/** 
 * @brief maps a floating point number to a signed 32 bit int
 * @param y0 lowest value of new range
 * @param y1 highest value of new range
 * @param x0 lowest value of new range
 * @param x1 highest value of old range
 * @param x current x value to evaluate at
 * @returns signed int
*/
int32_t rb_map_float_to_int(const int y0,const int y1,const float x0,const float x1, float x);

/**
 * @brief Changes motor direction based on sign of value demand
 * @param motor ponter to bdc motor handle
 * @param current_value the signed float value
*/
void set_motor_direction(bdc_motor_handle_t motor,float current_value);

/**
 * @brief Creates a motor instance
 * @param pin_a GPIO A
 * @param pin_b GPIO B
 * @returns bdc motor handle
*/
bdc_motor_handle_t new_motor(uint32_t pin_a, uint32_t pin_b);

/**
 * @brief Creates an encoder instance
 * @param pin_a GPIO A
 * @param pin_b GPIO B
 * @param max_pulse_count after which the count will reset to min count. min count is negated value
 * @returns encodr handle
*/
pcnt_unit_handle_t new_encoder(uint32_t pin_a, uint32_t pin_b, int32_t max_pulse_count);

pid_ctrl_block_handle_t new_pid();
// pid_ctrl_parameter_t pid_config = {
//     .kp = 0.6,
//     .ki = 0.4,
//     .kd = 0.2,
//     .cal_type = PID_CAL_TYPE_INCREMENTAL,
// };

#endif