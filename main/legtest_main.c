/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "bdc_motor.h"

#define MCPWM_FREQ_HZ 10000 // 10 kHz
#define MCPWM_GPIO_A 32
#define MCPWM_GPIO_B 33
#define MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10 MHz

#define INCLUDE_xTaskDelayUntil 1
#define MCPWM_DUTY_TICK_MAX       (MCPWM_TIMER_RESOLUTION_HZ / MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks

static const char *TAG = "rb";
static time_t rb_clock = 0;

typedef enum{
    rb_OK,
    rb_FAIL,
    rb_OUT_OF_RANGE
}rb_err_t;

typedef enum{
    IN_PROGRESS,
    COMPLETE,
    TIMEOUT
}rb_action_t;

/**
 @brief performs a linear interpolation between two points at a specific time
 @param y0 pointer to initial value
 @param y1 pointer to goal value
 @param y pointer to the resulting y value
 @param t0 point to time that interpolation starts from (usually now)
 @param t pointer to current time
 @param m rate of change
*/
rb_err_t rb_interpolate_linear(const float *y0,const float *y1, float *y,const time_t *t0,const time_t *t,const float *m){
    if ((*t < *t0) || (*m < -1) || (*m > 1))
        return rb_OUT_OF_RANGE;
    *y = *m*(*t-*t0) + *y0;
    return rb_OK;
}

/** 
 * @brief maps a floating point number to a signed 32 bit int
 * @param y0 lowest value of new range
 * @param y1 highest value of new range
 * @param x0 lowest value of new range
 * @param x1 highest value of old range
 * @param x current x value to evaluate at
 * @returns signed int
*/
int32_t rb_map_float_to_int(const int y0,const int y1,const float x0,const float x1, float x){
    float m = (y1 - y0)/(x1 - x0);
    return (int32_t)(m*x);
}

typedef enum {
    DIRECTION_FORWARD,
    DIRECTION_REVERSE
}rb_direction_t;

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

static float goal_list[] = {0.0,50.0,90.0,20.0,-65.0,-95.0,0.0};
static int goal_index = 0;
static void leg_task(bdc_motor_handle_t motor)
{

    float goal_prev = 0;
    float current_value = 0;
    ESP_LOGI(TAG, "Create timer and operator");

    while(1){
        float goal = goal_list[goal_index++];
        if(goal_index > 7)
        {
            ESP_LOGI(TAG,"-------- Complete ----------");
            while(1)vTaskDelay(pdMS_TO_TICKS(100));
        }
        ESP_LOGI(TAG,"Next goal: %f", goal);

        time_t t0 = rb_clock;
        float m = 0.1;
        m = (goal >= goal_prev) ? m : m*-1;

        if(goal != goal_prev)
        {
            while(current_value != goal)
            {
                rb_interpolate_linear(&goal_prev,&goal,&current_value,&t0,&rb_clock,&m);
                set_motor_direction(motor,current_value);
                int pwm_val = rb_map_float_to_int(0,MCPWM_DUTY_TICK_MAX,0,100.0,abs(current_value));
                ESP_LOGI(TAG,"current value: %f, mapped value: %d",current_value,pwm_val);
                ESP_ERROR_CHECK(bdc_motor_set_speed(motor,(uint32_t)pwm_val));
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            goal_prev = goal;
        }
    }
}

static void update_clock()
{
    while(1)
    {
        TickType_t last_wake_time = xTaskGetTickCount();
        rb_clock ++;
        xTaskDelayUntil(&last_wake_time,pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    static bdc_motor_handle_t wheel_motor = NULL;
    wheel_motor = new_motor(MCPWM_GPIO_A,MCPWM_GPIO_B);
    ESP_ERROR_CHECK(bdc_motor_enable(wheel_motor));
    ESP_ERROR_CHECK(bdc_motor_forward(wheel_motor));

    xTaskCreate(update_clock, "update_clock",2048,NULL,4,NULL);
    xTaskCreate(leg_task, "leg_task", 2048, wheel_motor, 10, NULL);
}
