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

#define INCLUDE_xTaskDelayUntil 1
#define SERVO_MIN_PULSEWIDTH_US 0  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2000  // Maximum pulse width in microsecond
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        2000    // 20000 ticks, 20ms
#define SERVO_PULSE_GPIO             32        // GPIO connects to the PWM signal line

static const char *TAG = "RLIB";
static time_t rlib_clock = 0;

typedef enum{
    RLIB_OK,
    RLIB_FAIL,
    RLIB_OUT_OF_RANGE
}rlib_err_t;

typedef enum{
    IN_PROGRESS,
    COMPLETE,
    TIMEOUT
}rlib_action_t;

rlib_err_t rlib_interpolate_linear(const float *y0,const float *y1, float *y,const time_t *t0,const time_t *t,const float *m)
{
    if ((*t < *t0) || (*m < -1) || (*m > 1))
        return RLIB_OUT_OF_RANGE;
    *y = *m*(*t-*t0) + *y0;
    return RLIB_OK;
}

int32_t rlib_map_float_to_int(const int y0,const int y1,const float x0,const float x1, float x){
    float m = (y1 - y0)/(x1 - x0);
    return (int32_t)(m*x);
}

static float goal_list[] = {0.0,50.0,90.0,20.0,65.0};
static int goal_index = 0;

static void leg_task()
{
    float goal_prev = 0;
    float current_value = 0;
    ESP_LOGI(TAG, "Create timer and operator");

    while(1){

        
        float goal = goal_list[goal_index++];

        ESP_LOGI(TAG,"Next goal: %f", goal);
        if(goal_index > 5)
        {
            ESP_LOGI(TAG,"-------- Complete ----------");
            while(1)vTaskDelay(pdMS_TO_TICKS(100));
        }

        time_t t0 = rlib_clock;
        float m = 0.1;
        m = (goal >= goal_prev) ? m : m*-1;

        if(goal != goal_prev)
        {
            while(current_value != goal)
            {
                rlib_interpolate_linear(&goal_prev,&goal,&current_value,&t0,&rlib_clock,&m);
                int pwm_val = rlib_map_float_to_int(SERVO_MIN_PULSEWIDTH_US,SERVO_MAX_PULSEWIDTH_US,0.0,100.0,current_value);
                ESP_LOGI(TAG,"current value: %f, mapped value: %d",current_value,pwm_val);
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, pwm_val));
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
        rlib_clock ++;
        xTaskDelayUntil(&last_wake_time,pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    xTaskCreate(update_clock, "update_clock",2048,NULL,4,NULL);
    xTaskCreate(leg_task, "leg_task", 2048, NULL, 10, NULL);
}
