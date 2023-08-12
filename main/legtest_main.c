#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "redbacklib.h"

#define INCLUDE_xTaskDelayUntil 1

static const char *TAG = "rb";
static time_t rb_clock = 0;

static float goal_list[] = {0.0,50.0,90.0,20.0,-65.0,-95.0,70.0,0.0};
static int goal_index = 0;
static void leg_task(bdc_motor_handle_t motor)
{
    float command_value = 0;
    ESP_LOGI(TAG, "Create timer and operator");
    while(1){
        float goal = goal_list[goal_index++];
        if(goal_index > 8) {
            ESP_LOGI(TAG,"-------- Complete ----------");
            while(1)vTaskDelay(pdMS_TO_TICKS(100));
        }
        ESP_LOGI(TAG,"Next goal: %f", goal);

        time_t t0 = rb_clock;
        rb_fparam_linear_t linf = {
            .m = 0.3,
            .t0 = rb_clock,
            .y0 = command_value,
            .y1 = goal,
        };
        rb_fcreate_linear(&linf);

        while(rb_feval_linear(&linf,rb_clock,&command_value) != rb_COMPLETE){
            set_motor_direction(motor,command_value);
            int pwm_val = rb_map_float_to_int(
                0,MCPWM_DUTY_TICK_MAX,
                0,100.0,
                abs(command_value));
            ESP_LOGI(TAG,"current value: %f, mapped value: %d",command_value,pwm_val);
            ESP_ERROR_CHECK(bdc_motor_set_speed(motor,(uint32_t)pwm_val));
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

static void update_clock()
{
    while(1){
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
