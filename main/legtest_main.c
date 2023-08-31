#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "redbacklib.h"
#include "pid_ctrl.h"

#define INCLUDE_xTaskDelayUntil 1
#define MCPWM_GPIO_A 32
#define MCPWM_GPIO_B 33
#define LEGPWM_GPIO_A 16
#define LEGPWM_GPIO_B 17
#define ENCODER_GPIO_A 27
#define ENCODER_GPIO_B 14
#define ENCODER_PPR 5281.10

static const char *TAG = "rb";
static time_t rb_clock = 0;
static float goal_list[] = {0.0,50.0,90.0,20.0,-65.0,-95.0,70.0,0.0};
static float angle_goal_list[] = {0.0,500.0,900.0,200.0,-650.0,-950.0,-300.0,0.0};
static int goal_index = 0;
static int angle_goal_index = 0;
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

static void report_angle_task(pcnt_unit_handle_t encoder)
{
    while(1){
        int cur_pulse_count = 0;
        pcnt_unit_get_count(encoder, &cur_pulse_count);
        ESP_LOGI(TAG,"pulse count: %d",cur_pulse_count);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void angle_tests()
{
    size_t array_size = (sizeof(angle_goal_list)/sizeof(angle_goal_list[0]));
    while(1){
        angle_goal_index++;
        if (angle_goal_index >= array_size)angle_goal_index = 0;
        vTaskDelay(pdMS_TO_TICKS(3000));
        ESP_LOGI(TAG,"Next goal: %f",angle_goal_list[angle_goal_index]);
    }
}

static void angle_control(rb_leg_t* leg)
{
    while(1){ 
        static int last_pulse_count = 0;
        int cur_pulse_count = 0;
        pcnt_unit_get_count(leg->encoder, &cur_pulse_count);

        // calculate position error
        float error = angle_goal_list[angle_goal_index] - cur_pulse_count;
        float new_speed = 0;

        // set the new speed
        pid_compute(leg->pid,error, &new_speed);
        if(new_speed > 0.5)bdc_motor_forward(leg->motor);
        else if (new_speed < -0.05)bdc_motor_reverse(leg->motor);
        else bdc_motor_brake(leg->motor);

        bdc_motor_set_speed(leg->motor,(uint32_t)abs(new_speed));
        ESP_LOGI(TAG,"current pulse: %d, error: %f, speed: %f",cur_pulse_count,error,new_speed);
        vTaskDelay(pdMS_TO_TICKS(20));
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

    static rb_leg_t leg;
    leg.encoder  = new_encoder(ENCODER_GPIO_A,ENCODER_GPIO_B,6000);
    leg.motor = new_motor(LEGPWM_GPIO_A,LEGPWM_GPIO_B);
    leg.pid = new_pid(MCPWM_DUTY_TICK_MAX/2);
    ESP_ERROR_CHECK(bdc_motor_enable(leg.motor));

    xTaskCreate(update_clock, "update_clock",2048,NULL,4,NULL);
    //xTaskCreate(leg_task, "leg_task", 2048, wheel_motor, 10, NULL);
    xTaskCreate(angle_control, "angle_control", 2048, &leg, 5, NULL);
    xTaskCreate(angle_tests, "angle_tests", 2048, NULL, 10, NULL);
}
