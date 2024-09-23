#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "math.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define ABRITRARY_SETPOINT 1
#define SETPOINT_ARBITRARY_V 6.00

#define PRINT_TASK_PERIOD_MS 100
#define READ_TASK_PERIOD_MS 10

#define PWM_FREQUENCY 1000
#define TIMER_PERIOD_US 10000
#define PWM_GPIO_NUM GPIO_NUM_4
#define FB_CHANNEL ADC_CHANNEL_5

adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc1_cali_handle;
TaskHandle_t xTaskPrint_handle = NULL;
TaskHandle_t xTaskRead_handle = NULL;

portMUX_TYPE _spinlock = portMUX_INITIALIZER_UNLOCKED;

int fb_value_mv = 0;
float fb_value_v = 0.0;
float fb_value_calibrated = 0.0;
int setpoint_mv = 0;
float setpoint_v = 0.0;
float accumulated_error = 0.0;

const float Ts = TIMER_PERIOD_US / 1000000.0;

int pwm_output_bits = 100;
//

// Feedback correction coefficients
const float fb_curve_coefficients[4] = {-0.0358, 4.0233, -0.487, 0.1506};
//

void timer_callback(void* arg);
void vTaskPrint();
void vTaskRead();

void app_main(void){

    // Configure PWM
    ledc_timer_config_t ledc_timer_cfg = {
        .duty_resolution = LEDC_TIMER_12_BIT,
        .freq_hz = PWM_FREQUENCY,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer_cfg);

    ledc_channel_config_t ledc_channel_cfg = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = 4,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
    };
    ledc_channel_config(&ledc_channel_cfg);
    ledc_fade_func_install(0);
    //

    // Configure ADC Oneshot and Channels
    adc_oneshot_unit_init_cfg_t adc1_init_cfg = {
        .unit_id = ADC_UNIT_1,
        .clk_src = 0,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    adc_oneshot_new_unit(&adc1_init_cfg, &adc1_handle);
    
    adc_oneshot_chan_cfg_t adc1_chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_5, &adc1_chan_cfg);
    //

    // Configure ADC Calibration
    adc_cali_curve_fitting_config_t adc1_cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_cali_create_scheme_curve_fitting(&adc1_cali_cfg, &adc1_cali_handle);
    //

    // Configure timer
    esp_timer_handle_t timer_handle;
    esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "PID Timer"
    };
    esp_timer_create(&timer_args, &timer_handle);
                            esp_timer_start_periodic(timer_handle, TIMER_PERIOD_US);
    //

    // Configure button
    gpio_set_direction(GPIO_NUM_7, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_7, GPIO_PULLUP_ONLY);
    //
    
    xTaskCreate(&vTaskPrint,
                "vTaskPrint",
                configMINIMAL_STACK_SIZE * 5,
                NULL,
                1,
                &xTaskPrint_handle);

    xTaskCreate(&vTaskRead,
                "vTaskRead",
                configMINIMAL_STACK_SIZE * 5,
                NULL,
                1,
                &xTaskRead_handle);
}

void timer_callback(void* arg){
    adc_oneshot_get_calibrated_result(adc1_handle, adc1_cali_handle, FB_CHANNEL, &fb_value_mv);
    fb_value_v = fb_value_mv / 1000.0;
    //printf("Feedback: %.2f V \n", fb_value_v);
    fb_value_calibrated = fb_curve_coefficients[3] * pow(fb_value_v, 3) + fb_curve_coefficients[2] * pow(fb_value_v, 2) + fb_curve_coefficients[1] * fb_value_v + fb_curve_coefficients[0];
    if(fb_value_v < 0.0){
        fb_value_v = 0.0;
    } else if(fb_value_v > 12.0){
        fb_value_v = 12.0;
    }
    
    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm_output_bits, 0);
}

void vTaskPrint(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    float fb_value_v_l = 0.0;
    float fb_value_calibrated_l = 0.0;
    int pwm_output_bits_l = 0;

    while(true){
        taskENTER_CRITICAL(&_spinlock);
        fb_value_v_l = fb_value_v;
        fb_value_calibrated_l = fb_value_calibrated;
        pwm_output_bits_l = pwm_output_bits;
        taskEXIT_CRITICAL(&_spinlock);

        printf("Feedback: %.2f V \n", fb_value_v_l);
        printf("Calibrated: %.2f V \n", fb_value_calibrated_l);
        printf("PWM: %d \n", pwm_output_bits_l);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PRINT_TASK_PERIOD_MS));
    }
}

void vTaskRead(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();

    bool antibounce = false;

    while(true){
        
        static TickType_t lastChangeTime = 0;
        if(!gpio_get_level(GPIO_NUM_7)){
            if(!antibounce){
            TickType_t currentTime = xTaskGetTickCount();
            if((currentTime - lastChangeTime) >= pdMS_TO_TICKS(1000)){
                taskENTER_CRITICAL(&_spinlock);
                pwm_output_bits = pwm_output_bits + 100;
                if(pwm_output_bits > 4095){
                pwm_output_bits = 4095;
                }
                taskEXIT_CRITICAL(&_spinlock);

                lastChangeTime = currentTime;
            }
            antibounce = true;
            }
        } else {
            antibounce = false;
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(READ_TASK_PERIOD_MS));
    }
}