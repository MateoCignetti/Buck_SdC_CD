#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "math.h"

#define PWM_FREQUENCY 19000
#define PWM_GPIO_NUM GPIO_NUM_4
#define POT_CHANNEL ADC_CHANNEL_4
#define FB_CHANNEL ADC_CHANNEL_5

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
    adc_oneshot_unit_handle_t adc1_handle;
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
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &adc1_chan_cfg);
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_5, &adc1_chan_cfg);
    //

    // Configure ADC Calibration
    adc_cali_handle_t adc1_cali_handle;
    adc_cali_curve_fitting_config_t adc1_cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_cali_create_scheme_curve_fitting(&adc1_cali_cfg, &adc1_cali_handle);
    //

    int fb_value_mv = 0;
    float fb_value_v = 0.0;
    int setpoint_mv = 0;
    float setpoint_v = 0.0;

    // PID constants and variables
    const float Kp = 1;
    const float Ki = 1;
    const float Kd = 0;
    const float Ts = 0.001;
    const int Ts_ms = Ts * 1000;

    const float a_coefficients[3] = {0, 0, -1};
    const float b_coefficients[3] = {
        Kp + (Ki * Ts / 2) + (2 * Kd / Ts),
        Ki * Ts - (4 * Kd / Ts),
        -Kp + (Ki * Ts / 2) + (2 * Kd / Ts)
    };
    float input_array[3] = {0, 0, 0};
    float output_array[3] = {0, 0, 0}; 
    //

    // Feedback correction coefficients
    const float fb_curve_coefficients[4] = {-0.1436, 4.2716, -0.6413, 0.1787};
    //

    int pwm_output_bits = 0;

    while(true){
        TickType_t xLastWakeTime = xTaskGetTickCount();

        
        adc_oneshot_get_calibrated_result(adc1_handle, adc1_cali_handle, POT_CHANNEL, &setpoint_mv);
        setpoint_v = setpoint_mv * (12.0 / 3300.0);
        adc_oneshot_get_calibrated_result(adc1_handle, adc1_cali_handle, FB_CHANNEL, &fb_value_mv);
        fb_value_v = fb_value_mv / 1000.0;
        fb_value_v = fb_curve_coefficients[3] * pow(fb_value_v, 3) + fb_curve_coefficients[2] * pow(fb_value_v, 2) + fb_curve_coefficients[1] * fb_value_v + fb_curve_coefficients[0];


        //printf("Setpoint: %.2f V \n", setpoint_v);
        //printf("Feedback: %.2f V \n", fb_value_v);
        setpoint_v = 9.6;
        input_array[0] = setpoint_v - fb_value_v;
        output_array[0] = b_coefficients[0] * input_array[0] + b_coefficients[1] * input_array[1] + b_coefficients[2] * input_array[2] - a_coefficients[1] * output_array[1] - a_coefficients[2] * output_array[2];
        
        pwm_output_bits = ((int) output_array[0]) * 4095 / 12;

        if(pwm_output_bits > 4095){
            pwm_output_bits = 4095;
        } else if(pwm_output_bits < 0) {
            pwm_output_bits = 0;
        }
        
        ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm_output_bits, 0);
        input_array[2] = input_array[1];
        input_array[1] = input_array[0];
        output_array[2] = output_array[1];
        output_array[1] = output_array[0];

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Ts_ms));
    }
}
