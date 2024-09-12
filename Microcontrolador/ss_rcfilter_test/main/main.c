#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"    
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "math.h"
#include "esp_timer.h"

#define PWM_FREQUENCY 10000
#define PWM_GPIO GPIO_NUM_4
#define V1_CHANNEL ADC_CHANNEL_4
#define V2_CHANNEL ADC_CHANNEL_5
#define TIMER_PERIOD_US 5000

#define SETPOINT 2.00

adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc1_cali_handle;

float setpoint = SETPOINT;
float error = 0;
float accumulated_error = 0;

float u_signal = 0;

const float K_new[2] = {34.7508, 928.2810};
//const float K_new[2] = {35.0092, 560.9583};
const float ki = -128.1462;
//const float ki = -24.9767;

int v1_value_mv = 0;
float v1_value_v = 0.0;

int v2_value_mv = 0;
float v2_value_v = 0.0;

float y_value_v = 0.0;

const float Ts = TIMER_PERIOD_US / 1000000.0;

float a11 = 0.9060;
float a12 = 0.0464;
float a21 = 0.0464;
float a22 = 0.9524;

float b11 = 0.0476;
float b21 = 0.0012;

float c11 = 0.0;
float c12 = 1.0;

float l11 = 0.5;
float l21 = 0.5;

float x1_hat[2] = {0.0, 0.0};
float x2_hat[2] = {0.0, 0.0};

void timer_callback(void* arg);

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
        .gpio_num = PWM_GPIO,
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
    adc_oneshot_config_channel(adc1_handle, V1_CHANNEL, &adc1_chan_cfg);
    adc_oneshot_config_channel(adc1_handle, V2_CHANNEL, &adc1_chan_cfg);
    //

    adc_cali_curve_fitting_config_t adc1_cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    adc_cali_create_scheme_curve_fitting(&adc1_cali_cfg, &adc1_cali_handle);


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
}

void timer_callback(void* arg){
    //adc_oneshot_get_calibrated_result(adc1_handle, adc1_cali_handle, V1_CHANNEL, &v1_value_mv);
    //v1_value_v = v1_value_mv / 1000.0;

    adc_oneshot_get_calibrated_result(adc1_handle, adc1_cali_handle, V2_CHANNEL, &v2_value_mv);
    v2_value_v = v2_value_mv / 1000.0;

    y_value_v = v2_value_v;

    error = setpoint - y_value_v;

    printf("%.2f \n", y_value_v);

    x1_hat[0] = x1_hat[1];
    x2_hat[0] = x2_hat[1];

    u_signal = -ki * accumulated_error - (K_new[0] * x1_hat[0] + K_new[1] * x2_hat[0]);

    x1_hat[1] = (a11 - l11 * c11) * x1_hat[0] + (a12 - l11 * c12) * x2_hat[0] + b11 * u_signal + l11 * y_value_v;
    x2_hat[1] = (a21 - l21 * c11) * x1_hat[0] + (a22 - l21 * c12) * x2_hat[0] + b21 * u_signal + l21 * y_value_v;

    if(u_signal > 4095){
        u_signal = 4095;
    } else if(u_signal < 0){
        u_signal = 0;
    }

    //printf("%.2f \n", u_signal);

    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (int) u_signal, 0);
    //printf("%d \n", u_signal);
    accumulated_error += error;
}
