#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "math.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SAMPLE_TIME_US 200
#define SAMPLE_SIZE 2000  // 5000 samples at 200us each = 1 second
#define PWM_FREQUENCY 19000
#define PWM_GPIO_NUM GPIO_NUM_4
#define POT_CHANNEL ADC_CHANNEL_4
#define FB_CHANNEL ADC_CHANNEL_5
#define SETPOINT_V 6.0

float output[SAMPLE_SIZE];;
float pwm[SAMPLE_SIZE];

adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc1_cali_handle;
esp_timer_handle_t timer_handle; // May need to implement timer stop outside of callback function

int fb_value_mv = 0;
float fb_value_v = 0.0;
int value_index = 0;

// PID constants and variables
const float Kp = 0.5381;
const float Ki = 52.39;
const float Kd = 0.0002741;
const float Ts = SAMPLE_TIME_US / 1000000.0;
const float Nc = 543.1830527;

const float a_coefficients[3] = {
    1,
    -2 + Nc * Ts,
    1 - Nc * Ts
};
const float b_coefficients[3] = {
    Kp + Kd * Nc,
    -2 * Kp - 2 * Kd * Nc + Ki * Ts + Kp * Nc * Ts,
    Kp + Kd * Nc - Ki * Ts - Kp * Nc * Ts + Ki * Nc * Ts * Ts
};
float input_array[3] = {0, 0, 0};
float output_array[3] = {0, 0, 0}; 
int pwm_output_bits = 0;
float setpoint_v = 0.0;
//

// Feedback correction coefficients
const float fb_curve_coefficients[4] = {-0.1436, 4.2716, -0.6413, 0.1787};
//

void timer_callback(void* arg);
void print_output_values();

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
    esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "PID Timer"
    };
    esp_timer_create(&timer_args, &timer_handle);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    printf("Starting identification sequence...\n");
    esp_timer_start_periodic(timer_handle, SAMPLE_TIME_US);

    while(true){
        if(!esp_timer_is_active(timer_handle)){
            printf("Setpoint sequence completed, printing output values:\n");
            print_output_values();
            break;
        }
    }
}

void timer_callback(void* arg){
    if(value_index == SAMPLE_SIZE){
        ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 0);
        esp_timer_stop(timer_handle);
        return;
    }

    adc_oneshot_get_calibrated_result(adc1_handle, adc1_cali_handle, FB_CHANNEL, &fb_value_mv);
    fb_value_v = fb_value_mv / 1000.0;
    fb_value_v = fb_curve_coefficients[3] * pow(fb_value_v, 3) + fb_curve_coefficients[2] * pow(fb_value_v, 2) + fb_curve_coefficients[1] * fb_value_v + fb_curve_coefficients[0];

    output[value_index] = fb_value_v;

    setpoint_v = SETPOINT_V;

    input_array[0] = setpoint_v - fb_value_v;
    output_array[0] = b_coefficients[0] * input_array[0] + b_coefficients[1] * input_array[1] + b_coefficients[2] * input_array[2] - a_coefficients[1] * output_array[1] - a_coefficients[2] * output_array[2];

    pwm_output_bits = (int) (output_array[0] * 4095.0 / 12.0);

    if(pwm_output_bits > 4095){
        pwm_output_bits = 4095;
    } else if(pwm_output_bits < 0) {
        pwm_output_bits = 0;
    }
    pwm[value_index] = pwm_output_bits;
    
    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm_output_bits, 0);
    input_array[2] = input_array[1];
    input_array[1] = input_array[0];
    output_array[2] = output_array[1];
    output_array[1] = output_array[0];

    value_index++;
}

void print_output_values(){
    printf("START\n");
    for(int i = 0; i < SAMPLE_SIZE; i++){
        if(output[i] < 0){
            output[i] = 0;
        }
        printf("%.2f\n", output[i]);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    printf("END\n");

    vTaskDelay(10000 / portTICK_PERIOD_MS);

    printf("START\n");
    for(int i = 0; i < SAMPLE_SIZE; i++){
        printf("%.2f\n", pwm[i]);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    printf("END\n");
}