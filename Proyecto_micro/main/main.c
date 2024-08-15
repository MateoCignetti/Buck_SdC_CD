#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

#define PWM_FREQUENCY 15000
#define PWM_GPIO_NUM GPIO_NUM_4

void app_main(void){
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

    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 100, 0);

    int adc_value = 0;
    int sp_value = 0;
    int buffer[10] = {0};
    int sum = 0;
    int index = 0;

    float a_coefficients[3] = {0, 0, -1};
    float b_coefficients[3];
    float input_array[3] = {0, 0, 0};
    float output_array[3] = {0, 0, 0};

    float Kp = 1;
    float Ki = 5;
    float Kd = 0;
    float Ts = 0.01;
    float Ts_ms = Ts * 1000;

    b_coefficients[0] = Kp + (Ki * Ts / 2) + (2 * Kd / Ts);
    b_coefficients[1] = Ki * Ts - (4 * Kd / Ts);
    b_coefficients[2] = -Kp + (Ki * Ts / 2) + (2 * Kd / Ts);

    float setpoint = 3000;

    int test = 0;

    int iterations = 0;
    while(true){
        
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &sp_value);
        sum -= buffer[index];
        buffer[index] = sp_value;
        sum += buffer[index];
        index = (index + 1) % 10;
        int average = sum / 10;
        //printf("Moving Average: %d\n", average);
        //ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, average, 0);
        setpoint = average;
    
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &adc_value);
        printf("ADC Value: %d\n", adc_value);
        printf("Setpoint: %d\n", (int)setpoint);
                    setpoint = 2500;

        input_array[0] = setpoint - adc_value;
        output_array[0] = b_coefficients[0] * input_array[0] + b_coefficients[1] * input_array[1] + b_coefficients[2] * input_array[2] - a_coefficients[1] * output_array[1] - a_coefficients[2] * output_array[2];
        if(output_array[0] > 4095){
            output_array[0] = 4095;
        } else if(output_array[0] < 0) {
            output_array[0] = 0;
        }
        
        
        ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (int) output_array[0], 0);
        input_array[2] = input_array[1];
        input_array[1] = input_array[0];
        output_array[2] = output_array[1];
        output_array[1] = output_array[0];

        
        /*if(iterations >= 1000){
            setpoint = 2500;
        }
        iterations++;*/
        vTaskDelay((Ts_ms) / portTICK_PERIOD_MS);
    }
}
