#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "math.h"

#define SAMPLE_TIME_US 200
#define SAMPLE_SIZE 1023
#define SYSTEM_INPUT_GPIO GPIO_NUM_4
#define SYSTEM_OUTPUT_ADC_CHANNEL ADC_CHANNEL_5
#define IDENT_USE_CONVERTED_RESULT 1

float output[SAMPLE_SIZE];
int input[SAMPLE_SIZE] = {0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,0,1,1,0,0,0,1,0,0,1,1,0,1,0,1,0,0,0,1,0,0,0,0,1,0,1,0,1,1,1,0,0,0,0,1,0,1,1,0,1,0,1,0,1,1,1,1,1,0,1,0,0,0,0,0,0,0,0,1,0,1,0,1,0,1,0,1,0,0,0,0,1,0,1,1,1,1,0,0,0,1,0,1,1,0,1,1,1,0,0,1,1,0,1,0,0,1,0,1,0,0,1,1,0,0,0,0,1,0,1,0,0,1,1,1,0,0,1,1,0,0,0,0,0,0,1,1,0,1,0,1,0,1,0,1,1,0,0,1,1,0,0,1,1,0,1,0,1,1,0,0,0,0,0,1,0,1,1,0,0,0,1,1,1,1,0,1,1,1,0,0,1,0,0,1,1,0,1,1,1,0,1,0,1,1,0,0,1,0,0,0,0,1,0,0,0,1,0,1,0,1,0,0,0,1,1,0,0,1,1,0,0,0,1,0,0,0,1,0,0,0,1,1,0,0,0,1,0,1,0,1,1,0,0,0,1,0,1,1,1,1,1,0,0,0,0,1,0,0,1,0,0,0,1,1,1,1,0,0,1,1,1,0,1,1,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,1,0,1,1,1,0,1,0,0,1,0,1,1,0,1,0,0,0,1,0,1,1,0,0,1,1,1,0,1,0,0,1,1,1,1,1,1,0,1,0,1,1,0,1,1,0,1,0,0,0,0,0,1,0,0,0,0,1,1,1,0,0,1,1,1,0,0,1,0,0,0,1,0,0,1,1,1,1,0,0,0,0,1,1,0,1,1,0,0,0,1,1,0,1,0,0,1,1,1,0,1,1,1,1,0,0,1,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,0,0,1,0,1,0,1,1,0,1,0,1,1,1,1,0,1,1,1,1,0,1,1,0,1,0,0,1,0,0,0,0,0,1,0,1,0,0,0,1,1,1,0,1,0,0,0,1,1,0,1,1,1,1,0,0,0,0,0,1,0,0,1,0,1,0,1,0,1,1,1,0,1,0,0,0,0,1,0,0,1,1,0,0,0,1,1,0,0,0,0,0,1,1,1,1,1,0,0,0,1,1,0,1,1,0,1,0,1,0,0,1,1,0,1,0,0,0,0,1,1,0,1,0,0,0,1,1,1,1,1,0,1,0,1,0,0,1,0,0,1,1,0,0,1,1,1,1,0,0,1,0,1,0,0,1,0,0,0,1,0,1,1,1,0,1,0,1,0,0,0,0,0,0,1,0,1,1,1,0,0,0,1,1,0,0,1,0,0,0,1,1,0,1,0,1,1,1,0,0,1,0,1,1,1,1,1,1,0,0,1,1,0,1,1,0,1,1,1,0,1,1,1,1,1,0,1,1,0,0,1,0,0,1,0,1,1,0,0,0,0,1,1,0,0,1,0,1,0,1,0,0,1,1,1,1,0,1,0,0,0,1,0,0,1,0,1,1,1,0,0,1,1,1,1,0,1,1,0,0,0,0,0,0,0,1,0,0,0,1,1,1,0,0,0,0,1,1,1,1,1,1,0,0,0,1,0,0,1,0,0,1,1,1,0,1,0,1,1,1,0,1,1,0,0,1,1,0,1,1,1,1,1,0,0,1,0,1,1,0,1,1,0,0,0,0,1,0,0,0,0,0,1,1,1,0,1,0,1,0,1,0,0,1,0,1,1,1,1,0,1,0,1,1,1,1,1,1,1,0,1,0,0,1,0,0,1,0,0,0,0,1,1,0,0,0,0,1,1,1,0,1,1,1,0,0,0,0,0,0,1,0,0,1,1,1,0,0,0,1,0,1,0,0,1,0,1,0,1,1,1,1,0,0,1,1,0,0,1,0,0,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,0,0,1,0,0,1,0,0,1,0,1,0,0,0,1,0,1,0,0,0,0,1,1,1,1,0,1,0,1,0,1,1,0,1,1,1,1,0,1,0,0,1,1,0,1,1,0,0,1,1,1,1,1,0,1,1,1,0,1,1,0,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0,1,1,0,1,1,0,1,1,0,0,1,0,1,1,0,0,1,0,1,0,0,0,0,0,1,1,0,0,1,1,1,0,0,0,0,0,1,1,0,1,1,1,0,0,0,1,0,0,0,0,0,0,1,1,1,1,0,0,0,1,1,1,1,1,1,1};

adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc1_cali_handle;
esp_timer_handle_t timer_handle; // May need to implement timer stop outside of callback function


int fb_value_mv = 0;
float fb_value_v = 0.0;
int value_index = 0;

#if IDENT_USE_CONVERTED_RESULT
// Feedback correction coefficients
const float fb_curve_coefficients[4] = {-0.1436, 4.2716, -0.6413, 0.1787};
//
#endif

void timer_callback(void* arg);
void print_output_values();

void app_main(void){
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
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &adc1_chan_cfg);
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
    esp_timer_start_periodic(timer_handle, SAMPLE_TIME_US);

    while(true){
        if(!esp_timer_is_active(timer_handle)){
            printf("Identification sequence completed, printing output values:\n");
            print_output_values();
            break;
        }
    }
}

void timer_callback(void* arg){
    if(value_index == SAMPLE_SIZE){
        esp_timer_stop(timer_handle);
        return;
    }

    gpio_set_level(SYSTEM_INPUT_GPIO, input[value_index]);

#if IDENT_USE_CONVERTED_RESULT
    adc_oneshot_get_calibrated_result(adc1_handle, adc1_cali_handle, SYSTEM_OUTPUT_ADC_CHANNEL, &fb_value_mv);
    fb_value_v = fb_value_mv / 1000.0;
    fb_value_v = fb_curve_coefficients[3] * pow(fb_value_v, 3) + fb_curve_coefficients[2] * pow(fb_value_v, 2) + fb_curve_coefficients[1] * fb_value_v + fb_curve_coefficients[0];
#else
    adc_oneshot_read(adc1_handle, SYSTEM_OUTPUT_ADC_CHANNEL, &fb_value_mv);
    fb_value_v = (float) fb_value_mv;
#endif

    output[value_index] = fb_value_v;
    value_index++;
}

void print_output_values(){
    printf("START\n");
    for(int i = 0; i < SAMPLE_SIZE; i++){
        printf("%.2f\n", output[i]);
    }
    printf("END\n");
}