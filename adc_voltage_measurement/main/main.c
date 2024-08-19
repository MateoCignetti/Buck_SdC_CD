#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"

#define LINE_SLOPE 1
#define LINE_OFFSET 0

float curve_coefficients[4] = {-0.1436, 4.2716, -0.6413, 0.1787};

void app_main(void) {
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
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_5, &adc1_chan_cfg);

    adc_cali_handle_t adc1_cali_handle;
    adc_cali_curve_fitting_config_t adc1_cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    adc_cali_create_scheme_curve_fitting(&adc1_cali_cfg, &adc1_cali_handle);
    int adc_value_mv = 0;
    float adc_value_v = 0;
    while (true){

        adc_oneshot_get_calibrated_result(adc1_handle, adc1_cali_handle, ADC_CHANNEL_5, &adc_value_mv);
        adc_value_v = adc_value_mv / 1000.0;
        adc_value_v = curve_coefficients[3] * pow(adc_value_v, 3) + curve_coefficients[2] * pow(adc_value_v, 2) + curve_coefficients[1] * adc_value_v + curve_coefficients[0];
        printf("ADC value: %.2f V\n", adc_value_v);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
