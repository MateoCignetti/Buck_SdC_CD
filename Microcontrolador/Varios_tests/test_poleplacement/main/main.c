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
#define SETPOINT_ARBITRARY_V 2.5

#define PRINT_TASK_PERIOD_MS 10

#define PWM_FREQUENCY 1000
#define TIMER_PERIOD_US 1000
#define PWM_GPIO_NUM GPIO_NUM_4
#define POT_CHANNEL ADC_CHANNEL_4
#define FB_CHANNEL ADC_CHANNEL_5

adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc1_cali_handle;
TaskHandle_t xTaskPrint_handle = NULL;

portMUX_TYPE _spinlock = portMUX_INITIALIZER_UNLOCKED;

int fb_value_mv = 0;
float fb_value_v = 0.0;
int setpoint_mv = 0;
float setpoint_v = 0.0;
float accumulated_error = 0.0;

const float Ts = TIMER_PERIOD_US / 1000000.0;

double error = 0.0;
double u_signal = 0.0;

const float K_new[2] = {1178.7, 0.3};
const float ki = -38.7326;

const double a11 = 0.9626;
const double a12 = 0.000254;
const double a21 = -48.61;
const double a22 = 0.01175;  

const double b11 = 0.0008139;
const double b21 = -1.861;

const double c11 = 1;
const double c12 = 0;

const float l11 = 1.0744;
const float l21 = -43.4405;

float x1_hat[2] = {0.0, 0.0};
float x2_hat[2] = {0.0, 0.0};

int pwm_output_bits = 0;
//

// Feedback correction coefficients
const float fb_curve_coefficients[4] = {-0.0358, 4.0233, -0.487, 0.1506};
//

void timer_callback(void* arg);
void vTaskPrint();

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
    esp_timer_handle_t timer_handle;
    esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "PID Timer"
    };
    esp_timer_create(&timer_args, &timer_handle);
                            esp_timer_start_periodic(timer_handle, TIMER_PERIOD_US);
    
    xTaskCreate(&vTaskPrint,
                "vTaskPrint",
                configMINIMAL_STACK_SIZE * 5,
                NULL,
                1,
                &xTaskPrint_handle);
}

void timer_callback(void* arg){
    #if !ARBITRARY_SETPOINT
        adc_oneshot_get_calibrated_result(adc1_handle, adc1_cali_handle, POT_CHANNEL, &setpoint_mv);
        setpoint_v = setpoint_mv * (12.0 / 3110.0);
        //setpoint_v = setpoint_mv / 1000.0;
    #endif

    adc_oneshot_get_calibrated_result(adc1_handle, adc1_cali_handle, FB_CHANNEL, &fb_value_mv);
    fb_value_v = fb_value_mv / 1000.0;
    fb_value_v = fb_curve_coefficients[3] * pow(fb_value_v, 3) + fb_curve_coefficients[2] * pow(fb_value_v, 2) + fb_curve_coefficients[1] * fb_value_v + fb_curve_coefficients[0];
    if(fb_value_v < 0.0){
        fb_value_v = 0.0;
    } else if(fb_value_v > 12.0){
        fb_value_v = 12.0;
    }

    //printf("Setpoint: %.2f V \n", setpoint_v);
    //printf("Feedback: %.2f V \n", fb_value_v);

    #if ABRITRARY_SETPOINT
        setpoint_v = SETPOINT_ARBITRARY_V;
    #endif

    error = setpoint_v - fb_value_v;

    if(x1_hat[1] > 12.0){
        x1_hat[0] = 12.0;
    } else if (x1_hat[1] < 0){
        x1_hat[0] = 0;
    } else{
        x1_hat[0] = x1_hat[1];
    }

    if(x2_hat[1] > 9999.9){
        x2_hat[0] = 9999.9;
    } else if (x2_hat[1] < -9999.9){
        x2_hat[0] = -9999.9;          
    } else {
        x2_hat[0] = x2_hat[1];
    }
    
    u_signal = -ki * accumulated_error - (K_new[0] * x1_hat[0] + K_new[1] * x2_hat[0]);

    //printf("u: %.2f \n", u_signal);

    if(u_signal > 1000){
        u_signal = 1000;
    } else if(u_signal < 0) {
        u_signal = 0;
    }

    x1_hat[1] = (a11 - l11 * c11) * x1_hat[0] + (a12 - l11 * c12) * x2_hat[0] + b11 * u_signal + l11 * fb_value_v;
    x2_hat[1] = (a21 - l21 * c11) * x1_hat[0] + (a22 - l21 * c12) * x2_hat[0] + b21 * u_signal + l21 * fb_value_v;

    

    pwm_output_bits = (int) u_signal;
    //pwm_output_bits = 394;

    /*if(pwm_output_bits > 4095){
        pwm_output_bits = 4095;
    } else if(pwm_output_bits < 0) {
        pwm_output_bits = 0;
    }*/
    
    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm_output_bits, 0);

    if(pwm_output_bits == 0){
        if(error > 0){
            accumulated_error += error;
        }
    } else if(pwm_output_bits == 1000){
        if(error < 0){
            accumulated_error += error;
        }
    } else {
        accumulated_error += error;
    }
   //accumulated_error += error;
}

void vTaskPrint(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    float setpoint_v_l = 0.0;
    float fb_value_v_l = 0.0;
    float error_l = 0.0;
    float accumulated_error_l = 0.0;
    float x1_hat_l = 0.0;
    float x2_hat_l = 0.0;
    int pwm_output_bits_l = 0;

    while(true){
        taskENTER_CRITICAL(&_spinlock);
        setpoint_v_l = setpoint_v;
        fb_value_v_l = fb_value_v;
        error_l = error;
        accumulated_error_l = accumulated_error;
        x1_hat_l = x1_hat[0];
        x2_hat_l = x2_hat[0];
        pwm_output_bits_l = pwm_output_bits;
        taskEXIT_CRITICAL(&_spinlock);

        printf("Setpoint: %.2f V \n", setpoint_v_l);
        printf("Feedback: %.2f V \n", fb_value_v_l);
        printf("Error: %.2f V \n", error_l);
        printf("q: %.2f \n", accumulated_error_l);
        printf("x1: %.2f \n",x1_hat_l);
        printf("x2: %.2f \n",x2_hat_l);
        printf("PWM: %d \n", pwm_output_bits_l);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PRINT_TASK_PERIOD_MS));
    }
}