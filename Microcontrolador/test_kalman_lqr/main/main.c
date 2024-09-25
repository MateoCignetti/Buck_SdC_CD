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

#define PWM_FREQUENCY 1000
#define TIMER_PERIOD_US 1000
#define PWM_GPIO_NUM GPIO_NUM_4
#define POT_CHANNEL ADC_CHANNEL_4
#define FB_CHANNEL ADC_CHANNEL_5
#define PRINT_TASK_PERIOD_MS 10

// Saturation limits
#define SATURATION_HIGH_LIMIT 4095
//

// Modify this to use an arbitrary setpoint or not
#define ABRITRARY_SETPOINT 0
#define SETPOINT_ARBITRARY_V 4.5
//

// Enable or disable windup
#define INTEGRATOR_ANTIWINDUP 1

// Discrete state space model
const float a11 = 0.9626;
const float a12 = 0.000254;
const float a21 = -48.61;
const float a22 = 0.01175;  

const float b11 = 0.0008139;
const float b21 = -1.861;

const float c11 = 1;
const float c12 = 0;
//

// Control constants
const float K_new[2] = {27.2030, -0.0060};
const float ki = 16.9678;
const float Ts = TIMER_PERIOD_US / 1000000.0;
//

// Kalman tuning parameters
float q11 = 0.0000001;   // Process noise
float q22 = 0.0000001;   // Process noise

float r = 1000000; // Measurement error covariance
//

// Handles
adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc1_cali_handle;
TaskHandle_t xTaskPrint_handle = NULL;

portMUX_TYPE _spinlock = portMUX_INITIALIZER_UNLOCKED;
//

// Variables
int fb_value_mv = 0;
float fb_value_v = 0.0;
int setpoint_mv = 0;
float setpoint_v = 0.0;
float accumulated_error = 0.0;
int pwm_output_bits = 0;
double error = 0.0;
double u_signal = 0.0;
float y_feedback = 0.0;
//

// Kalman filter variables
float x1_hat = 0.0, x2_hat = 0.0; // estimated states
float P_k11 = 1.0, P_k12 = 0.0, P_k21 = 0.0, P_k22 = 1.0; // Covariance error matrix
float P_k_pred11, P_k_pred12, P_k_pred21, P_k_pred22; // Predicted covariance error matrix
float x1_pred, x2_pred; // Predicted states
float S = 0.0;          // Denominator coefficient for kalman gains
float K11 = 0.0, K21 = 0.0; // Kalman gains
float y_hat = 0.0, y_error = 0.0;
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
    //

    // Start timer
    esp_timer_start_periodic(timer_handle, TIMER_PERIOD_US);
    //
    

    // Tasks creation
    xTaskCreate(&vTaskPrint,
                "vTaskPrint",
                configMINIMAL_STACK_SIZE * 5,
                NULL,
                1,
                &xTaskPrint_handle);
    //
}

void timer_callback(void* arg){

    // Setpoint by potentiometer
    #if !ARBITRARY_SETPOINT
        adc_oneshot_get_calibrated_result(adc1_handle, adc1_cali_handle, POT_CHANNEL, &setpoint_mv);
        setpoint_v = setpoint_mv * (12.0 / 3110.0);
        //setpoint_v = setpoint_mv / 1000.0;
    #endif
    //

    // Feedback measuring and calibration
    adc_oneshot_get_calibrated_result(adc1_handle, adc1_cali_handle, FB_CHANNEL, &fb_value_mv);
    fb_value_v = fb_value_mv / 1000.0;
    fb_value_v = fb_curve_coefficients[3] * pow(fb_value_v, 3) + fb_curve_coefficients[2] * pow(fb_value_v, 2) + fb_curve_coefficients[1] * fb_value_v + fb_curve_coefficients[0];
    if(fb_value_v < 0.0){
        fb_value_v = 0.0;
    } else if(fb_value_v > 12.0){
        fb_value_v = 12.0;
    }
    y_feedback = fb_value_v;
    //

    // Arbitrary setpoint
    #if ABRITRARY_SETPOINT
        setpoint_v = SETPOINT_ARBITRARY_V;
    #endif
    //

    // Error and control signal calculation
    error = setpoint_v - fb_value_v;

    // Kalman filter - predict states
    x1_pred = a11 * x1_hat + a12 * x2_hat + b11 * u_signal;
    x2_pred = a21 * x1_hat + a22 * x2_hat + b21 * u_signal;

    u_signal = ki * accumulated_error - (K_new[0] * x1_hat + K_new[1] * x2_hat);
    
    if(u_signal > SATURATION_HIGH_LIMIT){
        u_signal = SATURATION_HIGH_LIMIT;
    } else if(u_signal < 0.0){
        u_signal = 0.0;
    }
    //

    // Kalman filter - predict covariance error matrix
    P_k_pred11 = (a11 * P_k11 + a12 * P_k21) * a11 + (a11 * P_k12 + a12 * P_k22) * a12 + q11;
    P_k_pred12 = (a11 * P_k11 + a12 * P_k21) * a21 + (a11 * P_k12 + a12 * P_k22) * a22;
    P_k_pred21 = (a21 * P_k11 + a22 * P_k21) * a11 + (a21 * P_k12 + a22 * P_k22) * a12;
    P_k_pred22 = (a21 * P_k11 + a22 * P_k21) * a21 + (a21 * P_k12 + a22 * P_k22) * a22 + q22;

    // Kalman filter - calculate Kalman gains
    S = c11 * (c11 * P_k_pred11 + c12 * P_k_pred21) + c12 * (c11 * P_k_pred12 + c12 * P_k_pred22) + r;

    K11 = (P_k_pred11 * c11 + P_k_pred12 * c12) / S;
    K21 = (P_k_pred21 * c11 + P_k_pred22 * c12) / S;

    // Kalman filter - correct states
    y_hat = c11 * x1_pred + c12 * x2_pred;  
    y_error = y_feedback - y_hat;

    x1_hat = x1_pred + K11 * y_error;
    x2_hat = x2_pred + K21 * y_error;
    
    // Kalman filter - correct covariance error matrix
    P_k11 = (1 - K11 * c11) * P_k_pred11 - K11 * c12 * P_k_pred21;
    P_k12 = (1 - K11 * c11) * P_k_pred12 - K11 * c12 * P_k_pred22;
    P_k21 = (1 - K21 * c12) * P_k_pred21 - K21 * c11 * P_k_pred11;
    P_k22 = (1 - K21 * c12) * P_k_pred22 - K21 * c11 * P_k_pred12;
    //   

    // PWM output
    pwm_output_bits = (int) u_signal;

    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm_output_bits, 0);
    //

    // Anti-windup or not
#if INTEGRATOR_ANTIWINDUP
    if(pwm_output_bits == 0){
        if(error > 0){
            accumulated_error += error;
        }
    } else if(pwm_output_bits == SATURATION_HIGH_LIMIT){
        if(error < 0){
            accumulated_error += error;
        }
    } else {
        accumulated_error += error;
    }
#else
   accumulated_error += error;
#endif
    //
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
        x1_hat_l = x1_hat;
        x2_hat_l = x2_hat;
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
