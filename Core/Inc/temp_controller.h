/*
 * temp_controller.h
 *
 *  Created on: Nov 11, 2020
 *      Author: Norbert
 */

#ifndef INC_TEMP_CONTROLLER_H_
#define INC_TEMP_CONTROLLER_H_

#include "main.h"
#include "bmp280.h"
#include "float_to_string.h"

#define ADC_BUF_LEN				2
#define ROTARY_FAST				20 // [ms] rotary speed fast
#define ROTARY_SLOW				200 // [ms] rotary speed slow
#define MENU_MAX				4
#define ADC_AVARAGE				100
#define LONG_PRESS				5000
#define RIGHT					3
#define	LEFT					4

uint16_t adc_buf[ADC_BUF_LEN];

//UPDATED BY ADC DMA
typedef struct    {
	float current;
	float voltage;
	float resistance;
	float temperature;
}adc_data;

typedef struct PID{
	float Kp;
	float Kd;
	float Ki;
	float error;
	float errorSum;
	float delta_t;
}PID;

typedef struct    {
	float target_temp;
	float current_temp;
	float voltage;
	float current;
	float power;
	short menu;
	PID	  pid;
} temperature_controller_data;

typedef struct BMP280_data{
	float temp;
	float pressure;
	float humidity;	//BMP280 not support it
}BMP280_data;

extern short flag_10ms, flag_200ms, flag_1s, flag_10s;
extern short cnt_adc;

extern void calc_adc_values();
void turn_on_green_LED();
void turn_off_green_LED();
void toggle_green_LED();
void turn_on_red_LED();
void turn_off_red_LED();
void toggle_red_LED();
void blink();
void start_pwm(TIM_HandleTypeDef *);
void set_duty_cycle(int);
void HAL_GPIO_EXTI_Callback (uint16_t );
void HAL_SDADC_ConvCpltCallback(SDADC_HandleTypeDef* hsdadc);
float lookup_temp(float R);
void init_bmp280(struct _BMP280_HandleTypedef *, int );
void read_bmp280(struct _BMP280_HandleTypedef *, struct BMP280_data *);
void update_pid();
void set_defaults();
void Redraw_display();
void menu234();
void TIM3_callback();
void TIM4_callback();
void TIM6_callback();
void TIM7_callback();

#endif /* INC_TEMP_CONTROLLER_H_ */
