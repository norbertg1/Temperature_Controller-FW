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

#define ADC_BUF_LEN 2
#define ROTARY_FAST        20 // [ms] rotary speed fast
#define ROTARY_SLOW       200 // [ms] rotary speed slow
uint16_t adc_buf[ADC_BUF_LEN];

//UPDATED BY ADC DMA
typedef struct    {
	float current;
	float voltage;
	float resistance;
	float temperature;
}adc_data;

typedef struct    {
	float target_temp;
	float current_temp;
	float voltage;
	float current;
	float power;
} temperature_controller_data;

typedef struct BMP280_data{
	float temp;
	float pressure;
	float humidity;	//BMP280 not support it
}BMP280_data;

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
void init_bmp280(struct _BMP280_HandleTypedef *bmp280, int BMP280_ADRESS);
void read_bmp280(struct _BMP280_HandleTypedef *bmp280, struct BMP280_data *BMP280_data_storage);
void update_pid();
void set_defaults();
void Redraw_display();

#endif /* INC_TEMP_CONTROLLER_H_ */
