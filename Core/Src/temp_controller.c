/*
 * temp_controller.c
 *
 *  Created on: Nov 11, 2020
 *      Author: Norbert
 */

#include "temp_controller.h"

I2C_HandleTypeDef hi2c2;

temperature_controller_data temp_controller;

BMP280_data BMP280_sensor;
struct _BMP280_HandleTypedef bmp280;

short cnt_adc=0;

void set_duty_cycle(float percent){
	int set_pwm = (PWM_COUNTER_PERIOD/100.0)*percent;
	if(temp_controller.flash.menu == TOO_HOT_MENU) set_pwm = 0;
	TIM2->CCR1=set_pwm;
	TIM2->CCR2=set_pwm;
}

void update_pid(){
	static uint32_t t,cnt=0, last_t[100];
	static float prev_error;
	float error, d_error, delta_t, pid_out;
	temp_controller.flash.pid.delta_t = (HAL_GetTick()-t)/1000.0;
	error = temp_controller.flash.target_temp/10.0 - temp_controller.current_temp;
	d_error = error - prev_error;
	prev_error = error;
	float kd = temp_controller.flash.pid.Kd/10.0 * d_error * temp_controller.flash.pid.delta_t;
	temp_controller.flash.pid.out = temp_controller.flash.mode * (
			temp_controller.flash.pid.Kp/10.0 * error
			+ temp_controller.flash.pid.Kd/10.0 * d_error * temp_controller.flash.pid.delta_t
			+ temp_controller.flash.pid.Ki/10.0 * temp_controller.flash.pid.errorSum);

	temp_controller.flash.target_temp = round(temp_controller.flash.target_temp*10)/10;
	temp_controller.flash.pid.errorSum += 0.05*error;
	if(temp_controller.flash.pid.errorSum > 200) temp_controller.flash.pid.errorSum=200;
	if(temp_controller.flash.pid.errorSum < -200) temp_controller.flash.pid.errorSum=-200;
	if(temp_controller.flash.pid.out > temp_controller.flash.pid.max_P) temp_controller.flash.pid.out = temp_controller.flash.pid.max_P;
	if(temp_controller.flash.pid.out < 0) temp_controller.flash.pid.out = 0; 					//The hardware doesnt support heating
	if(temp_controller.current_temp > CUT_OFF_TEMP && temp_controller.flash.mode == -1)	temp_controller.flash.menu = TOO_HOT_MENU;
	if (temp_controller.flash.menu != SET_P_MENU)  set_duty_cycle(temp_controller.flash.pid.out);
	t=HAL_GetTick();
	last_t[cnt++]=temp_controller.flash.pid.delta_t*1000;
	if (cnt==99) cnt=0;
}

void read_flash(){
	uint32_t crc;

	flash_ReadN(0,&temp_controller.flash,ceil(sizeof(temp_controller.flash)/8.0),DATA_TYPE_64);
	crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)&temp_controller.flash, sizeof(temp_controller.flash));	//Calculate the CRC only for values which are set by button and exclude the CRC variable
	if(crc != temp_controller.crc)	set_defaults();
	set_duty_cycle(0);
	temp_controller.flash.pid.out = 0;
	temp_controller.flash.menu = 0;
	temp_controller.flash.pid.errorSum = 0;
	Redraw_display();
}

void write_flash(){
	temp_controller.crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)&temp_controller.flash, sizeof(temp_controller.flash));	//Calculate the CRC only for values which are set by button and exclude the CRC variable
	flash_WriteN(0, &temp_controller.flash,ceil(sizeof(temp_controller.flash)/8.0),DATA_TYPE_64);

}

void init_bmp280(BMP280_HandleTypedef *bmp280, int BMP280_ADRESS){
	bmp280_init_default_params(&bmp280->params);
	bmp280->addr = BMP280_ADRESS;
	bmp280->i2c = &hi2c2;
	while (!bmp280_init(bmp280, &bmp280->params)) {
		HAL_Delay(2000);
	}
	float temperature, pressure, humidity;
	while (!bmp280_read_float(bmp280, &temperature, &pressure, &humidity)) {
		HAL_Delay(2000);
		//Problem with BMP280
	}
}

void read_bmp280(struct _BMP280_HandleTypedef *bmp280, struct BMP280_data *BMP280_data_storage){
	while (!bmp280_read_float(bmp280, &BMP280_data_storage->temp, &BMP280_data_storage->pressure, &BMP280_data_storage->humidity)) {
		HAL_Delay(2000);
		//Problem with BMP280
	}
}

void turn_on_green_LED(){
	HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 1);
}
void turn_off_green_LED(){
	HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 0);
}
void toggle_green_LED(){
	HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, HAL_GPIO_ReadPin(GREEN_LED_GPIO_Port, GREEN_LED_Pin)^1);
}
void turn_on_red_LED(){
	HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 1);
}
void turn_off_red_LED(){
	HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 0);
}
void toggle_red_LED(){
	HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, HAL_GPIO_ReadPin(RED_LED_GPIO_Port, RED_LED_Pin)^1);
}

void blink(){
	toggle_red_LED();
	toggle_green_LED();
}
