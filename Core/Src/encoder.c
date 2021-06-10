/*
 * encoder.c
 *
 *  Created on: 2020. dec. 20.
 *      Author: Norbert
 */

#include "encoder.h"

short is_long_pressed(GPIO_TypeDef* gpio_port, uint16_t button_pin, short polarity, uint16_t long_press){
	//In interrupt we cannot use HAL counter
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	//ITM->LAR = 0xC5ACCE55;
	DWT->CTRL |= 1 ; // enable the counter
	DWT->CYCCNT = 0; // reset the counter
	while(HAL_GPIO_ReadPin(gpio_port, button_pin) == polarity){
		__NOP();
		if(((DWT->CYCCNT/(SystemCoreClock/1000))> long_press)) return 1;
	}
	return 0;
}

uint32_t get_long_press_legth(GPIO_TypeDef* gpio_port, uint16_t button_pin){
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	//ITM->LAR = 0xC5ACCE55;
	DWT->CTRL |= 1 ; // enable the counter
	DWT->CYCCNT = 0; // reset the counter
	while(HAL_GPIO_ReadPin(gpio_port, button_pin) == 0){
			__NOP();
	}
	return DWT->CYCCNT/(SystemCoreClock/1000);
}

void snake_game_control(uint16_t GPIO_Pin){
	static uint32_t last_time;

	switch (GPIO_Pin){
	case ENCODER_PUSH_BUTTON_Pin:
		if (is_long_pressed(ENCODER_PUSH_BUTTON_GPIO_Port, ENCODER_PUSH_BUTTON_Pin, 0, LONG_PRESS)){
			temp_controller.flash.menu = 1;
			snake_button(END); //END
			return;
		}
	case ENCODER_A_Pin:
		if(HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port,ENCODER_B_Pin) && (HAL_GetTick()-last_time) > ROTARY_SLOW){
			last_time = HAL_GetTick();
			snake_button(RIGHT);
		}
	case ENCODER_B_Pin:
		if(HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port,ENCODER_A_Pin) && (HAL_GetTick()-last_time) > ROTARY_SLOW)	{
			last_time = HAL_GetTick();
			snake_button(LEFT);
		}
	}
}

float *get_rotating_menu_item(temperature_controller_data* controller){
	if(controller->flash.menu==SET_Temp_MENU)		return	&controller->flash.target_temp;
	if(controller->flash.menu==SET_Kp_MENU)			return	&controller->flash.pid.Kp;
	if(controller->flash.menu==SET_Kd_MENU)			return	&controller->flash.pid.Kd;
	if(controller->flash.menu==SET_Ki_MENU)			return	&controller->flash.pid.Ki;
	if(controller->flash.menu==SET_MAX_P_MENU)		return	&controller->flash.pid.max_P;
	if(controller->flash.menu==SET_P_MENU)			return	&controller->flash.set_power;
	if(controller->flash.menu==SET_MODE_MENU)		return	&controller->flash.mode;
	if(controller->flash.menu==CHOOSE_NTC_MENU)		return	&controller->flash.sensor;
	return &controller->dummy;
}

void rotate(int value, int* ptr){
	*ptr += value;
}

//Interrupt function called on button press
void encoder (uint16_t GPIO_Pin){
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	if(temp_controller.flash.menu == SNAKE_MENU) {					//snake game
		snake_game_control(GPIO_Pin);
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
		return;
	}
	static uint32_t last_time;
	float* ptr=get_rotating_menu_item(&temp_controller);

	switch(GPIO_Pin){
	case ENCODER_PUSH_BUTTON_Pin:
		if (temp_controller.flash.menu == SET_DEFAULTS_MENU && is_long_pressed(ENCODER_PUSH_BUTTON_GPIO_Port, ENCODER_PUSH_BUTTON_Pin, 0, LONG_PRESS)){
			set_defaults();
			write_flash();
			last_time = HAL_GetTick();
			break;
		}
		if (is_long_pressed(ENCODER_PUSH_BUTTON_GPIO_Port, ENCODER_PUSH_BUTTON_Pin, 0, LONG_LONG_PRESS)){
			temp_controller.flash.menu = SNAKE_MENU;
			last_time = HAL_GetTick();
			break;
		}

		if((HAL_GetTick()-last_time) > SHORT_PRESS)  temp_controller.flash.menu++;
		if(temp_controller.flash.menu == SET_P_MENU) temp_controller.flash.set_power=0;
		if(temp_controller.flash.menu > MENU_MAX-1) temp_controller.flash.menu=1;
		last_time = HAL_GetTick();
		break;
	case ENCODER_A_Pin:	//decrement
		if(HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port,ENCODER_B_Pin))	{
			float change_slow=1;
			short change_fast=10;
			if(temp_controller.flash.menu == SET_MAX_P_MENU && temp_controller.flash.menu == SET_P_MENU) {
				change_slow=1;
				change_fast=2;
			}
			if(temp_controller.flash.menu == SET_Kd_MENU) {
				change_slow=10;
				change_fast=100;
			}
			if(temp_controller.flash.menu == SET_Ki_MENU || temp_controller.flash.menu == CHOOSE_NTC_MENU) {
				change_slow=1;
				change_fast=1;
			}
			if(temp_controller.flash.menu == SET_MODE_MENU) {
				change_slow=2;
				change_fast=2;
			}
			if((HAL_GetTick()-last_time) > ROTARY_SLOW)			rotate(-change_slow,ptr);
			else if((HAL_GetTick()-last_time) > ROTARY_FAST)	rotate(-change_fast,ptr);
			else												break;
			if(temp_controller.flash.mode <= -1)	temp_controller.flash.mode = -1;
			if(temp_controller.flash.sensor < 1)	temp_controller.flash.sensor = thermistor_count;
			temp_controller.flash.defaults = 0;
			write_flash();
			if(temp_controller.flash.menu == SET_P_MENU)	{
				set_duty_cycle(temp_controller.flash.set_power);
				if(temp_controller.flash.set_power<0) temp_controller.flash.set_power = 0;
			}
			last_time = HAL_GetTick();
		}
		break;
	case ENCODER_B_Pin:	//increment
		if(HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port,ENCODER_A_Pin))	{
			float change_slow=1;
			short change_fast=10;
			if(temp_controller.flash.menu == SET_MAX_P_MENU && temp_controller.flash.menu == SET_P_MENU) {
				change_slow=1;
				change_fast=2;
			}
			if(temp_controller.flash.menu == SET_Kd_MENU) {
				change_slow=10;
				change_fast=100;
			}
			if(temp_controller.flash.menu == SET_Ki_MENU) {
				change_slow=1;
				change_fast=1;
			}
			if(temp_controller.flash.menu == SET_MODE_MENU) {
				change_slow=2;
				change_fast=2;
			}
			if((HAL_GetTick()-last_time) > ROTARY_SLOW)			rotate(change_slow,ptr);
			else if((HAL_GetTick()-last_time) > ROTARY_FAST)	rotate(change_fast,ptr);
			else												break;
			if(temp_controller.flash.mode >= 1)	temp_controller.flash.mode = 1;
			if(temp_controller.flash.sensor > thermistor_count)	temp_controller.flash.sensor = 1;
			temp_controller.flash.defaults = 0;
			write_flash();
			if(temp_controller.flash.menu == SET_P_MENU)	{
				if(temp_controller.flash.set_power >100) temp_controller.flash.set_power = 100;
				set_duty_cycle(temp_controller.flash.set_power);
			}
			last_time = HAL_GetTick();
		}
		break;
	}
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
