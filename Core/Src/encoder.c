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

void *get_rotating_menu_item(temperature_controller_data* controller){
	if(controller->flash.menu==STARTUP_MENU)			return	&controller->flash.target_temp;
	if(controller->flash.menu==SET_Temp_MENU)			return	&controller->flash.target_temp;
	if(controller->flash.menu==SET_Kp_MENU)				return	&controller->flash.pid.Kp;
	if(controller->flash.menu==SET_Kd_MENU)				return	&controller->flash.pid.Kd;
	if(controller->flash.menu==SET_Ki_MENU)				return	&controller->flash.pid.Ki;
	if(controller->flash.menu==SET_MAX_P_MENU)			return	&controller->flash.pid.max_P;
	if(controller->flash.menu==SET_P_MENU)				return	&controller->flash.set_power;
	if(controller->flash.menu==SET_MODE_MENU)			return	&controller->flash.mode;
	if(controller->flash.menu==CHOOSE_NTC_MENU)			return	&controller->flash.sensor;
	if(controller->flash.menu==SET_FREQUENCY_MENU)		return	&controller->flash.freq;
	if(controller->flash.menu==SET_TEMPOFFSET_MENU)			return	&controller->flash.offset_temp;
	return &controller->dummy;
}

float get_rotate_slow_value(temperature_controller_data* controller, int menu){
	switch(menu){
	case SET_Kd_MENU:
		return 10;
	case SET_MODE_MENU:
		return 2;
	case SET_TEMPOFFSET_MENU:
		return 1;
	}
	if(controller->flash.target_temp > 1000 && (temp_controller.flash.menu == SET_Temp_MENU || temp_controller.flash.menu == STARTUP_MENU)) {
		temp_controller.flash.target_temp = (round(temp_controller.flash.target_temp/10))*10;
		return 1;
	}
	return 1;
}

short get_rotate_fast_value(temperature_controller_data* controller, int menu){
	switch(menu){
	case SET_MAX_P_MENU:
	case SET_MODE_MENU:
	case SET_P_MENU:
		return 2;
	case SET_Kd_MENU:
		return 100;
	case CHOOSE_NTC_MENU:
	case SET_Ki_MENU:
		return 1;
	case SET_TEMPOFFSET_MENU:
		return 10;
	}
	if(controller->flash.target_temp > 1000 && (temp_controller.flash.menu == SET_Temp_MENU || temp_controller.flash.menu == STARTUP_MENU)) {
		temp_controller.flash.target_temp = (round(temp_controller.flash.target_temp/10))*10;
		return 100;
	}
	return 10;
}

void rotate(int value, int* ptr){
	*ptr += value;
}

void settings_limits(temperature_controller_data* controller){
	controller->flash.defaults=0;
	switch(controller->flash.menu){
	case SET_MODE_MENU:
		if(controller->flash.mode <= -1)	controller->flash.mode = -1;
		if(controller->flash.mode >= 1)	controller->flash.mode = 1;
		break;
	case CHOOSE_NTC_MENU:
		if(controller->flash.sensor < 1)	controller->flash.sensor = thermistor_count;
		if(controller->flash.sensor > thermistor_count)	controller->flash.sensor = 1;
		break;
	case SET_P_MENU:
		if(controller->flash.set_power<0) controller->flash.set_power = 0;
		if(controller->flash.set_power >100) controller->flash.set_power = 100;
		set_duty_cycle(controller->flash.set_power);
		break;
	case SET_FREQUENCY_MENU:
		if(controller->flash.freq < PWM_FREQ_MIN)	controller->flash.freq = PWM_FREQ_MIN;
		if(controller->flash.freq > PWM_FREQ_MAX)	controller->flash.freq = PWM_FREQ_MAX;
		controller->pwm_counter_period = (HAL_RCC_GetSysClockFreq()/1E3)/controller->flash.freq;
		break;
	case SET_TEMPOFFSET_MENU:
		if(controller->flash.offset_temp < TEMP_OFFSET_MIN)	controller->flash.offset_temp = TEMP_OFFSET_MIN;
		if(controller->flash.offset_temp > TEMP_OFFSET_MAX)	controller->flash.offset_temp = TEMP_OFFSET_MAX;
	}
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
	int* ptr=get_rotating_menu_item(&temp_controller);
	float change_slow = get_rotate_slow_value(&temp_controller, temp_controller.flash.menu);
	short change_fast = get_rotate_fast_value(&temp_controller, temp_controller.flash.menu);

	switch(GPIO_Pin){
	case ENCODER_PUSH_BUTTON_Pin:	//Push button
		if (is_long_pressed(ENCODER_PUSH_BUTTON_GPIO_Port, ENCODER_PUSH_BUTTON_Pin, 0, LONG_PRESS)){	//start snake game
			if(MENU_LEVEL1_MIN <= temp_controller.flash.menu <= MENU_LEVEL1_MAX)	temp_controller.flash.menu = SET_Temp_MENU;
			if(MENU_LEVEL2_MIN <= temp_controller.flash.menu <= MENU_LEVEL2_MAX)	temp_controller.flash.menu = SET_Kp_MENU;
			last_time = HAL_GetTick();
			break;
			}
		if (temp_controller.flash.menu == SET_DEFAULTS_MENU && is_long_pressed(ENCODER_PUSH_BUTTON_GPIO_Port, ENCODER_PUSH_BUTTON_Pin, 0, LONG_PRESS)){	//set all settings on default
			set_defaults();
			write_flash();
			last_time = HAL_GetTick();
			break;
		}
		if (is_long_pressed(ENCODER_PUSH_BUTTON_GPIO_Port, ENCODER_PUSH_BUTTON_Pin, 0, LONG_LONG_PRESS)){	//start snake game
			temp_controller.flash.menu = SNAKE_MENU;
			last_time = HAL_GetTick();
			break;
			}

		if((HAL_GetTick()-last_time) > SHORT_PRESS)  temp_controller.flash.menu++;
		if(temp_controller.flash.menu == SET_P_MENU) temp_controller.flash.set_power=0;
		if(temp_controller.flash.menu == MENU_LEVEL1_MAX)  temp_controller.flash.menu=SET_Temp_MENU;
		if(temp_controller.flash.menu == MENU_LEVEL2_MAX)  temp_controller.flash.menu=SET_Kp_MENU;
		last_time = HAL_GetTick();
		break;
	case ENCODER_A_Pin:				//Decrement
		if(HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port,ENCODER_B_Pin))	{
			if((HAL_GetTick()-last_time) > ROTARY_SLOW)			rotate(-change_slow,ptr);
			else if((HAL_GetTick()-last_time) > ROTARY_FAST)	rotate(-change_fast,ptr);
			else												break;
			settings_limits(&temp_controller);
			write_flash();
			last_time = HAL_GetTick();
		}
		break;
	case ENCODER_B_Pin:	//Increment
		if(HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port,ENCODER_A_Pin))	{
			if((HAL_GetTick()-last_time) > ROTARY_SLOW)			rotate(change_slow,ptr);
			else if((HAL_GetTick()-last_time) > ROTARY_FAST)	rotate(change_fast,ptr);
			else												break;
			settings_limits(&temp_controller);
			write_flash();
			last_time = HAL_GetTick();
		}
		break;
	}
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
