/*
 * temp_controller.c
 *
 *  Created on: Nov 11, 2020
 *      Author: Norbert
 */

#include "temp_controller.h"

I2C_HandleTypeDef hi2c2;

adc_data ntc;
temperature_controller_data temp_controller;
BMP280_data BMP280_sensor;
struct _BMP280_HandleTypedef bmp280;
float Vref=1.225, R0 = 10000;
int lookup_temp_table[2][4] = {
		{0,1,2,3},
		{10,20,30,40}
};



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
	while(1){
		toggle_red_LED();
		toggle_green_LED();
		HAL_Delay(1000);
	}
}

void start_pwm(TIM_HandleTypeDef *htim){
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);		//PWM enable
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);		//PWM enable
}

void set_duty_cycle(int dc){
	TIM2->CCR1=dc;
	TIM2->CCR2=dc;
}

void update_pid(){

}

void set_defaults(){
	HAL_Delay(500);
	temp_controller.target_temp = 0;
	write_to_display();
}

//Interrupt function called on button press
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin){
	//HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	static int x=0,y=0,set_temp=0;
	if (GPIO_Pin == PUSH_BUTTON_Pin) x++;
	if (GPIO_Pin == ENCODER_PUSH_BUTTON_Pin)	y++;
	if (GPIO_Pin == ENCODER_A_Pin){
		if(HAL_GPIO_ReadPin(ENCODER_A_GPIO_Port,ENCODER_A_Pin)	==	HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port,ENCODER_B_Pin))	{
			temp_controller.target_temp-=0.1;
		}
		else	{
			temp_controller.target_temp+=0.1;
		}
	}
	write_to_display();
	//HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void write_to_display(){
	char str[10],chars;
	chars = ftoa(temp_controller.target_temp, str, 2);

}

//Interrupt function called on completed ADC conversion
void HAL_SDADC_ConvCpltCallback(SDADC_HandleTypeDef* hsdadc){
	ntc.current			=	((Vref/16635.0) *  adc_buf[0])/10000;
	ntc.voltage			=	(Vref/16635.0) *  adc_buf[1];
	ntc.resistance		=	ntc.voltage/ntc.current;
	ntc.temperature		=	lookup_temp(ntc.resistance);
}

//gives back the temperature based on NTC resistance value, lookup table needed!
float lookup_temp(float R){
    int i = 0;
    float deltaT,deltaR,T;
    while(R<lookup_temp_table[1][i]){
        i++;
        if((i+1)==sizeof(lookup_temp_table)/sizeof(int)/2) {
                deltaT = lookup_temp_table[0][i]-lookup_temp_table[0][i-1];
                deltaR = lookup_temp_table[1][i]-lookup_temp_table[1][i-1];
                T=lookup_temp_table[0][i]+(R-lookup_temp_table[1][i])*deltaT/deltaR;
                return T;
        }
    }
    deltaT = lookup_temp_table[0][i+1]-lookup_temp_table[0][i];
    deltaR = lookup_temp_table[1][i+1]-lookup_temp_table[1][i];
    T=lookup_temp_table[0][i]+(R-lookup_temp_table[1][i])*deltaT/deltaR;
    return T;
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
