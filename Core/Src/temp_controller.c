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
	toggle_red_LED();
	toggle_green_LED();
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
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	static uint32_t last_time;

	if (GPIO_Pin == PUSH_BUTTON_Pin) {};
	if (GPIO_Pin == ENCODER_PUSH_BUTTON_Pin)	{};
	if (GPIO_Pin == ENCODER_A_Pin){
		if(HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port,ENCODER_B_Pin))	{
			if((HAL_GetTick()-last_time) > ROTARY_SLOW)			temp_controller.target_temp-=0.1;
			else if((HAL_GetTick()-last_time) > ROTARY_FAST)	temp_controller.target_temp-=1;
			else												temp_controller.target_temp-=0.0;
			last_time = HAL_GetTick();
		}
	}
	if (GPIO_Pin == ENCODER_B_Pin){
		if(HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port,ENCODER_A_Pin))	{
			if((HAL_GetTick()-last_time) > ROTARY_SLOW)			temp_controller.target_temp+=0.1;
			else if((HAL_GetTick()-last_time) > ROTARY_FAST)	temp_controller.target_temp+=1;
			else												temp_controller.target_temp+=0.0;
			last_time = HAL_GetTick();
		}
	}
	//Redraw_display();
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}


void Redraw_display(){
	temp_controller.current_temp=temp_controller.target_temp;
	temp_controller.voltage=1.74;
	temp_controller.current=0.8;
	temp_controller.power=temp_controller.voltage*temp_controller.current;

	char current_temp_str[10], target_temp_str[10], voltage_str[10], current_str[10], power_str[10];
	short current_temp_str_nr, target_temp_str_nr, voltage_str_nr, current_str_nr, power_str_nr;
	short pwm_pixels = 40;
	short pwm = 0;

	current_temp_str_nr = ftoa(temp_controller.current_temp, current_temp_str, 2);
	target_temp_str_nr = ftoa(temp_controller.target_temp, target_temp_str, 1);
	voltage_str_nr = ftoa(temp_controller.voltage, voltage_str, 1);
	current_str_nr = ftoa(temp_controller.current, current_str, 1);
	power_str_nr = ftoa(temp_controller.power, power_str, 1);
	u8g2_FirstPage(&u8g2);
     do
     {
    	 u8g2_SetFont(&u8g2, u8g2_font_unifont_tf);
    	 u8g2_DrawUTF8(&u8g2, -1, 14, "Curr.");
    	 u8g2_DrawUTF8(&u8g2, 0, 26, "temp.:");
    	 u8g2_SetFont(&u8g2, u8g2_font_logisoso20_tf);
    	 if(temp_controller.current_temp<0) u8g2_DrawUTF8(&u8g2, 36, 20, "-"); //"-" sign
    	 u8g2_DrawUTF8(&u8g2, 48, 26, current_temp_str);
    	 u8g2_DrawUTF8(&u8g2, 48+current_temp_str_nr*15, 26, "°C");
    	 u8g2_SetFont(&u8g2, u8g2_font_helvR08_te);
    	 u8g2_DrawUTF8(&u8g2, 0, 46, "Set.");
    	 u8g2_DrawUTF8(&u8g2, 0, 58, "temp.:");
    	 u8g2_SetFont(&u8g2, u8g2_font_logisoso16_tf);	//The display has limited space
    	 if(temp_controller.target_temp<0) u8g2_DrawUTF8(&u8g2, 22, 52, "-");
    	 u8g2_DrawUTF8(&u8g2, 30, 58, target_temp_str);
    	 u8g2_DrawUTF8(&u8g2, 26+target_temp_str_nr*14, 58, "°C");
    	 u8g2_SetFont(&u8g2, u8g2_font_helvR08_te     );
    	 u8g2_DrawUTF8(&u8g2, 90, 46, "I:");
    	 u8g2_DrawUTF8(&u8g2, 100, 46, current_str);
    	 u8g2_DrawUTF8(&u8g2, 100+current_str_nr*8, 46, "A");
    	 u8g2_DrawUTF8(&u8g2, 86, 58, "P:");
    	 u8g2_DrawUTF8(&u8g2, 100, 58, power_str);
    	 u8g2_DrawUTF8(&u8g2, 100+power_str_nr*8, 58, "W");
    	 u8g2_SetDrawColor(&u8g2,2);
    	 u8g2_DrawHLine(&u8g2, 78, 33, (pwm/100.0)*pwm_pixels);
    	 u8g2_DrawHLine(&u8g2, 78, 32, (pwm/100.0)*pwm_pixels);
     }while (u8g2_NextPage(&u8g2));
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
