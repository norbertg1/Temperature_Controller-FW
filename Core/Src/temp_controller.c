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
float Vref=1.225,
		R0 = 10000;
int lookup_temp_table[2][4] = {
		{0,1,2,3},
		{10E3,20E3,30E3,40E3}
};
uint32_t adc[2];
short flag_10ms, flag_200ms, flag_1s,  flag_10s, cnt_adc=0;

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
	static uint32_t t;
	float error, prev_error, d_error, delta_t, pid_out;
	temp_controller.pid.delta_t = (HAL_GetTick()-t)/1000.0;
	prev_error = temp_controller.pid.error;
	error = temp_controller.target_temp - temp_controller.current_temp;
	d_error = prev_error - temp_controller.pid.error;
	pid_out = temp_controller.pid.Kp * error
			+ temp_controller.pid.Kd * d_error * delta_t
			+ temp_controller.pid.Ki * temp_controller.pid.errorSum;
	temp_controller.pid.errorSum += error;
	set_duty_cycle(50);
	t=HAL_GetTick();
}

void set_defaults(){
	HAL_Delay(500);
	temp_controller.target_temp = 0;
	temp_controller.menu = 1;
	temp_controller.pid.errorSum = 0;
	temp_controller.pid.Kp=0.1;
	temp_controller.pid.Kd=0.1;
	temp_controller.pid.Ki=0.1;
	Redraw_display();
}

float *get_rotating_menu_item(temperature_controller_data* controller){
	if(controller->menu==1)	return	&controller->target_temp;
	if(controller->menu==2)	return	&controller->pid.Kp;
	if(controller->menu==3)	return	&controller->pid.Kd;
	if(controller->menu==4)	return	&controller->pid.Ki;

}

void rotate(float value, float* ptr){
	*ptr+=value;
}

float round_n(float number, int dec){
	return roundf((10*dec) * number) / (10*dec);
}

short is_long_pressed(GPIO_TypeDef* gpio_port, uint16_t button_pin, short polarity, uint16_t long_press){
	//In interrupt we cannot use HAL counter
	DWT->CTRL |= 1 ; // enable the counter
	DWT->CYCCNT = 0; // reset the counter
	while(HAL_GPIO_ReadPin(gpio_port, button_pin) == polarity){
		__NOP();
		if(((DWT->CYCCNT/(SystemCoreClock/1000))> long_press)) return 1;
	}
	return 0;
}

void snake_game_control(uint16_t GPIO_Pin){
	static uint32_t last_time;
	switch (GPIO_Pin){
	case ENCODER_PUSH_BUTTON_Pin:
		if (is_long_pressed(ENCODER_PUSH_BUTTON_GPIO_Port, ENCODER_PUSH_BUTTON_Pin, 0, LONG_PRESS)){
			temp_controller.menu = 1;
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

//Interrupt function called on button press
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin){
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	if(temp_controller.menu == 5) {					//snake game
		snake_game_control(GPIO_Pin);
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
		return;
	}
	static uint32_t last_time;
	float* ptr=get_rotating_menu_item(&temp_controller);
	switch(GPIO_Pin){
	case ENCODER_PUSH_BUTTON_Pin:
		if (is_long_pressed(ENCODER_PUSH_BUTTON_GPIO_Port, ENCODER_PUSH_BUTTON_Pin, 0, LONG_PRESS)){
			temp_controller.menu = 5;
			return;
		}
		if((HAL_GetTick()-last_time) > SHORT_PRESS)  temp_controller.menu++;
		if(temp_controller.menu > MENU_MAX-1) temp_controller.menu=1;
		last_time = HAL_GetTick();
		break;
	case ENCODER_A_Pin:
		if(HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port,ENCODER_B_Pin))	{
			if((HAL_GetTick()-last_time) > ROTARY_SLOW)			rotate(-0.1,ptr);
			else if((HAL_GetTick()-last_time) > ROTARY_FAST)	rotate(-1,ptr);
			else												break;
			last_time = HAL_GetTick();
		}
		break;
	case ENCODER_B_Pin:
		if(HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port,ENCODER_A_Pin))	{
			if((HAL_GetTick()-last_time) > ROTARY_SLOW)			rotate(+0.1,ptr);
			else if((HAL_GetTick()-last_time) > ROTARY_FAST)	rotate(+1,ptr);
			else												break;
			last_time = HAL_GetTick();
		}
		break;
	}
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}


void Redraw_display(){
	int t1,t2,delta_t;

	switch(temp_controller.menu)
	{
	case 1:
		if(INA226_1.Result.Current_uA 	>= 	INA226_2.Result.Current_uA) {
			temp_controller.current	= 	INA226_1.Result.Current_uA/10E6;
			temp_controller.power 	= 	INA226_1.Result.Power_uW /10E6;
		}
		else {
			temp_controller.current =	INA226_2.Result.Current_uA/10E6;
			temp_controller.power 	= 	INA226_2.Result.Power_uW /10E6;
		}

		char current_temp_str[10], target_temp_str[10], voltage_str[10], current_str[10], power_str[10];
		short current_temp_str_nr, target_temp_str_nr, voltage_str_nr, current_str_nr, power_str_nr;
		short pwm_pixels = 40;
		short pwm = 0;

		current_temp_str_nr = ftoa(temp_controller.current_temp, current_temp_str, 2);
		target_temp_str_nr = ftoa(temp_controller.target_temp, target_temp_str, 1);
		voltage_str_nr = ftoa(temp_controller.voltage, voltage_str, 1);
		current_str_nr = ftoa(temp_controller.current, current_str, 1);
		power_str_nr = ftoa(temp_controller.power, power_str, 1);

		u8g2_ClearBuffer(&u8g2);
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
		u8g2_SendBuffer(&u8g2);
		break;

	case 2:
		menu234();
		u8g2_DrawUTF8(&u8g2, 0, 14, "*");
		u8g2_SendBuffer(&u8g2);
		break;
	case 3:
		menu234();
		u8g2_DrawUTF8(&u8g2, 0, 28, "*");
		u8g2_SendBuffer(&u8g2);
		break;
	case 4:
		menu234();
		u8g2_DrawUTF8(&u8g2, 0, 42, "*");
		u8g2_SendBuffer(&u8g2);
		break;
	}
}

void menu234(){
	//Case Menu = 2,3,4
	char Kp_str[10], Kd_str[10], Ki_str[10];
	ftoa(temp_controller.pid.Kp, Kp_str, 3);
	ftoa(temp_controller.pid.Kd, Kd_str, 3);
	ftoa(temp_controller.pid.Ki, Ki_str, 3);
	u8g2_ClearBuffer(&u8g2);
	u8g2_SetFont(&u8g2, u8g2_font_unifont_tf);
	u8g2_DrawUTF8(&u8g2, 10, 14, "Kp: ");
	u8g2_DrawUTF8(&u8g2, 42, 14, Kp_str);
	u8g2_DrawUTF8(&u8g2, 10, 28, "Kd: ");
	u8g2_DrawUTF8(&u8g2, 42, 28, Kd_str);
	u8g2_DrawUTF8(&u8g2, 10, 42, "Ki: ");
	u8g2_DrawUTF8(&u8g2, 42, 42, Ki_str);
}

void TIM4_callback(){ //10ms interrupt, 100Hz
	flag_10ms=1;
}

void TIM6_callback(){ //200ms interrupt
	//HAL_NVIC_DisableIRQ(TIM6_DAC1_IRQn);
	flag_200ms=1;
}

void TIM3_callback(){	//1s interrupt
	flag_1s=1;
}

void TIM7_callback(){ //10s interrupt, 0.1Hz
	flag_10s=1;
}

//Interrupt function called on completed ADC conversion
void HAL_SDADC_ConvCpltCallback(SDADC_HandleTypeDef* hsdadc){
	adc[0]		+=		adc_buf[0];
	adc[1]		+=		adc_buf[1];
	cnt_adc++;
	if(cnt_adc >= ADC_AVARAGE){
		HAL_SDADC_Stop_DMA(&hsdadc1);
		HAL_SDADC_Stop_DMA(&hsdadc2);
		calc_adc_values();
	}
}

void calc_adc_values(){
	adc[0]							=		adc[0]/cnt_adc;
	adc[1]							=		adc[1]/cnt_adc;
	ntc.current						=		((Vref/32767.0) *  adc[0])/10000;
	ntc.voltage						=		(Vref/32767.0) *  adc[1];
	ntc.resistance					=		ntc.voltage/ntc.current;
	ntc.temperature					=		lookup_temp(ntc.resistance);
	cnt_adc							=		0;
	adc[0]							=		0;
	adc[1]							=		0;
	temp_controller.current_temp	=		ntc.temperature;
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

void valami(){
	  if(flag_1s){
		  flag_1s=0;
		  INA226_MeasureAll(&INA226_1);
		  INA226_MeasureAll(&INA226_2);
	  }
	  if(flag_10ms){
		  flag_10ms=0;
	  }
	  Redraw_display();
	  HAL_Delay(100);
}
