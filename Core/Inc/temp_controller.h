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
#include "NTC_lookup_table.h"
#include "interrupt_callbacks.h"
#include "encoder.h"

#define ADC_BUF_LEN						2
#define ROTARY_FAST						20 // [ms] rotary speed fast
#define ROTARY_SLOW						200 // [ms] rotary speed slow
#define MENU_MAX						11
#define STARTUP_MENU					0
#define SET_Temp_MENU					1
#define SET_Kp_MENU						2
#define SET_Kd_MENU						3
#define SET_Ki_MENU						4
#define SET_MAX_P_MENU					5
#define SET_MODE_MENU					6	//Heating or cooling
#define CHOOSE_NTC_MENU					7
#define CHOOSE_FREQUENCY_MENU			8
#define SET_P_MENU						9
#define SET_DEFAULTS_MENU				10
#define SNAKE_MENU						11
#define TOO_HOT_MENU					12		//When the Peltier is connected with wrong polarity, the temperature can goo very high within seconds. This menu prevents it with cut off power.
#define NTC_INFINTE_RESISTANCE_MENU		13
#define NTC_INFINITE_RESISTANCE			100000	//Resistance measured greater than this value in ohms means NTC is not connected
#define CUT_OFF_TEMP					80		//Its happens when the Peltier reaches this temperature in °C
#define ADC_AVARAGE						50
#define LONG_PRESS						3000
#define LONG_LONG_PRESS					5000
#define SHORT_PRESS						100
#define	END								0
#define RIGHT							2
#define	LEFT							4
#define	PWM_FREQ_MIN					1		//In kHz In case of 72 Mhz internal clock ===> 72000/100   = 720 Khz
#define PWM_FREQ_MAX					720		//In kHz case of 72 Mhz internal clock ===> 72000/72000 = 1 Khz
#define	MAX_POWER_PERCENT				80

uint16_t adc_buf[ADC_BUF_LEN];

//UPDATED BY ADC DMA
typedef struct    {
	double current;
	double voltage;
	double resistance;
	float  temperature;
}adc_data;

typedef struct PID{
	int 	Kp;
	int 	Kd;
	int 	Ki;
	float 	error;
	float 	errorSum;
	float 	delta_t;
	int 	max_P;         //in percent
	float 	out;
}PID;

typedef struct flash{
	int 		target_temp;
	short 		menu;
	short 		defaults;
	short 		set_power;
	int 		mode;			//-1 ---> cooling, 1 ---> heating
	int 		sensor;		//NTC sensor choose from flash data
	long int	freq;		//PWM frequency for power modules
	PID	  		pid;
}flash;

typedef struct    {
	float 		current_temp;
	float 		voltage;
	float 		current;
	float 		power;
	long int	pwm_counter_period;
	short		ntc_interrupted;
	short 		dummy;		//if non exist menu is set, step this variable with encoder
	flash 		flash;
	uint32_t 	crc;
} temperature_controller_data;

typedef struct BMP280_data{
	float temp;
	float pressure;
	float humidity;	//BMP280 not support it
}BMP280_data;

enum sensors{
	NTCS0603E3222FMT	= 1,
	NTCG163JX103DTDS	= 2,
	NTC_100K			= 3,
	PT1000				= 4,
	thermistor_count 	= 4
};

extern short cnt_adc;
extern temperature_controller_data temp_controller;

extern void calc_adc_values();
void turn_on_green_LED();
void turn_off_green_LED();
void toggle_green_LED();
void turn_on_red_LED();
void turn_off_red_LED();
void toggle_red_LED();
void blink();
void start_pwm(TIM_HandleTypeDef *);
void set_duty_cycle(float);
void HAL_GPIO_EXTI_Callback (uint16_t );
void HAL_SDADC_ConvCpltCallback(SDADC_HandleTypeDef* hsdadc);
float lookup_temp(float R);
void init_bmp280(struct _BMP280_HandleTypedef *, int );
void read_bmp280(struct _BMP280_HandleTypedef *, struct BMP280_data *);
void update_pid();
void set_defaults();
void read_flash();
void write_flash();
void Redraw_display();
void menu234();


#endif /* INC_TEMP_CONTROLLER_H_ */
