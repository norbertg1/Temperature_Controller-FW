/*
 * interrupt_callbacks.c
 *
 *  Created on: 2020. dec. 20.
 *      Author: Norbert
 */

#include "main.h"
#include "interrupt_callbacks.h"

uint32_t adc[2];
adc_data ntc;
float Vref=1.225,
		R0 = 10000;
short defaults=0, flag_10ms, flag_200ms, flag_1s,  flag_10s;

void TIM4_callback(){ 	//10ms interrupt, 100Hz
	HAL_SDADC_Start_DMA(&hsdadc1, (uint32_t*) adc_buf, 1);
	HAL_SDADC_Start_DMA(&hsdadc2, (uint32_t*) &adc_buf[1], 1);
	//update_pid();
	flag_10ms=1;
}

void TIM12_callback(){	//10ms interrupt, 100Hz

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
		if(temp_controller.menu != STARTUP_MENU)	update_pid();
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
    while(R<lookup_temp_table[i][1]){
        i++;
        if((i+1)==sizeof(lookup_temp_table)/sizeof(int)/2) {
                deltaT = lookup_temp_table[i][0]-lookup_temp_table[i-1][0];
                deltaR = lookup_temp_table[i][1]-lookup_temp_table[i-1][1];
                T=lookup_temp_table[i][0]+(R-lookup_temp_table[i][1])*deltaT/deltaR;
                return T;
        }
    }
    deltaT = lookup_temp_table[i+1][0]-lookup_temp_table[i][0];
    deltaR = lookup_temp_table[i+1][1]-lookup_temp_table[i][1];
    T=lookup_temp_table[i][0]+(R-lookup_temp_table[i][1])*deltaT/deltaR;
    return T;
}


//Interrupt function called on button press
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin){
	encoder (GPIO_Pin);
}


