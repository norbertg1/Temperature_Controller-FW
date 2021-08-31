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
		R0 = 3005; //10000;
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

/*void copy_wpadding(flash *flash_padded, flash_unpadded *flash_unpadded, bool direction){
	if(direction){
		flash_unpadded->defaults = flash_padded->defaults;
	}
	if(!direction){

	}

}*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	typedef struct __attribute__((packed)){
		int 			Kp;
		int 			Kd;
		int 			Ki;
		float 			error;
		float 			errorSum;
		float 			delta_t;
		int 			max_P;         //in percent
		float 			out;
	}PID_unpadded;

	struct __attribute__((packed)) {
		int 			target_temp;
		int				offset_temp;
		short 			menu;
		short 			defaults;
		short 			set_power;
		int 			mode;			//-1 ---> cooling, 1 ---> heating
		int 			sensor;		//NTC sensor choose from flash data
		long int		freq;		//PWM frequency for power modules
		PID_unpadded	pid;
		uint32_t		crc;
	}flash_unpadded;

	HAL_NVIC_DisableIRQ(TIM6_DAC1_IRQn);
	if(UART_rxBuffer[0] == 'S'){
		flash_unpadded.target_temp = temp_controller.flash.target_temp;
		flash_unpadded.offset_temp = temp_controller.flash.offset_temp;
		flash_unpadded.menu = temp_controller.flash.menu;
		flash_unpadded.defaults = temp_controller.flash.defaults;
		flash_unpadded.set_power = temp_controller.flash.set_power;
		flash_unpadded.mode = temp_controller.flash.mode;
		flash_unpadded.sensor = temp_controller.flash.sensor;
		flash_unpadded.freq = temp_controller.flash.freq;
		flash_unpadded.pid.Kp = temp_controller.flash.pid.Kp;
		flash_unpadded.pid.Kd = temp_controller.flash.pid.Kd;
		flash_unpadded.pid.Ki = temp_controller.flash.pid.Ki;
		flash_unpadded.pid.error = temp_controller.flash.pid.error;
		flash_unpadded.pid.errorSum = temp_controller.flash.pid.errorSum;
		flash_unpadded.pid.delta_t = temp_controller.flash.pid.delta_t;
		flash_unpadded.pid.max_P = temp_controller.flash.pid.max_P;
		flash_unpadded.pid.out = temp_controller.flash.pid.out;

		flash_unpadded.crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)&flash_unpadded, sizeof(flash_unpadded) - sizeof(flash_unpadded.crc));
		HAL_UART_Transmit(&huart2, (uint8_t*)&flash_unpadded, sizeof (flash_unpadded),1000);
	}
	else{
		//temp_controller.flash.target_temp = UART_rxBuffer;
	}
	HAL_NVIC_EnableIRQ(TIM6_DAC1_IRQn);
	HAL_UART_Receive_IT (&huart2, &UART_rxBuffer[0], 64);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	HAL_StatusTypeDef error  = huart->ErrorCode;
}

void USART2_callback(){

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
		if (ntc.resistance > NTC_INFINITE_RESISTANCE) temp_controller.flash.menu = NTC_INFINTE_RESISTANCE_MENU;
		else if(temp_controller.flash.menu == NTC_INFINTE_RESISTANCE_MENU)temp_controller.flash.menu = STARTUP_MENU;
		if(temp_controller.flash.menu != STARTUP_MENU)	update_pid();
	}
}

void calc_adc_values(){
	adc[0]							=		adc[0]/cnt_adc;
	adc[1]							=		adc[1]/cnt_adc;
	//ntc.current					=		((Vref/32767.0) *  adc[0])/R0;
	//ntc.voltage					=		(Vref/32767.0) *  adc[1];
	//ntc.resistance				=		ntc.voltage/ntc.current;
	ntc.resistance					=		adc[1]/(adc[0]/R0);
	ntc.temperature					=		lookup_temp(ntc.resistance) + (float)temp_controller.flash.offset_temp/10;
	cnt_adc							=		0;
	adc[0]							=		0;
	adc[1]							=		0;
	temp_controller.current_temp	=		0.95*temp_controller.current_temp + 0.05*ntc.temperature;
}

//gives back the temperature based on NTC resistance value, lookup table needed!
float lookup_temp(float R){
    int size, i = 0;
    float deltaT,deltaR,T;
    int (*lookup_temp_table)[2];
    switch(temp_controller.flash.sensor)
    {
    case NTCS0603E3222FMT:
    	lookup_temp_table = NTCS0603E3222FMT_RT;
    	size = sizeof(NTCS0603E3222FMT_RT);
    	break;
    case NTCG163JX103DTDS:
    	lookup_temp_table = NTCG163JX103DTDS_RT;
    	size = sizeof(NTCG163JX103DTDS_RT);
    	break;
    case NTC_100K:
    	lookup_temp_table = NTC_100K_RT;
    	size = sizeof(NTC_100K_RT);
    	break;
    case PT1000:
        	lookup_temp_table = PT1000_RT;
        	size = sizeof(PT1000_RT);
        	break;
    }
    int x,y;
    while(R<lookup_temp_table[i][1]){
    	i++;
        if((i+1)==size/sizeof(int)/2) {
                deltaT = lookup_temp_table[i][0]-lookup_temp_table[i-1][0];
                deltaR = lookup_temp_table[i][1]-lookup_temp_table[i-1][1];
                T=lookup_temp_table[i][0]+(R-lookup_temp_table[i][1])*deltaT/deltaR;
                return T;
        }
    }
    deltaT = lookup_temp_table[i+1][0]-lookup_temp_table[i][0];
    deltaR = lookup_temp_table[i+1][1]-lookup_temp_table[i][1];
    T=lookup_temp_table[i][0]+(R-lookup_temp_table[i][1])*deltaT/deltaR;

    if(0){
    	static float R_debug[100];
    	static l=0;
    	R_debug[l] = R;
    	l++;
    	if(l>=99) l=0;

    }

    return T;
}


//Interrupt function called on button press
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin){
	encoder (GPIO_Pin);
}


