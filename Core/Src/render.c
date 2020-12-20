/*
 * render.c
 *
 *  Created on: 2020. dec. 20.
 *      Author: Norbert
 */

#include "render.h"

void Redraw_display(){
	int t1,t2,delta_t;

	switch(temp_controller.menu)
	{
	case 1:
		if(INA226_1.Result.Current_uA 	>= 	INA226_2.Result.Current_uA) {
			temp_controller.current	= 	INA226_1.Result.Current_uA/1E6;
			temp_controller.power 	= 	INA226_1.Result.Power_uW /1E6;
		}
		else {
			temp_controller.current =	INA226_2.Result.Current_uA/1E6;
			temp_controller.power 	= 	INA226_2.Result.Power_uW /1E6;
		}

		char current_temp_str[10], target_temp_str[10], voltage_str[10], current_str[10], power_str[10];
		short current_temp_str_nr, target_temp_str_nr, voltage_str_nr, current_str_nr, power_str_nr;
		short pwm_pixels = 40;
		short pwm = fabs(temp_controller.pid.out);

		current_temp_str_nr = ftoa(temp_controller.current_temp, current_temp_str, 2);
		target_temp_str_nr = ftoa(temp_controller.target_temp/10.0, target_temp_str, 1);
		voltage_str_nr = ftoa(temp_controller.voltage, voltage_str, 1);
		current_str_nr = ftoa(temp_controller.current, current_str, 1);
		power_str_nr = ftoa(temp_controller.power, power_str, 1);

		u8g2_ClearBuffer(&u8g2);
		u8g2_SetDrawColor(&u8g2, 1);
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
	case 5:
		u8g2_ClearBuffer(&u8g2);
		char maxP_str[10];
		ftoa(temp_controller.pid.max_P, maxP_str, 0);
		u8g2_SetFont(&u8g2, u8g2_font_unifont_tf);
		u8g2_DrawUTF8(&u8g2, 10, 14, "max P: ");
		u8g2_DrawUTF8(&u8g2, 64, 14, maxP_str);
		u8g2_DrawUTF8(&u8g2, 0, 14, "*");
		u8g2_DrawUTF8(&u8g2, 88, 14, "%");
		u8g2_SendBuffer(&u8g2);
		break;
	case SET_DEFAULTS_MENU:
		u8g2_ClearBuffer(&u8g2);
		u8g2_SetFont(&u8g2, u8g2_font_unifont_tf);
		u8g2_DrawUTF8(&u8g2, 0, 14, "For set defaults");
		u8g2_DrawUTF8(&u8g2, 0, 28, "long press the");
		u8g2_DrawUTF8(&u8g2, 0, 42, "button!");
		if(temp_controller.defaults) u8g2_DrawUTF8(&u8g2, 80, 42, "OK");
		u8g2_SendBuffer(&u8g2);
		break;
	}
}

void set_defaults(){
	temp_controller.defaults = 1;
	temp_controller.target_temp = 0;
	temp_controller.pid.errorSum = 0;
	temp_controller.pid.Kp=300;
	temp_controller.pid.Kd=10000;
	temp_controller.pid.Ki=5;
	temp_controller.pid.max_P = 80;
}

void menu234(){
	//Case Menu = 2,3,4
	char Kp_str[10], Kd_str[10], Ki_str[10], Ki_t[10];
	ftoa(temp_controller.pid.Kp/10.0, Kp_str, 1);
	ftoa(temp_controller.pid.Kd/10.0, Kd_str, 1);
	ftoa(temp_controller.pid.Ki/10.0, Ki_str, 1);
	ftoa(temp_controller.pid.delta_t*1000, Ki_t, 1);
	u8g2_ClearBuffer(&u8g2);
	u8g2_SetFont(&u8g2, u8g2_font_unifont_tf);
	u8g2_DrawUTF8(&u8g2, 10, 14, "Kp: ");
	u8g2_DrawUTF8(&u8g2, 42, 14, Kp_str);
	u8g2_DrawUTF8(&u8g2, 10, 28, "Kd: ");
	u8g2_DrawUTF8(&u8g2, 42, 28, Kd_str);
	u8g2_DrawUTF8(&u8g2, 10, 42, "Ki: ");
	u8g2_DrawUTF8(&u8g2, 42, 42, Ki_str);
	u8g2_DrawUTF8(&u8g2, 10, 56, "∆t: ");
	u8g2_DrawUTF8(&u8g2, 42, 56, Ki_t);
	u8g2_DrawUTF8(&u8g2, 90, 56, "ms");
}
