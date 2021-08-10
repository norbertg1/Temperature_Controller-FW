/*
 * render.c
 *
 *  Created on: 2020. dec. 20.
 *      Author: Norbert
 */

#include "render.h"

void menu15();
void menu234();

void Redraw_display(){
	char current_str[10], power_str[10];
	short current_str_nr, power_str_nr;
	switch(temp_controller.flash.menu)
	{
	case 0:
		menu1();
		break;
	case SET_Temp_MENU:
		menu1();
		break;

	case SET_Kp_MENU:
		menu_options1();
		u8g2_DrawUTF8(&u8g2, 0, 14, "*");
		u8g2_SendBuffer(&u8g2);
		break;
	case SET_Kd_MENU:
		menu_options1();
		u8g2_DrawUTF8(&u8g2, 0, 28, "*");
		u8g2_SendBuffer(&u8g2);
		break;
	case SET_Ki_MENU:
		menu_options1();
		u8g2_DrawUTF8(&u8g2, 0, 42, "*");
		u8g2_SendBuffer(&u8g2);
		break;
	case SET_MAX_P_MENU:
		menu_options2();
		u8g2_DrawUTF8(&u8g2, 0, 14, "*");
		u8g2_SendBuffer(&u8g2);
		break;
	case SET_MODE_MENU:
		menu_options2();
		u8g2_DrawUTF8(&u8g2, 0, 28, "*");
		u8g2_SendBuffer(&u8g2);
		break;
	case CHOOSE_NTC_MENU:
		menu_options2();
		u8g2_DrawUTF8(&u8g2, 0, 42, "*");
		u8g2_SendBuffer(&u8g2);
		break;
	case CHOOSE_FREQUENCY_MENU:
		menu_options2();
		u8g2_DrawUTF8(&u8g2, 0, 56, "*");
		u8g2_SendBuffer(&u8g2);
		break;

	case SET_P_MENU:
		u8g2_ClearBuffer(&u8g2);
		menu15();
		char target_P_str[10];
		short target_P_str_nr;
		target_P_str_nr = ftoa(temp_controller.flash.set_power, target_P_str, 0);
		current_str_nr = ftoa(temp_controller.current, current_str, 2);
		power_str_nr = ftoa(temp_controller.power, power_str, 2);
		u8g2_SetFont(&u8g2, u8g2_font_logisoso16_tf);
		u8g2_DrawUTF8(&u8g2, 0, 58, "P:");
		u8g2_SetFont(&u8g2, u8g2_font_logisoso16_tf);	//The display has limited space
		u8g2_DrawUTF8(&u8g2, 20, 58, target_P_str);
		u8g2_DrawUTF8(&u8g2, 16+target_P_str_nr*12, 58, "%");
		u8g2_SetFont(&u8g2, u8g2_font_helvR08_te     );
		u8g2_DrawUTF8(&u8g2, 70, 46, "I:");
		u8g2_DrawUTF8(&u8g2, 80, 46, current_str);
		u8g2_DrawUTF8(&u8g2, 80+current_str_nr*8, 46, "A");
		u8g2_DrawUTF8(&u8g2, 66, 58, "P:");
		u8g2_DrawUTF8(&u8g2, 80, 58, power_str);
		u8g2_DrawUTF8(&u8g2, 80+power_str_nr*8, 58, "W");
		u8g2_SendBuffer(&u8g2);
		break;
	case SET_DEFAULTS_MENU:
		u8g2_ClearBuffer(&u8g2);
		u8g2_SetFont(&u8g2, u8g2_font_unifont_tf);
		u8g2_DrawUTF8(&u8g2, 0, 14, "For set defaults");
		u8g2_DrawUTF8(&u8g2, 0, 28, "long press the");
		u8g2_DrawUTF8(&u8g2, 0, 42, "button!");
		if(temp_controller.flash.defaults) u8g2_DrawUTF8(&u8g2, 80, 42, "OK");
		u8g2_SendBuffer(&u8g2);
		break;
	case TOO_HOT_MENU:
		u8g2_ClearBuffer(&u8g2);
		u8g2_SetFont(&u8g2, u8g2_font_fur14_tf    );
		u8g2_DrawUTF8(&u8g2, 8, 24, "WARNING!!!");
		u8g2_SetFont(&u8g2, u8g2_font_luRS08_tr       );
		u8g2_DrawUTF8(&u8g2, 16, 48, "Peltier is too hot!");
		u8g2_SendBuffer(&u8g2);
		break;
	case NTC_INFINTE_RESISTANCE_MENU:
		u8g2_ClearBuffer(&u8g2);
		u8g2_SetFont(&u8g2, u8g2_font_fur14_tf    );
		u8g2_DrawUTF8(&u8g2, 8, 24, "WARNING!!!");
		u8g2_SetFont(&u8g2, u8g2_font_luRS08_tr       );
		u8g2_DrawUTF8(&u8g2, 30, 40, "Temp. sensor");
		u8g2_DrawUTF8(&u8g2, 24, 50, "not connected!");
		u8g2_SendBuffer(&u8g2);
		break;
	}
}

void set_defaults(){
	temp_controller.flash.defaults = 1;
	temp_controller.flash.target_temp = 0;
	temp_controller.flash.pid.errorSum = 0;
	temp_controller.flash.pid.Kp= 75;
	temp_controller.flash.pid.Kd= 4100;
	temp_controller.flash.pid.Ki= 3;
	temp_controller.flash.pid.max_P = 60;
	temp_controller.flash.mode = -1;
	temp_controller.flash.sensor = 1;
	temp_controller.flash.freq = 50;
}

void menu1(){
	char current_str[10], power_str[10];
	short current_str_nr, power_str_nr;
	u8g2_ClearBuffer(&u8g2);
	menu15();
	char target_temp_str[10];
	short target_temp_str_nr;
	short pwm_pixels = 40;
	short pwm = fabs(temp_controller.flash.pid.out);
	current_str_nr = ftoa(temp_controller.current, current_str, 1);
	power_str_nr = ftoa(temp_controller.power, power_str, 1);
	u8g2_SetFont(&u8g2, u8g2_font_helvR08_te);
	u8g2_DrawUTF8(&u8g2, 0, 46, "Set.");
	u8g2_DrawUTF8(&u8g2, 0, 58, "temp.:");
	u8g2_SetFont(&u8g2, u8g2_font_logisoso16_tf);	//The display has limited space
	if(temp_controller.flash.target_temp<0) u8g2_DrawUTF8(&u8g2, 22, 52, "-");
	if(abs(temp_controller.flash.target_temp) < 1000) target_temp_str_nr = ftoa(temp_controller.flash.target_temp/10.0, target_temp_str, 1);
	else target_temp_str_nr = ftoa(temp_controller.flash.target_temp/10.0, target_temp_str, 0);
	u8g2_DrawUTF8(&u8g2, 30, 58, target_temp_str);
	if(abs(temp_controller.flash.target_temp) < 1000) u8g2_DrawUTF8(&u8g2, 26+target_temp_str_nr*14, 58, "°C");
	else u8g2_DrawUTF8(&u8g2, 20+target_temp_str_nr*14, 58, "°C");
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
}

void menu15(){
	char current_temp_str[10];
	short current_temp_str_nr;

	if(INA226_1.Result.Current_uA 	>= 	INA226_2.Result.Current_uA) {
		temp_controller.current	= 	INA226_1.Result.Current_uA/1E6;
		temp_controller.power 	= 	INA226_1.Result.Power_uW /1E6;
	}
	else {
		temp_controller.current =	INA226_2.Result.Current_uA/1E6;
		temp_controller.power 	= 	INA226_2.Result.Power_uW /1E6;
	}

	current_temp_str_nr = ftoa(temp_controller.current_temp, current_temp_str, 2);
	u8g2_SetDrawColor(&u8g2, 1);
	u8g2_SetFont(&u8g2, u8g2_font_unifont_tf);
	u8g2_DrawUTF8(&u8g2, -1, 14, "Curr.");
	u8g2_DrawUTF8(&u8g2, 0, 26, "temp.:");
	u8g2_SetFont(&u8g2, u8g2_font_logisoso20_tf);
	if(temp_controller.current_temp<0) u8g2_DrawUTF8(&u8g2, 36, 20, "-"); //"-" sign
	u8g2_DrawUTF8(&u8g2, 48, 26, current_temp_str);
	u8g2_DrawUTF8(&u8g2, 48+current_temp_str_nr*15, 26, "°C");


}

void menu_options1(){
	//Case Menu = 2,3,4
	char Kp_str[10], Kd_str[10], Ki_str[10], Ki_t[10];
	ftoa(temp_controller.flash.pid.Kp/10.0, Kp_str, 1);
	ftoa(temp_controller.flash.pid.Kd/10.0, Kd_str, 1);
	ftoa(temp_controller.flash.pid.Ki/10.0, Ki_str, 1);
	ftoa(temp_controller.flash.pid.delta_t*1000, Ki_t, 1);
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

void menu_options2(){
	char maxP_str[10], freq_str[10];
	u8g2_ClearBuffer(&u8g2);
	ftoa(temp_controller.flash.pid.max_P, maxP_str, 0);
	ftoa(temp_controller.flash.freq, freq_str, 0);
	u8g2_SetFont(&u8g2, u8g2_font_unifont_tf);
	u8g2_DrawUTF8(&u8g2, 10, 14, "max P: ");
	u8g2_DrawUTF8(&u8g2, 64, 14, maxP_str);
	u8g2_DrawUTF8(&u8g2, 88, 14, "%");
	u8g2_DrawUTF8(&u8g2, 10, 28, "mode: ");
	if (temp_controller.flash.mode == -1) u8g2_DrawUTF8(&u8g2, 64, 28, "Coolig");
	if (temp_controller.flash.mode == 1) u8g2_DrawUTF8(&u8g2, 64, 28, "Heating");
	u8g2_DrawUTF8(&u8g2, 10, 42, "Rt: ");
	u8g2_SetFont(&u8g2, u8g2_font_helvR08_te);
	if (temp_controller.flash.sensor == NTCS0603E3222FMT)	u8g2_DrawUTF8(&u8g2, 42, 42, "NTCS0603E3222");
	if (temp_controller.flash.sensor == NTCG163JX103DTDS)	u8g2_DrawUTF8(&u8g2, 42, 42, "NTCG163JX103");
	if (temp_controller.flash.sensor == NTC_100K)			u8g2_DrawUTF8(&u8g2, 42, 42, "NTC_100K");
	if (temp_controller.flash.sensor == PT1000)				u8g2_DrawUTF8(&u8g2, 42, 42, "PT1000");
	u8g2_SetFont(&u8g2, u8g2_font_unifont_tf);
	u8g2_DrawUTF8(&u8g2, 10, 56, "Freq.: ");
	u8g2_DrawUTF8(&u8g2, 64, 56, freq_str);
	u8g2_DrawUTF8(&u8g2, 92, 56, "kHz");

}
