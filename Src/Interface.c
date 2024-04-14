#include "index.h"
#include"Interface.h"
#include"common.h"
#include"CMT.h"
#include"run.h"

#include<math.h>
//#include<mathf.h>

extern volatile int mode;

extern volatile int turnnuml;
extern volatile int turnnumr;

extern volatile float Sensor_L;
extern volatile float Sensor_R;
extern volatile float Sensor_FR;

extern volatile float R_Dis_Ave;
extern volatile float L_Dis_Ave;

extern volatile char BatCheckSW;
extern volatile char g_Sensor_Flag;

void reset_notice(void){
	LED1_ON();
	HAL_Delay(100);
	LED2_ON();
	HAL_Delay(100);
	LED3_ON();
	HAL_Delay(100);
	LED4_ON();
	HAL_Delay(100);
	LED5_ON();
	HAL_Delay(100);
	LED6_ON();
	HAL_Delay(100);
	LED1_OFF();
	HAL_Delay(100);
	LED2_OFF();
	HAL_Delay(100);
	LED3_OFF();
	HAL_Delay(100);
	LED4_OFF();
	HAL_Delay(100);
	LED5_OFF();
	HAL_Delay(100);
	LED6_OFF();
}

void searchend1(void){
	char t = 0;
	for(t = 0; t < 3; t++){
	led_all(1);
	HAL_Delay(100);
	led_all(0);
	HAL_Delay(100);
	led_all(1);
	HAL_Delay(100);
	led_all(0);
	HAL_Delay(300);
	}
}

void searchend2(void){
	led_all(1);
	HAL_Delay(100);
	led_all(0);
	HAL_Delay(100);
	led_all(1);
	HAL_Delay(100);
	led_all(0);
	HAL_Delay(100);
	led_all(1);
	HAL_Delay(100);
	led_all(0);
	HAL_Delay(300);
}

void led_all(int onoff){
	if(onoff == 1){
		LED1_ON();
		LED2_ON();
		LED3_ON();
		LED4_ON();
		LED5_ON();
		LED6_ON();
	}
	else if(onoff == 0){
		LED1_OFF();
		LED2_OFF();
		LED3_OFF();
		LED4_OFF();
		LED5_OFF();
		LED6_OFF();

	}
}

void upsound(void){
	int i;
	for(i = 0; i < 100000; i++){
		MOT_R_BACKWARD();
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 80);
	}
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
}

void downsound(void){
	int i;
	for(i = 0; i < 100000; i++){
		MOT_R_FORWARD();
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 80);
	}
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
}

void endsound(void){
	int i;
	//LED5_ON();
	for(i = 0; i < 100000; i++){
		MOT_L_BACKWARD();
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 80);
	}
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
	//LED5_OFF();
//	LED5_ON();
//	HAL_Delay(100);
//	LED5_OFF();
}

void sensesound(void){
	int i,j;
	led_all(1);
	for(j = 0; j < 50; j++){
		for(i = 0; i < 6700; i++){
	//		BZ = BZ_ON;
		}
		for(i = 0; i < 6700; i++){
	//		BZ = BZ_OFF;
		}
	}
}

void waithand(void){
	g_Sensor_Flag = 1;
	HAL_Delay(50);
	while((Sensor_R < MYHAND) || (Sensor_FR < MYHAND)){
		if(fabsf(R_Dis_Ave-Sensor_R_Ref_Stop) <= 1.0){
			LED2_ON();
		}
		else{
			LED2_OFF();
		}
		if(fabsf(L_Dis_Ave-Sensor_L_Ref_Stop) <= 1.0){
			LED5_ON();
		}
		else{
			LED5_OFF();
		}
		LED1_ON();
	//	LED_A2 = 0;
		HAL_Delay(75);
		LED1_OFF();
	//	LED_A2 = 1;
		HAL_Delay(75);
	}
	led_all(0);
	HAL_Delay(25);
	while((Sensor_R < MYHAND) || (Sensor_FR < MYHAND));
	HAL_Delay(25);
	g_Sensor_Flag = 0;
	sensesound();
	HAL_Delay(500);
	led_all(0);
	HAL_Delay(150);
	led_all(1);
	HAL_Delay(100);
	led_all(0);
	HAL_Delay(50);
}

void waitsw(void){
	while(SWITCH_ONOFF() == SW_OFF){
		LED1_ON();
		//LED2_ON();
		HAL_Delay(100);
		LED1_OFF();
		//LED2_OFF();
		HAL_Delay(100);
	}
	led_all(0);
	HAL_Delay(50);
	while(SWITCH_ONOFF() == SW_OFF);
	HAL_Delay(100);
}

void mode_change(int plus){
	//モードチェンジ（LEDに二進数で表示）
  	mode = mode + plus;
	if(mode >= 32)	mode = 0;
	if(mode <= -1)  mode = 31;		//モードの最大値15
	//
	if(mode%2 == 0)		LED1_OFF();
	else	LED1_ON();
	if(mode/2%2 == 0)	LED2_OFF();
	else	LED2_ON();
	if(mode/4%2 == 0)	LED3_OFF();
	else	LED3_ON();
	if(mode/8%2 == 0)	LED4_OFF();
	else	LED4_ON();
	if(mode/16%2 == 0)	LED5_OFF();
	else	LED5_ON();
	//
	if(plus < 0){
		downsound();
	}
	else if(plus == 1){
		upsound();
	}
	else{
	}
}

void enmode(void){
	turnnumr = 0;
	turnnuml = 0;
	led_all(0);
	while(1){
		mode_change(0);
		if(turnnuml > 1000){
			endsound();
			led_all(0);
			turnnuml = 0;
			break;
		}
		if(turnnumr > 250){
			mode_change(1);
			HAL_Delay(50);
			turnnumr = 0;
		}
		else if(turnnumr < -250){
			mode_change(-1);
			HAL_Delay(50);
			turnnumr = 0;
		}
		else if(SWITCH_ONOFF() == SW_ON){
			HAL_Delay(10);
			if(SWITCH_ONOFF() == SW_ON){
				mode_change(5);
				while(SWITCH_ONOFF() == SW_ON);
				HAL_Delay(100);
			}
		}
		else{
		}
	}
}
