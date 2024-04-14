#include "index.h"

extern volatile float FanVolt;	//�z���t�@���̈���d��
extern volatile float			Duty_fan;
extern volatile char FanEnbl;	//�z���t�@���̈���d��

/* ---------------------------------------------------------------
	���[�^�p�̃^�C�}�[���J�n����֐�
--------------------------------------------------------------- */
void Motor_Initialize( void )
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0x0000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,0x0000);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,0x0000);
}

void Motor_StopPWM( void )
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0x0000);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,0x0000);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,0x0000);
	//FanVolt = 0.0;
	Duty_fan = 0.0;
	FanEnbl = 0;
}
