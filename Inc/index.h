
#ifndef INDEX_H_
#define INDEX_H_

#include <stdio.h>
#include <math.h>
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* �֗��Ȓ萔�Q */
#define G					(9.80665f)					// �d�ʉ����x[m/s^2]
#define PI2					(3.1415926f)				// �~����
#define SQRT2				(1.41421356237f)			// ���[�g2
#define SQRT3				(1.73205080757f)			// ���[�g3
#define SQRT5				(2.2360679775f)				// ���[�g5
#define SQRT7				(2.64575131106f)			// ���[�g7

/* �֗��ȃ}�N���֐��Q */
#define DEG2RAD(x)			(((x)/180.0f)*PI2)			// �x���@���烉�W�A���ɕϊ�
#define RAD2DEG(x)			(180.0f*((x)/PI2))			// ���W�A������x���@�ɕϊ�
#define SWAP(a, b) 			((a != b) && (a += b, b = a - b, a -= b))
#define ABS(x) 				((x) < 0 ? -(x) : (x))		// ��Βl
#define SIGN(x)				((x) < 0 ? -1 : 1)			// ����
#define MAX(a, b) 			((a) > (b) ? (a) : (b))		// 2�̂����傫������Ԃ��܂�
#define MIN(a, b) 			((a) < (b) ? (a) : (b))		// 2�̂�������������Ԃ��܂�
#define MAX3(a, b, c) 		((a) > (MAX(b, c)) ? (a) : (MAX(b, c)))
#define MIN3(a, b, c) 		((a) < (MIN(b, c)) ? (a) : (MIN(b, c)))

/* LED�֐��Q */
#define LED1_ON()		HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)		// ��LED��_������
#define LED1_OFF()	HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)	// ��LED����������
#define LED1_TOGGLE()	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin)					// ���̊֐����ĂԂ��тɉ�LED�̓_���Ə�����؂�ւ���
#define LED2_ON()		HAL_GPIO_WritePin( LED2_GPIO_Port, 	 LED2_Pin, 	 GPIO_PIN_SET)		// ��LED��_������
#define LED2_OFF()		HAL_GPIO_WritePin( LED2_GPIO_Port, 	 LED2_Pin,    GPIO_PIN_RESET)	// ��LED����������
#define LED2_TOGGLE()	HAL_GPIO_TogglePin(LED2_GPIO_Port, 	 LED2_Pin)						// ���̊֐����ĂԂ��тɐ�LED�̓_���Ə�����؂�ւ���
#define LED3_ON()		HAL_GPIO_WritePin( LED3_GPIO_Port,  LED3_Pin,  GPIO_PIN_SET)		// ��LED��_������
#define LED3_OFF()		HAL_GPIO_WritePin( LED3_GPIO_Port,  LED3_Pin,  GPIO_PIN_RESET)	// ��LED����������
#define LED3_TOGGLE()	HAL_GPIO_TogglePin(LED3_GPIO_Port,  LED3_Pin)						// ���̊֐����ĂԂ��тɗ�LED�̓_���Ə�����؂�ւ���
#define LED4_ON()		HAL_GPIO_WritePin( LED4_GPIO_Port, 	 LED4_Pin,   GPIO_PIN_SET)		// ��LED��_������
#define LED4_OFF()		HAL_GPIO_WritePin( LED4_GPIO_Port, 	 LED4_Pin,   GPIO_PIN_RESET)	// ��LED����������
#define LED4_TOGGLE()	HAL_GPIO_TogglePin(LED4_GPIO_Port, 	 LED4_Pin)						// ���̊֐����ĂԂ��тɐ�LED�̓_���Ə�����؂�ւ���
#define LED5_ON()		HAL_GPIO_WritePin( LED5_GPIO_Port,  LED5_Pin,  GPIO_PIN_SET)		// ��LED��_������
#define LED5_OFF()		HAL_GPIO_WritePin( LED5_GPIO_Port,  LED5_Pin,  GPIO_PIN_RESET)	// ��LED����������
#define LED5_TOGGLE()	HAL_GPIO_TogglePin(LED5_GPIO_Port,  LED5_Pin)						// ���̊֐����ĂԂ��тɗ�LED�̓_���Ə�����؂�ւ���
#define LED6_ON()		HAL_GPIO_WritePin( LED6_GPIO_Port, 	 LED6_Pin,   GPIO_PIN_SET)		// ��LED��_������
#define LED6_OFF()		HAL_GPIO_WritePin( LED6_GPIO_Port, 	 LED6_Pin,   GPIO_PIN_RESET)	// ��LED����������
#define LED6_TOGGLE()	HAL_GPIO_TogglePin(LED6_GPIO_Port, 	 LED6_Pin)						// ���̊֐����ĂԂ��тɐ�LED�̓_���Ə�����؂�ւ���
//#define LED_ALL_ON()		HAL_GPIO_WritePin(GPIOA, LED_Yellow_Pin|LED_Red_Pin|LED_Green_Pin|LED_Blue_Pin, GPIO_PIN_SET)	// �SLED��_������
//#define LED_ALL_OFF()		HAL_GPIO_WritePin(GPIOA, LED_Yellow_Pin|LED_Red_Pin|LED_Green_Pin|LED_Blue_Pin, GPIO_PIN_RESET)	// �SLED����������
//#define LED_ALL_TOGGLE()	HAL_GPIO_TogglePin(GPIOA, LED_Yellow_Pin|LED_Red_Pin|LED_Green_Pin|LED_Blue_Pin)				// �SLED�̓_���Ə�����؂�ւ���

/* �X�C�b�`�֐��Q */
#define SWITCH_ONOFF()		HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin)				// �X�C�b�`���������ƃn�C���Ԃ��Ă���

/* ���[�^�֐��Q(motor.c) */
void 		Motor_Initialize( void );					// ���[�^�쓮�p�^�C�}�[�̊J�n
void 		Motor_StopPWM( void );						// ���[�^���~

/* �����Z���T�֐��Q(imu.c) */
uint8_t		IMU_CheckWHOAMI( void );					// �����Z���T�̓���m�F�֐�(0xe0���Ԃ��Ă���ΐ���)
void		IMU_Initialize( void );						// �����Z���T�̏����ݒ�
void 		IMU_ResetReference( void );					// �����Z���T�̃��t�@�����X��␳����
float 		IMU_GetAccel_X( void );						// X�������x�v�̉����x���擾����[m/s^2]
float 		IMU_GetGyro_Z( void );						// Z���W���C���̊p���x���擾����[rad/s]

/* �ԊO�Z���T�֐��Q(ir_sensor.c) */
void 		Sensor_Initialize( void );					// AD�ϊ��̏����ݒ�

/**/
uint16_t ENC_R_GetAngle( void );
uint16_t ENC_L_GetAngle( void );

#define MOT_L_FORWARD()		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET)
#define MOT_L_BACKWARD()		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET)
#define MOT_R_FORWARD()		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET)
#define MOT_R_BACKWARD()		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET)

void get_gyro_ref(void);

#endif /* INDEX_H_ */

