
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

/* 便利な定数群 */
#define G					(9.80665f)					// 重量加速度[m/s^2]
#define PI2					(3.1415926f)				// 円周率
#define SQRT2				(1.41421356237f)			// ルート2
#define SQRT3				(1.73205080757f)			// ルート3
#define SQRT5				(2.2360679775f)				// ルート5
#define SQRT7				(2.64575131106f)			// ルート7

/* 便利なマクロ関数群 */
#define DEG2RAD(x)			(((x)/180.0f)*PI2)			// 度数法からラジアンに変換
#define RAD2DEG(x)			(180.0f*((x)/PI2))			// ラジアンから度数法に変換
#define SWAP(a, b) 			((a != b) && (a += b, b = a - b, a -= b))
#define ABS(x) 				((x) < 0 ? -(x) : (x))		// 絶対値
#define SIGN(x)				((x) < 0 ? -1 : 1)			// 符号
#define MAX(a, b) 			((a) > (b) ? (a) : (b))		// 2つのうち大きい方を返します
#define MIN(a, b) 			((a) < (b) ? (a) : (b))		// 2つのうち小さい方を返します
#define MAX3(a, b, c) 		((a) > (MAX(b, c)) ? (a) : (MAX(b, c)))
#define MIN3(a, b, c) 		((a) < (MIN(b, c)) ? (a) : (MIN(b, c)))

/* LED関数群 */
#define LED1_ON()		HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)		// 黄LEDを点灯する
#define LED1_OFF()	HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)	// 黄LEDを消灯する
#define LED1_TOGGLE()	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin)					// この関数を呼ぶたびに黄LEDの点灯と消灯を切り替える
#define LED2_ON()		HAL_GPIO_WritePin( LED2_GPIO_Port, 	 LED2_Pin, 	 GPIO_PIN_SET)		// 赤LEDを点灯する
#define LED2_OFF()		HAL_GPIO_WritePin( LED2_GPIO_Port, 	 LED2_Pin,    GPIO_PIN_RESET)	// 赤LEDを消灯する
#define LED2_TOGGLE()	HAL_GPIO_TogglePin(LED2_GPIO_Port, 	 LED2_Pin)						// この関数を呼ぶたびに赤LEDの点灯と消灯を切り替える
#define LED3_ON()		HAL_GPIO_WritePin( LED3_GPIO_Port,  LED3_Pin,  GPIO_PIN_SET)		// 緑LEDを点灯する
#define LED3_OFF()		HAL_GPIO_WritePin( LED3_GPIO_Port,  LED3_Pin,  GPIO_PIN_RESET)	// 緑LEDを消灯する
#define LED3_TOGGLE()	HAL_GPIO_TogglePin(LED3_GPIO_Port,  LED3_Pin)						// この関数を呼ぶたびに緑LEDの点灯と消灯を切り替える
#define LED4_ON()		HAL_GPIO_WritePin( LED4_GPIO_Port, 	 LED4_Pin,   GPIO_PIN_SET)		// 青LEDを点灯する
#define LED4_OFF()		HAL_GPIO_WritePin( LED4_GPIO_Port, 	 LED4_Pin,   GPIO_PIN_RESET)	// 青LEDを消灯する
#define LED4_TOGGLE()	HAL_GPIO_TogglePin(LED4_GPIO_Port, 	 LED4_Pin)						// この関数を呼ぶたびに青LEDの点灯と消灯を切り替える
#define LED5_ON()		HAL_GPIO_WritePin( LED5_GPIO_Port,  LED5_Pin,  GPIO_PIN_SET)		// 緑LEDを点灯する
#define LED5_OFF()		HAL_GPIO_WritePin( LED5_GPIO_Port,  LED5_Pin,  GPIO_PIN_RESET)	// 緑LEDを消灯する
#define LED5_TOGGLE()	HAL_GPIO_TogglePin(LED5_GPIO_Port,  LED5_Pin)						// この関数を呼ぶたびに緑LEDの点灯と消灯を切り替える
#define LED6_ON()		HAL_GPIO_WritePin( LED6_GPIO_Port, 	 LED6_Pin,   GPIO_PIN_SET)		// 青LEDを点灯する
#define LED6_OFF()		HAL_GPIO_WritePin( LED6_GPIO_Port, 	 LED6_Pin,   GPIO_PIN_RESET)	// 青LEDを消灯する
#define LED6_TOGGLE()	HAL_GPIO_TogglePin(LED6_GPIO_Port, 	 LED6_Pin)						// この関数を呼ぶたびに青LEDの点灯と消灯を切り替える
//#define LED_ALL_ON()		HAL_GPIO_WritePin(GPIOA, LED_Yellow_Pin|LED_Red_Pin|LED_Green_Pin|LED_Blue_Pin, GPIO_PIN_SET)	// 全LEDを点灯する
//#define LED_ALL_OFF()		HAL_GPIO_WritePin(GPIOA, LED_Yellow_Pin|LED_Red_Pin|LED_Green_Pin|LED_Blue_Pin, GPIO_PIN_RESET)	// 全LEDを消灯する
//#define LED_ALL_TOGGLE()	HAL_GPIO_TogglePin(GPIOA, LED_Yellow_Pin|LED_Red_Pin|LED_Green_Pin|LED_Blue_Pin)				// 全LEDの点灯と消灯を切り替える

/* スイッチ関数群 */
#define SWITCH_ONOFF()		HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin)				// スイッチが押されるとハイが返ってくる

/* モータ関数群(motor.c) */
void 		Motor_Initialize( void );					// モータ駆動用タイマーの開始
void 		Motor_StopPWM( void );						// モータを停止

/* 慣性センサ関数群(imu.c) */
uint8_t		IMU_CheckWHOAMI( void );					// 慣性センサの動作確認関数(0xe0が返ってくれば正常)
void		IMU_Initialize( void );						// 慣性センサの初期設定
void 		IMU_ResetReference( void );					// 慣性センサのリファレンスを補正する
float 		IMU_GetAccel_X( void );						// X軸加速度計の加速度を取得する[m/s^2]
float 		IMU_GetGyro_Z( void );						// Z軸ジャイロの角速度を取得する[rad/s]

/* 赤外センサ関数群(ir_sensor.c) */
void 		Sensor_Initialize( void );					// AD変換の初期設定

/**/
uint16_t ENC_R_GetAngle( void );
uint16_t ENC_L_GetAngle( void );

#define MOT_L_FORWARD()		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET)
#define MOT_L_BACKWARD()		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET)
#define MOT_R_FORWARD()		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET)
#define MOT_R_BACKWARD()		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET)

void get_gyro_ref(void);

#endif /* INDEX_H_ */

