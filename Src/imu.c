
#include "index.h"
#include "MPU6500.h"
#include "CMT.h"

#define REFFERENCE_NUM		(256)		// 何回の平均をもってジャイロのリファレンス電圧とするか

// ジャイロ関連マクロ
#define GYRO_Z_SIGN			(1.0f)		// ジャイロの出力の符号（自分の座標系に合った方向に、1.0fか－1.0fを掛けて修正する）
#define GYRO_Z_SENSITIVITY	((8.2f)*(1.04f))

// 加速度計関連マクロ
#define ACCEL_X_SIGN		(1.0f)		// 加速度計の出力の符号（自分の座標系に合った方向に、1.0fか－1.0fを掛けて修正する）
#define ACCEL_X_SENSITIVITY	(4096.0f)

// ローカル関数宣言
void 	IMU_Write1byte( uint8_t , uint8_t );
uint8_t IMU_Read1byte( uint8_t );

// グローバル変数宣言
//static uint8_t  imu_address = ACCEL_XOUT_H | 0x80;
//static uint8_t	imu_value[15];			// value[0]はダミーデータ

static int16_t	accel_x_value;			// X軸加速度計の生データ
static int16_t	accel_x_reference;		// X軸加速度計のリファレンス

static int16_t	gyro_z_value;			// Z軸ジャイロの生データ
static int16_t	gyro_z_reference;		// Z軸ジャイロのリファレンス

extern volatile float			gyro_ref;

extern volatile double Angle_G;		//絶対角度（ジャイロのリファレンスとるときにリセット）
extern volatile double Tar_Angle_G;		//目標絶対角度（ジャイロのリファレンスとるときにリセット）

extern volatile float accelX_gyro;	//ジャイロで計測した加速度
extern volatile float speedX_gyro;

extern volatile float speedE;	//加速度センサ使った速度
extern volatile float speedE_prev;

extern volatile float			max_degree_G;
extern volatile float			degree;

extern volatile float			I_tar_speed;				//目標速度のI成分
extern volatile float			I_tar_ang_vel;				//目標角速度のI成分
extern volatile float I_len;
extern volatile float			I_degree;
extern volatile float			I_ang_vel;
extern volatile float			I_speed;
extern volatile float			I_speed2;

/* ---------------------------------------------------------------
	MPU-6500に1byte書き込む関数
--------------------------------------------------------------- */
void IMU_Write1byte( uint8_t addr , uint8_t data )
{
	uint8_t address = addr & 0x7f;

	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &address, 1, 100);
	HAL_SPI_Transmit(&hspi2, &data, 1, 100);
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
}

/* ---------------------------------------------------------------
	MPU-6500から1byte読み出す関数
--------------------------------------------------------------- */
uint8_t IMU_Read1byte( uint8_t addr )
{
	// 送信バッファに書き込み //
	uint8_t address = addr | 0x80;
	uint8_t value;

	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &address, 1, 100);
	HAL_SPI_Receive(&hspi2, &value, 1, 100);
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

	return value;
}

/* ---------------------------------------------------------------
	MPU-6500の動作確認関数（WHO_AM_I(0x70)を取得する）
--------------------------------------------------------------- */
uint8_t IMU_CheckWHOAMI( void )
{
	return IMU_Read1byte( WHO_AM_I );
}

/* ---------------------------------------------------------------
	MPU-6500の初期設定用関数
--------------------------------------------------------------- */
void IMU_Initialize( void )
{
//	uint8_t who = IMU_CheckWHOAMI();
//	HAL_Delay(50);
/*	if(who != 0x70){
		while(1){
			printf("Gyro Error!\r\n");
		}
	}*/


/*	HAL_Delay(10);

//	IMU_Write1byte(USER_CTRL, 0x10);	// I2CモードをDisableに設定
	//HAL_Delay(1);
	IMU_Write1byte(PWR_MGMT_1, 0x00);	// ICM20648をリセット
	HAL_Delay(10);
	//
	IMU_Write1byte(CONFIG, 0x00);
	HAL_Delay(10);
	// ジャイロの設定
	IMU_Write1byte(GYRO_CONFIG_1, 0x18);	// ジャイロのスケールを±2000deg/sに設定
	HAL_Delay(10);							// ジャイロのローパスフィルタをEableに設定
	// 加速度計の設定
	IMU_Write1byte(ACCEL_CONFIG, 0x10);		// 加速度計のスケールを±8gに設定

	HAL_Delay(10);*/


	IMU_Write1byte(REG_BANK_SEL, REG_USER_BANK_0);	// バンクの切り替え
	HAL_Delay(10);

	IMU_Write1byte(USER_CTRL, 0x10);	// I2CモードをDisableに設定
	HAL_Delay(1);
	IMU_Write1byte(PWR_MGMT_1, 0x01);	// ICM20648をリセット
	HAL_Delay(10);

	IMU_Write1byte(REG_BANK_SEL, REG_USER_BANK_2); // バンクの切り替え
	HAL_Delay(10);

	// ジャイロの設定
	IMU_Write1byte(GYRO_CONFIG_1, 0x07);	// ジャイロのスケールを±4000deg/sに設定
	HAL_Delay(1);							// ジャイロのローパスフィルタをEnableに設定
	// 加速度計の設定
	IMU_Write1byte(ACCEL_CONFIG, 0x0b);		// 加速度計のスケールを±8gに設定
	HAL_Delay(1);							// 加速度計のローパスフィルタをEnableに設定

	IMU_Write1byte(REG_BANK_SEL, REG_USER_BANK_0);	// バンクの切り替え
	HAL_Delay(10);

	// DMAの開始
//	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
//	HAL_SPI_TransmitReceive_DMA( &hspi2, &imu_address, imu_value, sizeof(imu_value)/sizeof(uint8_t) );
}

/* ---------------------------------------------------------------
	DMA送受信完了後のコールバック関数
--------------------------------------------------------------- */
/*void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
	HAL_GPIO_WritePin( SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET );
	accel_x_value = ( ( (int16_t)imu_value[3]<<8 ) | ( (int16_t)imu_value[4]&0x00ff ) );
	gyro_z_value =  ( ( (int16_t)imu_value[13]<<8 ) | ( (int16_t)imu_value[14]&0x00ff ) );
	//printf("zvalue : %d",gyro_z_value);
	HAL_GPIO_WritePin( SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET );
	HAL_SPI_TransmitReceive_DMA( &hspi2, &imu_address, imu_value, sizeof(imu_value)/sizeof(uint8_t) );
}
*/
/* ---------------------------------------------------------------
	IMUのリファレンスを補正する関数
--------------------------------------------------------------- */
void IMU_ResetReference( void )
{
	int16_t i;

	for(i = 0; i < REFFERENCE_NUM; i++) {
		HAL_Delay(1);
		accel_x_reference += accel_x_value;
		gyro_z_reference += gyro_z_value;
	}
	accel_x_reference /= REFFERENCE_NUM;
	gyro_z_reference /= REFFERENCE_NUM;
}

void get_gyro_ref(void){
	IMU_ResetReference();
	gyro_ref = gyro_z_reference;
	accelX_gyro = 0;
	speedX_gyro = 0;
	speedE = 0;
	speedE_prev = 0;
	Angle_G = 0;
	Tar_Angle_G = 0;
	degree = 0;
	max_degree_G = 0;
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	I_speed2 = 0;
	I_len = 0;
}

/* ---------------------------------------------------------------
	X軸加速度計の加速度を取得する関数[m/s^2]
--------------------------------------------------------------- */
float IMU_GetAccel_X( void )
{
	HAL_GPIO_WritePin( SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET );
	accel_x_value =  ( ( (int16_t)IMU_Read1byte(ACCEL_YOUT_H)<<8 ) | ( (int16_t)IMU_Read1byte(ACCEL_YOUT_L)&0x00ff ) );
	HAL_GPIO_WritePin( SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET );
	return ACCEL_X_SIGN * G * (accel_x_value - accel_x_reference) / ACCEL_X_SENSITIVITY;
}

/* ---------------------------------------------------------------
	Z軸ジャイロの角速度を取得する関数[rad/s]
--------------------------------------------------------------- */
float IMU_GetGyro_Z( void )
{
	//return (GYRO_Z_SIGN * ( (float)(gyro_z_value - gyro_z_reference) / GYRO_Z_SENSITIVITY )*PI/180.0);
	HAL_GPIO_WritePin( SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET );
	//accel_x_value = ( ( (int16_t)imu_value[3]<<8 ) | ( (int16_t)imu_value[4]&0x00ff ) );
	gyro_z_value =  ( ( (int16_t)IMU_Read1byte(GYRO_ZOUT_H)<<8 ) | ( (int16_t)IMU_Read1byte(GYRO_ZOUT_L)&0x00ff ) );
	HAL_GPIO_WritePin( SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET );
	return (GYRO_Z_SIGN * ( (float)(gyro_z_value - gyro_z_reference) / GYRO_Z_SENSITIVITY )*PI/180.0);
}
