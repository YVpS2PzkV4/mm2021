
#include "index.h"
#include "MPU6500.h"
#include "CMT.h"

#define REFFERENCE_NUM		(256)		// ����̕��ς������ăW���C���̃��t�@�����X�d���Ƃ��邩

// �W���C���֘A�}�N��
#define GYRO_Z_SIGN			(1.0f)		// �W���C���̏o�͂̕����i�����̍��W�n�ɍ����������ɁA1.0f���|1.0f���|���ďC������j
#define GYRO_Z_SENSITIVITY	((8.2f)*(1.04f))

// �����x�v�֘A�}�N��
#define ACCEL_X_SIGN		(1.0f)		// �����x�v�̏o�͂̕����i�����̍��W�n�ɍ����������ɁA1.0f���|1.0f���|���ďC������j
#define ACCEL_X_SENSITIVITY	(4096.0f)

// ���[�J���֐��錾
void 	IMU_Write1byte( uint8_t , uint8_t );
uint8_t IMU_Read1byte( uint8_t );

// �O���[�o���ϐ��錾
//static uint8_t  imu_address = ACCEL_XOUT_H | 0x80;
//static uint8_t	imu_value[15];			// value[0]�̓_�~�[�f�[�^

static int16_t	accel_x_value;			// X�������x�v�̐��f�[�^
static int16_t	accel_x_reference;		// X�������x�v�̃��t�@�����X

static int16_t	gyro_z_value;			// Z���W���C���̐��f�[�^
static int16_t	gyro_z_reference;		// Z���W���C���̃��t�@�����X

extern volatile float			gyro_ref;

extern volatile double Angle_G;		//��Ίp�x�i�W���C���̃��t�@�����X�Ƃ�Ƃ��Ƀ��Z�b�g�j
extern volatile double Tar_Angle_G;		//�ڕW��Ίp�x�i�W���C���̃��t�@�����X�Ƃ�Ƃ��Ƀ��Z�b�g�j

extern volatile float accelX_gyro;	//�W���C���Ōv�����������x
extern volatile float speedX_gyro;

extern volatile float speedE;	//�����x�Z���T�g�������x
extern volatile float speedE_prev;

extern volatile float			max_degree_G;
extern volatile float			degree;

extern volatile float			I_tar_speed;				//�ڕW���x��I����
extern volatile float			I_tar_ang_vel;				//�ڕW�p���x��I����
extern volatile float I_len;
extern volatile float			I_degree;
extern volatile float			I_ang_vel;
extern volatile float			I_speed;
extern volatile float			I_speed2;

/* ---------------------------------------------------------------
	MPU-6500��1byte�������ފ֐�
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
	MPU-6500����1byte�ǂݏo���֐�
--------------------------------------------------------------- */
uint8_t IMU_Read1byte( uint8_t addr )
{
	// ���M�o�b�t�@�ɏ������� //
	uint8_t address = addr | 0x80;
	uint8_t value;

	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &address, 1, 100);
	HAL_SPI_Receive(&hspi2, &value, 1, 100);
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

	return value;
}

/* ---------------------------------------------------------------
	MPU-6500�̓���m�F�֐��iWHO_AM_I(0x70)���擾����j
--------------------------------------------------------------- */
uint8_t IMU_CheckWHOAMI( void )
{
	return IMU_Read1byte( WHO_AM_I );
}

/* ---------------------------------------------------------------
	MPU-6500�̏����ݒ�p�֐�
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

//	IMU_Write1byte(USER_CTRL, 0x10);	// I2C���[�h��Disable�ɐݒ�
	//HAL_Delay(1);
	IMU_Write1byte(PWR_MGMT_1, 0x00);	// ICM20648�����Z�b�g
	HAL_Delay(10);
	//
	IMU_Write1byte(CONFIG, 0x00);
	HAL_Delay(10);
	// �W���C���̐ݒ�
	IMU_Write1byte(GYRO_CONFIG_1, 0x18);	// �W���C���̃X�P�[�����}2000deg/s�ɐݒ�
	HAL_Delay(10);							// �W���C���̃��[�p�X�t�B���^��Eable�ɐݒ�
	// �����x�v�̐ݒ�
	IMU_Write1byte(ACCEL_CONFIG, 0x10);		// �����x�v�̃X�P�[�����}8g�ɐݒ�

	HAL_Delay(10);*/


	IMU_Write1byte(REG_BANK_SEL, REG_USER_BANK_0);	// �o���N�̐؂�ւ�
	HAL_Delay(10);

	IMU_Write1byte(USER_CTRL, 0x10);	// I2C���[�h��Disable�ɐݒ�
	HAL_Delay(1);
	IMU_Write1byte(PWR_MGMT_1, 0x01);	// ICM20648�����Z�b�g
	HAL_Delay(10);

	IMU_Write1byte(REG_BANK_SEL, REG_USER_BANK_2); // �o���N�̐؂�ւ�
	HAL_Delay(10);

	// �W���C���̐ݒ�
	IMU_Write1byte(GYRO_CONFIG_1, 0x07);	// �W���C���̃X�P�[�����}4000deg/s�ɐݒ�
	HAL_Delay(1);							// �W���C���̃��[�p�X�t�B���^��Enable�ɐݒ�
	// �����x�v�̐ݒ�
	IMU_Write1byte(ACCEL_CONFIG, 0x0b);		// �����x�v�̃X�P�[�����}8g�ɐݒ�
	HAL_Delay(1);							// �����x�v�̃��[�p�X�t�B���^��Enable�ɐݒ�

	IMU_Write1byte(REG_BANK_SEL, REG_USER_BANK_0);	// �o���N�̐؂�ւ�
	HAL_Delay(10);

	// DMA�̊J�n
//	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
//	HAL_SPI_TransmitReceive_DMA( &hspi2, &imu_address, imu_value, sizeof(imu_value)/sizeof(uint8_t) );
}

/* ---------------------------------------------------------------
	DMA����M������̃R�[���o�b�N�֐�
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
	IMU�̃��t�@�����X��␳����֐�
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
	X�������x�v�̉����x���擾����֐�[m/s^2]
--------------------------------------------------------------- */
float IMU_GetAccel_X( void )
{
	HAL_GPIO_WritePin( SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET );
	accel_x_value =  ( ( (int16_t)IMU_Read1byte(ACCEL_YOUT_H)<<8 ) | ( (int16_t)IMU_Read1byte(ACCEL_YOUT_L)&0x00ff ) );
	HAL_GPIO_WritePin( SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET );
	return ACCEL_X_SIGN * G * (accel_x_value - accel_x_reference) / ACCEL_X_SENSITIVITY;
}

/* ---------------------------------------------------------------
	Z���W���C���̊p���x���擾����֐�[rad/s]
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
