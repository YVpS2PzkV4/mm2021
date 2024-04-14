
#include "index.h"
//#include "MPU6500.h"

// ローカル関数宣言
void 	ENC_R_Write1byte( uint8_t , uint8_t , uint8_t , uint8_t);
uint16_t ENC_R_Read1byte( uint16_t );
void 	ENC_L_Write1byte( uint8_t , uint8_t , uint8_t , uint8_t);
uint16_t ENC_L_Read1byte( uint16_t);


/* ---------------------------------------------------------------
	MPU-6500に1byte書き込む関数
--------------------------------------------------------------- */
void ENC_R_Write1byte( uint8_t addr1 , uint8_t addr2,uint8_t data1,uint8_t data2)
{
	uint8_t address[2] = {(addr1 & 0x7f),(addr2&0xff)};
	uint8_t datas[2] = {data1,data2};

	HAL_GPIO_WritePin(SPI1_CS_R_GPIO_Port, SPI1_CS_R_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, (uint8_t *)address, 1, 1);
	HAL_SPI_Transmit(&hspi3, (uint8_t *)datas, 1, 1);
	HAL_GPIO_WritePin(SPI1_CS_R_GPIO_Port, SPI1_CS_R_Pin, GPIO_PIN_SET);
}

/* ---------------------------------------------------------------
	MPU-6500から1byte読み出す関数
--------------------------------------------------------------- */
uint16_t ENC_R_Read1byte( uint16_t addr1)
{
	// 送信バッファに書き込み //
	uint16_t address = addr1;
	uint16_t value;

	HAL_GPIO_WritePin(SPI1_CS_L_GPIO_Port, SPI1_CS_L_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI1_CS_R_GPIO_Port, SPI1_CS_R_Pin, GPIO_PIN_RESET);
//	HAL_SPI_Transmit(&hspi1, (uint8_t *)address, 1, 1);
	//HAL_SPI_Receive(&hspi1, (uint8_t *)value, 1, 1);
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)&address, (uint8_t *)&value, 1, 100);
	HAL_GPIO_WritePin(SPI1_CS_L_GPIO_Port, SPI1_CS_L_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI1_CS_R_GPIO_Port, SPI1_CS_R_Pin, GPIO_PIN_SET);

	return value;
}


/* ---------------------------------------------------------------
	MPU-6500に1byte書き込む関数
--------------------------------------------------------------- */
void ENC_L_Write1byte( uint8_t addr1 , uint8_t addr2,uint8_t data1,uint8_t data2)
{
	uint8_t address[2] = {(addr1 & 0x7f),(addr2&0xff)};
	uint8_t datas[2] = {data1,data2};

	HAL_GPIO_WritePin(SPI1_CS_L_GPIO_Port, SPI1_CS_L_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, (uint8_t *)address, 1, 1);
	HAL_SPI_Transmit(&hspi3, (uint8_t *)datas, 1, 1);
	HAL_GPIO_WritePin(SPI1_CS_L_GPIO_Port, SPI1_CS_L_Pin, GPIO_PIN_SET);
}

/* ---------------------------------------------------------------
	MPU-6500から1byte読み出す関数
--------------------------------------------------------------- */
uint16_t ENC_L_Read1byte( uint16_t addr1)
{
	// 送信バッファに書き込み //
	uint16_t address = addr1;
	uint16_t value;

	HAL_GPIO_WritePin(SPI1_CS_R_GPIO_Port, SPI1_CS_R_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI1_CS_L_GPIO_Port, SPI1_CS_L_Pin, GPIO_PIN_RESET);
//	HAL_SPI_Transmit(&hspi1, (uint8_t *)address, 1, 1);
	//HAL_SPI_Receive(&hspi1, (uint8_t *)value, 1, 1);
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)&address, (uint8_t *)&value, 1, 100);
	HAL_GPIO_WritePin(SPI1_CS_R_GPIO_Port, SPI1_CS_R_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI1_CS_L_GPIO_Port, SPI1_CS_L_Pin, GPIO_PIN_SET);

	return value;
}


/* ---------------------------------------------------------------
	Z軸ジャイロの角速度を取得する関数[rad/s]
--------------------------------------------------------------- */
uint16_t ENC_R_GetAngle( void )
{
	uint16_t ang = ((ENC_R_Read1byte(0x0000)&0xFFF0)>>4);
	return ang;
}

uint16_t ENC_L_GetAngle( void )
{
	uint16_t ang = ((ENC_L_Read1byte(0x0000)&0xFFF0)>>4);
	return ang;
}
