
#include "index.h"

uint16_t			adc_value[5] = {0,0,0,0,0};		// AD変換値
uint16_t			battery_value = 0;		// バッテリ電圧の生データ
uint16_t	sensor_value_on[4] = {0,0,0,0};	// IRセンサの生データ
uint16_t	sensor_value_off[4] = {0,0,0,0};	// IRセンサの生データ

/* ---------------------------------------------------------------
	AD変換完了後のコールバック関数
--------------------------------------------------------------- */
void HAL_ADC_ConvCpltCallback( ADC_HandleTypeDef* hadc )
{
	//HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_value, sizeof(adc_value)/sizeof(uint16_t));
}

/* ---------------------------------------------------------------
	赤外センサの初期設定関数
--------------------------------------------------------------- */
void Sensor_Initialize( void )
{
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_value, sizeof(adc_value)/sizeof(uint16_t));
	//statusI1 = 0;
}

