
#include "index.h"

uint16_t			adc_value[5] = {0,0,0,0,0};		// AD�ϊ��l
uint16_t			battery_value = 0;		// �o�b�e���d���̐��f�[�^
uint16_t	sensor_value_on[4] = {0,0,0,0};	// IR�Z���T�̐��f�[�^
uint16_t	sensor_value_off[4] = {0,0,0,0};	// IR�Z���T�̐��f�[�^

/* ---------------------------------------------------------------
	AD�ϊ�������̃R�[���o�b�N�֐�
--------------------------------------------------------------- */
void HAL_ADC_ConvCpltCallback( ADC_HandleTypeDef* hadc )
{
	//HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_value, sizeof(adc_value)/sizeof(uint16_t));
}

/* ---------------------------------------------------------------
	�ԊO�Z���T�̏����ݒ�֐�
--------------------------------------------------------------- */
void Sensor_Initialize( void )
{
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_value, sizeof(adc_value)/sizeof(uint16_t));
	//statusI1 = 0;
}

