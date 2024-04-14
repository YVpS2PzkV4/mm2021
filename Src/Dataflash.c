#include "index.h"
#include "Dataflash.h"
#include"common.h"
#include"Interface.h"
#include <string.h>

//1byteずつの書き込み
#define FLASH_TYPEPROGRAM_BYTE  0x00000000U
//0x0110 <<3 より 0x0011 0000
#define FLASH_SENTOR6					0x30
//0x0111 <<3 より 0x0011 1000
#define FLASH_SENTOR7					0x38

// flash use address ( sector7 )
const uint32_t start_address = 0x8060000; //sentor7 start address
const uint32_t end_address = 0x807FFFF;

// flash use address ( sector6 )
const uint32_t start_address2 = 0x8040000; //sentor6 start address
const uint32_t end_address2 = 0x805FFFF;

extern unsigned char map[MAZESIZE_X][MAZESIZE_Y];
extern volatile int goalhead;

extern volatile unsigned short nowLog[10];
extern volatile unsigned int countLog;

volatile char flash6_erased = 0;

extern void stopWari(void);
extern void reStartWari(void);

void log_erase(void){
	LED6_ON();
	FLASH_Unlock();
	FLASH_Erase2();
	FLASH_Lock();
	LED6_OFF();
	flash6_erased = 1;
	countLog = 0;
}

void log_write(void){
	if(start_address2+sizeof(nowLog)*(countLog+1) <= end_address2){
		LED6_ON();
		FLASH_AddWriteData(start_address2+sizeof(nowLog)*countLog, (uint8_t*) &nowLog, sizeof(nowLog));
		LED6_OFF();
		countLog++;
	}
}

void log_output(void){
	short p = 0;
	int dummy = 0;
	countLog = 0;
	stopWari();
	HAL_Delay(50);
	while(1){
		if(start_address2+sizeof(nowLog)*(countLog+1) <= end_address2){
			FLASH_ReadData(start_address2+sizeof(nowLog)*countLog, (uint8_t*) &nowLog, sizeof(nowLog));
			countLog++;
			for(p = 0; p < 10; p++){
				if(nowLog[p] > 32767){
					dummy = nowLog[p]-65535;
				}
				else{
					dummy = nowLog[p];
				}
				if(p == 9){
					printf("%d\r\n",dummy);
				}
				else{
					printf("%d,",dummy);
				}
			}
		}
		else{
			break;
		}
	}
	reStartWari();
}

void param_write(void){
	LED5_ON();
	map_copy();
	FLASH_WriteData(start_address, (uint8_t*) &map, sizeof(map));
	FLASH_AddWriteData(start_address+sizeof(map), (uint8_t*) &goalhead, sizeof(goalhead));
	FLASH_AddWriteData(start_address+sizeof(map)+sizeof(goalhead), (uint8_t*) &flash1, sizeof(flash1));
	FLASH_AddWriteData(start_address+sizeof(map)+sizeof(goalhead)+sizeof(flash1), (uint8_t*) &flash2, sizeof(flash2));
	FLASH_AddWriteData(start_address+sizeof(map)+sizeof(goalhead)+sizeof(flash1)+sizeof(flash2), (uint8_t*) &flash3, sizeof(flash3));
	FLASH_AddWriteData(start_address+sizeof(map)+sizeof(goalhead)+sizeof(flash1)+sizeof(flash2)+sizeof(flash3), (uint8_t*) &flash4, sizeof(flash4));
	FLASH_AddWriteData(start_address+sizeof(map)+sizeof(goalhead)+sizeof(flash1)+sizeof(flash2)+sizeof(flash3)+sizeof(flash4), (uint8_t*) &flash5, sizeof(flash5));
	FLASH_AddWriteData(start_address+sizeof(map)+sizeof(goalhead)+sizeof(flash1)+sizeof(flash2)+sizeof(flash3)+sizeof(flash4)+sizeof(flash5), (uint8_t*) &flash6, sizeof(flash6));
	FLASH_AddWriteData(start_address+sizeof(map)+sizeof(goalhead)+sizeof(flash1)+sizeof(flash2)+sizeof(flash3)+sizeof(flash4)+sizeof(flash5)+sizeof(flash6), (uint8_t*) &flash_ID, sizeof(flash_ID));
	LED5_OFF();
}

void param_read(void){
	//LED5_ON();
	FLASH_ReadData(start_address+sizeof(map)+sizeof(goalhead), (uint8_t*) &flash1, sizeof(flash1));
	FLASH_ReadData(start_address+sizeof(map)+sizeof(goalhead)+sizeof(flash1), (uint8_t*) &flash2, sizeof(flash2));
	FLASH_ReadData(start_address+sizeof(map)+sizeof(goalhead)+sizeof(flash1)+sizeof(flash2), (uint8_t*) &flash3, sizeof(flash3));
	FLASH_ReadData(start_address+sizeof(map)+sizeof(goalhead)+sizeof(flash1)+sizeof(flash2)+sizeof(flash3), (uint8_t*) &flash4, sizeof(flash4));
	FLASH_ReadData(start_address+sizeof(map)+sizeof(goalhead)+sizeof(flash1)+sizeof(flash2)+sizeof(flash3)+sizeof(flash4), (uint8_t*) &flash5, sizeof(flash5));
	FLASH_ReadData(start_address+sizeof(map)+sizeof(goalhead)+sizeof(flash1)+sizeof(flash2)+sizeof(flash3)+sizeof(flash4)+sizeof(flash5), (uint8_t*) &flash6, sizeof(flash6));
	FLASH_ReadData(start_address+sizeof(map)+sizeof(goalhead)+sizeof(flash1)+sizeof(flash2)+sizeof(flash3)+sizeof(flash4)+sizeof(flash5)+sizeof(flash6), (uint8_t*) &flash_ID, sizeof(flash_ID));
	//LED5_OFF();
}

void map_write(void){
	LED6_ON();
	FLASH_WriteData(start_address, (uint8_t*) &map, sizeof(map));
	FLASH_AddWriteData(start_address+sizeof(map), (uint8_t*) &goalhead, sizeof(goalhead));
	FLASH_AddWriteData(start_address+sizeof(map)+sizeof(goalhead), (uint8_t*) &flash1, sizeof(flash1));
	FLASH_AddWriteData(start_address+sizeof(map)+sizeof(goalhead)+sizeof(flash1), (uint8_t*) &flash2, sizeof(flash2));
	FLASH_AddWriteData(start_address+sizeof(map)+sizeof(goalhead)+sizeof(flash1)+sizeof(flash2), (uint8_t*) &flash3, sizeof(flash3));
	FLASH_AddWriteData(start_address+sizeof(map)+sizeof(goalhead)+sizeof(flash1)+sizeof(flash2)+sizeof(flash3), (uint8_t*) &flash4, sizeof(flash4));
	FLASH_AddWriteData(start_address+sizeof(map)+sizeof(goalhead)+sizeof(flash1)+sizeof(flash2)+sizeof(flash3)+sizeof(flash4), (uint8_t*) &flash5, sizeof(flash5));
	FLASH_AddWriteData(start_address+sizeof(map)+sizeof(goalhead)+sizeof(flash1)+sizeof(flash2)+sizeof(flash3)+sizeof(flash4)+sizeof(flash5), (uint8_t*) &flash6, sizeof(flash6));
	FLASH_AddWriteData(start_address+sizeof(map)+sizeof(goalhead)+sizeof(flash1)+sizeof(flash2)+sizeof(flash3)+sizeof(flash4)+sizeof(flash5)+sizeof(flash6), (uint8_t*) &flash_ID, sizeof(flash_ID));
	LED6_OFF();
	//FLASH_WriteData(start_address+sizeof(map)+1, (uint8_t*) &goalhead, sizeof(goalhead));
}

void map_write2(void){
	LED6_ON();
	FLASH_WriteData(start_address, (uint8_t*) &map, sizeof(map));
	FLASH_AddWriteData(start_address+sizeof(map), (uint8_t*) &goalhead, sizeof(goalhead));
	FLASH_AddWriteData(start_address+sizeof(map)+sizeof(goalhead), (uint8_t*) &flash1, sizeof(flash1));
	FLASH_AddWriteData(start_address+sizeof(map)+sizeof(goalhead)+sizeof(flash1), (uint8_t*) &flash2, sizeof(flash2));
	FLASH_AddWriteData(start_address+sizeof(map)+sizeof(goalhead)+sizeof(flash1)+sizeof(flash2), (uint8_t*) &flash3, sizeof(flash3));
	FLASH_AddWriteData(start_address+sizeof(map)+sizeof(goalhead)+sizeof(flash1)+sizeof(flash2)+sizeof(flash3), (uint8_t*) &flash4, sizeof(flash4));
	FLASH_AddWriteData(start_address+sizeof(map)+sizeof(goalhead)+sizeof(flash1)+sizeof(flash2)+sizeof(flash3)+sizeof(flash4), (uint8_t*) &flash5, sizeof(flash5));
	FLASH_AddWriteData(start_address+sizeof(map)+sizeof(goalhead)+sizeof(flash1)+sizeof(flash2)+sizeof(flash3)+sizeof(flash4)+sizeof(flash5), (uint8_t*) &flash6, sizeof(flash6));
	FLASH_AddWriteData(start_address+sizeof(map)+sizeof(goalhead)+sizeof(flash1)+sizeof(flash2)+sizeof(flash3)+sizeof(flash4)+sizeof(flash5)+sizeof(flash6), (uint8_t*) &flash_ID, sizeof(flash_ID));
	LED6_OFF();
	//FLASH_WriteData(start_address+sizeof(map)+1, (uint8_t*) &goalhead, sizeof(goalhead));
}

void map_copy(void){
	LED6_ON();
	FLASH_ReadData(start_address, (uint8_t*) &map, sizeof(map));
	FLASH_ReadData(start_address+sizeof(map), (uint8_t*) &goalhead, sizeof(goalhead));
	LED6_OFF();
	//FLASH_ReadData(start_address+sizeof(map)+1, (uint8_t*) &goalhead, sizeof(goalhead));
}

void map_view(void){
	signed char i,j;

	printf("\x1b[0;0H");			//カーソルを0,0に移動
	printf("\n\r+");
	for(i = 0 ; i < MAZESIZE_X; i++){
			if((map[MAZESIZE_Y-1][i]&0x01) == 0x01){
				printf("\x1b[37m--+");//WALL
			}
			else{
				printf("\x1b[37m  +");//NOWALL
			}
	}

	printf("\n\r");
	for(j=(MAZESIZE_Y-1);j>-1;j--){
		if((map[j][0]&0x08) == 0x08){
			printf("\x1b[37m|");//WALL
		}
		else{
			printf("\x1b[37m ");//NOWALL
		}
		for(i=0;i<MAZESIZE_X; i++){
			if((map[j][i]&0x02) == 0x02){
				printf("\x1b[37m  |");//WALL
			}
			else{
				printf("\x1b[37m   ");//NOWALL
			}
		}
		printf("\n\r+");
		for(i=0;i<MAZESIZE_X; i++){
			if((map[j][i]&0x04) == 0x04){
				printf("\x1b[37m--+");//WALL
			}
			else{
				printf("\x1b[37m  +");//NOWALL
			}
		}
		printf("\n\r");
	}
	return;

}


/**
 * @brief 動作中の場合は、動作が終わるまで待つ
 */
void FLASH_WaitBusy(void)
{
	while( ( (FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY) == 1 );
}

/**
 * @brief フラッシュのセクタ7を消去
 */
void FLASH_Erase(void)
{
	FLASH->CR |= FLASH_CR_SER;
	FLASH->CR |= FLASH_SENTOR7 & FLASH_CR_SNB_Msk;
	FLASH->CR |= FLASH_CR_STRT;
	FLASH_WaitBusy();
}

/**
 * @brief フラッシュのセクタ6を消去
 */
void FLASH_Erase2(void)
{
	FLASH->CR |= FLASH_CR_SER;
	FLASH->CR |= FLASH_SENTOR6 & FLASH_CR_SNB_Msk;
	FLASH->CR |= FLASH_CR_STRT;
	FLASH_WaitBusy();
}

/**
 * @brief フラッシュの指定したアドレスに1byteのデータを書き込み
 * @param uint32_t address
 * @param uint8_t data
 */
void FLASH_WriteByte(uint32_t address, uint8_t data)
{

	FLASH->CR &= ~(FLASH_CR_PSIZE);

	FLASH->CR |= FLASH_TYPEPROGRAM_BYTE;
	FLASH->CR |= FLASH_CR_PG;

	*(__IO uint8_t*)address = data;

	FLASH_WaitBusy();

	FLASH->CR &= ~(FLASH_CR_PG);
}

/**
 * @brief フラッシュの指定したアドレスからサイズで指定したところまでデータを書き込む
 * @param uint32_t address
 * @param uint8_t *data
 * @param uint32_t size
 */
void FLASH_WriteData(uint32_t address, uint8_t* data, uint32_t size)
{
	FLASH_Unlock();

	FLASH_Erase();

	do {
		FLASH_WriteByte(address, *data);
	}while(++address, ++data, --size);

	FLASH_Lock();
}

/**
 * @brief フラッシュの指定したアドレスからサイズで指定したところまでデータを書き込む
 * @param uint32_t address
 * @param uint8_t *data
 * @param uint32_t size
 * @detail 前提条件として、イレーズされている番地であることが条件
 */
void FLASH_AddWriteData(uint32_t address, uint8_t* data, uint32_t size)
{
	FLASH_Unlock();

	do {
		FLASH_WriteByte(address, *data);
	}while(++address, ++data, --size);

	FLASH_Lock();

}

/**
 * @brief フラッシュの指定したアドレスからサイズで指定したところまでデータを読み込む
 * @param uint32_t address
 * @param uint8_t *data
 * @param uint32_t size
 */
void FLASH_ReadData(uint32_t address, uint8_t* data, uint32_t size)
{
  memcpy(data, (uint8_t*)address, size);
}
