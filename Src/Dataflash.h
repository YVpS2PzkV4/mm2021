#ifndef __FLASH_H
#define __FLASH_H

#include <stdint.h>

#include "main.h"

#define FLASH_KEY1               0x45670123U
#define FLASH_KEY2               0xCDEF89ABU

// flash use address ( sector11 )
extern const uint32_t start_address; //sentor11 start address
extern const uint32_t end_adress;

__STATIC_INLINE void FLASH_Lock(void)
{
	FLASH->CR |= FLASH_CR_LOCK;

}

__STATIC_INLINE void FLASH_Unlock(void)
{
	FLASH->KEYR =  FLASH_KEY1;
	FLASH->KEYR =  FLASH_KEY2;
}

void FLASH_WaitBusy(void);
void FLASH_Erase(void);
void FLASH_Erase2(void);
void FLASH_WriteByte(uint32_t address, uint8_t data);
void FLASH_ReadData(uint32_t address, uint8_t* data, uint32_t size);
void FLASH_WriteData(uint32_t address, uint8_t* data, uint32_t size);
void FLASH_AddWriteData(uint32_t address, uint8_t* data, uint32_t size);


#define MAZESIZE_X		(32)		//迷路の大きさ(MAZESIZE_X * MAZESIZE_Y)迷路
#define MAZESIZE_Y		(32)		//迷路の大きさ(MAZESIZE_X * MAZESIZE_Y)迷路

void log_erase(void);
void log_write(void);
void log_output(void);
void param_write(void);
void param_read(void);
void map_write(void);
void map_write2(void);
void map_copy(void);
void map_view(void);

#define SEARCHPARANUM	4
#define SAITANPARANUM   15

typedef struct
{
	float fanV[SEARCHPARANUM];
	float maxV[SEARCHPARANUM];
	float acc[SEARCHPARANUM];
	float knownMaxV[SEARCHPARANUM];
	float knownAcc[SEARCHPARANUM];
	float slaAlpha[SEARCHPARANUM];
	float slaW[SEARCHPARANUM];
	float offsetRA[SEARCHPARANUM];
	float offsetRB[SEARCHPARANUM];
	float offsetLA[SEARCHPARANUM];
	float offsetLB[SEARCHPARANUM];
	float slaMinV[SEARCHPARANUM];
	float degR[SEARCHPARANUM];
	float degL[SEARCHPARANUM];
	float fWall[SEARCHPARANUM];
}t_searchpara;

typedef struct
{
	//ターン単体のパラメータ（台形or三角関数）
	float v;		//速度
	float alpha;	//角加速度
	float w;		//角速度
	float angleR;	//角度
	float angleL;
	float offset1;	//前距離
	float offset2;	//後距離
}t_turn1;

typedef struct
{
	//１セットの最短ターンパラメータ
	t_turn1 big90;
	t_turn1 big180;
	t_turn1 in45;
	t_turn1 in135;
	t_turn1 v90;
	t_turn1 out45;
	t_turn1 out135;
	t_turn1 kojima;

	//直進その他のパラメータ
	float fan;			//ファン印可電圧
	char sin;			//ターン加速方法（0:台形　1:三角関数　2:ネイピア）
	//
	float dashv;		//直進max
	float acc;			//直進加速度
	float dec;			//直進減速度
	//
	float slantdashv;	//斜めmax
	float slantacc;		//斜め加速度
	float slantdec;		//斜め減速度
	//
	float fastacc;		//ターンからターンの加速度
	float fastdec;		//ターンからターンの減速度
	float firstacc;		//初手ターンの加速度
	float acc1;			//一区画直進の加速度
	float dec1;			//一区画直進の減速度
	float slantacc1;	//一区画直進の加速度（斜め）
	float slantdec1;	//一区画直進の減速度（斜め）
	float stopdec;		//止まるときの減速度
	float minv;			//最低速度？
	float offsetmaxv;	//ターンオフセットのmax速度
}t_saitanpara;

typedef struct
{
	//迷路関連のパラメータ
	unsigned char goalSize;		//ゴールマス数
	unsigned char goalX;		//ゴール座標X
	unsigned char goalY;		//ゴール座標Y
	unsigned char compSizeX;	//迷路の大きさX
	unsigned char compSizeY;	//迷路の大きさY
	float searchTime;			//探索制限時間（min）
	float lenDown;				//探索速度落とす距離(m)
}t_mazepara;

typedef struct
{
	//制御関連のパラメータ
	//右
	float speedKP;		//並進方向P
	float speedKI;		//並進方向I
	float speedKD;		//並進方向D
	//左
	float speedKP2;		//並進方向P
	float speedKI2;		//並進方向I
	float speedKD2;		//並進方向D
	//右
	float omegaKP;		//回転方向P
	float omegaKI;		//回転方向I
	float omegaKD;		//回転方向D
	//左
	float omegaKP2;		//回転方向P
	float omegaKI2;		//回転方向I
	float omegaKD2;		//回転方向D

	float omegaKP2R;		//回転方向P
	float omegaKI2R;		//回転方向I
	float omegaKD2R;		//回転方向D
	float omegaKP2L;		//回転方向P
	float omegaKI2L;		//回転方向I
	float omegaKD2L;		//回転方向D
	float angleKP;		//絶対角度P
	//
	float speedKPminus;
	float speedKIminus;
	float speedKDminus;
	float omegaKPminus;
	float omegaKIminus;
	float omegaKDminus;
	//
	float ffAccel;
	float ffSpeed;
	float ffConst;
	float ffAngAcc;
	float ffAngVel;
	float ffAngConst;
	//
	float ffAccel2R;	//吸引時
	float ffSpeed2R;
	float ffConst2R;
	float ffAccel2L;	//吸引時
	float ffSpeed2L;
	float ffConst2L;

	float ffAngAcc2Rr;	//右ターン右モーター
	float ffAngVel2Rr;
	float ffAngConst2Rr;
	float ffAngAcc2Rl;	//右ターン左モーター
	float ffAngVel2Rl;
	float ffAngConst2Rl;

	float ffAngAcc2Lr;	//左ターン右モーター
	float ffAngVel2Lr;
	float ffAngConst2Lr;
	float ffAngAcc2Ll;	//左ターン左モーター
	float ffAngVel2Ll;
	float ffAngConst2Ll;

}t_ctrlpara;

//Flash領域に保存しておくパラメータたち
t_searchpara flash1;					//探索パラメータ
t_saitanpara flash2[SAITANPARANUM];		//最短パラメータ
t_mazepara flash3;						//迷路関連パラメータ
t_ctrlpara flash4;						//制御関連パラメータ
float flash5[110];						//CMT.hで定義されているパラメータ（主に壁関連）
float flash6[10];						//その他
float flash_ID;							//パラメータ書き込みが異常でないかチェックする用

#define MACHINE_ID	123.45				//機体ID

#endif /* __FLASH_H */

