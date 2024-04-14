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


#define MAZESIZE_X		(32)		//���H�̑傫��(MAZESIZE_X * MAZESIZE_Y)���H
#define MAZESIZE_Y		(32)		//���H�̑傫��(MAZESIZE_X * MAZESIZE_Y)���H

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
	//�^�[���P�̂̃p�����[�^�i��`or�O�p�֐��j
	float v;		//���x
	float alpha;	//�p�����x
	float w;		//�p���x
	float angleR;	//�p�x
	float angleL;
	float offset1;	//�O����
	float offset2;	//�㋗��
}t_turn1;

typedef struct
{
	//�P�Z�b�g�̍ŒZ�^�[���p�����[�^
	t_turn1 big90;
	t_turn1 big180;
	t_turn1 in45;
	t_turn1 in135;
	t_turn1 v90;
	t_turn1 out45;
	t_turn1 out135;
	t_turn1 kojima;

	//���i���̑��̃p�����[�^
	float fan;			//�t�@����d��
	char sin;			//�^�[���������@�i0:��`�@1:�O�p�֐��@2:�l�C�s�A�j
	//
	float dashv;		//���imax
	float acc;			//���i�����x
	float dec;			//���i�����x
	//
	float slantdashv;	//�΂�max
	float slantacc;		//�΂߉����x
	float slantdec;		//�΂ߌ����x
	//
	float fastacc;		//�^�[������^�[���̉����x
	float fastdec;		//�^�[������^�[���̌����x
	float firstacc;		//����^�[���̉����x
	float acc1;			//���撼�i�̉����x
	float dec1;			//���撼�i�̌����x
	float slantacc1;	//���撼�i�̉����x�i�΂߁j
	float slantdec1;	//���撼�i�̌����x�i�΂߁j
	float stopdec;		//�~�܂�Ƃ��̌����x
	float minv;			//�Œᑬ�x�H
	float offsetmaxv;	//�^�[���I�t�Z�b�g��max���x
}t_saitanpara;

typedef struct
{
	//���H�֘A�̃p�����[�^
	unsigned char goalSize;		//�S�[���}�X��
	unsigned char goalX;		//�S�[�����WX
	unsigned char goalY;		//�S�[�����WY
	unsigned char compSizeX;	//���H�̑傫��X
	unsigned char compSizeY;	//���H�̑傫��Y
	float searchTime;			//�T���������ԁimin�j
	float lenDown;				//�T�����x���Ƃ�����(m)
}t_mazepara;

typedef struct
{
	//����֘A�̃p�����[�^
	//�E
	float speedKP;		//���i����P
	float speedKI;		//���i����I
	float speedKD;		//���i����D
	//��
	float speedKP2;		//���i����P
	float speedKI2;		//���i����I
	float speedKD2;		//���i����D
	//�E
	float omegaKP;		//��]����P
	float omegaKI;		//��]����I
	float omegaKD;		//��]����D
	//��
	float omegaKP2;		//��]����P
	float omegaKI2;		//��]����I
	float omegaKD2;		//��]����D

	float omegaKP2R;		//��]����P
	float omegaKI2R;		//��]����I
	float omegaKD2R;		//��]����D
	float omegaKP2L;		//��]����P
	float omegaKI2L;		//��]����I
	float omegaKD2L;		//��]����D
	float angleKP;		//��Ίp�xP
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
	float ffAccel2R;	//�z����
	float ffSpeed2R;
	float ffConst2R;
	float ffAccel2L;	//�z����
	float ffSpeed2L;
	float ffConst2L;

	float ffAngAcc2Rr;	//�E�^�[���E���[�^�[
	float ffAngVel2Rr;
	float ffAngConst2Rr;
	float ffAngAcc2Rl;	//�E�^�[�������[�^�[
	float ffAngVel2Rl;
	float ffAngConst2Rl;

	float ffAngAcc2Lr;	//���^�[���E���[�^�[
	float ffAngVel2Lr;
	float ffAngConst2Lr;
	float ffAngAcc2Ll;	//���^�[�������[�^�[
	float ffAngVel2Ll;
	float ffAngConst2Ll;

}t_ctrlpara;

//Flash�̈�ɕۑ����Ă����p�����[�^����
t_searchpara flash1;					//�T���p�����[�^
t_saitanpara flash2[SAITANPARANUM];		//�ŒZ�p�����[�^
t_mazepara flash3;						//���H�֘A�p�����[�^
t_ctrlpara flash4;						//����֘A�p�����[�^
float flash5[110];						//CMT.h�Œ�`����Ă���p�����[�^�i��ɕǊ֘A�j
float flash6[10];						//���̑�
float flash_ID;							//�p�����[�^�������݂��ُ�łȂ����`�F�b�N����p

#define MACHINE_ID	123.45				//�@��ID

#endif /* __FLASH_H */

