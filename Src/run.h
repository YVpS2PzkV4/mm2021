#ifndef _RUN_
#define _RUN_

#include"Dataflash.h"

//�֐�
void straight(float, float, float, float,int,int);
void straight3(float, float,float, float, float,int,int);
void straight2(float, float, float, float, float, float,int,int);
void go(float,float,float,float,float,float,float,unsigned char);
void runoffseta(float, float, float, float);
void runoffsetb(float, float, float, float);
void back(float, float, float, float);
void turn(int, float, float, short);
void sla(float, float, float,float, short,float,float,float);
void sla2(float, float, float,float, short,float,float,float);
void sla3(float, float, float,float, short, float, float);
void f_wall(float,float);
void party(void);
void tracelog(void);
void turn_fwall(void);
void turn_fwall2(void);
void motor_setVoltage(float,unsigned char);


//���i�����t�B�[�h�o�b�N�Q�C��
//�E
#define SPEED_KP	flash4.speedKP//(12.0)//(13.33)//(20.0)				//P�Q�C��
#define SPEED_KI	flash4.speedKI//(0.1)//(0.13)//(0.2)				//I�Q�C��
#define SPEED_KD	flash4.speedKD//(5.0)//(8.0)				//D�Q�C��
//��
#define SPEED_KP2	flash4.speedKP2//(12.0)//(13.33)//(20.0)				//P�Q�C��
#define SPEED_KI2	flash4.speedKI2//(0.1)//(0.13)//(0.2)				//I�Q�C��
#define SPEED_KD2	flash4.speedKD2//(5.0)//(8.0)				//D�Q�C��

//��]�����t�B�[�h�o�b�N�Q�C��
//�E
#define OMEGA_KP	flash4.omegaKP//(10.0)//10.0...3/17//(26.67)//(40.0)				//P�Q�C��
#define OMEGA_KI	flash4.omegaKI//(2.0)//(5.0)//(7.5)			//I�Q�C��
#define OMEGA_KD	flash4.omegaKD//(0.0)//(10.0)//(15.0)				//D�Q�C��
//��
#define OMEGA_KP2	flash4.omegaKP2//(10.0)//10.0...3/17//(26.67)//(40.0)				//P�Q�C��
#define OMEGA_KI2	flash4.omegaKI2//(2.0)//(5.0)//(7.5)			//I�Q�C��
#define OMEGA_KD2	flash4.omegaKD2//(0.0)//(10.0)//(15.0)				//D�Q�C��

#define ANGLE_KP	flash4.angleKP

#define SPEED_KP_MINUS	flash4.speedKPminus
#define SPEED_KI_MINUS	flash4.speedKIminus
#define SPEED_KD_MINUS	flash4.speedKDminus
#define OMEGA_KP_MINUS	flash4.omegaKPminus
#define OMEGA_KI_MINUS	flash4.omegaKIminus
#define OMEGA_KD_MINUS	flash4.omegaKDminus

//FF���i�����p�����[�^
#define FF_ACCEL			flash4.ffAccel//0.102115f	//FF�����x��
#define FF_SPEED			flash4.ffSpeed//0.293453f	//FF���x��
#define FF_CONST			flash4.ffConst//0.16245f	//FF���C��

//FF��]�����p�����[�^
#define FF_ANG_ACC			flash4.ffAngAcc//0.00155f	//FF�p�����x��
#define FF_ANG_VEL			flash4.ffAngVel//0.021297f	//FF�p���x��
#define FF_ANG_CONST		flash4.ffAngConst//0.12629f	//FF���C���H�悭�킩��Ȃ�

//FF���i�����p�����[�^
#define FF_ACCEL2R			flash4.ffAccel2R//0.102115f	//FF�����x��
#define FF_SPEED2R			flash4.ffSpeed2R//0.293453f	//FF���x��
#define FF_CONST2R			flash4.ffConst2R//0.16245f	//FF���C��
#define FF_ACCEL2L			flash4.ffAccel2L//0.102115f	//FF�����x��
#define FF_SPEED2L			flash4.ffSpeed2L//0.293453f	//FF���x��
#define FF_CONST2L			flash4.ffConst2L//0.16245f	//FF���C��

//FF��]�����p�����[�^
#define FF_ANG_ACC2RR			flash4.ffAngAcc2Rr//0.00155f	//FF�p�����x��
#define FF_ANG_VEL2RR			flash4.ffAngVel2Rr//0.021297f	//FF�p���x��
#define FF_ANG_CONST2RR		flash4.ffAngConst2Rr//0.12629f	//FF���C���H�悭�킩��Ȃ�
#define FF_ANG_ACC2RL			flash4.ffAngAcc2Rl//0.00155f	//FF�p�����x��
#define FF_ANG_VEL2RL			flash4.ffAngVel2Rl//0.021297f	//FF�p���x��
#define FF_ANG_CONST2RL		flash4.ffAngConst2Rl//0.12629f	//FF���C���H�悭�킩��Ȃ�

#define FF_ANG_ACC2LR			flash4.ffAngAcc2Lr//0.00155f	//FF�p�����x��
#define FF_ANG_VEL2LR			flash4.ffAngVel2Lr//0.021297f	//FF�p���x��
#define FF_ANG_CONST2LR		flash4.ffAngConst2Lr//0.12629f	//FF���C���H�悭�킩��Ȃ�
#define FF_ANG_ACC2LL			flash4.ffAngAcc2Ll//0.00155f	//FF�p�����x��
#define FF_ANG_VEL2LL			flash4.ffAngVel2Ll//0.021297f	//FF�p���x��
#define FF_ANG_CONST2LL		flash4.ffAngConst2Ll//0.12629f	//FF���C���H�悭�킩��Ȃ�

//#define DEG_KP		(0.0)//(10.0)

//#define DEG_KP_S		(0.0)//(15.0)
//#define DEG_KI_S		(0.0)

//run_mode
#define STRAIGHT_MODE	0		//���i���̃��[�h
#define TURN_MODE		1		//���M�n���񎞂̃��[�h
#define BACK_MODE		2		//�o�b�N���̃��[�h
#define SLA_MODE		3		//�X�����[�����[�h(��`)
#define NON_CON_MODE	4		//�񐧌䃂�[�h
#define TEST_MODE		5		//�e�X�g���[�h(���荞�ݗp���[�^�����؂郂�[�h)
#define F_WALL_MODE		6		//�O�ǈʒu����
#define PARTY_MODE		7		//����|
#define OFFSET_A_MODE	8		//�X�����[���p�I�t�Z�b�g���i
#define OFFSET_B_MODE	9		//�X�����[���p�I�t�Z�b�g���i
#define TRACELOG_MODE	10		//���O�Đ�
#define SLA2_MODE		11		//�X�����[�����[�h(�O�p�֐�)
#define STRAIGHT2_MODE	12		//���i���̃��[�h(�O�p�֐�)
#define SLA3_MODE		13		//�X�����[�����[�h(�l�C�s�A)
#define VOLT_MODE		14		//�d���w�肷�邾��

//���s�p�����[�^
#define SEARCH_SPEED	(0.3f)				//�T�����s�̑��x	[m/s]
#define SEARCH_ACCEL	(1.0f)				//�T�����s�̉����x	[m/s^2]
#define MIN_SPEED	(0.1f)				//�Œᑬ�x	[m/s]

#define TURN_ACCEL	(PI*30.0f)//15				//���M�n����̉����x	[rad/s^2]
#define	TURN_SPEED	(PI*1.5f)//*3.6f				//���M�n����̍ō����x	[rad/s]
#define	TURN_SPEED2	(PI*1.5f)				//���M�n����̍ō����x	[rad/s]
#define TURN_MIN_SPEED	(PI*0.4f)			//���M�n����̍Œᑬ�x	[rad/s]

#endif
