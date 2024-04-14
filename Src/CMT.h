#ifndef _CMT_
#define _CMT_

#include"Dataflash.h"
//extern float  flash5[100];
//�Z���T�֘A�p�����[�^

//���ʁi��΂߁j�̕ǔ��f�p臒l
#define WALL_TH_R_WALL		flash5[0]//75//80		//�ǔ���p臒l�i�ǐ؂�ɂ͎g�p���Ȃ��j�i�E�j
#define WALL_TH_L_WALL		flash5[1]//75//90		//�ǔ���p臒l�i�ǐ؂�ɂ͎g�p���Ȃ��j�i���j
#define WALL_TH_FR			flash5[2]//135//180	//�ǔ���p臒l�i�E�O�j
#define WALL_TH_FL			flash5[3]//135//180	//�ǔ���p臒l�i���O�j

//���ʁi��΂߁j�̕ǐ���p臒l
#define WALL_CTRL_TH_R		flash5[4]//65			//���ʂ̕ǐ���p臒l�i�E�j
#define WALL_CTRL_TH_L		flash5[5]//65			//���ʂ̕ǐ���p臒l�i���j
#define WALL_STOP_CTRL_TH_R	flash5[6]//57			//�T�����~�܂�����̐���臒l(�����ۂ��Ƃ���ŕs����ɂȂ�̂ŃL�c��)�i�E�j
#define WALL_STOP_CTRL_TH_L	flash5[7]//57			//�T�����~�܂�����̐���臒l(�����ۂ��Ƃ���ŕs����ɂȂ�̂ŃL�c��)�i���j

////���ʁi��΂߁j�̕ǐ��䃊�t�@�����X�l
#define Sensor_R_Ref 		flash5[8]//45.3		//�E�ǒ����l
#define Sensor_L_Ref 		flash5[9]//44.0		//���ǒ����l

//�T�����̃X�����[���㋗���␳�p
#define Sensor_Sla_R_Ref	flash5[10]//45.3		//�E�ǒ����l�i�I�t�Z�b�g��Ԓ��j
#define Sensor_Sla_L_Ref	flash5[11]//44.0		//���ǒ����l�i�I�t�Z�b�g��Ԓ��j
#define Sensor_Sla_R_Ref_St	flash5[12]//45.3		//�E�ǒ����l�i���i���j
#define Sensor_Sla_L_Ref_St	flash5[13]//44.0		//���ǒ����l�i���i���j

//
#define Sensor_R_Ref_Stop	flash5[14]//45.3		//�ŏ��ɂ������Ƃ��ɒ����ɂ��邩�����Ă����p�i�E�j
#define Sensor_L_Ref_Stop	flash5[15]//44.0		//�ŏ��ɂ������Ƃ��ɒ����ɂ��邩�����Ă����p�i���j


//�ǐ؂�臒l
#define WALL_TH_R			flash5[16]//70//80		//�ǐ؂�臒l�i�E�j
#define WALL_TH_L			flash5[17]//70//90		//�ǐ؂�臒l�i���j

//�z�����܂�΍��p臒l
#define WALL_TH_R2			flash5[18]//65			//sudden�����p��臒l�i������߂��Ȃ�����v���X���̋}�ς�OK�j(�E)
#define WALL_TH_L2			flash5[19]//65			//sudden�����p��臒l�i������߂��Ȃ�����v���X���̋}�ς�OK�j(��)

#define AFTERCUT flash5[20]//100	//�ǐ؂��ǂꂭ�炢���䂩���Ȃ���

//����臒l
#define WALL_TH_COMB_R		flash5[21]//85//75		//�T�����ǐ؂�i�E�j
#define WALL_TH_COMB_L		flash5[22]//85//75		//�T�����ǐ؂�i���j
#define WALL_TH_COMB_R_B	flash5[23]//120//75		//�ŒZ���ǐ؂�i�E�j
#define WALL_TH_COMB_L_B	flash5[24]//120//75		//�ŒZ���ǐ؂�i���j

//�΂ߕ�臒l
#define WALL_TH_SLANT_R		flash5[25]//70//60		//�΂ߒ��i�ǐ؂�i�E�j
#define WALL_TH_SLANT_L		flash5[26]//70//60		//�΂ߒ��i�ǐ؂�i���j
#define WALL_TH_SLANT_R_O	flash5[27]//70//60		//�΂߃^�[���I�t�Z�b�g���̕ǐ؂�i�E�j
#define WALL_TH_SLANT_L_O	flash5[28]//70//60		//�΂߃^�[���I�t�Z�b�g���̕ǐ؂�i�j�i���j

//������臒l�i�ǐ؂�ʒu���j
#define COMB_CTRL_TH_R 		flash5[29]//90			//���ǐ���臒l�@�T���i�E�j
#define COMB_CTRL_TH_L 		flash5[30]//90			//���ǐ���臒l�@�T���i���j
#define COMB_CTRL_TH_R_B 	flash5[31]//120			//���ǐ���臒l�@�ŒZ�i�E�j
#define COMB_CTRL_TH_L_B 	flash5[32]//120			//���ǐ���臒l�@�ŒZ�i���j

//�΂ߐ���臒l�i�ǐ؂�ʒu���j
#define SLANT_CTRL_TH_COMB_R flash5[33]//100		//�΂ߐ���i�ǐ؂ꋗ�����j臒l�i�E�j
#define SLANT_CTRL_TH_COMB_L flash5[34]//100		//�΂ߐ���i�ǐ؂ꋗ�����j臒l�i���j
#define SUDDENCHANGEB_S 	flash5[35]//0.5			//���ꂾ���}�ς�����ǐ؂�Ɣ��f
#define OFFSET_COMB_S 		flash5[36]//1.0			//�΂ߕǐ؂�̍��E�̈ʒu�����I�t�Z�b�g

//#define COMB_FWALL_KP 30.0


//�ǐ���Q�C��
#define WALL_KP flash5[37]//(0.08)					//�ʏ펞
#define WALL_KP_HIGH flash5[38]//(0.2)				//�������i�������j
#define WALL_KD_HIGH flash5[39]//(0.1)				//
//#define WALL_KD2	(0.05)				//
#define K_HIGH	flash5[40]//(0.35)					//
//#define WALL_KI (0.0)					//
//#define WALL_KD (0.0)					//

#define SUDDENCHANGE flash5[41]//0.4				//�ǋ����}�ρ��ǐ؂�H
#define SUDDENCHANGECORRECTR flash5[42]//45			//�؂ꂽ�Ƃ��ǐ���臒l���グ��i�E�j
#define SUDDENCHANGECORRECTL flash5[43]//45			//�؂ꂽ�Ƃ��ǐ���臒l���グ��i���j

#define SEARCHWALLCUTA_R	flash5[44]//42.0		//�ǐ؂�ʒu�i�E�j
#define SEARCHWALLCUTB_R	flash5[45]//10.0		//���g�p
#define SEARCHWALLCUTA_L	flash5[46]//42.0		//�ǐ؂�ʒu�i���j
#define SEARCHWALLCUTB_L	flash5[47]//10.0		//���g�p

#define SLANTWALLCUT_R	flash5[48]//3.0				//�΂ߕǐ؂�i�E�j
#define SLANTWALLCUT_L	flash5[49]//3.0				//�΂ߕǐ؂�i���j

#define FWALL_REF_R	flash5[50]//37.2				//�܏��H�O�Ǖ␳�����i�E�O�j
#define FWALL_REF_L	flash5[51]//38.2				//�܏��H�O�Ǖ␳�����i���O�j
#define FWALL_KP_SP	flash5[52]//(0.018)				//�܏��H�O�Ǖ␳P�Q�C���i�����j
#define FWALL_KP_ANG	flash5[53]//(0.18)			//�܏��H�O�Ǖ␳P�Q�C���i�p�x�j
#define FWALL_KI_ANG	flash5[54]//(0.00)			//�܏��H�O�Ǖ␳I�Q�C���i�p�x�j


#define TURN_FWALL_R	flash5[55]//37.2			//���V�n����Ƃ��O�ǂ̋����������ɂȂ�܂Ői��
#define TURN_FWALL_L	flash5[56]//38.2			//���V�n����Ƃ��O�ǂ̋����������ɂȂ�܂Ői��
#define SLANTING_F_L flash5[57]//500				//�΂ߑO�ǐ���臒l�i���j
#define SLANTING_F_R flash5[58]//500				//�΂ߑO�ǐ���臒l�i�E�j
#define KP_WALL_SLANT flash5[59]//200.0//100.0		//�΂ߑO�ǐ���Q�C���i���ʁj
#define KP_WALL_SLANT2 flash5[60]//300.0//150.0		//�΂ߑO�ǐ���Q�C���i�������j
#define KP_WALL_SLANT3 flash5[61]//60.0//60.0		//�΂ߑO�ǐ���Q�C���i����ȉ��j

#define KP_BEFORETURN	flash5[62]//0.05			//�^�[���O�̕ǂŊp�x�␳����Ƃ��̃Q�C��
#define SLA_FWALL_ALL_B flash5[63]//120.0			//�ŒZ���O�ǂŋ����␳�H
#define SAITAN_CUTHOSEIR flash5[64]//(-2.0)
#define SAITAN_CUTHOSEIL flash5[65]//(-1.0)//-0.8 11/2

#define SLANT_FWALL_R flash5[66]//60				//�΂߃S�[�����̑O�ǁi�߂����j
#define SLANT_FWALL_L flash5[67]//120				//�΂߃S�[�����̑O�ǁi�������j

#define FWALL_CLOSE	flash5[68]//100//75				//�O�ǋ߂�
#define FWALL_CLOSE_SLA	flash5[69]//180				//�����g���

#define OFFSET_COMB flash5[70]//(0.0)			//���ǐ؂�̍��E�����I�t�Z�b�g
#define SUDDENCHANGEB flash5[71]//(0.4)			//�ŒZ���}��
#define SUDDENCHANGEB2 flash5[72]//(0.8)		//�ŒZ���}�ρi�������j

#define CONST_A_R	flash5[73]//(993.6522)			//�Z���T���l�������̒萔
#define CONST_B_R	flash5[74]//(103.6477)
#define CONST_A_L	flash5[75]//(1035.367)
#define CONST_B_L	flash5[76]//(108.6305)
#define CONST_A_FR	flash5[77]//(1113.262)
#define CONST_B_FR	flash5[78]//(106.9508)
#define CONST_A_FL	flash5[79]//(1157.536)
#define CONST_B_FL	flash5[80]//(109.0901)

//�����I�ȃp�����[�^
#define PI (3.14159265359f)			//�~����
#define TIRE_DIAMETER	flash5[81]//12.5463f//(12.74)				//�^�C���̒��a	[mm]
#define MMPP 		(TIRE_DIAMETER*PI)/(4096.0f)	//�G���R�[�_1�p���X������ɐi�ދ���[mm]
#define TREAD	flash5[82]//(0.038f)

#define SAITANWALLCUTA_R	flash5[83]	//���i���ǂ���ǐ؂�
#define SAITANWALLCUTA_L	flash5[84]
#define SAITANWALLCUTB_R	flash5[85]	//�^�[���O�ǂ���ǐ؂�
#define SAITANWALLCUTB_L	flash5[86]
#define SAITANWALLCUTC_R	flash5[87]	//���i���ǂȂ��ǐ؂�
#define SAITANWALLCUTC_L	flash5[88]
#define SAITANWALLCUTD_R	flash5[89]	//�^�[���O�ǂȂ��ǐ؂�
#define SAITANWALLCUTD_L	flash5[90]

#define R_REF_RAW	flash5[91]	//�Z���T���l���t�@�����X
#define L_REF_RAW	flash5[92]

#define ANTEI_ANG	flash5[93]
#define ANTEI_DIFF	flash5[94]
#define ANTEI_TIME	flash5[95]

#define FUSION_ALPHA	flash5[96]	//����t�B���^�ɂ��Z���T�t���[�W�����@�����x�Z���T�̊���

#define POLL flash5[97]

#define SEARCHWALLCUTC_R flash5[98]
#define SEARCHWALLCUTC_L flash5[99]

#define WALL_TH_R_SAI flash5[100]	//�ŒZ�ǗL�ǐ؂ꑬ������悤��臒l
#define WALL_TH_L_SAI flash5[101]

#define SLANT_CTRL_TH_R	flash5[102]	//�΂߉��ǃZ���T�p������
#define SLANT_CTRL_TH_L flash5[103]
#define KP_SLANT_SIDE	flash5[104]

#define CUTDIFF1	flash5[105]
#define CUTDIFF2	flash5[106]

#define K_SLIP_R		flash5[107]
#define K_SLIP_L		flash5[108]

///////////////////////////////////////////////////////////////////////////////////////////////////////

#define MYHAND 600						//��
#define FAR 150//1200						//����������Ƃ���������Ȃ炱��ɂ���

//����
#define RIGHT	(4)
#define LEFT	(2)
#define FRONT	(1)
#define REAR	(8)

#define LOG_CNT		1024


#define MAX_DPS 1950.0f	//�W���C���̏o�͊p���x���E[deg/s]
#define MAX_RPS (MAX_DPS*PI/180.0f)	//�W���C���̏o�͊p���x���E[rad/s]

#define FAIL_SPEED (0.6)//(0.4)
#define FAILOUTSP 75
#define FAIL_ANG_VEL (4.0)//(2.5)
#define FAILOUTANG 75

//#define FASTFORCTRL (2.0)

void intrpt1(void);
void intrpt2(void);
void intrpt3(void);

unsigned short getEncTable(char,short);
void resetCurrent(void);
void wait_ms(volatile int);

/*void vQ_push(float,float);
void estimateV(void);

typedef struct
{
	volatile float Enc[30];		//�G���R�[�_�v�����x
	volatile float Accel[30];	//�W���C���v�������x
	volatile unsigned char Head;
	volatile unsigned char Tail;
	volatile unsigned char Num;
}t_velocity;

t_velocity q_velo;	//���x�t���[�W�����p��Queue*/

#endif
