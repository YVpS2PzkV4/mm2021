#ifndef _CMT_
#define _CMT_

#include"Dataflash.h"
//extern float  flash5[100];
//センサ関連パラメータ

//普通（非斜め）の壁判断用閾値
#define WALL_TH_R_WALL		flash5[0]//75//80		//壁判定用閾値（壁切れには使用しない）（右）
#define WALL_TH_L_WALL		flash5[1]//75//90		//壁判定用閾値（壁切れには使用しない）（左）
#define WALL_TH_FR			flash5[2]//135//180	//壁判定用閾値（右前）
#define WALL_TH_FL			flash5[3]//135//180	//壁判定用閾値（左前）

//普通（非斜め）の壁制御用閾値
#define WALL_CTRL_TH_R		flash5[4]//65			//普通の壁制御用閾値（右）
#define WALL_CTRL_TH_L		flash5[5]//65			//普通の壁制御用閾値（左）
#define WALL_STOP_CTRL_TH_R	flash5[6]//57			//探索中止まった後の制御閾値(櫛っぽいところで不安定になるのでキツめ)（右）
#define WALL_STOP_CTRL_TH_L	flash5[7]//57			//探索中止まった後の制御閾値(櫛っぽいところで不安定になるのでキツめ)（左）

////普通（非斜め）の壁制御リファレンス値
#define Sensor_R_Ref 		flash5[8]//45.3		//右壁中央値
#define Sensor_L_Ref 		flash5[9]//44.0		//左壁中央値

//探索中のスラローム後距離補正用
#define Sensor_Sla_R_Ref	flash5[10]//45.3		//右壁中央値（オフセット区間中）
#define Sensor_Sla_L_Ref	flash5[11]//44.0		//左壁中央値（オフセット区間中）
#define Sensor_Sla_R_Ref_St	flash5[12]//45.3		//右壁中央値（直進中）
#define Sensor_Sla_L_Ref_St	flash5[13]//44.0		//左壁中央値（直進中）

//
#define Sensor_R_Ref_Stop	flash5[14]//45.3		//最初においたときに中央にあるか教えてくれる用（右）
#define Sensor_L_Ref_Stop	flash5[15]//44.0		//最初においたときに中央にあるか教えてくれる用（左）


//壁切れ閾値
#define WALL_TH_R			flash5[16]//70//80		//壁切れ閾値（右）
#define WALL_TH_L			flash5[17]//70//90		//壁切れ閾値（左）

//吸い込まれ対策用閾値
#define WALL_TH_R2			flash5[18]//65			//sudden解除用の閾値（これより近くなったらプラス側の急変はOK）(右)
#define WALL_TH_L2			flash5[19]//65			//sudden解除用の閾値（これより近くなったらプラス側の急変はOK）(左)

#define AFTERCUT flash5[20]//100	//壁切れ後どれくらい制御かけないか

//櫛壁閾値
#define WALL_TH_COMB_R		flash5[21]//85//75		//探索櫛壁切れ（右）
#define WALL_TH_COMB_L		flash5[22]//85//75		//探索櫛壁切れ（左）
#define WALL_TH_COMB_R_B	flash5[23]//120//75		//最短櫛壁切れ（右）
#define WALL_TH_COMB_L_B	flash5[24]//120//75		//最短櫛壁切れ（左）

//斜め壁閾値
#define WALL_TH_SLANT_R		flash5[25]//70//60		//斜め直進壁切れ（右）
#define WALL_TH_SLANT_L		flash5[26]//70//60		//斜め直進壁切れ（左）
#define WALL_TH_SLANT_R_O	flash5[27]//70//60		//斜めターンオフセット時の壁切れ（右）
#define WALL_TH_SLANT_L_O	flash5[28]//70//60		//斜めターンオフセット時の壁切れ（）（左）

//櫛制御閾値（壁切れ位置差）
#define COMB_CTRL_TH_R 		flash5[29]//90			//櫛壁制御閾値　探索（右）
#define COMB_CTRL_TH_L 		flash5[30]//90			//櫛壁制御閾値　探索（左）
#define COMB_CTRL_TH_R_B 	flash5[31]//120			//櫛壁制御閾値　最短（右）
#define COMB_CTRL_TH_L_B 	flash5[32]//120			//櫛壁制御閾値　最短（左）

//斜め制御閾値（壁切れ位置差）
#define SLANT_CTRL_TH_COMB_R flash5[33]//100		//斜め制御（壁切れ距離差）閾値（右）
#define SLANT_CTRL_TH_COMB_L flash5[34]//100		//斜め制御（壁切れ距離差）閾値（左）
#define SUDDENCHANGEB_S 	flash5[35]//0.5			//これだけ急変したら壁切れと判断
#define OFFSET_COMB_S 		flash5[36]//1.0			//斜め壁切れの左右の位置差をオフセット

//#define COMB_FWALL_KP 30.0


//壁制御ゲイン
#define WALL_KP flash5[37]//(0.08)					//通常時
#define WALL_KP_HIGH flash5[38]//(0.2)				//速い時（小島式）
#define WALL_KD_HIGH flash5[39]//(0.1)				//
//#define WALL_KD2	(0.05)				//
#define K_HIGH	flash5[40]//(0.35)					//
//#define WALL_KI (0.0)					//
//#define WALL_KD (0.0)					//

#define SUDDENCHANGE flash5[41]//0.4				//壁距離急変＝壁切れ？
#define SUDDENCHANGECORRECTR flash5[42]//45			//切れたとき壁制御閾値を上げる（右）
#define SUDDENCHANGECORRECTL flash5[43]//45			//切れたとき壁制御閾値を上げる（左）

#define SEARCHWALLCUTA_R	flash5[44]//42.0		//壁切れ位置（右）
#define SEARCHWALLCUTB_R	flash5[45]//10.0		//未使用
#define SEARCHWALLCUTA_L	flash5[46]//42.0		//壁切れ位置（左）
#define SEARCHWALLCUTB_L	flash5[47]//10.0		//未使用

#define SLANTWALLCUT_R	flash5[48]//3.0				//斜め壁切れ（右）
#define SLANTWALLCUT_L	flash5[49]//3.0				//斜め壁切れ（左）

#define FWALL_REF_R	flash5[50]//37.2				//袋小路前壁補正距離（右前）
#define FWALL_REF_L	flash5[51]//38.2				//袋小路前壁補正距離（左前）
#define FWALL_KP_SP	flash5[52]//(0.018)				//袋小路前壁補正Pゲイン（距離）
#define FWALL_KP_ANG	flash5[53]//(0.18)			//袋小路前壁補正Pゲイン（角度）
#define FWALL_KI_ANG	flash5[54]//(0.00)			//袋小路前壁補正Iゲイン（角度）


#define TURN_FWALL_R	flash5[55]//37.2			//超新地するとき前壁の距離がここになるまで進む
#define TURN_FWALL_L	flash5[56]//38.2			//超新地するとき前壁の距離がここになるまで進む
#define SLANTING_F_L flash5[57]//500				//斜め前壁制御閾値（左）
#define SLANTING_F_R flash5[58]//500				//斜め前壁制御閾値（右）
#define KP_WALL_SLANT flash5[59]//200.0//100.0		//斜め前壁制御ゲイン（普通）
#define KP_WALL_SLANT2 flash5[60]//300.0//150.0		//斜め前壁制御ゲイン（速い時）
#define KP_WALL_SLANT3 flash5[61]//60.0//60.0		//斜め前壁制御ゲイン（一区画以下）

#define KP_BEFORETURN	flash5[62]//0.05			//ターン前の壁で角度補正するときのゲイン
#define SLA_FWALL_ALL_B flash5[63]//120.0			//最短中前壁で距離補正？
#define SAITAN_CUTHOSEIR flash5[64]//(-2.0)
#define SAITAN_CUTHOSEIL flash5[65]//(-1.0)//-0.8 11/2

#define SLANT_FWALL_R flash5[66]//60				//斜めゴール時の前壁（近い方）
#define SLANT_FWALL_L flash5[67]//120				//斜めゴール時の前壁（遠い方）

#define FWALL_CLOSE	flash5[68]//100//75				//前壁近い
#define FWALL_CLOSE_SLA	flash5[69]//180				//多分使わん

#define OFFSET_COMB flash5[70]//(0.0)			//櫛壁切れの左右差をオフセット
#define SUDDENCHANGEB flash5[71]//(0.4)			//最短時急変
#define SUDDENCHANGEB2 flash5[72]//(0.8)		//最短時急変（速い時）

#define CONST_A_R	flash5[73]//(993.6522)			//センサ生値→距離の定数
#define CONST_B_R	flash5[74]//(103.6477)
#define CONST_A_L	flash5[75]//(1035.367)
#define CONST_B_L	flash5[76]//(108.6305)
#define CONST_A_FR	flash5[77]//(1113.262)
#define CONST_B_FR	flash5[78]//(106.9508)
#define CONST_A_FL	flash5[79]//(1157.536)
#define CONST_B_FL	flash5[80]//(109.0901)

//物理的なパラメータ
#define PI (3.14159265359f)			//円周率
#define TIRE_DIAMETER	flash5[81]//12.5463f//(12.74)				//タイヤの直径	[mm]
#define MMPP 		(TIRE_DIAMETER*PI)/(4096.0f)	//エンコーダ1パルスあたりに進む距離[mm]
#define TREAD	flash5[82]//(0.038f)

#define SAITANWALLCUTA_R	flash5[83]	//直進中壁あり壁切れ
#define SAITANWALLCUTA_L	flash5[84]
#define SAITANWALLCUTB_R	flash5[85]	//ターン前壁あり壁切れ
#define SAITANWALLCUTB_L	flash5[86]
#define SAITANWALLCUTC_R	flash5[87]	//直進中壁なし壁切れ
#define SAITANWALLCUTC_L	flash5[88]
#define SAITANWALLCUTD_R	flash5[89]	//ターン前壁なし壁切れ
#define SAITANWALLCUTD_L	flash5[90]

#define R_REF_RAW	flash5[91]	//センサ生値リファレンス
#define L_REF_RAW	flash5[92]

#define ANTEI_ANG	flash5[93]
#define ANTEI_DIFF	flash5[94]
#define ANTEI_TIME	flash5[95]

#define FUSION_ALPHA	flash5[96]	//相補フィルタによるセンサフュージョン　加速度センサの割合

#define POLL flash5[97]

#define SEARCHWALLCUTC_R flash5[98]
#define SEARCHWALLCUTC_L flash5[99]

#define WALL_TH_R_SAI flash5[100]	//最短壁有壁切れ速く見るようの閾値
#define WALL_TH_L_SAI flash5[101]

#define SLANT_CTRL_TH_R	flash5[102]	//斜め横壁センサ姿勢制御
#define SLANT_CTRL_TH_L flash5[103]
#define KP_SLANT_SIDE	flash5[104]

#define CUTDIFF1	flash5[105]
#define CUTDIFF2	flash5[106]

#define K_SLIP_R		flash5[107]
#define K_SLIP_L		flash5[108]

///////////////////////////////////////////////////////////////////////////////////////////////////////

#define MYHAND 600						//手
#define FAR 150//1200						//距離化するとき遠すぎるならこれにする

//方位
#define RIGHT	(4)
#define LEFT	(2)
#define FRONT	(1)
#define REAR	(8)

#define LOG_CNT		1024


#define MAX_DPS 1950.0f	//ジャイロの出力角速度限界[deg/s]
#define MAX_RPS (MAX_DPS*PI/180.0f)	//ジャイロの出力角速度限界[rad/s]

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
	volatile float Enc[30];		//エンコーダ計測速度
	volatile float Accel[30];	//ジャイロ計測加速度
	volatile unsigned char Head;
	volatile unsigned char Tail;
	volatile unsigned char Num;
}t_velocity;

t_velocity q_velo;	//速度フュージョン用のQueue*/

#endif
