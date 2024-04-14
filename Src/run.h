#ifndef _RUN_
#define _RUN_

#include"Dataflash.h"

//関数
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


//並進方向フィードバックゲイン
//右
#define SPEED_KP	flash4.speedKP//(12.0)//(13.33)//(20.0)				//Pゲイン
#define SPEED_KI	flash4.speedKI//(0.1)//(0.13)//(0.2)				//Iゲイン
#define SPEED_KD	flash4.speedKD//(5.0)//(8.0)				//Dゲイン
//左
#define SPEED_KP2	flash4.speedKP2//(12.0)//(13.33)//(20.0)				//Pゲイン
#define SPEED_KI2	flash4.speedKI2//(0.1)//(0.13)//(0.2)				//Iゲイン
#define SPEED_KD2	flash4.speedKD2//(5.0)//(8.0)				//Dゲイン

//回転方向フィードバックゲイン
//右
#define OMEGA_KP	flash4.omegaKP//(10.0)//10.0...3/17//(26.67)//(40.0)				//Pゲイン
#define OMEGA_KI	flash4.omegaKI//(2.0)//(5.0)//(7.5)			//Iゲイン
#define OMEGA_KD	flash4.omegaKD//(0.0)//(10.0)//(15.0)				//Dゲイン
//左
#define OMEGA_KP2	flash4.omegaKP2//(10.0)//10.0...3/17//(26.67)//(40.0)				//Pゲイン
#define OMEGA_KI2	flash4.omegaKI2//(2.0)//(5.0)//(7.5)			//Iゲイン
#define OMEGA_KD2	flash4.omegaKD2//(0.0)//(10.0)//(15.0)				//Dゲイン

#define ANGLE_KP	flash4.angleKP

#define SPEED_KP_MINUS	flash4.speedKPminus
#define SPEED_KI_MINUS	flash4.speedKIminus
#define SPEED_KD_MINUS	flash4.speedKDminus
#define OMEGA_KP_MINUS	flash4.omegaKPminus
#define OMEGA_KI_MINUS	flash4.omegaKIminus
#define OMEGA_KD_MINUS	flash4.omegaKDminus

//FF並進方向パラメータ
#define FF_ACCEL			flash4.ffAccel//0.102115f	//FF加速度項
#define FF_SPEED			flash4.ffSpeed//0.293453f	//FF速度項
#define FF_CONST			flash4.ffConst//0.16245f	//FF摩擦項

//FF回転方向パラメータ
#define FF_ANG_ACC			flash4.ffAngAcc//0.00155f	//FF角加速度項
#define FF_ANG_VEL			flash4.ffAngVel//0.021297f	//FF角速度項
#define FF_ANG_CONST		flash4.ffAngConst//0.12629f	//FF摩擦項？よくわからない

//FF並進方向パラメータ
#define FF_ACCEL2R			flash4.ffAccel2R//0.102115f	//FF加速度項
#define FF_SPEED2R			flash4.ffSpeed2R//0.293453f	//FF速度項
#define FF_CONST2R			flash4.ffConst2R//0.16245f	//FF摩擦項
#define FF_ACCEL2L			flash4.ffAccel2L//0.102115f	//FF加速度項
#define FF_SPEED2L			flash4.ffSpeed2L//0.293453f	//FF速度項
#define FF_CONST2L			flash4.ffConst2L//0.16245f	//FF摩擦項

//FF回転方向パラメータ
#define FF_ANG_ACC2RR			flash4.ffAngAcc2Rr//0.00155f	//FF角加速度項
#define FF_ANG_VEL2RR			flash4.ffAngVel2Rr//0.021297f	//FF角速度項
#define FF_ANG_CONST2RR		flash4.ffAngConst2Rr//0.12629f	//FF摩擦項？よくわからない
#define FF_ANG_ACC2RL			flash4.ffAngAcc2Rl//0.00155f	//FF角加速度項
#define FF_ANG_VEL2RL			flash4.ffAngVel2Rl//0.021297f	//FF角速度項
#define FF_ANG_CONST2RL		flash4.ffAngConst2Rl//0.12629f	//FF摩擦項？よくわからない

#define FF_ANG_ACC2LR			flash4.ffAngAcc2Lr//0.00155f	//FF角加速度項
#define FF_ANG_VEL2LR			flash4.ffAngVel2Lr//0.021297f	//FF角速度項
#define FF_ANG_CONST2LR		flash4.ffAngConst2Lr//0.12629f	//FF摩擦項？よくわからない
#define FF_ANG_ACC2LL			flash4.ffAngAcc2Ll//0.00155f	//FF角加速度項
#define FF_ANG_VEL2LL			flash4.ffAngVel2Ll//0.021297f	//FF角速度項
#define FF_ANG_CONST2LL		flash4.ffAngConst2Ll//0.12629f	//FF摩擦項？よくわからない

//#define DEG_KP		(0.0)//(10.0)

//#define DEG_KP_S		(0.0)//(15.0)
//#define DEG_KI_S		(0.0)

//run_mode
#define STRAIGHT_MODE	0		//直進時のモード
#define TURN_MODE		1		//超信地旋回時のモード
#define BACK_MODE		2		//バック時のモード
#define SLA_MODE		3		//スラロームモード(台形)
#define NON_CON_MODE	4		//非制御モード
#define TEST_MODE		5		//テストモード(割り込み用モータ制御を切るモード)
#define F_WALL_MODE		6		//前壁位置制御
#define PARTY_MODE		7		//宴会芸
#define OFFSET_A_MODE	8		//スラローム用オフセット直進
#define OFFSET_B_MODE	9		//スラローム用オフセット直進
#define TRACELOG_MODE	10		//ログ再生
#define SLA2_MODE		11		//スラロームモード(三角関数)
#define STRAIGHT2_MODE	12		//直進時のモード(三角関数)
#define SLA3_MODE		13		//スラロームモード(ネイピア)
#define VOLT_MODE		14		//電圧指定するだけ

//走行パラメータ
#define SEARCH_SPEED	(0.3f)				//探索走行の速度	[m/s]
#define SEARCH_ACCEL	(1.0f)				//探索走行の加速度	[m/s^2]
#define MIN_SPEED	(0.1f)				//最低速度	[m/s]

#define TURN_ACCEL	(PI*30.0f)//15				//超信地旋回の加速度	[rad/s^2]
#define	TURN_SPEED	(PI*1.5f)//*3.6f				//超信地旋回の最高速度	[rad/s]
#define	TURN_SPEED2	(PI*1.5f)				//超信地旋回の最高速度	[rad/s]
#define TURN_MIN_SPEED	(PI*0.4f)			//超信地旋回の最低速度	[rad/s]

#endif
