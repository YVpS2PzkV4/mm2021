#include"index.h"
#include"Interface.h"
#include"common.h"
#include"CMT.h"
#include"run.h"
#include"search.h"
#include"map.h"
#include"Dataflash.h"
#include "beeline.h"
#include"dijkstra.h"
#include<math.h>
//#include<mathf.h>

#define ARM_MATH_CM4
#include "arm_math.h"
#include "arm_const_structs.h"

t_edge exist[1024];

unsigned int slantingtask = 0;

volatile int goalhead = 0;
extern int mx;
extern int my;
extern volatile int head;

extern int goalwall;

extern unsigned short smap[MAZESIZE_X][MAZESIZE_Y];	//等高線map
extern unsigned char map[MAZESIZE_X][MAZESIZE_Y];
unsigned short saitanway[MAZESIZE_X*MAZESIZE_Y];
//unsigned short saitanway_n[MAZESIZE_X*MAZESIZE_Y];

extern volatile float Sensor_R_Dis;
extern volatile float Sensor_L_Dis;
extern volatile float Sensor_FR_Dis;
extern volatile float Sensor_FL_Dis;
extern unsigned volatile short Wall_R;
extern unsigned volatile short Wall_L;
extern unsigned volatile short Wall_FR;
extern unsigned volatile short Wall_FL;


extern volatile int			run_mode;

float saitandashv = 0;
float saitan90v = 0;
float saitanacc = 0;
float saitandec = 0;
float saitanacc_one = 0;	//一区画のみの直進時の加減速度
float saitandec_one = 0;
float saitanslantacc = 0;
float saitanslantdec = 0;
float saitanslantacc_one = 0;	//一区画のみの直進時の加減速度
float saitanslantdec_one = 0;
float saitanminv = 0;
float saitanslaacc = 0;
float saitanslav = 0;
float saitanbeforeslaoffset = 0;
float saitanafterslaoffset = 0;

float saitanoffset = 0;
float saitanbefore45offset = 0;
float saitanafter45offset = 0;
float saitanbeforeout45offset = 0;
float saitanafterout45offset = 0;
float saitanbeforebig90offset = 0;
float saitanafterbig90offset = 0;
float saitanbefore135offset = 0;
float saitanafter135offset = 0;
float saitanbeforeout135offset = 0;
float saitanafterout135offset = 0;
float saitanbefore180offset = 0;
float saitanafter180offset = 0;
float saitanbeforev90offset = 0;
float saitanafterv90offset = 0;
float saitanbeforekojimaoffset = 0;
float saitanafterkojimaoffset = 0;

float saitanfastacc = 0;
float saitanfastdec = 0;
float saitanfirstacc = 0;
float saitanstopdec = 0;
float saitan45v = 0;
float saitanout45v = 0;
float saitanbig90v = 0;
float saitan135v = 0;
float saitanout135v = 0;
float saitan180v = 0;
float saitanv90v = 0;
float saitankojimav = 0;
float saitandashslantingv = 0;
float saitanmaxv = 0;

float saitanbig90alpha = 0;
float saitanbig90w = 0;
float saitan45alpha = 0;
float saitan45w = 0;
float saitanout45alpha = 0;
float saitanout45w = 0;
float saitan135alpha = 0;
float saitan135w = 0;
float saitanout135alpha = 0;
float saitanout135w = 0;
float saitan180alpha = 0;
float saitan180w = 0;
float saitanv90alpha = 0;
float saitanv90w = 0;
float saitankojimaalpha = 0;
float saitankojimaw = 0;

float bigrightangleR = 0;
float big180angleR = 0;
float s135angleR = 0;
float sout135angleR = 0;
float s45angleR = 0;
float sout45angleR = 0;
float v90angleR = 0;
float kojimaangleR = 0;

float bigrightangleL = 0;
float big180angleL = 0;
float s135angleL = 0;
float sout135angleL = 0;
float s45angleL = 0;
float sout45angleL = 0;
float v90angleL = 0;
float kojimaangleL = 0;

extern unsigned char savetask;
extern volatile float			speed_r;				//現在の右タイヤ速度	[m/s]
extern volatile float			speed_l;				//現在の左タイヤ速度	[m/s]
extern volatile float			speed_old_r;				//右タイヤの過去の速度	[m/s]
extern volatile float			speed_new_r;				//右タイヤの最新の速度	[m/s]
extern volatile float			speed_old_l;				//左タイヤの過去の速度	[m/s]
extern volatile float			speed_new_l;				//左タイヤの最新の速度	[m/s]
extern volatile float			speed;					//現在車体速度		[m/s]
extern volatile float			p_speed;				//過去の車体速度	[m/s]
extern volatile float			len_mouse;
extern volatile float			I_speed;
extern volatile float			I_speed2;
extern volatile float			gyro_ref;
extern volatile float			p_ang_vel;
extern volatile float			ang_vel;
extern volatile float			I_ang_vel;
extern volatile float			degree;
extern volatile float			tar_ang_vel;				//目標角速度		[rad/s]
extern volatile float			tar_degree;				//目標角度		[deg]
extern volatile float			max_degree;				//旋回時の最大角度	[deg]
extern volatile float			start_degree;				//走行進入時の車体角度	[deg]
extern volatile float			max_ang_vel;				//最高角速度		[rad/s]
extern volatile float			ang_acc;				//角加速度		[rad/ss]
extern volatile float			accel;					//加速度		[m/ss]
extern volatile float			max_speed;				//最高速度		[m/s]
extern volatile float			Duty_r;
extern volatile float			Duty_l;
extern volatile float			V_r;
extern volatile float			V_l;
extern volatile float 			len_target;
extern volatile char 			TURN_DIR;
extern volatile int			run_mode;

extern volatile float			tar_speed;				//目標車体速度		[m/s]

//タイマ系グローバル変数
extern unsigned volatile  int		timer;					//1mSごとにカウントアップされる変数.

//制御用グローバル変数
extern volatile float			I_tar_speed;				//目標速度のI成分
extern volatile float			I_tar_ang_vel;				//目標角速度のI成分
extern volatile float 			end_speed;

extern volatile float			p_speed_r;				//過去の車体速度	[m/s]
extern volatile float			p_speed_l;				//過去の車体速度	[m/s]
extern volatile float 			tar_speed_r;
extern volatile float 			tar_speed_l;
extern  volatile int WallCtrlEnbl;
extern  volatile float WallCtrlErrP;
extern  volatile float WallCtrlErr;
extern  volatile int Sensor_R_Ctrl_Flag;
extern  volatile int Sensor_L_Ctrl_Flag;
extern  volatile float Sensor_R_Err;
extern  volatile float Sensor_L_Err;
extern  volatile float WallCtrlD;
extern  volatile float WallCtrlI;
extern  volatile float WallCtrlOmegaP;
extern  volatile float WallCtrlOmega;

extern volatile float Sensor_R;
extern volatile float Sensor_L;
extern volatile float Sensor_FR;
extern volatile float Sensor_FL;
extern volatile float Sensor_R_Dis;
extern volatile float Sensor_L_Dis;
extern volatile float Sensor_FR_Dis;
extern volatile float Sensor_FL_Dis;

extern volatile int slafwallflag;
extern volatile int slaswallflag;
extern volatile int firstread;

extern volatile char BatCheckSW;

volatile char saitaning = 0;
extern volatile float beecombr;
extern volatile float beecombl;
extern volatile float prev_combr;
extern volatile float prev_combl;

extern volatile float len_true;

extern volatile char turn_short;

volatile float next = 0;

extern volatile char path[1024];
extern volatile char dummy[1024];

volatile char lastslant = 0;
extern volatile char goalslantnum;
volatile float next2 = 0;
extern volatile char slantread;
volatile char slantblock = 0;

extern volatile char endx;	//ダイクストラ最短後の座標系
extern volatile char endy;
extern volatile char endh;

extern volatile int turnfwallflag;
extern volatile int slantfwallflag;
extern float searchmaxv;
extern float searchacc;

extern volatile char g_Sensor_Flag;

extern volatile float peekr;
extern volatile float nowpeekr;
extern volatile float peekl;
extern volatile float nowpeekl;

extern volatile char turn_read;	//壁なしターン用の何か
volatile float nextv = 0;

extern volatile char comb;

//volatile char exist.edge[256];

volatile char emode = 0;
volatile char rl = 0;

extern volatile char turnwflag;	//最短ターン前フラグ(前壁補正用)
extern volatile float offsetdis;

//volatile char yaba = 0;	//速いターン

extern volatile char sincurve;

extern volatile float beecombr_s;	//斜め制御（壁切れ距離差）系の何か
extern volatile float beecombl_s;
extern volatile int combbb_s;

extern volatile char off_flag;	//最短のターンオフセット中かどうか

extern volatile long			log_timer;				//ログ取りようのタイマ
extern volatile int			log_flag;				//ログ取得のタイミング

volatile char longst = 0;	//オフセット中か直線中か（加減速変えるため）

extern volatile char kaeri;
extern volatile char ttx;
extern volatile char tty;
extern volatile char tth;
extern volatile char lasth;

volatile char comb_onoff = 0;	//最短直進櫛制御かけるかどうか
//volatile char combsw[256];

//volatile char o45sp = 0;	//out45ターンだけsinじゃない+yabaじゃないようにしたいとき使おう

//extern volatile float beecombr;	//櫛制御（壁切れ距離差）系の何か
//extern volatile float beecombl;
extern volatile int combbb;

extern char goal_block_size;

extern volatile char naname_wallcut;

extern volatile char R_Ctrl_Disable;
extern volatile char L_Ctrl_Disable;	//壁切れ後しばらくトレースしない
extern volatile float Dis_to_Able;
extern volatile float Dis_to_Able2;	//壁切れ後しばらくトレースしない

extern volatile int V_bat;

extern volatile float FanVolt;	//吸引ファンの印加電圧
extern volatile float			Duty_fan;
extern volatile char FanEnbl;	//吸引ファンの印加電圧

extern volatile char flashLogFlag;
extern volatile char flash6_erased;

extern volatile char firstgo;

void saitan_shortest(int tx,int ty,char goorback,char hosu){
//	int j = 0;
//	int saitanstep = 0;	//区画ステップ数
	int pathnum = 0;
//	int temp1 = 0;
//	int slantingdis = 0;
	int wallcuton = 1;
	//short r = 0;
	volatile float dplus = 0;
	resetCurrent();
	g_Sensor_Flag = 1;
	firstread = 0;
	lastslant = 0;
	if(goorback != 0){
		goalslantnum = 0;
	}

	R_Ctrl_Disable = 0;
	L_Ctrl_Disable = 0;	//壁切れ後しばらくトレースしない
	Dis_to_Able = 0;
	Dis_to_Able2 = 0;	//壁切れ後しばらくトレースしない

	next2 = 0;
	nextv = 0;
	emode = 0;
	comb_onoff = 0;
	rl = 0;
	turnwflag = 0;
	offsetdis = 0;
	comb = 0;
	slantread = 0;
	slantblock = 0;
	slaswallflag = 0;
	slafwallflag = 0;
	turn_short = 0;
	turn_read = 0;
	peekr = 0;
	nowpeekr = 0;
	peekl = 0;
	nowpeekl = 0;
	longst = 0;
	turnfwallflag = 0;
	slantfwallflag = 0;
	naname_wallcut = 0;
	firstgo = 0;
	saitaning = 1;
	if(goorback != 0 && hosu == 0){
		map_to_maze();
		nodereset();
		LED4_ON();
		shortest(tx,ty);
		turnWallEdge();
		LED4_OFF();
	}else if((goorback != 0 && hosu == 1) || kaeri == 1){
		clearmap();
		map_copy();
		if(kaeri == 1){
		makesaitan(ttx,tty);
		if(lasth == 0){
			if(tth == 1){
				turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
			}
			else if(tth == 2){
				turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);
			}
			else if(tth == 3){
				turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
			}
		}
		else if(lasth == 1){
			if(tth == 0){
				turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
			}
			else if(tth == 2){
				turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
			}
			else if(tth == 3){
				turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);
			}
		}
		else if(lasth == 2){
			if(tth == 0){
				turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);
			}
			else if(tth == 1){
				turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
			}
			else if(tth == 3){
				turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
			}
		}
		else if(lasth == 3){
			if(tth == 0){
				turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
			}
			else if(tth == 1){
				turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);
			}
			else if(tth == 2){
				turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
			}
		}
		HAL_Delay(100);
		back(15,1.0,0.3,0);
		HAL_Delay(100);
		}
		else{
		makesaitan(tx,ty);
		}
		saitan_to_path();
		turnWallEdge();
	}

	if(goorback == 0 && kaeri == 0){
		if(saitandashv >= 2.0){
			saitandashv = 1.5;
		}
		if(saitandashslantingv >= 2.0){
			saitandashslantingv = 1.0;
		}
		if(saitanacc >= 5.0){
			saitanacc = 3.0;
		}
		if(saitandec >= 5.0){
			saitanacc = 3.0;
		}
		if(saitanslantacc >= 5.0){
			saitanslantacc = 3.0;
		}
		if(saitanslantdec >= 5.0){
			saitanslantacc = 3.0;
		}
		//for(r = 0; r < 256; r++){
		//	exist.edge[r] = 0;
		//}
	}

	if(goorback == 0){
		edge_reverse();
		path_reverse(hosu);
		if(path[0] >= DIA_GO1 && path[0] <= DIA_GO63){
			if((path[0]%2) ==0){//1,3,5....
				if(path[1] == OUT45R || path[1] == OUT135R || path[1] == V90R || path[1] == KOJIMAR){
					slantingtask = 1;
				}
				if(path[1] == OUT45L || path[1] == OUT135L || path[1] == V90L || path[1] == KOJIMAL){
					slantingtask = 2;
				}
			}
			else if((path[0]%2) ==1){
				if(path[1] == OUT45R || path[1] == OUT135R || path[1] == V90R || path[1] == KOJIMAR){
					slantingtask = 2;
				}
				if(path[1] == OUT45L || path[1] == OUT135L || path[1] == V90L || path[1] == KOJIMAL){
					slantingtask = 1;
				}
			}
		}
	/*	if(path[0] == DIA_GO2){
			if(path[1] == OUT45R || path[1] == OUT135R){
				slantingtask = 2;
			}
			if(path[1] == OUT45L || path[1] == OUT135L){
				slantingtask = 1;
			}
		}*/
	}
	//makesaitan(tx,ty);	//最短動作選択
	//get_gyro_ref();
	if(!(kaeri == 1 && FanVolt == 0)){
	HAL_Delay(STARTWAIT/2);	//少し待つ
	}

	FanEnbl = 1;

	if(!(kaeri == 1 && FanVolt == 0)){
	HAL_Delay(STARTWAIT/2);	//少し待つ
	}
	wallCtrlReset();
//	get_gyro_ref();
	BatCheckSW = 0;
	len_true = 0;
	next = 0;
	beecombr_s = 0;
	beecombl_s = 0;
	combbb_s = 0;
	beecombr = 0;
	beecombl = 0;
	prev_combr = 0;
	prev_combl = 0;
	combbb = 0;
	log_timer = 0;
	log_flag = 0;
	flashLogFlag = 1;
	while(1){
		turn_short = 0;
		next = 0;
		if(SWITCH_ONOFF() == SW_ON){	//非常用スイッチが押されたら緊急停止
			mx = 0;
			my = 0;
			head = 0;
			savetask = 0;
			run_mode = TEST_MODE;
			Motor_StopPWM();
			BatCheckSW = 1;
			saitaning = 0;
			HAL_Delay(1500);
			return;	//帰り対策
		}
		if(savetask == 1){
			mx = 0;
			my = 0;
			head = 0;
			while(SWITCH_ONOFF() == SW_OFF);
			run_mode = TEST_MODE;
			Motor_StopPWM();
			saitaning = 0;
			HAL_Delay(1500);
			savetask = 0;
			BatCheckSW = 1;
			return;
		}
		// saitanwayを元に移動する。
	//	j = saitanstep;
		if(path[pathnum] == SNODE){
			//go(ONESECTION,saitanacc,saitandec,saitanminv,saitandashv,speed,0,1);
			FanEnbl = 0;
			saitaning = 0;
			turnwflag = 0;
			turnfwallflag = 0;
			slantfwallflag = 0;
			log_flag = 0;
			flashLogFlag = 0;
			flash6_erased = 0;
			HAL_Delay(TURNWAIT);
			if(hosu == 0){
				if(goorback == 2){	//重ね探索用に向きを変える
					if(endh == 0){
						turn_fwall();
						mx = endx;
						my = endy;
						head = 2;
					}
					else if(endh == 1){
						turn_fwall();
						mx = endx;
						my = endy;
						head = 3;
					}
					else if(endh == 2){
						turn_fwall();
						mx = endx;
						my = endy;
						head = 0;
					}
					else if(endh == 3){
						turn_fwall();
						mx = endx;
						my = endy;
						head = 1;
					}
					else if(endh == 4){
						turn(45,TURN_ACCEL,TURN_SPEED2,RIGHT);
						mx = endx;
						my = endy+1;
						head = 2;
					}
					else if(endh == 5){
						turn(45,TURN_ACCEL,TURN_SPEED2,LEFT);
						mx = endx;
						my = endy+1;
						head = 2;
					}
					else if(endh == 6){
						turn(45,TURN_ACCEL,TURN_SPEED2,RIGHT);
						mx = endx;
						my = endy;
						head = 0;
					}
					else if(endh == 7){
						turn(45,TURN_ACCEL,TURN_SPEED2,LEFT);
						mx = endx;
						my = endy;
						head = 0;
					}
					else if(endh == 8){
						turn(45,TURN_ACCEL,TURN_SPEED2,LEFT);
						mx = endx;
						my = endy;
						head = 1;
					}
					else if(endh == 9){
						turn(45,TURN_ACCEL,TURN_SPEED2,RIGHT);
						mx = endx+1;
						my = endy;
						head = 3;
					}
					else if(endh == 10){
						turn(45,TURN_ACCEL,TURN_SPEED2,LEFT);
						mx = endx+1;
						my = endy;
						head = 3;
					}
					else if(endh == 11){
						turn(45,TURN_ACCEL,TURN_SPEED2,RIGHT);
						mx = endx;
						my = endy;
						head = 1;
					}
					if(endh >= 4){
						HAL_Delay(TURNWAIT);
						turnfwallflag = 1;
						straight(HALF_SECTION,1.5,0.35,0,FLAG_ON,0);
						turnfwallflag = 0;
						HAL_Delay(10);
						turn_fwall();
					}
				}
				else{
					if(slantingtask == 0){
						turn_fwall();
						if(goorback == 0){
							HAL_Delay(100);
							back(20,1.0,0.2,0);
						}
					}
					else{
						turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);
					}
				}
			}
			else{
				if(kaeri == 1){
					//LED2_ON();
					//turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);
					turn_fwall();
					HAL_Delay(100);
					back(20,1.0,0.2,0);
					//LED2_OFF();
				}
				else{
					turn_fwall();
				}
				mx = tx;
				my = ty;
				head = goalhead;
			}
			HAL_Delay(TURNWAIT);
			//mx = tx;
			//my = ty;
			//head = goalhead;
			run_mode = TEST_MODE;
			Motor_StopPWM();
			saitaning = 0;
			BatCheckSW = 1;
			turnfwallflag = 0;
			slantfwallflag = 0;
			emode = 0;
			comb_onoff = 0;
			beecombr = 0;
			beecombl = 0;
			prev_combr = 0;
			prev_combl = 0;
			combbb = 0;
			slantread = 0;
			emode = 0;
			rl = 0;
			next2 = 0;
			nextv = 0;
			turnwflag = 0;
			offsetdis = 0;
			comb = 0;
			slantblock = 0;
			slaswallflag = 0;
			slafwallflag = 0;
			turn_short = 0;
			turn_read = 0;
			peekr = 0;
			nowpeekr = 0;
			peekl = 0;
			nowpeekl = 0;
			longst = 0;
			//yaba = 0;
			R_Ctrl_Disable = 0;
			L_Ctrl_Disable = 0;	//壁切れ後しばらくトレースしない
			Dis_to_Able = 0;
			Dis_to_Able2 = 0;	//壁切れ後しばらくトレースしない
			return;	//普通なら0を返す
		}
		else if(path[pathnum] <= GO31){
			peekr = 0;
			nowpeekr = 0;
			peekl = 0;
			nowpeekl = 0;
			emode = exist[pathnum+1].edge;
			comb_onoff = exist[pathnum].combsw;
			beecombr = 0;
			beecombl = 0;
			prev_combr = 0;
			prev_combl = 0;
			combbb = 0;
			//emode = 0;
			//rl = 0;
			if(path[pathnum+1] == BIG90R || path[pathnum+1] == BIG180R || path[pathnum+1] == IN45R || path[pathnum+1] == IN135R){
				rl = RIGHT;
			}else if(path[pathnum+1] == BIG90L || path[pathnum+1] == BIG180L || path[pathnum+1] == IN45L || path[pathnum+1] == IN135L){
				rl = LEFT;
			}else{
				rl = 0;
			}
			longst = 1;

			if(pathnum == 0){
				firstgo = 1;
			}
			else{
				firstgo = 0;
			}

			if(path[pathnum] == GO1){
			if(pathnum == 0 && (kaeri == 1 ||goorback != 0)){
				if(path[pathnum+1] == SNODE){
					go((ONESECTION * path[pathnum]) + WALLTOMIDDLE,saitanacc_one,saitandec_one,saitanminv,saitandashv,0,0,1);
				}
				else if(path[pathnum+1] == BIG90R || path[pathnum+1] == BIG90L){
					go((ONESECTION * path[pathnum]) + WALLTOMIDDLE,saitanacc_one,saitandec_one,saitanminv,saitandashv,0,saitanbig90v,1);
				}
				else if(path[pathnum+1] == BIG180R || path[pathnum+1] == BIG180L){
					go((ONESECTION * path[pathnum]) + WALLTOMIDDLE,saitanacc_one,saitandec_one,saitanminv,saitandashv,0,saitan180v,1);
				}
				else if(path[pathnum+1] == IN45R || path[pathnum+1] == IN45L){
					go((ONESECTION * path[pathnum]) + WALLTOMIDDLE,saitanacc_one,saitandec_one,saitanminv,saitandashv,0,saitan45v,1);
				}
				else if(path[pathnum+1] == IN135R || path[pathnum+1] == IN135L){
					go((ONESECTION * path[pathnum]) + WALLTOMIDDLE,saitanacc_one,saitandec_one,saitanminv,saitandashv,0,saitan135v,1);
				}
			}
			else{
				if(path[pathnum+1] == SNODE){
					go((ONESECTION * path[pathnum]),saitanacc_one,saitandec_one,saitanminv,saitandashv,0,0,1);
				}
				else if(path[pathnum+1] == BIG90R || path[pathnum+1] == BIG90L){
					go((ONESECTION * path[pathnum]),saitanacc_one,saitandec_one,saitanminv,saitandashv,0,saitanbig90v,1);
				}
				else if(path[pathnum+1] == BIG180R || path[pathnum+1] == BIG180L){
					go((ONESECTION * path[pathnum]),saitanacc_one,saitandec_one,saitanminv,saitandashv,0,saitan180v,1);
				}
				else if(path[pathnum+1] == IN45R || path[pathnum+1] == IN45L){
					go((ONESECTION * path[pathnum]),saitanacc_one,saitandec_one,saitanminv,saitandashv,0,saitan45v,1);
				}
				else if(path[pathnum+1] == IN135R || path[pathnum+1] == IN135L){
					go((ONESECTION * path[pathnum]),saitanacc_one,saitandec_one,saitanminv,saitandashv,0,saitan135v,1);
				}
			}
			}
			else{
			if(pathnum == 0 && (kaeri == 1 ||goorback != 0)){
				if(path[pathnum+1] == SNODE){
					go((ONESECTION * path[pathnum]) + WALLTOMIDDLE,saitanacc,saitandec,saitanminv,saitandashv,0,0,1);
				}
				else if(path[pathnum+1] == BIG90R || path[pathnum+1] == BIG90L){
					go((ONESECTION * path[pathnum]) + WALLTOMIDDLE,saitanacc,saitandec,saitanminv,saitandashv,0,saitanbig90v,1);
				}
				else if(path[pathnum+1] == BIG180R || path[pathnum+1] == BIG180L){
					go((ONESECTION * path[pathnum]) + WALLTOMIDDLE,saitanacc,saitandec,saitanminv,saitandashv,0,saitan180v,1);
				}
				else if(path[pathnum+1] == IN45R || path[pathnum+1] == IN45L){
					go((ONESECTION * path[pathnum]) + WALLTOMIDDLE,saitanacc,saitandec,saitanminv,saitandashv,0,saitan45v,1);
				}
				else if(path[pathnum+1] == IN135R || path[pathnum+1] == IN135L){
					go((ONESECTION * path[pathnum]) + WALLTOMIDDLE,saitanacc,saitandec,saitanminv,saitandashv,0,saitan135v,1);
				}
			}
			else{
				if(path[pathnum+1] == SNODE){
					go((ONESECTION * path[pathnum]),saitanacc,saitandec,saitanminv,saitandashv,0,0,1);
				}
				else if(path[pathnum+1] == BIG90R || path[pathnum+1] == BIG90L){
					go((ONESECTION * path[pathnum]),saitanacc,saitandec,saitanminv,saitandashv,0,saitanbig90v,1);
				}
				else if(path[pathnum+1] == BIG180R || path[pathnum+1] == BIG180L){
					go((ONESECTION * path[pathnum]),saitanacc,saitandec,saitanminv,saitandashv,0,saitan180v,1);
				}
				else if(path[pathnum+1] == IN45R || path[pathnum+1] == IN45L){
					go((ONESECTION * path[pathnum]),saitanacc,saitandec,saitanminv,saitandashv,0,saitan45v,1);
				}
				else if(path[pathnum+1] == IN135R || path[pathnum+1] == IN135L){
					go((ONESECTION * path[pathnum]),saitanacc,saitandec,saitanminv,saitandashv,0,saitan135v,1);
				}
			}
			}
			longst = 0;
			comb_onoff = 0;
		}
		else if(path[pathnum] <= DIA_GO63){
				slantread = 0;
				emode = 0;
				rl = 0;
				longst = 1;
				naname_wallcut = 0;
				if((path[pathnum] % 2) == 0){//1,3,5...
					slantblock = 1;
				}
				else{
					slantblock = 0;
				}
				if(path[pathnum] == DIA_GO1){
				if(path[pathnum+1] == SNODE){
					lastslant = 1;
					go((ONESLANTING * (path[pathnum]- DIA_GO1 + 1)),saitanslantacc_one,saitanslantdec_one,saitanminv,saitandashslantingv,0,0,1);
					lastslant = 0;
				}
				else if(path[pathnum+1] == OUT45R || path[pathnum+1] == OUT45L){
					go((ONESLANTING * (path[pathnum]- DIA_GO1 + 1)),saitanslantacc_one,saitanslantdec_one,saitanminv,saitandashslantingv,0,saitanout45v,1);
				}
				else if(path[pathnum+1] == OUT135R || path[pathnum+1] == OUT135L){
					go((ONESLANTING * (path[pathnum]- DIA_GO1 + 1)),saitanslantacc_one,saitanslantdec_one,saitanminv,saitandashslantingv,0,saitanout135v,1);
				}
				else if(path[pathnum+1] == V90R || path[pathnum+1] == V90L){
					go((ONESLANTING * (path[pathnum]- DIA_GO1 + 1)),saitanslantacc_one,saitanslantdec_one,saitanminv,saitandashslantingv,0,saitanv90v,1);
				}
				else if(path[pathnum+1] == KOJIMAR || path[pathnum+1] == KOJIMAL){
					go((ONESLANTING * (path[pathnum]- DIA_GO1 + 1)),saitanslantacc_one,saitanslantdec_one,saitanminv,saitandashslantingv,0,saitankojimav,1);
				}
				}
				else{
				if(path[pathnum+1] == SNODE){
					lastslant = 1;
					go((ONESLANTING * (path[pathnum]- DIA_GO1 + 1)),saitanslantacc,saitanslantdec,saitanminv,saitandashslantingv,0,0,1);
					lastslant = 0;
				}
				else if(path[pathnum+1] == OUT45R || path[pathnum+1] == OUT45L){
					go((ONESLANTING * (path[pathnum]- DIA_GO1 + 1)),saitanslantacc,saitanslantdec,saitanminv,saitandashslantingv,0,saitanout45v,1);
				}
				else if(path[pathnum+1] == OUT135R || path[pathnum+1] == OUT135L){
					go((ONESLANTING * (path[pathnum]- DIA_GO1 + 1)),saitanslantacc,saitanslantdec,saitanminv,saitandashslantingv,0,saitanout135v,1);
				}
				else if(path[pathnum+1] == V90R || path[pathnum+1] == V90L){
					go((ONESLANTING * (path[pathnum]- DIA_GO1 + 1)),saitanslantacc,saitanslantdec,saitanminv,saitandashslantingv,0,saitanv90v,1);
				}
				else if(path[pathnum+1] == KOJIMAR || path[pathnum+1] == KOJIMAL){
					go((ONESLANTING * (path[pathnum]- DIA_GO1 + 1)),saitanslantacc,saitanslantdec,saitanminv,saitandashslantingv,0,saitankojimav,1);
				}
				}
				if((path[pathnum] % 2) == 0){//1,3,5...
					if(slantingtask == 1){
						slantingtask = 2;	//斜め→ターンの時　ターン前距離で壁切れを読むためにslantingtask切り替え 8/20
					}
					else if(slantingtask == 2){
						slantingtask = 1;
					}
				}
				else{
				}
				longst = 0;
		}
		else if(path[pathnum] == BIG90R){
			peekr = 0;
			nowpeekr = 0;
			peekl = 0;
			nowpeekl = 0;
			LED3_ON();
			next = saitanbeforebig90offset;
			nextv = saitanbig90v;
			emode = exist[pathnum].edge;
			rl = RIGHT;
			if(emode != 0){
			turnwflag = 1;
			}
			else{
			turnwflag = 0;
			}
			offsetdis = saitanbeforebig90offset;
			/*if((Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B && pathnum != 0 && (path[pathnum-1] == BIG90R || path[pathnum-1] == BIG90L || path[pathnum-1] == BIG180R || path[pathnum-1] == BIG180L || path[pathnum-1] == OUT45R || path[pathnum-1] == OUT45L || path[pathnum-1] == OUT135R || path[pathnum-1] == OUT135L) && turn_read == 0){
				waitForEdge(RIGHT);//壁切れ待ち
			}
			else */if(emode != 1 && turn_short == 0 && (Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B && Sensor_R_Dis < WALL_TH_R_SAI && SAITANWALLCUTB_R+saitanbeforebig90offset > 0 && wallcuton == 1 && (pathnum != 0)){
			//	control = 3;
				while(1){
					if(Sensor_R_Dis < WALL_TH_R_SAI){
						LED1_ON();
					}
					else{
						break;
					}
				}
				go(saitanbeforebig90offset + SAITANWALLCUTB_R,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanbig90v,3);
				LED1_OFF();
			}
			else if(emode == 1 && turn_short == 0 && (Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B && Sensor_R_Dis < WALL_TH_COMB_R_B && SAITANWALLCUTD_R+saitanbeforebig90offset > 0 && wallcuton == 1 && (pathnum != 0)){
				//	control = 3;
				while(1){
					if(Sensor_R_Dis < WALL_TH_COMB_R_B){
						LED1_ON();
					}
					else{
						break;
					}
				}
				go(saitanbeforebig90offset + SAITANWALLCUTD_R,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanbig90v,3);
				LED1_OFF();
			}
			else{
				if(pathnum == 0 && (kaeri == 1 ||goorback != 0)){
					if(turn_short == 0){
					go(saitanbeforebig90offset + WALLTOMIDDLE,saitanfirstacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanbig90v,3);
					}
					else{
					go(saitanbeforebig90offset + WALLTOMIDDLE-TURN_SHORT,saitanfirstacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanbig90v,3);
					}
				}
				else{
					if(turn_short == 0){
					go(saitanbeforebig90offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanbig90v,3);
					}
					else{
					go(saitanbeforebig90offset-TURN_SHORT,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanbig90v,3);
					}
				}
			}
			turnwflag = 0;
			offsetdis = 0;
			//sla(bigrightangle,saitanbig90alpha,saitanbig90w,LEFT);
			if(sincurve == 0){
			sla(bigrightangleR,saitanbig90alpha,saitanbig90w,saitanbig90v,RIGHT,0,0,(saitanbig90w*0.01));
			}
			else if(sincurve == 1){
			sla2(bigrightangleR,saitanbig90alpha,saitanbig90w,saitanbig90v,RIGHT,0,0,(saitanbig90w*0.01));
			}
			else{
			sla3(bigrightangleR,saitanbig90alpha,saitanbig90w,saitanbig90v,RIGHT,0,0);
			}
			turn_read  = 0;
			slantingtask = 0;
			LED3_OFF();
			next = 0;
			emode = exist[pathnum+1].edge;
			if(emode == 1){
				dplus = D_PLUS;
			}
			else{
				dplus = 0;
			}
			if(path[pathnum+1] == IN45R || path[pathnum+1] == IN135R || path[pathnum+1] == BIG90R || path[pathnum+1] == BIG180R){
				rl = RIGHT;
			}
			else if(path[pathnum+1] == IN45L || path[pathnum+1] == IN135L || path[pathnum+1] == BIG90L || path[pathnum+1] == BIG180L){
				rl = LEFT;
			}
			else{
				rl = 0;
			}
			if(path[pathnum+1] == IN45R || path[pathnum+1] == IN45L){
				go(saitanafterbig90offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan45v,3);
			}
			else if(path[pathnum+1] == BIG90R || path[pathnum+1] == BIG90L){
				go(saitanafterbig90offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanbig90v,3);
			}
			else if(path[pathnum+1] == IN135R || path[pathnum+1] == IN135L){
				go(saitanafterbig90offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan135v,3);
			}
			else if(path[pathnum+1] == BIG180R || path[pathnum+1] == BIG180L){
				go(saitanafterbig90offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan180v,3);
			}
			else if(path[pathnum+1] == SNODE){
				if(hosu == 0 && (goorback == 1 || goorback == 2)){
				//	if(goal_block_size == 1){
				//		go((saitanafterbig90offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
				//	}
				//	else if(goal_block_size == 4){
						go((ONESECTION + saitanafterbig90offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
				//	}
				//	else if(goal_block_size == 9){
				//		go((ONESECTION*2 + saitanafterbig90offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
				//	}
				}
				else{
					go((saitanafterbig90offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,1);
				}
			}
			else{
				go(saitanafterbig90offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,3);
			}
		}
		else if(path[pathnum] == BIG90L){
			peekr = 0;
			nowpeekr = 0;
			peekl = 0;
			nowpeekl = 0;
			LED5_ON();
			next = saitanbeforebig90offset;
			nextv = saitanbig90v;
			emode = exist[pathnum].edge;
			rl = LEFT;
			if(emode != 0){
			turnwflag = 1;
			}
			else{
			turnwflag = 0;
			}
			offsetdis = saitanbeforebig90offset;
			/*if((Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B && pathnum != 0 &&  (path[pathnum-1] == BIG90R || path[pathnum-1] == BIG90L || path[pathnum-1] == BIG180R || path[pathnum-1] == BIG180L || path[pathnum-1] == OUT45R || path[pathnum-1] == OUT45L || path[pathnum-1] == OUT135R || path[pathnum-1] == OUT135L) &&  turn_read == 0){
				waitForEdge(LEFT);//壁切れ待ち
			}
			else */if(emode != 1 && turn_short == 0 && (Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B && Sensor_L_Dis < WALL_TH_L_SAI && SAITANWALLCUTB_L+saitanbeforebig90offset > 0 && wallcuton == 1 && (pathnum != 0)){
			//	control = 3;
				while(1){
					if(Sensor_L_Dis < WALL_TH_L_SAI){
						LED2_ON();
					}
					else{
						break;
					}
				}
				go(saitanbeforebig90offset + SAITANWALLCUTB_L,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanbig90v,3);
				LED2_OFF();
			}
			else if(emode == 1 && turn_short == 0 && (Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B && Sensor_L_Dis < WALL_TH_COMB_L_B && SAITANWALLCUTD_L+saitanbeforebig90offset > 0 && wallcuton == 1 && (pathnum != 0)){
				//	control = 3;
				while(1){
					if(Sensor_L_Dis < WALL_TH_COMB_L_B){
						LED2_ON();
						}
					else{
						break;
					}
				}
				go(saitanbeforebig90offset + SAITANWALLCUTD_L,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanbig90v,3);
				LED2_OFF();
			}
			else{
				if(pathnum == 0 && (kaeri == 1 ||goorback != 0)){
					if(turn_short == 0){
					go(saitanbeforebig90offset + WALLTOMIDDLE,saitanfirstacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanbig90v,3);
					}
					else{
					go(saitanbeforebig90offset + WALLTOMIDDLE-TURN_SHORT,saitanfirstacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanbig90v,3);
					}
				}
				else{
					if(turn_short == 0){
					go(saitanbeforebig90offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanbig90v,3);
					}
					else{
					go(saitanbeforebig90offset-TURN_SHORT,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanbig90v,3);
					}
				}

			}
			turnwflag = 0;
			offsetdis = 0;
			//sla(bigrightangle,saitanbig90alpha,saitanbig90w,LEFT);
			if(sincurve == 0){
			sla(bigrightangleL,saitanbig90alpha,saitanbig90w,saitanbig90v,LEFT,0,0,(saitanbig90w*0.01));
			}
			else if(sincurve == 1){
			sla2(bigrightangleL,saitanbig90alpha,saitanbig90w,saitanbig90v,LEFT,0,0,(saitanbig90w*0.01));
			}
			else{
			sla3(bigrightangleL,saitanbig90alpha,saitanbig90w,saitanbig90v,LEFT,0,0);
			}
			turn_read  = 0;
			slantingtask = 0;
			LED5_OFF();
			next = 0;
			emode = exist[pathnum+1].edge;
			if(emode == 1){
				dplus = D_PLUS;
			}
			else{
				dplus = 0;
			}
			if(path[pathnum+1] == IN45R || path[pathnum+1] == IN135R || path[pathnum+1] == BIG90R || path[pathnum+1] == BIG180R){
				rl = RIGHT;
			}
			else if(path[pathnum+1] == IN45L || path[pathnum+1] == IN135L || path[pathnum+1] == BIG90L || path[pathnum+1] == BIG180L){
				rl = LEFT;
			}
			else{
				rl = 0;
			}
			if(path[pathnum+1] == IN45R || path[pathnum+1] == IN45L){
				go(saitanafterbig90offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan45v,3);
			}
			else if(path[pathnum+1] == BIG90R || path[pathnum+1] == BIG90L){
				go(saitanafterbig90offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanbig90v,3);
			}
			else if(path[pathnum+1] == IN135R || path[pathnum+1] == IN135L){
				go(saitanafterbig90offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan135v,3);
			}
			else if(path[pathnum+1] == BIG180R || path[pathnum+1] == BIG180L){
				go(saitanafterbig90offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan180v,3);
			}
			else if(path[pathnum+1] == SNODE){
				if(hosu == 0 && (goorback == 1 || goorback == 2)){
				//	if(goal_block_size == 1){
				//		go((saitanafterbig90offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
				//	}
				//	else if(goal_block_size == 4){
						go((ONESECTION + saitanafterbig90offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
				//	}
				//	else if(goal_block_size == 9){
				//		go((ONESECTION*2 + saitanafterbig90offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
				//	}
				}
				else{
					go((saitanafterbig90offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,1);
				}
			}
			else{
				go(saitanafterbig90offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,3);
			}
		}
		else if(path[pathnum] == BIG180R){
			peekr = 0;
			nowpeekr = 0;
			peekl = 0;
			nowpeekl = 0;
			LED3_ON();
			next = saitanbefore180offset;
			nextv = saitan180v;
			emode = exist[pathnum].edge;
			rl = RIGHT;
			if(emode != 0){
			turnwflag = 1;
			}
			else{
			turnwflag = 0;
			}
			offsetdis = saitanbefore180offset;
			/*if((Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B && pathnum != 0 && (path[pathnum-1] == BIG90R || path[pathnum-1] == BIG90L || path[pathnum-1] == BIG180R || path[pathnum-1] == BIG180L || path[pathnum-1] == OUT45R || path[pathnum-1] == OUT45L || path[pathnum-1] == OUT135R || path[pathnum-1] == OUT135L) &&  turn_read == 0){
				waitForEdge(RIGHT);//壁切れ待ち
			}
			else */if(emode != 1 && turn_short == 0 && (Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B && Sensor_R_Dis < WALL_TH_R_SAI && SAITANWALLCUTB_R+saitanbefore180offset > 0 && wallcuton == 1 && (pathnum != 0)){
			//	control = 3;
				while(1){
					if(Sensor_R_Dis < WALL_TH_R_SAI){
						LED1_ON();
					}
					else{
						break;
					}
				}
				go(saitanbefore180offset + SAITANWALLCUTB_R,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan180v,3);
				LED1_OFF();
			}
			else if(emode == 1 && turn_short == 0 && (Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B && Sensor_R_Dis < WALL_TH_COMB_R_B && SAITANWALLCUTD_R+saitanbefore180offset > 0 && wallcuton == 1 && (pathnum != 0)){
				//	control = 3;
				while(1){
					if(Sensor_R_Dis < WALL_TH_COMB_R_B){
						LED1_ON();
					}
					else{
						break;
					}
				}
				go(saitanbefore180offset + SAITANWALLCUTD_R,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan180v,3);
				LED1_OFF();
			}
			else{
				if(pathnum == 0 && (kaeri == 1 ||goorback != 0)){
					if(turn_short == 0){
					go(saitanbefore180offset + WALLTOMIDDLE,saitanfirstacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan180v,3);
					}
					else{
					go(saitanbefore180offset + WALLTOMIDDLE-TURN_SHORT,saitanfirstacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan180v,3);
					}
				}
				else{
					if(turn_short == 0){
					go(saitanbefore180offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan180v,3);
					}
					else{
					go(saitanbefore180offset-TURN_SHORT,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan180v,3);
					}
				}

			}
			turnwflag = 0;
			offsetdis = 0;
			//sla(bigrightangle,saitanbig90alpha,saitanbig90w,LEFT);
			if(sincurve == 0){
			sla(big180angleR,saitan180alpha,saitan180w,saitan180v,RIGHT,0,0,(saitan180w*0.01));
			}
			else if(sincurve == 1){
			sla2(big180angleR,saitan180alpha,saitan180w,saitan180v,RIGHT,0,0,(saitan180w*0.01));
			}
			else{
			sla3(big180angleR,saitan180alpha,saitan180w,saitan180v,RIGHT,0,0);
			}
			turn_read  = 0;
			slantingtask = 0;
			LED3_OFF();
			next = 0;
			emode = exist[pathnum+1].edge;
			if(emode == 1){
				dplus = D_PLUS;
			}
			else{
				dplus = 0;
			}
			if(path[pathnum+1] == IN45R || path[pathnum+1] == IN135R || path[pathnum+1] == BIG90R || path[pathnum+1] == BIG180R){
				rl = RIGHT;
			}
			else if(path[pathnum+1] == IN45L || path[pathnum+1] == IN135L || path[pathnum+1] == BIG90L || path[pathnum+1] == BIG180L){
				rl = LEFT;
			}
			else{
				rl = 0;
			}
			if(path[pathnum+1] == IN45R || path[pathnum+1] == IN45L){
				go(saitanafter180offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan45v,3);
			}
			else if(path[pathnum+1] == BIG90R || path[pathnum+1] == BIG90L){
				go(saitanafter180offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanbig90v,3);
			}
			else if(path[pathnum+1] == IN135R || path[pathnum+1] == IN135L){
				go(saitanafter180offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan135v,3);
			}
			else if(path[pathnum+1] == BIG180R || path[pathnum+1] == BIG180L){
				go(saitanafter180offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan180v,3);
			}
			else if(path[pathnum+1] == SNODE){
				if(hosu == 0 && (goorback == 1 || goorback == 2)){
				//	if(goal_block_size == 1){
				//		go((saitanafter180offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
				//	}
				//	else if(goal_block_size == 4){
						go((ONESECTION + saitanafter180offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
				//	}
				//	else if(goal_block_size == 9){
				//		go((ONESECTION*2 + saitanafter180offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
				//	}
				}
				else{
					go((saitanafter180offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,1);
				}
			}
			else{
				go(saitanafter180offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,3);
			}
		}
		else if(path[pathnum] == BIG180L){
			peekr = 0;
			nowpeekr = 0;
			peekl = 0;
			nowpeekl = 0;
			LED5_ON();
			next = saitanbefore180offset;
			nextv = saitan180v;
			emode = exist[pathnum].edge;
			rl = LEFT;
			if(emode != 0){
			turnwflag = 1;
			}
			else{
			turnwflag = 0;
			}
			offsetdis = saitanbefore180offset;
			/*if((Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B && pathnum != 0 && (path[pathnum-1] == BIG90R || path[pathnum-1] == BIG90L || path[pathnum-1] == BIG180R || path[pathnum-1] == BIG180L || path[pathnum-1] == OUT45R || path[pathnum-1] == OUT45L || path[pathnum-1] == OUT135R || path[pathnum-1] == OUT135L) &&  turn_read == 0){
				waitForEdge(LEFT);//壁切れ待ち
			}
			else */if(emode != 1 && turn_short == 0 && (Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B && Sensor_L_Dis < WALL_TH_L_SAI && SAITANWALLCUTB_L+saitanbefore180offset > 0 && wallcuton == 1 && (pathnum != 0)){
			//	control = 3;
				while(1){
					if(Sensor_L_Dis < WALL_TH_L_SAI){
						LED2_ON();
					}
					else{
						break;
					}
				}
				go(saitanbefore180offset + SAITANWALLCUTB_L,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan180v,3);
				LED2_OFF();
			}
			else if(emode == 1 && turn_short == 0 && (Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B && Sensor_L_Dis < WALL_TH_COMB_L_B && SAITANWALLCUTD_L+saitanbefore180offset > 0 && wallcuton == 1 && (pathnum != 0)){
				//	control = 3;
					while(1){
						if(Sensor_L_Dis < WALL_TH_COMB_L_B){
							LED2_ON();
						}
						else{
							break;
						}
					}
					go(saitanbefore180offset + SAITANWALLCUTD_L,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan180v,3);
					LED2_OFF();
				}
			else{
				if(pathnum == 0 && (kaeri == 1 ||goorback != 0)){
					if(turn_short == 0){
					go(saitanbefore180offset + WALLTOMIDDLE,saitanfirstacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan180v,3);
					}
					else{
					go(saitanbefore180offset + WALLTOMIDDLE-TURN_SHORT,saitanfirstacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan180v,3);
					}
				}
				else{
					if(turn_short == 0){
					go(saitanbefore180offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan180v,3);
					}
					else{
					go(saitanbefore180offset-TURN_SHORT,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan180v,3);
					}
				}

			}
			turnwflag = 0;
			offsetdis = 0;
			//sla(bigrightangle,saitanbig90alpha,saitanbig90w,LEFT);
			if(sincurve == 0){
			sla(big180angleL,saitan180alpha,saitan180w,saitan180v,LEFT,0,0,(saitan180w*0.01));
			}
			else if(sincurve == 1){
			sla2(big180angleL,saitan180alpha,saitan180w,saitan180v,LEFT,0,0,(saitan180w*0.01));
			}
			else{
			sla3(big180angleL,saitan180alpha,saitan180w,saitan180v,LEFT,0,0);
			}
			turn_read  = 0;
			slantingtask = 0;
			LED5_OFF();
			next = 0;
			emode = exist[pathnum+1].edge;
			if(emode == 1){
				dplus = D_PLUS;
			}
			else{
				dplus = 0;
			}
			if(path[pathnum+1] == IN45R || path[pathnum+1] == IN135R || path[pathnum+1] == BIG90R || path[pathnum+1] == BIG180R){
				rl = RIGHT;
			}
			else if(path[pathnum+1] == IN45L || path[pathnum+1] == IN135L || path[pathnum+1] == BIG90L || path[pathnum+1] == BIG180L){
				rl = LEFT;
			}
			else{
				rl = 0;
			}
			if(path[pathnum+1] == IN45R || path[pathnum+1] == IN45L){
				go(saitanafter180offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan45v,3);
			}
			else if(path[pathnum+1] == BIG90R || path[pathnum+1] == BIG90L){
				go(saitanafter180offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanbig90v,3);
			}
			else if(path[pathnum+1] == IN135R || path[pathnum+1] == IN135L){
				go(saitanafter180offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan135v,3);
			}
			else if(path[pathnum+1] == BIG180R || path[pathnum+1] == BIG180L){
				go(saitanafter180offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan180v,3);
			}
			else if(path[pathnum+1] == SNODE){
				if(hosu == 0 && (goorback == 1 || goorback == 2)){
				//	if(goal_block_size == 1){
				//		go((saitanafter180offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
				//	}
				//	else if(goal_block_size == 4){
						go((ONESECTION + saitanafter180offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
				//	}
				//	else if(goal_block_size == 9){
				//		go((ONESECTION*2 + saitanafter180offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
				//	}
				}
				else{
					go((saitanafter180offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,1);
				}
			}
			else{
				go(saitanafter180offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,3);
			}
		}
		else if(path[pathnum] == IN45R){
			peekr = 0;
			nowpeekr = 0;
			peekl = 0;
			nowpeekl = 0;
			LED3_ON();
			next = saitanbefore45offset;
			nextv = saitan45v;
			emode = exist[pathnum].edge;
			rl = RIGHT;
			if(emode != 0){
			turnwflag = 1;
			}
			else{
			turnwflag = 0;
			}
			offsetdis = saitanbefore45offset;
			/*if((Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B && pathnum != 0 && (path[pathnum-1] == BIG90R || path[pathnum-1] == BIG90L || path[pathnum-1] == BIG180R || path[pathnum-1] == BIG180L || path[pathnum-1] == OUT45R || path[pathnum-1] == OUT45L || path[pathnum-1] == OUT135R || path[pathnum-1] == OUT135L) &&  turn_read == 0){
				waitForEdge(RIGHT);//壁切れ待ち
			}
			else */if(emode != 1 && turn_short == 0 && (Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B && Sensor_R_Dis < WALL_TH_R_SAI && SAITANWALLCUTB_R +saitanbefore45offset > 0 && wallcuton == 1 && (pathnum != 0)){
			//	control = 3;
				while(1){
					if(Sensor_R_Dis < WALL_TH_R_SAI){
						LED1_ON();
					}
					else{
						break;
					}
				}
				go(saitanbefore45offset + SAITANWALLCUTB_R,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan45v,3);
				LED1_OFF();
			}
			else if(emode == 1 && turn_short == 0 && (Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B && Sensor_R_Dis < WALL_TH_COMB_R_B && SAITANWALLCUTD_R +saitanbefore45offset > 0 && wallcuton == 1 && (pathnum != 0)){
				//	control = 3;
					while(1){
						if(Sensor_R_Dis < WALL_TH_COMB_R_B){
							LED1_ON();
						}
						else{
							break;
						}
					}
					go(saitanbefore45offset + SAITANWALLCUTD_R,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan45v,3);
					LED1_OFF();
				}
			else{
				if(pathnum == 0 && (kaeri == 1 ||goorback != 0)){
					if(turn_short == 0){
					go(saitanbefore45offset + WALLTOMIDDLE,saitanfirstacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan45v,3);
					}
					else{
					go(saitanbefore45offset + WALLTOMIDDLE-TURN_SHORT,saitanfirstacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan45v,3);
					}
				}
				else{
					if(turn_short == 0){
					go(saitanbefore45offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan45v,3);
					}
					else{
					go(saitanbefore45offset-TURN_SHORT,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan45v,3);
					}
				}

			}
			turnwflag = 0;
			offsetdis = 0;
			//sla(bigrightangle,saitanbig90alpha,saitanbig90w,LEFT);
			if(sincurve == 0){
			sla(s45angleR,saitan45alpha,saitan45w,saitan45v,RIGHT,0,0,(saitan45w*0.01));
			}
			else if(sincurve == 1){
			sla2(s45angleR,saitan45alpha,saitan45w,saitan45v,RIGHT,0,0,(saitan45w*0.01));
			}
			else{
			sla3(s45angleR,saitan45alpha,saitan45w,saitan45v,RIGHT,0,0);
			}
			naname_wallcut = 0;
			turn_read  = 0;
			slantread = 0;
			slantingtask = 1;
			LED3_OFF();
			next = 0;
			beecombr_s = 0;
			beecombl_s = 0;
			combbb_s = 0;
			if(path[pathnum+1] <= DIA_GO63){
				go(saitanafter45offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,6);
			}
			else if(path[pathnum+1] == OUT45R || path[pathnum+1] == OUT45L){
				go(saitanafter45offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanout45v,6);
			}
			else if(path[pathnum+1] == OUT135R || path[pathnum+1] == OUT135L){
				go(saitanafter45offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanout135v,6);
			}
			else if(path[pathnum+1] == V90R || path[pathnum+1] == V90L){
				go(saitanafter45offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanv90v,6);
			}
			else if(path[pathnum+1] == KOJIMAR || path[pathnum+1] == KOJIMAL){
				go(saitanafter45offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitankojimav,6);
			}
			else if(path[pathnum+1] == SNODE){
			//	lastslant = 1;
				go((ONESLANTING*goalslantnum + saitanafter45offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
			//	lastslant = 0;
			}
			else{
				go(saitanafter45offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,6);
			}
		}
		else if(path[pathnum] == IN45L){
			peekr = 0;
			nowpeekr = 0;
			peekl = 0;
			nowpeekl = 0;
			LED5_ON();
			next = saitanbefore45offset;
			nextv = saitan45v;
			emode = exist[pathnum].edge;
			rl = LEFT;
			if(emode != 0){
			turnwflag = 1;
			}
			else{
			turnwflag = 0;
			}
			offsetdis = saitanbefore45offset;
			/*if((Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B && pathnum != 0 && (path[pathnum-1] == BIG90R || path[pathnum-1] == BIG90L || path[pathnum-1] == BIG180R || path[pathnum-1] == BIG180L || path[pathnum-1] == OUT45R || path[pathnum-1] == OUT45L || path[pathnum-1] == OUT135R || path[pathnum-1] == OUT135L) &&  turn_read == 0){
				waitForEdge(LEFT);//壁切れ待ち
			}
			else */if(emode != 1 && turn_short == 0 && (Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B && Sensor_L_Dis < WALL_TH_L_SAI && SAITANWALLCUTB_L+saitanbefore45offset > 0 && wallcuton == 1 && (pathnum != 0)){
			//	control = 3;
				while(1){
					if(Sensor_L_Dis < WALL_TH_L_SAI){
						LED2_ON();
					}
					else{
						break;
					}
				}
				go(saitanbefore45offset + SAITANWALLCUTB_L,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan45v,3);
				LED2_OFF();
			}
			else if(emode == 1 && turn_short == 0 && (Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B && Sensor_L_Dis < WALL_TH_COMB_L_B && SAITANWALLCUTD_L+saitanbefore45offset > 0 && wallcuton == 1 && (pathnum != 0)){
				//	control = 3;
					while(1){
						if(Sensor_L_Dis < WALL_TH_COMB_L_B){
							LED2_ON();
						}
						else{
							break;
						}
					}
					go(saitanbefore45offset + SAITANWALLCUTD_L,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan45v,3);
					LED2_OFF();
				}
			else{
				if(pathnum == 0 && (kaeri == 1 ||goorback != 0)){
					if(turn_short == 0){
					go(saitanbefore45offset + WALLTOMIDDLE,saitanfirstacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan45v,3);
					}
					else{
					go(saitanbefore45offset + WALLTOMIDDLE-TURN_SHORT,saitanfirstacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan45v,3);
					}
				}
				else{
					if(turn_short == 0){
					go(saitanbefore45offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan45v,3);
					}
					else{
					go(saitanbefore45offset-TURN_SHORT,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan45v,3);
					}
				}

			}
			turnwflag = 0;
			offsetdis = 0;
			//sla(bigrightangle,saitanbig90alpha,saitanbig90w,LEFT);
			if(sincurve == 0){
			sla(s45angleL,saitan45alpha,saitan45w,saitan45v,LEFT,0,0,(saitan45w*0.01));
			}
			else if(sincurve == 1){
			sla2(s45angleL,saitan45alpha,saitan45w,saitan45v,LEFT,0,0,(saitan45w*0.01));
			}
			else{
			sla3(s45angleL,saitan45alpha,saitan45w,saitan45v,LEFT,0,0);
			}
			naname_wallcut = 0;
			turn_read  = 0;
			slantread = 0;
			slantingtask = 2;
			LED5_OFF();
			next = 0;
			beecombr_s = 0;
			beecombl_s = 0;
			combbb_s = 0;
			if(path[pathnum+1] <= DIA_GO63){
				go(saitanafter45offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,6);
			}
			else if(path[pathnum+1] == OUT45R || path[pathnum+1] == OUT45L){
				go(saitanafter45offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanout45v,6);
			}
			else if(path[pathnum+1] == OUT135R || path[pathnum+1] == OUT135L){
				go(saitanafter45offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanout135v,6);
			}
			else if(path[pathnum+1] == V90R || path[pathnum+1] == V90L){
				go(saitanafter45offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanv90v,6);
			}
			else if(path[pathnum+1] == KOJIMAR || path[pathnum+1] == KOJIMAL){
				go(saitanafter45offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitankojimav,6);
			}
			else if(path[pathnum+1] == SNODE){
			//	lastslant = 1;
				go((ONESLANTING*goalslantnum + saitanafter45offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
			//	lastslant = 0;
			}
			else{
				go(saitanafter45offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,6);
			}
		}
		else if(path[pathnum] == IN135R){
			peekr = 0;
			nowpeekr = 0;
			peekl = 0;
			nowpeekl = 0;
			LED3_ON();
			next = saitanbefore135offset;
			nextv = saitan135v;
			emode = exist[pathnum].edge;
			rl = RIGHT;
			if(emode != 0){
			turnwflag = 1;
			}
			else{
			turnwflag = 0;
			}
			offsetdis = saitanbefore135offset;
			/*if((Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B && pathnum != 0 && (path[pathnum-1] == BIG90R || path[pathnum-1] == BIG90L || path[pathnum-1] == BIG180R || path[pathnum-1] == BIG180L || path[pathnum-1] == OUT45R || path[pathnum-1] == OUT45L || path[pathnum-1] == OUT135R || path[pathnum-1] == OUT135L) &&  turn_read == 0){
				waitForEdge(RIGHT);//壁切れ待ち
			}
			else */if(emode != 1 && turn_short == 0 && (Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B && Sensor_R_Dis < WALL_TH_R_SAI && SAITANWALLCUTB_R+saitanbefore135offset > 0 && wallcuton == 1 && (pathnum != 0)){
			//	control = 3;
				while(1){
					if(Sensor_R_Dis < WALL_TH_R_SAI){
						LED1_ON();
					}
					else{
						break;
					}
				}
				go(saitanbefore135offset + SAITANWALLCUTB_R,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan135v,3);
				LED1_OFF();
			}
			else if(emode == 1 && turn_short == 0 && (Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B && Sensor_R_Dis < WALL_TH_COMB_R_B && SAITANWALLCUTD_R+saitanbefore135offset > 0 && wallcuton == 1 && (pathnum != 0)){
				//	control = 3;
					while(1){
						if(Sensor_R_Dis < WALL_TH_COMB_R_B){
							LED1_ON();
						}
						else{
							break;
						}
					}
					go(saitanbefore135offset + SAITANWALLCUTD_R,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan135v,3);
					LED1_OFF();
				}
			else{
				if(pathnum == 0 && (kaeri == 1 ||goorback != 0)){
					if(turn_short == 0){
					go(saitanbefore135offset + WALLTOMIDDLE,saitanfirstacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan135v,3);
					}
					else{
					go(saitanbefore135offset + WALLTOMIDDLE-TURN_SHORT,saitanfirstacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan135v,3);
					}
				}
				else{
					if(turn_short == 0){
					go(saitanbefore135offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan135v,3);
					}
					else{
					go(saitanbefore135offset-TURN_SHORT,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan135v,3);
					}
				}

			}
			turnwflag = 0;
			offsetdis = 0;
			//sla(bigrightangle,saitanbig90alpha,saitanbig90w,LEFT);
			if(sincurve == 0){
			sla(s135angleR,saitan135alpha,saitan135w,saitan135v,RIGHT,0,0,(saitan135w*0.01));
			}
			else if(sincurve == 1){
			sla2(s135angleR,saitan135alpha,saitan135w,saitan135v,RIGHT,0,0,(saitan135w*0.01));
			}
			else{
			sla3(s135angleR,saitan135alpha,saitan135w,saitan135v,RIGHT,0,0);
			}
			naname_wallcut = 0;
			turn_read  = 0;
			slantread = 0;
			slantingtask = 1;
			LED3_OFF();
			next = 0;
			beecombr_s = 0;
			beecombl_s = 0;
			combbb_s = 0;
			if(path[pathnum+1] <= DIA_GO63){
				go(saitanafter135offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,6);
			}
			else if(path[pathnum+1] == OUT45R || path[pathnum+1] == OUT45L){
				go(saitanafter135offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanout45v,6);
			}
			else if(path[pathnum+1] == OUT135R || path[pathnum+1] == OUT135L){
				go(saitanafter135offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanout135v,6);
			}
			else if(path[pathnum+1] == V90R || path[pathnum+1] == V90L){
				go(saitanafter135offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanv90v,6);
			}
			else if(path[pathnum+1] == KOJIMAR || path[pathnum+1] == KOJIMAL){
				go(saitanafter135offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitankojimav,6);
			}
			else if(path[pathnum+1] == SNODE){
			//	lastslant = 1;
				go((ONESLANTING*goalslantnum + saitanafter135offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
			//	lastslant = 0;
			}
			else{
				go(saitanafter135offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,6);
			}
		}
		else if(path[pathnum] == IN135L){
			peekr = 0;
			nowpeekr = 0;
			peekl = 0;
			nowpeekl = 0;
			LED5_ON();
			next = saitanbefore135offset;
			nextv = saitan135v;
			emode = exist[pathnum].edge;
			rl = LEFT;
			if(emode != 0){
			turnwflag = 1;
			}
			else{
			turnwflag = 0;
			}
			offsetdis = saitanbefore135offset;
			if(emode != 1 && turn_short == 0 && (Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B && Sensor_L_Dis < WALL_TH_L_SAI && SAITANWALLCUTB_L+saitanbefore135offset > 0 && wallcuton == 1 && (pathnum != 0)){
			//	control = 3;
				while(1){
					if(Sensor_L_Dis < WALL_TH_L_SAI){
						LED2_ON();
					}
					else{
						break;
					}
				}
				go(saitanbefore135offset + SAITANWALLCUTB_L,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan135v,3);
				LED2_OFF();
			}
			else if(emode == 1 && turn_short == 0 && (Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B && Sensor_L_Dis < WALL_TH_COMB_L_B && SAITANWALLCUTD_L+saitanbefore135offset > 0 && wallcuton == 1 && (pathnum != 0)){
				//	control = 3;
					while(1){
						if(Sensor_L_Dis < WALL_TH_COMB_L_B){
							LED2_ON();
						}
						else{
							break;
						}
					}
					go(saitanbefore135offset + SAITANWALLCUTD_L,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan135v,3);
					LED2_OFF();
				}
			else{
				if(pathnum == 0 && (kaeri == 1 ||goorback != 0)){
					if(turn_short == 0){
					go(saitanbefore135offset + WALLTOMIDDLE,saitanfirstacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan135v,3);
					}
					else{
					go(saitanbefore135offset + WALLTOMIDDLE-TURN_SHORT,saitanfirstacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan135v,3);
					}
				}
				else{
					if(turn_short == 0){
					go(saitanbefore135offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan135v,3);
					}
					else{
					go(saitanbefore135offset-TURN_SHORT,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan135v,3);
					}
				}
				/*if((Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B && pathnum != 0 && (path[pathnum-1] == BIG90R || path[pathnum-1] == BIG90L || path[pathnum-1] == BIG180R || path[pathnum-1] == BIG180L || path[pathnum-1] == OUT45R || path[pathnum-1] == OUT45L || path[pathnum-1] == OUT135R || path[pathnum-1] == OUT135L) &&  turn_read == 0){
					waitForEdge(LEFT);//壁切れ待ち
				}*/
			}
			turnwflag = 0;
			offsetdis = 0;
			//sla(bigrightangle,saitanbig90alpha,saitanbig90w,LEFT);
			if(sincurve == 0){
			sla(s135angleL,saitan135alpha,saitan135w,saitan135v,LEFT,0,0,(saitan135w*0.01));
			}
			else if(sincurve == 1){
			sla2(s135angleL,saitan135alpha,saitan135w,saitan135v,LEFT,0,0,(saitan135w*0.01));
			}
			else{
			sla3(s135angleL,saitan135alpha,saitan135w,saitan135v,LEFT,0,0);
			}
			naname_wallcut = 0;
			turn_read  = 0;
			slantread = 0;
			slantingtask = 2;
			LED5_OFF();
			next = 0;
			beecombr_s = 0;
			beecombl_s = 0;
			combbb_s = 0;
			if(path[pathnum+1] <= DIA_GO63){
				go(saitanafter135offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,6);
			}
			else if(path[pathnum+1] == OUT45R || path[pathnum+1] == OUT45L){
				go(saitanafter135offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanout45v,6);
			}
			else if(path[pathnum+1] == OUT135R || path[pathnum+1] == OUT135L){
				go(saitanafter135offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanout135v,6);
			}
			else if(path[pathnum+1] == V90R || path[pathnum+1] == V90L){
				go(saitanafter135offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanv90v,6);
			}
			else if(path[pathnum+1] == KOJIMAR || path[pathnum+1] == KOJIMAL){
				go(saitanafter135offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitankojimav,6);
			}
			else if(path[pathnum+1] == SNODE){
			//	lastslant = 1;
				go((ONESLANTING*goalslantnum + saitanafter135offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
			//	lastslant = 0;
			}
			else{
				go(saitanafter135offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,6);
			}
		}
		else if(path[pathnum] == OUT45R){
			LED3_ON();
			//next = saitanbeforeout45offset;
			next2 = saitanbeforeout45offset;
			go(saitanbeforeout45offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanout45v,3);
			next2 = 0;
			if(naname_wallcut == 0 && saitanbeforeout45offset+SLANTWALLCUT_R > 0 && Sensor_R_Dis <= WALL_TH_SLANT_R_O){
				while(Sensor_R_Dis <= WALL_TH_SLANT_R_O){
					LED1_ON();
				}
				LED1_OFF();
				go(saitanbeforeout45offset+SLANTWALLCUT_R,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanout45v,3);
			}
			//sla(bigrightangle,saitanbig90alpha,saitanbig90w,LEFT);
			if(sincurve == 0){
			sla(sout45angleR,saitanout45alpha,saitanout45w,saitanout45v,RIGHT,0,0,(saitanout45w*0.01));
			}
			else if(sincurve == 1){
			sla2(sout45angleR,saitanout45alpha,saitanout45w,saitanout45v,RIGHT,0,0,(saitanout45w*0.01));
			}
			else{
			sla3(sout45angleR,saitanout45alpha,saitanout45w,saitanout45v,RIGHT,0,0);
			}
			turn_read  = 0;
			slantingtask = 0;
			LED3_OFF();
			next = 0;
			emode = exist[pathnum+1].edge;
			if(emode == 1){
				dplus = D_PLUS;
			}
			else{
				dplus = 0;
			}
			if(path[pathnum+1] == IN45R || path[pathnum+1] == IN135R || path[pathnum+1] == BIG90R || path[pathnum+1] == BIG180R){
				rl = RIGHT;
			}
			else if(path[pathnum+1] == IN45L || path[pathnum+1] == IN135L || path[pathnum+1] == BIG90L || path[pathnum+1] == BIG180L){
				rl = LEFT;
			}
			else{
				rl = 0;
			}
			if(path[pathnum+1] <= GO31){
				go(saitanafterout45offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,3);
			}
			else if(path[pathnum+1] == IN45R || path[pathnum+1] == IN45L){
				go(saitanafterout45offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan45v,3);
			}
			else if(path[pathnum+1] == IN135R || path[pathnum+1] == IN135L){
				go(saitanafterout45offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan135v,3);
			}
			else if(path[pathnum+1] == BIG90R || path[pathnum+1] == BIG90L){
				go(saitanafterout45offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanbig90v,3);
			}
			else if(path[pathnum+1] == BIG180R || path[pathnum+1] == BIG180L){
				go(saitanafterout45offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan180v,3);
			}
			else if(path[pathnum+1] == SNODE){
				if(hosu == 0 && (goorback == 1 || goorback == 2)){
				//	if(goal_block_size == 1){
				//		go((saitanafterout45offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
				//	}
				//	else if(goal_block_size == 4){
						go((ONESECTION + saitanafterout45offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
				//	}
				//	else if(goal_block_size == 9){
				//		go((ONESECTION*2 + saitanafterout45offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
				//	}
				}
				else{
					go((saitanafterout45offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,1);
				}
			}
			else{
				go(saitanafterout45offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,3);
			}
		}
		else if(path[pathnum] == OUT45L){
			LED5_ON();
			//next = saitanbeforeout45offset;
			next2 = saitanbeforeout45offset;
			go(saitanbeforeout45offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanout45v,3);
			next2 = 0;

			if(naname_wallcut == 0 && saitanbeforeout45offset+SLANTWALLCUT_L > 0 && Sensor_L_Dis <= WALL_TH_SLANT_L_O){
				while(Sensor_L_Dis <= WALL_TH_SLANT_L_O){
					LED2_ON();
				}
				LED2_OFF();
				go(saitanbeforeout45offset+SLANTWALLCUT_L,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanout45v,3);
			}

			//sla(bigrightangle,saitanbig90alpha,saitanbig90w,LEFT);
			if(sincurve == 0){
			sla(sout45angleL,saitanout45alpha,saitanout45w,saitanout45v,LEFT,0,0,(saitanout45w*0.01));
			}
			else if(sincurve == 1){
			sla2(sout45angleL,saitanout45alpha,saitanout45w,saitanout45v,LEFT,0,0,(saitanout45w*0.01));
			}
			else{
			sla3(sout45angleL,saitanout45alpha,saitanout45w,saitanout45v,LEFT,0,0);
			}
			turn_read  = 0;
			slantingtask = 0;
			LED5_OFF();
			next = 0;
			emode = exist[pathnum+1].edge;
			if(emode == 1){
				dplus = D_PLUS;
			}
			else{
				dplus = 0;
			}
			if(path[pathnum+1] == IN45R || path[pathnum+1] == IN135R || path[pathnum+1] == BIG90R || path[pathnum+1] == BIG180R){
				rl = RIGHT;
			}
			else if(path[pathnum+1] == IN45L || path[pathnum+1] == IN135L || path[pathnum+1] == BIG90L || path[pathnum+1] == BIG180L){
				rl = LEFT;
			}
			else{
				rl = 0;
			}
			if(path[pathnum+1] <= GO31){
				go(saitanafterout45offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,3);
			}
			else if(path[pathnum+1] == IN45R || path[pathnum+1] == IN45L){
				go(saitanafterout45offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan45v,3);
			}
			else if(path[pathnum+1] == IN135R || path[pathnum+1] == IN135L){
				go(saitanafterout45offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan135v,3);
			}
			else if(path[pathnum+1] == BIG90R || path[pathnum+1] == BIG90L){
				go(saitanafterout45offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanbig90v,3);
			}
			else if(path[pathnum+1] == BIG180R || path[pathnum+1] == BIG180L){
				go(saitanafterout45offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan180v,3);
			}
			else if(path[pathnum+1] == SNODE){
				if(hosu == 0 && (goorback == 1 || goorback == 2)){
				//	if(goal_block_size == 1){
				//		go((saitanafterout45offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
				//	}
				//	else if(goal_block_size == 4){
						go((ONESECTION + saitanafterout45offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
				//	}
				//	else if(goal_block_size == 9){
				//		go((ONESECTION*2 + saitanafterout45offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
				//	}
				}
				else{
					go((saitanafterout45offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,1);
				}
			}
			else{
				go(saitanafterout45offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,3);
			}
		}
		else if(path[pathnum] == OUT135R){
			LED3_ON();
			//next = saitanbeforeout45offset;
			next2 = saitanbeforeout135offset;
			go(saitanbeforeout135offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanout135v,3);
			next2 = 0;

			if(naname_wallcut == 0 && saitanbeforeout135offset+SLANTWALLCUT_R > 0 && Sensor_R_Dis <= WALL_TH_SLANT_R_O){
				while(Sensor_R_Dis <= WALL_TH_SLANT_R_O){
					LED1_ON();
				}
				LED1_OFF();
				go(saitanbeforeout135offset+SLANTWALLCUT_R,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanout135v,3);
			}

			//sla(bigrightangle,saitanbig90alpha,saitanbig90w,LEFT);
			if(sincurve == 0){
			sla(sout135angleR,saitanout135alpha,saitanout135w,saitanout135v,RIGHT,0,0,(saitanout135w*0.01));
			}
			else if(sincurve == 1){
			sla2(sout135angleR,saitanout135alpha,saitanout135w,saitanout135v,RIGHT,0,0,(saitanout135w*0.01));
			}
			else{
			sla3(sout135angleR,saitanout135alpha,saitanout135w,saitanout135v,RIGHT,0,0);
			}
			turn_read  = 0;
			slantingtask = 0;
			LED3_OFF();
			next = 0;
			emode = exist[pathnum+1].edge;
			if(emode == 1){
				dplus = D_PLUS;
			}
			else{
				dplus = 0;
			}
			if(path[pathnum+1] == IN45R || path[pathnum+1] == IN135R || path[pathnum+1] == BIG90R || path[pathnum+1] == BIG180R){
				rl = RIGHT;
			}
			else if(path[pathnum+1] == IN45L || path[pathnum+1] == IN135L || path[pathnum+1] == BIG90L || path[pathnum+1] == BIG180L){
				rl = LEFT;
			}
			else{
				rl = 0;
			}
			if(path[pathnum+1] <= GO31){
				go(saitanafterout135offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,3);
			}
			else if(path[pathnum+1] == IN45R || path[pathnum+1] == IN45L){
				go(saitanafterout135offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan45v,3);
			}
			else if(path[pathnum+1] == IN135R || path[pathnum+1] == IN135L){
				go(saitanafterout135offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan135v,3);
			}
			else if(path[pathnum+1] == BIG90R || path[pathnum+1] == BIG90L){
				go(saitanafterout135offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanbig90v,3);
			}
			else if(path[pathnum+1] == BIG180R || path[pathnum+1] == BIG180L){
				go(saitanafterout135offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan180v,3);
			}
			else if(path[pathnum+1] == SNODE){
				if(hosu == 0 && (goorback == 1 || goorback == 2)){
				//	if(goal_block_size == 1){
				//		go((saitanafterout135offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
				//	}
				//	else if(goal_block_size == 4){
						go((ONESECTION + saitanafterout135offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
				//	}
				//	else if(goal_block_size == 9){
				//		go((ONESECTION*2 + saitanafterout135offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
				//	}
				}
				else{
					go((saitanafterout135offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,1);
				}
			}
			else{
				go(saitanafterout135offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,3);
			}
		}
		else if(path[pathnum] == OUT135L){
			LED5_ON();
			//next = saitanbeforeout45offset;
			next2 = saitanbeforeout135offset;
			go(saitanbeforeout135offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanout135v,3);
			next2 = 0;

			if(naname_wallcut == 0 && saitanbeforeout135offset+SLANTWALLCUT_L > 0 && Sensor_L_Dis <= WALL_TH_SLANT_L_O){
				while(Sensor_L_Dis <= WALL_TH_SLANT_L_O){
					LED2_ON();
				}
				LED2_OFF();
				go(saitanbeforeout135offset+SLANTWALLCUT_L,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanout135v,3);
			}

			//sla(bigrightangle,saitanbig90alpha,saitanbig90w,LEFT);
			if(sincurve == 0){
			sla(sout135angleL,saitanout135alpha,saitanout135w,saitanout135v,LEFT,0,0,(saitanout135w*0.01));
			}
			else if(sincurve == 1){
			sla2(sout135angleL,saitanout135alpha,saitanout135w,saitanout135v,LEFT,0,0,(saitanout135w*0.01));
			}
			else{
			sla3(sout135angleL,saitanout135alpha,saitanout135w,saitanout135v,LEFT,0,0);
			}
			turn_read  = 0;
			slantingtask = 0;
			LED5_OFF();
			next = 0;
			emode = exist[pathnum+1].edge;
			if(emode == 1){
				dplus = D_PLUS;
			}
			else{
				dplus = 0;
			}
			if(path[pathnum+1] == IN45R || path[pathnum+1] == IN135R || path[pathnum+1] == BIG90R || path[pathnum+1] == BIG180R){
				rl = RIGHT;
			}
			else if(path[pathnum+1] == IN45L || path[pathnum+1] == IN135L || path[pathnum+1] == BIG90L || path[pathnum+1] == BIG180L){
				rl = LEFT;
			}
			else{
				rl = 0;
			}
			if(path[pathnum+1] <= GO31){
				go(saitanafterout135offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,3);
			}
			else if(path[pathnum+1] == IN45R || path[pathnum+1] == IN45L){
				go(saitanafterout135offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan45v,3);
			}
			else if(path[pathnum+1] == IN135R || path[pathnum+1] == IN135L){
				go(saitanafterout135offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan135v,3);
			}
			else if(path[pathnum+1] == BIG90R || path[pathnum+1] == BIG90L){
				go(saitanafterout135offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanbig90v,3);
			}
			else if(path[pathnum+1] == BIG180R || path[pathnum+1] == BIG180L){
				go(saitanafterout135offset+dplus,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitan180v,3);
			}
			else if(path[pathnum+1] == SNODE){
				if(hosu == 0 && (goorback == 1 || goorback == 2)){
				//	if(goal_block_size == 1){
				//		go((saitanafterout135offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
				//	}
				//	else if(goal_block_size == 4){
						go((ONESECTION + saitanafterout135offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
				//	}
				//	else if(goal_block_size == 9){
				//		go((ONESECTION*2 + saitanafterout135offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
				//	}
				}
				else{
					go((saitanafterout135offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,1);
				}
			}
			else{
				go(saitanafterout135offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,3);
			}
		}
		else if(path[pathnum] == V90R){
			LED3_ON();
			//next = saitanbeforeout45offset;
			next2 = saitanbeforev90offset;
			go(saitanbeforev90offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanv90v,3);
			next2 = 0;

			if(naname_wallcut == 0 && saitanbeforev90offset+SLANTWALLCUT_R > 0 && Sensor_R_Dis <= WALL_TH_SLANT_R_O){
				while(Sensor_R_Dis <= WALL_TH_SLANT_R_O){
					LED1_ON();
				}
				LED1_OFF();
				go(saitanbeforev90offset+SLANTWALLCUT_R,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanv90v,3);
			}

			//sla(bigrightangle,saitanbig90alpha,saitanbig90w,LEFT);
			if(sincurve == 0){
			sla(v90angleR,saitanv90alpha,saitanv90w,saitanv90v,RIGHT,0,0,(saitanv90w*0.01));
			}
			else if(sincurve == 1){
			sla2(v90angleR,saitanv90alpha,saitanv90w,saitanv90v,RIGHT,0,0,(saitanv90w*0.01));
			}
			else{
			sla3(v90angleR,saitanv90alpha,saitanv90w,saitanv90v,RIGHT,0,0);
			}
			naname_wallcut = 0;
			turn_read  = 0;
			slantread = 0;
			slantingtask = 1;
			LED3_OFF();
			next = 0;
			beecombr_s = 0;
			beecombl_s = 0;
			combbb_s = 0;
			if(path[pathnum+1] <= DIA_GO63){
				go(saitanafterv90offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,6);
			}
			else if(path[pathnum+1] == OUT45R || path[pathnum+1] == OUT45L){
				go(saitanafterv90offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanout45v,6);
			}
			else if(path[pathnum+1] == OUT135R || path[pathnum+1] == OUT135L){
				go(saitanafterv90offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanout135v,6);
			}
			else if(path[pathnum+1] == V90R || path[pathnum+1] == V90L){
				go(saitanafterv90offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanv90v,6);
			}
			else if(path[pathnum+1] == KOJIMAR || path[pathnum+1] == KOJIMAL){
				go(saitanafterv90offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitankojimav,6);
			}
			else if(path[pathnum+1] == SNODE){
			//	lastslant = 1;
				go((ONESLANTING*goalslantnum + saitanafterv90offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
			//	lastslant = 0;
			}
			else{
				go(saitanafterv90offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,6);
			}
		}
		else if(path[pathnum] == V90L){
			LED5_ON();
			//next = saitanbeforeout45offset;
			next2 = saitanbeforev90offset;
			go(saitanbeforev90offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanv90v,3);
			next2 = 0;

			if(naname_wallcut == 0 && saitanbeforev90offset+SLANTWALLCUT_L > 0 && Sensor_L_Dis <= WALL_TH_SLANT_L_O){
				while(Sensor_L_Dis <= WALL_TH_SLANT_L_O){
					LED2_ON();
				}
				LED2_OFF();
				go(saitanbeforev90offset+SLANTWALLCUT_L,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanv90v,3);
			}

			//sla(bigrightangle,saitanbig90alpha,saitanbig90w,LEFT);
			if(sincurve == 0){
			sla(v90angleL,saitanv90alpha,saitanv90w,saitanv90v,LEFT,0,0,(saitanv90w*0.01));
			}
			else if(sincurve == 1){
			sla2(v90angleL,saitanv90alpha,saitanv90w,saitanv90v,LEFT,0,0,(saitanv90w*0.01));
			}
			else{
			sla3(v90angleL,saitanv90alpha,saitanv90w,saitanv90v,LEFT,0,0);
			}
			naname_wallcut = 0;
			turn_read  = 0;
			slantread = 0;
			slantingtask = 2;
			LED5_OFF();
			next = 0;
			beecombr_s = 0;
			beecombl_s = 0;
			combbb_s = 0;
			if(path[pathnum+1] <= DIA_GO63){
				go(saitanafterv90offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,6);
			}
			else if(path[pathnum+1] == OUT45R || path[pathnum+1] == OUT45L){
				go(saitanafterv90offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanout45v,6);
			}
			else if(path[pathnum+1] == OUT135R || path[pathnum+1] == OUT135L){
				go(saitanafterv90offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanout135v,6);
			}
			else if(path[pathnum+1] == V90R || path[pathnum+1] == V90L){
				go(saitanafterv90offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanv90v,6);
			}
			else if(path[pathnum+1] == KOJIMAR || path[pathnum+1] == KOJIMAL){
				go(saitanafterv90offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitankojimav,6);
			}
			else if(path[pathnum+1] == SNODE){
			//	lastslant = 1;
				go((ONESLANTING*goalslantnum + saitanafterv90offset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
			//	lastslant = 0;
			}
			else{
				go(saitanafterv90offset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,6);
			}
		}
		else if(path[pathnum] == KOJIMAR){
			LED3_ON();
			//next = saitanbeforeou45offset;
			next2 = saitanbeforekojimaoffset;
			go(saitanbeforekojimaoffset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitankojimav,3);
			next2 = 0;

			if(naname_wallcut == 0 && saitanbeforekojimaoffset+SLANTWALLCUT_R > 0 && Sensor_R_Dis <= WALL_TH_SLANT_R_O){
				while(Sensor_R_Dis <= WALL_TH_SLANT_R_O){
					LED1_ON();
				}
				LED1_OFF();
				go(saitanbeforekojimaoffset+SLANTWALLCUT_R,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitankojimav,3);
			}

			//sla(bigrightangle,saitanbig90alpha,saitanbig90w,LEFT);
			if(sincurve == 0){
			sla(kojimaangleR,saitankojimaalpha,saitankojimaw,saitankojimav,RIGHT,0,0,(saitankojimaw*0.01));
			}
			else if(sincurve == 1){
			sla2(kojimaangleR,saitankojimaalpha,saitankojimaw,saitankojimav,RIGHT,0,0,(saitankojimaw*0.01));
			}
			else{
			sla3(kojimaangleR,saitankojimaalpha,saitankojimaw,saitankojimav,RIGHT,0,0);
			}
			naname_wallcut = 0;
			turn_read  = 0;
			slantread = 0;
			slantingtask = 1;
			LED3_OFF();
			next = 0;
			beecombr_s = 0;
			beecombl_s = 0;
			combbb_s = 0;
			if(path[pathnum+1] <= DIA_GO63){
				go(saitanafterkojimaoffset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,6);
			}
			else if(path[pathnum+1] == OUT45R || path[pathnum+1] == OUT45L){
				go(saitanafterkojimaoffset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanout45v,6);
			}
			else if(path[pathnum+1] == OUT135R || path[pathnum+1] == OUT135L){
				go(saitanafterkojimaoffset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanout135v,6);
			}
			else if(path[pathnum+1] == V90R || path[pathnum+1] == V90L){
				go(saitanafterkojimaoffset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanv90v,6);
			}
			else if(path[pathnum+1] == KOJIMAR || path[pathnum+1] == KOJIMAL){
				go(saitanafterkojimaoffset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitankojimav,6);
			}
			else if(path[pathnum+1] == SNODE){
			//	lastslant = 1;
				go((ONESLANTING*goalslantnum + saitanafterkojimaoffset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
			//	lastslant = 0;
			}
			else{
				go(saitanafterkojimaoffset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,6);
			}
		}
		else if(path[pathnum] == KOJIMAL){
			LED5_ON();
			//next = saitanbeforeout45offset;
			next2 = saitanbeforekojimaoffset;
			go(saitanbeforekojimaoffset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitankojimav,3);
			next2 = 0;

			if(naname_wallcut == 0 && saitanbeforekojimaoffset+SLANTWALLCUT_L > 0 && Sensor_L_Dis <= WALL_TH_SLANT_L_O){
				while(Sensor_L_Dis <= WALL_TH_SLANT_L_O){
					LED2_ON();
				}
				LED2_OFF();
				go(saitanbeforekojimaoffset+SLANTWALLCUT_L,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitankojimav,3);
			}

			//sla(bigrightangle,saitanbig90alpha,saitanbig90w,LEFT);
			if(sincurve == 0){
			sla(kojimaangleL,saitankojimaalpha,saitankojimaw,saitankojimav,LEFT,0,0,(saitankojimaw*0.01));
			}
			else if(sincurve == 1){
			sla2(kojimaangleL,saitankojimaalpha,saitankojimaw,saitankojimav,LEFT,0,0,(saitankojimaw*0.01));
			}
			else{
			sla3(kojimaangleL,saitankojimaalpha,saitankojimaw,saitankojimav,LEFT,0,0);
			}
			naname_wallcut = 0;
			turn_read  = 0;
			slantread = 0;
			slantingtask = 2;
			LED5_OFF();
			next = 0;
			beecombr_s = 0;
			beecombl_s = 0;
			combbb_s = 0;
			if(path[pathnum+1] <= DIA_GO63){
				go(saitanafterkojimaoffset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,6);
			}
			else if(path[pathnum+1] == OUT45R || path[pathnum+1] == OUT45L){
				go(saitanafterkojimaoffset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanout45v,6);
			}
			else if(path[pathnum+1] == OUT135R || path[pathnum+1] == OUT135L){
				go(saitanafterkojimaoffset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanout135v,6);
			}
			else if(path[pathnum+1] == V90R || path[pathnum+1] == V90L){
				go(saitanafterkojimaoffset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanv90v,6);
			}
			else if(path[pathnum+1] == KOJIMAR || path[pathnum+1] == KOJIMAL){
				go(saitanafterkojimaoffset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitankojimav,6);
			}
			else if(path[pathnum+1] == SNODE){
			//	lastslant = 1;
				go((ONESLANTING*goalslantnum + saitanafterkojimaoffset),saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,0,0);
			//	lastslant = 0;
			}
			else{
				go(saitanafterkojimaoffset,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,saitanmaxv,6);
			}
		}
		else{
		}
		pathnum++;
	}
}

void makesaitan(int tx,int ty){	//最短の動作順
  	int mmx = mx;
	int mmy = my;	//ダミー座ふょう
	short head1 = head;
  	short i = 0;
	short head0 = 0;	//進む方向や方角
	short temp0 = 0;	//mapdata
	short s0 = 0;	//最適値
	short s1 = 0;	//調査地
	makesmap(tx,ty,1);	//最短中は未探索区間壁ありで走行
	while(1){
		if(head1 == 0){
			mmy++;
		}
		else if(head1 == 1){
			mmx++;
		}
		else if(head1 == 2){
			mmy--;
		}
		else if(head1 == 3){
			mmx--;	//座標更新
		}
		if(mmx == tx && mmy == ty){	//ゴールにたどり着いたら?
			saitanway[i] = 0;
			head1+=2;
			if(head1 > 3){
				head1 = head1 - 4;
			}
			if(head1 < 0){
				head1 = head1 + 4;	//向きの訂正
			}
			tth = head1;
			return;
		}
		/* 回りの4区間の内、その方向に壁がなくて、目的地に一番近い		*/
		/* 区間に移動する。ただし、移動できる一番近い区間が複数ある		*/
		/* 場合は(既探索区間,直進）(既探索区間,旋回）の順で選択する		*/
		/* 探索ではないので未探索区間は優先しない。というか行かない		*/

		temp0 = map[mmy][mmx];
		s0 = 1024;
		if ((temp0 & 0x11) == 0x10){				/*	北方向の区間の確認		*/
			s1 = smap[mmy+1][mmx] * 4 + 4;
		//	if ((map[mmy+1][mmx]&0x0f0)!=0x0f0)	s1 = s1 - 2;	//探索フラグを歩数マップの次に優先
			if (head1 == 0){
				s1 = s1 - 1;	//直進>旋回
			}
			if (s1 < s0) {
				s0 = s1;
				head0 = 0;
			}	//今までより有利ならそっちに転向
		}
		if ((temp0 & 0x22) == 0x20){				/*	東方向の区間の確認		*/
			s1 = smap[mmy][mmx+1] * 4 + 4;
		//	if ((map[mmy][mmx+1]&0x0f0)!=0x0f0)	s1 = s1 - 2;
			if (head1 == 1){
				s1 = s1 - 1;
			}
			if (s1 < s0) {
				s0 = s1;
				head0 = 1;
			}
		}
		if ((temp0 & 0x44) == 0x40){				/*	南方向の区間の確認		*/
			s1 = smap[mmy-1][mmx] * 4 + 4;
		//	if ((map[mmy-1][mmx]&0x0f0)!=0x0f0)	s1 = s1 - 2;
			if (head1 == 2){
				s1 = s1 - 1;
			}
			if (s1 < s0) {
				s0 = s1;
				head0 = 2;
			}
		}
		if ((temp0 & 0x88) == 0x80){				/*	西方向の区間の確認		*/
			s1 = smap[mmy][mmx-1] * 4 + 4;
		//	if ((map[mmy][mmx-1]&0x0f0)!=0x0f0)	s1 = s1 - 2;
			if (head1 == 3)	{
				s1 = s1 - 1;
			}
			if (s1 < s0) {
				s0 = s1;
				head0 = 3;
			}
		}

		head0 = (head0 + 4 - head1) & 3;			/* 移動する方向を決定	*/

		saitanway[i] = 0;	//マップ初期化

		if(head0 == 0){	//直進するなら
			saitanway[i] = FRONT;
		}
		else if(head0 == 1){	//右旋回
			saitanway[i] = RIGHT;
			head1 = (head1 + 1) % 4;
		}
		else if(head0 == 2){	//180旋回
			//ありえないに決まってる
		}
		else if(head0 == 3){	//左旋回
			saitanway[i] = LEFT;
			head1 = (head1 + 3) % 4;
		}
		i++;	//まぷの配列変数の順を上げる
	}
}

void saitan_to_path(void){
	int i = 0, j = 0;
	int s_count = 0;
	int diagonal_count = 0;
	char diagonal_flag = 0;

//	for (i = 0; i <= g_flag_step_goal; i++) {
	while(1){
		if(saitanway[i] == 0){
			s_count++;
			if (s_count > 0) {
				path[j] = s_count;
				path[j+1] = SNODE;
			} else {
				path[j] = SNODE;
			}
			break;
		}
		else if (saitanway[i] == FRONT) {
			s_count++;
		} else if (saitanway[i] == LEFT) {
			/*if (i == 1) {//開幕大回りできないとき用
			 path[j] = s_count;
			 j++;
			 path[j] = S_LEFT;
			 s_count = 0;
			 } else */
			if ((i-1 >= 0 && saitanway[i - 1] == FRONT) && (saitanway[i + 1] == FRONT || saitanway[i + 1] == 0)) {
				//大回り
				if (s_count > 0) {
					path[j] = s_count;
					j++;
					s_count = 0;
				}
				s_count = -1;
				path[j] = BIG90L;
			} else if ((i-1 >= 0 && saitanway[i - 1] == FRONT) && saitanway[i + 1] == LEFT
					&& (saitanway[i + 2] == FRONT || saitanway[i + 2] == 0)) {
				//Uターン
				if (s_count > 0) {
					path[j] = s_count;
					j++;
					s_count = 0;
				}
				s_count = -1;
				path[j] = BIG180L;
				i += 1;
			} else if ((i-1 >= 0 && saitanway[i - 1] == FRONT) && saitanway[i + 1] == RIGHT) {
				//入45度ターン
				if (s_count > 0) {
					path[j] = s_count;
					j++;
					s_count = 0;
				}
				path[j] = IN45L;
				diagonal_flag = 1;
			} else if ((saitanway[i + 1] == FRONT || saitanway[i + 1] == 0) && diagonal_flag == 1) {
				//出45度ターン
				if (diagonal_count > 0) {
					path[j] = DIAGONAL + diagonal_count;
					j++;
					diagonal_count = 0;
				}
				path[j] = OUT45L;
				s_count = -1;
				diagonal_flag = 0;
			} else if (saitanway[i + 1] == LEFT && saitanway[i + 2] == RIGHT
					&& diagonal_flag == 1) {
				//斜め90度ターン
				if (diagonal_count > 0) {
					path[j] = DIAGONAL + diagonal_count;
					j++;
					diagonal_count = 0;
				}
				path[j] = V90L;
				i += 1;
			} else if ((i-1 >= 0 && saitanway[i - 1] == FRONT) && saitanway[i + 1] == LEFT) {
				//入135度ターン
				if (s_count > 0) {
					path[j] = s_count;
					j++;
					s_count = 0;
				}
				path[j] = IN135L;
				i += 1;
				diagonal_flag = 1;
			} else if (saitanway[i + 1] == LEFT && (saitanway[i + 2] == FRONT || saitanway[i + 2] == 0)
					&& diagonal_flag == 1) {
				//出135度ターン
				if (diagonal_count > 0) {
					path[j] = DIAGONAL + diagonal_count;
					j++;
					diagonal_count = 0;
				}
				s_count = -1;
				path[j] = OUT135L;
				i += 1;
				diagonal_flag = 0;
			} else if ((i-1 >= 0 && saitanway[i - 1] == RIGHT) && saitanway[i + 1] == RIGHT
					&& diagonal_flag == 1) {
				//斜め直線
				diagonal_count += 1;
				j--;
			} else {
				if (s_count > 0) {
					path[j] = s_count;
					j++;
				}
				s_count = 0;
				path[j] = 0;
			}
			j++;
		} else if (saitanway[i] == RIGHT) {
			/*if (i == 1) {
			 path[j] = s_count;
			 j++;
			 path[j] = S_RIGHT;
			 s_count = 0;
			 } else*/
			if ((i == 0 || (i-1 >= 0 && saitanway[i - 1] == FRONT)) && (saitanway[i + 1] == FRONT || saitanway[i + 1] == 0)) {
				//大回り
				if (s_count > 0) {
					path[j] = s_count;
					j++;
					s_count = 0;
				}
				s_count = -1;

				path[j] = BIG90R;
			} else if ((i == 0 || (i-1 >= 0 && saitanway[i - 1] == FRONT)) && saitanway[i + 1] == RIGHT
					&& (saitanway[i + 2] == FRONT || saitanway[i + 2] == 0)) {
				//Uターン
				if (s_count > 0) {
					path[j] = s_count;
					j++;
					s_count = 0;
				}
				s_count = -1;
				path[j] = BIG180R;
				i += 1;
			} else if ((i == 0 || (i-1 >= 0 && saitanway[i - 1] == FRONT)) && saitanway[i + 1] == LEFT) {
				//入45度ターン
				if (s_count > 0) {
					path[j] = s_count;
					j++;
					s_count = 0;
				}
				path[j] = IN45R;
				diagonal_flag = 1;
			} else if ((saitanway[i + 1] == FRONT || saitanway[i + 1] == 0) && diagonal_flag == 1) {
				//出45度ターン
				if (diagonal_count > 0) {
					path[j] = DIAGONAL + diagonal_count;
					j++;
					diagonal_count = 0;
				}
				path[j] = OUT45R;
				s_count = -1;
				diagonal_flag = 0;
			} else if (saitanway[i + 1] == RIGHT && saitanway[i + 2] == LEFT
					&& diagonal_flag == 1) {
				//斜め90度ターン
				if (diagonal_count > 0) {
					path[j] = DIAGONAL + diagonal_count;
					j++;
					diagonal_count = 0;
				}
				path[j] = V90R;
				i += 1;
			} else if ((i == 0 || (i-1 >= 0 && saitanway[i - 1] == FRONT)) && saitanway[i + 1] == RIGHT) {
				//入135度ターン
				if (s_count > 0) {
					path[j] = s_count;
					j++;
					s_count = 0;
				}
				path[j] = IN135R;
				i += 1;
				diagonal_flag = 1;
			} else if (saitanway[i + 1] == RIGHT && (saitanway[i + 2] == FRONT || saitanway[i + 2] == 0)
					&& diagonal_flag == 1) {
				//出135度ターン
				if (diagonal_count > 0) {
					path[j] = DIAGONAL + diagonal_count;
					j++;
					diagonal_count = 0;
				}
				s_count = -1;
				path[j] = OUT135R;
				i += 1;
				diagonal_flag = 0;
			} else if ((i-1 >= 0 && saitanway[i - 1] == LEFT) && saitanway[i + 1] == LEFT
					&& diagonal_flag == 1) {
				//斜め直線
				diagonal_count += 1;
				j--;
			} else {
				if (s_count > 0) {
					path[j] = s_count;
					j++;
				}
				s_count = 0;
				path[j] = 0;
			}
			j++;
		}
		i++;
	}

	//g_flag_step_goal_3 = j;
}

void waitForEdge(char RorL){
	//壁なしターン連続時の壁切れ待ち関数
	if(RorL == RIGHT){
		if(Sensor_R_Dis >= WALL_TH_R){
			while(Sensor_R_Dis >= WALL_TH_R);
			LED4_ON();
			while(Sensor_R_Dis < WALL_TH_R);
			LED4_OFF();
		}
		else{
			//while(Sensor_R_Dis >= WALL_TH_R);
			LED4_ON();
			while(Sensor_R_Dis < WALL_TH_R);
			LED4_OFF();
		}
		go(next - SAITANWALLCUT_R,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,nextv,3);
	}
	else if(RorL == LEFT){
		if(Sensor_L_Dis >= WALL_TH_L){
			while(Sensor_L_Dis >= WALL_TH_L);
			LED4_ON();
			while(Sensor_L_Dis < WALL_TH_L);
			LED4_OFF();
		}
		else{
			//while(Sensor_R_Dis >= WALL_TH_R);
			LED4_ON();
			while(Sensor_L_Dis < WALL_TH_L);
			LED4_OFF();
		}
		go(next - SAITANWALLCUT_L,saitanfastacc,saitanfastdec,saitanminv,saitanmaxv,speed,nextv,3);
	}
}

void turnWallEdge(void){
	//ターン時の壁切れパターンを記録
	short mh = 0;
	short i = 0;
	short p1 = 0,p2 = 0;
	short xb = 0,yb = 0,hb = 0;
	short r = 0;
	short num = 0;
	for(r = 0; r < 1024; r++){
		exist[r].edge = 0;
		exist[r].combsw = 0;
		//combsw[256] = 0;
	}
	mx = 0;
	my = 0;
	while(1){
		if(path[i] == SNODE){
			break;
		}
		p2 = p1;
		p1 = path[i];
		xb = mx;
		yb = my;
		hb = mh;
		if (path[i] <= GO31) {
			if (mh == 0) {
				for(num = 0; num < path[i]; num++){
					if((map[my+num][mx] & 0x22) == 0x20 && (map[my+num][mx] & 0x88) == 0x80){
						exist[i].combsw = 1;
					}
				}
				my += path[i];
			}
			else if (mh == 1) {
				for(num = 0; num < path[i]; num++){
					if((map[my][mx+num] & 0x11) == 0x10 && (map[my][mx+num] & 0x44) == 0x40){
						exist[i].combsw = 1;
					}
				}
				mx += path[i];
			}
			else if (mh == 2) {
				for(num = 0; num < path[i]; num++){
					if((map[my-num][mx] & 0x22) == 0x20 && (map[my-num][mx] & 0x88) == 0x80){
						exist[i].combsw = 1;
					}
				}
				my -= path[i];
			}
			else if (mh == 3) {
				for(num = 0; num < path[i]; num++){
					if((map[my][mx-num] & 0x11) == 0x10 && (map[my][mx-num] & 0x44) == 0x40){
						exist[i].combsw = 1;
					}
				}
				mx -= path[i];
			}
		}
		else if (path[i] <= DIA_GO63) {
			if (mh == 4) {
				if (path[i] % 2 == 0) {
					mx -= ((path[i] - 38) / 2);
					my += ((path[i] - 38) / 2);
					mh = 8;
				}
				else {
					mx -= ((path[i] - 39) / 2);
					my += ((path[i] - 39) / 2);
					mh = 4;
				}
			}
			else if (mh == 5) {
				if (path[i] % 2 == 0) {
					mx += ((path[i] - 40) / 2);
					my += ((path[i] - 38) / 2);
					mh = 9;
				}
				else {
					mx += ((path[i] - 39) / 2);
					my += ((path[i] - 39) / 2);
					mh = 5;
				}
			}
			else if (mh == 6) {
				if (path[i] % 2 == 0) {
					mx += ((path[i] - 40) / 2);
					my -= ((path[i] - 40) / 2);
					mh = 10;
				}
				else {
					mx += ((path[i] - 39) / 2);
					my -= ((path[i] - 39) / 2);
					mh = 6;
				}
			}
			else if (mh == 7) {
				if (path[i] % 2 == 0) {
					mx -= ((path[i] - 38) / 2);
					my -= ((path[i] - 40) / 2);
					mh = 11;
				}
				else {
					mx -= ((path[i] - 39) / 2);
					my -= ((path[i] - 39) / 2);
					mh = 7;
				}
			}
			else if (mh == 8) {
				if (path[i] % 2 == 0) {
					mx -= ((path[i] - 40) / 2);
					my += ((path[i] - 40) / 2);
					mh = 4;
				}
				else {
					mx -= ((path[i] - 39) / 2);
					my += ((path[i] - 39) / 2);
					mh = 8;
				}
			}
			else if (mh == 9) {
				if (path[i] % 2 == 0) {
					mx += ((path[i] - 38) / 2);
					my += ((path[i] - 40) / 2);
					mh = 5;
				}
				else {
					mx += ((path[i] - 39) / 2);
					my += ((path[i] - 39) / 2);
					mh = 9;
				}
			}
			else if (mh == 10) {
				if (path[i] % 2 == 0) {
					mx += ((path[i] - 38) / 2);
					my -= ((path[i] - 38) / 2);
					mh = 6;
				}
				else {
					mx += ((path[i] - 39) / 2);
					my -= ((path[i] - 39) / 2);
					mh = 10;
				}
			}
			else if (mh == 11) {
				if (path[i] % 2 == 0) {
					mx -= ((path[i] - 40) / 2);
					my -= ((path[i] - 38) / 2);
					mh = 7;
				}
				else {
					mx -= ((path[i] - 39) / 2);
					my -= ((path[i] - 39) / 2);
					mh = 11;
				}
			}
		}
		else if (path[i] == BIG90R) {
			if (mh == 0) {
				mx += 1;
				my += 1;
				mh = 1;
			}
			else if (mh == 1) {
				mx += 1;
				my -= 1;
				mh = 2;
			}
			else if (mh == 2) {
				mx -= 1;
				my -= 1;
				mh = 3;
			}
			else if (mh == 3) {
				mx -= 1;
				my += 1;
				mh = 0;
			}
		}
		else if (path[i] == BIG90L) {
			if (mh == 0) {
				mx -= 1;
				my += 1;
				mh = 3;
			}
			else if (mh == 1) {
				mx += 1;
				my += 1;
				mh = 0;
			}
			else if (mh == 2) {
				mx += 1;
				my -= 1;
				mh = 1;
			}
			else if (mh == 3) {
				mx -= 1;
				my -= 1;
				mh = 2;
			}
		}
		else if (path[i] == BIG180R) {
			if (mh == 0) {
				mx += 1;
				//	my += 1;
				mh = 2;
			}
			else if (mh == 1) {
				//	mx += 1;
				my -= 1;
				mh = 3;
			}
			else if (mh == 2) {
				mx -= 1;
				//	my -= 1;
				mh = 0;
			}
			else if (mh == 3) {
				//	mx -= 1;
				my += 1;
				mh = 1;
			}
		}
		else if (path[i] == BIG180L) {
			if (mh == 0) {
				mx -= 1;
				//	my += 1;
				mh = 2;
			}
			else if (mh == 1) {
				//	mx += 1;
				my += 1;
				mh = 3;
			}
			else if (mh == 2) {
				mx += 1;
				//	my -= 1;
				mh = 0;
			}
			else if (mh == 3) {
				//	mx -= 1;
				my -= 1;
				mh = 1;
			}
		}
		else if (path[i] == IN45R) {
			if (mh == 0) {
				//mx += 1;
				my += 1;
				mh = 9;
			}
			else if (mh == 1) {
				mx += 1;
				my -= 1;
				mh = 6;
			}
			else if (mh == 2) {
				mx -= 1;
				my -= 1;
				mh = 11;
			}
			else if (mh == 3) {
				mx -= 1;
				//my += 1;
				mh = 4;
			}
		}
		else if (path[i] == IN45L) {
			if (mh == 0) {
				mx -= 1;
				my += 1;
				mh = 8;
			}
			else if (mh == 1) {
				mx += 1;
				//my -= 1;
				mh = 5;
			}
			else if (mh == 2) {
				//mx -= 1;
				my -= 1;
				mh = 10;
			}
			else if (mh == 3) {
				mx -= 1;
				my -= 1;
				mh = 7;
			}
		}
		else if (path[i] == IN135R) {
			if (mh == 0) {
				mx += 1;
				//my += 1;
				mh = 6;
			}
			else if (mh == 1) {
				//mx += 1;
				my -= 1;
				mh = 11;
			}
			else if (mh == 2) {
				mx -= 1;
				my -= 1;
				mh = 4;
			}
			else if (mh == 3) {
				mx -= 1;
				my += 1;
				mh = 9;
			}
		}
		else if (path[i] == IN135L) {
			if (mh == 0) {
				mx -= 1;
				//my += 1;
				mh = 7;
			}
			else if (mh == 1) {
				//mx += 1;
				my += 1;
				mh = 8;
			}
			else if (mh == 2) {
				mx += 1;
				my -= 1;
				mh = 5;
			}
			else if (mh == 3) {
				mx -= 1;
				my -= 1;
				mh = 10;
			}
		}
		else if (path[i] == OUT45R) {
			if (mh == 5) {
				mx += 1;
				my += 1;
				mh = 1;
			}
			else if (mh == 10) {
				mx += 1;
				my -= 1;
				mh = 2;
			}
			else if (mh == 7) {
				mx -= 1;
				//my -= 1;
				mh = 3;
			}
			else if (mh == 8) {
				//mx -= 1;
				my += 1;
				mh = 0;
			}
		}
		else if (path[i] == OUT45L) {
			if (mh == 4) {
				mx -= 1;
				my += 1;
				mh = 3;
			}
			else if (mh == 9) {
				mx += 1;
				my += 1;
				mh = 0;
			}
			else if (mh == 6) {
				mx += 1;
				//my -= 1;
				mh = 1;
			}
			else if (mh == 11) {
				//mx -= 1;
				my -= 1;
				mh = 2;
			}
		}
		else if (path[i] == OUT135R) {
			if (mh == 5) {
				mx += 1;
				//my += 1;
				mh = 2;
			}
			else if (mh == 10) {
				//mx += 1;
				my -= 1;
				mh = 3;
			}
			else if (mh == 7) {
				mx -= 1;
				my += 1;
				mh = 0;
			}
			else if (mh == 8) {
				mx += 1;
				my += 1;
				mh = 1;
			}
		}
		else if (path[i] == OUT135L) {
			if (mh == 4) {
				mx -= 1;
				//	my += 1;
				mh = 2;
			}
			else if (mh == 9) {
				//	mx += 1;
				my += 1;
				mh = 3;
			}
			else if (mh == 6) {
				mx += 1;
				my += 1;
				mh = 0;
			}
			else if (mh == 11) {
				mx += 1;
				my -= 1;
				mh = 1;
			}
		}
		else if (path[i] == V90R) {
			if (mh == 5) {
				mx += 1;
				//my += 1;
				mh = 6;
			}
			else if (mh == 10) {
				//mx += 1;
				my -= 1;
				mh = 11;
			}
			else if (mh == 7) {
				mx -= 1;
				//my -= 1;
				mh = 4;
			}
			else if (mh == 8) {
				//mx -= 1;
				my += 1;
				mh = 9;
			}
		}
		else if (path[i] == V90L) {
			if (mh == 4) {
				mx -= 1;
				//	my += 1;
				mh = 7;
			}
			else if (mh == 9) {
				//	mx += 1;
				my += 1;
				mh = 8;
			}
			else if (mh == 6) {
				mx += 1;
				//my -= 1;
				mh = 5;
			}
			else if (mh == 11) {
				//mx -= 1;
				my -= 1;
				mh = 10;
			}
		}
		else if (path[i] == KOJIMAR) {
			if (mh == 5) {
				mx += 2;
				//my += 1;
				mh = 6;
			}
			else if (mh == 10) {
				//mx += 1;
				my -= 2;
				mh = 11;
			}
			else if (mh == 7) {
				mx -= 2;
				//my -= 1;
				mh = 4;
			}
			else if (mh == 8) {
				//mx -= 1;
				my += 2;
				mh = 9;
			}
		}
		else if (path[i] == KOJIMAL) {
			if (mh == 4) {
				mx -= 2;
				//	my += 1;
				mh = 7;
			}
			else if (mh == 9) {
				//	mx += 1;
				my += 2;
				mh = 8;
			}
			else if (mh == 6) {
				mx += 2;
				//my -= 1;
				mh = 5;
			}
			else if (mh == 11) {
				//mx -= 1;
				my -= 2;
				mh = 10;
			}
		}
		else {
			break;
		}
		if((p2 == BIG90R || p2 == BIG90L || p2 == BIG180R || p2 == BIG180L || p2 == OUT45R || p2 == OUT45L || p2 == OUT135R || p2 == OUT135L || p2 <= GO31)
			&& (p1 == BIG90R || p1 == BIG90L || p1 == BIG180R || p1 == BIG180L || p1 == IN45R || p1 == IN45L || p1 == IN135R || p1 == IN135L)){
			if(p1 == BIG90R || p1 == BIG180R || p1 == IN45R || p2 == IN135R){
				if(hb == 0){
					if((map[yb][xb] & 0x22) == 0x20){
						exist[i].edge = 1;
					}
					else if((map[yb][xb] & 0x22) == 0x22){
						exist[i].edge = 2;
					}
				}
				else if(hb == 1){
					if((map[yb][xb] & 0x44) == 0x40){
						exist[i].edge = 1;
					}
					else if((map[yb][xb] & 0x44) == 0x44){
						exist[i].edge = 2;
					}
				}
				else if(hb == 2){
					if((map[yb][xb] & 0x88) == 0x80){
						exist[i].edge = 1;
					}
					else if((map[yb][xb] & 0x88) == 0x88){
						exist[i].edge = 2;
					}
				}
				else if(hb == 3){
					if((map[yb][xb] & 0x11) == 0x10){
						exist[i].edge = 1;
					}
					else if((map[yb][xb] & 0x11) == 0x11){
						exist[i].edge = 2;
					}
				}
			}
			else{
				if(hb == 0){
					if((map[yb][xb] & 0x88) == 0x80){
						exist[i].edge = 1;
					}
					else if((map[yb][xb] & 0x88) == 0x88){
						exist[i].edge = 2;
					}
				}
				else if(hb == 1){
					if((map[yb][xb] & 0x11) == 0x10){
						exist[i].edge = 1;
					}
					else if((map[yb][xb] & 0x11) == 0x11){
						exist[i].edge = 2;
					}
				}
				else if(hb == 2){
					if((map[yb][xb] & 0x22) == 0x20){
						exist[i].edge = 1;
					}
					else if((map[yb][xb] & 0x22) == 0x22){
						exist[i].edge = 2;
					}
				}
				else if(hb == 3){
					if((map[yb][xb] & 0x44) == 0x40){
						exist[i].edge = 1;
					}
					else if((map[yb][xb] & 0x44) == 0x44){
						exist[i].edge = 2;
					}
				}
			}
		}
		i++;
	}
}

void edge_reverse(void){
	short num = 0;
	volatile short i = 0;
	//char dum[1024];
	for (i = 0; i < 1024; i++) {
		dummy[i] = 0;
	}
	while(path[num] != SNODE){
		num++;
	}
	for(i = 0; i < num; i++){
		dummy[i] = exist[num-1-i].edge;
	}
	for(i = 0; i < num; i++){
		exist[i].edge = dummy[i];
	}
	//
	for(i = 0; i < num; i++){
		dummy[i] = exist[num-1-i].combsw;
	}
	for(i = 0; i < num; i++){
		exist[i].combsw = dummy[i];
	}
}


void setpara(int a){

	FanVolt = flash2[a].fan;
	sincurve = flash2[a].sin;
	saitanacc_one = flash2[a].acc1;
	saitandec_one = flash2[a].dec1;
	saitanslantacc_one = flash2[a].slantacc1;
	saitanslantdec_one = flash2[a].slantdec1;
	saitandashv = flash2[a].dashv;
	saitanacc = flash2[a].acc;
	saitanslantacc = flash2[a].slantacc;
	saitanslantdec = flash2[a].slantdec;
	saitanfastacc = flash2[a].fastacc;
	saitanfastdec = flash2[a].fastdec;
	saitanfirstacc = flash2[a].firstacc;
	saitandec = flash2[a].dec;
	saitanstopdec = flash2[a].stopdec;
	saitanminv = flash2[a].minv;
	//斜め
	//斜め速とoffset(goal時)とオフセット時のマックス
	saitandashslantingv = flash2[a].slantdashv;
	saitanmaxv = flash2[a].offsetmaxv;
	saitanoffset = 0;
	//45度
	saitanbefore45offset = flash2[a].in45.offset1;//5
	saitanafter45offset = flash2[a].in45.offset2;//25
	saitanbeforeout45offset = flash2[a].out45.offset1;//24
	saitanafterout45offset = flash2[a].out45.offset2;//10
	saitan45v = flash2[a].in45.v;
	saitan45alpha = flash2[a].in45.alpha;
	saitan45w = flash2[a].in45.w;
	saitanout45v = flash2[a].out45.v;
	saitanout45alpha = flash2[a].out45.alpha;
	saitanout45w = flash2[a].out45.w;
	s45angleR = flash2[a].in45.angleR;
	s45angleL = flash2[a].in45.angleL;
	sout45angleR = flash2[a].out45.angleR;
	sout45angleL = flash2[a].out45.angleL;
	//大回り90度
	saitanbeforebig90offset = flash2[a].big90.offset1;//12;//15
	saitanafterbig90offset = flash2[a].big90.offset2;//15
	saitanbig90v = flash2[a].big90.v;
	saitanbig90alpha = flash2[a].big90.alpha;
	saitanbig90w = flash2[a].big90.w;
	bigrightangleR = flash2[a].big90.angleR;
	bigrightangleL = flash2[a].big90.angleL;
	//135度
	saitanbefore135offset = flash2[a].in135.offset1;//18;//20
	saitanafter135offset = flash2[a].in135.offset2;//19;//15
	saitanbeforeout135offset = flash2[a].out135.offset1;//14;
	saitanafterout135offset = flash2[a].out135.offset2;//28;
	saitan135v = flash2[a].in135.v;
	saitan135alpha = flash2[a].in135.alpha;//130.9;
	saitan135w = flash2[a].in135.w;
	saitanout135v = flash2[a].out135.v;
	saitanout135alpha = flash2[a].out135.alpha;//130.9;
	saitanout135w = flash2[a].out135.w;
	s135angleR = flash2[a].in135.angleR;
	s135angleL = flash2[a].in135.angleL;
	sout135angleR = flash2[a].out135.angleR;
	sout135angleL = flash2[a].out135.angleL;
	//180度
	saitanbefore180offset = flash2[a].big180.offset1;
	saitanafter180offset = flash2[a].big180.offset2;//28
	saitan180v = flash2[a].big180.v;
	saitan180alpha = flash2[a].big180.alpha;
	saitan180w = flash2[a].big180.w;
	big180angleR = flash2[a].big180.angleR;
	big180angleL = flash2[a].big180.angleL;
	//V90
	saitanbeforev90offset = flash2[a].v90.offset1;//3;//7;//10
	saitanafterv90offset = flash2[a].v90.offset2;//20;//17;//27;//10;
	saitanv90v = flash2[a].v90.v;
	saitanv90alpha = flash2[a].v90.alpha;//209.43;
	saitanv90w = flash2[a].v90.w;//13.96;
	v90angleR = flash2[a].v90.angleR;
	v90angleL = flash2[a].v90.angleL;
	//kojima
	saitanbeforekojimaoffset = flash2[a].kojima.offset1;//10;//10
	saitanafterkojimaoffset = flash2[a].kojima.offset2;//13;//27;//10;
	saitankojimav = flash2[a].kojima.v;
	saitankojimaalpha = flash2[a].kojima.alpha;//296.7;
	saitankojimaw = flash2[a].kojima.w;//17.45;
	kojimaangleR = flash2[a].kojima.angleR;
	kojimaangleL = flash2[a].kojima.angleL;

	//check_lipo();
}
