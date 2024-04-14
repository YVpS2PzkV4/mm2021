#include "index.h"
#include"Interface.h"
#include"common.h"
#include"CMT.h"
#include"run.h"
#include"search.h"
#include"map.h"
#include"Dataflash.h"
#include<math.h>
//#include<mathf.h>

#define ARM_MATH_CM4
#include "arm_math.h"
#include "arm_const_structs.h"

extern volatile float Sensor_R_Dis;
extern volatile float Sensor_L_Dis;
extern volatile float Sensor_FR_Dis;
extern volatile float Sensor_FL_Dis;
extern unsigned volatile short Wall_R;
extern unsigned volatile short Wall_L;
extern unsigned volatile short Wall_FR;
extern unsigned volatile short Wall_FL;

volatile char BlockWall_R = 0;
volatile char BlockWall_L = 0;
volatile char BlockWall_F = 0;

extern volatile float max_speed;
extern volatile float accel;

extern volatile int WallCtrlEnbl;
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

extern volatile float offsetplus_st;
extern volatile int offsetcnt_st;


float searchmaxv = 0;
float searchacc = 0;

float searchknownmaxv = 0;
float searchknownacc = 0;

float searchslaacc = 0;
float searchslav = 0;
float searchbeforeslaoffsetr = 0;
float searchafterslaoffsetr = 0;
float searchbeforeslaoffsetl = 0;
float searchafterslaoffsetl = 0;
float searchsladegr = 0;
float searchsladegl = 0;
float searchslaminv = 0;


int searched = 0;
int allsearched = 0;
int goalwall = 0;


extern int mx;
extern int my;
extern volatile int head;
extern volatile int goalhead;

/*	壁データはmap[][]に記録される
	壁データ記録方法
	壁の位置 0bit:上 1bit:右 2bit:下 3bit:左	 1:壁有り 0:壁無し
	壁の探索 4bit:上 5bit:右 6bit:下 7bit:左	 1:済み	 0:未探索		*/
extern unsigned short smap[MAZESIZE_X][MAZESIZE_Y];	//等高線map
extern unsigned char map[MAZESIZE_X][MAZESIZE_Y];
//extern unsigned char mapb[MAZESIZE_X][MAZESIZE_Y];
extern unsigned char allsearchflag;
unsigned char savetask;

extern float turnnumL;
extern float turnnumR;


extern volatile int searching_flag;

extern volatile int			run_mode;

extern volatile int firstread;

extern volatile int slafwallflag;
extern volatile int slaswallflag;
extern volatile int turnfwallflag;

extern volatile float			len_mouse;

volatile int firstsla = 0;

extern volatile char comb;

extern volatile char BatCheckSW;

extern unsigned volatile long		searchtimer;					//1mSごとにカウントアップされる変数.

extern volatile char stopped;

extern volatile float len_true;

/*char wallDataSave[5] = {0,0,0,0,0};
unsigned char dataSaveX[5] = {0,0,0,0,0};
unsigned char dataSaveY[5] = {0,0,0,0,0};*/

extern volatile char g_Sensor_Flag;

extern volatile char StopFlagR;
extern volatile char StopFlagL;

extern volatile float sla_fwall_a;

extern void wait_ms(volatile int waittime);

extern volatile char sincurve;

extern volatile char spd_cnt;
extern volatile char set_param;

extern volatile char R_Ctrl_Disable;
extern volatile char L_Ctrl_Disable;	//壁切れ後しばらくトレースしない
extern volatile float Dis_to_Able;
extern volatile float Dis_to_Able2;	//壁切れ後しばらくトレースしない

extern volatile float FanVolt;	//吸引ファンの印加電圧
extern volatile float			Duty_fan;
extern volatile char FanEnbl;	//吸引ファンの印加電圧

extern volatile float R_Raw_Err;	//センサ生値のエラー
extern volatile float L_Raw_Err;
extern volatile float Sensor_Raw_Err;


void wallCtrlReset(void){
	WallCtrlErrP = 0;
	WallCtrlErr = 0;
	Sensor_R_Err = 0;
	Sensor_L_Err = 0;
	WallCtrlD = 0;
	WallCtrlI = 0;
	WallCtrlOmegaP = 0;
	WallCtrlOmega = 0;
	R_Raw_Err = 0;
	L_Raw_Err = 0;
	Sensor_Raw_Err = 0;
}

void slaadachi(int tx,int ty,int goorback){	//足立法探索
	stopped = 0;
	int head0 = 0;	//進む方向や方角
	int hhead0 = 0;
	int temp0 = 0;	//mapdata
//	int temp2 = 0;
	int ttemp0 = 0;
	int s0 = 0;	//最適値
	int s1 = 0;	//調査地
	int ss0 = 0;
	int ss1 = 0;
	static int searchmiss[5];
	int whichmiss = 0;
	int mxbefore = 0;
	int mybefore = 0;
//	int goalwall = 0;
	int xtmp = 0,ytmp = 0;
	int knownseccnt = 0;
	float thinktime = 0;

	R_Ctrl_Disable = 0;
	L_Ctrl_Disable = 0;	//壁切れ後しばらくトレースしない
	Dis_to_Able = 0;
	Dis_to_Able2 = 0;	//壁切れ後しばらくトレースしない

	StopFlagR = 0;
	StopFlagL = 0;
	resetCurrent();
	g_Sensor_Flag = 1;
	if(goorback == 1){
		wait_ms(STARTWAIT/2);
	}
	if(FanVolt != 0){
		FanEnbl = 1;
	}
	if(goorback == 1 || FanVolt != 0){
		wait_ms(STARTWAIT/2);
	}
	wallCtrlReset();
	BatCheckSW = 0;

	if(goorback == 1){
		allsearchflag = 0;
	}
	else{
		allsearchflag = 1;
	}

	if(goorback == 1){
		goalwallset(tx,ty,GOALSIZE);
		len_true = 0;
		spd_cnt = 1;
	}
	else{
	}

	if(goorback == 1){
	searchtimer = 0;
	Saved.qTail = 0;
	}
	searching_flag = 1;
	allsearched = 0;
	searched = 0;
//	back(SEARCHBACK,searchacc,searchmaxv,0);
	firstread = 0;
	offsetplus_st = 0;
	offsetcnt_st = 0;
	if(goorback == 1){
		straight(HALF_SECTION+WALLTOMIDDLE/*+SEARCHBACK*/,searchacc,searchmaxv,searchmaxv,FLAG_ON,1);			//まず、半区画進む
	}
	else{
		straight(HALF_SECTION/*+SEARCHBACK*/,searchacc,searchmaxv,searchmaxv,FLAG_ON,1);			//まず、半区画進む
	}
	firstsla = 1;
	//firstread = 0;
	while(1){
		stopped = 0;
		StopFlagR = 0;
		StopFlagL = 0;
		if(SWITCH_ONOFF() == SW_ON){	//非常用スイッチが押されたら緊急停止
			mx = 0;
			my = 0;
			head = 0;
			run_mode = TEST_MODE;
			Motor_StopPWM();
			searching_flag = 0;
			savetask = 0;
			if(goorback == 1){
				map[searchmiss[0]%100][(searchmiss[0]-(searchmiss[0]%100))/100] = 0;
				map[searchmiss[1]%100][(searchmiss[1]-(searchmiss[0]%100))/100] = 0;
				map[searchmiss[2]%100][(searchmiss[2]-(searchmiss[0]%100))/100] = 0;
				map[searchmiss[3]%100][(searchmiss[3]-(searchmiss[0]%100))/100] = 0;
				map[searchmiss[4]%100][(searchmiss[4]-(searchmiss[0]%100))/100] = 0;
			}
			else if(goorback==0){
				recoverymap();
			}
			wait_ms(1000);
			searched = 0;
			BatCheckSW = 1;
			//wait_ms(1000);
			return;	//帰り対策
		}
		if(savetask == 1){
			mx = 0;
			my = 0;
			head = 0;
			run_mode = TEST_MODE;
			Motor_StopPWM();
			searching_flag = 0;
			if(goorback == 1){
				map[searchmiss[0]%100][(searchmiss[0]-(searchmiss[0]%100))/100] = 0;
				map[searchmiss[1]%100][(searchmiss[1]-(searchmiss[0]%100))/100] = 0;
				map[searchmiss[2]%100][(searchmiss[2]-(searchmiss[0]%100))/100] = 0;
				map[searchmiss[3]%100][(searchmiss[3]-(searchmiss[0]%100))/100] = 0;
				map[searchmiss[4]%100][(searchmiss[4]-(searchmiss[0]%100))/100] = 0;
			}
			else if(goorback==0){
				recoverymap();
			}
			searched = 0;
			BatCheckSW = 1;
			while(SWITCH_ONOFF() == SW_OFF);
			wait_ms(1000);
			savetask = 0;
			return;
		}
		if(head == 0){
			my++;
		}
		else if(head == 1){
			mx++;
		}
		else if(head == 2){
			my--;
		}
		else if(head == 3){
			mx--;
		}
		if(goorback == 1){
			mxbefore = mx;
			mybefore = my;
			if(whichmiss == 0){
				searchmiss[0] = (mx * 100) + my;
			}
			else if(whichmiss == 1){
				searchmiss[1] = (mx * 100) + my;
			}
			else if(whichmiss == 2){
				searchmiss[2] = (mx * 100) + my;
			}
			else if(whichmiss == 3){
				searchmiss[3] = (mx * 100) + my;
			}
			else if(whichmiss == 4){
				searchmiss[4] = (mx * 100) + my;
			}
			mx = mxbefore;
			my = mybefore;
			whichmiss++;
			if(whichmiss > 4){
				whichmiss = 0;
			}
		}
		//LEDmap(mx,my);//座標表示
		if((goorback == 0 && mx == tx && my == ty) || (GOALSIZE == 9 && goorback == 1 && (mx == tx || mx == tx+1 || mx == tx+2) && (my == ty || my == ty+1 || my == ty+2)) || (GOALSIZE == 4 && goorback == 1 && (mx == tx || mx == tx+1) && (my == ty || my == ty+1)) || (GOALSIZE == 1 && goorback == 1 && (mx == tx) && (my == ty))){  //goal
			makemap();
			if(goorback == 1){
			allsearchflag = 1;
			makesmap(0,0,0);
			if(smap[my][mx] == STEPMAX){
				allsearchflag = 0;
				allsearched = 1;
				makesmap(0,0,1);	//探索中は未探索区間壁なしで走行
			}
			if(stopped == 0){
				turnfwallflag = 1;
				straight(HALF_SECTION,searchacc,searchmaxv,0,FLAG_ON,0);			//まず、半区画進む
				turnfwallflag = 0;
				wait_ms(10);
				f_wall(FWALL_REF_R,FWALL_REF_L);
			}
			//
			temp0 = map[my][mx];
			s0 = 1024;
			if ((temp0 & 1) == 0){				//	北方向の区間の確認
				s1 = smap[my+1][mx] * 4 + 4;
				if(allsearched == 0){
					if ((map[my+1][mx] & 0x0f0) != 0x0f0){
						s1 = s1 - 2;	//探索フラグを歩数マップの次に優先
					}
				}
				else{
				}
				if (head == 0){
					s1 = s1 - 1;	//直進>旋回
				}
				if (s1 < s0) {
					s0 = s1;
					head0 = 0;
				}	//今までより有利ならそっちに転向
			}
			if ((temp0 & 2) == 0){				//	東方向の区間の確認
				s1 = smap[my][mx+1] * 4 + 4;
				if(allsearched == 0){
					if ((map[my][mx+1] & 0x0f0) != 0x0f0){
						s1 = s1 - 2;
					}
				}
				else{
				}
				if (head == 1){
					s1 = s1 - 1;
				}
				if (s1 < s0) {
					s0 = s1;
					head0 = 1;
				}
			}
			if ((temp0 & 4) == 0){				//	南方向の区間の確認
				s1 = smap[my-1][mx] * 4 + 4;
				if(allsearched == 0){
					if ((map[my-1][mx] & 0x0f0) != 0x0f0){
						s1 = s1 - 2;
					}
				}
				else{
				}
				if (head == 2){
					s1 = s1 - 1;
				}
				if (s1 < s0) {
					s0 = s1;
					head0 = 2;
				}
			}
			if ((temp0 & 8) == 0){				//	西方向の区間の確認
				s1 = smap[my][mx-1] * 4 + 4;
				if(allsearched == 0){
					if ((map[my][mx-1] & 0x0f0) != 0x0f0){
						s1 = s1 - 2;
					}
				}
				else{
				}
				if (head == 3){
					s1 = s1 - 1;
				}
				if (s1 < s0) {
					s0 = s1;
					head0 = 3;
				}
			}

			head0 = (head0 + 4 - head) & 3;			// 移動する歩行を決定
			//
			if((Wall_FR == WALL_ON && Wall_FL == WALL_ON) && (tx != 0 && ty != 0)){
				goalwall = 1;
			}
		/*	turn_fwall2();
			//back(SEARCHBACK,searchacc,searchmaxv,0);*/
			head = head + 2;
			if(head > 3){
				head = head - 4;
			}
			if(head < 0){
				head = head + 4;	//向きの訂正
			}
			//
			if(goorback == 1){
			goalhead = head;
			}
			//
			head = head - 2;
			if(head > 3){
				head = head - 4;
			}
			if(head < 0){
				head = head + 4;	//向きの訂正
			}
			//
			if(head0 == 0){
				wait_ms(10);
				//turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
			}
			else if(head0 == 1){
				wait_ms(10);
				turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
				head++;
				if(head > 3){
					head = head - 4;
				}
				if(head < 0){
					head = head + 4;	//向きの訂正
				}
			}
			else if(head0 == 2){
				wait_ms(10);
				turn_fwall2();
				head+=2;
				if(head > 3){
					head = head - 4;
				}
				if(head < 0){
					head = head + 4;	//向きの訂正
				}
			}
			else if(head0 == 3){
				wait_ms(10);
				turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
				head--;
				if(head > 3){
					head = head - 4;
				}
				if(head < 0){
					head = head + 4;	//向きの訂正
				}
			}
			//
			wait_ms(TURNWAIT);
			}
			else if(goorback == 0){
				turnfwallflag = 1;
				straight(HALF_SECTION,searchacc,searchmaxv,0,FLAG_ON,0);			//まず、半区画進む
				turnfwallflag = 0;
				if((Wall_FR == WALL_ON && Wall_FL == WALL_ON) && (tx != 0 && ty != 0)){
					goalwall = 1;
				}
				turn_fwall2();
				//back(SEARCHBACK,searchacc,searchmaxv,0);
				head = head + 2;
				if(head > 3){
					head = head - 4;
				}
				if(head < 0){
					head = head + 4;	//向きの訂正
				}
			}
			if(goorback == 1){
				mapsave();
			}
			savetask = 0;
			searching_flag = 0;
			searched = 1;
			BatCheckSW = 1;
			run_mode = TEST_MODE;
			Motor_StopPWM();
			//if(allsearched == 1){
			//	back(1,1.0,0.2,0.2);
			//}
			return;
		}
		makemap();
		if(allsearched == 1 || (goorback == 0 && searchtimer > SEARCHTIMEMAX*60000)){
			//makesmap(tx,ty,1);	//探索中は未探索区間壁なしで走行
			thinktime = fabsf(len_mouse);
			turnfwallflag = 1;
			straight(HALF_SECTION-thinktime,searchacc,searchmaxv,0,FLAG_ON,0);			//まず、半区画進む
			turnfwallflag = 0;
			wait_ms(100);
			f_wall(FWALL_REF_R,FWALL_REF_L);
			wait_ms(100);
			searching_flag = 0;
			run_mode = TEST_MODE;
			Motor_StopPWM();
			if(allsearched == 1){
				//searchend1();
			}
			else{
				//searchend2();
			}
			BatCheckSW = 1;
			return;
		}
		else{
			makesmap(tx,ty,0);	//探索中は未探索区間壁なしで走行
		}
		/*if(allsearched == 0 || (searchtimer < SEARCHTIMEMAX && goorback == 0)){
			makesmap(tx,ty,0);	//探索中は未探索区間壁なしで走行
		}
		else{
			makesmap(tx,ty,1);	//探索中は未探索区間壁なしで走行
		}*/

		if(allsearchflag == 1 && smap[my][mx] == STEPMAX){
		//	straight(HALF_SECTION,searchacc,searchmaxv,0,FLAG_ON,0);			//まず、半区画進む
		//	run_mode = TEST_MODE;
		//	MTU0.TGRA = 0;
		//	MTU0.TGRC = 0;
			allsearchflag = 0;
			allsearched = 1;
			makesmap(tx,ty,1);	//探索中は未探索区間壁なしで走行
		}

		/* 回りの4区間の内、その方向に壁がなくて、目的地に一番近い
		   区間に移動する。ただし、移動できる一番近い区間が複数ある
		   場合は(未探索区間,直進)(未探索区間,旋回)(既探索区間,直進）
		   (既探索区間,旋回）の順で選択する								*/

		temp0 = map[my][mx];
		s0 = 1024;
		if ((temp0 & 1) == 0){				//	北方向の区間の確認
			s1 = smap[my+1][mx] * 4 + 4;
			if(allsearched == 0){
				if ((map[my+1][mx] & 0x0f0) != 0x0f0){
					s1 = s1 - 2;	//探索フラグを歩数マップの次に優先
				}
			}
			else{
			}
			if (head == 0){
				s1 = s1 - 1;	//直進>旋回
			}
			if (s1 < s0) {
				s0 = s1;
				head0 = 0;
			}	//今までより有利ならそっちに転向
		}
		if ((temp0 & 2) == 0){				//	東方向の区間の確認
			s1 = smap[my][mx+1] * 4 + 4;
			if(allsearched == 0){
				if ((map[my][mx+1] & 0x0f0) != 0x0f0){
					s1 = s1 - 2;
				}
			}
			else{
			}
			if (head == 1){
				s1 = s1 - 1;
			}
			if (s1 < s0) {
				s0 = s1;
				head0 = 1;
			}
		}
		if ((temp0 & 4) == 0){				//	南方向の区間の確認
			s1 = smap[my-1][mx] * 4 + 4;
			if(allsearched == 0){
				if ((map[my-1][mx] & 0x0f0) != 0x0f0){
					s1 = s1 - 2;
				}
			}
			else{
			}
			if (head == 2){
				s1 = s1 - 1;
			}
			if (s1 < s0) {
				s0 = s1;
				head0 = 2;
			}
		}
		if ((temp0 & 8) == 0){				//	西方向の区間の確認
			s1 = smap[my][mx-1] * 4 + 4;
			if(allsearched == 0){
				if ((map[my][mx-1] & 0x0f0) != 0x0f0){
					s1 = s1 - 2;
				}
			}
			else{
			}
			if (head == 3){
				s1 = s1 - 1;
			}
			if (s1 < s0) {
				s0 = s1;
				head0 = 3;
			}
		}

		head0 = (head0 + 4 - head) & 3;			// 移動する歩行を決定
			//stepreset();

		if(head0 == 0){	//直進するなら

			if(stopped == 1){
				firstread = 0;
			//	offsetplus_st = 0;
			//	offsetcnt_st = 0;
				if(BlockWall_R == 0){
					StopFlagR = 1;
				}
				if(BlockWall_L == 0){
					StopFlagL = 1;
				}
				straight(HALF_SECTION/*+SEARCHBACK*/,searchacc,searchmaxv,searchmaxv,FLAG_ON,1);			//まず、半区画進む
				//f_wall(FWALL_REF_R,FWALL_REF_L);
				stopped = 0;
				StopFlagR = 0;
				StopFlagL = 0;
			}
			else{
			xtmp = mx;
			ytmp = my;
			knownseccnt = 0;
			while(1){
				if((map[ytmp][xtmp] & 0xf0) != 0xf0){
					break;
				}
				else{
					ttemp0 = map[ytmp][xtmp];
					ss0 = 1024;
					if ((ttemp0 & 1) == 0){				//	北方向の区間の確認
						ss1 = smap[ytmp+1][xtmp] * 4 + 4;
						if ((map[ytmp+1][xtmp] & 0x0f0) != 0x0f0){
							ss1 = ss1 - 2;	//探索フラグを歩数マップの次に優先
						}
						if (head == 0){
							ss1 = ss1 - 1;	//直進>旋回
						}
						if (ss1 < ss0) {
							ss0 = ss1;
							hhead0 = 0;
						}	//今までより有利ならそっちに転向
					}
					if ((ttemp0 & 2) == 0){				//	東方向の区間の確認
						ss1 = smap[ytmp][xtmp+1] * 4 + 4;
						if ((map[ytmp][xtmp+1] & 0x0f0) != 0x0f0){
							ss1 = ss1 - 2;
						}
						if (head == 1){
							ss1 = ss1 - 1;
						}
						if (ss1 < ss0) {
							ss0 = ss1;
							hhead0 = 1;
						}
					}
					if ((ttemp0 & 4) == 0){				//	南方向の区間の確認
						ss1 = smap[ytmp-1][xtmp] * 4 + 4;
						if ((map[ytmp-1][xtmp] & 0x0f0) != 0x0f0){
							ss1 = ss1 - 2;
						}
						if (head == 2){
							ss1 = ss1 - 1;
						}
						if (ss1 < ss0) {
							ss0 = ss1;
							hhead0 = 2;
						}
					}
					if ((ttemp0 & 8) == 0){				//	西方向の区間の確認
						ss1 = smap[ytmp][xtmp-1] * 4 + 4;
						if ((map[ytmp][xtmp-1] & 0x0f0) != 0x0f0){
							ss1 = ss1 - 2;
						}
						if (head == 3){
							ss1 = ss1 - 1;
						}
						if (ss1 < ss0) {
							ss0 = ss1;
							hhead0 = 3;
						}
					}
					hhead0 = (hhead0 + 4 - head) & 3;			// 移動する歩行を決定
					if(hhead0 == 0){
						knownseccnt++;
					}
					else{
						break;
					}
					if(head == 0){
						ytmp++;
					}
					else if(head == 1){
						xtmp++;
					}
					else if(head == 2){
						ytmp--;
					}
					else if(head == 3){
						xtmp--;
					}
					if(xtmp < 0 || ytmp < 0 || xtmp > (MAZESIZE_X-1) || ytmp > (MAZESIZE_Y-1)){
						break;
					}
				}
			}
			if(knownseccnt >= 2){
				firstread = 0;
				offsetplus_st = 0;
				offsetcnt_st = 0;
				thinktime = fabsf(len_mouse);
				//straight(ONESECTION*knownseccnt-thinktime,searchknownacc,searchknownmaxv,searchmaxv,FLAG_ON,3);			//まず、半区画進む
				int p;
				for(p = 0; p < knownseccnt;p++){
					if(head == 0){
						if((map[my+p][mx]&0x22) == 0x20 && (map[my+p][mx]&0x88) == 0x80){
							comb = 3;
						}
						else if((map[my+p][mx]&0x22) == 0x22 && (map[my+p][mx]&0x88) == 0x80){
							comb = 2;
						}
						else if((map[my+p][mx]&0x22) == 0x20 && (map[my+p][mx]&0x88) == 0x88){
							comb = 1;
						}
						else if((map[my+p][mx]&0x22) == 0x22 && (map[my+p][mx]&0x88) == 0x88){
							comb = 0;
						}
					}
					else if(head == 1){
						if((map[my][mx+p]&0x44) == 0x40 && (map[my][mx+p]&0x11) == 0x10){
							comb = 3;
						}
						else if((map[my][mx+p]&0x44) == 0x44 && (map[my][mx+p]&0x11) == 0x10){
							comb = 2;
						}
						else if((map[my][mx+p]&0x44) == 0x40 && (map[my][mx+p]&0x11) == 0x11){
							comb = 1;
						}
						else if((map[my][mx+p]&0x44) == 0x44 && (map[my][mx+p]&0x11) == 0x11){
							comb = 0;
						}
					}
					else if(head == 2){
						if((map[my-p][mx]&0x88) == 0x80 && (map[my-p][mx]&0x22) == 0x20){
							comb = 3;
						}
						else if((map[my-p][mx]&0x88) == 0x88 && (map[my-p][mx]&0x22) == 0x20){
							comb = 2;
						}
						else if((map[my-p][mx]&0x88) == 0x80 && (map[my-p][mx]&0x22) == 0x22){
							comb = 1;
						}
						else if((map[my-p][mx]&0x88) == 0x88 && (map[my-p][mx]&0x22) == 0x22){
							comb = 0;
						}
					}
					else if(head == 3){
						if((map[my][mx-p]&0x11) == 0x10 && (map[my][mx-p]&0x44) == 0x40){
							comb = 3;
						}
						else if((map[my][mx-p]&0x11) == 0x11 && (map[my][mx-p]&0x44) == 0x40){
							comb = 2;
						}
						else if((map[my][mx-p]&0x11) == 0x10 && (map[my][mx-p]&0x44) == 0x44){
							comb = 1;
						}
						else if((map[my][mx-p]&0x11) == 0x11 && (map[my][mx-p]&0x44) == 0x44){
							comb = 0;
						}
					}
					firstread = 0;
					if(p == 0){
						straight(ONESECTION-thinktime,searchknownacc,searchknownmaxv,searchknownmaxv,FLAG_ON,1);
					}
					else if(p == knownseccnt-1){
						straight(ONESECTION,searchknownacc,searchknownmaxv,searchmaxv,FLAG_ON,1);
					}
					else{
						straight(ONESECTION,searchknownacc,searchknownmaxv,searchknownmaxv,FLAG_ON,1);
					}
					comb = 0;
				}
				if(head == 0){
					my += (knownseccnt-1);
				}
				else if(head == 1){
					mx += (knownseccnt-1);
				}
				else if(head == 2){
					my -= (knownseccnt-1);
				}
				else if(head == 3){
					mx -= (knownseccnt-1);
				}
			}
			else{
				firstread = 0;
				offsetplus_st = 0;
				offsetcnt_st = 0;
				thinktime = fabsf(len_mouse);
				if(BlockWall_R == 0 && BlockWall_L == 0){
					comb = 3;
				}
				else if(BlockWall_R == 1 && BlockWall_L == 0){
					comb = 2;
				}
				else if(BlockWall_R == 0 && BlockWall_L == 1){
					comb = 1;
				}
				else if(BlockWall_R == 1 && BlockWall_L == 1){
					comb = 0;
				}
				straight(ONESECTION-thinktime,searchacc,searchmaxv,searchmaxv,FLAG_ON,1);			//まず、半区画進む
				comb = 0;

			}
			}
			//firstread = 0;
			firstsla = 0;
		}
		else if(head0 == 1){	//右旋回
		if(stopped == 1){
			f_wall(FWALL_REF_R,FWALL_REF_L);
			turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
			firstread = 0;
			offsetplus_st = 0;
			offsetcnt_st = 0;
			//len_true = 0;
			if(/*BlockWall_R == 0 &&*/ BlockWall_F == 0){
				comb = 3;
			}
			else if(/*BlockWall_R == 0 && */BlockWall_F == 1){
				comb = 1;
			}
			//if(BlockWall_R == 0){
				StopFlagR = 1;
			//}
			if(BlockWall_F == 0){
				StopFlagL = 1;
			}
			wait_ms(TURNWAIT);
			if((BlockWall_F == 1 && Wall_L == WALL_OFF) && BlockWall_L == 0){
				back(SEARCHBACK,searchacc,searchmaxv,0);
				straight(HALF_SECTION+SEARCHBACK,searchacc,searchmaxv,searchmaxv,FLAG_ON,1);
			}else{
				straight(HALF_SECTION/*+SEARCHBACK*/,searchacc,searchmaxv,searchmaxv,FLAG_ON,1);
			}
			firstsla = 1;
			stopped = 0;
			StopFlagR = 0;
			StopFlagL = 0;
			comb = 0;
		}
		else{
			//右折
		thinktime = fabsf(len_mouse);
		if(thinktime > searchbeforeslaoffsetr-1.0){
			thinktime = searchbeforeslaoffsetr-1.0;
		}
		slafwallflag = 1;
		if(firstsla == 1){
			slaswallflag = 0;
		}else{
			slaswallflag = 0;//1;
		}
		if(sincurve == 0){
		sla(searchsladegr,searchslaacc,searchslav,searchmaxv,RIGHT,searchbeforeslaoffsetr-thinktime,searchafterslaoffsetr,searchslaminv);
		}
		else{
		sla2(searchsladegr,searchslaacc,searchslav,searchmaxv,RIGHT,searchbeforeslaoffsetr-thinktime,searchafterslaoffsetr,searchslaminv);
		}
		slafwallflag = 0;
		slaswallflag = 0;
		offsetplus_st = 0;
			offsetcnt_st = 0;
			firstread = 0;
			firstsla = 0;
			//len_true = 0;
		}
			head++;
			if(head > 3){
				head = head - 4;
			}
			if(head < 0){
				head = head + 4;	//向きの訂正
			}
		}
		else if(head0 == 2){	//180旋回

			if(stopped == 0){
			thinktime = fabsf(len_mouse);
			turnfwallflag = 1;
			straight(HALF_SECTION-thinktime,searchacc,searchmaxv,0,FLAG_ON,0);			//まず、半区画進む
			turnfwallflag = 0;
			}
			else{
				stopped = 0;
			}
			firstread = 0;
			turn_fwall2();
			//len_true = 0;
			head = head + 2;
			if(head > 3){
				head = head - 4;
			}
			if(head < 0){
				head = head + 4;	//向きの訂正
			}
		//	wait_ms(TURNWAIT);
		//	back(SEARCHBACK,searchacc,searchmaxv,0);
			//firstread = 0;
			//firstsla = 0;
			firstread = 0;
			offsetplus_st = 0;
			offsetcnt_st = 0;
			if(BlockWall_L == 0 && BlockWall_R == 0){
					comb = 3;
			}
			else if(BlockWall_L == 1 && BlockWall_R == 0){
				comb = 2;
			}
			else if(BlockWall_L == 0 && BlockWall_R == 1){
				comb = 1;
			}
			else if(BlockWall_L == 1 && BlockWall_R == 1){
				comb = 0;
			}
			if(BlockWall_L == 0){
				StopFlagR = 1;
			}
			if(BlockWall_R == 0){
				StopFlagL = 1;
			}
			//
			if(((BlockWall_R == 1 && Wall_L == WALL_OFF) || (BlockWall_L == 1 && Wall_R == WALL_OFF)) && BlockWall_F == 0){
				back(SEARCHBACK,searchacc,searchmaxv,0);
				straight(HALF_SECTION+SEARCHBACK,searchacc,searchmaxv,searchmaxv,FLAG_ON,1);
			}
			else{
			//
				straight(HALF_SECTION/*+SEARCHBACK*/,searchacc,searchmaxv,searchmaxv,FLAG_ON,1);			//まず、半区画進む
			}
			StopFlagR = 0;
			StopFlagL = 0;
			firstsla = 1;
			comb = 0;
		}
		else if(head0 == 3){	//左旋回
		if(stopped == 1){
			f_wall(FWALL_REF_R,FWALL_REF_L);
			turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
			//len_true = 0;
			firstread = 0;
			offsetplus_st = 0;
			offsetcnt_st = 0;
			if(BlockWall_F == 0/* && BlockWall_L == 0*/){
					comb = 3;
			}
			else if(BlockWall_F == 1/* && BlockWall_L == 0*/){
				comb = 2;
			}
			if(BlockWall_F == 0){
				StopFlagR = 1;
			}
			//if(BlockWall_R == 0){
				StopFlagL = 1;
			//}
			if((BlockWall_F == 1 && Wall_R == WALL_OFF) && BlockWall_R == 0){
				back(SEARCHBACK,searchacc,searchmaxv,0);
				straight(HALF_SECTION+SEARCHBACK,searchacc,searchmaxv,searchmaxv,FLAG_ON,1);
			}else{
				straight(HALF_SECTION/*+SEARCHBACK*/,searchacc,searchmaxv,searchmaxv,FLAG_ON,1);
			}
			firstsla = 1;
			stopped = 0;
			StopFlagR = 0;
			StopFlagL = 0;
			comb = 0;
		}
		else{
		thinktime = fabsf(len_mouse);
		if(thinktime > searchbeforeslaoffsetl-1.0){
			thinktime = searchbeforeslaoffsetl-1.0;
		}
		slafwallflag = 1;
		if(firstsla == 1){
			slaswallflag = 0;
		}else{
			slaswallflag = 0;//1;
		}
		if(sincurve == 0){
		sla(searchsladegl,searchslaacc,searchslav,searchmaxv,LEFT,searchbeforeslaoffsetl-thinktime,searchafterslaoffsetl,searchslaminv);
		}
		else{
		sla2(searchsladegl,searchslaacc,searchslav,searchmaxv,LEFT,searchbeforeslaoffsetl-thinktime,searchafterslaoffsetl,searchslaminv);

		}
		slafwallflag = 0;
		slaswallflag = 0;
		offsetplus_st = 0;
		offsetcnt_st = 0;
		firstread = 0;
		//len_true = 0;
		}
		head--;
			if(head > 3){
				head = head - 4;
			}
			if(head < 0){
				head = head + 4;	//向きの訂正
			}
		}
	}
}

void setpara_search(int a){
//	if(a == 0){
		FanVolt = flash1.fanV[a];//0;
		searchmaxv = flash1.maxV[a];//0.25;
		searchacc = flash1.acc[a];//1.5;
		searchknownmaxv = flash1.knownMaxV[a];//0.7;
		searchknownacc = flash1.knownAcc[a];//2.0;
		searchslaacc = flash1.slaAlpha[a];//261.8;
		searchslav = flash1.slaW[a];//10.5;
		searchbeforeslaoffsetr = flash1.offsetRA[a];//17.0;
		searchafterslaoffsetr = flash1.offsetRB[a];//14.0;
		searchbeforeslaoffsetl = flash1.offsetLA[a];//17.0;
		searchafterslaoffsetl = flash1.offsetLB[a];//14.0;
		searchslaminv = flash1.slaMinV[a];//0.2618;
		searchsladegr = flash1.degR[a];//90;
		searchsladegl = flash1.degL[a];//90;
		sla_fwall_a = flash1.fWall[a];//83.0;
//	}
/*	else if(a == 1){
		FanVolt = 0;
		searchmaxv = 0.3;
		searchacc = 1.5;
		searchknownmaxv = 0.7;
		searchknownacc = 2.0;
		searchslaacc = 261.8;
		searchslav = 10.5;
		searchbeforeslaoffsetr = 5.0;
		searchafterslaoffsetr = 15.0;
		searchbeforeslaoffsetl = 5.0;
		searchafterslaoffsetl = 15.0;
		searchslaminv = 0.2618;
		searchsladegr = 91.1;
		searchsladegl = 91.1;
		sla_fwall_a = 82.0;
	}
	else if(a == 2){
		FanVolt = 0;
		searchmaxv = 0.35;
		searchacc = 1.5;
		searchknownmaxv = 0.7;
		searchknownacc = 2.0;
		searchslaacc = 314.15;
		searchslav = 17.45;
		searchbeforeslaoffsetr = 13;
		searchafterslaoffsetr = 19;
		searchbeforeslaoffsetl = 13;
		searchafterslaoffsetl = 19;
		searchslaminv = 0.31415;
		searchsladegr = 91.5;
		searchsladegl = 91.5;
		sla_fwall_a = 85.0;

	}
	else if(a == 3){
		FanVolt = 1.0;
		searchmaxv = 0.4;
		searchacc = 2.0;
		searchknownmaxv = 0.8;
		searchknownacc = 3.0;
		searchslaacc = 349.06;
		searchslav = 15.7075;
		searchbeforeslaoffsetr = 8;
		searchafterslaoffsetr = 10;
		searchbeforeslaoffsetl = 8;
		searchafterslaoffsetl = 10;
		searchsladegr = 90;
		searchsladegl = 90;
		searchslaminv = 0.025;
		sla_fwall_a = 80.0;
	}*/
}
