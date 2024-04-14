#include "index.h"
#include "CMT.h"
#include"Interface.h"
#include"common.h"
#include"run.h"
#include"search.h"
#include<math.h>
//#include<mathf.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h>

#define ARM_MATH_CM4
#include "arm_math.h"
#include "arm_const_structs.h"

extern volatile int V_bat;
extern volatile short V_bat_ref;
extern unsigned volatile short Wall_R;
extern unsigned volatile short Wall_L;
extern unsigned volatile short Wall_FR;
extern unsigned volatile short Wall_FL;
extern volatile float Sensor_R;
extern volatile float Sensor_L;
extern volatile float Sensor_FR;
extern volatile float Sensor_FL;
extern volatile float Sensor_R_Dis;
extern volatile float Sensor_L_Dis;
extern volatile float Sensor_FR_Dis;
extern volatile float Sensor_FL_Dis;
extern volatile float FR_Dis_Ave;
extern volatile float FL_Dis_Ave;

//エンコーダ角度系のグローバル変数
extern unsigned volatile int	locate_l;				//現在の車軸位置	[無次元]
extern unsigned volatile int	locate_r;				//現在の車軸位置	[無次元]
extern unsigned volatile int	before_locate_r;			//過去の車軸位置	[無次元]
extern unsigned volatile int	before_locate_l;			//過去の車軸位置	[無次元]
extern volatile int	diff_pulse_r;				//車軸位置の微分値(車軸の回転速度[pulse/ms])
extern volatile int	diff_pulse_l;				//車軸位置の微分値(車軸の回転速度[pulse/ms])
extern volatile int turnnuml;
extern volatile int turnnumr;
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
extern volatile float			I_degree;
extern volatile float			tar_ang_vel;				//目標角速度		[rad/s]
extern volatile float			tar_degree;				//目標角度		[deg]
extern volatile float			max_degree;				//旋回時の最大角度	[deg]
volatile float			max_degree_G = 0;
extern volatile float			start_degree;				//走行進入時の車体角度	[deg]
extern volatile float			max_ang_vel;				//最高角速度		[rad/s]
extern volatile float			max_ang_vel_sin;				//最高角速度		[rad/s]
extern volatile float			ang_acc;				//角加速度		[rad/ss]
extern volatile float			ang_acc_sin;				//角加速度		[rad/ss]
extern volatile float			accel;					//加速度		[m/ss]
extern volatile float			max_speed;				//最高速度		[m/s]
extern volatile float			min_speed;				//最高速度		[m/s]
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

extern volatile float I_len;

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

extern volatile float			mouse_x;
extern volatile float			mouse_y;

extern volatile int FF_Enbl;
extern volatile int FB_Enbl;

extern volatile int wallCutTask;
extern volatile int slafwallflag;
extern volatile int slaswallflag;

volatile int tracecnt = 0;

extern volatile int fwallnoflag;

extern volatile float offsetplus;
extern volatile int offsetcnt;
extern volatile float offsetplus_st;
extern volatile int offsetcnt_st;

extern volatile char BlockWall_R;
extern volatile char BlockWall_L;
extern volatile char BlockWall_F;

extern volatile char BatCheckSW;

extern volatile char noslaflag;
extern volatile int searching_flag;

extern float searchmaxv;
extern float searchacc;

extern float searchknownmaxv;
extern float searchknownacc;

extern float searchslaacc;
extern float searchslav;
extern float searchbeforeslaoffsetr;
extern float searchafterslaoffsetr;
extern float searchbeforeslaoffsetl;
extern float searchafterslaoffsetl;
extern float searchsladegr;
extern float searchsladegl;
extern float searchslaminv;
extern volatile int firstread;

extern volatile float beecombr;
extern volatile float beecombl;
extern volatile float prev_combr;
extern volatile float prev_combl;

extern volatile int combbb;

extern volatile float len_true;

extern volatile float kojima_kd;

volatile float ichi_x = 0;//オドメトリ用座標
volatile float ichi_y = 0;
volatile float tar_ichi_x = 0;//オドメトリ用座標
volatile float tar_ichi_y = 0;

volatile int tsin = 0;
volatile char accflag = 0;

volatile int tsin2 = 0;
volatile float accel_sin = 0;
volatile float max_speed_sin = 0;
volatile char accflag2 = 0;

extern float searchacc;

extern volatile char saitaning;
//extern volatile char yaba;

extern volatile char longst;

extern volatile float I_fwall;
volatile int fwallcnt = 0;


extern unsigned int slantingtask;

extern volatile char R_Ctrl_Disable;
extern volatile char L_Ctrl_Disable;	//壁切れ後しばらくトレースしない
extern volatile float Dis_to_Able;
extern volatile float Dis_to_Able2;	//壁切れ後しばらくトレースしない

volatile float Nap_a = 0;
volatile float Nap_n = 0;
volatile float Nap_h = 0;
volatile float Nap_x = 0;
volatile char Nap_flag = 0;
volatile float Nap_b = 0;

volatile char anteiflag = 0;
volatile long anteitimer = 0;//整定判定用タイマー

volatile char firstgo = 0;

//ログ用のグローバル変数
extern volatile long			log_timer;				//ログ取りようのタイマ
extern volatile int			log_flag;				//ログ取得のタイミング
extern volatile float			V_r;
extern volatile float			V_l;
volatile float vvv_r = 0;
volatile float vvv_l = 0;

extern volatile char lastslant;

void motor_setVoltage(float v,unsigned char mode){
	run_mode = VOLT_MODE;
	FF_Enbl = 0;
	FB_Enbl = 0;
	log_timer = 0;
	log_flag = 1;
	LED2_ON();
	if(mode == 0){
		vvv_r = v;
		vvv_l = v;
	}
	else if(mode == 1){
		vvv_r = v;
		vvv_l = -v;
	}
	HAL_Delay(1000);
	log_flag = 0;
	LED2_OFF();
	run_mode = TEST_MODE;
	Motor_StopPWM();
}

void go(float len,float acc,float dec,float minv,float maxv,float firstv,float endv,unsigned char wallctrlsw){
	//if(wallctrlsw == 1){
	//	straight2(len,acc,dec,maxv,firstv,endv,wallctrlsw,5);
	//}
	//else{
		//straight(len,acc,maxv,endv,wallctrlsw,5);
		straight3(len,acc,dec,maxv,endv,wallctrlsw,5);
	//}
}

void straight(float len, float acc, float max_sp, float end_sp,int wallctrlsw,int wallcutsw){
	if(len > 0){
	if(max_sp >= 1.5f){
		V_bat_ref = V_bat;
	}
	float accel2 = acc;
	kojima_kd = 0;
	len_mouse = 0;
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	I_speed2 = 0;
	I_len = 0;
	//degree = 0;
	beecombr = 0;
	beecombl = 0;
	prev_combr = 0;
	prev_combl = 0;
	combbb = 0;
	//走行モードを直線にする
	run_mode = STRAIGHT_MODE;
	WallCtrlEnbl = wallctrlsw;
	wallCutTask = wallcutsw;
	FF_Enbl = 1;
	FB_Enbl = 1;
	//目標距離をグローバル変数に代入する
	len_target = len;
	//目標速度を設定
	end_speed = end_sp;
	//加速度を設定
	accel = acc;
	//最高速度を設定
	max_speed = max_sp;
	ang_acc = 0;

	//while(speed < max_speed);
	//accel = 0;
	//モータ出力をON
	//MOT_POWER_ON;

	if(end_speed == 0){	//最終的に停止する場合
		//減速処理を始めるべき位置まで加速、定速区間を続行
		while( ((len_target -10.0f) - len_mouse) >  1000.0f*((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed))/(float)(2.0f*accel2)){
			/*if(tar_speed >= 2.5){
				if(accel2 >= 10.0){
					accel = accel2/1.5;
				}
				else{
					accel = accel2/1.25;
				}
			}*/
		}
		//減速処理開始
		accel = -acc;					//減速するために加速度を負の値にする
		while(len_mouse < len_target -1.0f){		//停止したい距離の少し手前まで継続
			//一定速度まで減速したら最低駆動トルクで走行
			if(tar_speed <= MIN_SPEED){	//目標速度が最低速度になったら、加速度を0にする
				accel = 0;
				tar_speed = MIN_SPEED;
			}
		}
		accel = 0;
		tar_speed = 0;
		//速度が0以下になるまで逆転する
		while(speed >= 0.0f);

	}else{
		//減速処理を始めるべき位置まで加速、定速区間を続行
		while( ((len_target-10.0f) - len_mouse) >  1000.0f*((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed))/(float)(2.0f*accel2)){
			/*if(tar_speed >= 2.5){
				if(accel2 >= 10.0){
					accel = accel2/1.5;
				}
				else{
					accel = accel2/1.25;
				}
			}*/
		}
		//減速処理開始
		accel = -acc;					//減速するために加速度を負の値にする
		while(len_mouse < len_target){		//停止したい距離の少し手前まで継続
			//一定速度まで減速したら最低駆動トルクで走行
			if(tar_speed <= end_speed){	//目標速度が最低速度になったら、加速度を0にする
				accel = 0;
				//tar_speed = end_speed;
			}
		}
	}
	//加速度を0にする
	accel = 0;
	//現在距離を0にリセット
	len_mouse = 0;
//	len_true = 0;
	beecombr = 0;
	beecombl = 0;
	prev_combr = 0;
	prev_combl = 0;
	combbb = 0;
	kojima_kd = 0;
	I_len = 0;
	}
	else{
	}
	V_bat_ref = 0;
}

void straight3(float len, float acc,float dec, float max_sp, float end_sp,int wallctrlsw,int wallcutsw){
	if(len > 0){
	if(max_sp >= 1.5f){
		V_bat_ref = V_bat;
	}
	//float accel2 = acc;
	float dec2 = dec;
	//short antei = 0;
	kojima_kd = 0;
	len_mouse = 0;
	if(saitaning == 0){
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	I_speed2 = 0;
	I_len = 0;
	}
	//degree = 0;
	beecombr = 0;
	beecombl = 0;
	prev_combr = 0;
	prev_combl = 0;
	combbb = 0;
	//走行モードを直線にする
	run_mode = STRAIGHT_MODE;
	WallCtrlEnbl = wallctrlsw;
	wallCutTask = wallcutsw;
	FF_Enbl = 1;
	FB_Enbl = 1;
	//目標距離をグローバル変数に代入する
	len_target = len;
	//目標速度を設定
	end_speed = end_sp;
	//加速度を設定
	accel = 0;
	//最高速度を設定
	max_speed = max_sp;
	ang_acc = 0;

	if(longst == 1 && saitaning == 1 && firstgo == 0 && lastslant != 1){
		LED5_ON();
		anteiflag = 0;
		anteitimer = 0;
		while(1){//直進整定判定
			if(len_target-45.0f <= len_mouse){
				break;
				//直進終わり頃には強制加速
			}
			if(fabsf(tar_ang_vel) < ANTEI_ANG && fabsf(ang_vel) < ANTEI_ANG && fabsf(tar_ang_vel - ang_vel) < ANTEI_DIFF){
				anteiflag = 1;
				//wait_ms(1);
			}
			else{
				anteiflag = 0;
				anteitimer = 0;
			}
			if(anteitimer > (short)(ANTEI_TIME)){
				anteiflag = 0;
				anteitimer = 0;
				break;
			}
		}
		LED5_OFF();
	}

	accel = acc;
	//while(speed < max_speed);
	//accel = 0;
	//モータ出力をON
	//MOT_POWER_ON;

	if(end_speed == 0){	//最終的に停止する場合
		//減速処理を始めるべき位置まで加速、定速区間を続行
		while( ((len_target -10.0f) - len_mouse) >  1000.0f*((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed))/(float)(2.0f*dec2)){
			/*if(tar_speed >= 2.5){
				if(accel2 >= 10.0){
					accel = accel2/1.5;
				}
				else{
					accel = accel2/1.25;
				}
			}*/
		}
		//減速処理開始
		accel = -dec2;					//減速するために加速度を負の値にする
		while(len_mouse < len_target -1.0f){		//停止したい距離の少し手前まで継続
			//一定速度まで減速したら最低駆動トルクで走行
			if(tar_speed <= MIN_SPEED){	//目標速度が最低速度になったら、加速度を0にする
				accel = 0;
				tar_speed = MIN_SPEED;
			}
		}
		accel = 0;
		tar_speed = 0;
		//速度が0以下になるまで逆転する
		while(speed >= 0.0f);

	}else{
		if(longst == 1){
			if(dec2 >= 13.0f){
				//減速処理を始めるべき位置まで加速、定速区間を続行
				while( ((len_target-30.0f) - len_mouse) >  (1000.0f*((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed))/(float)(2.0f*dec2))){
					/*if(slantingtask == 0){
						if(tar_speed >= 2.5){
							if(accel2 >= 10.0){
								accel = accel2/1.5;
							}
							else{
								accel = accel2/1.25;
							}
						}
					}
					else if(slantingtask == 1 || slantingtask == 2){
						if(tar_speed >= 1.25){
							if(accel2 > 6.0){
								accel = accel2/1.75;
							}
							else{
								accel = accel2/1.5;
							}
						}
					}*/
				}
			}
			else{
				//減速処理を始めるべき位置まで加速、定速区間を続行
				while( ((len_target-10.0f) - len_mouse) >  (1000.0f*((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed))/(float)(2.0f*dec2))){
					/*if(slantingtask == 0){
						if(tar_speed >= 2.5){
							if(accel2 >= 10.0){
								accel = accel2/1.5;
							}
							else{
								accel = accel2/1.25;
							}
						}
					}
					else if(slantingtask == 1 || slantingtask == 2){
						if(tar_speed >= 1.25){
							if(accel2 > 6.0){
								accel = accel2/1.75;
							}
							else{
								accel = accel2/1.5;
							}
						}
					}*/
				}
			}
		}
		else{
			//減速処理を始めるべき位置まで加速、定速区間を続行
			while( ((len_target-10.0f) - len_mouse) >  1000.0f*((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed))/(float)(2.0f*dec2)){
			/*	if(tar_speed >= 2.5){
					if(accel2 >= 10.0){
						accel = accel2/1.5;
					}
					else{
						accel = accel2/1.25;
					}
				}*/
			}
		}
		//減速処理開始
		accel = -dec2;					//減速するために加速度を負の値にする
		while(len_mouse < len_target){		//停止したい距離の少し手前まで継続
			//一定速度まで減速したら最低駆動トルクで走行
			if(tar_speed <= end_speed){	//目標速度が最低速度になったら、加速度を0にする
				accel = 0;
				//tar_speed = end_speed;
			}
		}
	}
	//加速度を0にする
	accel = 0;
	//if(yaba == 1 || yaba == 2){
		tar_speed = end_speed;
	//}
	//現在距離を0にリセット
	len_mouse = 0;
//	len_true = 0;
	beecombr = 0;
	beecombl = 0;
	prev_combr = 0;
	prev_combl = 0;
	combbb = 0;
	kojima_kd = 0;
	I_len = 0;
	}
	else{
	}
	V_bat_ref = 0;
}


void straight2(float len, float acc, float dec, float max_sp, float start_sp, float end_sp,int wallctrlsw,int wallcutsw){
	if(len > 0){
	float x_acc = ((max_sp*max_sp)-(start_sp*start_sp))/(2.0f*acc);//加速区間
	float x_dec = ((max_sp*max_sp)-(end_sp*end_sp))/(2.0f*dec);//加速区間
	kojima_kd = 0;
	len_mouse = 0;
//	len_true = 0;
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	I_speed2 = 0;
	I_len = 0;
	//degree = 0;
	beecombr = 0;
	beecombl = 0;
	prev_combr = 0;
	prev_combl = 0;
	combbb = 0;
	tsin2 = 0;
	accel_sin = 0;
	max_speed_sin = 0;
	accflag2 = 0;
	//走行モードを直線にする
	run_mode = STRAIGHT2_MODE;
	WallCtrlEnbl = wallctrlsw;
	wallCutTask = wallcutsw;
	FF_Enbl = 1;
	FB_Enbl = 1;
	//目標距離をグローバル変数に代入する
	len_target = len;
	//目標速度を設定
	end_speed = end_sp;
	//加速度を設定
	accel = 0;
	if((x_acc+x_dec)*1000.0f > len_target){
	max_speed = sqrt(((len_target/1000.0f) + (start_sp * start_sp)/acc/2.0f + (end_speed * end_speed)/dec/2.0f) * 2.0f * acc * dec/(acc + dec));
	}
	else{
	//最高速度を設定
	max_speed = max_sp;
	}
	ang_acc = 0;

	//while(speed < max_speed);
	//accel = 0;
	//モータ出力をON
	//MOT_POWER_ON;

	tsin2 = 0;
	accflag2 = 1;
	accel_sin = acc;
	max_speed_sin = max_speed;
	while(tar_speed < max_speed);//{
	//LED1 = 1;
	//}

	//LED2 = 1;
	tsin2 = 0;
	accflag2 = 0;
	accel = 0;
	accel_sin = 0;
	max_speed_sin = max_speed;
	while((len_target-len_mouse-10.0f) > (1000.0f*(((max_speed * max_speed) - (end_speed * end_speed))/dec/2.0f)));//{
	//LED2 = 1;
	//}

	tsin2 = 0;
	accflag2 = 2;
	accel = 0;
	accel_sin = -dec;
	max_speed_sin = (-1.0f*max_speed);
	while(len_mouse < len_target){
		//LED3 = 1;
		//accel_sin = -1.0*((1000.0*(((max_speed * max_speed) - (end_speed * end_speed))/2.0))/(len_target-len_mouse));
		if(end_speed != 0 && tar_speed <= end_speed){
			//tar_speed = end_speed;
			accel = 0;
			accel_sin = 0;
		}
		if(end_speed == 0 && tar_speed <= MIN_SPEED){
			tar_speed = MIN_SPEED;
			accel = 0;
			accel_sin = 0;
		}
	}
	//LED4 = 1;
	tsin2 = 0;
	accel_sin = 0;
	max_speed_sin = 0;
	accflag2 = 0;
	//加速度を0にする
	accel = 0;
	//現在距離を0にリセット
	len_mouse = 0;
//	len_true = 0;
	beecombr = 0;
	beecombl = 0;
	prev_combr = 0;
	prev_combl = 0;
	combbb = 0;
	kojima_kd = 0;
	I_len = 0;
	}
	else{
	}
	V_bat_ref = 0;
}

void runoffseta(float len, float acc, float max_sp, float end_sp){
	if(len > 0){
		len_mouse = 0;
		I_tar_ang_vel = 0;
		I_ang_vel = 0;
		I_tar_speed = 0;
		I_speed = 0;
		I_speed2 = 0;

		offsetplus = 0;
		offsetcnt = 0;
		//degree = 0;
		//走行モードを直線にする
		run_mode = OFFSET_A_MODE;
		FF_Enbl = 1;
		FB_Enbl = 1;
		//目標距離をグローバル変数に代入する
		len_target = len;
		//目標速度を設定
		end_speed = end_sp;
		//加速度を設定
		accel = acc;
		//最高速度を設定
		max_speed = max_sp;
		ang_acc = 0;

		if(end_speed == 0){	//最終的に停止する場合
			//減速処理を始めるべき位置まで加速、定速区間を続行
			while( ((len_target -10.0f) - len_mouse) >  1000.0f*((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed))/(float)(2.0f*accel));
			//減速処理開始
			accel = -acc;					//減速するために加速度を負の値にする
			while(len_mouse < len_target -1.0f){		//停止したい距離の少し手前まで継続
				//一定速度まで減速したら最低駆動トルクで走行
				if(tar_speed <= MIN_SPEED){	//目標速度が最低速度になったら、加速度を0にする
					accel = 0;
					tar_speed = MIN_SPEED;
				}
			}
			accel = 0;
			tar_speed = 0;
			//速度が0以下になるまで逆転する
			while(speed >= 0.0f);

		}else{
			//減速処理を始めるべき位置まで加速、定速区間を続行
			while( ((len_target-10.0f) - len_mouse) >  1000.0f*((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed))/(float)(2.0f*accel));

			//減速処理開始
			accel = -acc;					//減速するために加速度を負の値にする
			while(len_mouse < len_target){		//停止したい距離の少し手前まで継続
				//一定速度まで減速したら最低駆動トルクで走行
				if(tar_speed <= end_speed){	//目標速度が最低速度になったら、加速度を0にする
					accel = 0;
					//tar_speed = end_speed;
				}
			}
		}
		//加速度を0にする
		accel = 0;
		//現在距離を0にリセット
		len_mouse = 0;
		wallCtrlReset();
		}
	else{
	}
}

void runoffsetb(float len, float acc, float max_sp, float end_sp){
	if(len > 0){
		len_mouse = 0;
		I_tar_ang_vel = 0;
		I_ang_vel = 0;
		I_tar_speed = 0;
		I_speed = 0;
		I_speed2 = 0;
				//degree = 0;
		//走行モードを直線にする
		run_mode = OFFSET_B_MODE;
		FF_Enbl = 1;
		FB_Enbl = 1;
		//目標距離をグローバル変数に代入する
		if(offsetcnt > 0 && slaswallflag == 1){
			len_target = (len + (offsetplus/(float)offsetcnt));
			//LED_B1 = 1;
		}
		else{
			if(offsetcnt_st > 0 && slaswallflag == 1){
				if(TURN_DIR == RIGHT){
					len_target = (len + (offsetplus_st/(float)offsetcnt_st));
					//LED_B2 = 1;
				}
				else if(TURN_DIR == LEFT){
					len_target = (len - (offsetplus_st/(float)offsetcnt_st));
					//LED_B2 = 1;
				}
			}
			else{
				len_target = len;
				//LED_B1 = 0;
				//LED_B2 = 0;
			}
		}
		//目標速度を設定
		end_speed = end_sp;
		//加速度を設定
		accel = acc;
		//最高速度を設定
		max_speed = max_sp;
		ang_acc = 0;

		if(end_speed == 0){	//最終的に停止する場合
			//減速処理を始めるべき位置まで加速、定速区間を続行
			while( ((len_target -10.0f) - len_mouse) >  1000.0f*((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed))/(float)(2.0f*accel));
			//減速処理開始
			accel = -acc;					//減速するために加速度を負の値にする
			while(len_mouse < len_target -1.0f){		//停止したい距離の少し手前まで継続
				//一定速度まで減速したら最低駆動トルクで走行
				if(tar_speed <= MIN_SPEED){	//目標速度が最低速度になったら、加速度を0にする
					accel = 0;
					tar_speed = MIN_SPEED;
				}
			}
			accel = 0;
			tar_speed = 0;
			//速度が0以下になるまで逆転する
			while(speed >= 0.0f);
		}else{
			//減速処理を始めるべき位置まで加速、定速区間を続行
			while( ((len_target-10.0f) - len_mouse) >  1000.0f*((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed))/(float)(2.0f*accel));
			//減速処理開始
			accel = -acc;					//減速するために加速度を負の値にする
			while(len_mouse < len_target){		//停止したい距離の少し手前まで継続
				//一定速度まで減速したら最低駆動トルクで走行
				if(tar_speed <= end_speed){	//目標速度が最低速度になったら、加速度を0にする
					accel = 0;
					//tar_speed = end_speed;
				}
			}
		}
		//加速度を0にする
		accel = 0;
		//現在距離を0にリセット
		len_mouse = 0;
		offsetplus = 0;
		offsetcnt = 0;
		offsetplus_st = 0;
		offsetcnt_st = 0;
		wallCtrlReset();
		//LED_B1 = 0;
		//LED_B2 = 0;
	}
	else{
	}
}

void back(float len, float acc, float max_sp, float end_sp){
	len_mouse = 3000.0f;
	//degree = 0;
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	I_speed2 = 0;
	run_mode = BACK_MODE;
	FF_Enbl = 1;
	FB_Enbl = 1;
	//目標距離をグローバル変数に代入する
	len_target = 3000.0f-len;
	//目標速度を設定
	end_speed = end_sp;
	//加速度を設定
	accel = acc;
	//最高速度を設定
	max_speed = max_sp;
	ang_acc = 0;

	if(end_speed == 0){	//最終的に停止する場合
		timer = 0;
		//減速処理を始めるべき位置まで加速、定速区間を続行
		while( (len_mouse-(len_target +10.0f)) >  1000.0f*((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed))/(float)(2.0f*accel)){
			if(timer >= 2000){
				break;
			}
		}
		//減速処理開始
		accel = -acc;					//減速するために加速度を負の値にする
		while(len_mouse > len_target +1.0f){		//停止したい距離の少し手前まで継続
			//一定速度まで減速したら最低駆動トルクで走行
			if(tar_speed <= MIN_SPEED){	//目標速度が最低速度になったら、加速度を0にする
				accel = 0;
				tar_speed = MIN_SPEED;
			}
			if(timer >= 2000){
				break;
			}
		}
		accel = 0;
		tar_speed = 0;
		//速度が0以下になるまで逆転する
		while(speed <= 0.0f){
			if(timer >= 2000){
				break;
			}
		}
		timer = 0;

	}else{
		//減速処理を始めるべき位置まで加速、定速区間を続行
		while( (len_mouse-(len_target +10.0f)) >  1000.0f*((float)(tar_speed * tar_speed) - (float)(end_speed * end_speed))/(float)(2.0f*accel));
		//減速処理開始
		accel = -acc;					//減速するために加速度を負の値にする
		while(len_mouse > len_target){		//停止したい距離の少し手前まで継続
			//一定速度まで減速したら最低駆動トルクで走行
			if(tar_speed <= end_speed){	//目標速度が最低速度になったら、加速度を0にする
				accel = 0;
				//tar_speed = end_speed;
			}
		}
	}
	//加速度を0にする
	accel = 0;
	//現在距離を0にリセット
	len_mouse = 0;
	wallCtrlReset();
}

void turn(int deg, float ang_accel, float max_ang_velocity, short dir){
	HAL_Delay(50);
	//degree = 0;
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	I_speed2 = 0;
	tar_degree = 0;

	float	local_degree = 0;
	accel = 0;
	tar_speed = 0;
	tar_ang_vel = 0;
	//走行モードをスラロームモードにする
	run_mode = TURN_MODE;
	FF_Enbl = 1;
	FB_Enbl = 1;

	//回転方向定義
	TURN_DIR = dir;

	//車体の現在角度を取得
	local_degree = degree;
	start_degree = degree;
	tar_degree = 0;

	if(dir == LEFT){
		ang_acc = ang_accel;			//角加速度を設定
		max_ang_vel = max_ang_velocity;
		max_degree = (float)deg;
		while(tar_ang_vel < max_ang_vel);
	}else if(dir == RIGHT){
		ang_acc = -ang_accel;			//角加速度を設定
		max_ang_vel = -max_ang_velocity;
		max_degree = -(float)deg;
		while(tar_ang_vel > max_ang_vel);
	}
	//
	if(dir == LEFT){
		ang_acc = 0;			//角加速度を設定
		max_ang_vel = max_ang_velocity;
		max_degree = (float)deg;
		while( (max_degree - (degree - local_degree))*PI/180.0f > (tar_ang_vel*tar_ang_vel/(2.0f * ang_accel)));
	}else if(dir == RIGHT){
		ang_acc = 0;			//角加速度を設定
		max_ang_vel = -max_ang_velocity;
		max_degree = -(float)deg;
		while(-(float)(max_degree - (degree - local_degree))*PI/180.0f > (float)(tar_ang_vel*tar_ang_vel/(float)(2.0f * ang_accel)));
	}

	if(dir == LEFT){
		ang_acc = -ang_accel;			//角加速度を設定
		//減速区間走行
		while(((degree - local_degree) < max_degree)/* && tar_ang_vel >= 0.0*/){
			//led_all(1);
			if(tar_ang_vel < TURN_MIN_SPEED){
				ang_acc = 0;
				tar_ang_vel = TURN_MIN_SPEED;
			}
		}

		ang_acc = 0;
		tar_ang_vel = 0;
		tar_degree = max_degree;
		//while(ang_vel >= 0.0f);

	}else if(dir == RIGHT){
		ang_acc = ang_accel;			//角加速度を設定
		//減速区間走行
		while(((degree - local_degree) > max_degree)/* && tar_ang_vel <= 0.0*/){
			//led_all(1);
			if(-tar_ang_vel < TURN_MIN_SPEED){
				ang_acc = 0;
				tar_ang_vel = -TURN_MIN_SPEED;
			}
		}
		ang_acc = 0;
		tar_ang_vel = 0;
		tar_degree = max_degree;
		//while(ang_vel <= 0.0f);
	}

//	while(ang_vel >= 0.1 || ang_vel <= -0.1 );
	tar_ang_vel = 0;
	ang_acc = 0;
	run_mode = TEST_MODE;
	Motor_StopPWM();
	//現在距離を0にリセット
	len_mouse = 0;
	HAL_Delay(50);
	wallCtrlReset();
}

void sla(float deg, float ang_accel, float max_ang_velocity,float center_speed, short dir, float diffa , float diffb,float min_ang_velocity){
	runoffseta(diffa,searchacc,center_speed,center_speed);
	if(searching_flag == 1 && noslaflag == 1){
		f_wall(FWALL_REF_R,FWALL_REF_L);
		HAL_Delay(10);
		turn(90,TURN_ACCEL,TURN_SPEED,dir);
		firstread = 0;
		straight(HALF_SECTION/*+SEARCHBACK*/,searchacc,searchmaxv,searchmaxv,FLAG_ON,1);			//まず、半区画進む
		noslaflag = 0;
	}
	else{
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	I_speed2 = 0;
	I_degree = 0;
	tar_degree = 0;

	ichi_x = 0;
	ichi_y = 0;
	tar_ichi_x = 0;
	tar_ichi_y = 0;

	float	local_degree = 0;
	accel = 0;
	tar_speed = center_speed;
	tar_ang_vel = 0;
	//走行モードをスラロームモードにする
	run_mode = SLA_MODE;

	FF_Enbl = 1;
	FB_Enbl = 1;

	//回転方向定義
	TURN_DIR = dir;

	//車体の現在角度を取得
	local_degree = degree;
	start_degree = degree;
	mouse_x = 0;
	mouse_y = 0;
	tar_degree = 0;

	if(dir == LEFT){
		ang_acc = ang_accel;			//角加速度を設定
		max_ang_vel = max_ang_velocity;
		max_degree = (float)deg;
		while(tar_ang_vel < max_ang_vel);
	}else if(dir == RIGHT){
		ang_acc = -ang_accel;			//角加速度を設定
		max_ang_vel = -max_ang_velocity;
		max_degree = -(float)deg;
		while(tar_ang_vel > max_ang_vel);
	}
	//
	if(dir == LEFT){
		ang_acc = 0;			//角加速度を設定
		max_ang_vel = max_ang_velocity;
		max_degree = (float)deg;
		while( (float)((float)(max_degree - (float)(degree - local_degree))*PI/180.0f) > (float)((float)(tar_ang_vel*tar_ang_vel/(float)(2.0f * ang_accel))));
		//while((degree - local_degree))
	}else if(dir == RIGHT){
		ang_acc = 0;			//角加速度を設定
		max_ang_vel = -max_ang_velocity;
		max_degree = -(float)deg;
		while(-(float)(max_degree - (degree - local_degree))*PI/180.0f > (float)(tar_ang_vel*tar_ang_vel/(float)(2.0f * ang_accel)));
	}
	if(dir == LEFT){
		ang_acc = -ang_accel;			//角加速度を設定
		//減速区間走行
		while(((degree - local_degree) < max_degree)&& tar_ang_vel >= 0.0f){
			//led_all(1);
		//	if(tar_ang_vel < min_ang_velocity){
		//		ang_acc = 0;
		//		tar_ang_vel = min_ang_velocity;
			//}
		}
		ang_acc = 0;
		tar_ang_vel = 0;
		tar_degree = max_degree;
		//while(ang_vel >= 0.0);

	}else if(dir == RIGHT){
		ang_acc = ang_accel;			//角加速度を設定
		//減速区間走行
		while(((degree - local_degree) > max_degree) && tar_ang_vel <= 0.0f){
			//led_all(1);
		//	if(-tar_ang_vel < min_ang_velocity){
		//		ang_acc = 0;
		//		tar_ang_vel = -min_ang_velocity;
		//	}
		}
		ang_acc = 0;
		tar_ang_vel = 0;
		tar_degree = max_degree;
		//while(ang_vel <= 0.0);
	}
	tar_ang_vel = 0;
	ang_acc = 0;

	ichi_x = 0;
	ichi_y = 0;
	tar_ichi_x = 0;
	tar_ichi_y = 0;
	R_Ctrl_Disable = 0;
	L_Ctrl_Disable = 0;	//壁切れ後しばらくトレースしない
	Dis_to_Able = 0;
	Dis_to_Able2 = 0;	//壁切れ後しばらくトレースしない
	//現在距離を0にリセット
	len_mouse = 0;
	wallCtrlReset();
	runoffsetb(diffb,searchacc,center_speed,center_speed);
	}
}

void sla2(float deg, float ang_accel, float max_ang_velocity,float center_speed, short dir, float diffa , float diffb,float min_ang_velocity){
	//runoffseta(diffa,SEARCH_ACCEL,center_speed,center_speed);
	if(searching_flag == 1 && noslaflag == 1){
		f_wall(FWALL_REF_R,FWALL_REF_L);
		HAL_Delay(10);
		turn(90,TURN_ACCEL,TURN_SPEED,dir);
		firstread = 0;
		straight(HALF_SECTION/*+SEARCHBACK*/,searchacc,searchmaxv,searchmaxv,FLAG_ON,1);			//まず、半区画進む
		noslaflag = 0;
	}
	else{
		if(saitaning == 0){
			I_tar_ang_vel = 0;
			I_ang_vel = 0;
			I_tar_speed = 0;
			I_speed = 0;
			I_speed2 = 0;
			I_degree = 0;
		}
	tar_degree = 0;

	ichi_x = 0;
	ichi_y = 0;
	tar_ichi_x = 0;
	tar_ichi_y = 0;
	tsin = 0;
	max_ang_vel_sin = 0;
	ang_acc_sin = 0;
	accflag = 0;

	float	local_degree = 0;
	accel = 0;
	tar_speed = center_speed;
	tar_ang_vel = 0;
	//走行モードをスラロームモードにする
	run_mode = SLA2_MODE;

	//FF_Enbl = 1;
	//if(saitaning == 1 && yaba == 1){
	//	FF_Enbl = 0;
	//}
	//else{
	FF_Enbl = 1;
	//}
	FB_Enbl = 1;

	//回転方向定義
	TURN_DIR = dir;

	//車体の現在角度を取得
	local_degree = degree;
	start_degree = degree;
	mouse_x = 0;
	mouse_y = 0;
	tar_degree = 0;
	//LED1 = 1;
	accflag = 1;
	if(dir == LEFT){
		//ang_acc = ang_accel;			//角加速度を設定
		ang_acc = 0;
		ang_acc_sin = ang_accel;
		max_ang_vel = max_ang_velocity;
		max_ang_vel_sin = max_ang_velocity;
		max_degree = (float)deg;
		while(tar_ang_vel < max_ang_vel);
	}else if(dir == RIGHT){
		//ang_acc = -ang_accel;			//角加速度を設定
		ang_acc = 0;
		ang_acc_sin = -ang_accel;
		max_ang_vel = -max_ang_velocity;
		max_ang_vel_sin = -max_ang_velocity;
		max_degree = -(float)deg;
		while(tar_ang_vel > max_ang_vel);
	}
	tsin = 0;
	//
	//LED2 = 1;
	accflag = 0;
	if(dir == LEFT){
		ang_acc = 0;			//角加速度を設定
		ang_acc_sin = 0;
		max_ang_vel = max_ang_velocity;
		max_ang_vel_sin = max_ang_velocity;
		max_degree = (float)deg;
		while( (float)((float)(max_degree - (float)(tar_degree/*degree - local_degree*/))*PI/180.0f) > (float)((float)(tar_ang_vel*tar_ang_vel/(float)(2.0f * ang_accel))));
		//while((degree - local_degree))
	}else if(dir == RIGHT){
		ang_acc = 0;			//角加速度を設定
		ang_acc_sin = 0;
		max_ang_vel = -max_ang_velocity;
		max_ang_vel_sin = -max_ang_velocity;
		max_degree = -(float)deg;
		while(-1.0f*(float)((float)(max_degree - (tar_degree/*degree - local_degree*/))*PI/180.0f) > (float)(tar_ang_vel*tar_ang_vel/(float)(2.0f * ang_accel)));
	}
	//LED3 = 1;
	tsin = 0;
	max_ang_vel_sin = (max_ang_vel_sin*(-1.0f));
	accflag = 2;
	if(dir == LEFT){
		//ang_acc = -ang_accel;			//角加速度を設定
		ang_acc = 0;
		ang_acc_sin = -ang_accel;
		//減速区間走行
		while(((tar_degree/*degree - local_degree*/) < max_degree)&& tar_ang_vel >= 0.0f){
			//led_all(1);
		//	if(tar_ang_vel < min_ang_velocity){
		//		ang_acc = 0;
		//		tar_ang_vel = min_ang_velocity;
		//	}
		}
		ang_acc = 0;
		ang_acc_sin = 0;
		tar_ang_vel = 0;
		tar_degree = max_degree;
		//while(ang_vel >= 0.0);

	}else if(dir == RIGHT){
		//ang_acc = ang_accel;			//角加速度を設定
		ang_acc = 0;
		ang_acc_sin =  ang_accel;
		//減速区間走行
		while(((tar_degree/*degree - local_degree*/) > max_degree) && tar_ang_vel <= 0.0f){
			//led_all(1);
		//	if(-tar_ang_vel < min_ang_velocity){
		//		ang_acc = 0;
		//		tar_ang_vel = -min_ang_velocity;
		//	}
		}
		ang_acc = 0;
		ang_acc_sin = 0;
		tar_ang_vel = 0;
		tar_degree = max_degree;
		//while(ang_vel <= 0.0);
	}
	tar_ang_vel = 0;
	ang_acc = 0;
	ang_acc_sin = 0;
	//max_ang_vel_sin = 0;

	ichi_x = 0;
	ichi_y = 0;
	tar_ichi_x = 0;
	tar_ichi_y = 0;
	tsin = 0;
	accflag = 0;
	R_Ctrl_Disable = 0;
	L_Ctrl_Disable = 0;	//壁切れ後しばらくトレースしない
	Dis_to_Able = 0;
	Dis_to_Able2 = 0;	//壁切れ後しばらくトレースしない
	//現在距離を0にリセット
	len_mouse = 0;
	wallCtrlReset();
	//runoffsetb(diffb,SEARCH_ACCEL,center_speed,center_speed);
	}
}

void sla3(float deg, float ang_accel, float max_ang_velocity ,float center_speed, short dir, float diffa , float diffb){
	//ネイピアターン
	runoffseta(diffa,searchacc,center_speed,center_speed);
	if(searching_flag == 1 && noslaflag == 1){
		f_wall(FWALL_REF_R,FWALL_REF_L);
		HAL_Delay(10);
		turn(90,TURN_ACCEL,TURN_SPEED,dir);
		firstread = 0;
		straight(HALF_SECTION/*+SEARCHBACK*/,searchacc,searchmaxv,searchmaxv,FLAG_ON,1);			//まず、半区画進む
		noslaflag = 0;
	}
	else{
		if(saitaning == 0){
			I_tar_ang_vel = 0;
			I_ang_vel = 0;
			I_tar_speed = 0;
			I_speed = 0;
			I_speed2 = 0;
			I_degree = 0;
		}
	tar_degree = 0;

	ichi_x = 0;
	ichi_y = 0;
	tar_ichi_x = 0;
	tar_ichi_y = 0;

	float	local_degree = 0;
	float deg_acc = 0;
	accel = 0;
	tar_speed = center_speed;
	tar_ang_vel = 0;
	//走行モードをスラロームモードにする
	run_mode = SLA3_MODE;

	FF_Enbl = 1;
	FB_Enbl = 1;

	//回転方向定義
	TURN_DIR = dir;

	//車体の現在角度を取得
	local_degree = degree;
	start_degree = degree;
	mouse_x = 0;
	mouse_y = 0;
	tar_degree = 0;
	Nap_a = 0;
	Nap_n = 0;
	Nap_h = 0;
	Nap_x = 0;
	Nap_b = 0;
	Nap_flag = 0;

	if(dir == LEFT){
		Nap_a = max_ang_velocity/0.0064207f;
		Nap_n = 2;
		Nap_h = 1.0f/((max_ang_velocity/ang_accel)*1000.0f);
		Nap_x = 1;
		max_degree = (float)deg;
		max_degree_G += (float)deg;
		Nap_flag = 1;
	//	LED1_ON();
		while(Nap_x > 0.0f);
	//	LED2_ON();
		Nap_flag = 0;
		deg_acc = tar_degree;//-local_degree;
		while(max_degree-tar_degree > deg_acc);
		Nap_flag = 2;
		while(Nap_flag == 2);
	//	LED3_ON();
	}else if(dir == RIGHT){
		Nap_a = -1.0f*max_ang_velocity/0.0064207f;
		Nap_n = 2;
		Nap_h = 1.0f/((max_ang_velocity/ang_accel)*1000.0f);
		Nap_x = 1;
		max_degree = -(float)deg;
		max_degree_G -= (float)deg;
		Nap_flag = 1;
		while(Nap_x > 0.0f);
		Nap_flag = 0;
		deg_acc = tar_degree;//-local_degree;
		while(max_degree-tar_degree < deg_acc);
		Nap_flag = 2;
		while(Nap_flag == 2);
	}

	Nap_a = 0;
	Nap_n = 0;
	Nap_h = 0;
	Nap_x = 0;
	Nap_b = 0;
	Nap_flag = 0;
	tar_ang_vel = 0;
	ang_acc = 0;

	ichi_x = 0;
	ichi_y = 0;
	tar_ichi_x = 0;
	tar_ichi_y = 0;
	R_Ctrl_Disable = 0;
	L_Ctrl_Disable = 0;	//壁切れ後しばらくトレースしない
	Dis_to_Able = 0;
	Dis_to_Able2 = 0;	//壁切れ後しばらくトレースしない
	//現在距離を0にリセット
	len_mouse = 0;
	wallCtrlReset();
	runoffsetb(diffb,searchacc,center_speed,center_speed);
	}
}

void f_wall(float distancer,float distancel){
	fwallnoflag = 0;
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	I_speed2 = 0;
	I_fwall = 0;
	fwallcnt = 0;
	//走行モードを直線にする
	run_mode = F_WALL_MODE;
	FF_Enbl = 0;
	FB_Enbl = 1;
	while(((Sensor_FR_Dis - distancer > 0.1f || distancer - Sensor_FR_Dis > 0.1f) || (Sensor_FL_Dis - distancel > 0.1f || distancel - Sensor_FL_Dis > 0.1f)) && fwallnoflag == 0 && fwallcnt <= 1000);
	tar_ang_vel = 0;
	tar_speed = 0;
	ang_acc = 0;
	//現在距離を0にリセット
	len_mouse = 0;
	run_mode = TEST_MODE;
	Motor_StopPWM();
	wallCtrlReset();
}


void party(void){
	I_tar_ang_vel = 0;
	I_ang_vel = 0;
	I_tar_speed = 0;
	I_speed = 0;
	I_speed2 = 0;
	//走行モードを直線にする
	run_mode = PARTY_MODE;
	FF_Enbl = 0;
	FB_Enbl = 1;
	//目標距離をグローバル変数に代入する
	len_target = 0;
	//目標速度を設定
	end_speed = 0;
	//加速度を設定
	accel = 0;
	//最高速度を設定
	max_speed = 0;
	ang_acc = 0;
	while(SWITCH_ONOFF() == SW_OFF);
	run_mode = TEST_MODE;
	Motor_StopPWM();
	tar_ang_vel = 0;
	ang_acc = 0;
	//現在距離を0にリセット
	len_mouse = 0;
}

void tracelog(void){
	HAL_Delay(50);
}

void turn_fwall(void){
	if(Wall_FR == WALL_ON && Wall_FL == WALL_ON){
		HAL_Delay(10);
		f_wall(FWALL_REF_R,FWALL_REF_L);
		if((Wall_R == WALL_ON && Wall_L == WALL_OFF)
		|| (Wall_R == WALL_ON && Wall_L == WALL_ON && Sensor_R_Dis < Sensor_L_Dis)){//逆にした 8/20
			turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
			HAL_Delay(10);
			f_wall(FWALL_REF_R,FWALL_REF_L);
			turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
			HAL_Delay(10);
		}
		else{
			turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
			HAL_Delay(10);
			f_wall(FWALL_REF_R,FWALL_REF_L);
			turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
			HAL_Delay(10);
		}
	}
	else{
		if((Wall_R == WALL_ON && Wall_L == WALL_OFF)
		|| (Wall_R == WALL_ON && Wall_L == WALL_ON && Sensor_R_Dis < Sensor_L_Dis)){
			HAL_Delay(10);
			turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
			HAL_Delay(10);
			f_wall(FWALL_REF_R,FWALL_REF_L);
			turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
			HAL_Delay(10);
		}
		else{
			HAL_Delay(10);
			turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
			HAL_Delay(10);
			f_wall(FWALL_REF_R,FWALL_REF_L);
			turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
			HAL_Delay(10);
		}
	}
}

void turn_fwall2(void){
	if(BlockWall_F == 1){
		HAL_Delay(10);
		f_wall(FWALL_REF_R,FWALL_REF_L);
		if(BlockWall_R == 0 && BlockWall_L == 0){
			HAL_Delay(10);
			turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);
			HAL_Delay(10);
		}
		else{
			if((BlockWall_R == 1 && BlockWall_L == 0)
			|| (BlockWall_R == 1 && BlockWall_L == 1 && Sensor_R_Dis > Sensor_L_Dis)){
				turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
				HAL_Delay(10);
				f_wall(FWALL_REF_R,FWALL_REF_L);
				turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
				HAL_Delay(10);
			}
			else{
				turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
				HAL_Delay(10);
				f_wall(FWALL_REF_R,FWALL_REF_L);
				turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
				HAL_Delay(10);
			}
		}
	}
	else{
		if(BlockWall_R == 0 && BlockWall_L == 0){
			HAL_Delay(10);
			turn(180,TURN_ACCEL,TURN_SPEED,LEFT);
			HAL_Delay(10);
		}
		else{
			if((BlockWall_R == 1 && BlockWall_L == 0)
			|| (BlockWall_R == 1 && BlockWall_L == 1 && Sensor_R_Dis > Sensor_L_Dis)){
				HAL_Delay(10);
				turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
				HAL_Delay(10);
				f_wall(FWALL_REF_R,FWALL_REF_L);
				turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
				HAL_Delay(10);
			}
			else{
				HAL_Delay(10);
				turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
				HAL_Delay(10);
				f_wall(FWALL_REF_R,FWALL_REF_L);
				turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
				HAL_Delay(10);
			}
		}
	}
}
