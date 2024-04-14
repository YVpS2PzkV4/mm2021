/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"index.h"
#include"Interface.h"
#include"common.h"
#include"CMT.h"
#include"run.h"
#include"map.h"
#include"search.h"
#include"beeline.h"
#include"Dataflash.h"
#include"dijkstra.h"
#include<math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
//#include<mathf.h>

#define ARM_MATH_CM4
#include "arm_math.h"
#include "arm_const_structs.h"

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int f getc(FILE* f)
#endif

void __io_putchar(uint8_t ch) {
	HAL_UART_Transmit(&huart1, &ch, 1, 1);
}

int __io_getchar(void) {
	HAL_StatusTypeDef Status = HAL_BUSY;
	uint8_t Data;
	while(Status != HAL_OK)
		Status = HAL_UART_Receive(&huart1, &Data, 1, 10);
	return(Data);
}
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile int V_bat = 0;
volatile int V_bat_a = 0;
volatile int V_bat_b = 0;
unsigned volatile short Wall_R = 0;
unsigned volatile short Wall_L = 0;
unsigned volatile short Wall_FR = 0;
unsigned volatile short Wall_FL = 0;
volatile float Sensor_FL = 0;
volatile float Sensor_FR = 0;
volatile float Sensor_L = 0;
volatile float Sensor_R = 0;
volatile float Sensor_R_Dis = 0;
volatile float Sensor_L_Dis = 0;
volatile float Sensor_FR_Dis = 0;
volatile float Sensor_FL_Dis = 0;
volatile float Before_R_Dis = 0;
volatile float Before_L_Dis = 0;
volatile float Before_FR_Dis = 0;
volatile float Before_FL_Dis = 0;

unsigned volatile int	locate_l = 0;				//現在の車軸位置	[無次元]
unsigned volatile int	locate_r = 0;				//現在の車軸位置	[無次元]
unsigned volatile int	before_locate_r = 0;			//過去の車軸位置	[無次元]
unsigned volatile int	before_locate_l = 0;			//過去の車軸位置	[無次元]
volatile int	diff_pulse_r = 0;				//車軸位置の微分値(車軸の回転速度[pulse/ms])
volatile int	diff_pulse_l = 0;				//車軸位置の微分値(車軸の回転速度[pulse/ms])
volatile int turnnuml = 0;
volatile int turnnumr = 0;
volatile float			speed_r = 0;				//現在の右タイヤ速度	[m/s]
volatile float			speed_l = 0;				//現在の左タイヤ速度	[m/s]
volatile float			speed_old_r = 0;				//右タイヤの過去の速度	[m/s]
volatile float			speed_new_r = 0;				//右タイヤの最新の速度	[m/s]
volatile float			speed_old_l = 0;				//左タイヤの過去の速度	[m/s]
volatile float			speed_new_l = 0;				//左タイヤの最新の速度	[m/s]
volatile float			speed = 0;					//現在車体速度		[m/s]
volatile float			p_speed = 0;				//過去の車体速度	[m/s]
//
volatile float			speed_r2 = 0;				//ターン中の速度制御用？
volatile float 			speed_l2 = 0;
volatile float 			speed_old_r2 = 0;
volatile float 			speed_old_l2 = 0;
volatile float 			speed2 = 0;
//
volatile float			len_mouse = 0;
volatile float			I_speed = 0;
volatile float			I_speed2 = 0;
volatile float			gyro_ref = 0;
volatile float			ang_vel = 0;
volatile float			p_ang_vel = 0;
volatile float			ang_vel_current[5] = {0,0,0,0,0};
volatile float			ang_vel_ave = 0;
volatile float			ang_vel_ave_b = 0;
volatile float			new_ang_vel = 0;
volatile float			I_ang_vel = 0;
volatile float			degree = 0;
volatile float			I_degree = 0;
volatile float			tar_ang_vel = 0;				//目標角速度		[rad/s]
volatile float			tar_degree = 0;				//目標角度		[deg]
volatile float			max_degree = 0;				//旋回時の最大角度	[deg]
volatile float			start_degree = 0;				//走行進入時の車体角度	[deg]
volatile float			max_ang_vel = 0;				//最高角速度		[rad/s]
volatile float			max_ang_vel_sin = 0;				//最高角速度		[rad/s]
volatile float			ang_acc = 0;				//角加速度		[rad/ss]
volatile float			ang_acc_sin = 0;				//角加速度		[rad/ss]
volatile float			accel = 0;					//加速度		[m/ss]
volatile float			max_speed = 0;				//最高速度		[m/s]
volatile float			Duty_r = 0;
volatile float			Duty_l = 0;
volatile float			V_r = 0;
volatile float			V_l = 0;
volatile float 			len_target = 0;
volatile char 			TURN_DIR = 0;
volatile int			run_mode = TEST_MODE;
volatile float			tar_speed = 0;				//目標車体速度		[m/s]
//タイマ系グローバル変数
unsigned volatile  int		timer = 0;					//1mSごとにカウントアップされる変数.
unsigned volatile long		searchtimer = 0;					//1mSごとにカウントアップされる変数.

//制御用グローバル変数
volatile float			I_tar_speed = 0;				//目標速度のI成分
volatile float			I_tar_ang_vel = 0;				//目標角速度のI成分
volatile float 			end_speed = 0;
volatile int mode = 0;
//ログ用のグローバル変数
volatile long			log_timer = 0;				//ログ取りようのタイマ
volatile int			log_flag = 0;				//ログ取得のタイミング

volatile float 			torque_r = 0;
volatile float 			torque_l = 0;
volatile float 			FF_r = 0;
volatile float 			FF_l = 0;
volatile float 			FB_r = 0;
volatile float 			FB_l = 0;

volatile float			mouse_x = 0;
volatile float			mouse_y = 0;

volatile float			p_speed_r = 0;				//過去の車体速度	[m/s]
volatile float			p_speed_l = 0;				//過去の車体速度	[m/s]
volatile float 			tar_speed_r = 0;
volatile float 			tar_speed_l = 0;


extern int mx;
extern int my;
extern volatile int head;

extern volatile int tracecnt;

volatile float min_speed = 0;

extern float saitandashv;
extern float saitan90v;
extern float saitanacc;
extern float saitandec;
extern float saitanminv;
extern float saitanslaacc;
extern float saitanslav;
extern float saitanbeforeslaoffset;
extern float saitanafterslaoffset;

extern float saitanoffset;
extern float saitanbefore45offset;
extern float saitanafter45offset;
extern float saitanbeforeout45offset;
extern float saitanafterout45offset;
extern float saitanbeforebig90offset;
extern float saitanafterbig90offset;
extern float saitanbefore135offset;
extern float saitanafter135offset;
extern float saitanbeforeout135offset;
extern float saitanafterout135offset;
extern float saitanbefore180offset;
extern float saitanafter180offset;
extern float saitanbeforev90offset;
extern float saitanafterv90offset;
extern float saitanbeforekojimaoffset;
extern float saitanafterkojimaoffset;

extern float saitanfastacc;
extern float saitanfastdec;
extern float saitanstopdec;
extern float saitan45v;
extern float saitanout45v;
extern float saitanbig90v;
extern float saitan135v;
extern float saitanout135v;
extern float saitan180v;
extern float saitanv90v;
extern float saitankojimav;
extern float saitandashslantingv;
extern float saitanmaxv;

extern float saitanbig90alpha;
extern float saitanbig90w;
extern float saitan45alpha;
extern float saitan45w;
extern float saitanout45alpha;
extern float saitanout45w;
extern float saitan135alpha;
extern float saitan135w;
extern float saitanout135alpha;
extern float saitanout135w;
extern float saitan180alpha;
extern float saitan180w;
extern float saitanv90alpha;
extern float saitanv90w;
extern float saitankojimaalpha;
extern float saitankojimaw;

extern float bigrightangleR;
extern float big180angleR;
extern float s135angleR;
extern float sout135angleR;
extern float s45angleR;
extern float sout45angleR;
extern float v90angleR;
extern float kojimaangleR;

extern float bigrightangleL;
extern float big180angleL;
extern float s135angleL;
extern float sout135angleL;
extern float s45angleL;
extern float sout45angleL;
extern float v90angleL;
extern float kojimaangleL;

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
extern unsigned int slantingtask;

extern volatile char comb;

extern volatile char BatCheckSW;

extern volatile char saitaning;

extern volatile char path[1024];
extern volatile char kojima_enbl;

extern int allsearched;

extern volatile char g_Sensor_Flag;

//extern volatile char edge[256];

extern unsigned char map[MAZESIZE_X][MAZESIZE_Y];

volatile char sincurve = 0;

extern volatile int searching_flag;

extern volatile char off_flag;	//最短のターンオフセット中かどうか

volatile char kaeri = 0;	//探索後の帰り
volatile char ttx = 0;	//探索後の帰り
volatile char tty = 0;	//探索後の帰り
volatile char tth = 0;	//探索後の帰り
volatile char lasth = 0;	//探索後の帰り

extern t_edge exist[1024];

extern volatile char set_param;
extern volatile char spd_cnt;

extern volatile float len_true;

extern t_dijkstra nodeinfo[32][32][12];

volatile float FanVolt = 0.0f;	//吸引ファンの印加電圧
volatile float			Duty_fan = 0;
volatile char FanEnbl = 0;	//吸引ファンの印加電圧

extern volatile char locate_flag_l;
extern volatile char locate_flag_r;

volatile double Angle_G = 0;	//絶対角度（ジャイロのリファレンスとるときにリセット）
volatile double Tar_Angle_G = 0;	//目標絶対角度（ジャイロのリファレンスとるときにリセット）

void adjust_turn(void);
void show_logdata(unsigned char);
void stopWari(void);
void reStartWari(void);

void param_transmit(void);
void param_receive(void);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t Communication_TerminalRecv( void )
{
	uint8_t data[1];
	HAL_UART_Receive( &huart1, (uint8_t*)data, sizeof(data), 100 );
	return (*data);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  //Communication_Initialize();	// ターミナルとの通信設定
  setbuf( stdout, NULL );
  setbuf( stdin, NULL );
  Motor_Initialize();			// IMU用通信およびICM20648の初期設定
  Sensor_Initialize();		// 壁センサ用AD変換およびタイマー設定
  IMU_Initialize();			// IMU用通信およびICM20648の初期設定
  param_read();
  if(!(flash_ID > 123.44 && flash_ID < 123.46)/*fabsf(flash_ID-MACHINE_ID) > 0.001*/){
	  printf("Flash Data Error!\r\n");
	  char f;
	  for(f = 0; f < 3; f++){
		  led_all(1);
		  HAL_Delay(100);
		  led_all(0);
		  HAL_Delay(100);
	  }
	  param_receive();
	  param_read();
	  while(1){
		  led_all(1);
		  HAL_Delay(50);
		  led_all(0);
		  HAL_Delay(50);
	  }
  }
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim9);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  run_mode = TEST_MODE;
  BatCheckSW = 1;
  g_Sensor_Flag = 0;
  reset_notice();
  long j;

  while(1){
  		g_Sensor_Flag = 0;
  		enmode();
  /*		g_Sensor_Flag = 1;
  		MOT_R_FORWARD();
  		MOT_L_FORWARD();
  		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 50);
  		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 50);
  		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 300);
  		while(1){
  			printf("Batt : %d",V_bat);
  			printf("\r\n");
  			printf("Sensor_R : %d (Dis : %4.1f)",(short)(Sensor_R),Sensor_R_Dis);
  			printf("\r\n");
  			printf("Sensor_L : %d (Dis : %4.1f)",(short)(Sensor_L),Sensor_L_Dis);
  			printf("\r\n");
  			printf("Sensor_FR : %d (Dis : %4.1f)",(short)(Sensor_FR),Sensor_FR_Dis);
  			printf("\r\n");
  			printf("Sensor_FL : %d (Dis : %4.1f)",(short)(Sensor_FL),Sensor_FL_Dis);
  			printf("\r\n");
  			printf("Gyro : %4.1f",(ang_vel*100.0f));
  			printf("\r\n");
  			printf("SpeedR : %4.1f",(speed_r*100.0f));
  			printf("\r\n");
  			printf("SpeedL : %4.1f",(speed_l*100.0f));
  			printf("\r\n");
  			HAL_Delay(150);
  		}*/

  		/***************************************************************************************/
  		//大会用モード
  		if(mode == 0){
  			/******************************/
  			//設定用の値
  			char setx = 0,sety = 0;	//ゴール座標（設定）
  			char back1 = 0;	//帰り最短
  			char parapara = 0;	//最短パラメータ
  			char hosuu = 0;	//歩数orダイクストラ
  			char spara = 0;	//探索パラメータ
  			char wall2 = 0;	//よくわからんやつ
  			/******************************/
  			//デフォルト値
  			char defo = 0;
  			const char defo_x = flash3.goalX,defo_y = flash3.goalY;	//デフォルトのゴール座標←大会前に確認確認確認!!!
  			const char defo_spara = 1;	//探索パラ
  			const char defo_hosuu = 1;	//歩数（ダイクストラ）
  			const char defo_sin = 0;	//sin加速するか？（台形加速）
  			const char defo_kojima = 0;	//こじまターンするかどうか（しない）
  			const char defo_back = 0;	//帰り最短するかどうか（しない）
  			/*****************************/
  			volatile char k;

  			while(1){
  				mode = 0;
  				defo = 0;
  				enmode();
  				defo = mode;
  				/******************************************/
  				//探索
  				if(defo == 0){	//デフォルト探索（0.35m/s,台形加速,全面）
  					setx = defo_x;
  					sety = defo_y;
  					setpara_search(defo_spara);
  					set_param = defo_spara;
  					sincurve = defo_sin;
  					mx = 0;
  					my = 0;
  					head = 0;
  					clearmap();
  				}
  				else if(defo == 1){	//直接設定できる方
  					setx = defo_x;
  					sety = defo_y;
  					mode = 0;
  					enmode();
  					setpara_search(mode);
  					spara = mode;
  					set_param = mode;
  					//
  					mode = 0;
  					sincurve = 0;
  					enmode();
  					sincurve = mode;
  					//
  					mode = 0;
  					enmode();
  					//
  					mx = 0;
  					my = 0;
  					head = 0;
  					if(mode == 0){
  						clearmap();
  					}
  					else{
  						clearmap();
  						map_copy();
  					}
  				}
  				else if(defo == 2){	//直接設定できる方
  					mode = 0;
  					enmode();
  					setx = mode;
  					mode = 0;
  					enmode();
  					sety = mode;
  					mode = 0;
  					enmode();
  					setpara_search(mode);
  					spara = mode;
  					set_param = mode;
  					//
  					mode = 0;
  					sincurve = 0;
  					enmode();
  					sincurve = mode;
  					//
  					mode = 0;
  					enmode();
  					//
  					mx = 0;
  					my = 0;
  					head = 0;
  					if(mode == 0){
  						clearmap();
  					}
  					else{
  						clearmap();
  						map_copy();
  					}
  				}
  				if(defo == 0 || defo == 1 || defo == 2){
  					waithand();
  					get_gyro_ref();
  					slaadachi(setx,sety,1);
  					g_Sensor_Flag = 0;
  					BatCheckSW = 0;
  					map_write();
  					BatCheckSW = 1;
  					//map_copy();
  					slaadachi(0,0,0);
  					g_Sensor_Flag = 0;
  					BatCheckSW = 0;
  					map_write();
  					BatCheckSW = 1;
  					if(allsearched == 1){
  						ttx = mx;
  						tty = my;
  						lasth = head;
  						tth = 0;
  						mx = 0;
  						my = 0;
  						head = 0;
  						kaeri = 1;
  						kojima_enbl = 0;
  						sincurve = 0;
  						setpara(3);
  						get_gyro_ref();
  						saitan_shortest(0,0,0,1);
  						kaeri = 0;
  						g_Sensor_Flag = 0;
  					}
  				}
  				/*******************************************/
  				/*******************************************/
  				//最短
  				if(defo == 3){	//デフォルト最短（ダイクストラ、コジマなし、台形加速、パラメータのみ設定）
  					setx = defo_x;
  					sety = defo_y;
  					mx = 0;
  					my = 0;
  					head = 0;
  					hosuu = defo_hosuu;
  					kojima_enbl = defo_kojima;
  				//	sincurve = defo_sin;
  					back1 = defo_back;
  					mode = 0;
  					enmode();
  					setpara(mode);
  					parapara = mode;
  				//	if(parapara >= 3){
  				//		sincurve = 1;
  				//	}
  				}
  				else if(defo == 4){	//直接設定できる方
  					setx = defo_x;
  					sety = defo_y;
  					mode = 0;
  					enmode();
  					mx = 0;
  					my = 0;
  					head = 0;
  					setpara(mode);
  					parapara = mode;
  					mode = 0;
  					enmode();
  					hosuu = mode;
  					mode = 0;
  					enmode();
  					kojima_enbl = mode;
  					mode = 0;
  				//	sincurve = 0;
  				//	enmode();
  				//	sincurve = mode;
  					mode = 0;
  					enmode();
  					back1 = mode;
  				//	if(parapara >= 3){
  				//		sincurve = 1;
  				//	}
  				}
  				else if(defo == 5){	//直接設定できる方
  					mode = 0;
  					enmode();
  					setx = mode;
  					mode = 0;
  					enmode();
  					sety = mode;
  					mode = 0;
  					enmode();
  					mx = 0;
  					my = 0;
  					head = 0;
  					setpara(mode);
  					parapara = mode;
  					mode = 0;
  					enmode();
  					hosuu = mode;
  					mode = 0;
  					enmode();
  					kojima_enbl = mode;
  					mode = 0;
  				//	sincurve = 0;
  				//	enmode();
  				//	sincurve = mode;
  					mode = 0;
  					enmode();
  					back1 = mode;
  				//	if(parapara >= 3){
  				//		sincurve = 1;
  				//	}
  				}

  				if(defo == 3 || defo == 4 || defo == 5){
  					clearmap2();
  					map_copy();
  					waithand();
  					get_gyro_ref();

  						if(hosuu == 1){
  							runTimeCalc();
  							if(back1 == 0){
  								saitan_shortest(setx,sety,1,0);
  								g_Sensor_Flag = 0;
  							}
  							else if(back1 == 1){
  								saitan_shortest(setx,sety,1,0);
  								g_Sensor_Flag = 0;
  								saitan_shortest(setx,sety,0,0);
  								g_Sensor_Flag = 0;
  							}
  							else if(back1 == 2){
  								saitan_shortest(setx,sety,2,0);
  								g_Sensor_Flag = 0;
  							//	LED1_ON();
  								clearmap2();
  							//	LED2_ON();
  								map_copy();
  							//	LED3_ON();
  								g_Sensor_Flag = 1;
  								HAL_Delay(100);
  								if((map[my][mx]&0xf0) != 0xf0){
  									//ゴール区画が未探索だったとき
  									if(head == 1){	//headを0にしておく
  										turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
  									}
  									else if(head == 2){
  										turn(180,TURN_ACCEL,TURN_SPEED,LEFT);
  									}
  									else if(head == 3){
  										turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
  									}
  									HAL_Delay(20);
  									f_wall(FWALL_REF_R,FWALL_REF_L);
  									if(Sensor_FR_Dis <= 120.0 && Sensor_FL_Dis <= 120.0){
  										wall2 = (wall2 | 0x11);
  									}
  									turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
  									HAL_Delay(20);
  									f_wall(FWALL_REF_R,FWALL_REF_L);
  									if(Sensor_FR_Dis <= 120.0 && Sensor_FL_Dis <= 120.0){
  										wall2 = (wall2 | 0x22);
  									}
  									turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
  									HAL_Delay(20);
  									f_wall(FWALL_REF_R,FWALL_REF_L);
  									if(Sensor_FR_Dis <= 120.0 && Sensor_FL_Dis <= 120.0){
  										wall2 = (wall2 | 0x44);
  									}
  									turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
  										HAL_Delay(20);
  									f_wall(FWALL_REF_R,FWALL_REF_L);
  									if(Sensor_FR_Dis <= 120.0 && Sensor_FL_Dis <= 120.0){
  										wall2 = (wall2 | 0x88);
  									}
  									turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
  									HAL_Delay(20);
  									wall2 = (wall2 | 0xf0);
  									map[my][mx] = wall2;
  									// 隣の区間のＭＡＰデータも更新する
  									if (mx != (MAZESIZE_X-1)){
  										map[my][mx + 1] = (map[my][mx + 1] & 0x77) | 0x80 | ((wall2 << 2) & 0x08);
  									}
  									if (mx != 0){
  										map[my][mx - 1] = (map[my][mx - 1] & 0xdd) | 0x20 | ((wall2 >> 2) & 0x02);
  									}
  									if (my != (MAZESIZE_Y-1)){
  										map[my + 1][mx] = (map[my + 1][mx] & 0xbb) | 0x40 | ((wall2 << 2) & 0x04);
  									}
  									if (my != 0){
  										map[my - 1][mx] = (map[my - 1][mx] & 0xee) | 0x10 | ((wall2 >> 2) & 0x01);
  									}
  									if(head == 1){	//headを0にしておく
  										turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);
  									}
  									else if(head == 2){
  										turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);
  									}
  									else if(head == 3){
  										turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
  									}
  								}
  								run_mode = TEST_MODE;
  							//	LED4 = 1;
  								setpara_search(defo_spara);
  							//	LED5 = 1;
  								set_param = defo_spara;
  								len_true = 0;
  								spd_cnt = 1;
  								sincurve = defo_sin;
  								HAL_Delay(STARTWAIT);
  								slantingtask = 0;
  								searchtimer = 0;
  								//LED1_OFF();
  								get_gyro_ref();
  								led_all(0);
  								slaadachi(0,0,0);
  								g_Sensor_Flag = 0;
  								BatCheckSW = 0;
  								map_write();
  								BatCheckSW = 1;
  								if(allsearched == 1){
  									ttx = mx;
  									tty = my;
  									lasth = head;
  									tth = 0;
  									mx = 0;
  									my = 0;
  									head = 0;
  									kaeri = 1;
  									kojima_enbl = 0;
  									sincurve = 0;
  									setpara(3);
  									saitan_shortest(0,0,0,1);
  									kaeri = 0;
  									g_Sensor_Flag = 0;
  								}
  							}
  							//saitan2(setx,sety);
  						}
  						else{
  							if(back1 == 0){
  								saitan_shortest(setx,sety,1,1);
  								g_Sensor_Flag = 0;
  							}
  							else if(back1 == 1){
  								saitan_shortest(setx,sety,1,1);
  								g_Sensor_Flag = 0;
  								saitan_shortest(0,0,0,1);
  								g_Sensor_Flag = 0;
  							}
  							else if(back1 == 2){
  								saitan_shortest(setx,sety,2,1);
  								g_Sensor_Flag = 0;
  								clearmap2();
  								map_copy();
  								setpara_search(1);
  								HAL_Delay(STARTWAIT);
  								slantingtask = 0;
  								searchtimer = 0;
  								slaadachi(0,0,0);
  								g_Sensor_Flag = 0;
  								BatCheckSW = 0;
  								map_write();
  								BatCheckSW = 1;
  								if(allsearched == 1){
  									ttx = mx;
  									tty = my;
  									lasth = head;
  									tth = 0;
  									mx = 0;
  									my = 0;
  									head = 0;
  									kaeri = 1;
  									kojima_enbl = 0;
  									sincurve = 0;
  									setpara(3);
  									saitan_shortest(0,0,0,1);
  									kaeri = 0;
  									g_Sensor_Flag = 0;
  								}
  							}
  						}
  					HAL_Delay(100);
  				}

  				if(defo == 6){
  					waithand();
  					log_output();
  				}
  				else if(defo == 7){
  					waithand();
  					log_erase();
  					HAL_Delay(100);
  					for(k = 0; k < 3; k++){
  						LED6_ON();
  						HAL_Delay(50);
  						LED6_OFF();
  						HAL_Delay(50);
  					}

  				}
  			}
  			mode = 0;
  		}

  		else if(mode == 1){
  			/*waithand();
  			g_Sensor_Flag = 1;
  			HAL_Delay(100);
  			log_timer = 0;
  			log_flag = 1;
  			HAL_Delay(500);
  			log_flag = 0;
  			show_logdata(10);
  			show_logdata(11);
  			show_logdata(0);
  			show_logdata(3);
  			HAL_Delay(100);*/

  			waithand();
  			g_Sensor_Flag = 1;
  			while(1){
  				if(Sensor_R_Dis < WALL_TH_R){
  					LED1_ON();
  				}
  				else{
  					LED1_OFF();
  				}
  				if(Sensor_L_Dis < WALL_TH_L){
  				  	LED6_ON();
  				}
  				else{
  					LED6_OFF();
  				}
  				if(Wall_FR == WALL_ON){
  					LED2_ON();
  				}
  				else{
  					LED2_OFF();
  				}
  				if(Wall_FL == WALL_ON){
  					LED5_ON();
  				}
  				else{
  					LED5_OFF();
  				}
  				//HAL_Delay(50);
  			}
  		}
  		else if(mode == 2){
  			mode = 0;
  			enmode();
  			if(mode == 0){
  				waithand();
  				param_transmit();
  			}
  			else if(mode == 1){
  				waithand();
  				param_receive();
  				param_read();
  				while(1){
  					led_all(1);
  					HAL_Delay(50);
  					led_all(0);
  					HAL_Delay(50);
  				}
  			}
  			else{

  			}
  			mode = 2;
  		}
  		else if(mode == 3){
  			waithand();
  			g_Sensor_Flag = 0;
  			FanVolt = 2.5;
  			FanEnbl = 1;
  			get_gyro_ref();
  			wallCtrlReset();
  			speed = 0;
  			tar_speed = 0;

  				len_mouse = 0;
  			//	log_timer = 0;
  			//	log_flag = 1;
  				firstread = 0;
  				//slantingtask = 1;
  				comb = 0;
  				//saitaning = 1;
  			//	searching_flag = 1;
  				log_timer = 0;
  				log_flag = 1;
  				straight(200,25.0,2.5,0.0,FLAG_OFF,0);
  				log_flag = 0;
  				//saitaning = 0;
  			//	searching_flag = 0;
  				comb = 0;
  				g_Sensor_Flag = 0;
  				FanVolt = 0.0;
  				FanEnbl = 0;
  				run_mode = TEST_MODE;
  				Motor_StopPWM();
  				waithand();
  				show_logdata(1);
  				show_logdata(2);
  				//slantingtask = 0;
  			//	log_flag = 0;

  		//	}

  		HAL_Delay(200);
  		}
  		else if(mode == 4){
  			int u;
  			//waithand();

  			for(u = 1; u < 7; u++){
  				waithand();
  				get_gyro_ref();
  				FanVolt = 2.5;
  				FanEnbl = 1;
  				HAL_Delay(500);
  				motor_setVoltage((float)(0.25f*(float)u),1);
  				FanEnbl = 0;
  				waithand();
  				show_logdata(9);
  				//show_logdata(3);
  			}


  	/*		make_map_known();
  			map_to_maze();
  			nodereset();
  			setpara(13);
  			runTimeCalc();
  			LED3_ON();
  			shortest(19,20);

  			//makesaitan(15,5);
  			LED1_ON();
  			//saitan_to_path();
  			turnWallEdge();
  			LED1_OFF();
  			volatile short p = 0;
  			while(1){
  				if(path[p] == SNODE){
  					printf("%d(%d)",(short)path[p],(short)p);
  					break;
  				}
  				else{
  					printf("%d(%d)(edge %d,comb %d)",(short)path[p],(short)p,(short)exist[p].edge,(short)exist[p].combsw);
  					printf("	");
  					p++;
  				}
  			}*/
  		}
  		else if(mode == 5){
  			adjust_turn();
  		}
  		else if(mode == 6){
  			mode = 0;
  			enmode();
  			if(mode == 0){
  				FanVolt = 2.5;
  			}
  			else if(mode == 1){
  				FanVolt = 3.0;
  			}
  			waithand();
  			//FanVolt = 2.5;
  			FanEnbl = 1;
  			while(1);
  		//	clearmap();
  			//map_copy();
  			//map_view();
  			//HAL_Delay(100);
  		}
  		else if(mode == 7){
  			mode = 0;
  			enmode();
  			if(mode == 0){
  				FanVolt = 0;
  			}
  			else if(mode == 1){
  				FanVolt = 2.5;
  			}
  			else if(mode == 2){
  				FanVolt = 3.0;
  			}
  			waithand();
  			get_gyro_ref();
  			g_Sensor_Flag = 0;
  			//FanVolt = 2.5;
  			FanEnbl = 1;
  			HAL_Delay(250);
  			LED1_ON();
  			LED6_ON();
  		//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 240);
  			party();
  			LED1_OFF();
  			LED6_OFF();
  		//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  			HAL_Delay(250);
  		}
  		else if(mode == 8){
  			waithand();
  			g_Sensor_Flag = 1;
  			get_gyro_ref();
  			wallCtrlReset();
  			while(1){
  			f_wall(45.0,45.0);
  			}
  			HAL_Delay(50);
  			g_Sensor_Flag = 0;

  		}
  		else if(mode == 9){
  			waithand();
  			turnnuml = 0;
  			//MOT_R_FORWARD();
  			MOT_L_FORWARD();
  			//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 300);
  			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 25);
  		//	while(1);
  			HAL_Delay(150);
  			LED1_ON();
  			locate_flag_l = 0;
  			while(locate_flag_l != 1);
  			//short start = locate_l;
  			turnnuml = 0;
  			log_timer = 0;
  			log_flag = 1;
  		//	while(locate_r == 0);
  			LED2_ON();
  			while(turnnuml < 5000);
  			log_flag = 0;
  		//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  		//	waithand();
  			show_logdata(5);
  			waithand();
  			show_logdata(5);
  		}
  		else if(mode == 10){
  			waithand();
  			turnnumr = 0;
  			MOT_R_BACKWARD();
  			//	MOT_L_FORWARD();
  			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 25);
  			//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 29);
  			//	while(1);
  			HAL_Delay(150);
  			locate_flag_r = 0;
  			LED1_ON();
  			while(locate_flag_r != 1);
  			//short start = locate_l;
  			turnnumr = 0;
  			log_timer = 0;
  			log_flag = 1;
  			//	while(locate_r == 0);
  			LED2_ON();
  			while(turnnumr > -5000);
  			log_flag = 0;
  			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  			//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  			//	waithand();
  			show_logdata(4);
  			waithand();
  			show_logdata(4);
  		}
  		else if(mode == 11){
  			waithand();
  			/*while(1){
  			MOT_R_FORWARD();
  			MOT_L_FORWARD();
  			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 80);
  			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 80);
  			HAL_Delay(5000);
  			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  			HAL_Delay(500);
  			MOT_R_BACKWARD();
  			MOT_L_BACKWARD();
  			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 80);
  			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 80);
  			HAL_Delay(5000);
  			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  			HAL_Delay(500);
  			}*/

  			g_Sensor_Flag = 1;
  			while(1){
  				printf("Batt : %d",V_bat);
  				printf("\r\n");
  				printf("Sensor_R : %d (Dis : %4.1f)",(short)(Sensor_R),Sensor_R_Dis);
  				printf("\r\n");
  				printf("Sensor_L : %d (Dis : %4.1f)",(short)(Sensor_L),Sensor_L_Dis);
  				printf("\r\n");
  				printf("Sensor_FR : %d (Dis : %4.1f)",(short)(Sensor_FR),Sensor_FR_Dis);
  				printf("\r\n");
  				printf("Sensor_FL : %d (Dis : %4.1f)",(short)(Sensor_FL),Sensor_FL_Dis);
  				printf("\r\n");
  				printf("\r\n");
  				HAL_Delay(150);
  				//printf("\x1b[2J");			//クリアスクリーン[CLS]
  				//printf("\x1b[0;0H");		//カーソルを0,0に移動
  			}
  			g_Sensor_Flag = 0;
  		}
  		else if(mode == 12){
  			waithand();
  			g_Sensor_Flag = 0;
  			get_gyro_ref();
  			FanVolt = 2.5;
  			FanEnbl = 1;
  			HAL_Delay(500);
  			wallCtrlReset();
  			speed = 0;
  			tar_speed = 0;

  				len_mouse = 0;
  			//	log_timer = 0;
  			//	log_flag = 1;
  				firstread = 0;
  				//slantingtask = 1;
  				comb = 0;
  			//	saitaning = 1;
  			//	searching_flag = 1;
  				log_timer = 0;
  				log_flag = 1;
  				//straight(200,5.0,0.5,0,FLAG_OFF,0);
  				turn(180,100.0,5.0,LEFT);
  				log_flag = 0;
  				saitaning = 0;
  				searching_flag = 0;
  				comb = 0;
  				g_Sensor_Flag = 0;
  				HAL_Delay(500);
  				FanEnbl = 0;
  				FanVolt = 0;
  				run_mode = TEST_MODE;
  				Motor_StopPWM();
  				waithand();
  				show_logdata(8);
  				show_logdata(9);
  				show_logdata(0);
  				show_logdata(11);
  				//HAL_Delay(200);
  		}
  		else if(mode == 13){
  			waithand();
  			get_gyro_ref();
  			wallCtrlReset();
  			g_Sensor_Flag = 0;
  			speed = 0;
  			tar_speed = 0;
  			degree = 0;
  			//len_mouse = 0;
  			for(j = 0; j < 20; j++){
  				straight(45,SEARCH_ACCEL,0.3,0.3,FLAG_OFF,0);
  				log_timer = 0;
  				log_flag = 1;
  				straight(5,SEARCH_ACCEL,0.3,0.3,FLAG_OFF,0);
  				sla(91.1,261.8,10.5,0.3,RIGHT,0,0,0.2618);
  				straight(15,SEARCH_ACCEL,0.3,0.3,FLAG_OFF,0);
  				log_flag = 0;
  			}
  			straight(45,SEARCH_ACCEL,0.3,0,FLAG_OFF,0);
  			HAL_Delay(100);
  		}
  		else if(mode == 14){
  			waithand();
  			get_gyro_ref();
  			wallCtrlReset();
  			g_Sensor_Flag = 0;
  			speed = 0;
  			tar_speed = 0;
  			//len_mouse = 0;
  			for(j = 0; j < 1; j++){
  				straight(45,SEARCH_ACCEL,0.3,0.3,FLAG_OFF,0);
  				straight(17,SEARCH_ACCEL,0.3,0.3,FLAG_OFF,0);
  				log_timer = 0;
  				log_flag = 1;
  				//sla(90,261.8,10.5,0.25,LEFT,0,0,0.2618);
  				sla3(90,126.95,12.84,0.3,LEFT,0,0);
  				log_flag = 0;
  				straight(19,SEARCH_ACCEL,0.3,0.3,FLAG_OFF,0);

  			}
  			straight(45,SEARCH_ACCEL,0.3,0,FLAG_OFF,0);
  			HAL_Delay(100);
  		}
  		else if(mode == 15){
  			char muki = 0;
  			mode = 0;
  			enmode();
  			if(mode == 0){
  				muki = RIGHT;
  			}
  			else if(mode == 1){
  				muki = LEFT;
  			}
  			waithand();
  			get_gyro_ref();
  			FanVolt = 2.5;
  			FanEnbl = 1;
  			HAL_Delay(500);
  			wallCtrlReset();
  			g_Sensor_Flag = 0;
  			speed = 0;
  			tar_speed = 0;
  			//len_mouse = 0;
  			for(j = 0; j < 1; j++){
  			//	straight(45,SEARCH_ACCEL,0.3,0,FLAG_OFF,0);
  				wait_ms(100);
  				log_timer = 0;
  				log_flag = 1;
  				turn(3600,TURN_ACCEL,TURN_SPEED,muki);
  				log_flag = 0;
  			}
  			//straight(45,SEARCH_ACCEL,0.4,0,FLAG_OFF,0);
  			HAL_Delay(200);
  		}
  		g_Sensor_Flag = 0;
  		run_mode = TEST_MODE;
  		Motor_StopPWM();
  	}
  }

  void adjust_turn(void){
	  int mode0 = 0;
	  char muki = 0;
	  int p = 0;
  	//char sinflag = 0;
//  	mode = 0;
//  	enmode();
//  	if(mode == 0){
//  		mode = 0;
//  		enmode();
//  		setpara_search(mode);
//  	}
//  	else{
  		mode = 0;
  		enmode();
  		setpara(mode);
//  	}
 // 	mode = 0;
 //	enmode();
 // 	sinflag = mode;
  	mode = 0;
  	while(1){
  		mode = mode0;
  		enmode();
  		//waithand();
  		mode0 = mode;
  		//
  		mode = 0;
  		enmode();
  		if(mode == 0){
  			muki = RIGHT;
  		}
  		else if(mode == 1){
  			muki = LEFT;
  		}
  		waithand();
  		get_gyro_ref();
  		FanEnbl = 1;
  		HAL_Delay(500);
  		wallCtrlReset();
  		g_Sensor_Flag = 0;
  		speed = 0;
  		tar_speed = 0;
  		saitaning = 1;
  		searching_flag = 0;
  		if(mode0 == 0){
  			straight(90,saitanacc,saitandashv,saitanbig90v,FLAG_OFF,0);
  			off_flag = 1;
  			straight(saitanbeforebig90offset,saitanfastacc,saitanmaxv,saitanbig90v,FLAG_OFF,0);
  			log_timer = 0;
  			log_flag = 1;
  			if(sincurve == 0){
  				sla(bigrightangleR,saitanbig90alpha,saitanbig90w,saitanbig90v,muki,0,0,(saitanbig90w*0.001));
  			}
  			else if(sincurve == 1){
  				sla2(bigrightangleR,saitanbig90alpha,saitanbig90w,saitanbig90v,muki,0,0,(saitanbig90w*0.001));
  			}
  			else{
  				sla3(bigrightangleR,saitanbig90alpha,saitanbig90w,saitanbig90v,muki,0,0);
  			}
  			log_flag = 0;
  			straight(saitanafterbig90offset,saitanfastacc,saitanmaxv,saitanmaxv,FLAG_OFF,0);
  			straight(90,saitanacc,saitandashv,0,FLAG_OFF,0);
  			off_flag = 0;
  		}
  		else if(mode0 == 1){
  			g_Sensor_Flag = 0;
  			straight(90,saitanacc,saitandashv,saitan180v,FLAG_OFF,0);
  			off_flag = 1;
  			straight(saitanbefore180offset,saitanfastacc,saitanmaxv,saitan180v,FLAG_OFF,0);
  			log_timer = 0;
  			log_flag = 1;
  			if(sincurve == 0){
  				sla(big180angleR,saitan180alpha,saitan180w,saitan180v,muki,0,0,(saitan180w*0.001));
  			}
  			else if(sincurve == 1){
  				sla2(big180angleR,saitan180alpha,saitan180w,saitan180v,muki,0,0,(saitan180w*0.001));
  			}
  			else{
  				sla3(big180angleR,saitan180alpha,saitan180w,saitan180v,muki,0,0);
  			}
  			log_flag = 0;
  			straight(saitanafter180offset,saitanfastacc,saitanmaxv,saitanmaxv,FLAG_OFF,0);
  			//straight(saitanbeforebig90offset,saitanfastacc,saitanmaxv,saitanbig90v,FLAG_OFF,0);
  			//log_flag = 0;
  			/*if(sinflag == 0){
  			sla(bigrightangle,saitanbig90alpha,saitanbig90w,saitanbig90v,LEFT,0,0,(saitanbig90w*0.001));
  			}
  			else{
  			sla2(bigrightangle,saitanbig90alpha,saitanbig90w,saitanbig90v,LEFT,0,0,(saitanbig90w*0.001));
  			}
  			straight(saitanafterbig90offset,saitanfastacc,saitanmaxv,saitanmaxv,FLAG_OFF,0);
  			*/
  			straight(90,saitanacc,saitandashv,0,FLAG_OFF,0);
  			off_flag = 0;
  			//g_Sensor_Flag = 0;
  		}
  		else if(mode0 == 2){
  			g_Sensor_Flag = 0;
  			straight(90,saitanacc,saitandashv,saitan45v,FLAG_OFF,0);
  			off_flag = 1;
  			straight(saitanbefore45offset,saitanfastacc,saitanmaxv,saitan45v,FLAG_OFF,0);
  			log_timer = 0;
  			log_flag = 1;
  			if(sincurve == 0){
  				sla(s45angleR,saitan45alpha,saitan45w,saitan45v,muki,0,0,(saitan45w*0.001));
  			}
  			else if(sincurve == 1){
  				sla2(s45angleR,saitan45alpha,saitan45w,saitan45v,muki,0,0,(saitan45w*0.001));
  			}
  			else{
  				sla3(s45angleR,saitan45alpha,saitan45w,saitan45v,muki,0,0);
  			}
  			log_flag = 0;
  			straight(saitanafter45offset,saitanfastacc,saitanmaxv,saitanmaxv,FLAG_OFF,0);
  			straight(ONESLANTING,saitanacc,saitandashv,0,FLAG_OFF,0);
  			g_Sensor_Flag = 0;
  			off_flag = 0;
  		}
  		else if(mode0 == 3){
  			straight(90,saitanacc,saitandashv,saitan135v,FLAG_OFF,0);
  			off_flag = 1;
  			straight(saitanbefore135offset,saitanfastacc,saitanmaxv,saitan135v,FLAG_OFF,0);
  			log_timer = 0;
  			log_flag = 1;
  			if(sincurve == 0){
  				sla(s135angleR,saitan135alpha,saitan135w,saitan135v,muki,0,0,(saitan135w*0.001));
  			}
  			else if(sincurve == 1){
  				sla2(s135angleR,saitan135alpha,saitan135w,saitan135v,muki,0,0,(saitan135w*0.001));
  			}
  			else{
  				sla3(s135angleR,saitan135alpha,saitan135w,saitan135v,muki,0,0);
  			}
  			log_flag = 0;
  			straight(saitanafter135offset,saitanfastacc,saitanmaxv,saitanmaxv,FLAG_OFF,0);
  			straight(ONESLANTING,saitanacc,saitandashv,0,FLAG_OFF,0);
  			off_flag = 0;
  		}
  		else if(mode0 == 4){
  			straight(ONESLANTING,saitanacc,saitandashv,saitanv90v,FLAG_OFF,0);
  			log_timer = 0;
  			straight(saitanbeforev90offset,saitanfastacc,saitanmaxv,saitanv90v,FLAG_OFF,0);
  			log_timer = 0;
  			log_flag = 1;
  			if(sincurve == 0){
  				sla(v90angleR,saitanv90alpha,saitanv90w,saitanv90v,muki,0,0,(saitanv90w*0.001));
  			}
  			else if(sincurve == 1){
  				sla2(v90angleR,saitanv90alpha,saitanv90w,saitanv90v,muki,0,0,(saitanv90w*0.001));
  			}
  			else{
  				sla3(v90angleR,saitanv90alpha,saitanv90w,saitanv90v,muki,0,0);
  			}
  			log_flag = 0;
  			straight(saitanafterv90offset,saitanfastacc,saitanmaxv,saitanmaxv,FLAG_OFF,0);
  			off_flag = 0;
  			straight(ONESLANTING,saitanacc,saitandashv,0,FLAG_OFF,0);

  		}
  		else if(mode0 == 5){
  			straight(ONESLANTING,saitanacc,saitandashv,saitanout45v,FLAG_OFF,0);
  			off_flag = 1;
  			straight(saitanbeforeout45offset,saitanfastacc,saitanmaxv,saitanout45v,FLAG_OFF,0);
  			log_timer = 0;
  			log_flag = 1;
  			if(sincurve == 0){
  				sla(sout45angleR,saitanout45alpha,saitanout45w,saitanout45v,muki,0,0,(saitanout45w*0.001));
  			}
  			else if(sincurve == 1){
  				sla2(sout45angleR,saitanout45alpha,saitanout45w,saitanout45v,muki,0,0,(saitanout45w*0.001));
  			}
  			else{
  				sla3(sout45angleR,saitanout45alpha,saitanout45w,saitanout45v,muki,0,0);
  			}
  			log_flag = 0;
  			straight(saitanafterout45offset,saitanfastacc,saitanmaxv,saitanmaxv,FLAG_OFF,0);
  			straight(90,saitanacc,saitandashv,0,FLAG_OFF,0);
  			off_flag = 0;
  		}
  		else if(mode0 == 6){
  			straight(ONESLANTING,saitanacc,saitandashv,saitanout135v,FLAG_OFF,0);
  			off_flag = 1;
  			straight(saitanbeforeout135offset,saitanfastacc,saitanmaxv,saitanout135v,FLAG_OFF,0);
  			log_timer = 0;
  			log_flag = 1;
  			if(sincurve == 0){
  				sla(sout135angleR,saitanout135alpha,saitanout135w,saitanout135v,muki,0,0,(saitanout135w*0.001));
  			}
  			else if(sincurve == 1){
  				sla2(sout135angleR,saitanout135alpha,saitanout135w,saitanout135v,muki,0,0,(saitanout135w*0.001));
  			}
  			else{
  				sla3(sout135angleR,saitanout135alpha,saitanout135w,saitanout135v,muki,0,0);
  			}
  			log_flag = 0;
  			straight(saitanafterout135offset,saitanfastacc,saitanmaxv,saitanmaxv,FLAG_OFF,0);
  			straight(90,saitanacc,saitandashv,0,FLAG_OFF,0);
  			off_flag = 0;
  		}
  		else if(mode0 == 7){
  			straight(ONESLANTING,saitanacc,saitandashv,saitankojimav,FLAG_OFF,0);
  			off_flag = 1;
  			straight(saitanbeforekojimaoffset,saitanfastacc,saitanmaxv,saitankojimav,FLAG_OFF,0);
  			log_timer = 0;
			log_flag = 1;
  			if(sincurve == 0){
  				sla(kojimaangleR,saitankojimaalpha,saitankojimaw,saitankojimav,muki,0,0,(saitankojimaw*0.001));
  			}
  			else if(sincurve == 1){
  				sla2(kojimaangleR,saitankojimaalpha,saitankojimaw,saitankojimav,muki,0,0,(saitankojimaw*0.001));
  			}
  			else{
  				sla3(kojimaangleR,saitankojimaalpha,saitankojimaw,saitankojimav,muki,0,0);
  			}
  			log_flag = 0;
  			straight(saitanafterkojimaoffset,saitanfastacc,saitanmaxv,saitanmaxv,FLAG_OFF,0);
  			straight(ONESLANTING,saitanacc,saitandashv,0,FLAG_OFF,0);
  			off_flag = 0;
  		}
  		else if(mode0 == 8){
  			g_Sensor_Flag = 1;
  			log_timer = 0;
  			log_flag = 1;
  			wait_ms(1000);
  			log_flag = 0;
  			g_Sensor_Flag = 0;

  		}
  		else if(mode0 == 9){
  			straight(90,saitanacc,saitandashv,saitan45v,FLAG_OFF,0);
  			off_flag = 1;
  			straight(saitanbefore45offset,saitanfastacc,saitanmaxv,saitan45v,FLAG_OFF,0);
  			sla2(s45angleR,saitan45alpha,saitan45w,saitan45v,RIGHT,0,0,(saitan45w*0.001));
  			straight(saitanafter45offset,saitanfastacc,saitanmaxv,saitanmaxv,FLAG_OFF,0);
  			straight(saitanbeforeout45offset,saitanfastacc,saitanmaxv,saitanout45v,FLAG_OFF,0);
  			sla2(s45angleL,saitanout45alpha,saitanout45w,saitanout45v,LEFT,0,0,(saitanout45w*0.001));
  			straight(saitanafterout45offset,saitanfastacc,saitanmaxv,saitanmaxv,FLAG_OFF,0);
  			straight(90,saitanacc,saitandashv,0,FLAG_OFF,0);
  			//straight(ONESLANTING,saitanacc,saitandashv,0,FLAG_OFF,0);
  			off_flag = 0;
  		}
  		else if(mode0 == 10){
  			straight(ONESLANTING,saitanacc,saitandashv,saitanv90v,FLAG_OFF,0);
  			log_timer = 0;
  			log_flag = 1;
  			straight(saitanbeforev90offset,saitanfastacc,saitanmaxv,saitanv90v,FLAG_OFF,0);
  			sla2(v90angleR,saitanv90alpha,saitanv90w,saitanv90v,RIGHT,0,0,(saitanv90w*0.001));
  			straight(saitanafterv90offset,saitanfastacc,saitanmaxv,saitanmaxv,FLAG_OFF,0);
  			log_flag = 0;
  			straight(saitanbeforeout45offset,saitanfastacc,saitanmaxv,saitanout45v,FLAG_OFF,0);
  			sla2(s45angleR,saitanout45alpha,saitanout45w,saitanout45v,LEFT,0,0,(saitanout45w*0.001));
  			straight(saitanafterout45offset,saitanfastacc,saitanmaxv,saitanmaxv,FLAG_OFF,0);
  			straight(90,saitanacc,saitandashv,0,FLAG_OFF,0);
  			//straight(ONESLANTING,saitanacc,saitandashv,0,FLAG_OFF,0);
  		}
  		else if(mode0 == 11){
  			straight(ONESLANTING,saitanacc,saitandashv,0.7,FLAG_OFF,0);
  			  			log_timer = 0;
  			  			log_flag = 1;
  			  			off_flag = 1;
  			  			straight(10,saitanfastacc,saitanmaxv,0.7,FLAG_OFF,0);
  			  			sla3(90,296.98,19.65,0.7,RIGHT,0,0);
  			  			straight(8,saitanfastacc,saitanmaxv,saitanmaxv,FLAG_OFF,0);
  			  			log_flag = 0;
  			  			off_flag = 0;
  			  			straight(ONESLANTING,saitanacc,saitandashv,0,FLAG_OFF,0);
  		}
  		else if(mode0 == 12){
  			straight(90,saitanacc,saitandashv,saitanbig90v,FLAG_OFF,0);
  			off_flag = 1;
  			for(p = 0; p < 20; p++){
  			straight(saitanbeforebig90offset,saitanfastacc,saitanmaxv,saitanbig90v,FLAG_OFF,0);
  			if(sincurve == 0){
  				sla(bigrightangleR,saitanbig90alpha,saitanbig90w,saitanbig90v,muki,0,0,(saitanbig90w*0.001));
  			}
  			else if(sincurve == 1){
  				sla2(bigrightangleR,saitanbig90alpha,saitanbig90w,saitanbig90v,muki,0,0,(saitanbig90w*0.001));
  			}
  			else{
  				sla3(bigrightangleR,saitanbig90alpha,saitanbig90w,saitanbig90v,muki,0,0);
  			}
  			straight(saitanafterbig90offset,saitanfastacc,saitanmaxv,saitanmaxv,FLAG_OFF,0);
  			}
  			straight(90,saitanacc,saitandashv,0,FLAG_OFF,0);
  			off_flag = 0;
  		}
  		else if(mode0 == 13){
  			show_logdata(9);
  			waithand();
  			show_logdata(8);
  			waithand();
  			show_logdata(11);
  			waithand();
  			show_logdata(10);
  			waithand();
  			show_logdata(3);
  		}
  		else if(mode0 == 14){
  			show_logdata(1);
  			//waithand();
  			show_logdata(2);
  			//waithand();
  			show_logdata(6);
  		}
  		else if(mode0 == 15){
  			show_logdata(0);
  			show_logdata(1);
  			//waithand();
  			show_logdata(2);
  			show_logdata(3);
  			show_logdata(4);
  			show_logdata(5);
  			show_logdata(6);
  			show_logdata(7);
  			show_logdata(8);
  			show_logdata(9);
  			show_logdata(10);
  			show_logdata(11);

  			//show_logdata(11);
  		}
  		else{
  		}
  		saitaning = 0;
  		FanEnbl = 0;
  		run_mode = TEST_MODE;
  		Motor_StopPWM();
  	}
  }

  void show_logdata(unsigned char num){
	unsigned char a = 0,b = 0;
  	int dummy = 0;
  	stopWari();
  	HAL_Delay(50);
  	for(a = 0; a < 32; a++){
  		for(b = 0; b < 32; b++){
  			if(nodeinfo[a][b][num].node1 > 32767){
  				dummy = nodeinfo[a][b][num].node1-65536;
  			}
  			else{
  				dummy = nodeinfo[a][b][num].node1;
  			}
  			printf("%d",dummy);
  			if(a == 31 && b == 31){
  				printf("\r\n");
  			}else{
  				printf(",");
  			}
  		}
  	}
  	reStartWari();
  }

void stopWari(void){
	HAL_TIM_Base_Stop_IT(&htim5);
	HAL_TIM_Base_Stop_IT(&htim9);
	HAL_TIM_Base_Stop_IT(&htim3);
	HAL_GPIO_WritePin( LED_FR_GPIO_Port, LED_FR_Pin, GPIO_PIN_RESET );			//LED消灯
	HAL_GPIO_WritePin( LED_FL_GPIO_Port, LED_FL_Pin, GPIO_PIN_RESET );			//LED消灯
	HAL_GPIO_WritePin( LED_SR_GPIO_Port, LED_SR_Pin, GPIO_PIN_RESET );			//LED消灯
	HAL_GPIO_WritePin( LED_SL_GPIO_Port, LED_SL_Pin, GPIO_PIN_RESET );			//LED消灯
	Motor_StopPWM();
}

void reStartWari(void){
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_Base_Start_IT(&htim9);
	HAL_TIM_Base_Start_IT(&htim3);
}

void param_transmit(void){
	int i = 0;
	stopWari();
	//パラメータ編集ソフトにデータ送信
	printf(" KuriMonaka Parameters\r\n");

	printf("\r\n");
	printf(" ---------- Search Parameters ----------\r\n");
	for(i = 0; i < SEARCHPARANUM; i++){
		if(i == 0){
			printf("  ----- Param 0 -----\r\n");
		}
		else if(i == 1){
			printf("  ----- Param 1 -----\r\n");
		}
		else if(i == 2){
			printf("  ----- Param 2 -----\r\n");
		}
		else if(i == 3){
			printf("  ----- Param 3 -----\r\n");
		}
		printf("   fan : (%g)\r\n",flash1.fanV[i]);
		printf("   maxv : (%g)\r\n",flash1.maxV[i]);
		printf("   acc : (%g)\r\n",flash1.acc[i]);
		printf("   knownmaxv : (%g)\r\n",flash1.knownMaxV[i]);
		printf("   knownacc : (%g)\r\n",flash1.knownAcc[i]);
		printf("   slaacc : (%g)\r\n",flash1.slaAlpha[i]);
		printf("   slav : (%g)\r\n",flash1.slaW[i]);
		printf("   beforeoffsetr : (%g)\r\n",flash1.offsetRA[i]);
		printf("   afteroffsetr : (%g)\r\n",flash1.offsetRB[i]);
		printf("   beforeoffsetl : (%g)\r\n",flash1.offsetLA[i]);
		printf("   afteroffsetl : (%g)\r\n",flash1.offsetLB[i]);
		printf("   slaminv : (%g)\r\n",flash1.slaMinV[i]);
		printf("   degr : (%g)\r\n",flash1.degR[i]);
		printf("   degl : (%g)\r\n",flash1.degL[i]);
		printf("   fwall : (%g)\r\n",flash1.fWall[i]);
	}

	printf("\r\n");
	printf(" ---------- Beeline Parameters ----------\r\n");
	for(i = 0; i < SAITANPARANUM; i++){
		if(i == 0){
			printf("  ----- Param 0 -----\r\n");
		}
		else if(i == 1){
			printf("  ----- Param 1 -----\r\n");
		}
		else if(i == 2){
			printf("  ----- Param 2 -----\r\n");
		}
		else if(i == 3){
			printf("  ----- Param 3 -----\r\n");
		}
		else if(i == 4){
			printf("  ----- Param 4 -----\r\n");
		}
		else if(i == 5){
			printf("  ----- Param 5 -----\r\n");
		}
		else if(i == 6){
			printf("  ----- Param 6 -----\r\n");
		}
		else if(i == 7){
			printf("  ----- Param 7 -----\r\n");
		}
		else if(i == 8){
			printf("  ----- Param 8 -----\r\n");
		}
		else if(i == 9){
			printf("  ----- Param 9 -----\r\n");
		}
		else if(i == 10){
			printf("  ----- Param 10 -----\r\n");
		}
		else if(i == 11){
			printf("  ----- Param 11 -----\r\n");
		}
		else if(i == 12){
			printf("  ----- Param 12 -----\r\n");
		}
		else if(i == 13){
			printf("  ----- Param 13 -----\r\n");
		}
		else if(i == 14){
			printf("  ----- Param 14 -----\r\n");
		}
		//
		printf("   fan : (%g)\r\n",flash2[i].fan);
		printf("   sin : (%d)\r\n",flash2[i].sin);
		printf("   dashv : (%g)\r\n",flash2[i].dashv);
		printf("   acc : (%g)\r\n",flash2[i].acc);
		printf("   dec : (%g)\r\n",flash2[i].dec);
		printf("   slantdashv : (%g)\r\n",flash2[i].slantdashv);
		printf("   slantacc : (%g)\r\n",flash2[i].slantacc);
		printf("   slantdec : (%g)\r\n",flash2[i].slantdec);
		printf("   fastacc : (%g)\r\n",flash2[i].fastacc);
		printf("   fastdec : (%g)\r\n",flash2[i].fastdec);
		printf("   firstacc : (%g)\r\n",flash2[i].firstacc);
		printf("   acc1 : (%g)\r\n",flash2[i].acc1);
		printf("   dec1 : (%g)\r\n",flash2[i].dec1);
		printf("   slantacc1 : (%g)\r\n",flash2[i].slantacc1);
		printf("   slantdec1 : (%g)\r\n",flash2[i].slantdec1);
		printf("   stopdec : (%g)\r\n",flash2[i].stopdec);
		printf("   minv : (%g)\r\n",flash2[i].minv);
		printf("   offsetmaxv : (%g)\r\n",flash2[i].offsetmaxv);
		//
		printf("   --- big90 ---\r\n");
		printf("    v : (%g)\r\n",flash2[i].big90.v);
		printf("    alpha : (%g)\r\n",flash2[i].big90.alpha);
		printf("    w : (%g)\r\n",flash2[i].big90.w);
		printf("    angleR : (%g)\r\n",flash2[i].big90.angleR);
		printf("    angleL : (%g)\r\n",flash2[i].big90.angleL);
		printf("    offset1 : (%g)\r\n",flash2[i].big90.offset1);
		printf("    offset2 : (%g)\r\n",flash2[i].big90.offset2);
		printf("   --- big180 ---\r\n");
		printf("    v : (%g)\r\n",flash2[i].big180.v);
		printf("    alpha : (%g)\r\n",flash2[i].big180.alpha);
		printf("    w : (%g)\r\n",flash2[i].big180.w);
		printf("    angleR : (%g)\r\n",flash2[i].big180.angleR);
		printf("    angleL : (%g)\r\n",flash2[i].big180.angleL);
		printf("    offset1 : (%g)\r\n",flash2[i].big180.offset1);
		printf("    offset2 : (%g)\r\n",flash2[i].big180.offset2);
		printf("   --- in45 ---\r\n");
		printf("    v : (%g)\r\n",flash2[i].in45.v);
		printf("    alpha : (%g)\r\n",flash2[i].in45.alpha);
		printf("    w : (%g)\r\n",flash2[i].in45.w);
		printf("    angleR : (%g)\r\n",flash2[i].in45.angleR);
		printf("    angleL : (%g)\r\n",flash2[i].in45.angleL);
		printf("    offset1 : (%g)\r\n",flash2[i].in45.offset1);
		printf("    offset2 : (%g)\r\n",flash2[i].in45.offset2);
		printf("   --- in135 ---\r\n");
		printf("    v : (%g)\r\n",flash2[i].in135.v);
		printf("    alpha : (%g)\r\n",flash2[i].in135.alpha);
		printf("    w : (%g)\r\n",flash2[i].in135.w);
		printf("    angleR : (%g)\r\n",flash2[i].in135.angleR);
		printf("    angleL : (%g)\r\n",flash2[i].in135.angleL);
		printf("    offset1 : (%g)\r\n",flash2[i].in135.offset1);
		printf("    offset2 : (%g)\r\n",flash2[i].in135.offset2);
		printf("   --- v90 ---\r\n");
		printf("    v : (%g)\r\n",flash2[i].v90.v);
		printf("    alpha : (%g)\r\n",flash2[i].v90.alpha);
		printf("    w : (%g)\r\n",flash2[i].v90.w);
		printf("    angleR : (%g)\r\n",flash2[i].v90.angleR);
		printf("    angleL : (%g)\r\n",flash2[i].v90.angleL);
		printf("    offset1 : (%g)\r\n",flash2[i].v90.offset1);
		printf("    offset2 : (%g)\r\n",flash2[i].v90.offset2);
		printf("   --- out45 ---\r\n");
		printf("    v : (%g)\r\n",flash2[i].out45.v);
		printf("    alpha : (%g)\r\n",flash2[i].out45.alpha);
		printf("    w : (%g)\r\n",flash2[i].out45.w);
		printf("    angleR : (%g)\r\n",flash2[i].out45.angleR);
		printf("    angleL : (%g)\r\n",flash2[i].out45.angleL);
		printf("    offset1 : (%g)\r\n",flash2[i].out45.offset1);
		printf("    offset2 : (%g)\r\n",flash2[i].out45.offset2);
		printf("   --- out135 ---\r\n");
		printf("    v : (%g)\r\n",flash2[i].out135.v);
		printf("    alpha : (%g)\r\n",flash2[i].out135.alpha);
		printf("    w : (%g)\r\n",flash2[i].out135.w);
		printf("    angleR : (%g)\r\n",flash2[i].out135.angleR);
		printf("    angleL : (%g)\r\n",flash2[i].out135.angleL);
		printf("    offset1 : (%g)\r\n",flash2[i].out135.offset1);
		printf("    offset2 : (%g)\r\n",flash2[i].out135.offset2);
		printf("   --- kojima ---\r\n");
		printf("    v : (%g)\r\n",flash2[i].kojima.v);
		printf("    alpha : (%g)\r\n",flash2[i].kojima.alpha);
		printf("    w : (%g)\r\n",flash2[i].kojima.w);
		printf("    angleR : (%g)\r\n",flash2[i].kojima.angleR);
		printf("    angleL : (%g)\r\n",flash2[i].kojima.angleL);
		printf("    offset1 : (%g)\r\n",flash2[i].kojima.offset1);
		printf("    offset2 : (%g)\r\n",flash2[i].kojima.offset2);
	}
	//
	printf("\r\n");
	printf(" ---------- Maze ----------\r\n");
	printf("  GOALSIZE : (%d)\r\n",flash3.goalSize);
	printf("  GOAL_X : (%d)\r\n",flash3.goalX);
	printf("  GOAL_Y : (%d)\r\n",flash3.goalY);
	printf("  COMP_SIZE_X : (%d)\r\n",flash3.compSizeX);
	printf("  COMP_SIZE_Y : (%d)\r\n",flash3.compSizeY);
	printf("  SEARCHTIMEMAX : (%g)\r\n",flash3.searchTime);
	printf("  LEN_DOWN : (%g)\r\n",flash3.lenDown);
	//
	printf("\r\n");
	printf(" ---------- Control Parameters ----------\r\n");
	printf("  ----- PID Gain -----\r\n");
	printf("   SPEED_KP : (%g)\r\n",flash4.speedKP);
	printf("   SPEED_KI : (%g)\r\n",flash4.speedKI);
	printf("   SPEED_KD : (%g)\r\n",flash4.speedKD);
	printf("   SPEED_KP2 : (%g)\r\n",flash4.speedKP2);
	printf("   SPEED_KI2 : (%g)\r\n",flash4.speedKI2);
	printf("   SPEED_KD2 : (%g)\r\n",flash4.speedKD2);
	printf("   OMEGA_KP : (%g)\r\n",flash4.omegaKP);
	printf("   OMEGA_KI : (%g)\r\n",flash4.omegaKI);
	printf("   OMEGA_KD : (%g)\r\n",flash4.omegaKD);
	printf("   OMEGA_KP2 : (%g)\r\n",flash4.omegaKP2);
	printf("   OMEGA_KI2 : (%g)\r\n",flash4.omegaKI2);
	printf("   OMEGA_KD2 : (%g)\r\n",flash4.omegaKD2);
	printf("   OMEGA_KP2R : (%g)\r\n",flash4.omegaKP2R);
	printf("   OMEGA_KI2R : (%g)\r\n",flash4.omegaKI2R);
	printf("   OMEGA_KD2R : (%g)\r\n",flash4.omegaKD2R);
	printf("   OMEGA_KP2L : (%g)\r\n",flash4.omegaKP2L);
	printf("   OMEGA_KI2L : (%g)\r\n",flash4.omegaKI2L);
	printf("   OMEGA_KD2L : (%g)\r\n",flash4.omegaKD2L);
	printf("   ANGLE_KP : (%g)\r\n",flash4.angleKP);
	printf("   SPEED_KP_MINUS : (%g)\r\n",flash4.speedKPminus);
	printf("   SPEED_KI_MINUS : (%g)\r\n",flash4.speedKIminus);
	printf("   SPEED_KD_MINUS : (%g)\r\n",flash4.speedKDminus);
	printf("   OMEGA_KP_MINUS : (%g)\r\n",flash4.speedKPminus);
	printf("   OMEGA_KI_MINUS : (%g)\r\n",flash4.speedKIminus);
	printf("   OMEGA_KD_MINUS : (%g)\r\n",flash4.speedKDminus);
	printf("  ----- FF Gain -----\r\n");
	printf("   FF_ACCEL : (%g)\r\n",flash4.ffAccel);
	printf("   FF_SPEED : (%g)\r\n",flash4.ffSpeed);
	printf("   FF_CONST : (%g)\r\n",flash4.ffConst);
	printf("   FF_ANG_ACC : (%g)\r\n",flash4.ffAngAcc);
	printf("   FF_ANG_VEL : (%g)\r\n",flash4.ffAngVel);
	printf("   FF_ANG_CONST : (%g)\r\n",flash4.ffAngConst);
	printf("   FF_ACCEL2R : (%g)\r\n",flash4.ffAccel2R);
	printf("   FF_SPEED2R : (%g)\r\n",flash4.ffSpeed2R);
	printf("   FF_CONST2R : (%g)\r\n",flash4.ffConst2R);
	printf("   FF_ACCEL2L : (%g)\r\n",flash4.ffAccel2L);
	printf("   FF_SPEED2L : (%g)\r\n",flash4.ffSpeed2L);
	printf("   FF_CONST2L : (%g)\r\n",flash4.ffConst2L);
	printf("   FF_ANG_ACC2RR : (%g)\r\n",flash4.ffAngAcc2Rr);
	printf("   FF_ANG_VEL2RR : (%g)\r\n",flash4.ffAngVel2Rr);
	printf("   FF_ANG_CONST2RR : (%g)\r\n",flash4.ffAngConst2Rr);
	printf("   FF_ANG_ACC2RL : (%g)\r\n",flash4.ffAngAcc2Rl);
	printf("   FF_ANG_VEL2RL : (%g)\r\n",flash4.ffAngVel2Rl);
	printf("   FF_ANG_CONST2RL: (%g)\r\n",flash4.ffAngConst2Rl);
	printf("   FF_ANG_ACC2LR : (%g)\r\n",flash4.ffAngAcc2Lr);
	printf("   FF_ANG_VEL2LR : (%g)\r\n",flash4.ffAngVel2Lr);
	printf("   FF_ANG_CONST2LR : (%g)\r\n",flash4.ffAngConst2Lr);
	printf("   FF_ANG_ACC2LL : (%g)\r\n",flash4.ffAngAcc2Ll);
	printf("   FF_ANG_VEL2LL : (%g)\r\n",flash4.ffAngVel2Ll);
	printf("   FF_ANG_CONST2LL : (%g)\r\n",flash4.ffAngConst2Ll);
	//
	printf("\r\n");
	printf(" ---------- CMT.h ----------\r\n");
	printf("  WALL_TH_R_WALL : (%g)\r\n",flash5[0]);//
	printf("  WALL_TH_L_WALL : (%g)\r\n",flash5[1]);//
	printf("  WALL_TH_FR : (%g)\r\n",flash5[2]);//
	printf("  WALL_TH_FL : (%g)\r\n",flash5[3]);//
	printf("  WALL_CTRL_TH_R : (%g)\r\n",flash5[4]);//
	printf("  WALL_CTRL_TH_L : (%g)\r\n",flash5[5]);//
	printf("  WALL_STOP_CTRL_TH_R : (%g)\r\n",flash5[6]);//
	printf("  WALL_STOP_CTRL_TH_L : (%g)\r\n",flash5[7]);//
	printf("  Sensor_R_Ref : (%g)\r\n",flash5[8]);//
	printf("  Sensor_L_Ref : (%g)\r\n",flash5[9]);//
	printf("  Sensor_Sla_R_Ref : (%g)\r\n",flash5[10]);//
	printf("  Sensor_Sla_L_Ref : (%g)\r\n",flash5[11]);//
	printf("  Sensor_Sla_R_Ref_St : (%g)\r\n",flash5[12]);//
	printf("  Sensor_Sla_L_Ref_St : (%g)\r\n",flash5[13]);//
	printf("  Sensor_R_Ref_Stop : (%g)\r\n",flash5[14]);//
	printf("  Sensor_L_Ref_Stop : (%g)\r\n",flash5[15]);//
	printf("  WALL_TH_R : (%g)\r\n",flash5[16]);//
	printf("  WALL_TH_L : (%g)\r\n",flash5[17]);//
	printf("  WALL_TH_R2 : (%g)\r\n",flash5[18]);//
	printf("  WALL_TH_L2 : (%g)\r\n",flash5[19]);//
	printf("  AFTERCUT : (%g)\r\n",flash5[20]);//
	printf("  WALL_TH_COMB_R : (%g)\r\n",flash5[21]);//
	printf("  WALL_TH_COMB_L : (%g)\r\n",flash5[22]);//
	printf("  WALL_TH_COMB_R_B : (%g)\r\n",flash5[23]);//
	printf("  WALL_TH_COMB_L_B : (%g)\r\n",flash5[24]);//
	printf("  WALL_TH_SLANT_R : (%g)\r\n",flash5[25]);//
	printf("  WALL_TH_SLANT_L : (%g)\r\n",flash5[26]);//
	printf("  WALL_TH_SLANT_R_O : (%g)\r\n",flash5[27]);//
	printf("  WALL_TH_SLANT_L_O : (%g)\r\n",flash5[28]);//
	printf("  COMB_CTRL_TH_R : (%g)\r\n",flash5[29]);//
	printf("  COMB_CTRL_TH_L : (%g)\r\n",flash5[30]);//
	printf("  COMB_CTRL_TH_R_B : (%g)\r\n",flash5[31]);//
	printf("  COMB_CTRL_TH_L_B : (%g)\r\n",flash5[32]);//
	printf("  SLANT_CTRL_TH_COMB_R : (%g)\r\n",flash5[33]);//
	printf("  SLANT_CTRL_TH_COMB_L : (%g)\r\n",flash5[34]);//
	printf("  SUDDENCHANGEB_S : (%g)\r\n",flash5[35]);//
	printf("  OFFSET_COMB_S : (%g)\r\n",flash5[36]);//
	printf("  WALL_KP : (%g)\r\n",flash5[37]);//
	printf("  WALL_KP_HIGH : (%g)\r\n",flash5[38]);//
	printf("  WALL_KD_HIGH : (%g)\r\n",flash5[39]);//
//	printf("  WALL_KD2	(0.05)				//
	printf("  K_HIGH : (%g)\r\n",flash5[40]);//
//	printf("  WALL_KI (0.0)					//
//	printf("  WALL_KD (0.0)					//
	printf("  SUDDENCHANGE : (%g)\r\n",flash5[41]);//
	printf("  SUDDENCHANGECORRECTR : (%g)\r\n",flash5[42]);//
	printf("  SUDDENCHANGECORRECTL : (%g)\r\n",flash5[43]);//
	printf("  SEARCHWALLCUTA_R : (%g)\r\n",flash5[44]);//
	printf("  SEARCHWALLCUTB_R : (%g)\r\n",flash5[45]);//
	printf("  SEARCHWALLCUTA_L : (%g)\r\n",flash5[46]);//
	printf("  SEARCHWALLCUTB_L : (%g)\r\n",flash5[47]);//
	printf("  SLANTWALLCUT_R : (%g)\r\n",flash5[48]);//
	printf("  SLANTWALLCUT_L : (%g)\r\n",flash5[49]);//
	printf("  FWALL_REF_R : (%g)\r\n",flash5[50]);//
	printf("  FWALL_REF_L : (%g)\r\n",flash5[51]);//
	printf("  FWALL_KP_SP : (%g)\r\n",flash5[52]);//
	printf("  FWALL_KP_ANG : (%g)\r\n",flash5[53]);//
	printf("  FWALL_KI_ANG : (%g)\r\n",flash5[54]);//
	printf("  TURN_FWALL_R : (%g)\r\n",flash5[55]);//
	printf("  TURN_FWALL_L : (%g)\r\n",flash5[56]);//
	printf("  SLANTING_F_L : (%g)\r\n",flash5[57]);//
	printf("  SLANTING_F_R : (%g)\r\n",flash5[58]);//
	printf("  KP_WALL_SLANT : (%g)\r\n",flash5[59]);//
	printf("  KP_WALL_SLANT2 : (%g)\r\n",flash5[60]);//
	printf("  KP_WALL_SLANT3 : (%g)\r\n",flash5[61]);//
	printf("  KP_BEFORETURN : (%g)\r\n",flash5[62]);//
	printf("  SLA_FWALL_ALL_B : (%g)\r\n",flash5[63]);//
	printf("  SAITAN_CUTHOSEIR : (%g)\r\n",flash5[64]);//
	printf("  SAITAN_CUTHOSEIL : (%g)\r\n",flash5[65]);//
	printf("  SLANT_FWALL_R : (%g)\r\n",flash5[66]);//
	printf("  SLANT_FWALL_L : (%g)\r\n",flash5[67]);//
	printf("  FWALL_CLOSE : (%g)\r\n",flash5[68]);//
	printf("  FWALL_CLOSE_SLA : (%g)\r\n",flash5[69]);//
	printf("  OFFSET_COMB : (%g)\r\n",flash5[70]);//
	printf("  SUDDENCHANGEB : (%g)\r\n",flash5[71]);//
	printf("  SUDDENCHANGEB2 : (%g)\r\n",flash5[72]);//
	printf("  CONST_A_R : (%g)\r\n",flash5[73]);//
	printf("  CONST_B_R : (%g)\r\n",flash5[74]);//
	printf("  CONST_A_L : (%g)\r\n",flash5[75]);//
	printf("  CONST_B_L : (%g)\r\n",flash5[76]);//
	printf("  CONST_A_FR : (%g)\r\n",flash5[77]);//
	printf("  CONST_B_FR : (%g)\r\n",flash5[78]);//
	printf("  CONST_A_FL : (%g)\r\n",flash5[79]);//
	printf("  CONST_B_FL : (%g)\r\n",flash5[80]);//
	printf("  TIRE_DIAMETER : (%g)\r\n",flash5[81]);//
//	printf("  MMPP : (%g)\r\n",flash5[]);//
	printf("  TREAD : (%g)\r\n",flash5[82]);//
	printf("  SAITANWALLCUTA_R : (%g)\r\n",flash5[83]);//
	printf("  SAITANWALLCUTA_L : (%g)\r\n",flash5[84]);//
	printf("  SAITANWALLCUTB_R : (%g)\r\n",flash5[85]);//
	printf("  SAITANWALLCUTB_L : (%g)\r\n",flash5[86]);//
	printf("  SAITANWALLCUTC_R : (%g)\r\n",flash5[87]);//
	printf("  SAITANWALLCUTC_L : (%g)\r\n",flash5[88]);//
	printf("  SAITANWALLCUTD_R : (%g)\r\n",flash5[89]);//
	printf("  SAITANWALLCUTD_L : (%g)\r\n",flash5[90]);//
	printf("  R_REF_RAW : (%g)\r\n",flash5[91]);//
	printf("  L_REF_RAW : (%g)\r\n",flash5[92]);//
	printf("  ANTEI_ANG : (%g)\r\n",flash5[93]);//
	printf("  ANTEI_DIFF : (%g)\r\n",flash5[94]);//
	printf("  ANTEI_TIME : (%g)\r\n",flash5[95]);//
	printf("  FUSION_ALPHA : (%g)\r\n",flash5[96]);//
	printf("  POLL : (%g)\r\n",flash5[97]);//
	printf("  SEARCHWALLCUTC_R : (%g)\r\n",flash5[98]);//
	printf("  SEARCHWALLCUTC_L : (%g)\r\n",flash5[99]);//
	printf("  WALL_TH_R_SAI : (%g)\r\n",flash5[100]);
	printf("  WALL_TH_L_SAI : (%g)\r\n",flash5[101]);
	printf("  SLANT_CTRL_TH_R : (%g)\r\n",flash5[102]);
	printf("  SLANT_CTRL_TH_L : (%g)\r\n",flash5[103]);
	printf("  KP_SLANT_SIDE : (%g)\r\n",flash5[104]);
	printf("  CUTDIFF1 : (%g)\r\n",flash5[105]);
	printf("  CUTDIFF2 : (%g)\r\n",flash5[106]);
	printf("  K_SLIP : (%g)\r\n",flash5[107]);
	printf("  NULL : (%g)\r\n",flash5[108]);
	printf("  NULL : (%g)\r\n",flash5[109]);
	//
	printf("\r\n");
	printf(" ---------- Others ----------\r\n");
	printf("  SAITANWALLCUT_R : (%g)\r\n",flash6[0]);//
	printf("  SAITANWALLCUT_L : (%g)\r\n",flash6[1]);//
	printf("  TURN_SHORT : (%g)\r\n",flash6[2]);//
	printf("  FWALL_CLOSE_B : (%g)\r\n",flash6[3]);//
	printf("  D_PLUS : (%g)\r\n",flash6[4]);//
	printf("  FWALL_CLOSE2 : (%g)\r\n",flash6[5]);//
	printf("  FWALL_DIFF_BIG : (%g)\r\n",flash6[6]);//
	printf("  NULL : (%g)\r\n",flash6[7]);//
	printf("  NULL : (%g)\r\n",flash6[8]);//
	printf("  NULL : (%g)\r\n",flash6[9]);//
	//
	printf("\r\n");
	printf(" MACHINE_ID : (%g)\r\n",flash_ID);

	reStartWari();
}

void param_receive(void){
	//パラメータ編集ソフトからデータ受信
	int i = 0;
	int t = 0;
	char flag = 0;
	char buff[6000];
	LED2_ON();
	stopWari();
	while(1){
	  	buff[i] = Communication_TerminalRecv();
	  	if(flag == 0){
	  		if(buff[i] == ','){
	  			flag = 1;
	  			i = 0;
	  		}
	 	}
	  	else{
	  		if(buff[i] == 'E'){
	  			buff[i] = '\0';
	  			for(t = 0; t < SEARCHPARANUM; t++){
	  				if(t == 0){
	  					flash1.fanV[t] = (float)(atof(strtok(buff, ",")));
	  				}
	  				else{
	  					flash1.fanV[t] = (float)(atof(strtok(NULL, ",")));
	  				}
	  				flash1.maxV[t] = (float)(atof(strtok(NULL, ",")));
	  				flash1.acc[t] = (float)(atof(strtok(NULL, ",")));
	  				flash1.knownMaxV[t] = (float)(atof(strtok(NULL, ",")));
	  				flash1.knownAcc[t] = (float)(atof(strtok(NULL, ",")));
	  				flash1.slaAlpha[t] = (float)(atof(strtok(NULL, ",")));
	  				flash1.slaW[t] = (float)(atof(strtok(NULL, ",")));
	  				flash1.offsetRA[t] = (float)(atof(strtok(NULL, ",")));
	  				flash1.offsetRB[t] = (float)(atof(strtok(NULL, ",")));
	  				flash1.offsetLA[t] = (float)(atof(strtok(NULL, ",")));
	  				flash1.offsetLB[t] = (float)(atof(strtok(NULL, ",")));
	  				flash1.slaMinV[t] = (float)(atof(strtok(NULL, ",")));
	  				flash1.degR[t] = (float)(atof(strtok(NULL, ",")));
	  				flash1.degL[t] = (float)(atof(strtok(NULL, ",")));
	  				flash1.fWall[t] = (float)(atof(strtok(NULL, ",")));
	  			}

	  			for(t = 0; t < SAITANPARANUM; t++){
	  				flash2[t].fan = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].sin = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].dashv = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].acc = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].dec = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].slantdashv = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].slantacc = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].slantdec = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].fastacc = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].fastdec = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].firstacc = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].acc1 = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].dec1 = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].slantacc1 = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].slantdec1 = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].stopdec = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].minv = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].offsetmaxv = (float)(atof(strtok(NULL, ",")));
	  				//
	  				flash2[t].big90.v = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].big90.alpha = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].big90.w = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].big90.angleR = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].big90.angleL = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].big90.offset1 = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].big90.offset2 = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].big180.v = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].big180.alpha = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].big180.w = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].big180.angleR = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].big180.angleL = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].big180.offset1 = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].big180.offset2 = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].in45.v = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].in45.alpha = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].in45.w = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].in45.angleR = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].in45.angleL = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].in45.offset1 = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].in45.offset2 = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].in135.v = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].in135.alpha = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].in135.w = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].in135.angleR = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].in135.angleL = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].in135.offset1 = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].in135.offset2 = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].v90.v = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].v90.alpha = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].v90.w = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].v90.angleR = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].v90.angleL = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].v90.offset1 = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].v90.offset2 = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].out45.v = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].out45.alpha = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].out45.w = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].out45.angleR = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].out45.angleL = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].out45.offset1 = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].out45.offset2 = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].out135.v = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].out135.alpha = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].out135.w = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].out135.angleR = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].out135.angleL = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].out135.offset1 = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].out135.offset2 = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].kojima.v = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].kojima.alpha = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].kojima.w = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].kojima.angleR = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].kojima.angleL = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].kojima.offset1 = (float)(atof(strtok(NULL, ",")));
	  				flash2[t].kojima.offset2 = (float)(atof(strtok(NULL, ",")));
	  			}
	  			//
	  			flash3.goalSize = (atoi(strtok(NULL, ",")));
	  			flash3.goalX = (atoi(strtok(NULL, ",")));
	  			flash3.goalY = (atoi(strtok(NULL, ",")));
	  			flash3.compSizeX = (atoi(strtok(NULL, ",")));
	  			flash3.compSizeY = (atoi(strtok(NULL, ",")));
	  			flash3.searchTime = (float)(atof(strtok(NULL, ",")));
	  			flash3.lenDown = (float)(atof(strtok(NULL, ",")));
	  			//
	  			flash4.speedKP = (float)(atof(strtok(NULL, ",")));
	  			flash4.speedKI = (float)(atof(strtok(NULL, ",")));
	  			flash4.speedKD = (float)(atof(strtok(NULL, ",")));
	  			flash4.speedKP2 = (float)(atof(strtok(NULL, ",")));
	  			flash4.speedKI2 = (float)(atof(strtok(NULL, ",")));
	  			flash4.speedKD2 = (float)(atof(strtok(NULL, ",")));
	  			flash4.omegaKP = (float)(atof(strtok(NULL, ",")));
	  			flash4.omegaKI = (float)(atof(strtok(NULL, ",")));
	  			flash4.omegaKD = (float)(atof(strtok(NULL, ",")));
	  			flash4.omegaKP2 = (float)(atof(strtok(NULL, ",")));
	  			flash4.omegaKI2 = (float)(atof(strtok(NULL, ",")));
	  			flash4.omegaKD2 = (float)(atof(strtok(NULL, ",")));
	  			flash4.omegaKP2R = (float)(atof(strtok(NULL, ",")));
	  			flash4.omegaKI2R = (float)(atof(strtok(NULL, ",")));
	  			flash4.omegaKD2R = (float)(atof(strtok(NULL, ",")));
	  			flash4.omegaKP2L = (float)(atof(strtok(NULL, ",")));
	  			flash4.omegaKI2L = (float)(atof(strtok(NULL, ",")));
	  			flash4.omegaKD2L = (float)(atof(strtok(NULL, ",")));
	  			flash4.angleKP = (float)(atof(strtok(NULL, ",")));
	  			flash4.speedKPminus = (float)(atof(strtok(NULL, ",")));
	  			flash4.speedKIminus = (float)(atof(strtok(NULL, ",")));
	  			flash4.speedKDminus = (float)(atof(strtok(NULL, ",")));
	  			flash4.omegaKPminus = (float)(atof(strtok(NULL, ",")));
	  			flash4.omegaKIminus = (float)(atof(strtok(NULL, ",")));
	  			flash4.omegaKDminus = (float)(atof(strtok(NULL, ",")));
	  			flash4.ffAccel = (float)(atof(strtok(NULL, ",")));
	  			flash4.ffSpeed = (float)(atof(strtok(NULL, ",")));
	  			flash4.ffConst = (float)(atof(strtok(NULL, ",")));
	  			flash4.ffAngAcc = (float)(atof(strtok(NULL, ",")));
	  			flash4.ffAngVel = (float)(atof(strtok(NULL, ",")));
	  			flash4.ffAngConst = (float)(atof(strtok(NULL, ",")));
	  			flash4.ffAccel2R = (float)(atof(strtok(NULL, ",")));
	  			flash4.ffSpeed2R = (float)(atof(strtok(NULL, ",")));
	  			flash4.ffConst2R = (float)(atof(strtok(NULL, ",")));
	  			flash4.ffAccel2L = (float)(atof(strtok(NULL, ",")));
	  			flash4.ffSpeed2L = (float)(atof(strtok(NULL, ",")));
	  			flash4.ffConst2L = (float)(atof(strtok(NULL, ",")));
	  			flash4.ffAngAcc2Rr = (float)(atof(strtok(NULL, ",")));
	  			flash4.ffAngVel2Rr = (float)(atof(strtok(NULL, ",")));
	  			flash4.ffAngConst2Rr = (float)(atof(strtok(NULL, ",")));
	  			flash4.ffAngAcc2Rl = (float)(atof(strtok(NULL, ",")));
	  			flash4.ffAngVel2Rl = (float)(atof(strtok(NULL, ",")));
	  			flash4.ffAngConst2Rl = (float)(atof(strtok(NULL, ",")));
	  			flash4.ffAngAcc2Lr = (float)(atof(strtok(NULL, ",")));
	  			flash4.ffAngVel2Lr = (float)(atof(strtok(NULL, ",")));
	  			flash4.ffAngConst2Lr = (float)(atof(strtok(NULL, ",")));
	  			flash4.ffAngAcc2Ll = (float)(atof(strtok(NULL, ",")));
	  			flash4.ffAngVel2Ll = (float)(atof(strtok(NULL, ",")));
	  			flash4.ffAngConst2Ll = (float)(atof(strtok(NULL, ",")));
	  			//
	  			for(t = 0; t < 110; t++){
	  				flash5[t] = (float)(atof(strtok(NULL, ",")));
	  			}
	  			for(t = 0; t < 10; t++){
	  				flash6[t] = (float)(atof(strtok(NULL, ",")));
	  			}
	  			flash_ID = (float)(atof(strtok(NULL, ",")));
 				i = 0;
				break;
			}else{
	  			//if(buff[i] != ' '){
	  			i += 1;
	  			//}
	  		}
	  	}
	}
	LED2_OFF();
	param_write();
	reStartWari();
}


  /*
  while (1)
  {

//	  float acc = 100*IMU_GetAccel_X();
//	  float gyro = 100*IMU_GetGyro_Z();
	  float bat = Battery_GetVoltage();
	  if(st == 0){
		  //uint16_t eee = ENC_L_GetAngle();
		  eeee = ENC_R_GetAngle();
		  st = 1;
	  }
	  else if(st == 1){
		  //uint16_t eee = ENC_L_GetAngle();
		  eeee2 = ENC_L_GetAngle();
		  st = 0;
	  }
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 40);//(uint32_t)(40000/100*100/1000)-1);
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 40);//(uint32_t)(40000/100*100/1000)-1);
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	//  uint8_t who = IMU_CheckWHOAMI();
	 // printf("WHO_AM_I : %d",who);
//	  printf("<IMU> Accel_X: %d[m/s^2], Gyro_Z: %d[rad/s]\r\n",
//	  				(short)acc, (short)gyro);// line++;
	  printf("<IMU> speed: %d[m/s], deg: %d[deg]\r\n",
	  	  				(short)test_speed, (short)test_deg);// line++;
	  printf("<Battery> %d[V]\r\n", (short)(bat*100));
	  printf("<ENC_L> %d\r\n", (short)eeee2);
	  printf("<ENC_R> %d\r\n", (short)eeee);
	  printf("<IR Sensor> FL: %d, SL: %d SR: %d, FR: %d\r\n",
	  				(short)Sensor_GetValue(3), (short)Sensor_GetValue(2), (short)Sensor_GetValue(1), (short)Sensor_GetValue(0));
	 // while ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0));
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, GPIO_PIN_SET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, GPIO_PIN_RESET);
	  HAL_Delay(100);*/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//  }
  /* USER CODE END 3 */
//}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
