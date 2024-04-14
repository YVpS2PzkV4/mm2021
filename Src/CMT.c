#include "index.h"

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
#include<stdlib.h>
//#include<mathf.h>

#define ARM_MATH_CM4
#include "arm_math.h"
#include "arm_const_structs.h"

extern volatile int V_bat;
extern volatile int V_bat_a;
extern volatile int V_bat_b;
volatile short V_bat_current[5] = {0,0,0,0,0};
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
extern volatile float Before_R_Dis;
extern volatile float Before_L_Dis;
extern volatile float Before_FR_Dis;
extern volatile float Before_FL_Dis;

volatile float R_Dis_Current[5] = {0,0,0,0,0};
volatile float L_Dis_Current[5] = {0,0,0,0,0};
volatile float FR_Dis_Current[5] = {0,0,0,0,0};
volatile float FL_Dis_Current[5] = {0,0,0,0,0};
volatile float R_Dis_Ave = 0;
volatile float L_Dis_Ave = 0;
volatile float R_Dis_Current_A[5] = {0,0,0,0,0};
volatile float L_Dis_Current_A[5] = {0,0,0,0,0};
volatile float FR_Dis_Ave = 0;
volatile float FL_Dis_Ave = 0;
volatile float R_Dis_Ave_B = 0;
volatile float L_Dis_Ave_B = 0;


//エンコーダ角度系のグローバル変数
extern unsigned volatile int	locate_l;				//現在の車軸位置	[無次元]
extern unsigned volatile int	locate_r;				//現在の車軸位置	[無次元]

unsigned volatile int locate_raw_r = 0;
unsigned volatile int locate_raw_l = 0;	//エンコーダ調整用　生値

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
//
extern volatile float			speed_r2;				//ターン中の速度制御用？
extern volatile float 			speed_l2;
extern volatile float 			speed_old_r2;
extern volatile float 			speed_old_l2;
extern volatile float 			speed2;
//
extern volatile float			p_speed;				//過去の車体速度	[m/s]
extern volatile float			p_speed_r;				//過去の車体速度	[m/s]
extern volatile float			p_speed_l;				//過去の車体速度	[m/s]
extern volatile float			len_mouse;
extern volatile float			I_speed;
extern volatile float			I_speed2;

extern volatile float			gyro_ref;
extern volatile float			p_ang_vel;
extern volatile float			ang_vel;
extern volatile float			ang_vel_current[5];
extern volatile float			ang_vel_ave;
extern volatile float			ang_vel_ave_b;
extern volatile float			new_ang_vel;
extern volatile float			I_ang_vel;
extern volatile float			degree;
extern volatile float			I_degree;
//extern volatile float			I_tar_degree;
extern volatile float			tar_ang_vel;				//目標角速度		[rad/s]
extern volatile float			tar_degree;				//目標角度		[deg]
extern volatile float			max_degree;				//旋回時の最大角度	[deg]
extern volatile float			start_degree;				//走行進入時の車体角度	[deg]
extern volatile float			max_ang_vel;				//最高角速度		[rad/s]
extern volatile float			max_ang_vel_sin;				//最高角速度		[rad/s]
extern volatile float			ang_acc;				//角加速度		[rad/ss
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
extern volatile float 			tar_speed_r;
extern volatile float 			tar_speed_l;

//タイマ系グローバル変数
extern unsigned volatile int		timer;					//1mSごとにカウントアップされる変数.
extern unsigned volatile long		searchtimer;					//1mSごとにカウントアップされる変数.

//制御用グローバル変数
extern volatile float			I_tar_speed;				//目標速度のI成分
extern volatile float			I_tar_ang_vel;				//目標角速度のI成分

volatile float I_len = 0;

extern volatile float 			end_speed;

//ログ用のグローバル変数
extern volatile long			log_timer;				//ログ取りようのタイマ
extern volatile int			log_flag;				//ログ取得のタイミング

extern volatile float 			torque_r;
extern volatile float 			torque_l;
extern volatile float 			FF_r;
extern volatile float 			FF_l;
extern volatile float 			FB_r;
extern volatile float 			FB_l;

extern volatile float			mouse_x;
extern volatile float			mouse_y;

volatile int WallCtrlEnbl = 0;
volatile float WallCtrlErrP = 0;
volatile float WallCtrlErr = 0;
volatile int Sensor_R_Ctrl_Flag = 0;
volatile int Sensor_L_Ctrl_Flag = 0;
volatile float Sensor_R_Err = 0;
volatile float Sensor_L_Err = 0;
volatile float WallCtrlD = 0;
volatile float WallCtrlI = 0;
volatile float WallCtrlOmegaP = 0;
volatile float WallCtrlOmega = 0;

volatile float R_Raw_Err = 0;	//センサ生値のエラー
volatile float L_Raw_Err = 0;
volatile float Sensor_Raw_Err = 0;

volatile int FF_Enbl = 0;
volatile int FB_Enbl = 0;

static int batout = 0;
static int failcnt1 = 0;
static int failcnt2 = 0;
static int failcnt3 = 0;
static int failcnt4 = 0;
static int failcnt5 = 0;
static int failcnt6 = 0;

volatile int wallCutTask = 0;
volatile int firstread = 0;
volatile int slafwallflag = 0;
volatile int slaswallflag = 0;
volatile int fwallnoflag = 0;

volatile int turnfwallflag = 0;
volatile int slantfwallflag = 0;

volatile float offsetplus = 0;
volatile int offsetcnt = 0;
volatile float offsetplus_st = 0;
volatile int offsetcnt_st = 0;

volatile int searching_flag = 0;

extern volatile int tracecnt;
extern unsigned int slantingtask;

volatile float lentmp = 0;
volatile int blockcnt = 0;

volatile char rsensor_sudden_minus = 0;
volatile char lsensor_sudden_minus = 0;
volatile char rsensor_sudden_plus = 0;
volatile char lsensor_sudden_plus = 0;

volatile char comb = 0;

volatile char BatCheckSW = 1;

volatile char noslaflag = 0;

volatile char turn_short = 0;

extern volatile char saitaning;

volatile float beecombr = 0;	//櫛制御（壁切れ距離差）系の何か
volatile float beecombl = 0;
volatile float prev_combr = 0;	//櫛制御（壁切れ距離差）系の何か　過去の値（片櫛のみ用）
volatile float prev_combl = 0;
volatile int combbb = 0;

volatile float beecombr_s = 0;	//斜め制御（壁切れ距離差）系の何か
volatile float beecombl_s = 0;
volatile int combbb_s = 0;

volatile int walloncnt_r = 0;
volatile int walloncnt_l = 0;

volatile float len_true = 0;
extern volatile float next;
extern volatile char lastslant;

extern volatile float next2;
volatile char slantread = 0;
extern volatile char slantblock;

volatile short V_bat_ref = 0;

extern volatile int mode;

/*extern char wallDataSave[5];
extern unsigned char dataSaveX[5];
extern unsigned char dataSaveY[5];*/

volatile float kojima_kd = 0;

extern unsigned char map[MAZESIZE_X][MAZESIZE_Y];

volatile char g_Sensor_Flag = 0;

volatile char StopFlagR = 0;
volatile char StopFlagL = 0;

volatile float peekr = 0;
volatile float nowpeekr = 0;
volatile float peekl = 0;
volatile float nowpeekl = 0;

volatile char turn_read = 0;	//壁なしターン用の何か
extern volatile char emode;
extern volatile char rl;

extern volatile float ichi_x;//オドメトリ用座標
extern volatile float ichi_y;
extern volatile float tar_ichi_x;//オドメトリ用座標
extern volatile float tar_ichi_y;

volatile char turnwflag = 0;	//最短ターン前フラグ(前壁補正用)
volatile float offsetdis = 0;

volatile float sla_fwall_a = 0;

//extern volatile char yaba;

extern volatile int tsin;
extern volatile char accflag;

extern volatile int tsin2;
extern volatile float accel_sin;
extern volatile float max_speed_sin;
extern volatile char accflag2;

volatile float ang_vel_from_enc = 0;	//エンコーダで求めた角速度

volatile char off_flag = 0;	//最短のターンオフセット中かどうか

volatile float I_fwall = 0;
extern volatile int fwallcnt;

extern volatile char comb_onoff;

extern volatile char endsla;

extern volatile char longst;

extern t_dijkstra nodeinfo[32][32][12];

volatile char naname_wallcut = 0;//ターン前の斜め壁切れしたか？

volatile char R_Ctrl_Disable = 0;
volatile char L_Ctrl_Disable = 0;	//壁切れ後しばらくトレースしない
volatile float Dis_to_Able = 0;
volatile float Dis_to_Able2 = 0;	//壁切れ後しばらくトレースしない


extern uint16_t			adc_value[5];		// AD変換値
extern uint16_t			battery_value;		// バッテリ電圧の生データ
extern uint16_t	sensor_value_on[4];	// IRセンサの生データ
extern uint16_t	sensor_value_off[4];	// IRセンサの生データ

extern volatile float FanVolt;	//吸引ファンの印加電圧
extern volatile float			Duty_fan;
extern volatile char FanEnbl;	//吸引ファンの印加電圧

volatile float ErrSpeed = 0;
volatile float ErrSpeedP = 0;
volatile float ErrAng = 0;
volatile float ErrAngP = 0;

volatile float hensa_speed = 0;
volatile float hensa_speed_p = 0;
volatile float hensa_speed2 = 0;
volatile float hensa_speed_p2 = 0;
volatile float hensa_ang = 0;
volatile float hensa_ang_p = 0;

volatile float nowG = 0;	//横G = 中心速度[m/s]*角速度[rad/s]/9.8[m/s^2]

const short encL_RawData[120] = {1,28,52,84,112,140,173,196,229,256,285,313,341,369,400,436,468,493,524,589,620,656,692,724,757,796,832,865,901,941,976,1016,1053,1092,1132,1168,1208,1245,1281,1321,1392,1425,1461,1497,1537,1568,1605,1640,1673,1709,1736,1772,1809,1845,1877,1908,1945,1972,2004,2068,2105,2137,2164,2197,2225,2257,2285,2312,2345,2377,2404,2437,2468,2497,2528,2557,2588,2621,2688,2721,2756,2789,2821,2856,2893,2924,2960,2993,3028,3065,3097,3133,3169,3205,3236,3272,3308,3373,3413,3441,3477,3508,3540,3577,3604,3632,3669,3700,3733,3764,3793,3824,3856,3884,3916,3949,4008,4040,4069,4097};//{1,16,28,44,52,69,84,97,112,124,137,152,164,181,196,208,220,236,248,265,280,292,309,321,341,352,364,381,397,409,425,440,453,468,484,496,512,529,541,557,572,589,601,620,637,653,668,689,709,724,744,764,781,800,817,837,856,877,896,920,941,956,981,1001,1024,1048,1069,1089,1109,1132,1153,1173,1196,1221,1240,1261,1281,1301,1324,1344,1364,1384,1404,1425,1444,1465,1480,1500,1521,1537,1557,1577,1592,1612,1629,1648,1664,1684,1704,1724,1741,1756,1780,1792,1812,1828,1852,1869,1888,1905,1924,1941,1961,1976,1996,2017,2037,2053,2072,2088,2108,2128,2149,2164,2184,2204,2220,2240,2260,2280,2297,2317,2332,2353,2373,2388,2404,2425,2441,2461,2477,2497,2512,2533,2548,2568,2588,2604,2624,2641,2660,2672,2693,2708,2724,2741,2756,2773,2789,2808,2825,2845,2861,2876,2896,2912,2932,2948,2972,2988,3005,3025,3044,3061,3080,3100,3121,3141,3160,3181,3200,3220,3240,3257,3277,3296,3316,3337,3357,3373,3393,3417,3433,3453,3472,3493,3508,3525,3544,3560,3577,3596,3613,3629,3644,3661,3676,3692,3709,3725,3740,3756,3768,3781,3796,3812,3824,3836,3853,3861,3877,3889,3901,3913,3924,3937,3949,3961,3977,3988,3997,4008,4021,4036,4045,4057,4069,4081,4097};//{1,28,56,84,112,164,196,224,253,280,313,369,400,428,460,521,552,580,613,681,713,740,781,849,885,916,949,1021,1053,1092,1129,1196,1236,1269,1305,1377,1408,1444,1476,1544,1577,1609,1636,1709,1736,1769,1801,1860,1884,1917,1941,1996,2024,2057,2081,2133,2161,2184,2213,2264,2293,2317,2341,2392,2416,2441,2468,2512,2540,2564,2588,2644,2669,2693,2721,2773,2796,2821,2845,2896,2924,2952,2981,3025,3053,3076,3104,3156,3181,3209,3233,3285,3313,3337,3364,3413,3441,3469,3493,3544,3573,3596,3624,3676,3700,3725,3749,3800,3829,3856,3881,3933,3961,3988,4013,4069,4097};//{1,13,25,37,44,69,81,93,104,117,128,152,164,176,188,205,213,236,244,261,265,280,296,309,316,328,336,348,369,376,385,392,405,409,428,436,448,453,465,484,488,505,512,521,545,552,565,577,597,604,617,625,648,661,672,681,704,716,728,737,764,776,784,796,824,837,852,861,885,901,916,925,953,964,976,992,1021,1036,1053,1069,1096,1113,1125,1140,1173,1189,1204,1221,1252,1273,1284,1305,1336,1353,1373,1392,1425,1441,1461,1476,1509,1524,1540,1561,1597,1609,1624,1645,1676,1697,1712,1732,1765,1784,1801,1816,1852,1864,1884,1900,1933,1948,1964,1981,2013,2029,2044,2065,2096,2113,2128,2149,2177,2197,2208,2228,2260,2276,2293,2305,2336,2348,2365,2377,2408,2425,2441,2452,2480,2497,2509,2524,2557,2568,2581,2597,2629,2641,2660,2672,2700,2712,2728,2745,2773,2784,2796,2808,2836,2852,2864,2876,2901,2912,2929,2941,2960,2976,2988,3005,3028,3041,3053,3068,3093,3104,3121,3128,3156,3169,3184,3196,3224,3236,3253,3265,3292,3305,3316,3333,3361,3376,3393,3405,3433,3448,3460,3477,3505,3520,3532,3549,3577,3589,3601,3613,3641,3652,3669,3685,3709,3720,3737,3749,3776,3788,3800,3812,3841,3853,3865,3877,3901,3913,3924,3940,3968,3980,3988,4004,4028,4045,4057,4069,4097};//{1,16,32,44,56,73,84,97,112,128,140,157,168,185,200,217,229,244,273,289,301,316,333,345,364,381,392,409,425,436,453,465,477,496,512,524,541,572,589,608,620,641,653,672,689,704,724,740,757,776,793,809,829,844,865,880,916,932,949,968,988,1004,1021,1041,1057,1077,1092,1113,1132,1149,1165,1184,1204,1221,1236,1273,1288,1308,1321,1341,1361,1373,1392,1413,1428,1441,1456,1476,1488,1513,1521,1537,1557,1573,1605,1620,1640,1653,1673,1693,1712,1729,1741,1760,1777,1792,1809,1828,1845,1860,1877,1897,1908,1941,1957,1976,1996,2013,2029,2041,2065,2077,2093,2108,2133,2144,2161,2180,2197,2208,2228,2240,2273,2288,2305,2320,2336,2348,2365,2377,2388,2404,2421,2437,2452,2465,2480,2492,2509,2524,2537,2568,2585,2601,2612,2629,2644,2657,2669,2684,2697,2712,2724,2741,2756,2768,2780,2801,2816,2828,2856,2873,2884,2896,2912,2929,2941,2952,2969,2985,3000,3017,3028,3044,3061,3076,3088,3104,3116,3148,3160,3176,3189,3200,3217,3233,3245,3260,3277,3289,3305,3320,3333,3348,3361,3376,3393,3408,3436,3453,3464,3477,3493,3508,3525,3540,3556,3568,3584,3601,3613,3629,3644,3656,3673,3689,3704,3733,3744,3761,3776,3788,3809,3824,3836,3853,3868,3881,3896,3909,3924,3940,3952,3968,3980,3997,4025,4040,4053,4069,4081,4097};//{1,13,25,37,44,61,73,81,97,104,117,128,137,148,157,173,181,196,217,229,236,244,256,265,276,285,296,309,321,333,341,352,364,381,388,400,412,433,440,453,460,468,481,488,501,508,517,529,536,545,552,569,572,584,597,604,625,637,644,648,661,668,677,684,696,704,713,721,733,740,752,764,772,784,796,812,824,837,849,856,868,880,892,905,916,929,941,953,968,976,988,1001,1012,1021,1044,1057,1069,1084,1096,1109,1120,1132,1144,1160,1173,1189,1201,1213,1225,1236,1252,1264,1281,1308,1324,1341,1353,1364,1380,1397,1413,1425,1441,1453,1468,1485,1500,1513,1528,1544,1557,1568,1597,1612,1629,1645,1657,1673,1688,1700,1712,1729,1741,1756,1769,1789,1801,1812,1828,1837,1857,1881,1897,1908,1921,1936,1945,1957,1972,1984,1993,2004,2017,2029,2041,2053,2060,2068,2081,2093,2108,2120,2133,2140,2153,2161,2168,2180,2189,2201,2208,2220,2228,2240,2249,2264,2273,2280,2288,2308,2320,2329,2341,2356,2360,2373,2385,2392,2404,2413,2421,2432,2444,2456,2465,2472,2485,2492,2512,2524,2533,2548,2552,2564,2573,2585,2592,2601,2612,2621,2629,2641,2648,2657,2669,2681,2693,2708,2721,2728,2745,2753,2765,2773,2780,2796,2808,2816,2828,2840,2852,2861,2869,2881,2893,2901,2924,2932,2948,2960,2969,2981,2988,2996,3008,3020,3028,3041,3048,3065,3073,3085,3097,3109,3116,3141,3153,3165,3172,3184,3196,3209,3217,3229,3240,3253,3260,3272,3285,3292,3301,3313,3325,3333,3361,3373,3381,3388,3400,3413,3417,3429,3436,3448,3457,3469,3477,3488,3500,3508,3520,3532,3537,3556,3565,3577,3584,3596,3604,3613,3624,3632,3644,3652,3661,3669,3673,3685,3692,3704,3709,3716,3737,3744,3753,3764,3773,3781,3793,3800,3809,3821,3829,3841,3848,3861,3868,3877,3889,3901,3909,3933,3944,3952,3964,3973,3985,3997,4004,4021,4033,4040,4053,4064,4076,4088,4097};//{1,13,25,37,49,61,73,81,97,117,133,145,157,164,181,193,205,217,241,256,268,280,292,304,321,333,345,369,385,392,409,421,436,448,460,477,501,512,524,541,552,569,577,589,604,632,644,656,677,689,701,716,728,744,772,789,809,824,832,852,868,885,896,932,949,968,985,1001,1016,1036,1053,1069,1104,1125,1137,1156,1168,1193,1204,1225,1236,1273,1284,1305,1321,1341,1356,1373,1389,1404,1441,1461,1473,1493,1504,1524,1537,1561,1573,1609,1620,1640,1653,1673,1684,1700,1717,1732,1760,1772,1789,1804,1821,1832,1849,1857,1872,1905,1921,1936,1948,1964,1981,1996,2013,2024,2053,2068,2084,2101,2113,2128,2144,2156,2168,2197,2213,2228,2240,2257,2269,2280,2293,2308,2332,2353,2360,2377,2388,2404,2416,2432,2441,2468,2480,2497,2509,2524,2537,2552,2568,2581,2604,2621,2633,2644,2660,2672,2684,2700,2712,2745,2753,2768,2780,2796,2813,2825,2836,2852,2884,2893,2908,2921,2941,2952,2969,2981,2996,3025,3041,3053,3065,3085,3097,3113,3128,3141,3169,3184,3200,3212,3233,3245,3260,3272,3289,3316,3333,3345,3361,3376,3393,3408,3420,3433,3464,3477,3488,3500,3517,3529,3544,3556,3573,3596,3613,3629,3641,3656,3669,3685,3697,3713,3740,3753,3768,3781,3793,3809,3821,3836,3848,3861,3892,3901,3916,3933,3944,3957,3973,3988,4001,4028,4045,4057,4069,4084,4097};
const short encR_RawData[102] = {1,37,73,109,152,185,229,268,313,357,400,448,496,548,597,644,696,749,844,892,944,988,1041,1084,1132,1180,1225,1276,1321,1368,1417,1456,1504,1544,1588,1633,1673,1753,1792,1828,1864,1900,1936,1969,2001,2037,2065,2101,2133,2164,2197,2232,2269,2297,2332,2368,2441,2472,2509,2545,2585,2616,2657,2693,2728,2765,2804,2836,2873,2912,2948,2985,3020,3061,3097,3133,3165,3236,3272,3308,3345,3381,3413,3453,3484,3520,3553,3589,3624,3656,3692,3725,3756,3793,3824,3861,3924,3957,3992,4028,4060,4097};//{1,16,37,52,69,88,109,124,145,164,185,205,220,241,256,280,301,324,345,364,388,409,433,457,477,501,524,557,577,601,625,648,677,701,721,744,769,793,820,841,865,889,913,932,956,981,1004,1029,1053,1077,1101,1120,1149,1168,1193,1216,1240,1264,1288,1312,1341,1356,1380,1408,1428,1448,1473,1488,1513,1533,1552,1573,1592,1617,1640,1660,1681,1700,1721,1741,1760,1780,1801,1821,1840,1860,1881,1897,1917,1948,1969,1984,2001,2020,2037,2053,2068,2084,2101,2113,2128,2144,2161,2177,2189,2208,2225,2240,2257,2273,2288,2308,2325,2341,2356,2373,2388,2404,2421,2437,2452,2472,2485,2504,2517,2533,2548,2564,2581,2601,2616,2633,2648,2664,2684,2700,2717,2733,2748,2768,2784,2801,2816,2836,2849,2864,2876,2893,2908,2924,2936,2952,2969,2981,2993,3008,3020,3032,3048,3056,3068,3080,3097,3109,3116,3133,3145,3156,3169,3181,3193,3205,3217,3233,3245,3257,3272,3289,3301,3313,3325,3337,3348,3373,3385,3400,3413,3420,3436,3448,3460,3472,3484,3500,3512,3525,3537,3549,3560,3573,3589,3601,3617,3629,3641,3652,3669,3680,3697,3704,3720,3737,3744,3761,3773,3788,3800,3816,3829,3844,3856,3868,3881,3892,3904,3921,3937,3952,3968,3985,3997,4013,4021,4033,4045,4060,4073,4084,4097};//{1,25,52,76,100,124,181,205,233,261,285,341,364,397,421,448,477,532,560,589,617,644,668,728,757,784,817,844,905,932,964,992,1021,1053,1113,1140,1168,1201,1233,1261,1317,1344,1373,1401,1461,1485,1513,1540,1592,1620,1645,1669,1721,1744,1772,1797,1845,1869,1897,1917,1964,1993,2013,2041,2084,2113,2137,2161,2208,2228,2257,2280,2329,2353,2377,2401,2444,2472,2492,2521,2564,2588,2616,2636,2684,2708,2728,2753,2801,2825,2849,2869,2917,2941,2965,2988,3037,3061,3085,3109,3156,3181,3205,3229,3277,3301,3325,3352,3396,3420,3441,3464,3508,3532,3556,3580,3624,3644,3669,3692,3740,3761,3785,3805,3853,3877,3901,3924,3973,3997,4021,4045,4097};//{1,13,28,44,73,88,100,117,145,157,173,188,217,236,248,265,296,309,324,341,369,385,405,416,448,465,481,493,529,545,557,572,604,620,637,648,684,701,716,733,764,781,796,812,844,861,880,896,929,949,964,981,1016,1036,1053,1072,1109,1125,1144,1160,1201,1216,1236,1256,1293,1312,1329,1349,1384,1401,1420,1441,1476,1493,1513,1528,1561,1580,1597,1612,1648,1664,1684,1697,1732,1749,1765,1784,1816,1832,1849,1864,1897,1912,1928,1945,1981,1996,2013,2024,2057,2072,2088,2105,2133,2153,2164,2180,2213,2228,2245,2260,2293,2305,2320,2336,2368,2385,2401,2416,2444,2461,2477,2489,2521,2537,2552,2568,2601,2616,2633,2648,2681,2697,2712,2728,2765,2784,2801,2816,2852,2869,2884,2905,2941,2952,2972,2988,3025,3041,3061,3076,3113,3128,3145,3160,3196,3212,3233,3245,3277,3292,3308,3325,3357,3373,3388,3400,3433,3448,3464,3477,3508,3525,3540,3556,3584,3601,3617,3629,3661,3676,3692,3704,3737,3749,3768,3781,3812,3829,3844,3861,3892,3913,3928,3944,3980,3992,4008,4028,4060,4076,4097};//{1,13,25,41,64,81,93,109,137,148,161,173,200,217,229,244,273,285,296,313,341,352,369,385,412,428,445,460,493,508,524,541,572,592,604,625,661,672,692,709,744,761,776,796,832,849,865,885,920,936,956,976,1012,1029,1048,1069,1109,1125,1144,1165,1204,1221,1240,1256,1296,1312,1332,1353,1384,1408,1428,1444,1480,1497,1516,1533,1564,1585,1600,1617,1653,1669,1681,1697,1729,1744,1760,1772,1804,1816,1832,1849,1877,1888,1905,1917,1948,1964,1981,1993,2020,2032,2048,2065,2093,2108,2125,2137,2168,2184,2201,2213,2245,2260,2276,2293,2320,2336,2353,2365,2397,2413,2425,2444,2472,2485,2500,2512,2540,2557,2573,2581,2609,2621,2636,2648,2672,2688,2700,2712,2741,2753,2765,2780,2804,2816,2833,2845,2869,2884,2896,2908,2932,2945,2957,2969,2993,3005,3017,3032,3056,3068,3080,3093,3116,3128,3141,3156,3181,3193,3205,3217,3240,3257,3265,3280,3305,3316,3328,3345,3364,3376,3388,3400,3424,3436,3448,3457,3484,3493,3505,3517,3537,3549,3560,3573,3596,3604,3617,3629,3652,3661,3673,3685,3700,3716,3725,3737,3756,3768,3776,3788,3809,3816,3829,3836,3861,3868,3881,3889,3913,3921,3933,3940,3964,3977,3985,3992,4016,4028,4036,4053,4073,4084,4097};//{1,16,37,56,88,109,128,145,181,200,220,241,276,292,313,333,372,388,412,428,468,484,505,524,569,584,604,628,665,689,709,724,764,784,805,820,861,885,901,925,964,985,1004,1029,1064,1084,1109,1129,1165,1184,1204,1225,1269,1293,1312,1332,1373,1392,1413,1432,1473,1493,1513,1533,1568,1588,1609,1624,1660,1681,1697,1717,1753,1769,1784,1801,1837,1852,1872,1893,1924,1941,1957,1972,2008,2024,2041,2057,2093,2108,2125,2140,2177,2192,2208,2225,2260,2280,2297,2312,2345,2365,2380,2401,2432,2452,2468,2485,2517,2537,2552,2568,2601,2621,2641,2657,2688,2708,2724,2741,2777,2793,2813,2828,2864,2881,2896,2917,2952,2969,2988,3005,3041,3056,3073,3093,3128,3148,3165,3181,3217,3233,3248,3268,3301,3316,3337,3352,3385,3400,3417,3433,3469,3484,3500,3520,3556,3573,3584,3604,3637,3652,3669,3685,3720,3737,3749,3768,3800,3821,3836,3853,3892,3909,3928,3944,3980,3997,4016,4036,4076,4097};
volatile char locate_flag_l = 0;
volatile char locate_flag_r = 0;

extern volatile float Nap_a;
extern volatile float Nap_n;
extern volatile float Nap_h;
extern volatile float Nap_x;
extern volatile char Nap_flag;
extern volatile float Nap_b;

extern volatile double Angle_G;		//絶対角度（ジャイロのリファレンスとるときにリセット）
extern volatile double Tar_Angle_G;		//目標絶対角度（ジャイロのリファレンスとるときにリセット）

volatile unsigned short nowLog[10] = {0,0,0,0,0,0,0,0,0,0};
volatile unsigned int countLog = 0;

extern volatile char flash6_erased;
volatile char flashLogFlag = 0;

volatile float speedFF = 0;	//FF並進方向制御量
volatile float speedFB = 0;	//FB並進方向制御量
//volatile float angleFF = 0;

volatile float accelX_gyro = 0;	//ジャイロで計測した加速度
volatile float speedX_gyro = 0;

extern volatile char anteiflag;
extern volatile long anteitimer;//整定判定用タイマー

volatile float speedE = 0;	//加速度センサ使った速度
volatile float speedE_prev = 0;

volatile unsigned short pollingR = 0;
volatile unsigned short pollingL = 0;	//最短櫛壁切れ判定用　ある程度の時間AD値が閾値以上

volatile float R_Diff_Smoothed = 0;
volatile float L_Diff_Smoothed = 0;	//赤外線センサ平滑化微分値

volatile float slip_new = 0;
volatile float slip_old = 0;
volatile float slip_diff = 0;//コジマ式スリップ角

extern volatile float vvv_r;
extern volatile float vvv_l;

volatile float FF_prev_R = 0;
volatile float FF_prev_L = 0;	//FF一個前値

unsigned short getEncTable(char RorL,short num){	//補正テーブル作成関数
	short a = 0,b = 0;
	unsigned short value = 0;
	//unsigned short nR,nL;
	//nR = sizeof(encR_RawData)/sizeof(encR_RawData[0])-1;
	//nL = sizeof(encL_RawData)/sizeof(encL_RawData[0])-1;
	if(num == 0){
		a = 4096;
	}
	else{
		a = num;
	}
	if(RorL == RIGHT){
		while(b != 101 && encR_RawData[b+1] < a){
			b++;
		}
		if(b == 101){
			value = a;//(short)(encR_RawData[b]+(float)(a-encR_RawData[b])*4049.0f/48.0f/(float)(encR_RawData[b]-encR_RawData[b-1]));
		}
		else{
			value = (short)((float)(b*4096.0f/101.0f)+(float)(a-encR_RawData[b])*4096.0f/101.0f/(float)(encR_RawData[b+1]-encR_RawData[b]))+1;
		}
	}
	else if(RorL == LEFT){
		while(b != 119 && encL_RawData[b+1] < a){
			b++;
		}
		if(b == 119){
			value = a;//(short)(encR_RawData[b]+(float)(a-encR_RawData[b])*4049.0f/48.0f/(float)(encR_RawData[b]-encR_RawData[b-1]));
		}
		else{
			value = (short)((float)(b*4096.0f/119.0f)+(float)(a-encL_RawData[b])*4096.0f/119.0f/(float)(encL_RawData[b+1]-encL_RawData[b]))+1;
		}
	}
	if(num == 0){
		value -= 4096;
	}
	return value;
}

/*void vQ_push(float e, float a){
	//ココカラ
}

void estimateV(void){
	short i = 0;
	volatile float sum = 0;
	for(i = 0; i < 30; i++){
		sum += q_velo.Enc[i]/30.0f;
	}
	for(i = 0; i < 30; i++){
		sum += q_velo.Accel[i]/30.0f*0.03f/2.0f;
	}
	speedE = sum;
}*/

void resetCurrent(void){
	unsigned char num = 0;
	for(num = 0; num < 5; num++){
		R_Dis_Current[num] = 0;
		L_Dis_Current[num] = 0;
		FR_Dis_Current[num] = 0;
		FL_Dis_Current[num] = 0;
	}
	R_Dis_Ave = 0;
	L_Dis_Ave = 0;
	FR_Dis_Ave = 0;
	FL_Dis_Ave = 0;
	R_Dis_Ave_B = 0;
	L_Dis_Ave_B = 0;
}

void intrpt1(void){
	static int state = 0;	//読み込むセンサのローテーション管理用変数
	int i = 0;
	static unsigned short sensor_r_high = 0;
	static unsigned short sensor_r_low = 0;
	static unsigned short sensor_l_high = 0;
	static unsigned short sensor_l_low = 0;
	static unsigned short sensor_fr_high = 0;
	static unsigned short sensor_fr_low = 0;
	static unsigned short sensor_fl_high = 0;
	static unsigned short sensor_fl_low = 0;
	static unsigned int bat = 0;

	if(g_Sensor_Flag == 1){
	switch(state){

		case 0:		//右センサ読み込み
			HAL_GPIO_WritePin( LED_SR_GPIO_Port, LED_SR_Pin, GPIO_PIN_RESET );
			sensor_r_low = adc_value[2];
			HAL_GPIO_WritePin( LED_SR_GPIO_Port, LED_SR_Pin, GPIO_PIN_SET );
			break;

		case 1:
			break;

		case 2:
			sensor_r_high = adc_value[2];
			HAL_GPIO_WritePin( LED_SR_GPIO_Port, LED_SR_Pin, GPIO_PIN_RESET );			//LED消灯

			Sensor_R = (float)(sensor_r_high - sensor_r_low);	//値を保存
			Before_R_Dis = Sensor_R_Dis;
			if(Sensor_R >= 1){
				Sensor_R_Dis = (float)((CONST_A_R / log(Sensor_R)) - CONST_B_R);
			}
			else{
				Sensor_R_Dis = FAR;
			}
			if(Sensor_R_Dis > FAR){
				Sensor_R_Dis = FAR;
			}

			R_Dis_Ave_B = R_Dis_Ave;
			R_Dis_Current[4] = R_Dis_Current[3];
			R_Dis_Current[3] = R_Dis_Current[2];
			R_Dis_Current[2] = R_Dis_Current[1];
			R_Dis_Current[1] = R_Dis_Current[0];
			R_Dis_Current[0] = Sensor_R_Dis;
			R_Dis_Ave = (float)((R_Dis_Current[0] + R_Dis_Current[1] + R_Dis_Current[2] + R_Dis_Current[3] + R_Dis_Current[4])/5.0f);

			R_Dis_Current_A[4] = R_Dis_Current_A[3];
			R_Dis_Current_A[3] = R_Dis_Current_A[2];
			R_Dis_Current_A[2] = R_Dis_Current_A[1];
			R_Dis_Current_A[1] = R_Dis_Current_A[0];
			R_Dis_Current_A[0] = R_Dis_Ave;
			R_Diff_Smoothed = (float)(R_Dis_Current_A[0]-R_Dis_Current_A[3])*3.0f+(float)(R_Dis_Current_A[1]-R_Dis_Current_A[2])*1.0f;

			/*if(slantingtask != 0){
			if(nowpeekr == 0){	//斜め制御用　柱ピーク値
				nowpeekr = R_Dis_Ave;
			}
			if(R_Dis_Ave < nowpeekr){
				nowpeekr = R_Dis_Ave;
			}
			}*/

			if(Before_R_Dis < WALL_TH_COMB_R_B){
				pollingR++;
			}
			else{
				pollingR = 0;
			}

			if(/*Sensor_R_Dis*/R_Dis_Ave < WALL_TH_R_WALL)			//壁の有無を判断
			{
				Wall_R = WALL_ON;			//右壁あり
				//rsensor_sudden_plus = 0;
				walloncnt_r++;
			}
			else
			{
				Wall_R = WALL_OFF;			//右壁なし
				//rsensor_sudden_minus = 0;
				walloncnt_r = 0;
				//LED_B1 = 0;
			}

			if(/*Sensor_L_Dis*/R_Dis_Ave < WALL_TH_R2 && fabsf(R_Dis_Ave - R_Dis_Ave_B) <= SUDDENCHANGE)		//壁の有無を判断
			{
				rsensor_sudden_plus = 0;
			}
			else
			{
				rsensor_sudden_minus = 0;
			}

			if((R_Dis_Ave - R_Dis_Ave_B) > SUDDENCHANGE){
				if(searching_flag == 1 && (run_mode == STRAIGHT_MODE || run_mode == STRAIGHT2_MODE)){
					rsensor_sudden_minus = 1;
				}
				else{
					rsensor_sudden_minus = 0;
				}
				//LED_B1 = 1;
				if(/*Sensor_R_Dis*/R_Dis_Ave < (WALL_CTRL_TH_R-SUDDENCHANGECORRECTR))		//制御をかけるか否かを判断
				{
					Sensor_R_Err = (float)Sensor_R_Ref - R_Dis_Ave;//Sensor_R_Dis;//Sensor_R - Sensor_R_Ref;	//制御をかける場合は偏差を計算
					R_Raw_Err = (float)(R_REF_RAW - Sensor_R);
					Sensor_R_Ctrl_Flag = FLAG_ON;		//右センサを制御に使う
				}
				else
				{
					Sensor_R_Err = 0;			//制御に使わない場合は偏差を0にしておく
					R_Raw_Err = 0;
					Sensor_R_Ctrl_Flag = FLAG_OFF;		//右センサを制御に使わない
				}
				//LED_B3 = 1;
			}
			else if((R_Dis_Ave_B - R_Dis_Ave) > SUDDENCHANGE){
				if(searching_flag == 1 && (run_mode == STRAIGHT_MODE || run_mode == STRAIGHT2_MODE)){
					rsensor_sudden_plus = 1;
				}
				else{
					rsensor_sudden_plus = 0;
				}
				//LED_B1 = 1;
				if(/*Sensor_R_Dis*/R_Dis_Ave < (WALL_CTRL_TH_R-SUDDENCHANGECORRECTR))		//制御をかけるか否かを判断
				{
					Sensor_R_Err = (float)Sensor_R_Ref - R_Dis_Ave;//Sensor_R_Dis;//Sensor_R - Sensor_R_Ref;	//制御をかける場合は偏差を計算
					R_Raw_Err = (float)(R_REF_RAW - Sensor_R);
					Sensor_R_Ctrl_Flag = FLAG_ON;		//右センサを制御に使う
				}
				else
				{
					Sensor_R_Err = 0;			//制御に使わない場合は偏差を0にしておく
					R_Raw_Err = 0;
					Sensor_R_Ctrl_Flag = FLAG_OFF;		//右センサを制御に使わない
				}
				//LED_B3 = 1;
			}
			else{
				if(searching_flag == 1 && StopFlagR == 1){
					if(/*Sensor_R_Dis*/R_Dis_Ave < WALL_STOP_CTRL_TH_R)		//制御をかけるか否かを判断
					{
						Sensor_R_Err = (float)Sensor_R_Ref - R_Dis_Ave;//Sensor_R_Dis;//Sensor_R - Sensor_R_Ref;	//制御をかける場合は偏差を計算
						R_Raw_Err = (float)(R_REF_RAW - Sensor_R);
						Sensor_R_Ctrl_Flag = FLAG_ON;		//右センサを制御に使う
						StopFlagR = 0;
					}
					else
					{
						Sensor_R_Err = 0;			//制御に使わない場合は偏差を0にしておく
						R_Raw_Err = 0;
						Sensor_R_Ctrl_Flag = FLAG_OFF;		//右センサを制御に使わない
					}
				}
				else{
					if(/*Sensor_R_Dis*/R_Dis_Ave < WALL_CTRL_TH_R)		//制御をかけるか否かを判断
					{
						Sensor_R_Err = (float)Sensor_R_Ref - R_Dis_Ave;//Sensor_R_Dis;//Sensor_R - Sensor_R_Ref;	//制御をかける場合は偏差を計算
						R_Raw_Err = (float)(R_REF_RAW - Sensor_R);
						Sensor_R_Ctrl_Flag = FLAG_ON;		//右センサを制御に使う
					}
					else
					{
						Sensor_R_Err = 0;			//制御に使わない場合は偏差を0にしておく
						R_Raw_Err = 0;
						Sensor_R_Ctrl_Flag = FLAG_OFF;		//右センサを制御に使わない
					}
				}
				//LED_B3 = 0;
			}
			break;

		case 3:
			break;
		case 4:
			break;

		case 5:		//左センサ読み込み
			HAL_GPIO_WritePin( LED_SL_GPIO_Port, LED_SL_Pin, GPIO_PIN_RESET );
			sensor_l_low = adc_value[3];
			HAL_GPIO_WritePin( LED_SL_GPIO_Port, LED_SL_Pin, GPIO_PIN_SET );
			break;

		case 6:
			break;

		case 7:
			sensor_l_high = adc_value[3];
			HAL_GPIO_WritePin( LED_SL_GPIO_Port, LED_SL_Pin, GPIO_PIN_RESET );			//LED消灯

			Sensor_L = (float)(sensor_l_high - sensor_l_low);	//値を保存
			Before_L_Dis = Sensor_L_Dis;
			if(Sensor_L >= 1){
				Sensor_L_Dis = (float)((CONST_A_L / log(Sensor_L)) - CONST_B_L);
			}
			else{
				Sensor_L_Dis = FAR;
			}
			if(Sensor_L_Dis > FAR){
				Sensor_L_Dis = FAR;
			}

			L_Dis_Ave_B = L_Dis_Ave;
			L_Dis_Current[4] = L_Dis_Current[3];
			L_Dis_Current[3] = L_Dis_Current[2];
			L_Dis_Current[2] = L_Dis_Current[1];
			L_Dis_Current[1] = L_Dis_Current[0];
			L_Dis_Current[0] = Sensor_L_Dis;
			L_Dis_Ave = (float)((L_Dis_Current[0] + L_Dis_Current[1] + L_Dis_Current[2] + L_Dis_Current[3] + L_Dis_Current[4])/5.0f);

			L_Dis_Current_A[4] = L_Dis_Current_A[3];
			L_Dis_Current_A[3] = L_Dis_Current_A[2];
			L_Dis_Current_A[2] = L_Dis_Current_A[1];
			L_Dis_Current_A[1] = L_Dis_Current_A[0];
			L_Dis_Current_A[0] = L_Dis_Ave;
			L_Diff_Smoothed = (float)(L_Dis_Current_A[0]-L_Dis_Current_A[3])*3.0f+(float)(L_Dis_Current_A[1]-L_Dis_Current_A[2])*1.0f;

			/*if(slantingtask != 0){
			if(nowpeekl == 0){	//斜め制御用　柱ピーク値
				nowpeekl = L_Dis_Ave;
			}
			if(L_Dis_Ave < nowpeekl){
				nowpeekl = L_Dis_Ave;
			}
			}*/

			if(Before_L_Dis < WALL_TH_COMB_L_B){
				pollingL++;
			}
			else{
				pollingL = 0;
			}

			if(/*Sensor_L_Dis*/L_Dis_Ave < WALL_TH_L_WALL)		//壁の有無を判断
			{
				Wall_L = WALL_ON;			//左前壁あり
			//	lsensor_sudden_plus = 0;
				walloncnt_l++;
			}
			else
			{
				Wall_L = WALL_OFF;			//左前壁なし
			//	lsensor_sudden_minus = 0;
				walloncnt_l = 0;
				//LED_B2 = 0;
			}

			if(/*Sensor_L_Dis*/L_Dis_Ave < WALL_TH_L2 && fabsf(L_Dis_Ave - L_Dis_Ave_B) <= SUDDENCHANGE)		//壁の有無を判断
			{
				lsensor_sudden_plus = 0;
			}
			else
			{
				lsensor_sudden_minus = 0;
			}

			if((L_Dis_Ave - L_Dis_Ave_B) > SUDDENCHANGE){
				if(searching_flag == 1 && (run_mode == STRAIGHT_MODE || run_mode == STRAIGHT2_MODE)){
					lsensor_sudden_minus = 1;
				}
				else{
					lsensor_sudden_minus = 0;
				}
				if(/*Sensor_L_Dis*/L_Dis_Ave < (WALL_CTRL_TH_L-SUDDENCHANGECORRECTL))		//制御をかけるか否かを判断
				{
					Sensor_L_Err = (float)Sensor_L_Ref - L_Dis_Ave;//Sensor_L_Dis;//Sensor_L - Sensor_L_Ref;	//制御をかける場合は偏差を計算
					L_Raw_Err = (float)(L_REF_RAW - Sensor_L);
					Sensor_L_Ctrl_Flag = FLAG_ON;		//右センサを制御に使う
				}
				else
				{
					Sensor_L_Err = 0;			//制御に使わない場合は偏差を0にしておく
					L_Raw_Err = 0;
					Sensor_L_Ctrl_Flag = FLAG_OFF;		//右センサを制御に使わない
				}
			//	LED_B4 = 1;
			}
			else if((L_Dis_Ave_B - L_Dis_Ave) > SUDDENCHANGE){
				if(searching_flag == 1 && (run_mode == STRAIGHT_MODE || run_mode == STRAIGHT2_MODE)){
					lsensor_sudden_plus = 1;
				}
				else{
					lsensor_sudden_plus = 0;
				}
				if(/*Sensor_L_Dis*/L_Dis_Ave < (WALL_CTRL_TH_L-SUDDENCHANGECORRECTL))		//制御をかけるか否かを判断
				{
					Sensor_L_Err = (float)Sensor_L_Ref - L_Dis_Ave;//Sensor_L_Dis;//Sensor_L - Sensor_L_Ref;	//制御をかける場合は偏差を計算
					L_Raw_Err = (float)(L_REF_RAW - Sensor_L);
					Sensor_L_Ctrl_Flag = FLAG_ON;		//右センサを制御に使う
				}
				else
				{
					Sensor_L_Err = 0;			//制御に使わない場合は偏差を0にしておく
					L_Raw_Err = 0;
					Sensor_L_Ctrl_Flag = FLAG_OFF;		//右センサを制御に使わない
				}
			//	LED_B4 = 1;
			}
			else{
				if(searching_flag == 1 && StopFlagL == 1){
					if(/*Sensor_R_Dis*/L_Dis_Ave < WALL_STOP_CTRL_TH_L)		//制御をかけるか否かを判断
					{
						Sensor_L_Err = (float)Sensor_L_Ref - L_Dis_Ave;//Sensor_L_Dis;//Sensor_L - Sensor_L_Ref;	//制御をかける場合は偏差を計算
						L_Raw_Err = (float)(L_REF_RAW - Sensor_L);
						Sensor_L_Ctrl_Flag = FLAG_ON;		//右センサを制御に使う
						StopFlagL = 0;
					}
					else
					{
						Sensor_L_Err = 0;			//制御に使わない場合は偏差を0にしておく
						L_Raw_Err = 0;
						Sensor_L_Ctrl_Flag = FLAG_OFF;		//右センサを制御に使わない
					}
				}
				else{
					if(/*Sensor_L_Dis*/L_Dis_Ave < WALL_CTRL_TH_L)		//制御をかけるか否かを判断
					{
						Sensor_L_Err = (float)Sensor_L_Ref - L_Dis_Ave;//Sensor_L_Dis;//Sensor_L - Sensor_L_Ref;	//制御をかける場合は偏差を計算
						L_Raw_Err = (float)(L_REF_RAW - Sensor_L);
						Sensor_L_Ctrl_Flag = FLAG_ON;		//右センサを制御に使う
					}
					else
					{
						Sensor_L_Err = 0;			//制御に使わない場合は偏差を0にしておく
						L_Raw_Err = 0;
						Sensor_L_Ctrl_Flag = FLAG_OFF;		//右センサを制御に使わない
					}
				}
			//	LED_B4 = 0;
			}
			break;

		case 8:
			break;
		case 9:
			break;

		case 10:		//前左センサ読み込み
			HAL_GPIO_WritePin( LED_FR_GPIO_Port, LED_FR_Pin, GPIO_PIN_RESET );
			sensor_fr_low = adc_value[4];
			HAL_GPIO_WritePin( LED_FR_GPIO_Port, LED_FR_Pin, GPIO_PIN_SET );
			break;

		case 11:
			break;

		case 12:
			sensor_fr_high = adc_value[4];
			HAL_GPIO_WritePin( LED_FR_GPIO_Port, LED_FR_Pin, GPIO_PIN_RESET );			//LED消灯

			Sensor_FR = (float)(sensor_fr_high - sensor_fr_low);	//値を保存
			if(Sensor_FR >= 1){
				Sensor_FR_Dis = (float)((CONST_A_FR / log(Sensor_FR)) - CONST_B_FR);
			}
			else{
				Sensor_FR_Dis = FAR;
			}
			if(Sensor_FR_Dis > FAR){
				Sensor_FR_Dis = FAR;
			}

			FR_Dis_Current[4] = FR_Dis_Current[3];
			FR_Dis_Current[3] = FR_Dis_Current[2];
			FR_Dis_Current[2] = FR_Dis_Current[1];
			FR_Dis_Current[1] = FR_Dis_Current[0];
			FR_Dis_Current[0] = Sensor_FR_Dis;
			FR_Dis_Ave = (float)((FR_Dis_Current[0] + FR_Dis_Current[1] + FR_Dis_Current[2] + FR_Dis_Current[3] + FR_Dis_Current[4])/5.0f);

			if(/*Sensor_FL_Dis*/FR_Dis_Ave < WALL_TH_FR)		//壁の有無を判断
			{
				Wall_FR = WALL_ON;			//右前壁あり
			}
			else
			{
				Wall_FR = WALL_OFF;			//右前壁なし
			}
			break;

		case 13:
			break;
		case 14:
			break;

		case 15:		//前右センサ読み込み
			HAL_GPIO_WritePin( LED_FL_GPIO_Port, LED_FL_Pin, GPIO_PIN_RESET );
			sensor_fl_low = adc_value[1];
			HAL_GPIO_WritePin( LED_FL_GPIO_Port, LED_FL_Pin, GPIO_PIN_SET );
			break;

		case 16:
			break;

		case 17:
			sensor_fl_high = adc_value[1];
			HAL_GPIO_WritePin( LED_FL_GPIO_Port, LED_FL_Pin, GPIO_PIN_RESET );			//LED消灯

			Sensor_FL = (float)(sensor_fl_high - sensor_fl_low);	//値を保存
			if(Sensor_FL >= 1){
				Sensor_FL_Dis = (float)((CONST_A_FL / log(Sensor_FL)) - CONST_B_FL);
			}
			else{
				Sensor_FL_Dis = FAR;
			}
			if(Sensor_FR_Dis > FAR){
				Sensor_FL_Dis = FAR;
			}

			FL_Dis_Current[4] = FL_Dis_Current[3];
			FL_Dis_Current[3] = FL_Dis_Current[2];
			FL_Dis_Current[2] = FL_Dis_Current[1];
			FL_Dis_Current[1] = FL_Dis_Current[0];
			FL_Dis_Current[0] = Sensor_FL_Dis;
			FL_Dis_Ave = (float)((FL_Dis_Current[0] + FL_Dis_Current[1] + FL_Dis_Current[2] + FL_Dis_Current[3] + FL_Dis_Current[4])/5.0f);

			if(/*Sensor_FR_Dis*/FL_Dis_Ave < WALL_TH_FL)			//壁の有無を判断
			{
				Wall_FL = WALL_ON;			//左壁あり
			}
			else
			{
				Wall_FL = WALL_OFF;			//左壁なし
			}
			break;

		case 18:
			break;
		case 19:
			break;
	}

	state++;		//4回ごとに繰り返す
	if(state > 19){
		state = 0;
	}
	}
	else{
		HAL_GPIO_WritePin( LED_FR_GPIO_Port, LED_FR_Pin, GPIO_PIN_RESET );			//LED消灯
		HAL_GPIO_WritePin( LED_FL_GPIO_Port, LED_FL_Pin, GPIO_PIN_RESET );			//LED消灯
		HAL_GPIO_WritePin( LED_SR_GPIO_Port, LED_SR_Pin, GPIO_PIN_RESET );			//LED消灯
		HAL_GPIO_WritePin( LED_SL_GPIO_Port, LED_SL_Pin, GPIO_PIN_RESET );			//LED消灯
	}

	if(log_timer % 20 == 10){
		bat = adc_value[0];
		V_bat_current[0] = V_bat_current[1];
		V_bat_current[1] = V_bat_current[2];
		V_bat_current[2] = V_bat_current[3];
		V_bat_current[3] = V_bat_current[4];
		V_bat_current[4] = bat;
		V_bat_b = V_bat_a;
		V_bat_a = ((V_bat_current[0] + V_bat_current[1] + V_bat_current[2] + V_bat_current[3] + V_bat_current[4])/5);//(float)((bat/4096.0)*3.3*1.5);
		V_bat = (int)((float)V_bat_a*0.1f+(float)V_bat_b*0.9f);

		if(BatCheckSW == 1){
			if(V_bat < 2234){//1489
				batout++;
			}else{
				batout = 0;
			}
			if(batout > 500){
				//モータ止める
				Motor_StopPWM();
				led_all(0);
				//ブザー鳴らし続ける
				while(SWITCH_ONOFF() == SW_OFF){
					for(i = 0; i < 200000; i++){
						LED5_ON();
					}
					for(i = 0; i < 200000; i++){
						LED5_OFF();
					}
					for(i = 0; i < 200000; i++){
						LED5_ON();
					}
					for(i = 0; i < 200000; i++){
						LED5_OFF();
					}
				}
			}
		}else{
			if(bat < 1986){//2110
				batout++;
			}else{
				batout = 0;
			}
			if(batout > 150){
				//モータ止める
				run_mode = TEST_MODE;
				Motor_StopPWM();
				led_all(0);
				HAL_GPIO_WritePin( LED_FR_GPIO_Port, LED_FR_Pin, GPIO_PIN_RESET );			//LED消灯
				HAL_GPIO_WritePin( LED_FL_GPIO_Port, LED_FL_Pin, GPIO_PIN_RESET );			//LED消灯
				HAL_GPIO_WritePin( LED_SR_GPIO_Port, LED_SR_Pin, GPIO_PIN_RESET );			//LED消灯
				HAL_GPIO_WritePin( LED_SL_GPIO_Port, LED_SL_Pin, GPIO_PIN_RESET );			//LED消灯
				//ブザー鳴らし続ける
				while(SWITCH_ONOFF() == SW_OFF){
					for(i = 0; i < 200000; i++){
						LED5_ON();
					}
					for(i = 0; i < 200000; i++){
						LED5_OFF();
					}
					for(i = 0; i < 200000; i++){
						LED5_ON();
					}
					for(i = 0; i < 200000; i++){
						LED5_OFF();
					}
				}
				volatile long n;
				for(n = 0; n < 100*1000*20; n++);
				while(1){
					mode = 0;
					while(1){
						if(SWITCH_ONOFF() == SW_ON){
							LED5_ON();
							for(n = 0; n < 100*1000*20; n++);
							LED5_OFF();
							if(SWITCH_ONOFF() == SW_OFF){
								mode++;
								if(mode == 16){
									mode = 0;
								}
							}
							else if(SWITCH_ONOFF() == SW_ON){
								break;
							}
						}
						else{
						}
						if(mode%2 == 0)		LED1_OFF();
						else	LED1_ON();
						if(mode/2%2 == 0)	LED2_OFF();
						else	LED2_ON();
						if(mode/4%2 == 0)	LED3_OFF();
						else	LED3_ON();
						if(mode/8%2 == 0)	LED4_OFF();
						else	LED4_ON();
					}
					led_all(0);
					LED5_ON();
					mapCtrlZ(mode);
					map_write2();
					LED5_OFF();
					mode = 0;
					break;
				}
			}
		}
	}

	if(log_timer % 20 == 0 && log_flag==1 ){
		if(log_timer < (LOG_CNT*20)){
			nodeinfo[(log_timer/20)/32][(log_timer/20)%32][0].node1 = (short)((FB_r)*1000.0f);
			nodeinfo[(log_timer/20)/32][(log_timer/20)%32][1].node1 = (short)(1000.0f*tar_speed);
			nodeinfo[(log_timer/20)/32][(log_timer/20)%32][2].node1 = (short)(1000.0f*speedE);
			nodeinfo[(log_timer/20)/32][(log_timer/20)%32][3].node1 = (short)(1000.0f*(speedE-speedE_prev));
			nodeinfo[(log_timer/20)/32][(log_timer/20)%32][4].node1 = (short)(locate_raw_r);
			nodeinfo[(log_timer/20)/32][(log_timer/20)%32][5].node1 = (short)(locate_raw_l);
			nodeinfo[(log_timer/20)/32][(log_timer/20)%32][6].node1 = (short)(slip_diff*100.0f);
			nodeinfo[(log_timer/20)/32][(log_timer/20)%32][7].node1 = (short)(accel*1000.0f);
			nodeinfo[(log_timer/20)/32][(log_timer/20)%32][8].node1 = (short)(tar_ang_vel*1000.0f);
			nodeinfo[(log_timer/20)/32][(log_timer/20)%32][9].node1 = (short)(ang_vel*1000.0f);
			nodeinfo[(log_timer/20)/32][(log_timer/20)%32][10].node1 = (short)(ang_acc*10.0f);
			nodeinfo[(log_timer/20)/32][(log_timer/20)%32][11].node1 = (short)(FF_r*1000.0f);
		//	nodeinfo[(log_timer/4)/32][(log_timer/4)%32][12].node1 = (short)(Duty_r*100);
		//	nodeinfo[(log_timer/4)/32][(log_timer/4)%32][13].node1 = (short)(WallCtrlErr*10);
		}
	}else{
	}

	if(log_timer % 20 == 8 && flashLogFlag == 1 && flash6_erased == 1 && saitaning == 1 && run_mode != TEST_MODE){
		nowLog[0] = (short)(tar_ang_vel*100);
		nowLog[1] = (short)(ang_vel*100);
		nowLog[2] = (short)(tar_speed*1000);
		nowLog[3] = (short)(speed*1000);
		nowLog[4] = (short)((((float)V_bat/4096.0f)*3.3f*2.0f)*1000);
		nowLog[5] = (short)(Sensor_L_Dis*10);
		nowLog[6] = (short)(Sensor_FR_Dis*10);
		nowLog[7] = (short)(Sensor_FL_Dis*10);
		nowLog[8] = (short)((FF_r+FB_r)*1000);
		nowLog[9] = (short)((FF_l+FB_l)*1000);
		log_write();
	}

	log_timer++;
}

void intrpt2(void){
	int p = 0;
	//volatile int cncn = 0;
	static unsigned int	enc_data_r;	//エンコーダの生データ
	static unsigned int	enc_data_l;	//エンコーダの生データ
	static short	state2 = 0;
	short a = 0, b = 0, c = 0, d = 0, e = 0 , f = 0;
	float read_imu = 0;
	//short q = 0,ma = 0,mi = 0;

	if(state2 == 0){
		enc_data_r = ENC_R_GetAngle();//MTU1.TCNT;
		state2 = 1;
	}else{
		enc_data_l = ENC_L_GetAngle();//MTU2.TCNT;
		locate_raw_r = enc_data_r;
		locate_raw_l = enc_data_l;
		locate_r = enc_data_r;//getEncTable(RIGHT,enc_data_r);
		locate_l = enc_data_l;//getEncTable(LEFT,enc_data_l);
		if(locate_raw_r == 1){
			locate_flag_r = 1;
		}
		if(locate_raw_l == 1){
			locate_flag_l = 1;
		}
		a = locate_r-before_locate_r;
		b = a+4096;
		c = a-4096;
		d = abs(a);
		e = abs(b);
		f = abs(c);
		if(d <= e){
			if(d <= f){
				diff_pulse_r = a;
			}
			else{
				diff_pulse_r = c;
			}
		}
		else{
			if(e <= f){
				diff_pulse_r = b;
			}
			else{
				diff_pulse_r = c;
			}
		}
		turnnumr += diff_pulse_r;
		//
		a = locate_l-before_locate_l;
		b = a+4096;
		c = a-4096;
		d = abs(a);
		e = abs(b);
		f = abs(c);
		if(d <= e){
			if(d <= f){
				diff_pulse_l = a;
			}
			else{
				diff_pulse_l = c;
			}
		}
		else{
			if(e <= f){
				diff_pulse_l = b;
			}
			else{
				diff_pulse_l = c;
			}
		}
		turnnuml -= diff_pulse_l;

		//現在速度を算出
		speed_new_r = (float)((float)diff_pulse_r * (float)MMPP);
		speed_new_l = -(float)((float)diff_pulse_l * (float)MMPP);

		//過去の値を保存
		speed_old_r= speed_r;
		speed_old_l= speed_l;
		//速度のローパスフィルタ
		speed_r = speed_new_r * 0.1f + speed_old_r * 0.9f;
		speed_l = speed_new_l * 0.1f + speed_old_l * 0.9f;

		//過去の値を保存
		speed_old_r2= speed_r2;
		speed_old_l2= speed_l2;
		//速度のローパスフィルタ
		speed_r2 = speed_new_r * 0.1f + speed_old_r2 * 0.9f;
		speed_l2 = speed_new_l * 0.1f + speed_old_l2 * 0.9f;

		p_speed = speed;
		p_speed_r = speed_r;
		p_speed_l = speed_l;

		speedE_prev = speedE;
		speedE = FUSION_ALPHA*(speedE_prev+accelX_gyro*0.001f) + (1-FUSION_ALPHA)*(speed_new_r+speed_new_l)*0.5f;

		//車体速度を計算
		//if(searching_flag == 1){
		//	speed = ((speed_r + speed_l)/2.0f);
		//	speed2 = ((speed_r2 + speed_l2)/2.0f);
		//}
		//else{
			speed = speedE;//((speed_r + speed_l)/2.0f);
			speed2 = speedE;//((speed_r2 + speed_l2)/2.0f);
		//}
		ang_vel_from_enc = (speed_r-speed_l)/TREAD;

		//I成分のオーバーフローとアンダーフロー対策
		I_speed += speed;
		if(I_speed >30*10000000000){
			I_speed = 30*10000000000;
		}else if(I_speed < -1*10000000000){
			I_speed = -1*10000000000;
		}

		I_speed2 += speed2;
		if(I_speed2 >30*10000000000){
			I_speed2 = 30*10000000000;
		}else if(I_speed2 < -1*10000000000){
			I_speed2 = -1*10000000000;
		}


		if(run_mode != TEST_MODE && run_mode != BACK_MODE && run_mode != VOLT_MODE){
			if(fabsf(speed_r - tar_speed_r) > FAIL_SPEED){
				failcnt1++;
			}
			else{
				failcnt1 = 0;
			}
			if(fabsf(speed_l - tar_speed_l) > FAIL_SPEED){
				failcnt3++;
			}
			else{
				failcnt3 = 0;
			}
			if(fabsf(tar_ang_vel - ang_vel) > FAIL_ANG_VEL){
				failcnt5++;
			}
			else{
				failcnt5 = 0;
			}

			if(failcnt1 > FAILOUTSP || failcnt2 > FAILOUTSP || failcnt3 > FAILOUTSP || failcnt4 > FAILOUTSP || failcnt5 > FAILOUTANG || failcnt6 > FAILOUTANG){
				//モータ止める
				Motor_StopPWM();
				led_all(0);
				HAL_GPIO_WritePin( LED_FR_GPIO_Port, LED_FR_Pin, GPIO_PIN_RESET );			//LED消灯
				HAL_GPIO_WritePin( LED_FL_GPIO_Port, LED_FL_Pin, GPIO_PIN_RESET );			//LED消灯
				HAL_GPIO_WritePin( LED_SR_GPIO_Port, LED_SR_Pin, GPIO_PIN_RESET );			//LED消灯
				HAL_GPIO_WritePin( LED_SL_GPIO_Port, LED_SL_Pin, GPIO_PIN_RESET );			//LED消灯
				//ブザー鳴らし続ける
				while(SWITCH_ONOFF() == SW_OFF){
					for(p = 0; p < 200000; p++){
					LED3_ON();
					//LED4_ON();
					}
					for(p = 0; p < 200000; p++){
						LED3_OFF();
						//LED_B3 = 0;
					}
					for(p = 0; p < 200000; p++){
						LED3_ON();
						//LED_B3 = 1;
					}
					for(p = 0; p < 600000; p++){
						LED3_OFF();
						//LED_B3 = 0;
					}
				}
				volatile long n;
				for(n = 0; n < 100*1000*20; n++);
				while(1){
					mode = 0;
					while(1){
						if(SWITCH_ONOFF() == SW_ON){
							LED5_ON();
							for(n = 0; n < 100*1000*20; n++);
							LED5_OFF();
							if(SWITCH_ONOFF() == SW_OFF){
								mode++;
								if(mode == 16){
									mode = 0;
								}
							}
							else if(SWITCH_ONOFF() == SW_ON){
								break;
							}
						}
						else{
						}
						if(mode%2 == 0)		LED1_OFF();
						else	LED1_ON();
						if(mode/2%2 == 0)	LED2_OFF();
						else	LED2_ON();
						if(mode/4%2 == 0)	LED3_OFF();
						else	LED3_ON();
						if(mode/8%2 == 0)	LED4_OFF();
						else	LED4_ON();
					}
					led_all(0);
					LED5_ON();
					mapCtrlZ(mode);
					map_write2();
					LED5_OFF();
					mode = 0;
					break;
				}
			}

		}
		else{
			failcnt1 = 0;
			failcnt2 = 0;
			failcnt3 = 0;
			failcnt4 = 0;
			failcnt5 = 0;
			failcnt6 = 0;
		}
		//距離の計算
		len_mouse += (speed_new_r + speed_new_l)/2.0f;
		len_true += (speed_new_r + speed_new_l)/2.0f;
		//過去の値を保存
		before_locate_r = locate_r;
		before_locate_l = locate_l;
		state2 = 0;
	}

	if(state2 == 1){
		//ジャイロセンサの値の更新
		read_imu = IMU_GetGyro_Z();
		accelX_gyro = IMU_GetAccel_X();


		p_ang_vel = ang_vel;



		if((run_mode == SLA2_MODE || run_mode == SLA3_MODE) && FanVolt != 0 && saitaning == 1){
			slip_old = slip_new;
			if(TURN_DIR == RIGHT){
				slip_new = (float)((1000.0f*slip_old-read_imu)/(1000.0f+K_SLIP_R*100.0f/speedE));
			}
			else if(TURN_DIR == LEFT){
				slip_new = (float)((1000.0f*slip_old-read_imu)/(1000.0f+K_SLIP_L*100.0f/speedE));
			}
			slip_diff = slip_new*1000.0f;
			/*slip_diff = -K_SLIP*slip_new/speedE - read_imu;
			slip_new += slip_diff*0.001f;*/
			ang_vel = read_imu;//+slip_diff;
		}
		else{
			ang_vel = read_imu;
			slip_diff = 0;
			slip_new = 0;
			slip_old = 0;
		}
	//	speedX_gyro += accelX_gyro*0.001f;


	//	if(fabsf(ang_vel) >= MAX_DPS){
	//		ang_vel = ang_vel_from_enc;
	//	}
		ang_vel_current[0] = ang_vel_current[1];
		ang_vel_current[1] = ang_vel_current[2];
		ang_vel_current[2] = ang_vel_current[3];
		ang_vel_current[3] = ang_vel_current[4];
		//ang_vel_current[4] = ang_vel_current[5];
		//ang_vel_current[5] = ang_vel_current[6];
		//ang_vel_current[6] = ang_vel_current[7];
		//ang_vel_current[7] = ang_vel_current[8];
		//ang_vel_current[8] = ang_vel_current[9];
		ang_vel_current[4] = ang_vel;
		/*ma = 0;
		mi = 0;
		for(q = 0; q < 7; q++){
			if(ang_vel_current[mi] >= ang_vel_current[q]){
				mi = q;
			}
			if(ang_vel_current[ma] <= ang_vel_current[q]){
				ma = q;
			}
		}*/
		ang_vel_ave = ((ang_vel_current[0]+ang_vel_current[1]+ang_vel_current[2]+ang_vel_current[3]+ang_vel_current[4]/*+ang_vel_current[5]+ang_vel_current[6]+ang_vel_current[7]+ang_vel_current[8]+ang_vel_current[9]-ang_vel_current[ma]-ang_vel_current[mi]*/)/5.0f);
		//ang_vel_ave_b = ang_vel_ave;
		//ang_vel = p_ang_vel * 0.9 + new_ang_vel * 0.1;
		//積分値の更新
		I_ang_vel += ang_vel;
		if(I_ang_vel >30*10000000000){
			I_ang_vel = 30*10000000000;
		}else if(I_ang_vel < -1*10000000000){
			I_ang_vel = -1*10000000000;
		}
		//ジャイロの値を角度に変換
		degree += (0.001f*/*read_imu*/ang_vel*180.0f/PI);
		Angle_G += (0.001*read_imu*180.0/3.14159265359);
		Tar_Angle_G += (0.001*tar_ang_vel*180.0/3.14159265359);
		//state = 0;
	}
}


void intrpt3(void){
//	volatile char timesss = 0;

	if(run_mode == STRAIGHT_MODE || run_mode == BACK_MODE || run_mode == OFFSET_A_MODE || run_mode == OFFSET_B_MODE){
		tar_speed += accel/1000.0f;	//目標速度を設定加速度で更新
		//最高速度制限
		if(tar_speed > max_speed){
			tar_speed = max_speed;	//目標速度を設定最高速度に設定
			accel = 0;
		}
		tar_speed_r = tar_speed;
		tar_speed_l = tar_speed;
	//	I_len += (len_target-len_mouse);
	//	if(I_len >30*10000000000){
	//		I_len = 30*10000000000;
	//	}else if(I_len < -1*10000000000){
	//		I_len = 1*10000000000;
	//	}
	}
	else if(run_mode == STRAIGHT2_MODE){
		tsin2++;
		accel = (((PI*accel_sin)/2.0f) * arm_sin_f32(((PI*accel_sin)/max_speed_sin)*(float)tsin2*0.001f));
		if(accflag2 == 1 && accel <= 0){
			accel = (((PI*accel_sin)/2.0f) * arm_sin_f32(((PI*accel_sin)/max_speed_sin)*0.001f));
		}
		if(accflag2 == 2 && accel > 0){
			accel = (((PI*accel_sin)/2.0f) * arm_sin_f32(((PI*accel_sin)/max_speed_sin)*0.001f));
		}
		tar_speed += accel/1000.0f;	//目標速度を設定加速度で更新
		//最高速度制限
		if(tar_speed > max_speed){
			tar_speed = max_speed;	//目標速度を設定最高速度に設定
		}
		tar_speed_r = tar_speed;
		tar_speed_l = tar_speed;
	}
	else if(run_mode == TURN_MODE){

		//角加速度更新
		tar_ang_vel += (float)(ang_acc/1000.0f);	//目標角速度を設定加速度で更新
		tar_degree  += (float)((tar_ang_vel/1000.0f)*180.0f/PI);

		//左回転の場合
		if(TURN_DIR == LEFT){
			//最高角速度制限
			if(tar_ang_vel > max_ang_vel){
				tar_ang_vel = max_ang_vel;	//目標速度を設定最高速度に設定
			}
			if(tar_degree > max_degree){
				tar_degree = max_degree;
			}
			tar_speed_r = (float)(TREAD*PI*(tar_ang_vel*180.0f/PI)/360.0f);
			tar_speed_l = (-1)*(float)(TREAD*PI*(tar_ang_vel*180.0f/PI)/360.0f);
		}else if(TURN_DIR == RIGHT){
		//右回転の場合
			//最高角速度制限
			if(tar_ang_vel < max_ang_vel){
				tar_ang_vel = max_ang_vel;	//目標速度を設定最高速度に設定
			}
			if(tar_degree < max_degree){
				tar_degree = max_degree;
			}
			tar_speed_r = (float)(TREAD*PI*(tar_ang_vel*180.0f/PI)/360.0f);
			tar_speed_l = (-1.0f)*(float)(TREAD*PI*(tar_ang_vel*180.0f/PI)/360.0f);
		}
	}
	else if(run_mode == SLA_MODE){

		//角加速度更新
		tar_ang_vel += (float)(ang_acc/1000.0f);	//目標角速度を設定加速度で更新
		//tar_degree  += (float)((tar_ang_vel/1000.0)*180.0/PI);
		//mouse_x += sin((degree - start_degree)*180.0/PI)*tar_speed;
		//mouse_y += cos((degree - start_degree)*180.0/PI)*tar_speed;


		//左回転の場合
		if(TURN_DIR == LEFT){
			//最高角速度制限
			if(tar_ang_vel > max_ang_vel){
				tar_ang_vel = max_ang_vel;	//目標速度を設定最高速度に設定
			}
			//if(tar_ang_vel < SLA_MIN_SPEED){
			//	tar_ang_vel = SLA_MIN_SPEED;
			//}
			tar_degree  += (float)((tar_ang_vel/1000.0f)*180.0f/PI);
			if(tar_degree > max_degree){
				tar_degree = max_degree;
			}
			tar_speed_r = (float)(tar_speed + ((tar_ang_vel * TREAD)/2.0f));
			tar_speed_l = (float)(tar_speed - ((tar_ang_vel * TREAD)/2.0f));
		}else if(TURN_DIR == RIGHT){
		//右回転の場合
			//最高角速度制限
			if(tar_ang_vel < max_ang_vel){
				tar_ang_vel = max_ang_vel;	//目標速度を設定最高速度に設定
			}
			//if(-tar_ang_vel < SLA_MIN_SPEED){
			//	tar_ang_vel = -SLA_MIN_SPEED;
			//}
			tar_degree  += (float)((tar_ang_vel/1000.0f)*180.0f/PI);
			if(tar_degree < max_degree){
				tar_degree = max_degree;
			}
			tar_speed_r = (float)(tar_speed + ((tar_ang_vel * TREAD)/2.0f));
			tar_speed_l = (float)(tar_speed - ((tar_ang_vel * TREAD)/2.0f));
		}
		tar_ichi_x -= tar_speed * arm_sin_f32((tar_degree - start_degree)*PI/180.0f);
		tar_ichi_y += tar_speed * arm_cos_f32(fabsf(tar_degree - start_degree)*PI/180.0f);
		ichi_x -= speed * arm_sin_f32((degree - start_degree)*PI/180.0f);
		ichi_y += speed * arm_cos_f32(fabsf(degree - start_degree)*PI/180.0f);

		nowG = tar_ang_vel*tar_speed/9.8f;
	}
	else if(run_mode == SLA2_MODE){
	//	if(ang_acc_sin != 0){
		tsin++;
		if(max_ang_vel_sin != 0){
		ang_acc = (((PI*ang_acc_sin)/2.0f) * arm_sin_f32(((PI*ang_acc_sin)/max_ang_vel_sin)*(float)tsin*0.001f));


		if(TURN_DIR == RIGHT){
			if(accflag == 1 && ang_acc >= 0){
				ang_acc = (((PI*ang_acc_sin)/2.0f) * arm_sin_f32(((PI*ang_acc_sin)/max_ang_vel_sin)*0.001f));
			}
			if(accflag == 2 && ang_acc <= 0){
				ang_acc = (((PI*ang_acc_sin)/2.0f) * arm_sin_f32(((PI*ang_acc_sin)/max_ang_vel_sin)*0.001f));
				accflag = 0;
			}
		}
		if(TURN_DIR == LEFT){
			if(accflag == 1 && ang_acc <= 0){
				ang_acc = (((PI*ang_acc_sin)/2.0f) * arm_sin_f32(((PI*ang_acc_sin)/max_ang_vel_sin)*0.001f));
			}
			if(accflag == 2 && ang_acc >= 0){
				ang_acc = (((PI*ang_acc_sin)/2.0f) * arm_sin_f32(((PI*ang_acc_sin)/max_ang_vel_sin)*0.001f));
				accflag = 0;
			}
		}
		}
	//	}
	//	else{
	//	ang_acc = 0;
	//	}
		//角加速度更新
		tar_ang_vel += (float)(ang_acc/1000.0f);	//目標角速度を設定加速度で更新
		//tar_degree  += (float)((tar_ang_vel/1000.0)*180.0/PI);
		//mouse_x += sin((degree - start_degree)*180.0/PI)*tar_speed;
		//mouse_y += cos((degree - start_degree)*180.0/PI)*tar_speed;


		//左回転の場合
		if(TURN_DIR == LEFT){
			//最高角速度制限
			if(tar_ang_vel > max_ang_vel){
				tar_ang_vel = max_ang_vel;	//目標速度を設定最高速度に設定
			}
			//if(tar_ang_vel < SLA_MIN_SPEED){
			//	tar_ang_vel = SLA_MIN_SPEED;
			//}
			tar_degree  += (float)((tar_ang_vel/1000.0f)*180.0f/PI);
			if(tar_degree > max_degree){
				tar_degree = max_degree;
			}
			tar_speed_r = (float)(tar_speed + ((tar_ang_vel * TREAD)/2.0f));
			tar_speed_l = (float)(tar_speed - ((tar_ang_vel * TREAD)/2.0f));
		}else if(TURN_DIR == RIGHT){
		//右回転の場合
			//最高角速度制限
			if(tar_ang_vel < max_ang_vel){
				tar_ang_vel = max_ang_vel;	//目標速度を設定最高速度に設定
			}
			//if(-tar_ang_vel < SLA_MIN_SPEED){
			//	tar_ang_vel = -SLA_MIN_SPEED;
			//}
			tar_degree  += (float)((tar_ang_vel/1000.0f)*180.0f/PI);
			if(tar_degree < max_degree){
				tar_degree = max_degree;
			}
			tar_speed_r = (float)(tar_speed + ((tar_ang_vel * TREAD)/2.0f));
			tar_speed_l = (float)(tar_speed - ((tar_ang_vel * TREAD)/2.0f));
		}
		tar_ichi_x -= tar_speed * arm_sin_f32((tar_degree - start_degree)*PI/180.0f);
		tar_ichi_y += tar_speed * arm_cos_f32(fabsf(tar_degree - start_degree)*PI/180.0f);
		ichi_x -= speed * arm_sin_f32((degree - start_degree)*PI/180.0f);
		ichi_y += speed * arm_cos_f32(fabsf(degree - start_degree)*PI/180.0f);

		nowG = fabsf(tar_ang_vel*tar_speed/9.8f);
	}
	else if(run_mode == SLA3_MODE){

		//角加速度更新
		Nap_b = tar_ang_vel;
		if(Nap_flag == 1){
			Nap_x -= Nap_h;
			if(Nap_x < 0){
				Nap_x = 0;
			}
			tar_ang_vel = (float)(Nap_a * exp(1.0f/(pow(Nap_x,Nap_n)-1.0f)))*PI/180.0;
			if(tar_ang_vel > 100 || tar_ang_vel < -100){
				tar_ang_vel = 0;
			}

		}
		else if(Nap_flag == 2){
			Nap_x += Nap_h;
			if(Nap_x >= 1){
				Nap_x -= Nap_h;
				Nap_flag = 0;
			}
			tar_ang_vel = (float)(Nap_a * exp(1.0f/(pow(Nap_x,Nap_n)-1.0f)))*PI/180.0f;
			if(tar_ang_vel > 100 || tar_ang_vel < -100){
				tar_ang_vel = 0;
			}
		}
		else{

		}

		//左回転の場合
		if(TURN_DIR == LEFT){
			ang_acc = 1000.0f*(tar_ang_vel-Nap_b);
			tar_degree  += (float)((tar_ang_vel/1000.0f)*180.0f/PI);
			if(tar_degree > max_degree){
				tar_degree = max_degree;
			}
			tar_speed_r = (float)(tar_speed + ((tar_ang_vel * TREAD)/2.0f));
			tar_speed_l = (float)(tar_speed - ((tar_ang_vel * TREAD)/2.0f));
		}else if(TURN_DIR == RIGHT){
			ang_acc = 1000.0f*(tar_ang_vel-Nap_b);
			tar_degree  += (float)((tar_ang_vel/1000.0f)*180.0f/PI);
			if(tar_degree < max_degree){
				tar_degree = max_degree;
			}
			tar_speed_r = (float)(tar_speed + ((tar_ang_vel * TREAD)/2.0f));
			tar_speed_l = (float)(tar_speed - ((tar_ang_vel * TREAD)/2.0f));
		}
		tar_ichi_x -= tar_speed * arm_sin_f32((tar_degree - start_degree)*PI/180.0f);
		tar_ichi_y += tar_speed * arm_cos_f32(fabsf(tar_degree - start_degree)*PI/180.0f);
		ichi_x -= speed * arm_sin_f32((degree - start_degree)*PI/180.0f);
		ichi_y += speed * arm_cos_f32(fabsf(degree - start_degree)*PI/180.0f);
		nowG = tar_ang_vel*tar_speed/9.8f;
	}
	else if(run_mode == PARTY_MODE){
		tar_speed = 0;
		tar_speed_r = 0;
		tar_speed_l = 0;
		tar_ang_vel = 0;
		tar_degree = 0;
	}
	else if(run_mode == TRACELOG_MODE){
		tracecnt++;
	}
	else if(run_mode == NON_CON_MODE){
	}else{
	}
	//壁制御
	if(run_mode == STRAIGHT_MODE || run_mode == STRAIGHT2_MODE){

		if(saitaning == 1 && slantingtask == 0 && R_Ctrl_Disable == 1 && len_true >= Dis_to_Able){
			R_Ctrl_Disable = 0;
		}
		if(saitaning == 1 && slantingtask == 0 && L_Ctrl_Disable == 1 && len_true >= Dis_to_Able2){
			L_Ctrl_Disable = 0;
		}

		if(combbb == 0 && FR_Dis_Ave + FL_Dis_Ave > 250.0f && slantingtask == 0 && ((saitaning == 1 && comb_onoff == 1) || (searching_flag == 1 && comb == 3))){
			if(saitaning == 1){

				if(tar_speed <= 2.0f){
				if(Sensor_R_Dis > COMB_CTRL_TH_R_B && Before_R_Dis < COMB_CTRL_TH_R_B){
				//if(Sensor_R_Dis < COMB_CTRL_TH_R_B && (Sensor_R_Dis-Before_R_Dis) > SUDDENCHANGEB*tar_speed/0.3f){
				//	if(beecombr != 0){
				//		prev_combr = beecombr;
				//	}
					beecombr = len_true+OFFSET_COMB;
				}

				//if(Sensor_L_Dis < COMB_CTRL_TH_L_B && (Sensor_L_Dis-Before_L_Dis) > SUDDENCHANGEB*tar_speed/0.3f){
				if(Sensor_L_Dis > COMB_CTRL_TH_L_B && Before_L_Dis < COMB_CTRL_TH_L_B){
					//	if(beecombl != 0){
				//		prev_combl = beecombl;
				//	}
					beecombl = len_true;
				}
				}
				else{
					if(Sensor_R_Dis > COMB_CTRL_TH_R_B && Before_R_Dis < COMB_CTRL_TH_R_B){
				//if(Sensor_R_Dis < COMB_CTRL_TH_R_B && (Sensor_R_Dis-Before_R_Dis) > SUDDENCHANGEB2*tar_speed/0.3f){
				//	if(beecombr != 0){
				//		prev_combr = beecombr;
				//	}
					beecombr = len_true+OFFSET_COMB;
				}
				if(Sensor_L_Dis > COMB_CTRL_TH_L_B && Before_L_Dis < COMB_CTRL_TH_L_B){
				//if(Sensor_L_Dis < COMB_CTRL_TH_L_B && (Sensor_L_Dis-Before_L_Dis) > SUDDENCHANGEB2*tar_speed/0.3f){
				//	if(beecombl != 0){
				//		prev_combl = beecombl;
				//	}
					beecombl = len_true;
				}
				}
			}
			else{
				if(Sensor_R_Dis > COMB_CTRL_TH_R && Before_R_Dis < COMB_CTRL_TH_R){
				//if(Sensor_R_Dis < COMB_CTRL_TH_R && (Sensor_R_Dis-Before_R_Dis) > SUDDENCHANGEB*tar_speed/0.3f){
				//	if(beecombr != 0){
				//		prev_combr = beecombr;
				//	}
					beecombr = len_true+OFFSET_COMB;
				}
				if(Sensor_L_Dis > COMB_CTRL_TH_L && Before_L_Dis < COMB_CTRL_TH_L){
				//if(Sensor_L_Dis < COMB_CTRL_TH_L && (Sensor_L_Dis-Before_L_Dis) > SUDDENCHANGEB*tar_speed/0.3f){
				//	if(beecombr != 0){
				//		prev_combl = beecombl;
				//	}
					beecombl = len_true;
				}
			}
		}
		//斜め壁切れで角度補正
		if(combbb_s == 0 && slantingtask != 0 && saitaning == 1 /*&& WallCtrlEnbl == 1 */&& (len_target-len_mouse) > ONESLANTING){
			//if(saitaning == 1){
				//if(Sensor_R_Dis > SLANT_CTRL_TH_COMB_R && Before_R_Dis < SLANT_CTRL_TH_COMB_R){
				if(/*Sensor_R_Dis < SLANT_CTRL_TH_COMB_R && (Sensor_R_Dis-Before_R_Dis) > SUDDENCHANGEB_S*tar_speed/0.3f*/Sensor_R_Dis > SLANT_CTRL_TH_COMB_R && Before_R_Dis < SLANT_CTRL_TH_COMB_R && (beecombr_s == 0 || (beecombr_s != 0 && len_true-beecombr_s >= ONESLANTING*2.0f))){
					beecombr_s = len_true+OFFSET_COMB_S;
				}
				//if(Sensor_L_Dis > COMB_CTRL_TH_L_B && Before_L_Dis < COMB_CTRL_TH_L_B){
				if(/*Sensor_L_Dis < SLANT_CTRL_TH_COMB_L && (Sensor_L_Dis-Before_L_Dis) > SUDDENCHANGEB_S*tar_speed/0.3f*/Sensor_L_Dis > SLANT_CTRL_TH_COMB_L && Before_L_Dis < SLANT_CTRL_TH_COMB_L && (beecombl_s == 0 || (beecombl_s != 0 && len_true-beecombl_s >= ONESLANTING*2.0f))){
					beecombl_s = len_true;
				}
			//}
		}
		//
		if(/*Wall_FR == WALL_ON && Wall_FL == WALL_ON*/Sensor_FR_Dis < 90 && Sensor_FL_Dis < 90 && turnfwallflag == 1/* && firstread == 0*/){
			if(fabsf(Sensor_FR_Dis - TURN_FWALL_R) < 0.2f && fabsf(Sensor_FL_Dis - TURN_FWALL_L) < 0.2f){
				turnfwallflag = 0;
			}
			else{
				len_target = len_mouse + (((Sensor_FR_Dis - TURN_FWALL_R)+(Sensor_FL_Dis - TURN_FWALL_L))/2.0f);
				//turnfwallflag = 0;
			}
		}
		//
		if(Sensor_FR_Dis < 180.0f && Sensor_FL_Dis < 180.0f && slantfwallflag == 1/* && firstread == 0*/){
			if((fabsf(Sensor_FR_Dis - SLANT_FWALL_R) < 0.2f && fabsf(Sensor_FL_Dis - SLANT_FWALL_L) < 0.2f) || (fabsf(Sensor_FR_Dis - SLANT_FWALL_L) < 0.2f && fabsf(Sensor_FL_Dis - SLANT_FWALL_R) < 0.2f)){
				slantfwallflag = 0;
			}
			else{
				len_target = len_mouse + (((Sensor_FR_Dis - SLANT_FWALL_R)+(Sensor_FL_Dis - SLANT_FWALL_L))/2.0f);
				//turnfwallflag = 0;
			}
		}
		//
		if(searching_flag == 1){
			if(offsetcnt_st > 100){
			}
			else{
				if(Wall_R == WALL_ON && Wall_L == WALL_ON && FR_Dis_Ave + FL_Dis_Ave > FWALL_CLOSE_SLA){
					if((Sensor_R_Dis - Before_R_Dis) > SUDDENCHANGE){
						offsetplus_st -= (Sensor_L_Dis - Sensor_Sla_L_Ref_St);
					}
					else if((Sensor_L_Dis - Before_L_Dis) > SUDDENCHANGE){
						offsetplus_st += (Sensor_R_Dis - Sensor_Sla_R_Ref_St);
					}
					else{
						offsetplus_st += (((Sensor_R_Dis - Sensor_Sla_R_Ref_St)-(Sensor_L_Dis - Sensor_Sla_L_Ref_St))/2.0f);
					}
					offsetcnt_st++;
				}
				else if(Wall_R == WALL_ON && Wall_L == WALL_OFF && FR_Dis_Ave + FL_Dis_Ave > FWALL_CLOSE_SLA){
					if((Sensor_R_Dis - Before_R_Dis) > SUDDENCHANGE){
					}
					else{
						offsetplus_st += (Sensor_R_Dis - Sensor_Sla_R_Ref_St);
						offsetcnt_st++;
					}
				}
				else if(Wall_R == WALL_OFF && Wall_L == WALL_ON && FR_Dis_Ave + FL_Dis_Ave > FWALL_CLOSE_SLA){
					if((Sensor_L_Dis - Before_L_Dis) > SUDDENCHANGE){
					}
					else{
						offsetplus_st -= (Sensor_L_Dis - Sensor_Sla_L_Ref_St);
						offsetcnt_st++;
					}
				}
				else{
					offsetplus_st = 0;
					offsetcnt_st = 0;
				}
			}
		}
		else{
			offsetplus_st = 0;
			offsetcnt_st = 0;
		}
		//
		if((comb == 0 || comb == 2) && !(saitaning == 1 && emode == 1 && slantingtask == 0 && rl != 0/* == RIGHT*/ && ((WallCtrlEnbl == 1 && len_target-len_mouse <= 45.0f) || (WallCtrlEnbl != 1)))){
			if(slantingtask == 0){
				if((saitaning == 0 && Sensor_R_Dis > WALL_TH_R && Before_R_Dis < WALL_TH_R)
						|| (saitaning == 1 &&/* Sensor_R_Dis > WALL_TH_R_SAI && */Before_R_Dis < WALL_TH_R_SAI &&  R_Diff_Smoothed > CUTDIFF1)){
					if(firstread == 0 && Before_R_Dis != 0 && Before_L_Dis != 0 && wallCutTask != 0 && slantingtask == 0){
						//LED1_ON();
						if(wallCutTask == 1){
							len_mouse = len_target - SEARCHWALLCUTA_R;
							firstread = 1;
							LED1_ON();
						}
						else if(wallCutTask == 2){
							len_mouse = len_target - SEARCHWALLCUTB_R;
							firstread = 1;
							LED1_ON();
						}
						else if(wallCutTask == 3){
							lentmp = len_mouse;
							blockcnt = 0;
							while(lentmp > 180.0f){
								lentmp -= 180.0f;
								blockcnt++;
							}
							if(lentmp >= 90.0f){
								len_mouse = 90.0f*(2*blockcnt+2) - SEARCHWALLCUTA_R;
							}
							else{
								len_mouse = 90.0f*(2*blockcnt+1) - SEARCHWALLCUTA_R;
							}
							LED1_ON();
							//firstread = 1;
						}
						else if(wallCutTask == 4){
							lentmp = (len_mouse-45.0f);
							blockcnt = 0;
							while(lentmp > 180.0f){
								lentmp -= 180.0f;
								blockcnt++;
							}
							if(lentmp < 0){
								//len_mouse = (float)(45.0 - SEARCHWALLCUTA_R);
								//LED_B1 = 1;
							}
							else{
								if(lentmp >= 90.0f){
									len_mouse = 90.0f*(2*blockcnt+2) - SEARCHWALLCUTA_R+45.0f;
								}
								else{
									len_mouse = 90.0f*(2*blockcnt+1) - SEARCHWALLCUTA_R+45.0f;
								}
							}
							LED1_ON();
							//firstread = 1;
						}
						else if(wallCutTask == 5 && (Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B/*7/6 壁切れ無限回読んでたので*/){
							lentmp = (len_mouse-45.0f);
							blockcnt = 0;
							if((len_target - len_mouse) > 90.0f){
								while(lentmp > 180.0f){
									lentmp -= 180.0f;
									blockcnt++;
								}
								if(lentmp < 0){
									//len_mouse = (float)(45.0 - SEARCHWALLCUTA_R);
									//LED_B1 = 1;
								}
								else{
									if(lentmp >= 90.0f){
										len_mouse = 90.0f*(2*blockcnt+2) - SAITANWALLCUTA_R+45.0f;
										R_Ctrl_Disable = 1;
										Dis_to_Able = len_true + AFTERCUT;
									}
									else{
										len_mouse = 90.0f*(2*blockcnt+1) - SAITANWALLCUTA_R+45.0f;
										R_Ctrl_Disable = 1;
										Dis_to_Able = len_true + AFTERCUT;
									}
								}
								LED1_ON();
							//firstread = 1;
							}
							else if((len_target - len_mouse) < 45.0f){
								if(next == 0){
									len_mouse = len_target-SAITANWALLCUTB_R;
									R_Ctrl_Disable = 1;
									Dis_to_Able = len_true + AFTERCUT;
								}
								else{
									len_mouse = len_target-SAITANWALLCUTB_R-next+TURN_SHORT;
									R_Ctrl_Disable = 1;
									Dis_to_Able = len_true + AFTERCUT;
								}
								turn_short = 1;
								turn_read = 1;
								LED1_ON();
							}
							else{
							}
						}
						else{
							LED1_OFF();
						}
					}
					else{
						LED1_OFF();
					}
				}
				else{
					LED1_OFF();
				}
			}
			else if(slantingtask == 1 || slantingtask == 2){
			if((longst == 1 && Sensor_R_Dis > WALL_TH_SLANT_R && Before_R_Dis < WALL_TH_SLANT_R) || (longst == 0 && Sensor_R_Dis > WALL_TH_SLANT_R_O && Before_R_Dis < WALL_TH_SLANT_R_O)/* || (longst != 1 && R_Dis_Ave <=  SLANT_CTRL_TH_COMB_R && (Sensor_R_Dis-Before_R_Dis) >= SUDDENCHANGEB_S)*/){

				if(/*firstread == 0 && */Before_R_Dis != 0 /*&& Before_L_Dis != 0 */&& wallCutTask != 0 && slantingtask == 1 && (WallCtrlEnbl == 1 || WallCtrlEnbl == 3) && !(lastslant == 1 && len_target-len_mouse <= ONESLANTING*2.0f)){
					if(wallCutTask == 5){
						lentmp = len_mouse+SLANTWALLCUT_R+ONESLANTING;
						blockcnt = 0;
					//	if((len_target - lentmp) > 0){
						if(WallCtrlEnbl == 1 && slantblock == 1 && (len_target-len_mouse) <= ONESLANTING && slantread == 0){
							len_mouse = len_target-SLANTWALLCUT_R;
							peekr = nowpeekr;
							nowpeekr = 0;
							slantread = 1;
							LED1_ON();
						}
						else{
							while(lentmp > ONESLANTING*2.0f){
								lentmp -= ONESLANTING*2.0f;
								blockcnt++;
							}
							if(lentmp >= ONESLANTING){
								len_mouse = ONESLANTING*(blockcnt*2+1) - SLANTWALLCUT_R;
								peekr = nowpeekr;
								nowpeekr = 0;
								LED1_ON();
							}
							else{
								if(blockcnt != 0){
									len_mouse = ONESLANTING*(blockcnt*2-1) - SLANTWALLCUT_R;
									peekr = nowpeekr;
									nowpeekr = 0;
									LED1_ON();
								}
								else{
									LED1_OFF();
								}
							}
						}

						//firstread = 1;
					//	}
					//	else{
					//	}
					}
					else{
						LED1_OFF();
					}
				}
				else if(/*firstread == 0 &&*/ Before_R_Dis != 0/* && Before_L_Dis != 0 */&& wallCutTask != 0 && slantingtask == 2 && (WallCtrlEnbl == 1 || WallCtrlEnbl == 3 || WallCtrlEnbl == 6) && !(lastslant == 1 && len_target-len_mouse <= ONESLANTING*2.0f)){
					if(wallCutTask == 5){
						lentmp = len_mouse+SLANTWALLCUT_R;
						blockcnt = 0;
					//	if((len_target - lentmp) > ONESLANTING){
						if(WallCtrlEnbl == 1 && slantblock == 0 && (len_target-len_mouse) <= ONESLANTING && slantread == 0){
							len_mouse = len_target-SLANTWALLCUT_R;
							peekr = nowpeekr;
							nowpeekr = 0;
							slantread = 1;
							LED1_ON();
						}
						else{
							while(lentmp > ONESLANTING*2.0f){
								lentmp -= ONESLANTING*2.0f;
								blockcnt++;
							}
							if(lentmp >= ONESLANTING){
								len_mouse = ONESLANTING*(blockcnt*2+2) - SLANTWALLCUT_R;
								peekr = nowpeekr;
								nowpeekr = 0;
								LED1_ON();
							}
							else{
								if(blockcnt != 0){
									len_mouse = ONESLANTING*(blockcnt*2) - SLANTWALLCUT_R;
									peekr = nowpeekr;
									nowpeekr = 0;
									LED1_ON();
								}
								else{
									if(WallCtrlEnbl == 6){
										len_mouse = len_target-SLANTWALLCUT_R;
										peekr = nowpeekr;
										nowpeekr = 0;
										slantread = 1;
										naname_wallcut = 1;
										LED1_ON();
									}
								//	else{
								//		LED1_OFF();
								//	}
									if(WallCtrlEnbl == 3 && next2 != 0 && slantread == 0){
										len_mouse = len_target-SLANTWALLCUT_R-next2;
										peekr = nowpeekr;
										nowpeekr = 0;
										slantread = 1;
										naname_wallcut = 1;
										LED1_ON();
									}
								//	else{
								//		LED1_OFF();
								//	}
								}
							}
						}

						//firstread = 1;
					//	}
					//	else{
					//	}
					}
					else{
						LED1_OFF();
					}
				}
				else{
					LED1_OFF();
				}
			}
			else{
				LED1_OFF();
			}
			}
			else{
				LED1_OFF();
				//LED_B1 = 0;
			}
		}
		else if(comb == 1 || comb == 3 || (saitaning == 1 && emode == 1 && slantingtask == 0 && rl == RIGHT && ((WallCtrlEnbl == 1 && len_target-len_mouse <= 45.0f) || (WallCtrlEnbl != 1)))){
			if((!(saitaning == 1 && emode == 1 && slantingtask == 0 && rl == RIGHT && ((WallCtrlEnbl == 1 && len_target-len_mouse <= 45.0f) || (WallCtrlEnbl != 1))) && Sensor_R_Dis > WALL_TH_COMB_R && Before_R_Dis < WALL_TH_COMB_R) ||
							((saitaning == 1 && emode == 1 && slantingtask == 0 && rl == RIGHT && ((WallCtrlEnbl == 1 && len_target-len_mouse <= 45.0f) || (WallCtrlEnbl != 1))) && Before_R_Dis < WALL_TH_COMB_R_B && R_Diff_Smoothed > CUTDIFF2/* && Sensor_R_Dis > WALL_TH_COMB_R_B/*Before_R_Dis < WALL_TH_COMB_R_B && fabsf(Sensor_R_Dis-Before_R_Dis)>(SUDDENCHANGEB*tar_speed/0.3f)*//*Sensor_R_Dis > WALL_TH_COMB_R_B && Before_R_Dis < WALL_TH_COMB_R_B*/)){
				if(firstread == 0 && Before_R_Dis != 0 && Before_L_Dis != 0 && wallCutTask != 0 && slantingtask == 0){
					//LED1_ON();
					if(wallCutTask == 1){
						len_mouse = len_target - SEARCHWALLCUTC_R;
						firstread = 1;
						LED1_ON();
					}
					else if(wallCutTask == 2){
						len_mouse = len_target - SEARCHWALLCUTC_R;
						firstread = 1;
						LED1_ON();
					}
					else if(wallCutTask == 3){
						lentmp = len_mouse;
						blockcnt = 0;
						while(lentmp > 180.0f){
							lentmp -= 180.0f;
							blockcnt++;
						}
						if(lentmp >= 90.0f){
							len_mouse = 90.0f*(2*blockcnt+2) - SEARCHWALLCUTC_R;
						}
						else{
							len_mouse = 90.0f*(2*blockcnt+1) - SEARCHWALLCUTC_R;
						}
						LED1_ON();
						//firstread = 1;
					}
					else if(wallCutTask == 4){
						lentmp = (len_mouse-45.0f);
						blockcnt = 0;
						while(lentmp > 180.0f){
							lentmp -= 180.0f;
							blockcnt++;
						}
						if(lentmp < 0){
							//len_mouse = (float)(45.0 - SEARCHWALLCUTA_R);
							//LED_B1 = 1;
						}
						else{
							if(lentmp >= 90.0f){
								len_mouse = 90.0f*(2*blockcnt+2) - SEARCHWALLCUTC_R+45.0f;
							}
							else{
								len_mouse = 90.0f*(2*blockcnt+1) - SEARCHWALLCUTC_R+45.0f;
							}
						}
						LED1_ON();
						//firstread = 1;
					}
					else if(wallCutTask == 5 && (Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B/*7/6 壁切れ無限回読んでたので*/){
						lentmp = (len_mouse-45.0f);
						blockcnt = 0;
						if((len_target - len_mouse) > 90.0f){
							while(lentmp > 180.0f){
								lentmp -= 180.0f;
								blockcnt++;
							}
							if(lentmp < 0){
								//len_mouse = (float)(45.0 - SEARCHWALLCUTA_R);
								//LED_B1 = 1;
							}
							else{
								if(lentmp >= 90.0f){
									len_mouse = 90.0f*(2*blockcnt+2) - SAITANWALLCUTC_R+45.0f;
									R_Ctrl_Disable = 1;
									Dis_to_Able = len_true + AFTERCUT;
								}
								else{
									len_mouse = 90.0f*(2*blockcnt+1) - SAITANWALLCUTC_R+45.0f;
									R_Ctrl_Disable = 1;
									Dis_to_Able = len_true + AFTERCUT;
								}
							}
							LED1_ON();
						//firstread = 1;
						}
						else if((len_target - len_mouse) < 45.0f){
							/*len_mouse = len_target;
							turn_short = 1;
							turn_read = 1;
							LED1_ON();*/
							if(next == 0){
								len_mouse = len_target - SAITANWALLCUTD_R;//+SAITAN_CUTHOSEIR;
								R_Ctrl_Disable = 1;
								Dis_to_Able = len_true + AFTERCUT;
							}
							else{
								len_mouse = len_target - SAITANWALLCUTD_R/*+SAITAN_CUTHOSEIR*/-next+TURN_SHORT;
								R_Ctrl_Disable = 1;
								Dis_to_Able = len_true + AFTERCUT;
							}
							turn_short = 1;
							turn_read = 1;
							LED1_ON();
						}
						else{
						}
					}
					else{
						LED1_OFF();
					}
				}
				else{
					LED1_OFF();
				}
			}
			else{
				LED1_OFF();
				//LED_B1 = 0;
			}
		}
		if((comb == 0 || comb == 1) && !(saitaning == 1 && emode == 1 && slantingtask == 0 && rl != 0/*== LEFT*/ && ((WallCtrlEnbl == 1 && len_target-len_mouse <= 45.0f) || (WallCtrlEnbl != 1)))){
			if(slantingtask == 0){
				if((saitaning == 0 && Sensor_L_Dis > WALL_TH_L && Before_L_Dis < WALL_TH_L)
						|| (saitaning == 1 && /*Sensor_L_Dis > WALL_TH_L_SAI && */Before_L_Dis < WALL_TH_L_SAI &&  L_Diff_Smoothed > CUTDIFF1)){
					if(firstread == 0 && Before_R_Dis != 0 && Before_L_Dis != 0 && wallCutTask != 0 && slantingtask == 0){
						//LED2_ON();
						if(wallCutTask == 1){
							len_mouse = len_target - SEARCHWALLCUTA_L;
							firstread = 1;
							LED2_ON();
						}
						else if(wallCutTask == 2){
							len_mouse = len_target - SEARCHWALLCUTB_L;
							firstread = 1;
							LED2_ON();
						}
						else if(wallCutTask == 3){
							lentmp = len_mouse;
							blockcnt = 0;
							while(lentmp > 180.0f){
								lentmp -= 180.0f;
								blockcnt++;
							}
							if(lentmp >= 90.0f){
								len_mouse = 90.0f*(blockcnt*2+2) - SEARCHWALLCUTA_L;
							}
							else{
								len_mouse = 90.0f*(blockcnt*2+1) - SEARCHWALLCUTA_L;
							}
							LED2_ON();
							//firstread = 1;
						}
						else if(wallCutTask == 4){
							lentmp = (len_mouse-45.0f);
							blockcnt = 0;
							while(lentmp > 180.0f){
								lentmp -= 180.0f;
								blockcnt++;
							}
							if(lentmp < 0){
								//len_mouse = (float)(45.0 - SEARCHWALLCUTA_L);
								//LED_B2 = 1;
							}
							else{
								if(lentmp >= 90.0f){
									len_mouse = 90.0f*(blockcnt*2+2) - SEARCHWALLCUTA_L + 45.0f;
								}
								else{
									len_mouse = 90.0f*(blockcnt*2+1) - SEARCHWALLCUTA_L + 45.0f;
								}
							}
							LED2_ON();
							//firstread = 1;
						}
						else if(wallCutTask == 5 && (Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B/*7/6 壁切れ無限回読んでたので*/){
							lentmp = (len_mouse-45.0f);
							blockcnt = 0;
							if((len_target - len_mouse) > 90.0f){
								while(lentmp > 180.0f){
									lentmp -= 180.0f;
									blockcnt++;
								}
								if(lentmp < 0){
									//len_mouse = (float)(45.0 - SEARCHWALLCUTA_L);
									//LED_B2 = 1;
								}
								else{
									if(lentmp >= 90.0f){
										len_mouse = 90.0f*(blockcnt*2+2) - SAITANWALLCUTA_L + 45.0f;
										L_Ctrl_Disable = 1;
										Dis_to_Able2 = len_true + AFTERCUT;
									}
									else{
										len_mouse = 90.0f*(blockcnt*2+1) - SAITANWALLCUTA_L + 45.0f;
										L_Ctrl_Disable = 1;
										Dis_to_Able2 = len_true + AFTERCUT;
									}
								}
								LED2_ON();
							//firstread = 1;
							}
							else if((len_target - len_mouse) < 45.0f){
								if(next == 0){
									len_mouse = len_target-SAITANWALLCUTB_L;
									L_Ctrl_Disable = 1;
									Dis_to_Able2 = len_true + AFTERCUT;
								}
								else{
									len_mouse = len_target-SAITANWALLCUTB_L-next+TURN_SHORT;
									L_Ctrl_Disable = 1;
									Dis_to_Able2 = len_true + AFTERCUT;
								}
								turn_short = 1;
								turn_read = 1;
								LED2_ON();
							}
							else{
								LED2_OFF();
							}
						}
						else{
							LED2_OFF();
						}
					}

					else{
						LED2_OFF();
					}
				}
				else{
					LED2_OFF();
				}
			}
			else if(slantingtask == 1 || slantingtask == 2){
				if((longst == 1 && Sensor_L_Dis > WALL_TH_SLANT_L && Before_L_Dis < WALL_TH_SLANT_L) || (longst == 0 && Sensor_L_Dis > WALL_TH_SLANT_L_O && Before_L_Dis < WALL_TH_SLANT_L_O)/* || (longst != 1 && L_Dis_Ave <=  SLANT_CTRL_TH_COMB_L && (Sensor_L_Dis-Before_L_Dis) >= SUDDENCHANGEB_S)*/){
				if(/*firstread == 0 && Before_R_Dis != 0 &&*/ Before_L_Dis != 0 && wallCutTask != 0 && slantingtask == 1 && (WallCtrlEnbl == 1 || WallCtrlEnbl == 3 || WallCtrlEnbl == 6) && !(lastslant == 1 && len_target-len_mouse <= ONESLANTING*2.0f)){
					if(wallCutTask == 5){
						lentmp = len_mouse+SLANTWALLCUT_L;
						blockcnt = 0;
					//	if((len_target - lentmp) > ONESLANTING){
						if(WallCtrlEnbl == 1 && slantblock == 0 && (len_target-len_mouse) <= ONESLANTING && slantread == 0){
							len_mouse = len_target-SLANTWALLCUT_L;
							peekl = nowpeekl;
							nowpeekl = 0;
							slantread = 1;
							LED2_ON();
						}
						else{
							while(lentmp > ONESLANTING*2.0f){
								lentmp -= ONESLANTING*2.0f;
								blockcnt++;
							}
							if(lentmp >= ONESLANTING){
								len_mouse = ONESLANTING*(blockcnt*2+2) - SLANTWALLCUT_L;
								peekl = nowpeekl;
								nowpeekl = 0;
								LED2_ON();
							}
							else{
								if(blockcnt != 0){
									len_mouse = ONESLANTING*(blockcnt*2) - SLANTWALLCUT_L;
									peekl = nowpeekl;
									nowpeekl = 0;
									LED2_ON();
								}
								else{
									if(WallCtrlEnbl == 6){
									len_mouse = len_target-SLANTWALLCUT_L;
									peekl = nowpeekl;
									nowpeekl = 0;
									slantread = 1;
									naname_wallcut = 1;
									LED2_ON();
									}
								//	else{
								//		LED2_OFF();
								//	}
									if(WallCtrlEnbl == 3 && next2 != 0 && slantread == 0){
										len_mouse = len_target-SLANTWALLCUT_L-next2;
										peekl = nowpeekl;
										nowpeekl = 0;
										slantread = 1;
										naname_wallcut = 1;
										LED2_ON();
									}
								//	else{
								//		LED2_OFF();
								//	}
								}
							}
						}

						//firstread = 1;
					//	}
					//	else{
					//	}
					}
					else{
						LED2_OFF();
					}
				}
				else if(/*firstread == 0 && Before_R_Dis != 0 &&*/ Before_L_Dis != 0 && wallCutTask != 0 && slantingtask == 2 && (WallCtrlEnbl == 1 || WallCtrlEnbl == 3) && !(lastslant == 1 && len_target-len_mouse <= ONESLANTING*2.0f)){
					if(wallCutTask == 5){
						lentmp = len_mouse+SLANTWALLCUT_L+ONESLANTING;
						blockcnt = 0;
					//	if((len_target - lentmp) > 0){
						if(WallCtrlEnbl == 1 && slantblock == 1 && (len_target-len_mouse) <= ONESLANTING && slantread == 0){
							len_mouse = len_target-SLANTWALLCUT_L;
							peekl = nowpeekl;
							nowpeekl = 0;
							slantread = 1;
							LED2_ON();
						}
						else{
							while(lentmp > ONESLANTING*2.0f){
								lentmp -= ONESLANTING*2.0f;
								blockcnt++;
							}
							if(lentmp >= ONESLANTING){
								len_mouse = ONESLANTING*(blockcnt*2+1) - SLANTWALLCUT_L;
								peekl = nowpeekl;
								nowpeekl = 0;
								LED2_ON();
							}
							else{
								if(blockcnt != 0){
									len_mouse = ONESLANTING*(blockcnt*2-1) - SLANTWALLCUT_L;
									peekl = nowpeekl;
									nowpeekl = 0;
									LED2_ON();
								}
								else{
									LED2_OFF();
								}
							}
						}
						//firstread = 1;
					//	}
					//	else{
					//	}
					}
					else{
						LED2_OFF();
					}
				}
				else{
					LED2_OFF();
				}
			}
				else{
					LED2_OFF();
				}
			}
			else{
				LED2_OFF();
			}
		}
		else if(comb == 2 || comb == 3 || (saitaning == 1 && emode == 1 && slantingtask == 0 && rl == LEFT && ((WallCtrlEnbl == 1 && len_target-len_mouse <= 45.0f) || (WallCtrlEnbl != 1)))){
			if((!(saitaning == 1 && emode == 1 && slantingtask == 0 && rl == LEFT && ((WallCtrlEnbl == 1 && len_target-len_mouse <= 45.0f) || (WallCtrlEnbl != 1))) && Sensor_L_Dis > WALL_TH_COMB_L && Before_L_Dis < WALL_TH_COMB_L) ||
							((saitaning == 1 && emode == 1 && slantingtask == 0 && rl == LEFT && ((WallCtrlEnbl == 1 && len_target-len_mouse <= 45.0f) || (WallCtrlEnbl != 1))) && Before_L_Dis < WALL_TH_COMB_L_B && L_Diff_Smoothed > CUTDIFF2/* && Sensor_L_Dis > WALL_TH_COMB_L_B/*Before_L_Dis < WALL_TH_COMB_L_B && fabsf(Sensor_L_Dis-Before_L_Dis)>(SUDDENCHANGEB*tar_speed/0.3f) /*&& Sensor_L_Dis > WALL_TH_COMB_L_B && Before_L_Dis < WALL_TH_COMB_L_B*/)){
				if(firstread == 0 && Before_R_Dis != 0 && Before_L_Dis != 0 && wallCutTask != 0 && slantingtask == 0){
					//LED2_ON();
					if(wallCutTask == 1){
						len_mouse = len_target - SEARCHWALLCUTC_L;
						firstread = 1;
						LED2_ON();
					}
					else if(wallCutTask == 2){
						len_mouse = len_target - SEARCHWALLCUTC_L;
						firstread = 1;
						LED2_ON();
					}
					else if(wallCutTask == 3){
						lentmp = len_mouse;
						blockcnt = 0;
						while(lentmp > 180.0f){
							lentmp -= 180.0f;
							blockcnt++;
						}
						if(lentmp >= 90.0f){
							len_mouse = 90.0f*(blockcnt*2+2) - SEARCHWALLCUTC_L;
						}
						else{
							len_mouse = 90.0f*(blockcnt*2+1) - SEARCHWALLCUTC_L;
						}
						LED2_ON();
						//firstread = 1;
					}
					else if(wallCutTask == 4){
						lentmp = (len_mouse-45.0f);
						blockcnt = 0;
						while(lentmp > 180.0f){
							lentmp -= 180.0f;
							blockcnt++;
						}
						if(lentmp < 0){
							//len_mouse = (float)(45.0 - SEARCHWALLCUTA_L);
							//LED_B2 = 1;
						}
						else{
							if(lentmp >= 90.0f){
								len_mouse = 90.0f*(blockcnt*2+2) - SEARCHWALLCUTC_L + 45.0f;
							}
							else{
								len_mouse = 90.0f*(blockcnt*2+1) - SEARCHWALLCUTC_L + 45.0f;
							}
						}
						LED2_ON();
						//firstread = 1;
					}
					else if(wallCutTask == 5 && (Sensor_FL_Dis + Sensor_FR_Dis) > FWALL_CLOSE_B/*7/6 壁切れ無限回読んでたので*/){
						lentmp = (len_mouse-45.0f);
						blockcnt = 0;
						if((len_target - len_mouse) > 90.0f){
							while(lentmp > 180.0f){
								lentmp -= 180.0f;
								blockcnt++;
							}
							if(lentmp < 0){
								//len_mouse = (float)(45.0 - SEARCHWALLCUTA_L);
								//LED_B2 = 1;
							}
							else{
								if(lentmp >= 90.0f){
									len_mouse = 90.0f*(blockcnt*2+2) - SAITANWALLCUTC_L + 45.0f;
									L_Ctrl_Disable = 1;
									Dis_to_Able2 = len_true + AFTERCUT;
								}
								else{
									len_mouse = 90.0f*(blockcnt*2+1) - SAITANWALLCUTC_L + 45.0f;
									L_Ctrl_Disable = 1;
									Dis_to_Able2 = len_true + AFTERCUT;
								}
							}
							LED2_ON();
						//firstread = 1;
						}
						else if((len_target - len_mouse) < 45.0f){
							/*len_mouse = len_target;
							turn_short = 1;
							turn_read = 1;
							LED2_ON();*/
							if(next == 0){
								len_mouse = len_target-SAITANWALLCUTD_L;
								L_Ctrl_Disable = 1;
								Dis_to_Able2 = len_true + AFTERCUT;
							}
							else{
								len_mouse = len_target-SAITANWALLCUTD_L-next+TURN_SHORT;
								L_Ctrl_Disable = 1;
								Dis_to_Able2 = len_true + AFTERCUT;
							}
							turn_short = 1;
							turn_read = 1;
							LED2_ON();
						}
						else{
							LED2_OFF();
						}
					}
					else{
						LED2_OFF();
					}

				}

				else{
					LED2_OFF();
				}
			}
			else{
				LED2_OFF();
			}
		}
		if(wallCutTask == 0){
			firstread = 0;
		}
		if((WallCtrlEnbl == 1|| WallCtrlEnbl == 3) && slantingtask == 0 && /*Sensor_FR_Dis + Sensor_FL_Dis*/FR_Dis_Ave + FL_Dis_Ave > FWALL_CLOSE && !(accel < 0 && (tar_speed*4.0f) < max_speed && saitaning == 0)/* && tar_speed <= FASTFORCTRL*/){
			WallCtrlErrP = WallCtrlErr;	//過去の偏差を保存
				if( !(((saitaning == 1 && WallCtrlEnbl == 1 && comb_onoff == 1) || (searching_flag == 1 && comb == 3)) && ((beecombr != 0 && prev_combr != 0 && fabsf(beecombr-prev_combr) >= 45.0f && fabsf(beecombr-prev_combr) <= 135.0f) || (beecombl != 0 && prev_combl != 0 && fabsf(beecombl-prev_combl) >= 45.0f && fabsf(beecombl-prev_combl) <= 135.0f) || (fabsf(beecombr - beecombl) <= 45.0f && beecombr != 0 && beecombl != 0))) && combbb== 0 && ( Sensor_R_Ctrl_Flag == FLAG_ON && rsensor_sudden_minus == 0 && rsensor_sudden_plus == 0 && R_Ctrl_Disable != 1) && ( Sensor_L_Ctrl_Flag == FLAG_ON && lsensor_sudden_minus == 0  && lsensor_sudden_plus == 0 && L_Ctrl_Disable != 1) ){									//両方とも有効だった場合の偏差を計算

						WallCtrlErr = (float)(Sensor_R_Err - Sensor_L_Err);
						Sensor_Raw_Err = (float)(R_Raw_Err - L_Raw_Err);
						I_tar_ang_vel = 0;
						I_ang_vel = 0;

				}
				else if(!(((saitaning == 1 && WallCtrlEnbl == 1 && comb_onoff == 1) || (searching_flag == 1 && comb == 3)) && ((beecombr != 0 && prev_combr != 0 && fabsf(beecombr-prev_combr) >= 45.0f && fabsf(beecombr-prev_combr) <= 135.0f) || (beecombl != 0 && prev_combl != 0 && fabsf(beecombl-prev_combl) >= 45.0f && fabsf(beecombl-prev_combl) <= 135.0f) || (fabsf(beecombr - beecombl) <= 45.0f && beecombr != 0 && beecombl != 0))) && combbb == 0 && (Sensor_R_Ctrl_Flag == FLAG_OFF && rsensor_sudden_minus == 0/* && rsensor_sudden_plus == 0 8/20 制御のかかりが悪いので削除*/) && (Sensor_L_Ctrl_Flag == FLAG_ON && lsensor_sudden_minus == 0  && lsensor_sudden_plus == 0 && L_Ctrl_Disable != 1) )							//片方もしくは両方のセンサが無効だった場合の偏差を計算
				{

						WallCtrlErr = (float)(2.0f*(/*Sensor_R_Err */- Sensor_L_Err));
						Sensor_Raw_Err = (float)(2.0f*(- L_Raw_Err));
						I_tar_ang_vel = 0;
						I_ang_vel = 0;
						LED3_OFF();

				}
				else if(!(((saitaning == 1 && WallCtrlEnbl == 1 && comb_onoff == 1) || (searching_flag == 1 && comb == 3)) && ((beecombr != 0 && prev_combr != 0 && fabsf(beecombr-prev_combr) >= 45.0f && fabsf(beecombr-prev_combr) <= 135.0f) || (beecombl != 0 && prev_combl != 0 && fabsf(beecombl-prev_combl) >= 45.0f && fabsf(beecombl-prev_combl) <= 135.0f) || (fabsf(beecombr - beecombl) <= 45.0f && beecombr != 0 && beecombl != 0))) && combbb== 0 && (Sensor_R_Ctrl_Flag == FLAG_ON && rsensor_sudden_minus == 0 && rsensor_sudden_plus == 0 && R_Ctrl_Disable != 1) && (Sensor_L_Ctrl_Flag == FLAG_OFF && lsensor_sudden_minus == 0  /*&& lsensor_sudden_plus == 0*/) ){

						WallCtrlErr = (float)(2.0f*(Sensor_R_Err/* - Sensor_L_Err*/));
						Sensor_Raw_Err = (float)(2.0f*R_Raw_Err);
						I_tar_ang_vel = 0;
						I_ang_vel = 0;
						LED3_OFF();

				}
				else{
					if((((saitaning == 1 && WallCtrlEnbl == 1 && comb_onoff == 1) || (searching_flag == 1 && comb == 3)) && ((beecombr != 0 && prev_combr != 0 && fabsf(beecombr-prev_combr) >= 45.0f && fabsf(beecombr-prev_combr) <= 135.0f) || (beecombl != 0 && prev_combl != 0 && fabsf(beecombl-prev_combl) >= 45.0f && fabsf(beecombl-prev_combl) <= 135.0f) || (fabsf(beecombr - beecombl) <= 45.0f && beecombr != 0 && beecombl != 0))) || combbb != 0){
						if(beecombr != 0 && beecombl != 0){
							if(beecombr > beecombl){
								if(saitaning == 1){
									if(max_speed <= 1.5f){
										WallCtrlErr = ((asin((beecombr-beecombl)/ONESECTION))/WALL_KP)*20.0f;
										Sensor_Raw_Err = 0;
									}
									else if(max_speed <= 2.5f){
										WallCtrlErr = ((asin((beecombr-beecombl)/ONESECTION))/WALL_KP)*40.0f;
										Sensor_Raw_Err = 0;
									}
									else{
										WallCtrlErr = ((asin((beecombr-beecombl)/ONESECTION))/WALL_KP)*100.0f;
										Sensor_Raw_Err = 0;
									}
								}
								else{
									WallCtrlErr = ((asin((beecombr-beecombl)/ONESECTION))/WALL_KP)*10.0f;
									Sensor_Raw_Err = 0;
								}
								LED3_ON();
								combbb++;

							}
							else if(beecombr < beecombl){
								if(saitaning == 1){
									if(max_speed <= 1.5f){
										WallCtrlErr = -1.0f*((asin((beecombl-beecombr)/ONESECTION))/WALL_KP)*20.0f;
										Sensor_Raw_Err = 0;
									}
									else if(max_speed <= 2.5f){
										WallCtrlErr = -1.0f*((asin((beecombl-beecombr)/ONESECTION))/WALL_KP)*40.0f;
										Sensor_Raw_Err = 0;
									}
									else{
										WallCtrlErr = -1.0f*((asin((beecombl-beecombr)/ONESECTION))/WALL_KP)*100.0f;
										Sensor_Raw_Err = 0;
									}
								}
								else{
									WallCtrlErr = -1.0f*((asin((beecombl-beecombr)/ONESECTION))/WALL_KP)*10.0f;
									Sensor_Raw_Err = 0;
								}
								LED3_ON();
								combbb++;
							}
							else{
								WallCtrlErr = 0;
								Sensor_Raw_Err = 0;
								LED3_ON();
								combbb++;
							}
						}
						else if(beecombr != 0 && prev_combr != 0 && beecombr > prev_combr &&  fabsf(beecombr-prev_combr) >= 45.0f && (beecombr-prev_combr) <= 135.0f){
							if(saitaning == 1){
								if(max_speed <= 1.5f){
									if((beecombr-prev_combr) > ONESECTION){
										WallCtrlErr = ((acos(ONESECTION/(beecombr-prev_combr)))/WALL_KP)*20.0f;
										Sensor_Raw_Err = 0;
									}
									else{
										WallCtrlErr = -1.0f*((acos((beecombr-prev_combr)/ONESECTION))/WALL_KP)*20.0f;
										Sensor_Raw_Err = 0;
									}
								}
								else{
									if((beecombr-prev_combr) > ONESECTION){
										WallCtrlErr = ((acos(ONESECTION/(beecombr-prev_combr)))/WALL_KP)*40.0f;
										Sensor_Raw_Err = 0;
									}
									else{
										WallCtrlErr = -1.0f*((acos((beecombr-prev_combr)/ONESECTION))/WALL_KP)*40.0f;
										Sensor_Raw_Err = 0;
									}
								}
							}
							else{
								if((beecombr-prev_combr) > ONESECTION){
									WallCtrlErr = ((acos(ONESECTION/(beecombr-prev_combr)))/WALL_KP)*10.0f;
									Sensor_Raw_Err = 0;
								}
								else{
									WallCtrlErr = -1.0f*((acos((beecombr-prev_combr)/ONESECTION))/WALL_KP)*10.0f;
									Sensor_Raw_Err = 0;
								}
							}
							LED3_ON();
							combbb++;
						}
						else if(beecombl != 0 && prev_combl != 0 && beecombl > prev_combl && fabsf(beecombr-prev_combr) >= 45.0f && (beecombl-prev_combl) <= 135.0f){
							if(saitaning == 1){
								if(max_speed <= 1.5f){
									if((beecombl-prev_combl) > ONESECTION){
										WallCtrlErr = -1.0f*((acos(ONESECTION/(beecombl-prev_combl)))/WALL_KP)*20.0f;
										Sensor_Raw_Err = 0;
									}
									else{
										WallCtrlErr = ((acos((beecombl-prev_combl)/ONESECTION))/WALL_KP)*20.0f;
										Sensor_Raw_Err = 0;
									}
								}
								else{
									if((beecombl-prev_combl) > ONESECTION){
										WallCtrlErr = -1.0f*((acos(ONESECTION/(beecombl-prev_combl)))/WALL_KP)*40.0f;
										Sensor_Raw_Err = 0;
									}
									else{
										WallCtrlErr = ((acos((beecombl-prev_combl)/ONESECTION))/WALL_KP)*40.0f;
										Sensor_Raw_Err = 0;
									}
								}
							}
							else{
								if((beecombl-prev_combl) > ONESECTION){
									WallCtrlErr = -1.0f*((acos(ONESECTION/(beecombl-prev_combl)))/WALL_KP)*10.0f;
									Sensor_Raw_Err = 0;
								}
								else{
									WallCtrlErr = ((acos((beecombl-prev_combl)/ONESECTION))/WALL_KP)*10.0f;
									Sensor_Raw_Err = 0;
								}
							}
							LED3_ON();
							combbb++;
						}
						else{
						/*	if(saitaning == 1 && WallCtrlEnbl == 1 && comb_onoff == 1){
								if(FR_Dis_Ave <= 180 && FL_Dis_Ave <= 180){
									WallCtrlErr = (-1.0*COMB_FWALL_KP/FL_Dis_Ave+COMB_FWALL_KP/FR_Dis_Ave);
									LED5_ON();
								}
								else if(FR_Dis_Ave <= 180 && FL_Dis_Ave > 180){
									WallCtrlErr = (2.0*COMB_FWALL_KP/FR_Dis_Ave);
									LED5_ON();
								}
								else if(FR_Dis_Ave > 180 && FL_Dis_Ave <= 180){
									WallCtrlErr = (-2.0*COMB_FWALL_KP/FL_Dis_Ave);
									LED5_ON();
								}
							}
							else{
								LED5_OFF();
							}*/
						}

						if(saitaning == 1 && max_speed <= 1.5f && combbb >= 50.0f){
							LED3_OFF();
							combbb = 0;
						//	if(beecombr != 0){
						//		prev_combr = beecombr;
						//	}else{
								prev_combr = 0;
						//	}
						//	if(beecombl != 0){
						//		prev_combl = beecombl;
						//	}
						//	else{
								prev_combl = 0;
						//	}
							beecombr = 0;
							beecombl = 0;
							WallCtrlErr = 0;
							Sensor_Raw_Err = 0;
						}
						if(saitaning == 1 && max_speed > 1.5f && combbb >= 25.0f){
							LED3_OFF();
							combbb = 0;
						//	if(beecombr != 0){
						//		prev_combr = beecombr;
						//	}else{
								prev_combr = 0;
						//	}
						//	if(beecombl != 0){
						//		prev_combl = beecombl;
						//	}
						//	else{
								prev_combl = 0;
						//	}
							beecombr = 0;
							beecombl = 0;
							WallCtrlErr = 0;
							Sensor_Raw_Err = 0;
						}
						if(saitaning == 0 && combbb >= 100.0f){
							LED3_OFF();
							combbb = 0;
						//	if(beecombr != 0){
						//		prev_combr = beecombr;
						//	}else{
								prev_combr = 0;
						//	}
						//	if(beecombl != 0){
						//		prev_combl = beecombl;
						//	}
						//	else{
								prev_combl = 0;
						//	}
							beecombr = 0;
							beecombl = 0;
							WallCtrlErr = 0;
							Sensor_Raw_Err = 0;
						}
						I_tar_ang_vel = 0;
						I_ang_vel = 0;
					}
					else{
						LED3_OFF();
						WallCtrlErr = 0;
						Sensor_Raw_Err = 0;
						tar_ang_vel = 0;
						//I_tar_ang_vel = 0;
						//I_ang_vel = 0;
						wallCtrlReset();
						//LED_B3 = 0;
						//LED_B4 = 0;
					}
				}
		//	}
			//DI制御計算
				WallCtrlD = WallCtrlErr - WallCtrlErrP;	//偏差の微分値を計算
				//WallCtrlI += WallCtrlErr;				//偏差の積分値を計算



				WallCtrlOmegaP = WallCtrlOmega;
				if(saitaning == 1/* && max_speed >= 1.5f*/ && combbb == 0){
					if(R_Ctrl_Disable == 1 || L_Ctrl_Disable == 1){
					WallCtrlOmega = 0;//(WALL_KP_HIGH * WallCtrlErr/**tar_speed/2.0-WALL_KD2*WallCtrlD*/);	//現在の目標角速度[rad/s]を計算
					}
					else{
					WallCtrlOmega = 0;//(WALL_KP_HIGH * WallCtrlErr/**tar_speed/2.0-WALL_KD2*WallCtrlD*/);	//現在の目標角速度[rad/s]を計算
					}
					//kojima_kd=ang_vel*(WALL_KD_HIGH + K_HIGH*speed);
					//WallCtrlOmega -= kojima_kd;
					ang_acc = -1.0f*(ang_vel*(WALL_KD_HIGH + K_HIGH*speed) + WALL_KP_HIGH * Sensor_Raw_Err);
				}
				else if(saitaning == 1 && combbb != 0){
					WallCtrlOmega = (WALL_KP * WallCtrlErr/*+WALL_KD*WallCtrlD*/);	//現在の目標角速度[rad/s]を計算
					ang_acc = -1.0f*(ang_vel*(WALL_KD_HIGH + K_HIGH*speed));
				}
				else{
					WallCtrlOmega = (WALL_KP * WallCtrlErr/*+WALL_KD*WallCtrlD*/);	//現在の目標角速度[rad/s]を計算
				}

				if(saitaning == 1/* && max_speed >= 1.5f */&& combbb == 0){
					tar_ang_vel = WallCtrlOmega+ang_acc*0.001f;
					if(tar_ang_vel > 10.0f){
						tar_ang_vel = 10.0f;
					}
					if(tar_ang_vel < -10.0f){
						tar_ang_vel = -10.0f;
					}
				}
				else if(saitaning == 1 && combbb  != 0){
					tar_ang_vel = WallCtrlOmega+ang_acc*0.001f;
					if(tar_ang_vel > 10.0f){
						tar_ang_vel = 10.0f;
					}
					if(tar_ang_vel < -10.0f){
						tar_ang_vel = -10.0f;
					}
				}
				else{
					tar_ang_vel = WallCtrlOmega;
				/*	if(saitaning == 1 && turnwflag == 1 && FR_Dis_Ave <= 150.0f &&  FL_Dis_Ave <= 150.0f){	//前壁補正
						//LED4_ON();
						//if(turn_read == 0){
					//	tar_ang_vel = WallCtrlOmega + (FR_Dis_Ave - FL_Dis_Ave)*KP_BEFORETURN;	//角度
						//}
							//else{
							//tar_ang_vel = WallCtrlOmega;
							//}
						if(emode != 0 && turn_read == 0){

							len_mouse = len_target - ((FR_Dis_Ave+FL_Dis_Ave)/2.0f - (SLA_FWALL_ALL_B-offsetdis));	//距離
							//LED4_ON();
						}
						else{
							//LED4_OFF();
						//	tar_ang_vel = WallCtrlOmega;
						}
					}
					else{
						//LED4_OFF();
						tar_ang_vel = WallCtrlOmega;
					}*/
				}

				if(saitaning == 1 && FR_Dis_Ave+FL_Dis_Ave <= 240.0f){	//前壁までの距離120mm以内なら打ち切り？

				}
		}
		else if(!(lastslant == 1 && len_target-len_mouse <= ONESLANTING*2.0f) && (WallCtrlEnbl == 1 || WallCtrlEnbl == 3 || WallCtrlEnbl == 6) && (slantingtask == 1 || slantingtask == 2) /*&& longst == 1 && len_target >= ONESLANTING*/){
			combbb = 0;
			beecombr = 0;
			beecombl = 0;
			if((Sensor_FL_Dis < SLANTING_F_L && Sensor_FR_Dis < SLANTING_F_R) || (Sensor_FL_Dis < SLANTING_F_R && Sensor_FR_Dis < SLANTING_F_L)){
				if(Sensor_FL_Dis != 0 && Sensor_FR_Dis != 0){
					if((len_target-len_mouse) <= ONESLANTING){
						tar_ang_vel =  (float)((Sensor_FL_Dis-Sensor_FR_Dis)*KP_WALL_SLANT3);//(float)(-1.0f * KP_WALL_SLANT3/Sensor_FL_Dis + KP_WALL_SLANT3/Sensor_FR_Dis);
					}
					else if(tar_speed > 1.0f){
					tar_ang_vel =  (float)((Sensor_FL_Dis-Sensor_FR_Dis)*KP_WALL_SLANT2);//(float)(-1.0f * KP_WALL_SLANT2/Sensor_FL_Dis + KP_WALL_SLANT2/Sensor_FR_Dis);
					}
					else{
					tar_ang_vel =  (float)((Sensor_FL_Dis-Sensor_FR_Dis)*KP_WALL_SLANT);//(float)(-1.0f * KP_WALL_SLANT/Sensor_FL_Dis + KP_WALL_SLANT/Sensor_FR_Dis);
					}
					I_tar_ang_vel = 0;
					I_ang_vel = 0;	//なんか入ってなかったのが違和感あったので追加 8/14
				}
			}
		/*	else if(Sensor_FL_Dis < SLANTING_F_L && Sensor_FR_Dis > SLANTING_F_R){
				if(Sensor_FL_Dis != 0){
					if((len_target-len_mouse) <= ONESLANTING){
						tar_ang_vel = (float)( -2.0f * KP_WALL_SLANT3/Sensor_FL_Dis);
					}
					else if(tar_speed > 1.0f){
					tar_ang_vel = (float)( -2.0f * KP_WALL_SLANT2/Sensor_FL_Dis);
					}
					else{
					tar_ang_vel = (float)( -2.0f * KP_WALL_SLANT/Sensor_FL_Dis);
					}
					I_tar_ang_vel = 0;
					I_ang_vel = 0;
				}
			}
			else if(Sensor_FL_Dis > SLANTING_F_L && Sensor_FR_Dis < SLANTING_F_R){
				if(Sensor_FR_Dis != 0){
					if((len_target-len_mouse) <= ONESLANTING){
						tar_ang_vel =  (float)(2.0f * KP_WALL_SLANT3/Sensor_FR_Dis);
					}
					else if(tar_speed > 1.0f){
					tar_ang_vel =  (float)(2.0f * KP_WALL_SLANT2/Sensor_FR_Dis);
					}
					else{
					tar_ang_vel =  (float)(2.0f * KP_WALL_SLANT/Sensor_FR_Dis);
					}
					I_tar_ang_vel = 0;
					I_ang_vel = 0;
				}
			}*/
			else/* if(Sensor_FL_Dis> SLANTING_F_L && Sensor_FR_Dis > SLANTING_F_R)*/{
				tar_ang_vel = 0;
			}

			if((Sensor_R_Dis < SLANT_CTRL_TH_R && Sensor_L_Dis < SLANT_CTRL_TH_L) || (Sensor_R_Dis < SLANT_CTRL_TH_L && Sensor_L_Dis < SLANT_CTRL_TH_R)){
				tar_ang_vel += (float)((- Sensor_R_Dis)*KP_SLANT_SIDE-(- Sensor_L_Dis)*KP_SLANT_SIDE);
				I_tar_ang_vel = 0;
				I_ang_vel = 0;
			}
			/*else if(Sensor_R_Dis < SLANT_CTRL_TH_R && Sensor_L_Dis >= SLANT_CTRL_TH_L){
				tar_ang_vel += (float)((SLANT_CTRL_TH_R - Sensor_R_Dis)*KP_SLANT_SIDE*1.0f);
				I_tar_ang_vel = 0;
				I_ang_vel = 0;
			}
			else if(Sensor_R_Dis >= SLANT_CTRL_TH_R && Sensor_L_Dis < SLANT_CTRL_TH_L){
				tar_ang_vel += (float)(-(SLANT_CTRL_TH_L - Sensor_L_Dis)*KP_SLANT_SIDE*1.0f);
				I_tar_ang_vel = 0;
				I_ang_vel = 0;
			}*/
			else{
				tar_ang_vel += 0;
			}

			//else{
			/*	if(((saitaning == 1 && WallCtrlEnbl == 1 && slantingtask != 0) && fabsf(beecombr_s - beecombl_s) <= (ONESLANTING*2.0f) && beecombr_s != 0 && beecombl_s != 0) || combbb_s != 0){
						if(beecombr_s > beecombl_s && (beecombr_s-ONESLANTING) > beecombl_s){
							//if(saitaning == 1){
								if(max_speed <= 1.0f){
									tar_ang_vel += asin((beecombr_s-beecombl_s-ONESLANTING)/(ONESLANTING*2.0f))*20.0f;
								}
								else if(max_speed <= 2.0f){
									tar_ang_vel += asin((beecombr_s-beecombl_s-ONESLANTING)/(ONESLANTING*2.0f))*40.0f;
								}
								else{
									tar_ang_vel += asin((beecombr_s-beecombl_s-ONESLANTING)/(ONESLANTING*2.0f))*100.0f;
								}
							//}
							//else{
							//	WallCtrlErr = ((asin((beecombr-beecombl)/ONESECTION))/WALL_KP)*10.0;
							//}
							LED3_ON();
							combbb_s++;

						}
						else if(beecombr_s > beecombl_s && (beecombr_s-ONESLANTING) <= beecombl_s){
							//if(saitaning == 1){
								if(max_speed <= 1.0f){
									tar_ang_vel += -1.0f*asin((beecombl_s-beecombr_s+ONESLANTING)/(ONESLANTING*2.0f))*20.0f;
								}
								else if(max_speed <= 2.0f){
									tar_ang_vel += -1.0f*asin((beecombl_s-beecombr_s+ONESLANTING)/(ONESLANTING*2.0f))*40.0f;
								}
								else{
									tar_ang_vel += -1.0f*asin((beecombl_s-beecombr_s+ONESLANTING)/(ONESLANTING*2.0f))*100.0f;
								}
							//}
							//else{
							//	WallCtrlErr = ((asin((beecombr-beecombl)/ONESECTION))/WALL_KP)*10.0;
							//}
							LED3_ON();
							combbb_s++;

						}
						else if(beecombr_s < beecombl_s && (beecombl_s-ONESLANTING) > beecombr_s){
							//if(saitaning == 1){
								if(max_speed <= 1.0){
									tar_ang_vel += -1.0f*asin((beecombl_s-beecombr_s-ONESLANTING)/(ONESLANTING*2.0f))*20.0f;
								}
								else if(max_speed <= 2.0){
									tar_ang_vel += -1.0f*asin((beecombl_s-beecombr_s-ONESLANTING)/(ONESLANTING*2.0f))*40.0;
								}
								else{
									tar_ang_vel += -1.0f*asin((beecombl_s-beecombr_s-ONESLANTING)/(ONESLANTING*2.0f))*100.0f;
								}
							//}
							//else{
							//	WallCtrlErr = -1*((asin((beecombl-beecombr)/ONESECTION))/WALL_KP)*10.0;
							//}
							LED3_ON();
							combbb_s++;
						}
						else if(beecombr_s < beecombl_s && (beecombl_s-ONESLANTING) <= beecombr_s){
							//if(saitaning == 1){
								if(max_speed <= 1.0f){
									tar_ang_vel += asin((beecombr_s-beecombl_s+ONESLANTING)/(ONESLANTING*2.0f))*20.0f;
								}
								else if(max_speed <= 2.0f){
									tar_ang_vel += asin((beecombr_s-beecombl_s+ONESLANTING)/(ONESLANTING*2.0f))*40.0f;
								}
								else{
									tar_ang_vel += asin((beecombr_s-beecombl_s+ONESLANTING)/(ONESLANTING*2.0f))*100.0f;
								}
							//}
							//else{
							//	WallCtrlErr = -1*((asin((beecombl-beecombr)/ONESECTION))/WALL_KP)*10.0;
							//}
							LED3_ON();
							combbb_s++;
						}
						else{
							tar_ang_vel += 0;
							LED3_ON();
							combbb_s++;
						}
						if(saitaning == 1 && max_speed <= 1.0f && combbb_s >= 50.0f){
							LED3_OFF();
							combbb_s = 0;
							if(beecombr_s > beecombl_s){
								beecombl_s = 0;
							}
							else if(beecombr_s < beecombl_s){
								beecombr_s = 0;
							}
							else{
							beecombr_s = 0;
							beecombl_s = 0;
							}
							tar_ang_vel += 0;
						}
						else if(saitaning == 1 && max_speed <= 2.0f && combbb_s >= 25.0f){
							LED3_OFF();
							combbb_s = 0;
							if(beecombr_s > beecombl_s){
								beecombl_s = 0;
							}
							else if(beecombr_s < beecombl_s){
								beecombr_s = 0;
							}
							else{
							beecombr_s = 0;
							beecombl_s = 0;
							}
							tar_ang_vel += 0;
						}
						else if(saitaning == 1 && max_speed > 2.0f && combbb_s >= 10.0f){
							LED3_OFF();
							combbb_s = 0;
							if(beecombr_s > beecombl_s){
								beecombl_s = 0;
							}
							else if(beecombr_s < beecombl_s){
								beecombr_s = 0;
							}
							else{
							beecombr_s = 0;
							beecombl_s = 0;
							}
							tar_ang_vel += 0;
						}
						I_tar_ang_vel = 0;
						I_ang_vel = 0;
					}*/
			//}


			if(saitaning == 1 && max_speed >= 1.5f){
					//WallCtrlOmega = (WALL_KP_HIGH * WallCtrlErr*tar_speed/2.0-WALL_KD2*WallCtrlD);	//現在の目標角速度[rad/s]を計算
					//kojima_kd=ang_vel*(WALL_KD_HIGH + K_HIGH*speed);
					//WallCtrlOmega -= kojima_kd;
				//ang_acc = -ang_vel*(WALL_KD_HIGH + K_HIGH*speed);
				//Ftar_ang_vel+= ang_acc*0.001f;
			}

			if(tar_ang_vel > 10.0f){
				tar_ang_vel = 10.0f;
			}
			if(tar_ang_vel < -10.0f){
				tar_ang_vel = -10.0f;
			}

		}
		else{
			combbb = 0;
			beecombr = 0;
			beecombl = 0;
			combbb_s = 0;
			beecombr_s = 0;
			beecombl_s = 0;
			WallCtrlErr = 0;
			Sensor_Raw_Err = 0;
			tar_ang_vel = 0;
			wallCtrlReset();
		}
	}
	else if(run_mode == OFFSET_A_MODE){
		if(Wall_FR == WALL_ON && Wall_FL == WALL_ON && slafwallflag == 1/* && firstread == 0*/){
			if(fabsf((Sensor_FR_Dis+Sensor_FL_Dis)/2.0f - sla_fwall_a) < 0.2f){
				slafwallflag = 0;
				//len_target = len_mouse;
				//tar_ang_vel = 0;
			}
			else{
				len_target = len_mouse + (((Sensor_FR_Dis+Sensor_FL_Dis)/2.0f)-sla_fwall_a);
				//tar_ang_vel = (FR_Dis_Ave - FL_Dis_Ave)*KP_BEFORETURN;	//角度
			}
		}

		if(slaswallflag == 1){
			if(TURN_DIR == RIGHT){
				if(Wall_R == WALL_ON && Wall_L == WALL_ON && FR_Dis_Ave + FL_Dis_Ave > FWALL_CLOSE_SLA){
					if(fabsf(Sensor_R_Dis - Before_R_Dis) > SUDDENCHANGE){
						offsetplus -= (Sensor_L_Dis - Sensor_Sla_L_Ref);
					}
					else if(fabsf(Sensor_L_Dis - Before_L_Dis) > SUDDENCHANGE){
						offsetplus += (Sensor_R_Dis - Sensor_Sla_R_Ref);
					}
					else{
						offsetplus += (((Sensor_R_Dis - Sensor_Sla_R_Ref)-(Sensor_L_Dis - Sensor_Sla_L_Ref))/2.0f);
					}
					offsetcnt++;
				}
				else if(Wall_R == WALL_ON && Wall_L == WALL_OFF && FR_Dis_Ave + FL_Dis_Ave > FWALL_CLOSE_SLA){
					if(fabsf(Sensor_R_Dis - Before_R_Dis) > SUDDENCHANGE){
					}
					else{
						offsetplus += (Sensor_R_Dis - Sensor_Sla_R_Ref);
						offsetcnt++;
					}
				}
				else if(Wall_R == WALL_OFF && Wall_L == WALL_ON && FR_Dis_Ave + FL_Dis_Ave > FWALL_CLOSE_SLA){
					if(fabsf(Sensor_L_Dis - Before_L_Dis) > SUDDENCHANGE){
					}
					else{
						offsetplus -= (Sensor_L_Dis - Sensor_Sla_L_Ref);
						offsetcnt++;
					}
				}
				else{
					offsetplus = 0;
					offsetcnt = 0;
				}
			}
			else if(TURN_DIR == LEFT){
				if(Wall_R == WALL_ON && Wall_L == WALL_ON && FR_Dis_Ave + FL_Dis_Ave > FWALL_CLOSE_SLA){
					offsetplus += (((Sensor_L_Dis - Sensor_Sla_L_Ref)-(Sensor_R_Dis - Sensor_Sla_R_Ref))/2.0f);
					offsetcnt++;
				}
				else if(Wall_R == WALL_ON && Wall_L == WALL_OFF && FR_Dis_Ave + FL_Dis_Ave > FWALL_CLOSE_SLA){
					offsetplus -= (Sensor_R_Dis - Sensor_Sla_R_Ref);
					offsetcnt++;
				}
				else if(Wall_R == WALL_OFF && Wall_L == WALL_ON && FR_Dis_Ave + FL_Dis_Ave > FWALL_CLOSE_SLA){
					offsetplus += (Sensor_L_Dis - Sensor_Sla_L_Ref);
					offsetcnt++;
				}
				else{
					offsetplus = 0;
					offsetcnt = 0;
				}
			}
		}
		else{
			offsetplus = 0;
			offsetcnt = 0;
		}
	}
	else if(run_mode == F_WALL_MODE){
		if(Sensor_FR_Dis < 120.0f && Sensor_FL_Dis < 120.0f/*90.0f*/ && fwallnoflag != 1){
			fwallcnt++;
			tar_speed = ((Sensor_FR_Dis - FWALL_REF_R)+(Sensor_FL_Dis - FWALL_REF_L))*FWALL_KP_SP;
			tar_ang_vel = ((Sensor_FR_Dis - FWALL_REF_R)-(Sensor_FL_Dis - FWALL_REF_L))*FWALL_KP_ANG + I_fwall * FWALL_KI_ANG;
			I_fwall += tar_ang_vel;
			if(I_fwall >30*10000000000){
				I_fwall = 30*10000000000;
			}else if(I_fwall < -1*10000000000){
				I_fwall = 1*10000000000;
			}
			if(tar_speed > 0.2f){
				tar_speed = 0.2f;
			}
			else if(tar_speed < -0.2f){
				tar_speed = -0.2f;
			}
			if(tar_ang_vel > 5.0f){//20.0f
				tar_ang_vel = 5.0f;
			}
			else if(tar_ang_vel < -5.0f){
				tar_ang_vel= -5.0f;
			}
		}
		else{
			fwallnoflag = 1;
		}
	}else if(run_mode == NON_CON_MODE){
	}else{
	}
	//
	I_tar_speed += tar_speed;
	if(I_tar_speed >30*10000000000){
		I_tar_speed = 30*10000000000;
	}else if(I_tar_speed < -1*10000000000){
		I_tar_speed = -1*10000000000;
	}

	I_tar_ang_vel += tar_ang_vel;
	if(I_tar_ang_vel >30*10000000000){
		I_tar_ang_vel = 30*10000000000;
	}else if(I_tar_ang_vel < -1*10000000000){
		I_tar_ang_vel = -1*10000000000;
	}

	hensa_speed_p = hensa_speed;
	hensa_speed = tar_speed-speed;
	hensa_speed_p2 = hensa_speed2;
	hensa_speed2 = tar_speed-speed2;
	hensa_ang_p = hensa_ang;
	hensa_ang = tar_ang_vel-ang_vel;

	FF_prev_R = FF_r;
	FF_prev_L = FF_l;

	FB_r = FB_l = 0.0;
	FF_r = FF_l = 0.0;
	V_r = V_l = 0.0;
	//FF
	if(FF_Enbl == 1 && (run_mode == STRAIGHT_MODE || run_mode == STRAIGHT2_MODE || run_mode == OFFSET_A_MODE || run_mode == OFFSET_B_MODE || run_mode == BACK_MODE || run_mode == TURN_MODE || run_mode == SLA_MODE || run_mode == SLA2_MODE || run_mode == SLA3_MODE)){

		if(FanVolt != 0){
			if(run_mode == STRAIGHT_MODE || run_mode == STRAIGHT2_MODE || run_mode == OFFSET_A_MODE || run_mode == OFFSET_B_MODE){
				FF_r = FF_ACCEL2R*accel + FF_SPEED2R*speed + FF_CONST2R;// + (FF_ANG_ACC2LR+FF_ANG_ACC2RR)/2.0f*ang_acc + (FF_ANG_VEL2LR+FF_ANG_VEL2RR)/2.0f*ang_vel;// + FF_ANG_ACC*ang_acc + FF_ANG_VEL*tar_ang_vel;//+ FF_OMEGA_R*ang_acc;
				FF_l = FF_ACCEL2L*accel + FF_SPEED2L*speed + FF_CONST2L;// - (FF_ANG_ACC2LL+FF_ANG_ACC2RL)/2.0f*ang_acc - (FF_ANG_VEL2LL+FF_ANG_VEL2RL)/2.0f*ang_vel;// - FF_ANG_ACC*ang_acc - FF_ANG_VEL*tar_ang_vel; //- FF_OMEGA_L*ang_acc;
			}
			else if(run_mode == SLA_MODE || run_mode == SLA2_MODE || run_mode == SLA3_MODE){
				if(tar_ang_vel > 20){
					FF_r = FF_prev_R;
					FF_l = FF_prev_L;
				}
				else{
					if(TURN_DIR == LEFT){
						//	if((ang_acc > 0 && tar_ang_vel <= max_ang_vel*0.5) || (ang_acc < 0 && tar_ang_vel >= max_ang_vel*0.5)){
						FF_r = FF_ACCEL2R*accel + FF_SPEED2R*speed + FF_CONST2R + FF_ANG_ACC2LR*ang_acc + FF_ANG_VEL2LR*ang_vel + FF_ANG_CONST2LR;//+ FF_OMEGA_R*ang_acc;
						FF_l = FF_ACCEL2L*accel + FF_SPEED2L*speed + FF_CONST2L - FF_ANG_ACC2LL*ang_acc - FF_ANG_VEL2LL*ang_vel - FF_ANG_CONST2LL; //- FF_OMEGA_L*ang_acc;
						//	}
					}
					else if(TURN_DIR == RIGHT){
						//	if((ang_acc < 0 && tar_ang_vel >= max_ang_vel*0.5) || (ang_acc > 0 && tar_ang_vel <= max_ang_vel*0.5)){
						FF_r = FF_ACCEL2R*accel + FF_SPEED2R*speed + FF_CONST2R + FF_ANG_ACC2RR*ang_acc + FF_ANG_VEL2RR*ang_vel - FF_ANG_CONST2RR;//+ FF_OMEGA_R*ang_acc;
						FF_l = FF_ACCEL2L*accel + FF_SPEED2L*speed + FF_CONST2L - FF_ANG_ACC2RL*ang_acc - FF_ANG_VEL2RL*ang_vel + FF_ANG_CONST2RL; //- FF_OMEGA_L*ang_acc;
						//	}
					}
				}
			}
			else if(run_mode == TURN_MODE){
				if(TURN_DIR == LEFT){
					FF_r = FF_ANG_ACC2RR*ang_acc + FF_ANG_VEL2RR*ang_vel + FF_ANG_CONST2RR;//+ FF_OMEGA_R*ang_acc;
					FF_l = - FF_ANG_ACC2RR*ang_acc - FF_ANG_VEL2RR*ang_vel - FF_ANG_CONST2RR; //- FF_OMEGA_L*ang_acc;
				}
				else if(TURN_DIR == RIGHT){
					FF_r = FF_ANG_ACC2RR*ang_acc + FF_ANG_VEL2RR*ang_vel - FF_ANG_CONST2RR;//+ FF_OMEGA_R*ang_acc;
					FF_l = - FF_ANG_ACC2RR*ang_acc - FF_ANG_VEL2RR*ang_vel + FF_ANG_CONST2RR; //- FF_OMEGA_L*ang_acc;
				}
			}
		}
		else{
			if(run_mode == STRAIGHT_MODE || run_mode == STRAIGHT2_MODE || run_mode == OFFSET_A_MODE || run_mode == OFFSET_B_MODE){
				FF_r = FF_ACCEL*accel + FF_SPEED*speed + FF_CONST;// + FF_ANG_ACC*ang_acc + FF_ANG_VEL*tar_ang_vel;//+ FF_OMEGA_R*ang_acc;
				FF_l = FF_ACCEL*accel + FF_SPEED*speed + FF_CONST;// - FF_ANG_ACC*ang_acc - FF_ANG_VEL*tar_ang_vel; //- FF_OMEGA_L*ang_acc;
			}
			else if(run_mode == SLA_MODE || run_mode == SLA2_MODE || run_mode == SLA3_MODE){
				if(TURN_DIR == LEFT){
				//	if((ang_acc > 0 && tar_ang_vel <= max_ang_vel*0.5) || (ang_acc < 0 && tar_ang_vel >= max_ang_vel*0.5)){
						FF_r = FF_ACCEL*accel + FF_SPEED*speed + FF_CONST + FF_ANG_ACC*ang_acc + FF_ANG_VEL*tar_ang_vel/**0.0903*exp(-0.14*fabsf(max_ang_vel))*/ + FF_ANG_CONST;//+ FF_OMEGA_R*ang_acc;
						FF_l = FF_ACCEL*accel + FF_SPEED*speed + FF_CONST - FF_ANG_ACC*ang_acc - FF_ANG_VEL*tar_ang_vel/**0.0903*exp(-0.14*fabsf(max_ang_vel))*/ - FF_ANG_CONST; //- FF_OMEGA_L*ang_acc;
					//	}
				}
				else if(TURN_DIR == RIGHT){
				//	if((ang_acc < 0 && tar_ang_vel >= max_ang_vel*0.5) || (ang_acc > 0 && tar_ang_vel <= max_ang_vel*0.5)){
						FF_r = FF_ACCEL*accel + FF_SPEED*speed + FF_CONST + FF_ANG_ACC*ang_acc + FF_ANG_VEL*tar_ang_vel/**0.0903*exp(-0.14*fabsf(max_ang_vel))*/ - FF_ANG_CONST;//+ FF_OMEGA_R*ang_acc;
						FF_l = FF_ACCEL*accel + FF_SPEED*speed + FF_CONST - FF_ANG_ACC*ang_acc - FF_ANG_VEL*tar_ang_vel/**0.0903*exp(-0.14*fabsf(max_ang_vel))*/ + FF_ANG_CONST; //- FF_OMEGA_L*ang_acc;
					//	}
				}
			}
		}

	}
	//FB
	if(FB_Enbl == 1){
		if(run_mode == STRAIGHT_MODE || run_mode == STRAIGHT2_MODE){
			if(WallCtrlEnbl == FLAG_ON){
				//速度に対するP制御
				FB_r += (float)(1.0f * (tar_speed - speed) *SPEED_KP/1.0f);
				FB_l += (float)(1.0f * (tar_speed - speed) *SPEED_KP2/1.0f);
				//速度に対するI制御
				FB_r += (float)(1.0f * (I_tar_speed - I_speed) *SPEED_KI/1.0f);
				FB_l += (float)(1.0f * (I_tar_speed - I_speed) *SPEED_KI2/1.0f);
				//速度に対するD制御
				FB_r -= (float)(1.0f * (hensa_speed_p - hensa_speed) *SPEED_KD/1.0f);
				FB_l -= (float)(1.0f * (hensa_speed_p - hensa_speed) *SPEED_KD2/1.0f);
				//角速度に対するP制御
				FB_r += (float)(1.0f * (tar_ang_vel - ang_vel) *(OMEGA_KP/100.0f));
				FB_l -= (float)(1.0f * (tar_ang_vel - ang_vel) *(OMEGA_KP2/100.0f));
				//角速度に対するI制御
				FB_r += (float)(1.0f * (I_tar_ang_vel - I_ang_vel) *(OMEGA_KI/100.0f));
				FB_l -= (float)(1.0f * (I_tar_ang_vel - I_ang_vel) *(OMEGA_KI2/100.0f));
				//角速度に対するD制御
				FB_r += (float)(1.0f * (hensa_ang - hensa_ang_p) *(OMEGA_KD/100.0f));
				FB_l -= (float)(1.0f * (hensa_ang - hensa_ang_p) *(OMEGA_KD2/100.0f));

				//角速度に対するI制御
				//FB_r += (float)(1.0f * (Tar_Angle_G - Angle_G) *(ANGLE_KP/100.0));
				//FB_l -= (float)(1.0f * (Tar_Angle_G - Angle_G) *(ANGLE_KP/100.0));
			}
			else{
				//速度に対するP制御
				FB_r += (float)(1.0f * (tar_speed - speed) *SPEED_KP/1.0f);
				FB_l += (float)(1.0f * (tar_speed - speed) *SPEED_KP2/1.0f);
				//速度に対するI制御
				FB_r += (float)(1.0f * (I_tar_speed - I_speed) *SPEED_KI/1.0f);
				FB_l += (float)(1.0f * (I_tar_speed - I_speed) *SPEED_KI2/1.0f);
				//速度に対するD制御
				FB_r -= (float)(1.0f * (hensa_speed_p - hensa_speed) *SPEED_KD/1.0f);
				FB_l -= (float)(1.0f * (hensa_speed_p - hensa_speed) *SPEED_KD2/1.0f);
				//角速度に対するP制御
				FB_r += (float)(1.0f * (tar_ang_vel - ang_vel) *(OMEGA_KP/100.0f));
				FB_l -= (float)(1.0f * (tar_ang_vel - ang_vel) *(OMEGA_KP2/100.0f));
				//角速度に対するI制御
				FB_r += (float)(1.0f * (I_tar_ang_vel - I_ang_vel) *(OMEGA_KI/100.0f));
				FB_l -= (float)(1.0f * (I_tar_ang_vel - I_ang_vel) *(OMEGA_KI2/100.0f));
				//角速度に対するD制御
				FB_r += (float)(1.0f * (hensa_ang - hensa_ang_p) *(OMEGA_KD/100.0f));
				FB_l -= (float)(1.0f * (hensa_ang - hensa_ang_p) *(OMEGA_KD2/100.0f));

				//絶対角度に対するP制御
				//FB_r += (float)(1.0 * (Tar_Angle_G - Angle_G) *(ANGLE_KP/100.0));
				//FB_l -= (float)(1.0 * (Tar_Angle_G - Angle_G) *(ANGLE_KP/100.0));
			}

			if(saitaning == 1 && slantingtask == 0 && end_speed == 0 && turnfwallflag == 0 && (len_target-len_mouse) <= 90.0){
				turnfwallflag = 1;
			}
			if(saitaning == 1 && slantingtask != 0 && end_speed == 0 && slantfwallflag == 0 && (len_target-len_mouse) <= ONESLANTING){
				slantfwallflag = 1;
			}

		}
		else if(run_mode == OFFSET_A_MODE || run_mode == OFFSET_B_MODE){
			//速度に対するP制御
			FB_r += (float)(1.0f * (tar_speed - speed) *SPEED_KP/1.0f);
			FB_l += (float)(1.0f * (tar_speed - speed) *SPEED_KP2/1.0f);
			//速度に対するI制御
			FB_r += (float)(1.0f * (I_tar_speed - I_speed) *SPEED_KI/1.0f);
			FB_l += (float)(1.0f * (I_tar_speed - I_speed) *SPEED_KI2/1.0f);
			//速度に対するD制御
			FB_r -= (float)(1.0f * (hensa_speed_p - hensa_speed) *SPEED_KD/1.0f);
			FB_l -= (float)(1.0f * (hensa_speed_p - hensa_speed) *SPEED_KD2/1.0f);
			//角速度に対するP制御
			FB_r += (float)(1.0f * (tar_ang_vel - ang_vel) *(OMEGA_KP/100.0f));
			FB_l -= (float)(1.0f * (tar_ang_vel - ang_vel) *(OMEGA_KP2/100.0f));
			//角速度に対するI制御
			FB_r += (float)(1.0f * (I_tar_ang_vel - I_ang_vel) *(OMEGA_KI/100.0f));
			FB_l -= (float)(1.0f * (I_tar_ang_vel - I_ang_vel) *(OMEGA_KI2/100.0f));
			//角速度に対するD制御
			FB_r += (float)(1.0f * (hensa_ang - hensa_ang_p) *(OMEGA_KD/100.0f));
			FB_l -= (float)(1.0f * (hensa_ang - hensa_ang_p) *(OMEGA_KD2/100.0f));

			//絶対角度に対するP制御
			//FB_r += (float)(1.0f * (Tar_Angle_G - Angle_G) *(ANGLE_KP/100.0));
			//FB_l -= (float)(1.0f * (Tar_Angle_G - Angle_G) *(ANGLE_KP/100.0));
		}
		else if(run_mode == F_WALL_MODE){
			//速度に対するP制御
			FB_r += (float)(1.0f * (tar_speed - speed) *SPEED_KP/1.0f);
			FB_l += (float)(1.0f * (tar_speed - speed) *SPEED_KP2/1.0f);
			//速度に対するI制御
			FB_r += (float)(1.0f * (I_tar_speed - I_speed) *SPEED_KI/1.0f);
			FB_l += (float)(1.0f * (I_tar_speed - I_speed) *SPEED_KI2/1.0f);
			//速度に対するD制御
			FB_r -= (float)(1.0f * (hensa_speed_p - hensa_speed) *SPEED_KD/1.0f);
			FB_l -= (float)(1.0f * (hensa_speed_p - hensa_speed) *SPEED_KD2/1.0f);
			//角速度に対するP制御
			FB_r += (float)(1.0f * (tar_ang_vel - ang_vel) *(OMEGA_KP/100.0f));
			FB_l -= (float)(1.0f * (tar_ang_vel - ang_vel) *(OMEGA_KP2/100.0f));
			//角速度に対するI制御
			FB_r += (float)(1.0f * (I_tar_ang_vel - I_ang_vel) *(OMEGA_KI/100.0f));
			FB_l -= (float)(1.0f * (I_tar_ang_vel - I_ang_vel) *(OMEGA_KI2/100.0f));
			//角速度に対するD制御
			FB_r += (float)(1.0f * (hensa_ang - hensa_ang_p) *(OMEGA_KD/100.0f));
			FB_l -= (float)(1.0f * (hensa_ang - hensa_ang_p) *(OMEGA_KD2/100.0f));

			//速度に対するP制御
			FB_r += (float)(1.0f * (tar_speed - speed2) *SPEED_KP_MINUS/1.0f);
			FB_l += (float)(1.0f * (tar_speed - speed2) *SPEED_KP_MINUS/1.0f);
			//速度に対するI制御
			FB_r += (float)(1.0f * (I_tar_speed - I_speed2) *SPEED_KI_MINUS/1.0f);
			FB_l += (float)(1.0f * (I_tar_speed - I_speed2) *SPEED_KI_MINUS/1.0f);
			//速度に対するD制御
			FB_r -= (float)(1.0f * (hensa_speed_p2 - hensa_speed2) *SPEED_KD_MINUS/1.0f);
			FB_l -= (float)(1.0f * (hensa_speed_p2 - hensa_speed2) *SPEED_KD_MINUS/1.0f);

			//角速度に対するP制御
			FB_r += (float)(1.0f * (tar_ang_vel - ang_vel) *(OMEGA_KP_MINUS/100.0f));
			FB_l -= (float)(1.0f * (tar_ang_vel - ang_vel) *(OMEGA_KP_MINUS/100.0));
			//角速度に対するI制御
			FB_r += (float)(1.0f * (I_tar_ang_vel - I_ang_vel) *(OMEGA_KI_MINUS/100.0f));
			FB_l -= (float)(1.0f * (I_tar_ang_vel - I_ang_vel) *(OMEGA_KI_MINUS/100.0f));
			//角速度に対するD制御
			FB_r += (float)(1.0f * (hensa_ang - hensa_ang_p) *(OMEGA_KD_MINUS/100.0f));
			FB_l -= (float)(1.0f * (hensa_ang - hensa_ang_p) *(OMEGA_KD_MINUS/100.0f));

			//絶対角度に対するP制御
			//FB_r += (float)(1.0 * (Tar_Angle_G - Angle_G) *(ANGLE_KP/100.0));
			//FB_l -= (float)(1.0 * (Tar_Angle_G - Angle_G) *(ANGLE_KP/100.0));
		}
		else if(run_mode == BACK_MODE){
			//速度に対するP制御
			FB_r += (float)(1.0f * (tar_speed + speed) *SPEED_KP/1.0f);
			FB_l += (float)(1.0f * (tar_speed + speed) *SPEED_KP2/1.0f);
			//速度に対するI制御
			FB_r += (float)(1.0f * (I_tar_speed + I_speed) *SPEED_KI/1.0f);
			FB_l += (float)(1.0f * (I_tar_speed + I_speed) *SPEED_KI2/1.0f);
			//速度に対するD制御
			FB_r -= (float)(1.0f * (hensa_speed_p - hensa_speed) *SPEED_KD/1.0f);
			FB_l -= (float)(1.0f * (hensa_speed_p - hensa_speed) *SPEED_KD2/1.0f);
			//角速度に対するP制御
			FB_r -= (float)(1.0f * (tar_ang_vel - ang_vel) *(OMEGA_KP/100.0f));
			FB_l += (float)(1.0f * (tar_ang_vel - ang_vel) *(OMEGA_KP2/100.0f));
			//角速度に対するI制御
			FB_r -= (float)(1.0f * (I_tar_ang_vel - I_ang_vel) *(OMEGA_KI/100.0f));
			FB_l += (float)(1.0f * (I_tar_ang_vel - I_ang_vel) *(OMEGA_KI2/100.0f));
			//角速度に対するD制御
			FB_r -= (float)(1.0f * (hensa_ang - hensa_ang_p) *(OMEGA_KD/100.0f));
			FB_l += (float)(1.0f * (hensa_ang - hensa_ang_p) *(OMEGA_KD2/100.0f));

			//絶対角度に対するP制御
			//FB_r += (float)(1.0f * (Tar_Angle_G - Angle_G) *(ANGLE_KP/100.0));
			//FB_l -= (float)(1.0f * (Tar_Angle_G - Angle_G) *(ANGLE_KP/100.0));
		}
		else if(run_mode == TURN_MODE){
			//速度に対するP制御
			FB_r += (float)(1.0f * (tar_speed - speed) *SPEED_KP/1.0f);
			FB_l += (float)(1.0f * (tar_speed - speed) *SPEED_KP2/1.0f);
			//速度に対するI制御
			FB_r += (float)(1.0f * (I_tar_speed - I_speed) *SPEED_KI/1.0f);
			FB_l += (float)(1.0f * (I_tar_speed - I_speed) *SPEED_KI2/1.0f);
			//速度に対するD制御
			FB_r -= (float)(1.0f * (hensa_speed_p - hensa_speed) *SPEED_KD/1.0f);
			FB_l -= (float)(1.0f * (hensa_speed_p - hensa_speed) *SPEED_KD2/1.0f);
			//角速度に対するP制御
			FB_r += (float)(1.0f * (tar_ang_vel - ang_vel) *(OMEGA_KP/100.0f));
			FB_l -= (float)(1.0f * (tar_ang_vel - ang_vel) *(OMEGA_KP2/100.0f));
			//角速度に対するI制御
			FB_r += (float)(1.0f * (I_tar_ang_vel - I_ang_vel) *(OMEGA_KI/100.0f));
			FB_l -= (float)(1.0f * (I_tar_ang_vel - I_ang_vel) *(OMEGA_KI2/100.0f));
			//角速度に対するD制御
			FB_r += (float)(1.0f * (hensa_ang - hensa_ang_p) *(OMEGA_KD/100.0f));
			FB_l -= (float)(1.0f * (hensa_ang - hensa_ang_p) *(OMEGA_KD2/100.0f));
			/*FB_r += (float)(1.0f * (tar_degree - (degree-start_degree)) *(DEG_KP/100.0));
			FB_l -= (float)(1.0f * (tar_degree - (degree-start_degree)) *(DEG_KP/100.0));*/

			//速度に対するP制御
			FB_r += (float)(1.0f * (tar_speed - speed2) *SPEED_KP_MINUS/1.0f);
			FB_l += (float)(1.0f * (tar_speed - speed2) *SPEED_KP_MINUS/1.0f);
			//速度に対するI制御
			FB_r += (float)(1.0f * (I_tar_speed - I_speed2) *SPEED_KI_MINUS/1.0f);
			FB_l += (float)(1.0f * (I_tar_speed - I_speed2) *SPEED_KI_MINUS/1.0f);
			//速度に対するD制御
			FB_r -= (float)(1.0f * (hensa_speed_p2 - hensa_speed2) *SPEED_KD_MINUS/1.0f);
			FB_l -= (float)(1.0f * (hensa_speed_p2 - hensa_speed2) *SPEED_KD_MINUS/1.0f);

			//角速度に対するP制御
			FB_r += (float)(1.0f * (tar_ang_vel - ang_vel) *(OMEGA_KP_MINUS/100.0f));
			FB_l -= (float)(1.0f * (tar_ang_vel - ang_vel) *(OMEGA_KP_MINUS/100.0));
			//角速度に対するI制御
			FB_r += (float)(1.0f * (I_tar_ang_vel - I_ang_vel) *(OMEGA_KI_MINUS/100.0f));
			FB_l -= (float)(1.0f * (I_tar_ang_vel - I_ang_vel) *(OMEGA_KI_MINUS/100.0f));
			//角速度に対するD制御
			FB_r += (float)(1.0f * (hensa_ang - hensa_ang_p) *(OMEGA_KD_MINUS/100.0f));
			FB_l -= (float)(1.0f * (hensa_ang - hensa_ang_p) *(OMEGA_KD_MINUS/100.0f));

			//絶対角度に対するP制御
			//FB_r += (float)(1.0f * (Tar_Angle_G - Angle_G) *(ANGLE_KP/100.0));
			//FB_l -= (float)(1.0f * (Tar_Angle_G - Angle_G) *(ANGLE_KP/100.0));
		}
		else if(run_mode == SLA_MODE || run_mode == SLA2_MODE || run_mode == SLA3_MODE){
			//速度に対するP制御
			FB_r += (float)(1.0f * (tar_speed - speed2) *SPEED_KP/1.0f);
			FB_l += (float)(1.0f * (tar_speed - speed2) *SPEED_KP2/1.0f);
			//速度に対するI制御
			FB_r += (float)(1.0f * (I_tar_speed - I_speed2) *SPEED_KI/1.0f);
			FB_l += (float)(1.0f * (I_tar_speed - I_speed2) *SPEED_KI2/1.0f);
			//速度に対するD制御
			FB_r -= (float)(1.0f * (hensa_speed_p2 - hensa_speed2) *SPEED_KD/1.0f);
			FB_l -= (float)(1.0f * (hensa_speed_p2 - hensa_speed2) *SPEED_KD2/1.0f);

			speedFB = FB_r;

			//角速度に対するP制御
			FB_r += (float)(1.0f * (tar_ang_vel - ang_vel) *(OMEGA_KP/100.0f));
			FB_l -= (float)(1.0f * (tar_ang_vel - ang_vel) *(OMEGA_KP2/100.0));
			//角速度に対するI制御
			FB_r += (float)(1.0f * (I_tar_ang_vel - I_ang_vel) *(OMEGA_KI/100.0f));
			FB_l -= (float)(1.0f * (I_tar_ang_vel - I_ang_vel) *(OMEGA_KI2/100.0f));
			//角速度に対するD制御
			FB_r += (float)(1.0f * (hensa_ang - hensa_ang_p) *(OMEGA_KD/100.0f));
			FB_l -= (float)(1.0f * (hensa_ang - hensa_ang_p) *(OMEGA_KD2/100.0f));

			if(saitaning == 1 && run_mode == SLA3_MODE){
				//絶対角度に対するP制御
				//FB_r += (float)(1.0f * (Tar_Angle_G - Angle_G) *(ANGLE_KP/100.0f));
				//FB_l -= (float)(1.0f * (Tar_Angle_G - Angle_G) *(ANGLE_KP/100.0f));


			}

			if(searching_flag == 1){
				//速度に対するP制御
				FB_r += (float)(1.0f * (tar_speed - speed2) *SPEED_KP_MINUS/1.0f);
				FB_l += (float)(1.0f * (tar_speed - speed2) *SPEED_KP_MINUS/1.0f);
				//速度に対するI制御
				FB_r += (float)(1.0f * (I_tar_speed - I_speed2) *SPEED_KI_MINUS/1.0f);
				FB_l += (float)(1.0f * (I_tar_speed - I_speed2) *SPEED_KI_MINUS/1.0f);
				//速度に対するD制御
				FB_r -= (float)(1.0f * (hensa_speed_p2 - hensa_speed2) *SPEED_KD_MINUS/1.0f);
				FB_l -= (float)(1.0f * (hensa_speed_p2 - hensa_speed2) *SPEED_KD_MINUS/1.0f);

				//角速度に対するP制御
				FB_r += (float)(1.0f * (tar_ang_vel - ang_vel) *(OMEGA_KP_MINUS/100.0f));
				FB_l -= (float)(1.0f * (tar_ang_vel - ang_vel) *(OMEGA_KP_MINUS/100.0));
				//角速度に対するI制御
				FB_r += (float)(1.0f * (I_tar_ang_vel - I_ang_vel) *(OMEGA_KI_MINUS/100.0f));
				FB_l -= (float)(1.0f * (I_tar_ang_vel - I_ang_vel) *(OMEGA_KI_MINUS/100.0f));
				//角速度に対するD制御
				FB_r += (float)(1.0f * (hensa_ang - hensa_ang_p) *(OMEGA_KD_MINUS/100.0f));
				FB_l -= (float)(1.0f * (hensa_ang - hensa_ang_p) *(OMEGA_KD_MINUS/100.0f));

			}
	//		if(saitaning == 1 && yaba == 1 && run_mode == SLA2_MODE){
		/*	I_degree += (tar_degree-(degree-start_degree));
			if(I_degree >30*10000000000){
				I_degree = 30*10000000000;
			}else if(I_degree < -1*10000000000){
				I_degree = 1*10000000000;
			}*/
			//角度に対するP制御
			//FB_r += (float)(1.0 * (tar_degree - (degree-start_degree)) *(DEG_KP_S/100.0));
			//FB_l -= (float)(1.0 * (tar_degree - (degree-start_degree)) *(DEG_KP_S/100.0));
			//角度に対するP制御
		//	FB_r += (float)(1.0 * (I_degree) *(DEG_KI_S/100.0));
		//	FB_l -= (float)(1.0 * (I_degree) *(DEG_KI_S/100.0));
	//		}
		//	if(nowG > 0.3){
			//	FB_r = FB_r*0.3f/nowG;
			//	FB_l = FB_l*0.3f/nowG;
		//	}
		}
		else if(run_mode == PARTY_MODE){
			//速度に対するP制御
			FB_r += (float)(1.0f * (tar_speed - speed) *SPEED_KP/1.0f);
			FB_l += (float)(1.0f * (tar_speed - speed) *SPEED_KP2/1.0f);
			//速度に対するI制御
			FB_r += (float)(1.0f * (I_tar_speed - I_speed) *SPEED_KI/1.0f);
			FB_l += (float)(1.0f * (I_tar_speed - I_speed) *SPEED_KI2/1.0f);
			//速度に対するD制御
			FB_r -= (float)(1.0f * (hensa_speed_p - hensa_speed) *SPEED_KD/1.0f);
			FB_l -= (float)(1.0f * (hensa_speed_p - hensa_speed) *SPEED_KD2/1.0f);
			//角速度に対するP制御
			FB_r += (float)(1.0f * (tar_ang_vel - ang_vel) *(OMEGA_KP/100.0f));
			FB_l -= (float)(1.0f * (tar_ang_vel - ang_vel) *(OMEGA_KP2/100.0f));
			//角速度に対するI制御
			FB_r += (float)(1.0f * (I_tar_ang_vel - I_ang_vel) *(OMEGA_KI/100.0f));
			FB_l -= (float)(1.0f * (I_tar_ang_vel - I_ang_vel) *(OMEGA_KI2/100.0f));
			//角速度に対するD制御
			FB_r += (float)(1.0f * (hensa_ang - hensa_ang_p) *(OMEGA_KD/100.0f));
			FB_l -= (float)(1.0f * (hensa_ang - hensa_ang_p) *(OMEGA_KD2/100.0f));
		}
		else if(run_mode == TRACELOG_MODE){
			//速度に対するP制御
			FB_r += (float)(1.0f * (tar_speed - speed) *SPEED_KP/1.0f);
			FB_l += (float)(1.0f * (tar_speed - speed) *SPEED_KP2/1.0f);
			//速度に対するI制御
			FB_r += (float)(1.0f * (I_tar_speed - I_speed) *SPEED_KI/1.0f);
			FB_l += (float)(1.0f * (I_tar_speed - I_speed) *SPEED_KI2/1.0f);
			//速度に対するD制御
			FB_r -= (float)(1.0f * (hensa_speed_p - hensa_speed) *SPEED_KD/1.0f);
			FB_l -= (float)(1.0f * (hensa_speed_p - hensa_speed) *SPEED_KD2/1.0f);
			//角速度に対するP制御
			FB_r += (float)(1.0f * (tar_ang_vel - ang_vel) *(OMEGA_KP/100.0f));
			FB_l -= (float)(1.0f * (tar_ang_vel - ang_vel) *(OMEGA_KP2/100.0f));
			//角速度に対するI制御
			FB_r += (float)(1.0f * (I_tar_ang_vel - I_ang_vel) *(OMEGA_KI/100.0f));
			FB_l -= (float)(1.0f * (I_tar_ang_vel - I_ang_vel) *(OMEGA_KI2/100.0f));
			//角速度に対するD制御
			FB_r += (float)(1.0f * (hensa_ang - hensa_ang_p) *(OMEGA_KD/100.0f));
			FB_l -= (float)(1.0f * (hensa_ang - hensa_ang_p) *(OMEGA_KD2/100.0f));
		}
		else if(run_mode == NON_CON_MODE){
		}else{
		}
	}


	if(run_mode == VOLT_MODE){
		V_r = vvv_r;
		V_l = vvv_l;
	}
	else{
		V_r = FF_r + FB_r;
		V_l = FF_l + FB_l;
	}


	if(run_mode != TEST_MODE){
		if(run_mode == BACK_MODE){
			if(V_r > 0){
				MOT_R_BACKWARD();
				//V_r = V_r;			//電圧は正なのでそのまま
			}else{
				MOT_R_FORWARD();
				V_r = -V_r;			//電圧を正の値へ反転
			}
			if(V_l > 0){
				MOT_L_BACKWARD();
				//V_l = V_l;			//電圧は正なのでそのまま
			}else{
				MOT_L_FORWARD();
				V_l = -V_l;			//電圧を正の値へ反転
			}
		}
		else{
			if(V_r > 0){
				MOT_R_FORWARD();
				//V_r = V_r;			//電圧は正なのでそのまま
			}else{
				MOT_R_BACKWARD();
				V_r = -V_r;			//電圧を正の値へ反転
			}
			if(V_l > 0){
				MOT_L_FORWARD();
				//V_l = V_l;			//電圧は正なのでそのまま
			}else{
				MOT_L_BACKWARD();
				V_l = -V_l;			//電圧を正の値へ反転
			}
		}
	}





	if(V_r > (((float)V_bat/4096.0f)*3.3f*2.0f)){
		V_r = (((float)V_bat/4096.0f)*3.3f*2.0f);
		//while(1){
			//V_r /= 2.0f;//-= (FF_r + FB_r)/100.0;
			//if(V_r <= 3.5f){
				//break;
			//}
		//}
	}
	if(V_l > (((float)V_bat/4096.0f)*3.3f*2.0f)){//3.5
		V_l = (((float)V_bat/4096.0f)*3.3f*2.0f);
		//while(1){
			//V_l /= 2.0f;//-= (FF_l + FB_l)/100.0;
			//if(V_l <= 3.5f){
			//	break;
			//}
		//}
	}

//	if(saitaning == 1 && run_mode == STRAIGHT_MODE && max_speed >= 1.5){
	//バッテリー電圧からデューティを計算
//	Duty_r = V_r/((float)((V_bat_ref/4096.0)*3.3*2.0));
//	Duty_l = V_l/((float)((V_bat_ref/4096.0)*3.3*2.0));
//	}
//	else{
	//バッテリー電圧からデューティを計算
	Duty_r = V_r/((float)(((float)V_bat/4096.0f)*3.3f*2.0f));
	Duty_l = V_l/((float)(((float)V_bat/4096.0f)*3.3f*2.0f));




	Duty_fan = FanVolt/((float)(((float)V_bat/4096.0f)*3.3f*2.0f));
//	}

	//モータにPWMを出力
	if(run_mode != TEST_MODE){
		//MOT_OUT_R =(short)((float)(240.0 * Duty_r));
		if(((short)((float)(800.0f*Duty_r))-1.0f) < 0){
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
		}
		else if(((short)((float)(800.0f*Duty_r))-1.0f) >= 799){
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 799);
		}
		else if(((short)((float)(800.0f*Duty_r))-1.0f) <= 24){
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 24);
		}
		else{
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ((short)((float)(800.0f*Duty_r))-1.0f));
		}
		//MOT_OUT_L =(short)((float)(240.0 * Duty_l));
		if(((short)((float)(800.0f*Duty_l))-1.0f) < 0){
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
		}
		else if(((short)((float)(800.0f*Duty_l))-1.0f) >= 799){
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 799);
		}
		else if(((short)((float)(800.0f*Duty_l))-1.0f) <= 24){
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 24);
		}
		else{
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, ((short)((float)(800.0f*Duty_l))-1.0f));
		}
	}

//	if(run_mode != TEST_MODE){
		if(FanVolt != 0.0 && FanEnbl == 1){
			if(((short)((float)(8000.0f*Duty_fan))-1.0f) < 0){
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
			}
			else if(((short)((float)(8000.0f*Duty_fan))-1.0f) >= 7999){
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 7999);
			}
			else{
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ((short)((float)(8000.0f*Duty_fan))-1.0f));
			}
		}
		else if(FanVolt != 0.0 && FanEnbl == 0){
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
//		}
	}

	timer++;
	if(anteiflag == 1){
		anteitimer++;
	}
	else{
		anteitimer = 0;
	}
	//if(searching_flag == 1){
	searchtimer++;
	//}
	//cnt++;
}


void wait_ms(volatile int waittime){
	timer = 0;
	while(timer<waittime);
}

