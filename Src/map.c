#include "index.h"
#include"Interface.h"
#include"common.h"
#include"CMT.h"
#include"run.h"
#include"map.h"
#include"Dataflash.h"
#include"search.h"
#include<math.h>

#define ARM_MATH_CM4
#include "arm_math.h"
#include "arm_const_structs.h"

/*	壁データはmap[][]に記録される
	壁データ記録方法
	壁の位置 0bit:上 1bit:右 2bit:下 3bit:左	 1:壁有り 0:壁無し
	壁の探索 4bit:上 5bit:右 6bit:下 7bit:左	 1:済み	 0:未探索		*/
unsigned short smap[MAZESIZE_X][MAZESIZE_Y];	//等高線map
unsigned char map[MAZESIZE_X][MAZESIZE_Y];/* = {14,14,12,5,5,5,6,12,6,12,4,6,12,6,12,6,12,4,6,12,4,6,14,12,5,4,5,4,4,5,5,6,
10,10,10,12,5,5,3,10,10,8,0,2,10,8,2,8,2,10,10,8,0,2,10,8,6,8,6,10,10,14,14,10,
10,11,10,9,5,5,6,10,10,9,1,3,10,10,8,2,8,2,10,9,1,3,10,10,9,2,9,0,0,0,0,2,
10,12,3,12,5,5,3,10,9,5,4,5,3,9,3,9,2,11,9,5,5,5,1,1,5,1,5,3,11,11,11,10,
8,1,7,8,5,5,5,2,14,12,2,12,6,12,5,5,1,6,13,4,5,4,5,4,5,5,5,6,12,4,4,2,
10,12,6,10,12,4,6,10,8,2,8,2,10,10,12,4,6,8,4,1,4,1,6,10,12,4,6,10,10,10,10,10,
10,10,10,10,8,0,2,10,10,8,2,8,2,10,8,0,2,10,9,4,1,4,3,10,8,0,2,10,10,10,10,10,
9,3,10,11,9,1,3,10,8,2,8,2,8,2,9,1,3,10,12,1,4,1,6,10,9,1,3,10,10,10,10,10,
12,5,3,12,5,5,4,3,10,9,3,9,3,9,4,5,5,3,9,5,1,4,3,8,5,5,5,1,1,1,1,2,
9,5,6,9,5,5,1,6,8,5,5,5,6,12,1,4,5,6,12,5,5,1,6,8,6,12,6,12,4,5,5,3,
12,6,10,12,5,5,5,3,10,12,4,6,10,10,14,11,14,10,10,12,4,6,8,2,8,2,8,2,10,12,4,6,
8,2,10,9,5,5,5,6,10,8,0,2,10,8,1,4,1,2,10,8,0,2,10,8,2,8,2,10,10,8,0,2,
9,3,10,12,5,5,5,3,11,9,1,3,10,10,14,11,14,10,10,9,1,3,8,2,8,2,8,2,10,9,1,3,
12,5,3,8,5,5,5,7,12,5,5,5,3,9,1,5,1,1,1,5,5,5,3,9,3,9,2,11,8,5,4,7,
9,4,7,8,5,5,5,6,9,5,5,5,4,4,5,5,5,4,4,5,4,5,6,12,5,5,1,6,9,4,1,6,
13,0,7,10,12,4,6,10,12,5,5,5,3,10,12,4,6,10,8,5,0,5,2,10,12,4,6,8,4,1,4,3,
13,0,7,10,8,0,2,10,9,5,5,5,6,10,8,0,2,10,9,4,1,4,3,10,8,0,2,10,9,4,1,6,
13,0,7,10,9,1,3,10,12,5,5,5,3,11,9,1,3,11,13,0,5,0,7,10,9,1,3,10,12,1,4,3,
12,1,7,8,5,5,5,3,8,5,5,5,7,12,5,5,5,6,13,1,5,1,5,1,5,5,5,3,9,5,1,6,
9,5,6,8,6,12,6,14,8,5,5,5,6,8,6,12,6,10,12,4,5,5,4,5,4,5,4,7,12,5,5,3,
12,6,10,10,8,2,8,2,10,12,4,6,10,10,10,10,10,10,10,8,4,6,10,12,1,4,1,6,10,12,4,6,
8,2,10,8,2,8,2,10,10,8,0,2,10,10,10,10,10,10,10,8,0,2,10,9,4,1,4,3,10,8,0,2,
9,3,10,10,9,3,9,2,10,9,1,3,10,10,9,3,11,10,10,9,1,3,10,12,1,4,1,6,10,9,1,3,
13,4,3,9,4,5,5,2,9,5,5,5,2,10,13,5,5,2,8,5,5,5,3,9,5,1,4,3,8,5,5,7,
12,1,6,12,1,5,7,10,12,5,4,5,2,10,13,5,5,2,9,5,5,5,6,12,5,5,1,6,8,6,12,6,
9,4,3,10,12,4,6,10,10,12,3,12,2,10,12,4,6,10,12,5,5,5,3,10,12,4,6,10,11,9,3,10,
12,1,6,10,8,0,2,10,8,3,12,3,10,10,8,0,2,10,9,5,5,5,6,10,8,0,2,10,12,5,5,3,
9,4,3,10,9,1,3,10,10,12,3,12,3,10,9,1,3,9,5,5,5,5,3,10,9,1,3,10,10,12,6,14,
12,1,7,10,12,5,5,3,8,3,12,3,14,9,5,5,6,12,5,5,5,5,5,1,5,5,5,3,9,3,9,2,
9,5,6,10,9,5,5,6,10,13,1,5,1,5,5,5,3,10,12,5,5,5,5,5,5,5,5,6,12,5,5,3,
12,6,10,10,12,5,5,3,10,12,4,6,12,5,5,5,5,3,10,12,4,6,12,5,5,5,5,3,10,12,4,6,
9,3,9,3,9,5,5,5,3,9,1,3,9,5,5,5,5,5,3,9,1,3,9,5,5,5,5,5,3,9,1,3
						};*/
//unsigned char mapb[MAZESIZE_X][MAZESIZE_Y];

extern volatile char BlockWall_R;
extern volatile char BlockWall_L;
extern volatile char BlockWall_F;

unsigned char allsearchflag = 0;

int mx = 0;
int my = 0;
volatile int head = 0;	//0:北 1:東 2:南 3:西
extern int saitaned;

extern unsigned volatile short Wall_R;
extern unsigned volatile short Wall_L;
extern unsigned volatile short Wall_FR;
extern unsigned volatile short Wall_FL;
//extern volatile float Sensor_R;
//extern volatile float Sensor_L;
//extern volatile float Sensor_FR;
//extern volatile float Sensor_FL;
extern volatile float Sensor_R_Dis;
extern volatile float Sensor_L_Dis;
extern volatile float Sensor_FR_Dis;
extern volatile float Sensor_FL_Dis;

extern volatile float FR_Dis_Ave;
extern volatile float FL_Dis_Ave;

extern volatile int turnfwallflag;

extern float searchmaxv;
//float searchminv = 0;
extern float searchacc;

extern float searchknownmaxv;
//float searchminv = 0;
extern float searchknownacc;

//float searchdec = 0;
//float searchturnacc = 0;
//float searchturnv = 0;
extern float searchslaacc;
extern float searchslav;
extern float searchbeforeslaoffsetr;
extern float searchafterslaoffsetr;
extern float searchbeforeslaoffsetl;
extern float searchafterslaoffsetl;
extern float searchsladegr;
extern float searchsladegl;
extern float searchslaminv;

volatile char stopped = 0;

/*extern char wallDataSave[5];
extern unsigned char dataSaveX[5];
extern unsigned char dataSaveY[5];*/

extern volatile float len_true;

volatile char spd_cnt = 0;	//何回スピードダウンしたか
volatile char set_param = 0;	//探索初期パラ

extern volatile char kaeri;

typedef struct
{
	unsigned char Qx;	//X座標
	unsigned char Qy;	//Y座標
}t_queue;

t_queue myQ[1024];
volatile unsigned short q_head = 0;
volatile unsigned short q_tail = 0;	//探索歩数マップ用Queue

void queue_push(char xp,char yp){
	if(q_tail > 1023){
		q_tail = 0;
	}
	myQ[q_tail].Qx = xp;
	myQ[q_tail].Qy = yp;
	q_tail++;
}

unsigned short queue_pop(void){
	unsigned short h = 0;
	if(q_head > 1023){
		q_head = 0;
	}
	h = q_head;
	q_head++;
	return (unsigned short)((myQ[h].Qx<<5) | myQ[h].Qy);
}

void queue_push2(unsigned char x, unsigned char y , unsigned char m){
	if(Saved.qTail > SEARCH_DATA_NUM-1){
		Saved.qTail = 0;
	}
	Saved.qX[Saved.qTail] = x;
	Saved.qY[Saved.qTail] = y;
	Saved.qMap[Saved.qTail] = m;
	Saved.qTail++;
}

short numOfWalls(short x,short y){
	short rt = (map[y][x] & 0x01) + ((map[y][x] & 0x02)>>1) + ((map[y][x] & 0x04)>>2) + ((map[y][x] & 0x08)>>3);
	return rt;
}

short closeDeadEnd(short lx, short ly){
	char cntE = 0, cntN = 0, cntW = 0, cntS = 0;

	if(lx != (MAZESIZE_X-1) && numOfWalls(lx+1,ly) == 3){//東区画に3枚壁
		if(map[ly][lx+1] == 0xbb && ly != 0 && numOfWalls(lx+1,ly-1) >= 2){
			map[ly][lx+1] = (map[ly][lx+1]&0xbb)|0x44;
			map[ly-1][lx+1] = (map[ly-1][lx+1]&0xee)|0x11;
			cntE = 1;
		}
		else if(map[ly][lx+1] == 0xdd && lx != (MAZESIZE_X-2) && numOfWalls(lx+2,ly) >= 2){
			map[ly][lx+1] = (map[ly][lx+1]&0xdd)|0x22;
			map[ly][lx+2] = (map[ly][lx+2]&0x77)|0x88;
			cntE = 1;
		}
		else if(map[ly][lx+1] == 0xee && ly != (MAZESIZE_Y-1) && numOfWalls(lx+1,ly+1) >= 2){
			map[ly][lx+1] = (map[ly][lx+1]&0xee)|0x11;
			map[ly+1][lx+1] = (map[ly+1][lx+1]&0xbb)|0x44;
			cntE = 1;
		}
	}
	if(ly != (MAZESIZE_Y-1) && numOfWalls(lx,ly+1) == 3){//北区画に3枚壁
		if(map[ly+1][lx] == 0x77 && lx != 0 && numOfWalls(lx-1,ly+1) >= 2){
			map[ly+1][lx] = (map[ly+1][lx]&0x77)|0x88;
			map[ly+1][lx-1] = (map[ly+1][lx-1]&0xdd)|0x22;
			cntN = 1;
		}
		else if(map[ly+1][lx] == 0xdd && lx != (MAZESIZE_X-1) && numOfWalls(lx+1,ly+1) >= 2){
			map[ly+1][lx] = (map[ly+1][lx]&0xdd)|0x22;
			map[ly+1][lx+1] = (map[ly+1][lx+1]&0x77)|0x88;
			cntN = 1;
		}
		else if(map[ly+1][lx] == 0xee && ly != (MAZESIZE_Y-2) && numOfWalls(lx,ly+2) >= 2){
			map[ly+1][lx] = (map[ly+1][lx]&0xee)|0x11;
			map[ly+2][lx] = (map[ly+2][lx]&0xbb)|0x44;
			cntN = 1;
		}
	}
	if(lx != 0 && numOfWalls(lx-1,ly) == 3){//西区画に3枚壁
		if(map[ly][lx-1] == 0xbb && ly != 0 && numOfWalls(lx-1,ly-1) >= 2){
			map[ly][lx-1] = (map[ly][lx-1]&0xbb)|0x44;
			map[ly-1][lx-1] = (map[ly-1][lx-1]&0xee)|0x11;
			cntW = 1;
		}
		else if(map[ly][lx-1] == 0x77 && lx != 1 && numOfWalls(lx-2,ly) >= 2){
			map[ly][lx-1] = (map[ly][lx-1]&0x77)|0x88;
			map[ly][lx-2] = (map[ly][lx-2]&0xdd)|0x22;
			cntW = 1;
		}
		else if(map[ly][lx-1] == 0xee && ly != (MAZESIZE_Y-1) && numOfWalls(lx-1,ly+1) >= 2){
			map[ly][lx-1] = (map[ly][lx-1]&0xee)|0x11;
			map[ly+1][lx-1] = (map[ly+1][lx-1]&0xbb)|0x44;
			cntW = 1;
		}
	}
	if(ly != 0 && numOfWalls(lx,ly-1) == 3){//南区画に3枚壁
		if(map[ly-1][lx] == 0x77 && lx != 0 && numOfWalls(lx-1,ly-1) >= 2){
			map[ly-1][lx] = (map[ly-1][lx]&0x77)|0x88;
			map[ly-1][lx-1] = (map[ly-1][lx-1]&0xdd)|0x22;
			cntS = 1;
		}
		else if(map[ly-1][lx] == 0xdd && lx != (MAZESIZE_X-1) && numOfWalls(lx+1,ly-1) >= 2){
			map[ly-1][lx] = (map[ly-1][lx]&0xdd)|0x22;
			map[ly-1][lx+1] = (map[ly-1][lx+1]&0x77)|0x88;
			cntS = 1;
		}
		else if(map[ly-1][lx] == 0xbb && ly != 1 && numOfWalls(lx,ly-2) >= 2){
			map[ly-1][lx] = (map[ly-1][lx]&0xbb)|0x44;
			map[ly-2][lx] = (map[ly-2][lx]&0xee)|0x11;
			cntS = 1;
		}
	}
	return (cntN+cntE*2+cntS*4+cntW*8);
}

void mapCtrlZ(unsigned char num){
	char i = 0;
	short p = 0;
	unsigned char x = 0, y = 0, m = 0;
	if(num <= SEARCH_DATA_NUM){
		for(i = 0; i < num; i++){
			p = Saved.qTail - 1 - i;
			if(p < 0){
				p = Saved.qTail - 1 - i + SEARCH_DATA_NUM;
			}
			x = Saved.qX[p];
			y = Saved.qY[p];
			m = Saved.qMap[p];
			map[y][x] = m;
			if(x != (MAZESIZE_X-1)){
				map[y][x + 1] = (map[y][x + 1] & 0x77) | 0x80 | ((m << 2) & 0x08);
			}
			if(x != 0){
				map[y][x - 1] = (map[y][x - 1] & 0xdd) | 0x20 | ((m >> 2) & 0x02);
			}
			if(y != (MAZESIZE_Y-1)){
				map[y + 1][x] = (map[y + 1][x] & 0xbb) | 0x40 | ((m << 2) & 0x04);
			}
			if(y != 0){
				map[y - 1][x] = (map[y - 1][x] & 0xee) | 0x10 | ((m >> 2) & 0x01);
			}
		}
	}
}

void make_map_known(void){
	//シミュレート用　全マップデータを既知にする
	unsigned char a = 0,b = 0;
	for(a = 0; a < 32; a++){
		for(b = 0; b < 32; b++){
			map[a][b] = (map[a][b] | 0xf0);
		}
	}
}

void recovery_from_crash(short x, short y){
	// 隣の区間のＭＡＰデータも更新する
	if (x != (MAZESIZE_X-1)){
		map[y][x + 1] = (map[y][x + 1] & 0x77) | 0x80 | ((map[y][x] << 2) & 0x08);
	}
	if (x != 0){
		map[y][x - 1] = (map[y][x - 1] & 0xdd) | 0x20 | ((map[y][x] >> 2) & 0x02);
	}
	if (y != (MAZESIZE_Y-1)){
		map[y + 1][x] = (map[y + 1][x] & 0xbb) | 0x40 | ((map[y][x] << 2) & 0x04);
	}
	if (y != 0){
		map[y - 1][x] = (map[y - 1][x] & 0xee) | 0x10 | ((map[y][x] >> 2) & 0x01);
	}
}

void goalwallset(int x,int y,int goalsize){
	if(goalsize == 1){
	}
	else if(goalsize == 4){
		map[y][x] = ((map[y][x] & 0xCC) + 0x30);
		map[y][x+1] = ((map[y][x+1] & 0x66) +0x90);
		map[y+1][x] = ((map[y+1][x] & 0x99) +0x60);
		map[y+1][x+1] = ((map[y+1][x+1] & 0x33) +0xC0);
	}
	else if(goalsize == 9){
		map[y][x] = ((map[y][x] & 0xCC) + 0x30);
		map[y][x+1] = ((map[y][x+1] & 0x44) + 0xB0);
		map[y][x+2] = ((map[y][x+2] & 0x66) + 0x90);
		map[y+1][x] = ((map[y+1][x] & 0x88) + 0x70);
		map[y+1][x+1] = ((map[y+1][x+1] & 0x00) + 0xF0);
		map[y+1][x+2] = ((map[y+1][x+2] & 0x22) + 0xD0);
		map[y+2][x] = ((map[y+2][x] & 0x99) + 0x60);
		map[y+2][x+1] = ((map[y+2][x+1] & 0x11) + 0xE0);
		map[y+2][x+2] = ((map[y+2][x+2] & 0x33) + 0xC0);
	}
	else{
	}
}

void clearmap(void){	//壁データー初期化(必ずスタート地点でやること)
	  int x = 0,y = 0,z = 0;
	  for(x = 0;x < MAZESIZE_X;x++){
		  for(y = 0;y < MAZESIZE_Y;y++){
			  z = 0;
			  if(x == 0){
				  z = z | 0x88;
			  }
			  if(x == (MAZESIZE_X-1)){
				  z = z | 0x22;
			  }
			  if(x == (COMP_SIZE_X-1)){
				  z = z | 0x22;
			  }
			  if(x == (COMP_SIZE_X)){
				  z = z | 0x88;
			  }
			  if(y == 0){
				  z = z | 0x44;
			  }
			  if(y == (MAZESIZE_Y-1)){
				  z = z | 0x11;
			  }
			  if(y == (COMP_SIZE_Y-1)){
				  z = z | 0x11;
			  }
			  if(y == (COMP_SIZE_Y)){
				  z = z | 0x44;
			  }
			  map[y][x] = z;
		  }
	  }
	  if(mx == 0 && my == 0){
	  makemap();	//スタート回りわかってるとこはもう更新
	  }
	  //スタート地点ではセンサーの状況にかかわり無く更新
}

void clearmap2(void){	//壁データー初期化(必ずスタート地点でやること)
	  int x = 0,y = 0,z = 0;
	  for(x = 0;x < MAZESIZE_X;x++){
		  for(y = 0;y < MAZESIZE_Y;y++){
			  z = 0;
			/*  if(x == 0){
				  z = z | 0x88;
			  }
			  if(x == (MAZESIZE_X-1)){
				  z = z | 0x22;
			  }
			  if(x == (COMP_SIZE_X-1)){
				  z = z | 0x22;
			  }
			  if(x == (COMP_SIZE_X)){
				  z = z | 0x88;
			  }
			  if(y == 0){
				  z = z | 0x44;
			  }
			  if(y == (MAZESIZE_Y-1)){
				  z = z | 0x11;
			  }
			  if(y == (COMP_SIZE_Y-1)){
				  z = z | 0x11;
			  }
			  if(y == (COMP_SIZE_Y)){
				  z = z | 0x44;
			  }*/
			  map[y][x] = z;
		  }
	  }
	//  if(mx == 0 && my == 0){
	 // makemap();	//スタート回りわかってるとこはもう更新
	 // }
	  //スタート地点ではセンサーの状況にかかわり無く更新
}

void mapsave(void){
	char x = 0;
	char y = 0;
	for(y = 0;y < MAZESIZE_Y;y++){
		for(x = 0;x < MAZESIZE_X;x++){
		//	mapb[y][x] = map[y][x];
    		}
	}
}

void recoverymap(void){
	char x = 0;
	char y = 0;
	for(y = 0;y < MAZESIZE_Y;y++){
		for(x = 0;x < MAZESIZE_X;x++){
			//map[y][x] = mapb[y][x];
		}
	}
}

void makemap(void){
	int wall = 0;
	short Lx = 0, Ly = 0;
	unsigned short c = 0;
	short a = 0;
	// 走行中にセンサから得た壁情報をＭＡＰデータに書きこむ
	if ((mx == 0) && (my == 0)) {
		wall = 0xfe;
	}	//スタートのときの設定eが壁の情報、fが壁を見たかどうか
	else{
		 wall = getwalldata();
	}
/*	wallDataSave[0] = wallDataSave[1];
	wallDataSave[1] = wallDataSave[2];
	wallDataSave[2] = wallDataSave[3];
	wallDataSave[3] = wallDataSave[4];
	wallDataSave[4] = map[my][mx];
	dataSaveX[0] = dataSaveX[1];
	dataSaveX[1] = dataSaveX[2];
	dataSaveX[2] = dataSaveX[3];
	dataSaveX[3] = dataSaveX[4];
	dataSaveX[4] = mx;
	dataSaveY[0] = dataSaveY[1];
	dataSaveY[1] = dataSaveY[2];
	dataSaveY[2] = dataSaveY[3];
	dataSaveY[3] = dataSaveY[4];
	dataSaveY[4] = my;*/
	queue_push2(mx,my,map[my][mx]);
	map[my][mx] = wall;
	// 隣の区間のＭＡＰデータも更新する
	if (mx != (MAZESIZE_X-1)){
		map[my][mx + 1] = (map[my][mx + 1] & 0x77) | 0x80 | ((wall << 2) & 0x08);
	}
	if (mx != 0){
		map[my][mx - 1] = (map[my][mx - 1] & 0xdd) | 0x20 | ((wall >> 2) & 0x02);
	}
	if (my != (MAZESIZE_Y-1)){
		map[my + 1][mx] = (map[my + 1][mx] & 0xbb) | 0x40 | ((wall << 2) & 0x04);
	}
	if (my != 0){
		map[my - 1][mx] = (map[my - 1][mx] & 0xee) | 0x10 | ((wall >> 2) & 0x01);
	}
	//
	if((map[my][mx]&0x33) == 0x30){
		if((map[my+1][mx+1]&0x44) == 0x40){
			if((map[my+1][mx] & 0x20) == 0x00){
				queue_push2(mx,my+1,map[my+1][mx]);
				map[my+1][mx] = (map[my+1][mx]&0xdd)+0x22;
 				map[my+1][mx+1] = (map[my+1][mx+1]&0x77)+0x88;
			}
 		}
		if((map[my+1][mx+1]&0x88) == 0x80){
			if((map[my][mx+1] & 0x10) == 0x00){
				queue_push2(mx+1,my,map[my][mx+1]);
				map[my][mx+1] = (map[my][mx+1]&0xee)+0x11;
				map[my+1][mx+1] = (map[my+1][mx+1]&0xbb)+0x44;
			}
		}
	}
	if((map[my][mx]&0x66) == 0x60){
		if((map[my-1][mx+1]&0x11) == 0x10){
			if((map[my-1][mx] & 0x20) == 0x00){
				queue_push2(mx,my-1,map[my-1][mx]);
				map[my-1][mx] = (map[my-1][mx]&0xdd)+0x22;
				map[my-1][mx+1] = (map[my-1][mx+1]&0x77)+0x88;
			}
		}
		if((map[my-1][mx+1]&0x88) == 0x80){
			if((map[my-1][mx+1] & 0x10) == 0x00){
				queue_push2(mx+1,my-1,map[my-1][mx+1]);
				map[my-1][mx+1] = (map[my-1][mx+1]&0xee)+0x11;
				map[my][mx+1] = (map[my][mx+1]&0xbb)+0x44;
			}
		}
	}
	if((map[my][mx]&0xcc) == 0xc0){
		if((map[my-1][mx-1]&0x11) == 0x10){
			if((map[my-1][mx] & 0x80) == 0x00){
				queue_push2(mx,my-1,map[my-1][mx]);
				map[my-1][mx] = (map[my-1][mx]&0x77)+0x88;
				map[my-1][mx-1] = (map[my-1][mx-1]&0xdd)+0x22;
			}
		}
		if((map[my-1][mx-1]&0x22) == 0x20){
			if((map[my][mx-1] & 0x40) == 0x00){
				queue_push2(mx-1,my,map[my][mx-1]);
				map[my][mx-1] = (map[my][mx-1]&0xbb)+0x44;
				map[my-1][mx-1] = (map[my-1][mx-1]&0xee)+0x11;
			}
		}
	}
	if((map[my][mx]&0x99) == 0x90){
		if((map[my+1][mx-1]&0x22) == 0x20){
			if((map[my][mx-1] & 0x10) == 0x00){
				queue_push2(mx-1,my,map[my][mx-1]);
				map[my][mx-1] = (map[my][mx-1]&0xee)+0x11;
				map[my+1][mx-1] = (map[my+1][mx-1]&0xbb)+0x44;
			}
		}
		if((map[my+1][mx-1]&0x44) == 0x40){
			if((map[my+1][mx] & 0x80) == 0x00){
				queue_push2(mx,my+1,map[my+1][mx]);
				map[my+1][mx] = (map[my+1][mx]&0x77)+0x88;
				map[my+1][mx-1] = (map[my+1][mx-1]&0xdd)+0x22;
			}
		}
	}

	/*q_head = 0;
	q_tail = 0;
	queue_push(mx,my);
	do{
		c = queue_pop();
		Lx = (c & 992)/32;
		Ly = (c & 31);
		a = closeDeadEnd(Lx,Ly);
		if((a & 0x01) == 0x01){
			queue_push(Lx,Ly+1);
		}
		if((a & 0x02) == 0x02){
			queue_push(Lx+1,Ly);
		}
		if((a & 0x04) == 0x04){
			queue_push(Lx,Ly-1);
		}
		if((a & 0x08) == 0x08){
			queue_push(Lx-1,Ly);
		}
	}while(q_head != q_tail);*/
}

int getwalldata(void){
	int wall = 0;
	volatile char kyoro = 0;
	volatile char kyoror = 0,kyorol = 0;
	// センサデータの入力し閾値と比較し壁の有無を判定する
	wall = 0;
	BlockWall_F = 0;
	BlockWall_R = 0;
	BlockWall_L = 0;
	if(Wall_FR == WALL_OFF && Wall_FL == WALL_OFF){
		if((head == 0 && (map[my][mx]&0x11) == 0x11) || (head == 1 && (map[my][mx]&0x22) == 0x22) || (head == 2 && (map[my][mx]&0x44) == 0x44)|| (head == 3 && (map[my][mx]&0x88) == 0x88)){
			if (Wall_L == WALL_ON){
				if(head == 0){
					if((map[my][mx]&0x88) == 0x80){
						kyorol = 1;
					}
				}
				else if(head == 1){
					if((map[my][mx]&0x11) == 0x10){
						kyorol = 1;
					}
				}
				else if(head == 2){
					if((map[my][mx]&0x22) == 0x20){
						kyorol = 1;
					}
				}
				else if(head == 3){
					if((map[my][mx]&0x44) == 0x40){
						kyorol = 1;
					}
				}
				if(kyorol != 1){
				wall = wall | 0x88;
				BlockWall_L = 1;
				}
			}else{
				if(head == 0){
					if((map[my][mx]&0x88) == 0x88){
						kyorol = 1;
					}
				}
				else if(head == 1){
					if((map[my][mx]&0x11) == 0x11){
						kyorol = 1;
					}
				}
				else if(head == 2){
					if((map[my][mx]&0x22) == 0x22){
						kyorol = 1;
					}
				}
				else if(head == 3){
					if((map[my][mx]&0x44) == 0x44){
						kyorol = 1;
					}
				}
			}
			if (Wall_R == WALL_ON){
				if(head == 0){
					if((map[my][mx]&0x22) == 0x20){
						kyoror = 1;
					}
				}
				else if(head == 1){
					if((map[my][mx]&0x44) == 0x40){
						kyoror = 1;
					}
				}
				else if(head == 2){
					if((map[my][mx]&0x88) == 0x80){
						kyoror = 1;
					}
				}
				else if(head == 3){
					if((map[my][mx]&0x11) == 0x10){
						kyoror = 1;
					}
				}
				if(kyoror != 1){
				wall = wall | 0x22;
				BlockWall_R = 1;
				}
			}
			else{
				if(head == 0){
					if((map[my][mx]&0x22) == 0x22){
						kyoror = 1;
					}
				}
				else if(head == 1){
					if((map[my][mx]&0x44) == 0x44){
						kyoror = 1;
					}
				}
				else if(head == 2){
					if((map[my][mx]&0x88) == 0x88){
						kyoror = 1;
					}
				}
				else if(head == 3){
					if((map[my][mx]&0x11) == 0x11){
						kyoror = 1;
					}
				}
			}
			turnfwallflag = 1;
			straight(HALF_SECTION,searchacc,searchmaxv,0,FLAG_ON,0);			//まず、半区画進む
			turnfwallflag = 0;
			f_wall(FWALL_REF_R,FWALL_REF_L);
			if(Sensor_FR_Dis <= 90.0 && Sensor_FL_Dis <= 90.0){
				wall = wall | 0x11;
				BlockWall_F = 1;
			}
			stopped = 1;
			kyoro = 0;
		}
		else{
			stopped = 0;
			kyoro = 0;
			BlockWall_F = 0;
			if(len_true >= (LEN_DOWN*1000*spd_cnt) && (set_param-spd_cnt) >= 0){
				if (Wall_L == WALL_ON){
					if(head == 0){
						if((map[my][mx]&0x88) == 0x80){
							kyorol = 1;
						}
					}
					else if(head == 1){
						if((map[my][mx]&0x11) == 0x10){
							kyorol = 1;
						}
					}
					else if(head == 2){
						if((map[my][mx]&0x22) == 0x20){
							kyorol = 1;
						}
					}
					else if(head == 3){
						if((map[my][mx]&0x44) == 0x40){
							kyorol = 1;
						}
					}
					if(kyorol != 1){
					wall = wall | 0x88;
					BlockWall_L = 1;
					}
				}else{
					if(head == 0){
						if((map[my][mx]&0x88) == 0x88){
							kyorol = 1;
						}
					}
					else if(head == 1){
						if((map[my][mx]&0x11) == 0x11){
							kyorol = 1;
						}
					}
					else if(head == 2){
						if((map[my][mx]&0x22) == 0x22){
							kyorol = 1;
						}
					}
					else if(head == 3){
						if((map[my][mx]&0x44) == 0x44){
							kyorol = 1;
						}
					}
				}
				if (Wall_R == WALL_ON){
					if(head == 0){
						if((map[my][mx]&0x22) == 0x20){
							kyoror = 1;
						}
					}
					else if(head == 1){
						if((map[my][mx]&0x44) == 0x40){
							kyoror = 1;
						}
					}
					else if(head == 2){
						if((map[my][mx]&0x88) == 0x80){
							kyoror = 1;
						}
					}
					else if(head == 3){
						if((map[my][mx]&0x11) == 0x10){
							kyoror = 1;
						}
					}
					if(kyoror != 1){
					wall = wall | 0x22;
					BlockWall_R = 1;
					}
				}
				else{
					if(head == 0){
						if((map[my][mx]&0x22) == 0x22){
							kyoror = 1;
						}
					}
					else if(head == 1){
						if((map[my][mx]&0x44) == 0x44){
							kyoror = 1;
						}
					}
					else if(head == 2){
						if((map[my][mx]&0x88) == 0x88){
							kyoror = 1;
						}
					}
					else if(head == 3){
						if((map[my][mx]&0x11) == 0x11){
							kyoror = 1;
						}
					}
				}
				turnfwallflag = 1;
				straight(HALF_SECTION,searchacc,searchmaxv,0,FLAG_ON,0);			//まず、半区画進む
				turnfwallflag = 0;
				f_wall(FWALL_REF_R,FWALL_REF_L);
				setpara_search(set_param-spd_cnt);
				spd_cnt++;
				stopped = 1;
				kyoro = 0;
			}
		}
	}
	else if (Wall_FR == WALL_ON && Wall_FL == WALL_ON && ((FR_Dis_Ave + FL_Dis_Ave)/2.0) >= FWALL_CLOSE2 && !((FR_Dis_Ave + FL_Dis_Ave) <= 180.0 && fabsf(FR_Dis_Ave-FL_Dis_Ave) > FWALL_DIFF_BIG)){
		if((head == 0 && (map[my][mx]&0x11) == 0x10) || (head == 1 && (map[my][mx]&0x22) == 0x20) || (head == 2 && (map[my][mx]&0x44) == 0x40)|| (head == 3 && (map[my][mx]&0x88) == 0x80)){
			if (Wall_L == WALL_ON){
				if(head == 0){
					if((map[my][mx]&0x88) == 0x80){
						kyorol = 1;
					}
				}
				else if(head == 1){
					if((map[my][mx]&0x11) == 0x10){
						kyorol = 1;
					}
				}
				else if(head == 2){
					if((map[my][mx]&0x22) == 0x20){
						kyorol = 1;
					}
				}
				else if(head == 3){
					if((map[my][mx]&0x44) == 0x40){
						kyorol = 1;
					}
				}
				if(kyorol != 1){
				wall = wall | 0x88;
				BlockWall_L = 1;
				}
			}else{
				if(head == 0){
					if((map[my][mx]&0x88) == 0x88){
						kyorol = 1;
					}
				}
				else if(head == 1){
					if((map[my][mx]&0x11) == 0x11){
						kyorol = 1;
					}
				}
				else if(head == 2){
					if((map[my][mx]&0x22) == 0x22){
						kyorol = 1;
					}
				}
				else if(head == 3){
					if((map[my][mx]&0x44) == 0x44){
						kyorol = 1;
					}
				}
			}
			if (Wall_R == WALL_ON){
				if(head == 0){
					if((map[my][mx]&0x22) == 0x20){
						kyoror = 1;
					}
				}
				else if(head == 1){
					if((map[my][mx]&0x44) == 0x40){
						kyoror = 1;
					}
				}
				else if(head == 2){
					if((map[my][mx]&0x88) == 0x80){
						kyoror = 1;
					}
				}
				else if(head == 3){
					if((map[my][mx]&0x11) == 0x10){
						kyoror = 1;
					}
				}
				if(kyoror != 1){
				wall = wall | 0x22;
				BlockWall_R = 1;
				}
			}
			else{
				if(head == 0){
					if((map[my][mx]&0x22) == 0x22){
						kyoror = 1;
					}
				}
				else if(head == 1){
					if((map[my][mx]&0x44) == 0x44){
						kyoror = 1;
					}
				}
				else if(head == 2){
					if((map[my][mx]&0x88) == 0x88){
						kyoror = 1;
					}
				}
				else if(head == 3){
					if((map[my][mx]&0x11) == 0x11){
						kyoror = 1;
					}
				}
			}
			turnfwallflag = 1;
			straight(HALF_SECTION,searchacc,searchmaxv,0,FLAG_ON,0);			//まず、半区画進む
			turnfwallflag = 0;
			f_wall(FWALL_REF_R,FWALL_REF_L);
			if(Sensor_FR_Dis <= 90.0 && Sensor_FL_Dis <= 90.0){
				wall = wall | 0x11;
				BlockWall_F = 1;
			}
			stopped = 1;
			kyoro = 0;
		}
		else{
			wall = wall | 0x11;
			BlockWall_F = 1;
			stopped = 0;
			kyoro = 0;
			if(len_true >= (LEN_DOWN*1000*spd_cnt) && (set_param-spd_cnt) >= 0){
				if (Wall_L == WALL_ON){
					if(head == 0){
						if((map[my][mx]&0x88) == 0x80){
							kyorol = 1;
						}
					}
					else if(head == 1){
						if((map[my][mx]&0x11) == 0x10){
							kyorol = 1;
						}
					}
					else if(head == 2){
						if((map[my][mx]&0x22) == 0x20){
							kyorol = 1;
						}
					}
					else if(head == 3){
						if((map[my][mx]&0x44) == 0x40){
							kyorol = 1;
						}
					}
					if(kyorol != 1){
						wall = wall | 0x88;
						BlockWall_L = 1;
					}
				}else{
					if(head == 0){
						if((map[my][mx]&0x88) == 0x88){
							kyorol = 1;
						}
					}
					else if(head == 1){
						if((map[my][mx]&0x11) == 0x11){
							kyorol = 1;
						}
					}
					else if(head == 2){
						if((map[my][mx]&0x22) == 0x22){
							kyorol = 1;
						}
					}
					else if(head == 3){
						if((map[my][mx]&0x44) == 0x44){
							kyorol = 1;
						}
					}
				}
				if (Wall_R == WALL_ON){
					if(head == 0){
						if((map[my][mx]&0x22) == 0x20){
							kyoror = 1;
						}
					}
					else if(head == 1){
						if((map[my][mx]&0x44) == 0x40){
							kyoror = 1;
						}
					}
					else if(head == 2){
						if((map[my][mx]&0x88) == 0x80){
							kyoror = 1;
						}
					}
					else if(head == 3){
						if((map[my][mx]&0x11) == 0x10){
							kyoror = 1;
						}
					}
					if(kyoror != 1){
					wall = wall | 0x22;
					BlockWall_R = 1;
					}
				}
				else{
					if(head == 0){
						if((map[my][mx]&0x22) == 0x22){
							kyoror = 1;
						}
					}
					else if(head == 1){
						if((map[my][mx]&0x44) == 0x44){
							kyoror = 1;
						}
					}
					else if(head == 2){
						if((map[my][mx]&0x88) == 0x88){
							kyoror = 1;
						}
					}
					else if(head == 3){
						if((map[my][mx]&0x11) == 0x11){
							kyoror = 1;
						}
					}
				}
				turnfwallflag = 1;
				straight(HALF_SECTION,searchacc,searchmaxv,0,FLAG_ON,0);			//まず、半区画進む
				turnfwallflag = 0;
				f_wall(FWALL_REF_R,FWALL_REF_L);
				setpara_search(set_param-spd_cnt);
				spd_cnt++;
				stopped = 1;
				kyoro = 0;
			}
		}
	}
	else if((Wall_FR == WALL_ON && Wall_FL == WALL_OFF) || (Wall_FR == WALL_OFF && Wall_FL == WALL_ON) || ((FR_Dis_Ave + FL_Dis_Ave) <= 180.0 && fabsf(FR_Dis_Ave-FL_Dis_Ave) > FWALL_DIFF_BIG)){
		turnfwallflag = 1;
		//LED5 = 1;
		straight(HALF_SECTION,searchacc,searchmaxv,0,FLAG_ON,0);			//まず、半区画進む
		//LED5 = 0;
		turnfwallflag = 0;
		f_wall(FWALL_REF_R,FWALL_REF_L);
		if(Sensor_FR_Dis <= 90.0 && Sensor_FL_Dis <= 90.0){
			wall = wall | 0x11;
			BlockWall_F = 1;
		}
		stopped = 1;
		kyoro = 1;
		if(len_true >= (LEN_DOWN*1000*spd_cnt) && (set_param-spd_cnt) >= 0){
			setpara_search(set_param-spd_cnt);
			spd_cnt++;
			stopped = 1;
			kyoro = 1;
		}
	}
	else if(((FR_Dis_Ave + FL_Dis_Ave)/2.0) < FWALL_CLOSE2){
		if (Wall_L == WALL_ON){
			if(head == 0){
				if((map[my][mx]&0x88) == 0x80){
					kyorol = 1;
				}
			}
			else if(head == 1){
				if((map[my][mx]&0x11) == 0x10){
					kyorol = 1;
				}
			}
			else if(head == 2){
				if((map[my][mx]&0x22) == 0x20){
					kyorol = 1;
				}
			}
			else if(head == 3){
				if((map[my][mx]&0x44) == 0x40){
					kyorol = 1;
				}
			}
			if(kyorol != 1){
			wall = wall | 0x88;
			BlockWall_L = 1;
			}
		}else{
			if(head == 0){
				if((map[my][mx]&0x88) == 0x88){
					kyorol = 1;
				}
			}
			else if(head == 1){
				if((map[my][mx]&0x11) == 0x11){
					kyorol = 1;
				}
			}
			else if(head == 2){
				if((map[my][mx]&0x22) == 0x22){
					kyorol = 1;
				}
			}
			else if(head == 3){
				if((map[my][mx]&0x44) == 0x44){
					kyorol = 1;
				}
			}
		}
		if (Wall_R == WALL_ON){
			if(head == 0){
				if((map[my][mx]&0x22) == 0x20){
					kyoror = 1;
				}
			}
			else if(head == 1){
				if((map[my][mx]&0x44) == 0x40){
					kyoror = 1;
				}
			}
			else if(head == 2){
				if((map[my][mx]&0x88) == 0x80){
					kyoror = 1;
				}
			}
			else if(head == 3){
				if((map[my][mx]&0x11) == 0x10){
					kyoror = 1;
				}
			}
			if(kyoror != 1){
			wall = wall | 0x22;
			BlockWall_R = 1;
			}
		}
		else{
			if(head == 0){
				if((map[my][mx]&0x22) == 0x22){
					kyoror = 1;
				}
			}
			else if(head == 1){
				if((map[my][mx]&0x44) == 0x44){
					kyoror = 1;
				}
			}
			else if(head == 2){
				if((map[my][mx]&0x88) == 0x88){
					kyoror = 1;
				}
			}
			else if(head == 3){
				if((map[my][mx]&0x11) == 0x11){
					kyoror = 1;
				}
			}
		}
		turnfwallflag = 1;
		//LED5 = 1;
		straight(HALF_SECTION,searchacc,searchmaxv,0,FLAG_ON,0);			//まず、半区画進む
		//LED5 = 0;
		turnfwallflag = 0;
		f_wall(FWALL_REF_R,FWALL_REF_L);
		if(Sensor_FR_Dis <= 90.0 && Sensor_FL_Dis <= 90.0){
			wall = wall | 0x11;
			BlockWall_F = 1;
		}
		stopped = 1;
		kyoro = 0;
		if(len_true >= (LEN_DOWN*1000*spd_cnt) && (set_param-spd_cnt) >= 0){
			setpara_search(set_param-spd_cnt);
			spd_cnt++;
			stopped = 1;
			kyoro = 0;
		}
	}
	else{
		stopped = 0;
		kyoro = 0;
		if(len_true >= (LEN_DOWN*1000*spd_cnt) && (set_param-spd_cnt) >= 0){
			if (Wall_L == WALL_ON){
				if(head == 0){
					if((map[my][mx]&0x88) == 0x80){
						kyorol = 1;
					}
				}
				else if(head == 1){
					if((map[my][mx]&0x11) == 0x10){
						kyorol = 1;
					}
				}
				else if(head == 2){
					if((map[my][mx]&0x22) == 0x20){
						kyorol = 1;
					}
				}
				else if(head == 3){
					if((map[my][mx]&0x44) == 0x40){
						kyorol = 1;
					}
				}
				if(kyorol != 1){
				wall = wall | 0x88;
				BlockWall_L = 1;
				}
			}else{
				if(head == 0){
					if((map[my][mx]&0x88) == 0x88){
						kyorol = 1;
					}
				}
				else if(head == 1){
					if((map[my][mx]&0x11) == 0x11){
						kyorol = 1;
					}
				}
				else if(head == 2){
					if((map[my][mx]&0x22) == 0x22){
						kyorol = 1;
					}
				}
				else if(head == 3){
					if((map[my][mx]&0x44) == 0x44){
						kyorol = 1;
					}
				}
			}
			if (Wall_R == WALL_ON){
				if(head == 0){
					if((map[my][mx]&0x22) == 0x20){
						kyoror = 1;
					}
				}
				else if(head == 1){
					if((map[my][mx]&0x44) == 0x40){
						kyoror = 1;
					}
				}
				else if(head == 2){
					if((map[my][mx]&0x88) == 0x80){
						kyoror = 1;
					}
				}
				else if(head == 3){
					if((map[my][mx]&0x11) == 0x10){
						kyoror = 1;
					}
				}
				if(kyoror != 1){
				wall = wall | 0x22;
				BlockWall_R = 1;
				}
			}
			else{
				if(head == 0){
					if((map[my][mx]&0x22) == 0x22){
						kyoror = 1;
					}
				}
				else if(head == 1){
					if((map[my][mx]&0x44) == 0x44){
						kyoror = 1;
					}
				}
				else if(head == 2){
					if((map[my][mx]&0x88) == 0x88){
						kyoror = 1;
					}
				}
				else if(head == 3){
					if((map[my][mx]&0x11) == 0x11){
						kyoror = 1;
					}
				}
			}
			turnfwallflag = 1;
			straight(HALF_SECTION,searchacc,searchmaxv,0,FLAG_ON,0);			//まず、半区画進む
			turnfwallflag = 0;
			f_wall(FWALL_REF_R,FWALL_REF_L);
			setpara_search(set_param-spd_cnt);
			spd_cnt++;
			stopped = 1;
			kyoro = 0;
		}
	}
/*	else if((Wall_FR == WALL_ON && Wall_FL == WALL_OFF) || (Wall_FR == WALL_OFF && Wall_FL == WALL_ON)){
		turnfwallflag = 1;
		straight(HALF_SECTION,searchacc,searchmaxv,0,FLAG_ON,0);			//まず、半区画進む
		if(Wall_FR == WALL_ON && Wall_FL == WALL_ON){
			wall = wall | 0x11;
			BlockWall_F = 1;
		}
		stopped = 1;
	}*/
	if(stopped == 0 && kyoro == 0){
		if (Wall_L == WALL_ON){
			if(head == 0){
				if((map[my][mx]&0x88) == 0x80){
					kyorol = 1;
				}
			}
			else if(head == 1){
				if((map[my][mx]&0x11) == 0x10){
					kyorol = 1;
				}
			}
			else if(head == 2){
				if((map[my][mx]&0x22) == 0x20){
					kyorol = 1;
				}
			}
			else if(head == 3){
				if((map[my][mx]&0x44) == 0x40){
					kyorol = 1;
				}
			}
			if(kyorol != 1){
			wall = wall | 0x88;
			BlockWall_L = 1;
			}
		}else{
			if(head == 0){
				if((map[my][mx]&0x88) == 0x88){
					kyorol = 1;
				}
			}
			else if(head == 1){
				if((map[my][mx]&0x11) == 0x11){
					kyorol = 1;
				}
			}
			else if(head == 2){
				if((map[my][mx]&0x22) == 0x22){
					kyorol = 1;
				}
			}
			else if(head == 3){
				if((map[my][mx]&0x44) == 0x44){
					kyorol = 1;
				}
			}
		}
		if (Wall_R == WALL_ON){
			if(head == 0){
				if((map[my][mx]&0x22) == 0x20){
					kyoror = 1;
				}
			}
			else if(head == 1){
				if((map[my][mx]&0x44) == 0x40){
					kyoror = 1;
				}
			}
			else if(head == 2){
				if((map[my][mx]&0x88) == 0x80){
					kyoror = 1;
				}
			}
			else if(head == 3){
				if((map[my][mx]&0x11) == 0x10){
					kyoror = 1;
				}
			}
			if(kyoror != 1){
			wall = wall | 0x22;
			BlockWall_R = 1;
			}
		}
		else{
			if(head == 0){
				if((map[my][mx]&0x22) == 0x22){
					kyoror = 1;
				}
			}
			else if(head == 1){
				if((map[my][mx]&0x44) == 0x44){
					kyoror = 1;
				}
			}
			else if(head == 2){
				if((map[my][mx]&0x88) == 0x88){
					kyoror = 1;
				}
			}
			else if(head == 3){
				if((map[my][mx]&0x11) == 0x11){
					kyoror = 1;
				}
			}
		}
	}
	if(kyoro == 1 || kyoror == 1 || kyorol == 1){
		stopped = 1;
		wall = 0;
		BlockWall_F = 0;
		BlockWall_R = 0;
		BlockWall_L = 0;
		HAL_Delay(20);
		f_wall(FWALL_REF_R,FWALL_REF_L);
		if(Sensor_FR_Dis <= 90.0 && Sensor_FL_Dis <= 90.0){
			wall = wall | 0x11;
			BlockWall_F = 1;
		}
		HAL_Delay(10);
		turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
		f_wall(FWALL_REF_R,FWALL_REF_L);
		if(Sensor_FR_Dis <= 90.0 && Sensor_FL_Dis <= 90.0){
			wall = wall | 0x88;
			BlockWall_L = 1;
		}
		HAL_Delay(10);
		turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
		HAL_Delay(10);
		turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
		f_wall(FWALL_REF_R,FWALL_REF_L);
		if(Sensor_FR_Dis <= 90.0 && Sensor_FL_Dis <= 90.0){
			wall = wall | 0x22;
			BlockWall_R = 1;
		}
		HAL_Delay(10);
		turn(90,TURN_ACCEL,TURN_SPEED,LEFT);
	}
	//
	if(head == 0){
		if(my == MAZESIZE_Y-1 || my == COMP_SIZE_Y-1){
			wall = wall | 0x11;
			BlockWall_F = 1;
		}
		if(mx == MAZESIZE_X-1 || mx == COMP_SIZE_X-1){
			wall = wall | 0x22;
			BlockWall_R = 1;
		}
		if(mx == 0){
			wall = wall | 0x88;
			BlockWall_L = 1;
		}
	}
	else if(head == 1){
		if(mx == MAZESIZE_X-1 || mx == COMP_SIZE_X-1){
			wall = wall | 0x11;
			BlockWall_F = 1;
		}
		if(my == 0){
			wall = wall | 0x22;
			BlockWall_R = 1;
		}
		if(my == MAZESIZE_Y-1 || my == COMP_SIZE_Y-1){
			wall = wall | 0x88;
			BlockWall_L = 1;
		}
	}
	else if(head == 2){
		if(my == 0){
			wall = wall | 0x11;
			BlockWall_F = 1;
		}
		if(mx == 0){
			wall = wall | 0x22;
			BlockWall_R = 1;
		}
		if(mx == MAZESIZE_X-1 || mx == COMP_SIZE_X-1){
			wall = wall | 0x88;
			BlockWall_L = 1;
		}
	}
	else if(head == 3){
		if(mx == 0){
			wall = wall | 0x11;
			BlockWall_F = 1;
		}
		if(my == MAZESIZE_Y-1 || my == COMP_SIZE_Y-1){
			wall = wall | 0x22;
			BlockWall_R = 1;
		}
		if(my == 0){
			wall = wall | 0x88;
			BlockWall_L = 1;
		}
	}
	// マウスの進行方向にあわせてセンサデータを移動し壁データとする
	if(head == 1){
		wall = wall >> 3;
	}
	else if (head == 2){
		wall = wall >> 2;
	}
	else if (head == 3){
		wall = wall >> 1;
	}
	// 探索済みフラグを立てる
	return(wall | 0xf0);
}

void makesmap(int gx,int gy,int smode){	//等高線マップつくり
 	 //mode0なら普通 1なら最短
	unsigned short	 /*pt0 = 0,*/pt1 = 0;//,ct = 0;
	unsigned char	 finish = 0;
	short	 x = 0,y = 0,z = 0;
	short 	 xx = 0,yy = 0;
	unsigned char	 wdata = 0;
	unsigned short c = 0;
	q_head = 0;
	q_tail = 0;

	LED4_ON();

	for (z = 0;z < (MAZESIZE_X*MAZESIZE_Y);z++){		/* 等高線マップを初期化する				*/
		smap[z/MAZESIZE_Y][z & (MAZESIZE_X-1)] = STEPMAX;
	}

	if(allsearchflag == 1 && smode == 0){
		for(yy = 0; yy < MAZESIZE_Y; yy++){
			for(xx = 0; xx < MAZESIZE_X; xx++){
				if((map[yy][xx] & 0xf0) != 0xf0){
					smap[yy][xx] = 0;
					queue_push(xx,yy);
				}
				else{
				}
			}
		}
	}
	else{
		if(GOALSIZE == 1 || kaeri == 1){
			smap[gy][gx] = 0;			/* 目標地点に距離0を書き込む*/
			queue_push(gx,gy);
		}
		else if(GOALSIZE == 4){
			smap[gy][gx] = 0;			/* 目標地点に距離0を書き込む*/
			queue_push(gx,gy);
			smap[gy+1][gx] = 0;
			queue_push(gx,gy+1);
			smap[gy][gx+1] = 0;
			queue_push(gx+1,gy);
			smap[gy+1][gx+1] = 0;
			queue_push(gx+1,gy+1);
		}
		else if(GOALSIZE == 9){
			smap[gy][gx] = 0;			/* 目標地点に距離0を書き込む*/
			queue_push(gx,gy);
			smap[gy+1][gx] = 0;
			queue_push(gx,gy+1);
			smap[gy+2][gx] = 0;
			queue_push(gx,gy+2);
			smap[gy][gx+1] = 0;
			queue_push(gx+1,gy);
			smap[gy+1][gx+1] = 0;
			queue_push(gx+1,gy+1);
			smap[gy+2][gx+1] = 0;
			queue_push(gx+1,gy+2);
			smap[gy][gx+2] = 0;
			queue_push(gx+2,gy);
			smap[gy+1][gx+2] = 0;
			queue_push(gx+2,gy+1);
			smap[gy+2][gx+2] = 0;
			queue_push(gx+2,gy+2);
		}
	}

//	pt0 = 0;
	while(q_head != q_tail && finish == 0){
					c = queue_pop();
					x = (c & 992)/32;
					y = (c & 31);
					wdata = map[y][x];
					pt1 = smap[y][x]+1;
					if (smode == 1){	//未探索区間は壁あり
						finish = 0;
						if (((wdata & 0x11) == 0x10) && (y != (MAZESIZE_Y-1))){//
							if(smap[y+1][x] == STEPMAX){
								smap[y+1][x] = pt1;
								queue_push(x,y+1);
					//			ct++;
							}
						}
						if (((wdata & 0x22) == 0x20) && (x != (MAZESIZE_X-1))){
							if(smap[y][x+1] == STEPMAX){
								smap[y][x+1] = pt1;
								queue_push(x+1,y);
					//			ct++;
							}
						}
						if (((wdata & 0x44) == 0x40) && (y != 0)){
							if(smap[y-1][x] == STEPMAX){
								smap[y-1][x] = pt1;
								queue_push(x,y-1);
					//			ct++;
							}
						}
						if (((wdata & 0x88) == 0x80) && (x != 0)){
							if(smap[y][x-1] == STEPMAX){
								smap[y][x-1]  = pt1;
								queue_push(x-1,y);
					//			ct++;
							}
						}
					}
					else{	//未探索区間は壁なし(通常)
						if ((wdata & 1) == 0  &&  y != (MAZESIZE_Y-1)){
							if(smap[y+1][x] == STEPMAX){
								smap[y+1][x] = pt1;
								queue_push(x,y+1);
								if(y+1 == my && x == mx){
									finish = 1;
								}
						//		ct++;
							}
						}
						if ((wdata & 2) == 0  &&  x != (MAZESIZE_X-1)){
							if(smap[y][x+1] == STEPMAX){
								smap[y][x+1] = pt1;
								queue_push(x+1,y);
								if(y == my && x+1 == mx){
									finish = 1;
								}
						//		ct++;
							}
						}
						if ((wdata & 4) == 0  &&  y != 0){
							if(smap[y-1][x] == STEPMAX){
								smap[y-1][x] = pt1;
								queue_push(x,y-1);
								if(y-1 == my && x == mx){
									finish = 1;
								}
						//		ct++;
							}
						}
						if ((wdata & 8) == 0  &&  x != 0){
							if(smap[y][x-1] == STEPMAX){
								smap[y][x-1] = pt1;
								queue_push(x-1,y);
								if(y == my && x-1 == mx){
									finish = 1;
								}
						//		ct++;
							}
						}
					}
	//			}
	//		}
	//	}
		//pt0 = pt0+1;
		//pt0++;
	}	//更新されなくなるまで待つ
//	if(smode == 0){
		LED4_OFF();
//	}
}


/*
void makesmap(int gx,int gy,int smode){	//等高線マップつくり
 	 //mode0なら普通 1なら最短
	unsigned char	 pt0 = 0,pt1 = 0,ct = 0;
	unsigned char	 finish = 0;
	short	 x = 0,y = 0,z = 0;
	short 	 xx = 0,yy = 0;
	unsigned char	 wdata = 0;

	if(smode != 15){//意味なし、書き換え用
	LED4_ON();
	}
	for (z = 0;z < (MAZESIZE_X*MAZESIZE_Y);z++){		// 等高線マップを初期化する
		smap[z/MAZESIZE_Y][z & (MAZESIZE_X-1)] = STEPMAX;
	}

	if(allsearchflag == 1 && smode == 0){
		for(yy = 0; yy < MAZESIZE_Y; yy++){
			for(xx = 0; xx < MAZESIZE_X; xx++){
				if((map[yy][xx] & 0xf0) != 0xf0){
					smap[yy][xx] = 0;
				}
				else{
				}
			}
		}
	}
	else{
		if(GOALSIZE == 1 || kaeri == 1){
			smap[gy][gx] = 0;			// 目標地点に距離0を書き込む
		}
		else if(GOALSIZE == 4){
			smap[gy][gx] = 0;			// 目標地点に距離0を書き込む
			smap[gy+1][gx] = 0;
			smap[gy][gx+1] = 0;
			smap[gy+1][gx+1] = 0;
		}
		else if(GOALSIZE == 9){
			smap[gy][gx] = 0;			// 目標地点に距離0を書き込む
			smap[gy+1][gx] = 0;
			smap[gy+2][gx] = 0;
			smap[gy][gx+1] = 0;
			smap[gy+1][gx+1] = 0;
			smap[gy+2][gx+1] = 0;
			smap[gy][gx+2] = 0;
			smap[gy+1][gx+2] = 0;
			smap[gy+2][gx+2] = 0;
		}
	}

	pt0 = 0;
	do{
		ct = 0;
		pt1 = pt0 + 1;
		for (y = 0;y < MAZESIZE_Y;y++){
			for (x = 0;x < MAZESIZE_X;x++){
				if (smap[y][x] == pt0){
					wdata = map[y][x];
					if (smode == 1){	//未探索区間は壁あり
						finish = 0;
						if (((wdata & 0x11) == 0x10) && (y != (MAZESIZE_Y-1))){//
							if(smap[y+1][x] == STEPMAX){
								smap[y+1][x] = pt1;
								ct++;
							}
						}
						if (((wdata & 0x22) == 0x20) && (x != (MAZESIZE_X-1))){
							if(smap[y][x+1] == STEPMAX){
								smap[y][x+1] = pt1;
								ct++;
							}
						}
						if (((wdata & 0x44) == 0x40) && (y != 0)){
							if(smap[y-1][x] == STEPMAX){
								smap[y-1][x] = pt1;
								ct++;
							}
						}
						if (((wdata & 0x88) == 0x80) && (x != 0)){
							if(smap[y][x-1] == STEPMAX){
								smap[y][x-1]  = pt1;
								ct++;
							}
						}
					}
					else{	//未探索区間は壁なし(通常)
						if ((wdata & 1) == 0  &&  y != (MAZESIZE_Y-1)){
							if(smap[y+1][x] == STEPMAX){
								smap[y+1][x] = pt1;
								if(y+1 == my && x == mx){
									finish = 1;
								}
								ct++;
							}
						}
						if ((wdata & 2) == 0  &&  x != (MAZESIZE_X-1)){
							if(smap[y][x+1] == STEPMAX){
								smap[y][x+1] = pt1;
								if(y == my && x+1 == mx){
									finish = 1;
								}
								ct++;
							}
						}
						if ((wdata & 4) == 0  &&  y != 0){
							if(smap[y-1][x] == STEPMAX){
								smap[y-1][x] = pt1;
								if(y-1 == my && x == mx){
									finish = 1;
								}
								ct++;
							}
						}
						if ((wdata & 8) == 0  &&  x != 0){
							if(smap[y][x-1] == STEPMAX){
								smap[y][x-1] = pt1;
								if(y == my && x-1 == mx){
									finish = 1;
								}
								ct++;
							}
						}
					}
				}
			}
		}
		//pt0 = pt0+1;
		pt0++;
	}while(ct != 0 && finish == 0);	//更新されなくなるまで待つ
//	if(smode == 0){
		LED4_OFF();
//	}
}

*/
