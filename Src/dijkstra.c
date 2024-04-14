#include"index.h"
#include"Interface.h"
#include"common.h"
#include"CMT.h"
#include"run.h"
#include"map.h"
#include"Dataflash.h"
#include"search.h"
#include"beeline.h"
#include "dijkstra.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#define ARM_MATH_CM4
#include "arm_math.h"
#include "arm_const_structs.h"

extern float saitandashv;
extern float saitan90v;
extern float saitanacc;
extern float saitandec;
extern float saitanslantacc;
extern float saitanslantdec;
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
extern float s45angleR;
extern float v90angleR;
extern float kojimaangleR;

extern float bigrightangleL;
extern float big180angleL;
extern float s135angleL;
extern float s45angleL;
extern float v90angleL;
extern float kojimaangleL;

extern unsigned char map[MAZESIZE_X][MAZESIZE_Y];
//uint8_t map[MAZESIZE_Y][MAZESIZE_X];

unsigned char maze[MAZESIZE_Y][MAZESIZE_X];
//int runtime[118];

short runtime[120];

//unsigned short node1[32][32][12];
//char checknode[16][16][12];
volatile char path[1024];

//volatile char fromx[16][16][12];
//volatile char fromy[16][16][12];
//volatile char fromh[16][16][12];

//extern volatile char edge[1024];

t_dijkstra nodeinfo[32][32][12];

//volatile char from_all[16][16][12];
volatile char dummy[1024];
//unsigned char node2[32][32][12];
//char mazeData[16][16] = { {14,6,6,6,6,6,6,6,4,6,6,6,6,6,4,5},{12,5,12,4,7,14,4,6,3,12,7,12,7,13,11,9},{9,10,3,10,5,12,2,7,12,2,4,2,4,2,5,9},{10,5,12,7,10,3,14,4,3,13,11,13,11,12,3,9},{12,3,9,14,4,6,6,3,13,10,4,2,6,3,13,9},{9,12,2,6,3,12,4,6,0,5,10,4,7,12,1,9},{9,9,13,13,12,3,11,14,3,10,5,10,5,9,9,9},{9,8,0,0,3,12,6,4,5,12,2,7,9,9,9,9},{9,9,11,11,14,0,7,10,3,10,5,12,3,9,9,9},{9,8,4,7,14,2,5,14,5,12,1,8,7,9,9,9},{9,11,10,6,5,14,2,6,2,1,11,10,5,9,9,9},{9,14,5,14,2,6,6,6,5,11,12,7,8,3,9,9},{10,5,8,7,12,5,12,5,10,5,9,13,9,13,9,9},{12,3,9,12,1,9,9,8,5,8,1,10,3,8,3,9},{8,7,8,3,9,9,9,9,10,1,8,5,13,9,14,1},{10,6,2,6,3,10,3,11,14,2,3,11,10,2,6,3} };


int saitantime = 0;

volatile char goalslantnum = 0;//ターン後の斜めゴ－ルの数

volatile char kojima_enbl = 0;

volatile char endx = 0;	//ダイクストラ最短後の座標系
volatile char endy = 0;
volatile char endh = 0;

extern volatile char sincurve;

void runTimeCalc(void) {
	short b = 0;
	float acc = 0, dec = 0;//, same = 0;
	float maxsp = 0;

	float saitanbefore45offset2 = (float)(saitanbefore45offset/1000.0);
	float saitanafter45offset2 = (float)(saitanafter45offset/1000.0);
	float saitanbeforeout45offset2 = (float)(saitanbeforeout45offset/1000.0);
	float saitanafterout45offset2 = (float)(saitanafterout45offset/1000.0);
	float saitanbeforebig90offset2 = (float)(saitanbeforebig90offset/1000.0);
	float saitanafterbig90offset2 = (float)(saitanafter45offset/1000.0);
	float saitanbefore135offset2 = (float)(saitanbefore135offset/1000.0);
	float saitanafter135offset2 = (float)(saitanafter135offset/1000.0);
	float saitanbeforeout135offset2 = (float)(saitanbeforeout135offset/1000.0);
	float saitanafterout135offset2 = (float)(saitanafterout135offset/1000.0);
	float saitanbefore180offset2 = (float)(saitanbefore180offset/1000.0);
	float saitanafter180offset2 = (float)(saitanafter180offset/1000.0);
	float saitanbeforev90offset2 = (float)(saitanbeforev90offset/1000.0);
	float saitanafterv90offset2 = (float)(saitanafterv90offset/1000.0);
	float saitanbeforekojimaoffset2 = (float)(saitanbeforekojimaoffset/1000.0);
	float saitanafterkojimaoffset2 = (float)(saitanafterkojimaoffset/1000.0);
	float saitanbig90alpha2 = (float)(saitanbig90alpha*180.0/3.1415);
	float saitanbig90w2 = (float)(saitanbig90w*180.0/3.1415);
	float saitan45alpha2 = (float)(saitan45alpha*180.0/3.1415);
	float saitan45w2 = (float)(saitan45w*180.0/3.1415);
	float saitanout45alpha2 = (float)(saitanout45alpha*180.0/3.1415);
	float saitanout45w2 = (float)(saitanout45w*180.0/3.1415);
	float saitan135alpha2 = (float)(saitan135alpha*180.0/3.1415);
	float saitan135w2 = (float)(saitan135w*180.0/3.1415);
	float saitanout135alpha2 = (float)(saitanout135alpha*180.0/3.1415);
	float saitanout135w2 = (float)(saitanout135w*180.0/3.1415);
	float saitan180alpha2 = (float)(saitan180alpha*180.0/3.1415);
	float saitan180w2 = (float)(saitan180w*180.0/3.1415);
	float saitanv90alpha2 = (float)(saitanv90alpha*180.0/3.1415);
	float saitanv90w2 = (float)(saitanv90w*180.0/3.1415);
	float saitankojimaalpha2 = (float)(saitankojimaalpha*180.0/3.1415);
	float saitankojimaw2 = (float)(saitankojimaw*180.0/3.1415);
	//setpara(para);
	runtime[0] = 0;
	for (b = 1; b < 32; b++) {
		acc = (((saitandashv*saitandashv) - (saitan45v*saitan45v)) / (float)(2.0 * saitanacc));//todoターン速度ごと、スタート時などの考慮？
		dec = (((saitandashv*saitandashv) - (saitan45v*saitan45v)) / (float)(2.0 * saitandec));
		if (acc + dec > (float)((90.0 / 1000.0) * (float)b)) {
			maxsp = sqrt(((float)((90.0 / 1000.0) * (float)b) + (saitan45v * saitan45v) / saitanacc / 2.0 + (saitan45v * saitan45v) / saitandec / 2.0) * 2.0 * saitanacc * saitandec / (saitanacc + saitandec));
			acc = (((maxsp*maxsp) - (saitan45v*saitan45v)) / (float)(2.0 * saitanacc));//todoターン速度ごと、スタート時などの考慮？
			dec = (((maxsp*maxsp) - (saitan45v*saitan45v)) / (float)(2.0 * saitandec));
			runtime[b] = (int)((acc / (float)(maxsp / 2.0) + dec / (float)(maxsp / 2.0)) * 100.0);
		}
		else {
			runtime[b] = (int)((acc / (float)(saitandashv / 2.0) + dec / (float)(saitandashv / 2.0) + (float)(((90.0 / 1000.0) * (float)b) - acc - dec) / (float)saitandashv) * 100.0);
		}
	}
	for (b = 32; b < 40; b++) {
		runtime[b] = 0;
	}
	for (b = 40; b < 103; b++) {
		acc = (((saitandashslantingv*saitandashslantingv) - (saitan45v*saitan45v)) / (float)(2.0 * saitanslantacc));//todoターン速度ごと、スタート時などの考慮？
		dec = (((saitandashslantingv*saitandashslantingv) - (saitan45v*saitan45v)) / (float)(2.0 * saitanslantdec));
		if (acc + dec > (float)((63.64 / 1000.0) * (float)(b-39))) {
			maxsp = sqrt(((float)((63.64 / 1000.0) * (float)(b-39)) + (saitan45v * saitan45v) / saitanslantacc / 2.0 + (saitan45v * saitan45v) / saitanslantdec / 2.0) * 2.0 * saitanslantacc * saitanslantdec / (saitanslantacc + saitanslantdec));
			acc = (((maxsp*maxsp) - (saitan45v*saitan45v)) / (float)(2.0 * saitanslantacc));//todoターン速度ごと、スタート時などの考慮？
			dec = (((maxsp*maxsp) - (saitan45v*saitan45v)) / (float)(2.0 * saitanslantdec));
			runtime[b] = (int)((acc / (float)(maxsp / 2.0) + dec / (float)(maxsp / 2.0)) * 100.0);
		}
		else {
			runtime[b] = (int)((acc / (float)(saitandashslantingv / 2.0) + dec / (float)(saitandashslantingv / 2.0) + ((float)((63.64 / 1000.0) * (float)(b-39)) - acc - dec) / (float)saitandashslantingv) * 100.0);
		}
	}

	if(sincurve == 0 || sincurve == 1){
		acc = (((saitanbig90w2*saitanbig90w2)) / (float)(2.0 * saitanbig90alpha2));//todoターン速度ごと、スタート時などの考慮？
		dec = (((saitanbig90w2*saitanbig90w2)) / (float)(2.0 * saitanbig90alpha2));
		runtime[103] = (int)((acc / (float)(saitanbig90w2 / 2.0) + dec / (float)(saitanbig90w2 / 2.0) + (bigrightangleR - acc - dec) / (float)saitanbig90w2 + (saitanbeforebig90offset2 + saitanafterbig90offset2) / (float)saitanbig90v) * 100.0);
		runtime[104] = (int)((acc / (float)(saitanbig90w2 / 2.0) + dec / (float)(saitanbig90w2 / 2.0) + (bigrightangleL - acc - dec) / (float)saitanbig90w2 + (saitanbeforebig90offset2 + saitanafterbig90offset2) / (float)saitanbig90v) * 100.0);
		//
		acc = (((saitan180w2*saitan180w2)) / (float)(2.0 * saitan180alpha2));//todoターン速度ごと、スタート時などの考慮？
		dec = (((saitan180w2*saitan180w2)) / (float)(2.0 * saitan180alpha2));
		runtime[105] = (int)((acc / (float)(saitan180w2 / 2.0) + dec / (float)(saitan180w2 / 2.0) + (big180angleR - acc - dec) / (float)saitan180w2 + (saitanbefore180offset2 + saitanafter180offset2) / (float)saitan180v) * 100.0);
		runtime[106] = (int)((acc / (float)(saitan180w2 / 2.0) + dec / (float)(saitan180w2 / 2.0) + (big180angleL - acc - dec) / (float)saitan180w2 + (saitanbefore180offset2 + saitanafter180offset2) / (float)saitan180v) * 100.0);
		//
		acc = (((saitan45w2*saitan45w2)) / (float)(2.0 * saitan45alpha2));//todoターン速度ごと、スタート時などの考慮？
		dec = (((saitan45w2*saitan45w2)) / (float)(2.0 * saitan45alpha2));
		runtime[107] = (int)((acc / (float)(saitan45w2 / 2.0) + dec / (float)(saitan45w2 / 2.0) + (s45angleR - acc - dec) / (float)saitan45w2 + (saitanbefore45offset2 + saitanafter45offset2) / (float)saitan45v) * 100.0);
		runtime[108] = (int)((acc / (float)(saitan45w2 / 2.0) + dec / (float)(saitan45w2 / 2.0) + (s45angleL - acc - dec) / (float)saitan45w2 + (saitanbefore45offset2 + saitanafter45offset2) / (float)saitan45v) * 100.0);
		//
		acc = (((saitan135w2*saitan135w2)) / (float)(2.0 * saitan135alpha2));//todoターン速度ごと、スタート時などの考慮？
		dec = (((saitan135w2*saitan135w2)) / (float)(2.0 * saitan135alpha2));
		runtime[109] = (int)((acc / (float)(saitan135w2 / 2.0) + dec / (float)(saitan135w2 / 2.0) + (s135angleR - acc - dec) / (float)saitan135w2 + (saitanbefore135offset2 + saitanafter135offset2) / (float)saitan135v) * 100.0);
		runtime[110] = (int)((acc / (float)(saitan135w2 / 2.0) + dec / (float)(saitan135w2 / 2.0) + (s135angleL - acc - dec) / (float)saitan135w2 + (saitanbefore135offset2 + saitanafter135offset2) / (float)saitan135v) * 100.0);
		//
		acc = (((saitanout45w2*saitanout45w2)) / (float)(2.0 * saitanout45alpha2));//todoターン速度ごと、スタート時などの考慮？
		dec = (((saitanout45w2*saitanout45w2)) / (float)(2.0 * saitanout45alpha2));
		runtime[111] = (int)((acc / (float)(saitanout45w2 / 2.0) + dec / (float)(saitanout45w2 / 2.0) + (s45angleR - acc - dec) / (float)saitanout45w2 + (saitanbeforeout45offset2 + saitanafterout45offset2) / (float)saitanout45v) * 100.0);
		runtime[112] = (int)((acc / (float)(saitanout45w2 / 2.0) + dec / (float)(saitanout45w2 / 2.0) + (s45angleL - acc - dec) / (float)saitanout45w2 + (saitanbeforeout45offset2 + saitanafterout45offset2) / (float)saitanout45v) * 100.0);
		//
		acc = (((saitanout135w2*saitan135w2)) / (float)(2.0 * saitanout135alpha2));//todoターン速度ごと、スタート時などの考慮？
		dec = (((saitanout135w2*saitan135w2)) / (float)(2.0 * saitanout135alpha2));
		runtime[113] = (int)((acc / (float)(saitanout135w2 / 2.0) + dec / (float)(saitanout135w2 / 2.0) + (s135angleR - acc - dec) / (float)saitanout135w2 + (saitanbeforeout135offset2 + saitanafterout135offset2) / (float)saitanout135v) * 100.0);
		runtime[114] = (int)((acc / (float)(saitanout135w2 / 2.0) + dec / (float)(saitanout135w2 / 2.0) + (s135angleL - acc - dec) / (float)saitanout135w2 + (saitanbeforeout135offset2 + saitanafterout135offset2) / (float)saitanout135v) * 100.0);
		//
		acc = (((saitanv90w2*saitanv90w2)) / (float)(2.0 * saitanv90alpha2));//todoターン速度ごと、スタート時などの考慮？
		dec = (((saitanv90w2*saitanv90w2)) / (float)(2.0 * saitanv90alpha2));
		runtime[115] = (int)((acc / (float)(saitanv90w2 / 2.0) + dec / (float)(saitanv90w2 / 2.0) + (v90angleR - acc - dec) / (float)saitanv90w2 + (saitanbeforev90offset2 + saitanafterv90offset2) / (float)saitanv90v) * 100.0);
		runtime[116] = (int)((acc / (float)(saitanv90w2 / 2.0) + dec / (float)(saitanv90w2 / 2.0) + (v90angleL - acc - dec) / (float)saitanv90w2 + (saitanbeforev90offset2 + saitanafterv90offset2) / (float)saitanv90v) * 100.0);
		//
		acc = (((saitankojimaw2*saitankojimaw2)) / (float)(2.0 * saitankojimaalpha2));//todoターン速度ごと、スタート時などの考慮？
		dec = (((saitankojimaw2*saitankojimaw2)) / (float)(2.0 * saitankojimaalpha2));
		runtime[117] = (int)((acc / (float)(saitankojimaw2 / 2.0) + dec / (float)(saitankojimaw2 / 2.0) + (kojimaangleR - acc - dec) / (float)saitankojimaw2 + (saitanbeforekojimaoffset2 + saitanafterkojimaoffset2) / (float)saitankojimav) * 100.0);
		runtime[118] = (int)((acc / (float)(saitankojimaw2 / 2.0) + dec / (float)(saitankojimaw2 / 2.0) + (kojimaangleL - acc - dec) / (float)saitankojimaw2 + (saitanbeforekojimaoffset2 + saitanafterkojimaoffset2) / (float)saitankojimav) * 100.0);
		//
	}
	else{
		runtime[103] = (int)(((saitanbig90w2/saitanbig90alpha2)*2.0f + (saitanbeforebig90offset2 + saitanafterbig90offset2) / (float)saitanbig90v)*100.0f);//(int)((acc / (float)(saitanbig90w2 / 2.0) + dec / (float)(saitanbig90w2 / 2.0) + (bigrightangle - acc - dec) / (float)saitanbig90w2 + (saitanbeforebig90offset2 + saitanafterbig90offset2) / (float)saitanbig90v) * 100.0);
		runtime[104] = (int)(((saitanbig90w2/saitanbig90alpha2)*2.0f + (saitanbeforebig90offset2 + saitanafterbig90offset2) / (float)saitanbig90v)*100.0f);//(int)((acc / (float)(saitanbig90w2 / 2.0) + dec / (float)(saitanbig90w2 / 2.0) + (bigrightangle - acc - dec) / (float)saitanbig90w2 + (saitanbeforebig90offset2 + saitanafterbig90offset2) / (float)saitanbig90v) * 100.0);
		//
		runtime[105] = (int)(((saitan180w2/saitan180alpha2)*2.0f + (saitanbefore180offset2 + saitanafter180offset2) / (float)saitan180v)*100.0f);//(int)((acc / (float)(saitan180w2 / 2.0) + dec / (float)(saitan180w2 / 2.0) + (big180angle - acc - dec) / (float)saitan180w2 + (saitanbefore180offset2 + saitanafter180offset2) / (float)saitan180v) * 100.0);
		runtime[106] = (int)(((saitan180w2/saitan180alpha2)*2.0f + (saitanbefore180offset2 + saitanafter180offset2) / (float)saitan180v)*100.0f);//(int)((acc / (float)(saitan180w2 / 2.0) + dec / (float)(saitan180w2 / 2.0) + (big180angle - acc - dec) / (float)saitan180w2 + (saitanbefore180offset2 + saitanafter180offset2) / (float)saitan180v) * 100.0);
		//
		runtime[107] = (int)(((saitan45w2/saitan45alpha2)*2.0f + (saitanbefore45offset2 + saitanafter45offset2) / (float)saitan45v)*100.0f);//(int)((acc / (float)(saitan45w2 / 2.0) + dec / (float)(saitan45w2 / 2.0) + (s45angle - acc - dec) / (float)saitan45w2 + (saitanbefore45offset2 + saitanafter45offset2) / (float)saitan45v) * 100.0);
		runtime[108] = (int)(((saitan45w2/saitan45alpha2)*2.0f + (saitanbefore45offset2 + saitanafter45offset2) / (float)saitan45v)*100.0f);//(int)((acc / (float)(saitan45w2 / 2.0) + dec / (float)(saitan45w2 / 2.0) + (s45angle - acc - dec) / (float)saitan45w2 + (saitanbefore45offset2 + saitanafter45offset2) / (float)saitan45v) * 100.0);
		//
		runtime[109] = (int)(((saitan135w2/saitan135alpha2)*2.0f + (saitanbefore135offset2 + saitanafter135offset2) / (float)saitan135v)*100.0f);//(int)((acc / (float)(saitan135w2 / 2.0) + dec / (float)(saitan135w2 / 2.0) + (s135angle - acc - dec) / (float)saitan135w2 + (saitanbefore135offset2 + saitanafter135offset2) / (float)saitan135v) * 100.0);
		runtime[110] = (int)(((saitan135w2/saitan135alpha2)*2.0f + (saitanbefore135offset2 + saitanafter135offset2) / (float)saitan135v)*100.0f);//(int)((acc / (float)(saitan135w2 / 2.0) + dec / (float)(saitan135w2 / 2.0) + (s135angle - acc - dec) / (float)saitan135w2 + (saitanbefore135offset2 + saitanafter135offset2) / (float)saitan135v) * 100.0);
		//
		runtime[111] = (int)(((saitanout45w2/saitanout45alpha2)*2.0f + (saitanbeforeout45offset2 + saitanafterout45offset2) / (float)saitanout45v)*100.0f);//(int)((acc / (float)(saitanout45w2 / 2.0) + dec / (float)(saitanout45w2 / 2.0) + (s45angle - acc - dec) / (float)saitanout45w2 + (saitanbeforeout45offset2 + saitanafterout45offset2) / (float)saitanout45v) * 100.0);
		runtime[112] = (int)(((saitanout45w2/saitanout45alpha2)*2.0f + (saitanbeforeout45offset2 + saitanafterout45offset2) / (float)saitanout45v)*100.0f);//(int)((acc / (float)(saitanout45w2 / 2.0) + dec / (float)(saitanout45w2 / 2.0) + (s45angle - acc - dec) / (float)saitanout45w2 + (saitanbeforeout45offset2 + saitanafterout45offset2) / (float)saitanout45v) * 100.0);
		//
		runtime[113] = (int)(((saitanout135w2/saitanout135alpha2)*2.0f + (saitanbeforeout135offset2 + saitanafterout135offset2) / (float)saitanout135v)*100.0f);//(int)((acc / (float)(saitanout135w2 / 2.0) + dec / (float)(saitanout135w2 / 2.0) + (s135angle - acc - dec) / (float)saitanout135w2 + (saitanbeforeout135offset2 + saitanafterout135offset2) / (float)saitanout135v) * 100.0);
		runtime[114] = (int)(((saitanout135w2/saitanout135alpha2)*2.0f + (saitanbeforeout135offset2 + saitanafterout135offset2) / (float)saitanout135v)*100.0f);//(int)((acc / (float)(saitanout135w2 / 2.0) + dec / (float)(saitanout135w2 / 2.0) + (s135angle - acc - dec) / (float)saitanout135w2 + (saitanbeforeout135offset2 + saitanafterout135offset2) / (float)saitanout135v) * 100.0);
		//
		runtime[115] = (int)(((saitanv90w2/saitanv90alpha2)*2.0f + (saitanbeforev90offset2 + saitanafterv90offset2) / (float)saitanv90v)*100.0f);//(int)((acc / (float)(saitanv90w2 / 2.0) + dec / (float)(saitanv90w2 / 2.0) + (v90angle - acc - dec) / (float)saitanv90w2 + (saitanbeforev90offset2 + saitanafterv90offset2) / (float)saitanv90v) * 100.0);
		runtime[116] = (int)(((saitanv90w2/saitanv90alpha2)*2.0f + (saitanbeforev90offset2 + saitanafterv90offset2) / (float)saitanv90v)*100.0f);//(int)((acc / (float)(saitanv90w2 / 2.0) + dec / (float)(saitanv90w2 / 2.0) + (v90angle - acc - dec) / (float)saitanv90w2 + (saitanbeforev90offset2 + saitanafterv90offset2) / (float)saitanv90v) * 100.0);
		//
		runtime[117] = (int)(((saitankojimaw2/saitankojimaalpha2)*2.0f + (saitanbeforekojimaoffset2 + saitanafterkojimaoffset2) / (float)saitankojimav)*100.0f);//(int)((acc / (float)(saitankojimaw2 / 2.0) + dec / (float)(saitankojimaw2 / 2.0) + (kojimaangle - acc - dec) / (float)saitankojimaw2 + (saitanbeforekojimaoffset2 + saitanafterkojimaoffset2) / (float)saitankojimav) * 100.0);
		runtime[118] = (int)(((saitankojimaw2/saitankojimaalpha2)*2.0f + (saitanbeforekojimaoffset2 + saitanafterkojimaoffset2) / (float)saitankojimav)*100.0f);//(int)((acc / (float)(saitankojimaw2 / 2.0) + dec / (float)(saitankojimaw2 / 2.0) + (kojimaangle - acc - dec) / (float)saitankojimaw2 + (saitanbeforekojimaoffset2 + saitanafterkojimaoffset2) / (float)saitankojimav) * 100.0);
		//
	}
	runtime[119] = 0;
}

void map_to_maze(void){
	unsigned char x = 0,y = 0;
	for(y = 0; y < MAZESIZE_Y; y++){
		for(x = 0; x < MAZESIZE_X; x++){
			maze[y][x] = 0;
		}
	}
	for(y = 0; y < MAZESIZE_Y; y++){
		for(x = 0; x < MAZESIZE_X; x++){
			maze[y][x] = map[y][x];
			//mazeは探索済みかどうかチェックしないので未探索区間は壁を入れておく
			if((maze[y][x] & 0x10) == 0x00){
				maze[y][x] |= 0x01;
			}
			if((maze[y][x] & 0x20) == 0x00){
				maze[y][x] |= 0x02;
			}
			if((maze[y][x] & 0x40) == 0x00){
				maze[y][x] |= 0x04;
			}
			if((maze[y][x] & 0x80) == 0x00){
				maze[y][x] |= 0x08;
			}
		}
	}
}

void nodereset(void) {
	volatile short a = 0, b = 0, c = 0;
	for (a = 0; a < 32; a++) {
		for (b = 0; b < 32; b++) {
			for (c = 0; c < 12; c++) {
				nodeinfo[a][b][c].node1 = 55555;
				nodeinfo[a][b][c].checked = 0;
				//checknode[a][b][c] = 0;
			}
		}
	}
	for (a = 0; a < 1024; a++) {
		path[a] = 0;
	}
}

void shortest(int tx, int ty) {
	//int (*a)[Y][Z] = (int(*)[Y][Z])malloc(X*Y*Z*sizeof(int));
	//char (*mazeData)[16] = (char (*)[16])malloc(sizeof(char)*16*16);//{ {14,6,6,6,6,6,6,6,4,6,6,6,6,6,4,5},{12,5,12,4,7,14,4,6,3,12,7,12,7,13,11,9},{9,10,3,10,5,12,2,7,12,2,4,2,4,2,5,9},{10,5,12,7,10,3,14,4,3,13,11,13,11,12,3,9},{12,3,9,14,4,6,6,3,13,10,4,2,6,3,13,9},{9,12,2,6,3,12,4,6,0,5,10,4,7,12,1,9},{9,9,13,13,12,3,11,14,3,10,5,10,5,9,9,9},{9,8,0,0,3,12,6,4,5,12,2,7,9,9,9,9},{9,9,11,11,14,0,7,10,3,10,5,12,3,9,9,9},{9,8,4,7,14,2,5,14,5,12,1,8,7,9,9,9},{9,11,10,6,5,14,2,6,2,1,11,10,5,9,9,9},{9,14,5,14,2,6,6,6,5,11,12,7,8,3,9,9},{10,5,8,7,12,5,12,5,10,5,9,13,9,13,9,9},{12,3,9,12,1,9,9,8,5,8,1,10,3,8,3,9},{8,7,8,3,9,9,9,9,10,1,8,5,13,9,14,1},{10,6,2,6,3,10,3,11,14,2,3,11,10,2,6,3} };
//	char (*maze)[16] = (char (*)[16])malloc(sizeof(char)*16*16);
//	char (*fromx)[16][12] = (char (*)[16][12])malloc(sizeof(char)*16*16*12);
//	char (*fromy)[16][12] = (char (*)[16][12])malloc(sizeof(char)*16*16*12);
//	char (*fromh)[16][12] = (char (*)[16][12])malloc(sizeof(char)*16*16*12);
//	char(*dummy) = (char *)malloc(sizeof(char) * 1000);
//	int(*nodeinfo)[16][12] = (int(*)[16][12])malloc(sizeof(int) * 16 * 16 * 12);
//	int(*node2)[16][12] = (int(*)[16][12])malloc(sizeof(int) * 16 * 16 * 12);
//	char (*checknode)[16][12] = (char (*)[16][12])malloc(sizeof(char)*16*16*12);
	char plus = 0;
	volatile int i = 0;
	unsigned short x = 0, y = 0;
	char head = 0;
	char head2 = 0;
	char nomorego = 0;
	unsigned char a = 0, b = 0, c = 0;
	unsigned char a2 = 0, b2 = 0, c2 = 0;
	unsigned char a3 = 0, b3 = 0, c3 = 0;
	unsigned short nowshort = 0;
	short dummycnt = 0;
	unsigned char ppp = 0;//32区画対応用
	saitantime = 0;
	endx = 0;
	endy = 0;
	endh = 0;

	for (a = 0; a < 32; a++) {
		for (b = 0; b < 32; b++) {
			for (c = 0; c < 12; c++) {
				nodeinfo[a][b][c].from_x = 0;
				nodeinfo[a][b][c].from_y = 0;
				nodeinfo[a][b][c].xy = 0;
				nodeinfo[a][b][c].from_h = 0;
			}
		}
	}
	for (i = 0; i < 1024; i++) {
		dummy[i] = 0;
	}
	nodeinfo[0][0][0].node1 = 0;
	nodeinfo[0][0][0].checked = 1;
	//checknode[0][0][0].node1= 1;
	a = 0;
	b = 0;
	c = 0;
	while (1) {
		nomorego = 0;
		if (head == 0) {
			//北向き
			//直進１～１５区画
			if (y + 1 <= MA && (maze[y + 1][x] & 0x05) == 0x00 && nomorego == 0) {
				if (nodeinfo[x][y + 1][0].node1> nodeinfo[x][y][0].node1+ runtime[GO1]) {
					nodeinfo[x][y + 1][0].node1= nodeinfo[x][y][0].node1+ runtime[GO1];
					infoup(x,y,head2,0,1,0);
					//from_all[x][y+1][0].node1= GO1;
				}
				head = 0;
			}
			else {
				nomorego = 1;
			}

			/////////////////////////////////
			ppp = 0;
			for(ppp = 2; ppp < 32; ppp++){
				if (y + ppp <= MA && (maze[y + ppp][x] & 0x01) == 0x00 && nomorego == 0) {
					if (nodeinfo[x][y + ppp][0].node1> nodeinfo[x][y][0].node1+ runtime[ppp]) {
						nodeinfo[x][y + ppp][0].node1= nodeinfo[x][y][0].node1+ runtime[ppp];
						infoup(x,y,head2,0,ppp,0);
					}
					head = 0;
				}
				else {
					nomorego = 1;
					ppp = 32;
				}
			}
			/////////////////////////////////
			//大回り９０
			if (x + 1 <= MA && y + 1 <= MA && (maze[y + 1][x] & 0x06) == 0x00 && (maze[y + 1][x + 1] & 0x02) == 0x00) {
				if (nodeinfo[x + 1][y + 1][1].node1> nodeinfo[x][y][0].node1+ runtime[BIG90R]) {
					nodeinfo[x + 1][y + 1][1].node1= nodeinfo[x][y][0].node1+ runtime[BIG90R];
					infoup(x,y,head2,1,1,1);
				}
				head = 1;
			}
			if (x - 1 >= MI && y + 1 <= MA && (maze[y + 1][x] & 0x0C) == 0x00 && (maze[y + 1][x - 1] & 0x08) == 0x00) {
				if (nodeinfo[x - 1][y + 1][3].node1> nodeinfo[x][y][0].node1+ runtime[BIG90L]) {
					nodeinfo[x - 1][y + 1][3].node1= nodeinfo[x][y][0].node1+ runtime[BIG90L];
					infoup(x,y,head2,-1,1,3);
				}
				head = 3;
			}
			//大回り１８０
			if (x + 1 <= MA && y + 1 <= MA && (maze[y + 1][x] & 0x06) == 0x00 && (maze[y][x + 1] & 0x05) == 0x00) {
				if (nodeinfo[x + 1][y][2].node1> nodeinfo[x][y][0].node1+ runtime[BIG180R]) {
					nodeinfo[x + 1][y][2].node1= nodeinfo[x][y][0].node1+ runtime[BIG180R];
					infoup(x,y,head2,1,0,2);
				}
				head = 2;
			}
			if (x - 1 >= MI && y + 1 <= MA && (maze[y + 1][x] & 0x0C) == 0x00 && (maze[y][x - 1] & 0x05) == 0x00) {
				if (nodeinfo[x - 1][y][2].node1> nodeinfo[x][y][0].node1+ runtime[BIG180L]) {
					nodeinfo[x - 1][y][2].node1= nodeinfo[x][y][0].node1+ runtime[BIG180L];
					infoup(x,y,head2,-1,0,2);
				}
				head = 2;
			}
			//in45
			if (x + 1 <= MA && y + 1 <= MA && (maze[y + 1][x] & 0x06) == 0x00 && (maze[y + 1][x + 1] & 0x01) == 0x00) {
				if (nodeinfo[x][y + 1][9].node1> nodeinfo[x][y][0].node1+ runtime[IN45R]) {
					nodeinfo[x][y + 1][9].node1= nodeinfo[x][y][0].node1+ runtime[IN45R];
					infoup(x,y,head2,0,1,9);
				}
				head = 9;
			}
			if (x - 1 >= MI && y + 1 <= MA && (maze[y + 1][x] & 0x0C) == 0x00 && (maze[y + 1][x - 1] & 0x01) == 0x00) {
				if (nodeinfo[x - 1][y + 1][8].node1> nodeinfo[x][y][0].node1+ runtime[IN45L]) {
					nodeinfo[x - 1][y + 1][8].node1= nodeinfo[x][y][0].node1+ runtime[IN45L];
					infoup(x,y,head2,-1,1,8);
				}
				head = 8;
			}
			//in135
			if (x + 1 <= MA && y + 1 <= MA && (maze[y + 1][x] & 0x06) == 0x00 && (maze[y][x + 1] & 0x03) == 0x00) {
				if (nodeinfo[x + 1][y][6].node1> nodeinfo[x][y][0].node1+ runtime[IN135R]) {
					nodeinfo[x + 1][y][6].node1= nodeinfo[x][y][0].node1+ runtime[IN135R];
					infoup(x,y,head2,1,0,6);
				}
				head = 6;
			}
			if (x - 1 >= MI && y + 1 <= MA && (maze[y + 1][x] & 0x0C) == 0x00 && (maze[y][x - 1] & 0x09) == 0x00) {
				if (nodeinfo[x - 1][y][7].node1> nodeinfo[x][y][0].node1+ runtime[IN135L]) {
					nodeinfo[x - 1][y][7].node1= nodeinfo[x][y][0].node1+ runtime[IN135L];
					infoup(x,y,head2,-1,0,7);
				}
				head = 7;
			}
		}
		else if (head == 1) {
			//東向き
			//直進１～１５区画
			if (x + 1 <= MA && (maze[y][x + 1] & 0x0A) == 0x00 && nomorego == 0) {
				if (nodeinfo[x + 1][y][1].node1> nodeinfo[x][y][1].node1+ runtime[GO1]) {
					nodeinfo[x + 1][y][1].node1= nodeinfo[x][y][1].node1+ runtime[GO1];
					infoup(x,y,head2,1,0,1);
				}
				head = 1;
			}
			else {
				nomorego = 1;
			}

			/////////////////////////////////
			ppp = 0;
			for(ppp = 2; ppp < 32; ppp++){
				if (x + ppp <= MA && (maze[y][x + ppp] & 0x02) == 0x00 && nomorego == 0) {
					if (nodeinfo[x + ppp][y][1].node1> nodeinfo[x][y][1].node1+ runtime[ppp]) {
						nodeinfo[x + ppp][y][1].node1= nodeinfo[x][y][1].node1+ runtime[ppp];
						infoup(x,y,head2,ppp,0,1);
					}
					head = 1;
				}
				else {
					nomorego = 1;
					ppp = 32;
				}
			}
			/////////////////////////////////
			//大回り９０
			if (x + 1 <= MA && y - 1 >= MI && (maze[y][x + 1] & 0x0C) == 0x00 && (maze[y - 1][x + 1] & 0x04) == 0x00) {
				if (nodeinfo[x + 1][y - 1][2].node1> nodeinfo[x][y][1].node1+ runtime[BIG90R]) {
					nodeinfo[x + 1][y - 1][2].node1= nodeinfo[x][y][1].node1+ runtime[BIG90R];
					infoup(x,y,head2,1,-1,2);
				}
				head = 2;
			}
			if (x + 1 <= MA && y + 1 <= MA && (maze[y][x + 1] & 0x09) == 0x00 && (maze[y + 1][x + 1] & 0x01) == 0x00) {
				if (nodeinfo[x + 1][y + 1][0].node1> nodeinfo[x][y][1].node1+ runtime[BIG90L]) {
					nodeinfo[x + 1][y + 1][0].node1= nodeinfo[x][y][1].node1+ runtime[BIG90L];
					infoup(x,y,head2,1,1,0);
				}
				head = 0;
			}
			//大回り１８０
			if (x + 1 <= MA && y - 1 >= MI && (maze[y][x + 1] & 0x0C) == 0x00 && (maze[y - 1][x] & 0x0A) == 0x00) {
				if (nodeinfo[x][y - 1][3].node1> nodeinfo[x][y][1].node1+ runtime[BIG180R]) {
					nodeinfo[x][y - 1][3].node1= nodeinfo[x][y][1].node1+ runtime[BIG180R];
					infoup(x,y,head2,0,-1,3);
				}
				head = 3;
			}
			if (x + 1 <= MA && y + 1 <= MA && (maze[y][x + 1] & 0x09) == 0x00 && (maze[y + 1][x] & 0x0A) == 0x00) {
				if (nodeinfo[x][y + 1][3].node1> nodeinfo[x][y][1].node1+ runtime[BIG180L]) {
					nodeinfo[x][y + 1][3].node1= nodeinfo[x][y][1].node1+ runtime[BIG180L];
					infoup(x,y,head2,0,1,3);
				}
				head = 3;
			}
			//in45
			if (x + 1 <= MA && y - 1 >= MI && (maze[y][x + 1] & 0x0C) == 0x00 && (maze[y - 1][x + 1] & 0x02) == 0x00) {
				if (nodeinfo[x + 1][y - 1][6].node1> nodeinfo[x][y][1].node1+ runtime[IN45R]) {
					nodeinfo[x + 1][y - 1][6].node1= nodeinfo[x][y][1].node1+ runtime[IN45R];
					infoup(x,y,head2,1,-1,6);
				}
				head = 6;
			}
			if (x + 1 <= MA && y + 1 <= MA && (maze[y][x + 1] & 0x09) == 0x00 && (maze[y + 1][x + 1] & 0x02) == 0x00) {
				if (nodeinfo[x + 1][y][5].node1> nodeinfo[x][y][1].node1+ runtime[IN45L]) {
					nodeinfo[x + 1][y][5].node1= nodeinfo[x][y][1].node1+ runtime[IN45L];
					infoup(x,y,head2,1,0,5);
				}
				head = 5;
			}
			//in135
			if (x + 1 <= MA && y - 1 >= MI && (maze[y][x + 1] & 0x0C) == 0x00 && (maze[y - 1][x] & 0x06) == 0x00) {
				if (nodeinfo[x][y - 1][11].node1> nodeinfo[x][y][1].node1+ runtime[IN135R]) {
					nodeinfo[x][y - 1][11].node1= nodeinfo[x][y][1].node1+ runtime[IN135R];
					infoup(x,y,head2,0,-1,11);
				}
				head = 11;
			}
			if (x + 1 <= MA && y + 1 <= MA && (maze[y][x + 1] & 0x09) == 0x00 && (maze[y + 1][x] & 0x03) == 0x00) {
				if (nodeinfo[x][y + 1][8].node1> nodeinfo[x][y][1].node1+ runtime[IN135L]) {
					nodeinfo[x][y + 1][8].node1= nodeinfo[x][y][1].node1+ runtime[IN135L];
					infoup(x,y,head2,0,1,8);
				}
				head = 8;
			}
		}
		else if (head == 2) {
			//南向き
			//直進１～１５区画
			if (y - 1 >= MI && (maze[y - 1][x] & 0x05) == 0x00 && nomorego == 0) {
				if (nodeinfo[x][y - 1][2].node1> nodeinfo[x][y][2].node1+ runtime[GO1]) {
					nodeinfo[x][y - 1][2].node1= nodeinfo[x][y][2].node1+ runtime[GO1];
					infoup(x,y,head2,0,-1,2);
				}
				head = 2;
			}
			else {
				nomorego = 1;
			}

			/////////////////////////////////
			ppp = 0;
			for(ppp = 2; ppp < 32; ppp++){
				if (y - ppp >= MI && (maze[y - ppp][x] & 0x04) == 0x00 && nomorego == 0) {
					if (nodeinfo[x][y - ppp][2].node1> nodeinfo[x][y][2].node1+ runtime[ppp]) {
						nodeinfo[x][y - ppp][2].node1= nodeinfo[x][y][2].node1+ runtime[ppp];
						infoup(x,y,head2,0,-ppp,2);
					}
					head = 2;
				}
				else {
					nomorego = 1;
					ppp = 32;
				}
			}
			/////////////////////////////////
			//大回り９０
			if (x - 1 >= MI && y - 1 >= MI && (maze[y - 1][x] & 0x09) == 0x00 && (maze[y - 1][x - 1] & 0x08) == 0x00) {
				if (nodeinfo[x - 1][y - 1][3].node1> nodeinfo[x][y][2].node1+ runtime[BIG90R]) {
					nodeinfo[x - 1][y - 1][3].node1= nodeinfo[x][y][2].node1+ runtime[BIG90R];
					infoup(x,y,head2,-1,-1,3);
				}
				head = 3;
			}
			if (x + 1 <= MA && y - 1 >= MI && (maze[y - 1][x] & 0x03) == 0x00 && (maze[y - 1][x + 1] & 0x02) == 0x00) {
				if (nodeinfo[x + 1][y - 1][1].node1> nodeinfo[x][y][2].node1+ runtime[BIG90L]) {
					nodeinfo[x + 1][y - 1][1].node1= nodeinfo[x][y][2].node1+ runtime[BIG90L];
					infoup(x,y,head2,1,-1,1);
				}
				head = 1;
			}
			//大回り１８０
			if (x - 1 >= MI && y - 1 >= MI && (maze[y - 1][x] & 0x09) == 0x00 && (maze[y][x - 1] & 0x05) == 0x00) {
				if (nodeinfo[x - 1][y][0].node1> nodeinfo[x][y][2].node1+ runtime[BIG180R]) {
					nodeinfo[x - 1][y][0].node1= nodeinfo[x][y][2].node1+ runtime[BIG180R];
					infoup(x,y,head2,-1,0,0);
				}
				head = 0;
			}
			if (x + 1 <= MA && y - 1 >= MI && (maze[y - 1][x] & 0x03) == 0x00 && (maze[y][x + 1] & 0x05) == 0x00) {
				if (nodeinfo[x + 1][y][0].node1> nodeinfo[x][y][2].node1+ runtime[BIG180L]) {
					nodeinfo[x + 1][y][0].node1= nodeinfo[x][y][2].node1+ runtime[BIG180L];
					infoup(x,y,head2,1,0,0);
				}
				head = 0;
			}
			//in45
			if (x - 1 >= MI && y - 1 >= MI && (maze[y - 1][x] & 0x09) == 0x00 && (maze[y - 1][x - 1] & 0x04) == 0x00) {
				if (nodeinfo[x - 1][y - 1][11].node1> nodeinfo[x][y][2].node1+ runtime[IN45R]) {
					nodeinfo[x - 1][y - 1][11].node1= nodeinfo[x][y][2].node1+ runtime[IN45R];
					infoup(x,y,head2,-1,-1,11);
				}
				head = 11;
			}
			if (x + 1 <= MA && y - 1 >= MI && (maze[y - 1][x] & 0x03) == 0x00 && (maze[y - 1][x + 1] & 0x04) == 0x00) {
				if (nodeinfo[x][y - 1][10].node1> nodeinfo[x][y][2].node1+ runtime[IN45L]) {
					nodeinfo[x][y - 1][10].node1= nodeinfo[x][y][2].node1+ runtime[IN45L];
					infoup(x,y,head2,0,-1,10);
				}
				head = 10;
			}
			//in135
			if (x - 1 >= MI && y - 1 >= MI && (maze[y - 1][x] & 0x09) == 0x00 && (maze[y][x - 1] & 0x0C) == 0x00) {
				if (nodeinfo[x - 1][y - 1][4].node1> nodeinfo[x][y][2].node1+ runtime[IN135R]) {
					nodeinfo[x - 1][y - 1][4].node1= nodeinfo[x][y][2].node1+ runtime[IN135R];
					infoup(x,y,head2,-1,-1,4);
				}
				head = 4;
			}
			if (x + 1 <= MA && y - 1 >= MI && y + 1 <= MA && (maze[y - 1][x] & 0x03) == 0x00 && (maze[y][x + 1] & 0x06) == 0x00) {
				if (nodeinfo[x + 1][y - 1][5].node1> nodeinfo[x][y][2].node1+ runtime[IN135L]) {
					nodeinfo[x + 1][y - 1][5].node1= nodeinfo[x][y][2].node1+ runtime[IN135L];
					infoup(x,y,head2,1,-1,5);
				}
				head = 9;
			}
		}
		else if (head == 3) {
			//西向き
			//直進１～１５区画
			if (x - 1 >= MI && (maze[y][x - 1] & 0x0A) == 0x00 && nomorego == 0) {
				if (nodeinfo[x - 1][y][3].node1> nodeinfo[x][y][3].node1+ runtime[GO1]) {
					nodeinfo[x - 1][y][3].node1= nodeinfo[x][y][3].node1+ runtime[GO1];
					infoup(x,y,head2,-1,0,3);
				}
				head = 3;
			}
			else {
				nomorego = 1;
			}

			/////////////////////////////////
			ppp = 0;
			for(ppp = 2; ppp < 32; ppp++){
				if (x - ppp >= MI && (maze[y][x - ppp] & 0x08) == 0x00 && nomorego == 0) {
					if (nodeinfo[x - ppp][y][3].node1> nodeinfo[x][y][3].node1+ runtime[ppp]) {
						nodeinfo[x - ppp][y][3].node1= nodeinfo[x][y][3].node1+ runtime[ppp];
						infoup(x,y,head2,-ppp,0,3);
					}
					head = 3;
				}
				else {
					nomorego = 1;
					ppp = 32;
				}
			}
			/////////////////////////////////
			//大回り９０
			if (x - 1 >= MI && y + 1 <= MA && (maze[y][x - 1] & 0x03) == 0x00 && (maze[y + 1][x - 1] & 0x01) == 0x00) {
				if (nodeinfo[x - 1][y + 1][0].node1> nodeinfo[x][y][3].node1+ runtime[BIG90R]) {
					nodeinfo[x - 1][y + 1][0].node1= nodeinfo[x][y][3].node1+ runtime[BIG90R];
					infoup(x,y,head2,-1,1,0);
				}
				head = 0;
			}
			if (x - 1 >= MI && y - 1 >= MI && (maze[y][x - 1] & 0x06) == 0x00 && (maze[y - 1][x - 1] & 0x04) == 0x00) {
				if (nodeinfo[x - 1][y - 1][2].node1> nodeinfo[x][y][3].node1+ runtime[BIG90L]) {
					nodeinfo[x - 1][y - 1][2].node1= nodeinfo[x][y][3].node1+ runtime[BIG90L];
					infoup(x,y,head2,-1,-1,2);
				}
				head = 2;
			}
			//大回り１８０
			if (x - 1 >= MI && y + 1 <= MA && (maze[y][x - 1] & 0x03) == 0x00 && (maze[y + 1][x] & 0x0A) == 0x00) {
				if (nodeinfo[x][y + 1][1].node1> nodeinfo[x][y][3].node1+ runtime[BIG180R]) {
					nodeinfo[x][y + 1][1].node1= nodeinfo[x][y][3].node1+ runtime[BIG180R];
					infoup(x,y,head2,0,1,1);
				}
				head = 1;
			}
			if (x - 1 >= MI && y - 1 >= MI && (maze[y][x - 1] & 0x06) == 0x00 && (maze[y - 1][x] & 0x0A) == 0x00) {
				if (nodeinfo[x][y - 1][1].node1> nodeinfo[x][y][3].node1+ runtime[BIG180L]) {
					nodeinfo[x][y - 1][1].node1= nodeinfo[x][y][3].node1+ runtime[BIG180L];
					infoup(x,y,head2,0,-1,1);
				}
				head = 1;
			}
			//in45
			if (x - 1 >= MI && y + 1 <= MA && (maze[y][x - 1] & 0x03) == 0x00 && (maze[y + 1][x - 1] & 0x08) == 0x00) {
				if (nodeinfo[x - 1][y][4].node1> nodeinfo[x][y][3].node1+ runtime[IN45R]) {
					nodeinfo[x - 1][y][4].node1= nodeinfo[x][y][3].node1+ runtime[IN45R];
					infoup(x,y,head2,-1,0,4);
				}
				head = 4;
			}
			if (x - 1 >= MI && y - 1 >= MI && (maze[y][x - 1] & 0x06) == 0x00 && (maze[y - 1][x - 1] & 0x08) == 0x00) {
				if (nodeinfo[x - 1][y - 1][7].node1> nodeinfo[x][y][3].node1+ runtime[IN45L]) {
					nodeinfo[x - 1][y - 1][7].node1= nodeinfo[x][y][3].node1+ runtime[IN45L];
					infoup(x,y,head2,-1,-1,7);
				}
				head = 7;
			}
			//in135
			if (x - 1 >= MI && y + 1 <= MA && (maze[y][x - 1] & 0x03) == 0x00 && (maze[y + 1][x] & 0x09) == 0x00) {
				if (nodeinfo[x - 1][y + 1][9].node1> nodeinfo[x][y][3].node1+ runtime[IN135R]) {
					nodeinfo[x - 1][y + 1][9].node1= nodeinfo[x][y][3].node1+ runtime[IN135R];
					infoup(x,y,head2,-1,1,9);
				}
				head = 9;
			}
			if (x - 1 >= MI && y - 1 >= MI && (maze[y][x - 1] & 0x06) == 0x00 && (maze[y - 1][x] & 0x0C) == 0x00) {
				if (nodeinfo[x - 1][y - 1][10].node1> nodeinfo[x][y][3].node1+ runtime[IN135L]) {
					nodeinfo[x - 1][y - 1][10].node1= nodeinfo[x][y][3].node1+ runtime[IN135L];
					infoup(x,y,head2,-1,-1,10);
				}
				head = 10;
			}
		}
		else if (head == 4) {
			//斜め直進１～３１区画
			if (x - 1 >= MI && y + 1 <= MA && (maze[y + 1][x - 1] & 0x03) == 0x00 && nomorego == 0) {
				if (nodeinfo[x - 1][y + 1][8].node1> nodeinfo[x][y][4].node1+ runtime[DIA_GO1]) {
					nodeinfo[x - 1][y + 1][8].node1= nodeinfo[x][y][4].node1+ runtime[DIA_GO1];
					infoup(x,y,head2,-1,1,8);
				}
				head = 8;
			}
			else {
				nomorego = 1;
			}

			///////////////////////////////////////
			ppp = 0;
			for(ppp = 2; ppp < 64; ppp++){
				if((ppp%2) == 1){//3,5,7,9,11...区画
					if (x - ((ppp+1)/2) >= MI && y + ((ppp+1)/2) <= MA && (maze[y + ((ppp+1)/2)][x - ((ppp+1)/2)] & 0x01) == 0x00 && nomorego == 0) {
						if (nodeinfo[x - ((ppp+1)/2)][y + ((ppp+1)/2)][8].node1> nodeinfo[x][y][4].node1+ runtime[DIA_GO1-1+ppp]) {
							nodeinfo[x - ((ppp+1)/2)][y + ((ppp+1)/2)][8].node1= nodeinfo[x][y][4].node1+ runtime[DIA_GO1-1+ppp];
							infoup(x,y,head2,-((ppp+1)/2),((ppp+1)/2),8);
						}
						head = 8;
					}
					else {
						nomorego = 1;
						ppp = 64;
					}
				}
				else if((ppp%2) == 0){//2,4,6,8,10...区画
					if (x - (ppp/2) >= MI && y + ((ppp/2)+1) <= MA && (maze[y + ((ppp/2)+1)][x - (ppp/2)] & 0x08) == 0x00 && nomorego == 0) {
						if (nodeinfo[x - (ppp/2)][y + (ppp/2)][4].node1> nodeinfo[x][y][4].node1+ runtime[DIA_GO1-1+ppp]) {
							nodeinfo[x - (ppp/2)][y + (ppp/2)][4].node1= nodeinfo[x][y][4].node1+ runtime[DIA_GO1-1+ppp];
							infoup(x,y,head2,-(ppp/2),(ppp/2),4);
						}
						head = 4;
					}
					else {
						nomorego = 1;
						ppp = 64;
					}
				}
			}
			///////////////////////////////////////

			//out45　右なし
		/*	if ((maze[y + 1][x] & 0x06) == 0x00 && (maze[y + 1][x + 1] & 0x01) == 0x00) {
				if (nodeinfo[x][y + 1][9].node1> nodeinfo[x][y][0].node1+ runtime[IN45R]) {
					nodeinfo[x][y + 1][9].node1= nodeinfo[x][y][0].node1+ runtime[IN45R];
				}
				head = 9;
			}*/
			if (x - 1 >= MI && y + 1 <= MA && (maze[y + 1][x - 1] & 0x0A) == 0x00) {
				if (nodeinfo[x - 1][y + 1][3].node1> nodeinfo[x][y][4].node1+ runtime[OUT45L]) {
					nodeinfo[x - 1][y + 1][3].node1= nodeinfo[x][y][4].node1+ runtime[OUT45L];
					infoup(x,y,head2,-1,1,3);
				}
				head = 3;
			}
			//out135　右なし
		/*	if ((maze[y + 1][x] & 0x06) == 0x00 && (maze[y][x + 1] & 0x03) == 0x00) {
				if (nodeinfo[x + 1][y][6].node1> nodeinfo[x][y][0].node1+ runtime[IN135R]) {
					nodeinfo[x + 1][y][6].node1= nodeinfo[x][y][0].node1+ runtime[IN135R];
				}
				head = 6;
			}*/
			if (x - 1 >= MI && y + 1 <= MA && (maze[y + 1][x - 1] & 0x06) == 0x00 && (maze[y][x - 1] & 0x04) == 0x00) {
				if (nodeinfo[x - 1][y][2].node1> nodeinfo[x][y][4].node1+ runtime[OUT135L]) {
					nodeinfo[x - 1][y][2].node1= nodeinfo[x][y][4].node1+ runtime[OUT135L];
					infoup(x,y,head2,-1,0,2);
				}
				head = 2;
			}
			//v90　右なし
			if (x - 1 >= MI && y + 1 <= MA && (maze[y + 1][x - 1] & 0x06) == 0x00 && (maze[y][x - 1] & 0x08) == 0x00) {
				if (nodeinfo[x - 1][y][7].node1> nodeinfo[x][y][4].node1+ runtime[V90L]) {
					nodeinfo[x - 1][y][7].node1= nodeinfo[x][y][4].node1+ runtime[V90L];
					infoup(x,y,head2,-1,0,7);
				}
				head = 7;
			}
			//こじまターン
			if(kojima_enbl == 1){	//条件きつめ
				if (x - 2 >= MI && y + 1 <= MA && (maze[y + 1][x - 1] & 0x0B) == 0x00 && (maze[y][x - 2] & 0x09) == 0x00) {
					if (nodeinfo[x - 2][y][7].node1> nodeinfo[x][y][4].node1+ runtime[KOJIMAL]) {
						nodeinfo[x - 2][y][7].node1= nodeinfo[x][y][4].node1+ runtime[KOJIMAL];
						infoup(x,y,head2,-2,0,7);
					}
					head = 7;
				}
			}
			else if(kojima_enbl == 2){
				if (x - 2 >= MI && y + 1 <= MA && (maze[y + 1][x - 1] & 0x0A) == 0x00 && (maze[y][x - 2] & 0x09) == 0x00) {
					if (nodeinfo[x - 2][y][7].node1> nodeinfo[x][y][4].node1+ runtime[KOJIMAL]) {
						nodeinfo[x - 2][y][7].node1= nodeinfo[x][y][4].node1+ runtime[KOJIMAL];
						infoup(x,y,head2,-2,0,7);
					}
					head = 7;
				}
			}
		}
		else if (head == 5) {
			//斜め直進１～３１区画
			if (x + 1 <= MA && y + 1 <= MA && (maze[y + 1][x + 1] & 0x09) == 0x00 && nomorego == 0) {
				if (nodeinfo[x][y + 1][9].node1> nodeinfo[x][y][5].node1+ runtime[DIA_GO1]) {
					nodeinfo[x][y + 1][9].node1= nodeinfo[x][y][5].node1+ runtime[DIA_GO1];
					infoup(x,y,head2,0,1,9);
				}
				head = 9;
			}
			else {
				nomorego = 1;
			}
			///////////////////////////////////////
			ppp = 0;
			for(ppp = 2; ppp < 64; ppp++){
				if((ppp%2) == 1){//3,5,7,9,11...区画
					if (x + ((ppp+1)/2) <= MA && y + ((ppp+1)/2) <= MA && (maze[y + ((ppp+1)/2)][x + ((ppp+1)/2)] & 0x01) == 0x00 && nomorego == 0) {
						if (nodeinfo[x + ((ppp-1)/2)][y + ((ppp+1)/2)][9].node1> nodeinfo[x][y][5].node1+ runtime[DIA_GO1-1+ppp]) {
							nodeinfo[x + ((ppp-1)/2)][y + ((ppp-1)/2+1)][9].node1= nodeinfo[x][y][5].node1+ runtime[DIA_GO1-1+ppp];
							infoup(x,y,head2,((ppp-1)/2),((ppp-1)/2+1),9);
						}
						head = 9;
					}
					else {
						nomorego = 1;
						ppp = 64;
					}
				}
				else if((ppp%2) == 0){//2,4,6,8,10...区画
					if (x + (ppp/2) <= MA && y + ((ppp/2)+1) <= MA && (maze[y + ((ppp/2)+1)][x + (ppp/2)] & 0x02) == 0x00 && nomorego == 0) {
						if (nodeinfo[x + (ppp/2)][y + (ppp/2)][5].node1> nodeinfo[x][y][5].node1+ runtime[DIA_GO1-1+ppp]) {
							nodeinfo[x + (ppp/2)][y + (ppp/2)][5].node1= nodeinfo[x][y][5].node1+ runtime[DIA_GO1-1+ppp];
							infoup(x,y,head2,(ppp/2),(ppp/2),5);
						}
						head = 5;
					}
					else {
						nomorego = 1;
						ppp = 64;
					}
				}
			}
			///////////////////////////////////////

			//out45　左なし
			if (x + 1 <= MA && y + 1 <= MA && (maze[y+1][x + 1] & 0x0A) == 0x00) {
				if (nodeinfo[x + 1][y + 1][1].node1> nodeinfo[x][y][5].node1+ runtime[OUT45R]) {
					nodeinfo[x + 1][y + 1][1].node1= nodeinfo[x][y][5].node1+ runtime[OUT45R];
					infoup(x,y,head2,1,1,1);
				}
				head = 1;
			}
			/*	if ((maze[y + 1][x - 1] & 0x0A) == 0x00) {
					if (nodeinfo[x - 1][y + 1][3].node1> nodeinfo[x][y][4].node1+ runtime[OUT45L]) {
						nodeinfo[x - 1][y + 1][3].node1= nodeinfo[x][y][4].node1+ runtime[OUT45L];
					}
					head = 3;
				}*/
				//out135　左なし
			if (x + 1 <= MA && y + 1 <= MA && (maze[y + 1][x + 1] & 0x0C) == 0x00 && (maze[y][x + 1] & 0x04) == 0x00) {
				if (nodeinfo[x + 1][y][2].node1> nodeinfo[x][y][5].node1+ runtime[OUT135R]) {
					nodeinfo[x + 1][y][2].node1= nodeinfo[x][y][5].node1+ runtime[OUT135R];
					infoup(x,y,head2,1,0,2);
				}
				head = 2;
			}
			/*	if ((maze[y + 1][x - 1] & 0x06) == 0x00 && (maze[y][x - 1] & 0x04) == 0x00) {
					if (nodeinfo[x - 1][y][2].node1> nodeinfo[x][y][4].node1+ runtime[OUT135L]) {
						nodeinfo[x - 1][y][2].node1= nodeinfo[x][y][4].node1+ runtime[OUT135L];
					}
					head = 2;
				}*/
				//v90　左なし
			if (x + 1 <= MA && y + 1 <= MA && (maze[y + 1][x + 1] & 0x0C) == 0x00 && (maze[y][x + 1] & 0x02) == 0x00) {
				if (nodeinfo[x + 1][y][6].node1> nodeinfo[x][y][5].node1+ runtime[V90R]) {
					nodeinfo[x + 1][y][6].node1= nodeinfo[x][y][5].node1+ runtime[V90R];
					infoup(x,y,head2,1,0,6);
				}
				head = 6;
			}
			//こじまターン
			if(kojima_enbl == 1){
				if (x + 2 <= MA && y + 1 <= MA && (maze[y + 1][x + 1] & 0x0B) == 0x00 && (maze[y][x + 2] & 0x03) == 0x00) {
					if (nodeinfo[x + 2][y][6].node1> nodeinfo[x][y][5].node1+ runtime[KOJIMAR]) {
						nodeinfo[x + 2][y][6].node1= nodeinfo[x][y][5].node1+ runtime[KOJIMAR];
						infoup(x,y,head2,2,0,6);
					}
					head = 6;
				}
			}
			else if(kojima_enbl == 2){
				if (x + 2 <= MA && y + 1 <= MA && (maze[y + 1][x + 1] & 0x0A) == 0x00 && (maze[y][x + 2] & 0x03) == 0x00) {
					if (nodeinfo[x + 2][y][6].node1> nodeinfo[x][y][5].node1+ runtime[KOJIMAR]) {
						nodeinfo[x + 2][y][6].node1= nodeinfo[x][y][5].node1+ runtime[KOJIMAR];
						infoup(x,y,head2,2,0,6);
					}
					head = 6;
				}
			}
		}
		else if (head == 6) {
			//斜め直進１～３１区画
			if (x + 1 <= MA && (maze[y][x + 1] & 0x0C) == 0x00 && nomorego == 0) {
				if (nodeinfo[x][y][10].node1> nodeinfo[x][y][6].node1+ runtime[DIA_GO1]) {
					nodeinfo[x][y][10].node1= nodeinfo[x][y][6].node1+ runtime[DIA_GO1];
					infoup(x,y,head2,0,0,10);
				}
				head = 10;
			}
			else {
				nomorego = 1;
			}
			///////////////////////////////////////
			ppp = 0;
			for(ppp = 2; ppp < 64; ppp++){
				if((ppp%2) == 1){//3,5,7,9,11...区画
					if (x + ((ppp+1)/2) <= MA && y - ((ppp+1)/2-1) >= MI && (maze[y - ((ppp+1)/2-1)][x + ((ppp+1)/2)] & 0x04) == 0x00 && nomorego == 0) {
						if (nodeinfo[x + ((ppp-1)/2)][y - ((ppp+1)/2-1)][10].node1> nodeinfo[x][y][6].node1+ runtime[DIA_GO1-1+ppp]) {
							nodeinfo[x + ((ppp-1)/2)][y - ((ppp-1)/2)][10].node1= nodeinfo[x][y][6].node1+ runtime[DIA_GO1-1+ppp];
							infoup(x,y,head2,((ppp-1)/2),-((ppp-1)/2),10);
						}
						head = 10;
					}
					else {
						nomorego = 1;
						ppp = 64;
					}
				}
				else if((ppp%2) == 0){//2,4,6,8,10...区画
					if (x + (ppp/2) <= MA && y - (ppp/2) >= MI && (maze[y - (ppp/2)][x + (ppp/2)] & 0x02) == 0x00 && nomorego == 0) {
						if (nodeinfo[x + (ppp/2)][y - (ppp/2)][6].node1> nodeinfo[x][y][6].node1+ runtime[DIA_GO1-1+ppp]) {
							nodeinfo[x + (ppp/2)][y - (ppp/2)][6].node1= nodeinfo[x][y][6].node1+ runtime[DIA_GO1-1+ppp];
							infoup(x,y,head2,(ppp/2),-(ppp/2),6);
						}
						head = 6;
					}
					else {
						nomorego = 1;
						ppp = 64;
					}
				}
			}
			///////////////////////////////////////

			//out45　右なし
		/*	if ((maze[y + 1][x] & 0x06) == 0x00 && (maze[y + 1][x + 1] & 0x01) == 0x00) {
				if (nodeinfo[x][y + 1][9].node1> nodeinfo[x][y][0].node1+ runtime[IN45R]) {
					nodeinfo[x][y + 1][9].node1= nodeinfo[x][y][0].node1+ runtime[IN45R];
				}
				head = 9;
			}*/
			if (x + 1 <= MA && (maze[y][x + 1] & 0x0A) == 0x00) {
				if (nodeinfo[x + 1][y][1].node1> nodeinfo[x][y][6].node1+ runtime[OUT45L]) {
					nodeinfo[x + 1][y][1].node1= nodeinfo[x][y][6].node1+ runtime[OUT45L];
					infoup(x,y,head2,1,0,1);
				}
				head = 1;
			}
			//out135　右なし
		/*	if ((maze[y + 1][x] & 0x06) == 0x00 && (maze[y][x + 1] & 0x03) == 0x00) {
				if (nodeinfo[x + 1][y][6].node1> nodeinfo[x][y][0].node1+ runtime[IN135R]) {
					nodeinfo[x + 1][y][6].node1= nodeinfo[x][y][0].node1+ runtime[IN135R];
				}
				head = 6;
			}*/
			if (x + 1 <= MA && y + 1 <= MA && (maze[y][x + 1] & 0x09) == 0x00 && (maze[y + 1][x + 1] & 0x01) == 0x00) {
				if (nodeinfo[x + 1][y + 1][0].node1> nodeinfo[x][y][6].node1+ runtime[OUT135L]) {
					nodeinfo[x + 1][y + 1][0].node1= nodeinfo[x][y][6].node1+ runtime[OUT135L];
					infoup(x,y,head2,1,1,0);
				}
				head = 0;
			}
			//v90　右なし
			if (x + 1 <= MA && y + 1 <= MA && (maze[y][x + 1] & 0x09) == 0x00 && (maze[y + 1][x + 1] & 0x02) == 0x00) {
				if (nodeinfo[x + 1][y][5].node1> nodeinfo[x][y][6].node1+ runtime[V90L]) {
					nodeinfo[x + 1][y][5].node1= nodeinfo[x][y][6].node1+ runtime[V90L];
					infoup(x,y,head2,1,0,5);
				}
				head = 5;
			}
			//こじまターン
			if(kojima_enbl == 1){
				if (x + 2 <= MA && y + 1 <= MA && (maze[y][x + 1] & 0x0E) == 0x00 && (maze[y+1][x + 2] & 0x06) == 0x00) {
					if (nodeinfo[x + 2][y][5].node1> nodeinfo[x][y][6].node1+ runtime[KOJIMAL]) {
						nodeinfo[x + 2][y][5].node1= nodeinfo[x][y][6].node1+ runtime[KOJIMAL];
						infoup(x,y,head2,2,0,5);
					}
					head = 5;
				}

			}
			else if(kojima_enbl == 2){
				if (x + 2 <= MA && y + 1 <= MA && (maze[y][x + 1] & 0x0A) == 0x00 && (maze[y+1][x + 2] & 0x06) == 0x00) {
					if (nodeinfo[x + 2][y][5].node1> nodeinfo[x][y][6].node1+ runtime[KOJIMAL]) {
						nodeinfo[x + 2][y][5].node1= nodeinfo[x][y][6].node1+ runtime[KOJIMAL];
						infoup(x,y,head2,2,0,5);
					}
					head = 5;
				}
			}
		}
		else if (head == 7) {
			//斜め直進１～３１区画
			if (x - 1 >= MI && (maze[y][x - 1] & 0x06) == 0x00 && nomorego == 0) {
				if (nodeinfo[x - 1][y][11].node1> nodeinfo[x][y][7].node1+ runtime[DIA_GO1]) {
					nodeinfo[x - 1][y][11].node1= nodeinfo[x][y][7].node1+ runtime[DIA_GO1];
					infoup(x,y,head2,-1,0,11);
				}
				head = 11;
			}
			else {
				nomorego = 1;
			}
			///////////////////////////////////////
			ppp = 0;
			for(ppp = 2; ppp < 64; ppp++){
				if((ppp%2) == 1){//3,5,7,9,11...区画
					if (x - ((ppp+1)/2) >= MI && y - ((ppp+1)/2-1) >= MI && (maze[y - ((ppp+1)/2-1)][x - ((ppp+1)/2)] & 0x04) == 0x00 && nomorego == 0) {
						if (nodeinfo[x - ((ppp+1)/2)][y - ((ppp+1)/2-1)][11].node1> nodeinfo[x][y][7].node1+ runtime[DIA_GO1-1+ppp]) {
							nodeinfo[x - ((ppp+1)/2)][y - ((ppp+1)/2-1)][11].node1= nodeinfo[x][y][7].node1+ runtime[DIA_GO1-1+ppp];
							infoup(x,y,head2,-((ppp+1)/2),-((ppp+1)/2-1),11);
						}
						head = 11;
					}
					else {
						nomorego = 1;
						ppp = 64;
					}
				}
				else if((ppp%2) == 0){//2,4,6,8,10...区画
					if (x - (ppp/2) >= MI && y - (ppp/2) >= MI && (maze[y - (ppp/2)][x - (ppp/2)] & 0x08) == 0x00 && nomorego == 0) {
						if (nodeinfo[x - (ppp/2)][y - (ppp/2)][7].node1> nodeinfo[x][y][7].node1+ runtime[DIA_GO1-1+ppp]) {
							nodeinfo[x - (ppp/2)][y - (ppp/2)][7].node1= nodeinfo[x][y][7].node1+ runtime[DIA_GO1-1+ppp];
							infoup(x,y,head2,-(ppp/2),-(ppp/2),7);
						}
						head = 7;
					}
					else {
						nomorego = 1;
						ppp = 64;
					}
				}
			}
			///////////////////////////////////////

			//out45　左なし
			if (x - 1 >= MI && (maze[y][x - 1] & 0x0A) == 0x00) {
				if (nodeinfo[x - 1][y][3].node1> nodeinfo[x][y][7].node1+ runtime[OUT45R]) {
					nodeinfo[x - 1][y][3].node1= nodeinfo[x][y][7].node1+ runtime[OUT45R];
					infoup(x,y,head2,-1,0,3);
				}
				head = 3;
			}
			/*	if ((maze[y + 1][x - 1] & 0x0A) == 0x00) {
					if (nodeinfo[x - 1][y + 1][3].node1> nodeinfo[x][y][4].node1+ runtime[OUT45L]) {
						nodeinfo[x - 1][y + 1][3].node1= nodeinfo[x][y][4].node1+ runtime[OUT45L];
					}
					head = 3;
				}*/
				//out135　左なし
			if (x - 1 >= MI && y + 1 <= MA && (maze[y][x - 1] & 0x03) == 0x00 && (maze[y + 1][x - 1] & 0x01) == 0x00) {
				if (nodeinfo[x - 1][y + 1][0].node1> nodeinfo[x][y][7].node1+ runtime[OUT135R]) {
					nodeinfo[x - 1][y + 1][0].node1= nodeinfo[x][y][7].node1+ runtime[OUT135R];
					infoup(x,y,head2,-1,1,0);
				}
				head = 0;
			}
			/*	if ((maze[y + 1][x - 1] & 0x06) == 0x00 && (maze[y][x - 1] & 0x04) == 0x00) {
					if (nodeinfo[x - 1][y][2].node1> nodeinfo[x][y][4].node1+ runtime[OUT135L]) {
						nodeinfo[x - 1][y][2].node1= nodeinfo[x][y][4].node1+ runtime[OUT135L];
					}
					head = 2;
				}*/
				//v90　左なし
			if (x - 1 >= MI && y + 1 <= MA && (maze[y][x - 1] & 0x03) == 0x00 && (maze[y + 1][x - 1] & 0x08) == 0x00) {
				if (nodeinfo[x - 1][y][4].node1> nodeinfo[x][y][7].node1+ runtime[V90R]) {
					nodeinfo[x - 1][y][4].node1= nodeinfo[x][y][7].node1+ runtime[V90R];
					infoup(x,y,head2,-1,0,4);
				}
				head = 4;
			}
			//こじまターン
			if(kojima_enbl == 1){
				if (x - 2 >= MI && y + 1 <= MA && (maze[y][x - 1] & 0x0E) == 0x00 && (maze[y+1][x - 2] & 0x0C) == 0x00) {
					if (nodeinfo[x - 2][y][4].node1> nodeinfo[x][y][7].node1+ runtime[KOJIMAR]) {
						nodeinfo[x - 2][y][4].node1= nodeinfo[x][y][7].node1+ runtime[KOJIMAR];
						infoup(x,y,head2,-2,0,4);
					}
					head = 4;
				}
			}
			else if(kojima_enbl == 2){
				if (x - 2 >= MI && y + 1 <= MA && (maze[y][x - 1] & 0x0A) == 0x00 && (maze[y+1][x - 2] & 0x0C) == 0x00) {
					if (nodeinfo[x - 2][y][4].node1> nodeinfo[x][y][7].node1+ runtime[KOJIMAR]) {
						nodeinfo[x - 2][y][4].node1= nodeinfo[x][y][7].node1+ runtime[KOJIMAR];
						infoup(x,y,head2,-2,0,4);
					}
					head = 4;
				}
			}
		}
		else if (head == 8) {
			//斜め直進１～３１区画
			if (y + 1 <= MA && (maze[y + 1][x] & 0x0C) == 0x00 && nomorego == 0) {
				if (nodeinfo[x][y][4].node1> nodeinfo[x][y][8].node1+ runtime[DIA_GO1]) {
					nodeinfo[x][y][4].node1= nodeinfo[x][y][8].node1+ runtime[DIA_GO1];
					infoup(x,y,head2,0,0,4);
				}
				head = 4;
			}
			else {
				nomorego = 1;
			}
			///////////////////////////////////////
			ppp = 0;
			for(ppp = 2; ppp < 64; ppp++){
				if((ppp%2) == 1){//3,5,7,9,11...区画
					if (x - ((ppp-1)/2) >= MI && y + ((ppp-1)/2+1) <= MA && (maze[y + ((ppp-1)/2+1)][x - ((ppp-1)/2)] & 0x08) == 0x00 && nomorego == 0) {
						if (nodeinfo[x - ((ppp-1)/2)][y + ((ppp+1)/2-1)][4].node1> nodeinfo[x][y][8].node1+ runtime[DIA_GO1-1+ppp]) {
							nodeinfo[x - ((ppp-1)/2)][y + ((ppp-1)/2)][4].node1= nodeinfo[x][y][8].node1+ runtime[DIA_GO1-1+ppp];
							infoup(x,y,head2,-((ppp-1)/2),((ppp-1)/2),4);
						}
						head = 4;
					}
					else {
						nomorego = 1;
						ppp = 64;
					}
				}
				else if((ppp%2) == 0){//2,4,6,8,10...区画
					if (x - (ppp/2) >= MI && y + (ppp/2) <= MA && (maze[y + (ppp/2)][x - (ppp/2)] & 0x01) == 0x00 && nomorego == 0) {
						if (nodeinfo[x - (ppp/2)][y + (ppp/2)][8].node1> nodeinfo[x][y][8].node1+ runtime[DIA_GO1-1+ppp]) {
							nodeinfo[x - (ppp/2)][y + (ppp/2)][8].node1= nodeinfo[x][y][8].node1+ runtime[DIA_GO1-1+ppp];
							infoup(x,y,head2,-(ppp/2),(ppp/2),8);
						}
						head = 8;
					}
					else {
						nomorego = 1;
						ppp = 64;
					}
				}
			}
			///////////////////////////////////////

			//out45　左なし
			if (y + 1 <= MA && (maze[y + 1][x] & 0x05) == 0x00) {
				if (nodeinfo[x][y + 1][0].node1> nodeinfo[x][y][8].node1+ runtime[OUT45R]) {
					nodeinfo[x][y + 1][0].node1= nodeinfo[x][y][8].node1+ runtime[OUT45R];
					infoup(x,y,head2,0,1,0);
				}
				head = 0;
			}
			/*	if ((maze[y + 1][x - 1] & 0x0A) == 0x00) {
					if (nodeinfo[x - 1][y + 1][3].node1> nodeinfo[x][y][4].node1+ runtime[OUT45L]) {
						nodeinfo[x - 1][y + 1][3].node1= nodeinfo[x][y][4].node1+ runtime[OUT45L];
					}
					head = 3;
				}*/
				//out135　左なし
			if (x + 1 <= MA && y + 1 <= MA && (maze[y + 1][x] & 0x06) == 0x00 && (maze[y + 1][x + 1] & 0x02) == 0x00) {
				if (nodeinfo[x + 1][y + 1][1].node1> nodeinfo[x][y][8].node1+ runtime[OUT135R]) {
					nodeinfo[x + 1][y + 1][1].node1= nodeinfo[x][y][8].node1+ runtime[OUT135R];
					infoup(x,y,head2,1,1,1);
				}
				head = 1;
			}
			/*	if ((maze[y + 1][x - 1] & 0x06) == 0x00 && (maze[y][x - 1] & 0x04) == 0x00) {
					if (nodeinfo[x - 1][y][2].node1> nodeinfo[x][y][4].node1+ runtime[OUT135L]) {
						nodeinfo[x - 1][y][2].node1= nodeinfo[x][y][4].node1+ runtime[OUT135L];
					}
					head = 2;
				}*/
				//v90　左なし
			if (x + 1 <= MA && y + 1 <= MA && (maze[y + 1][x] & 0x06) == 0x00 && (maze[y + 1][x + 1] & 0x01) == 0x00) {
				if (nodeinfo[x][y + 1][9].node1> nodeinfo[x][y][8].node1+ runtime[V90R]) {
					nodeinfo[x][y + 1][9].node1= nodeinfo[x][y][8].node1+ runtime[V90R];
					infoup(x,y,head2,0,1,9);
				}
				head = 9;
			}
			//こじまターン
			if(kojima_enbl == 1){
				if (x + 1 <= MA && y + 2 <= MA && (maze[y + 1][x] & 0x0D) == 0x00 && (maze[y+2][x + 1] & 0x09) == 0x00) {
					if (nodeinfo[x][y+2][9].node1> nodeinfo[x][y][8].node1+ runtime[KOJIMAR]) {
						nodeinfo[x][y+2][9].node1= nodeinfo[x][y][8].node1+ runtime[KOJIMAR];
						infoup(x,y,head2,0,2,9);
					}
					head = 9;
				}
			}
			else if(kojima_enbl == 2){
				if (x + 1 <= MA && y + 2 <= MA && (maze[y + 1][x] & 0x05) == 0x00 && (maze[y+2][x + 1] & 0x09) == 0x00) {
					if (nodeinfo[x][y+2][9].node1> nodeinfo[x][y][8].node1+ runtime[KOJIMAR]) {
						nodeinfo[x][y+2][9].node1= nodeinfo[x][y][8].node1+ runtime[KOJIMAR];
						infoup(x,y,head2,0,2,9);
					}
					head = 9;
				}
			}
		}
		else if (head == 9) {
			//斜め直進１～３１区画
			if (x + 1 <= MA && y + 1 <= MA && (maze[y + 1][x + 1] & 0x06) == 0x00 && nomorego == 0) {
				if (nodeinfo[x + 1][y][5].node1> nodeinfo[x][y][9].node1+ runtime[DIA_GO1]) {
					nodeinfo[x + 1][y][5].node1= nodeinfo[x][y][9].node1+ runtime[DIA_GO1];
					infoup(x,y,head2,1,0,5);
				}
				head = 5;
			}
			else {
				nomorego = 1;
			}
			///////////////////////////////////////
			ppp = 0;
			for(ppp = 2; ppp < 64; ppp++){
				if((ppp%2) == 1){//3,5,7,9,11...区画
					if (x + ((ppp+1)/2) <= MA && y + ((ppp+1)/2) <= MA && (maze[y + ((ppp+1)/2)][x + ((ppp+1)/2)] & 0x02) == 0x00 && nomorego == 0) {
						if (nodeinfo[x + ((ppp+1)/2)][y + ((ppp+1)/2-1)][5].node1> nodeinfo[x][y][9].node1+ runtime[DIA_GO1-1+ppp]) {
							nodeinfo[x + ((ppp+1)/2)][y + ((ppp+1)/2-1)][5].node1= nodeinfo[x][y][9].node1+ runtime[DIA_GO1-1+ppp];
							infoup(x,y,head2,((ppp+1)/2),((ppp+1)/2-1),5);
						}
						head = 5;
					}
					else {
						nomorego = 1;
						ppp = 64;
					}
				}
				else if((ppp%2) == 0){//2,4,6,8,10...区画
					if (x + (ppp/2+1) <= MA && y + (ppp/2) <= MA && (maze[y + (ppp/2)][x + (ppp/2+1)] & 0x01) == 0x00 && nomorego == 0) {
						if (nodeinfo[x + (ppp/2)][y + (ppp/2)][9].node1> nodeinfo[x][y][9].node1+ runtime[DIA_GO1-1+ppp]) {
							nodeinfo[x + (ppp/2)][y + (ppp/2)][9].node1= nodeinfo[x][y][9].node1+ runtime[DIA_GO1-1+ppp];
							infoup(x,y,head2,(ppp/2),(ppp/2),9);
						}
						head = 9;
					}
					else {
						nomorego = 1;
						ppp = 64;
					}
				}
			}
			///////////////////////////////////////

			//out45　右なし
		/*	if ((maze[y + 1][x] & 0x06) == 0x00 && (maze[y + 1][x + 1] & 0x01) == 0x00) {
				if (nodeinfo[x][y + 1][9].node1> nodeinfo[x][y][0].node1+ runtime[IN45R]) {
					nodeinfo[x][y + 1][9].node1= nodeinfo[x][y][0].node1+ runtime[IN45R];
				}
				head = 9;
			}*/
			if (x + 1 <= MA && y + 1 <= MA && (maze[y + 1][x + 1] & 0x05) == 0x00) {
				if (nodeinfo[x + 1][y + 1][0].node1> nodeinfo[x][y][9].node1+ runtime[OUT45L]) {
					nodeinfo[x + 1][y + 1][0].node1= nodeinfo[x][y][9].node1+ runtime[OUT45L];
					infoup(x,y,head2,1,1,0);
				}
				head = 0;
			}
			//out135　右なし
		/*	if ((maze[y + 1][x] & 0x06) == 0x00 && (maze[y][x + 1] & 0x03) == 0x00) {
				if (nodeinfo[x + 1][y][6].node1> nodeinfo[x][y][0].node1+ runtime[IN135R]) {
					nodeinfo[x + 1][y][6].node1= nodeinfo[x][y][0].node1+ runtime[IN135R];
				}
				head = 6;
			}*/
			if (x + 1 <= MA && y + 1 <= MA && (maze[y + 1][x + 1] & 0x0C) == 0x00 && (maze[y + 1][x] & 0x08) == 0x00) {
				if (nodeinfo[x][y + 1][3].node1> nodeinfo[x][y][9].node1+ runtime[OUT135L]) {
					nodeinfo[x][y + 1][3].node1= nodeinfo[x][y][9].node1+ runtime[OUT135L];
					infoup(x,y,head2,0,1,3);
				}
				head = 3;
			}
			//v90　右なし
			if (x + 1 <= MA && y + 1 <= MA && (maze[y + 1][x + 1] & 0x0C) == 0x00 && (maze[y + 1][x] & 0x01) == 0x00) {
				if (nodeinfo[x][y + 1][8].node1> nodeinfo[x][y][9].node1+ runtime[V90L]) {
					nodeinfo[x][y + 1][8].node1= nodeinfo[x][y][9].node1+ runtime[V90L];
					infoup(x,y,head2,0,1,8);
				}
				head = 8;
			}
			//こじまターン
			if(kojima_enbl == 1){
				if (x + 1 <= MA && y + 2 <= MA && (maze[y + 1][x + 1] & 0x07) == 0x00 && (maze[y+2][x] & 0x03) == 0x00) {
					if (nodeinfo[x][y+2][8].node1> nodeinfo[x][y][9].node1+ runtime[KOJIMAL]) {
						nodeinfo[x][y+2][8].node1= nodeinfo[x][y][9].node1+ runtime[KOJIMAL];
						infoup(x,y,head2,0,2,8);
					}
					head = 8;
				}
			}
			else if(kojima_enbl == 2){
				if (x + 1 <= MA && y + 2 <= MA && (maze[y + 1][x + 1] & 0x05) == 0x00 && (maze[y+2][x] & 0x03) == 0x00) {
					if (nodeinfo[x][y+2][8].node1> nodeinfo[x][y][9].node1+ runtime[KOJIMAL]) {
						nodeinfo[x][y+2][8].node1= nodeinfo[x][y][9].node1+ runtime[KOJIMAL];
						infoup(x,y,head2,0,2,8);
					}
					head = 8;
				}
			}
		}
		else if (head == 10) {
			//斜め直進１～３１区画
			if (x + 1 <= MA && y - 1 >= MI && (maze[y - 1][x + 1] & 0x03) == 0x00 && nomorego == 0) {
				if (nodeinfo[x + 1][y - 1][6].node1> nodeinfo[x][y][10].node1+ runtime[DIA_GO1]) {
					nodeinfo[x + 1][y - 1][6].node1= nodeinfo[x][y][10].node1+ runtime[DIA_GO1];
					infoup(x,y,head2,1,-1,6);
				}
				head = 6;
			}
			else {
				nomorego = 1;
			}
			///////////////////////////////////////
			ppp = 0;
			for(ppp = 2; ppp < 64; ppp++){
				if((ppp%2) == 1){//3,5,7,9,11...区画
					if (x + ((ppp+1)/2) <= MA && y - ((ppp+1)/2) >= MI && (maze[y - ((ppp+1)/2)][x + ((ppp+1)/2)] & 0x02) == 0x00 && nomorego == 0) {
						if (nodeinfo[x + ((ppp+1)/2)][y - ((ppp+1)/2)][6].node1> nodeinfo[x][y][10].node1+ runtime[DIA_GO1-1+ppp]) {
							nodeinfo[x + ((ppp+1)/2)][y - ((ppp+1)/2)][6].node1= nodeinfo[x][y][10].node1+ runtime[DIA_GO1-1+ppp];
							infoup(x,y,head2,((ppp+1)/2),-((ppp+1)/2),6);
						}
						head = 6;
					}
					else {
						nomorego = 1;
						ppp = 64;
					}
				}
				else if((ppp%2) == 0){//2,4,6,8,10...区画
					if (x + (ppp/2+1) <= MA && y - (ppp/2) >= MI && (maze[y - (ppp/2)][x + (ppp/2+1)] & 0x04) == 0x00 && nomorego == 0) {
						if (nodeinfo[x + (ppp/2)][y - (ppp/2)][10].node1> nodeinfo[x][y][10].node1+ runtime[DIA_GO1-1+ppp]) {
							nodeinfo[x + (ppp/2)][y - (ppp/2)][10].node1= nodeinfo[x][y][10].node1+ runtime[DIA_GO1-1+ppp];
							infoup(x,y,head2,(ppp/2),-(ppp/2),10);
						}
						head = 10;
					}
					else {
						nomorego = 1;
						ppp = 64;
					}
				}
			}
			///////////////////////////////////////

			//out45　左なし
			if (x + 1 <= MA && y - 1 >= MI && (maze[y - 1][x + 1] & 0x05) == 0x00) {
				if (nodeinfo[x + 1][y - 1][2].node1> nodeinfo[x][y][10].node1+ runtime[OUT45R]) {
					nodeinfo[x + 1][y - 1][2].node1= nodeinfo[x][y][10].node1+ runtime[OUT45R];
					infoup(x,y,head2,1,-1,2);
				}
				head = 2;
			}
			/*	if ((maze[y + 1][x - 1] & 0x0A) == 0x00) {
					if (nodeinfo[x - 1][y + 1][3].node1> nodeinfo[x][y][4].node1+ runtime[OUT45L]) {
						nodeinfo[x - 1][y + 1][3].node1= nodeinfo[x][y][4].node1+ runtime[OUT45L];
					}
					head = 3;
				}*/
				//out135　左なし
			if (x + 1 <= MA && y - 1 >= MI && (maze[y - 1][x + 1] & 0x09) == 0x00 && (maze[y - 1][x] & 0x08) == 0x00) {
				if (nodeinfo[x][y - 1][3].node1> nodeinfo[x][y][10].node1+ runtime[OUT135R]) {
					nodeinfo[x][y - 1][3].node1= nodeinfo[x][y][10].node1+ runtime[OUT135R];
					infoup(x,y,head2,0,-1,3);
				}
				head = 3;
			}
			/*	if ((maze[y + 1][x - 1] & 0x06) == 0x00 && (maze[y][x - 1] & 0x04) == 0x00) {
					if (nodeinfo[x - 1][y][2].node1> nodeinfo[x][y][4].node1+ runtime[OUT135L]) {
						nodeinfo[x - 1][y][2].node1= nodeinfo[x][y][4].node1+ runtime[OUT135L];
					}
					head = 2;
				}*/
				//v90　左なし
			if (x + 1 <= MA && y - 1 >= MI && (maze[y - 1][x + 1] & 0x09) == 0x00 && (maze[y - 1][x] & 0x04) == 0x00) {
				if (nodeinfo[x][y - 1][11].node1> nodeinfo[x][y][10].node1+ runtime[V90R]) {
					nodeinfo[x][y - 1][11].node1= nodeinfo[x][y][10].node1+ runtime[V90R];
					infoup(x,y,head2,0,-1,11);
				}
				head = 11;
			}
			//こじまターン
			if(kojima_enbl == 1){
				if (x +1 <= MA && y - 2 >= MI && (maze[y - 1][x + 1] & 0x07) == 0x00 && (maze[y-2][x] & 0x06) == 0x00) {
					if (nodeinfo[x][y-2][11].node1> nodeinfo[x][y][10].node1+ runtime[KOJIMAR]) {
						nodeinfo[x][y-2][11].node1= nodeinfo[x][y][10].node1+ runtime[KOJIMAR];
						infoup(x,y,head2,0,-2,11);
					}
					head = 11;
				}
			}
			else if(kojima_enbl == 2){
				if (x +1 <= MA && y - 2 >= MI && (maze[y - 1][x + 1] & 0x05) == 0x00 && (maze[y-2][x] & 0x06) == 0x00) {
					if (nodeinfo[x][y-2][11].node1> nodeinfo[x][y][10].node1+ runtime[KOJIMAR]) {
						nodeinfo[x][y-2][11].node1= nodeinfo[x][y][10].node1+ runtime[KOJIMAR];
						infoup(x,y,head2,1,-2,11);
					}
					head = 11;
				}
			}
		}
		else if (head == 11) {
			//斜め直進１～３１区画
			if (y - 1 >= MI && (maze[y - 1][x] & 0x09) == 0x00 && nomorego == 0) {
				if (nodeinfo[x][y - 1][7].node1> nodeinfo[x][y][11].node1+ runtime[DIA_GO1]) {
					nodeinfo[x][y - 1][7].node1= nodeinfo[x][y][11].node1+ runtime[DIA_GO1];
					infoup(x,y,head2,0,-1,7);
				}
				head = 7;
			}
			else {
				nomorego = 1;
			}
			///////////////////////////////////////
			ppp = 0;
			for(ppp = 2; ppp < 64; ppp++){
				if((ppp%2) == 1){//3,5,7,9,11...区画
					if (x - ((ppp-1)/2) >= MI && y - ((ppp-1)/2+1) >= MI && (maze[y - ((ppp-1)/2+1)][x - ((ppp-1)/2)] & 0x08) == 0x00 && nomorego == 0) {
						if (nodeinfo[x - ((ppp-1)/2)][y - ((ppp-1)/2+1)][7].node1> nodeinfo[x][y][11].node1+ runtime[DIA_GO1-1+ppp]) {
							nodeinfo[x - ((ppp-1)/2)][y - ((ppp-1)/2+1)][7].node1= nodeinfo[x][y][11].node1+ runtime[DIA_GO1-1+ppp];
							infoup(x,y,head2,-((ppp-1)/2),-((ppp-1)/2+1),7);
						}
						head = 7;
					}
					else {
						nomorego = 1;
						ppp = 64;
					}
				}
				else if((ppp%2) == 0){//2,4,6,8,10...区画
					if (x - (ppp/2) >= MI && y - (ppp/2) >= MI && (maze[y - (ppp/2)][x - (ppp/2)] & 0x04) == 0x00 && nomorego == 0) {
						if (nodeinfo[x - (ppp/2)][y - (ppp/2)][11].node1> nodeinfo[x][y][11].node1+ runtime[DIA_GO1-1+ppp]) {
							nodeinfo[x - (ppp/2)][y - (ppp/2)][11].node1= nodeinfo[x][y][11].node1+ runtime[DIA_GO1-1+ppp];
							infoup(x,y,head2,-(ppp/2),-(ppp/2),11);
						}
						head = 11;
					}
					else {
						nomorego = 1;
						ppp = 64;
					}
				}
			}
			///////////////////////////////////////

			//out45　右なし
		/*	if ((maze[y + 1][x] & 0x06) == 0x00 && (maze[y + 1][x + 1] & 0x01) == 0x00) {
				if (nodeinfo[x][y + 1][9].node1> nodeinfo[x][y][0].node1+ runtime[IN45R]) {
					nodeinfo[x][y + 1][9].node1= nodeinfo[x][y][0].node1+ runtime[IN45R];
				}
				head = 9;
			}*/
			if (y - 1 >= MI && (maze[y - 1][x] & 0x05) == 0x00) {
				if (nodeinfo[x][y - 1][2].node1> nodeinfo[x][y][11].node1+ runtime[OUT45L]) {
					nodeinfo[x][y - 1][2].node1= nodeinfo[x][y][11].node1+ runtime[OUT45L];
					infoup(x,y,head2,0,-1,2);
				}
				head = 2;
			}
			//out135　右なし
		/*	if ((maze[y + 1][x] & 0x06) == 0x00 && (maze[y][x + 1] & 0x03) == 0x00) {
				if (nodeinfo[x + 1][y][6].node1> nodeinfo[x][y][0].node1+ runtime[IN135R]) {
					nodeinfo[x + 1][y][6].node1= nodeinfo[x][y][0].node1+ runtime[IN135R];
				}
				head = 6;
			}*/
			if (x + 1 <= MA && y - 1 >= MI && (maze[y - 1][x] & 0x03) == 0x00 && (maze[y - 1][x + 1] & 0x02) == 0x00) {
				if (nodeinfo[x + 1][y - 1][1].node1> nodeinfo[x][y][11].node1+ runtime[OUT135L]) {
					nodeinfo[x + 1][y - 1][1].node1= nodeinfo[x][y][11].node1+ runtime[OUT135L];
					infoup(x,y,head2,1,-1,1);
				}
				head = 1;
			}
			//v90　右なし
			if (x + 1 <= MA && y - 1 >= MI && (maze[y - 1][x] & 0x03) == 0x00 && (maze[y - 1][x + 1] & 0x04) == 0x00) {
				if (nodeinfo[x][y - 1][10].node1> nodeinfo[x][y][11].node1+ runtime[V90L]) {
					nodeinfo[x][y - 1][10].node1= nodeinfo[x][y][11].node1+ runtime[V90L];
					infoup(x,y,head2,0,-1,10);
				}
				head = 10;
			}
			//こじまターン
			if(kojima_enbl == 1){
				if (x + 1 <= MA && y -2 >= MI && (maze[y - 1][x] & 0x0D) == 0x00 && (maze[y-2][x + 1] & 0x0C) == 0x00) {
					if (nodeinfo[x][y-2][10].node1> nodeinfo[x][y][11].node1+ runtime[KOJIMAL]) {
						nodeinfo[x][y-2][10].node1= nodeinfo[x][y][11].node1+ runtime[KOJIMAL];
						infoup(x,y,head2,0,-2,10);
					}
					head = 10;
				}
			}
			else if(kojima_enbl == 2){
				if (x + 1 <= MA && y -2 >= MI && (maze[y - 1][x] & 0x05) == 0x00 && (maze[y-2][x + 1] & 0x0C) == 0x00) {
					if (nodeinfo[x][y-2][10].node1> nodeinfo[x][y][11].node1+ runtime[KOJIMAL]) {
						nodeinfo[x][y-2][10].node1= nodeinfo[x][y][11].node1+ runtime[KOJIMAL];
						infoup(x,y,head2,0,-2,10);
					}
					head = 10;
				}
			}
		}
		else {
			break;
		}
		//
		nowshort = 55555;
		a2 = 0;
		b2 = 0;
		c2 = 0;

		for (a = 0; a < 32; a++) {
			for (b = 0; b < 32; b++) {
				for (c = 0; c < 12; c++) {
				//	if(c == 4 && (maze[b][a] & 0x01) == 0x01){
				//		c = 8;
				//	}
				//	else if(c == 8 && (maze[b][a] & 0x02) == 0x02){
				//		c = 12;
				//	}
				//	else{
					if(nodeinfo[a][b][c].node1 != 55555){
						if (nodeinfo[a][b][c].node1 < nowshort && nodeinfo[a][b][c].checked != 1) {
							nowshort = nodeinfo[a][b][c].node1;
							a2 = a;
							b2 = b;
							c2 = c;
						}
					}
				//	}
				}
			}
		}
		x = a2;
		y = b2;
		head = c2;
		head2 = head;
		//checknode[a2][b2][c2] = 1;
		nodeinfo[a2][b2][c2].checked = 1;
		//	fromx[a2][b2][c2] = a3;
		//	fromy[a2][b2][c2] = b3;
		//	fromh[a2][b2][c2] = c3;
		a3 = a2;
		b3 = b2;
		c3 = c2;

		//SCI_printf("%d,%d,%d,%d,%d	",x,y,head,nodeinfo[x][y][head].checked,nodeinfo[x][y][head].node1);

		if ((GOALSIZE == 9 && (((x == tx || x == tx + 1 || x == tx + 2) && (y == ty || y == ty + 1 || y == ty + 2)) || ((x == tx-1) && (y == ty || y == ty+1 || y == ty + 2) && (head == 9 || head == 10)) || ((x == tx || x == tx+1 || x == tx+2) && (y == ty-1) && (head == 4 || head == 5))))
			|| (GOALSIZE == 4 && (((x == tx || x == tx + 1) && (y == ty || y == ty + 1)) || ((x == tx-1) && (y == ty || y == ty+1) && (head == 9 || head == 10)) || ((x == tx || x == tx+1) && (y == ty-1) && (head == 4 || head == 5))))
			|| (GOALSIZE == 1 && (x == tx && y == ty && (head == 0 || head == 1 || head == 2 || head == 3)))) {
			dummycnt = 0;
			endx = x;
			endy = y;
			endh = head;
			while (1) {
				if((nodeinfo[a2][b2][c2].xy & 0x1) == 0x1){
					a3 = nodeinfo[a2][b2][c2].from_x + 16;//fromx[a2][b2][c2];
				}
				else{
					a3 = nodeinfo[a2][b2][c2].from_x;//fromx[a2][b2][c2];
				}
				if((nodeinfo[a2][b2][c2].xy & 0x2) == 0x2){
					b3 = nodeinfo[a2][b2][c2].from_y + 16;//fromy[a2][b2][c2];
				}
				else{
					b3 = nodeinfo[a2][b2][c2].from_y;//fromy[a2][b2][c2];
				}
				c3 = nodeinfo[a2][b2][c2].from_h;//fromh[a2][b2][c2];
				if (c2 == c3) {
					if (c2 == 0) {
						plus = 0;
						if (dummycnt == 0) {
							while (y + plus <= MA && (maze[y + plus][x] & 0x01) == 0x00) {
								plus++;
							}
						}
						dummy[dummycnt] = abs(b2 - b3) + plus;
						endy += plus;
					}
					else if (c2 == 1) {
						plus = 0;
						if (dummycnt == 0) {
							while (x + plus <= MA && (maze[y][x + plus] & 0x02) == 0x00) {
								plus++;
							}
						}
						dummy[dummycnt] = abs(a2 - a3) + plus;
						endx += plus;
					}
					else if (c2 == 2) {
						plus = 0;
						if (dummycnt == 0) {
							while (y - plus >= MI && (maze[y - plus][x] & 0x04) == 0x00) {
								plus++;
							}
						}
						dummy[dummycnt] = abs(b3 - b2) + plus;
						endy -= plus;
					}
					else if (c2 == 3) {
						plus = 0;
						if (dummycnt == 0) {
							while (x - plus >= MI && (maze[y][x - plus] & 0x08) == 0x00) {
								plus++;
							}
						}
						dummy[dummycnt] = abs(a3 - a2) + plus;
						endx -= plus;
					}
					else if (c2 == 4) {
						plus = 0;
						if (dummycnt == 0) {
							while (1) {
								x--;
								y++;
								if (x >= MI && y <= MA && (maze[y][x] & 0x02) == 0x00) {
									plus++;
								}
								else {
									break;
								}
								if (x >= MI && y <= MA && (maze[y][x] & 0x01) == 0x00) {
									plus++;
								}
								else {
									break;
								}
							}
						}
						if (plus > 0) {
						//	plus--;
						}
						dummy[dummycnt] = abs(b2 - b3) * 2 + 39 + plus;
						if((plus%2) == 0){
							endx -= plus/2;
							endy += plus/2;
							endh = endh;
						}
						else if((plus%2) == 1){
							endx -= (plus+1)/2;
							endy += (plus+1)/2;
							endh = 8;
						}
					}
					else if (c2 == 5) {
						plus = 0;
						if (dummycnt == 0) {
							while (1) {
								x++;
								y++;
								if (x <= MA && y <= MA && (maze[y][x] & 0x08) == 0x00) {
									plus++;
								}
								else {
									break;
								}
								if (x <= MA && y <= MA && (maze[y][x] & 0x01) == 0x00) {
									plus++;
								}
								else {
									break;
								}
							}
						}
						if (plus > 0) {
						//	plus--;
						}
						dummy[dummycnt] = abs(b2 - b3) * 2 + 39 + plus;
						if((plus%2) == 0){
							endx += plus/2;
							endy += plus/2;
							endh = endh;
						}
						else if((plus%2) == 1){
							endx += (plus-1)/2;
							endy += (plus+1)/2;
							endh = 9;
						}
					}
					else if (c2 == 6) {
						plus = 0;
						if (dummycnt == 0) {
							y++;
							while (1) {
								x++;
								y--;
								if (x <= MA && y >= MI && (maze[y][x] & 0x08) == 0x00) {
									plus++;
								}
								else {
									break;
								}
								if (x <= MA && y >= MI && (maze[y][x] & 0x04) == 0x00) {
									plus++;
								}
								else {
									break;
								}
							}
						}
						if (plus > 0) {
						//	plus--;
						}
						dummy[dummycnt] = abs(b3 - b2) * 2 + 39 + plus;
						if((plus%2) == 0){
							endx += plus/2;
							endy -= plus/2;
							endh = endh;
						}
						else if((plus%2) == 1){
							endx += (plus-1)/2;
							endy -= (plus-1)/2;
							endh = 10;
						}
					}
					else if (c2 == 7) {
						plus = 0;
						if (dummycnt == 0) {
							y++;
							while (1) {
								x--;
								y--;
								if (x >= MI && y >= MI && (maze[y][x] & 0x02) == 0x00) {
									plus++;
								}
								else {
									break;
								}
								if (x >= MI && y >= MI && (maze[y][x] & 0x04) == 0x00) {
									plus++;
								}
								else {
									break;
								}
							}
						}
						if (plus > 0) {
						//	plus--;
						}
						dummy[dummycnt] = abs(b3 - b2) * 2 + 39 + plus;
						if((plus%2) == 0){
							endx -= plus/2;
							endy -= plus/2;
							endh = endh;
						}
						else if((plus%2) == 1){
							endx -= (plus+1)/2;
							endy -= (plus-1)/2;
							endh = 11;
						}
					}
					else if (c2 == 8) {
						plus = 0;
						if (dummycnt == 0) {
							x++;
							while (1) {
								x--;
								y++;
								if (x >= MI && y <= MA && (maze[y][x] & 0x04) == 0x00) {
									plus++;
								}
								else {
									break;
								}
								if (x >= MI && y <= MA && (maze[y][x] & 0x08) == 0x00) {
									plus++;
								}
								else {
									break;
								}
							}
						}
						if (plus > 0) {
						//	plus--;
						}
						dummy[dummycnt] = abs(b2 - b3) * 2 + 39 + plus;
						if((plus%2) == 0){
							endx -= plus/2;
							endy += plus/2;
							endh = endh;
						}
						else if((plus%2) == 1){
							endx -= (plus-1)/2;
							endy += (plus-1)/2;
							endh = 4;
						}
					}
					else if (c2 == 9) {
						plus = 0;
						if (dummycnt == 0) {
							while (1) {
								x++;
								y++;
								if (x <= MA && y <= MA && (maze[y][x] & 0x04) == 0x00) {
									plus++;
								}
								else {
									break;
								}
								if (x <= MA && y <= MA && (maze[y][x] & 0x02) == 0x00) {
									plus++;
								}
								else {
									break;
								}
							}
						}
						if (plus > 0) {
						//	plus--;
						}
						dummy[dummycnt] = abs(b2 - b3) * 2 + 39 + plus;
						if((plus%2) == 0){
							endx += plus/2;
							endy += plus/2;
							endh = endh;
						}
						else if((plus%2) == 1){
							endx += (plus+1)/2;
							endy += (plus-1)/2;
							endh = 5;
						}
					}
					else if (c2 == 10) {
						plus = 0;
						if (dummycnt == 0) {
							while (1) {
								x++;
								y--;
								if (x <= MA && y >= MI && (maze[y][x] & 0x01) == 0x00) {
									plus++;
								}
								else {
									break;
								}
								if (x <= MA && y >= MI && (maze[y][x] & 0x02) == 0x00) {
									plus++;
								}
								else {
									break;
								}
							}
						}
						if (plus > 0) {
						//	plus--;
						}
						dummy[dummycnt] = abs(b3 - b2) * 2 + 39 + plus;
						if((plus%2) == 0){
							endx += plus/2;
							endy -= plus/2;
							endh = endh;
						}
						else if((plus%2) == 1){
							endx += (plus+1)/2;
							endy -= (plus+1)/2;
							endh = 6;
						}
					}
					else if (c2 == 11) {
						plus = 0;
						if (dummycnt == 0) {
							x++;
							while (1) {
								x--;
								y--;
								if (x >= MI && y >= MI && (maze[y][x] & 0x01) == 0x00) {
									plus++;
								}
								else {
									break;
								}
								if (x >= MI && y >= MI && (maze[y][x] & 0x08) == 0x00) {
									plus++;
								}
								else {
									break;
								}
							}
						}
						if (plus > 0) {
						//	plus--;
						}
						dummy[dummycnt] = abs(b3 - b2) * 2 + 39 + plus;
						if((plus%2) == 0){
							endx -= plus/2;
							endy -= plus/2;
							endh = endh;
						}
						else if((plus%2) == 1){
							endx -= (plus-1)/2;
							endy -= (plus+1)/2;
							endh = 7;
						}
					}
					else {
						dummy[dummycnt] = 0;
					}
				}
				else {
					if (c3 == 0) {
						if (c2 == 1) {
							dummy[dummycnt] = BIG90R;
							if(dummycnt == 0){
							endx++;
							}
						}
						else if (c2 == 2) {
							if (a2 > a3) {
								dummy[dummycnt] = BIG180R;
								if(dummycnt == 0){
								endy--;
								}
							}
							else {
								dummy[dummycnt] = BIG180L;
								if(dummycnt == 0){
								endy--;
								}
							}
						}
						else if (c2 == 3) {
							dummy[dummycnt] = BIG90L;
							if(dummycnt == 0){
							endx--;
							}
						}
						else if (c2 == 6) {
							if(dummycnt == 0){
								if(y-1 >= MI && x+1 <= MA && (maze[y-1][x+1]&0x01) == 0x00){
									goalslantnum = 2;
								}
								else{
									goalslantnum = 1;
								}
							}
							dummy[dummycnt] = IN135R;
						}
						else if (c2 == 7) {
							if(dummycnt == 0){
								if(y-1 >= MI && x-1 >= MI && (maze[y-1][x-1]&0x01) == 0x00){
									goalslantnum = 2;
								}
								else{
									goalslantnum = 1;
								}
							}
							dummy[dummycnt] = IN135L;
						}
						else if (c2 == 8) {
							if(dummycnt == 0){
								if(y+1 <= MA && x-1 >= MI && (maze[y+1][x-1]&0x02) == 0x00){
									goalslantnum = 2;
								}
								else{
									goalslantnum = 1;
								}
							}
							dummy[dummycnt] = IN45L;
						}
						else if (c2 == 9) {
							if(dummycnt == 0){
								if(y+1 <= MA && x+1 <= MA && (maze[y+1][x+1]&0x02) == 0x00){
									goalslantnum = 2;
								}
								else{
									goalslantnum = 1;
								}
							}
							dummy[dummycnt] = IN45R;
						}
						else {
							dummy[dummycnt] = 0;
						}
					}
					else if (c3 == 1) {
						if (c2 == 2) {
							dummy[dummycnt] = BIG90R;
							if(dummycnt == 0){
							endy--;
							}
						}
						else if (c2 == 3) {
							if (b2 < b3) {
								dummy[dummycnt] = BIG180R;
								if(dummycnt == 0){
								endx--;
								}
							}
							else {
								dummy[dummycnt] = BIG180L;
								if(dummycnt == 0){
								endx--;
								}
							}
						}
						else if (c2 == 0) {
							dummy[dummycnt] = BIG90L;
							if(dummycnt == 0){
							endy++;
							}
						}
						else if (c2 == 11) {
							if(dummycnt == 0){
								if(y-1 >= MI && x-1 >= MI && (maze[y-1][x-1]&0x02) == 0x00){
									goalslantnum = 2;
								}
								else{
									goalslantnum = 1;
								}
							}
							dummy[dummycnt] = IN135R;
						}
						else if (c2 == 8) {
							if(dummycnt == 0){
								if(y+1 <= MA && x-1 >= MI && (maze[y+1][x-1]&0x02) == 0x00){
									goalslantnum = 2;
								}
								else{
									goalslantnum = 1;
								}
							}
							dummy[dummycnt] = IN135L;
						}
						else if (c2 == 5) {
							if(dummycnt == 0){
								if(y+1 <= MA && x+1 <= MA && (maze[y+1][x+1]&0x01) == 0x00){
									goalslantnum = 2;
								}
								else{
									goalslantnum = 1;
								}
							}
							dummy[dummycnt] = IN45L;
						}
						else if (c2 == 6) {
							if(dummycnt == 0){
								if(y-1 >= MI && x+1 <= MA && (maze[y-1][x+1]&0x01) == 0x00){
									goalslantnum = 2;
								}
								else{
									goalslantnum = 1;
								}
							}
							dummy[dummycnt] = IN45R;
						}
						else {
							dummy[dummycnt] = 0;
						}
					}
					else if (c3 == 2) {
						if (c2 == 3) {
							dummy[dummycnt] = BIG90R;
							if(dummycnt == 0){
							endx--;
							}
						}
						else if (c2 == 0) {
							if (a2 < a3) {
								dummy[dummycnt] = BIG180R;
								if(dummycnt == 0){
								endy++;
								}
							}
							else {
								dummy[dummycnt] = BIG180L;
								if(dummycnt == 0){
								endy++;
								}
							}
						}
						else if (c2 == 1) {
							dummy[dummycnt] = BIG90L;
							if(dummycnt == 0){
							endx++;
							}
						}
						else if (c2 == 4) {
							if(dummycnt == 0){
								if(y+1 <= MA && x-1 >= MI && (maze[y+1][x-1]&0x01) == 0x00){
									goalslantnum = 2;
								}
								else{
									goalslantnum = 1;
								}
							}
							dummy[dummycnt] = IN135R;
						}
						else if (c2 == 5) {
							if(dummycnt == 0){
								if(y+1 <= MA && x+1 <= MA && (maze[y+1][x+1]&0x01) == 0x00){
									goalslantnum = 2;
								}
								else{
									goalslantnum = 1;
								}
							}
							dummy[dummycnt] = IN135L;
						}
						else if (c2 == 10) {
							if(dummycnt == 0){
								if(y-1 >= MI && x+1 <= MA && (maze[y-1][x+1]&0x02) == 0x00){
									goalslantnum = 2;
								}
								else{
									goalslantnum = 1;
								}
							}
							dummy[dummycnt] = IN45L;
						}
						else if (c2 == 11) {
							if(dummycnt == 0){
								if(y-1 >= MI && x-1 >= MI && (maze[y-1][x-1]&0x02) == 0x00){
									goalslantnum = 2;
								}
								else{
									goalslantnum = 1;
								}
							}
							dummy[dummycnt] = IN45R;
						}
						else {
							dummy[dummycnt] = 0;
						}
					}
					else if (c3 == 3) {
						if (c2 == 0) {
							dummy[dummycnt] = BIG90R;
							if(dummycnt == 0){
							endy++;
							}
						}
						else if (c2 == 1) {
							if (b2 > b3) {
								dummy[dummycnt] = BIG180R;
								if(dummycnt == 0){
								endx++;
								}
							}
							else {
								dummy[dummycnt] = BIG180L;
								if(dummycnt == 0){
								endx++;
								}
							}
						}
						else if (c2 == 2) {
							dummy[dummycnt] = BIG90L;
							if(dummycnt == 0){
							endy--;
							}
						}
						else if (c2 == 9) {
							if(dummycnt == 0){
								if(y+1 <= MA && x+1 <= MA && (maze[y+1][x+1]&0x02) == 0x00){
									goalslantnum = 2;
								}
								else{
									goalslantnum = 1;
								}
							}
							dummy[dummycnt] = IN135R;
						}
						else if (c2 == 10) {
							if(dummycnt == 0){
								if(y-1 >= MI && x+1 <= MA && (maze[y-1][x+1]&0x02) == 0x00){
									goalslantnum = 2;
								}
								else{
									goalslantnum = 1;
								}
							}
							dummy[dummycnt] = IN135L;
						}
						else if (c2 == 7) {
							if(dummycnt == 0){
								if(y-1 >= MI && x-1 >= MI && (maze[y-1][x-1]&0x01) == 0x00){
									goalslantnum = 2;
								}
								else{
									goalslantnum = 1;
								}
							}
							dummy[dummycnt] = IN45L;
						}
						else if (c2 == 4) {
							if(dummycnt == 0){
								if(y+1 <= MA && x-1 >= MI && (maze[y+1][x-1]&0x01) == 0x00){
									goalslantnum = 2;
								}
								else{
									goalslantnum = 1;
								}
							}
							dummy[dummycnt] = IN45R;
						}
						else {
							dummy[dummycnt] = 0;
						}
					}
					else if (c3 == 4) {
						if (c2 == 7) {
							if(dummycnt == 0){
								if(y-1 >= MI && x-1 >= MI && (maze[y-1][x-1]&0x01) == 0x00){
									goalslantnum = 2;
								}
								else{
									goalslantnum = 1;
								}
							}
							if ((a3 - a2) == 2) {
								dummy[dummycnt] = KOJIMAL;
							}
							else {
								dummy[dummycnt] = V90L;
							}
							//dummy[dummycnt] = V90L;
						}
						else if (c2 == 3) {
							dummy[dummycnt] = OUT45L;
							if(dummycnt == 0){
							endx--;
							}
						}
						else if (c2 == 2) {
							dummy[dummycnt] = OUT135L;
							if(dummycnt == 0){
							endy--;
							}
						}
						else if (c2 == 8) {
							plus = 0;
							if (dummycnt == 0) {
								x++;
								while (1) {
									x--;
									y++;
									if (x >= MI && y <= MA && (maze[y][x] & 0x04) == 0x00) {
										plus++;
									}
									else {
										break;
									}
									if (x >= MI && y <= MA && (maze[y][x] & 0x08) == 0x00) {
										plus++;
									}
									else {
										break;
									}
								}
							}
							if (plus > 0) {
							//	plus--;
							}
							dummy[dummycnt] = abs(b2 - b3) * 2 + 38 + plus;
							if((plus%2) == 0){
								endx -= plus/2;
								endy += plus/2;
								endh = endh;
							}
							else if((plus%2) == 1){
								endx -= (plus-1)/2;
								endy += (plus-1)/2;
								endh = 4;
							}
						}
						else {
							dummy[dummycnt] = 0;
						}
					}
					else if (c3 == 5) {
						if (c2 == 6) {
							if(dummycnt == 0){
								if(y-1 >= MI && x+1 <= MA && (maze[y-1][x+1]&0x01) == 0x00){
									goalslantnum = 2;
								}
								else{
									goalslantnum = 1;
								}
							}
							if ((a2 - a3) == 2) {
								dummy[dummycnt] = KOJIMAR;
							}
							else {
								dummy[dummycnt] = V90R;
							}
							//dummy[dummycnt] = V90R;
						}
						else if (c2 == 1) {
							dummy[dummycnt] = OUT45R;
							if(dummycnt == 0){
							endx++;
							}
						}
						else if (c2 == 2) {
							dummy[dummycnt] = OUT135R;
							if(dummycnt == 0){
							endy--;
							}
						}
						else if (c2 == 9) {
							plus = 0;
							if (dummycnt == 0) {
								while (1) {
									x++;
									y++;
									if (x <= MA && y <= MA && (maze[y][x] & 0x04) == 0x00) {
										plus++;
									}
									else {
										break;
									}
									if (x <= MA && y <= MA && (maze[y][x] & 0x02) == 0x00) {
										plus++;
									}
									else {
										break;
									}
								}
							}
							if (plus > 0) {
							//	plus--;
							}
							dummy[dummycnt] = abs(b2 - b3) * 2 + 38 + plus;
							if((plus%2) == 0){
								endx += plus/2;
								endy += plus/2;
								endh = endh;
							}
							else if((plus%2) == 1){
								endx += (plus+1)/2;
								endy += (plus-1)/2;
								endh = 5;
							}
						}
						else {
							dummy[dummycnt] = 0;
						}
					}
					else if (c3 == 6) {
						if (c2 == 5) {
							if(dummycnt == 0){
								if(y+1 <= MA && x+1 <= MA && (maze[y+1][x+1]&0x01) == 0x00){
									goalslantnum = 2;
								}
								else{
									goalslantnum = 1;
								}
							}
							if ((a2 - a3) == 2) {
								dummy[dummycnt] = KOJIMAL;
							}
							else {
								dummy[dummycnt] = V90L;
							}
							//dummy[dummycnt] = V90L;
						}
						else if (c2 == 1) {
							dummy[dummycnt] = OUT45L;
							if(dummycnt == 0){
							endx++;
							}
						}
						else if (c2 == 0) {
							dummy[dummycnt] = OUT135L;
							if(dummycnt == 0){
							endy++;
							}
						}
						else if (c2 == 10) {
							plus = 0;
							if (dummycnt == 0) {
								//y++;
								while (1) {
									x++;
									y--;
									if (x <= MA && y >= MI && (maze[y][x] & 0x01) == 0x00) {
										plus++;
									}
									else {
										break;
									}
									if (x <= MA && y >= MI && (maze[y][x] & 0x02) == 0x00) {
										plus++;
									}
									else {
										break;
									}
								}
							}
							if (plus > 0) {
							//	plus--;
							}
							dummy[dummycnt] = abs(b2 - b3) * 2 + 40 + plus;
							if((plus%2) == 0){
								endx += plus/2;
								endy -= plus/2;
								endh = endh;
							}
							else if((plus%2) == 1){
								endx += (plus+1)/2;
								endy -= (plus+1)/2;
								endh = 6;
							}
						}
						else {
							dummy[dummycnt] = 0;
						}
					}
					else if (c3 == 7) {
						if (c2 == 4) {
							if(dummycnt == 0){
								if(y+1 <= MA && x-1 >= MI && (maze[y+1][x-1]&0x01) == 0x00){
									goalslantnum = 2;
								}
								else{
									goalslantnum = 1;
								}
							}
							if ((a3 - a2) == 2) {
								dummy[dummycnt] = KOJIMAR;
							}
							else {
								dummy[dummycnt] = V90R;
							}
							//dummy[dummycnt] = V90R;
						}
						else if (c2 == 3) {
							dummy[dummycnt] = OUT45R;
							if(dummycnt == 0){
							endx--;
							}
						}
						else if (c2 == 0) {
							dummy[dummycnt] = OUT135R;
							if(dummycnt == 0){
							endy++;
							}
						}
						else if (c2 == 11) {
							plus = 0;
							if (dummycnt == 0) {
								x++;
								while (1) {
									x--;
									y--;
									if (x >= MI && y >= MI && (maze[y][x] & 0x01) == 0x00) {
										plus++;
									}
									else {
										break;
									}
									if (x >= MI && y >= MI && (maze[y][x] & 0x08) == 0x00) {
										plus++;
									}
									else {
										break;
									}
								}
							}
							if (plus > 0) {
							//	plus--;
							}
							dummy[dummycnt] = abs(b3 - b2) * 2 + 40 + plus;
							if((plus%2) == 0){
								endx -= plus/2;
								endy -= plus/2;
								endh = endh;
							}
							else if((plus%2) == 1){
								endx -= (plus-1)/2;
								endy -= (plus+1)/2;
								endh = 7;
							}
						}
						else {
							dummy[dummycnt] = 0;
						}
					}
					else if (c3 == 8) {
						if (c2 == 9) {
							if(dummycnt == 0){
								if(y+1 <= MA && x+1 <= MA && (maze[y+1][x+1]&0x02) == 0x00){
									goalslantnum = 2;
								}
								else{
									goalslantnum = 1;
								}
							}
							if ((b2 - b3) == 2) {
								dummy[dummycnt] = KOJIMAR;
							}
							else {
								dummy[dummycnt] = V90R;
							}
							//dummy[dummycnt] = V90R;
						}
						else if (c2 == 0) {
							dummy[dummycnt] = OUT45R;
							if(dummycnt == 0){
							endy++;
							}
						}
						else if (c2 == 1) {
							dummy[dummycnt] = OUT135R;
							if(dummycnt == 0){
							endx++;
							}
						}
						else if (c2 == 4) {
							plus = 0;
							if (dummycnt == 0) {
								//	x++;
								while (1) {
									x--;
									y++;
									if (x >= MI && y <= MA && (maze[y][x] & 0x02) == 0x00) {
										plus++;
									}
									else {
										break;
									}
									if (x >= MI && y <= MA && (maze[y][x] & 0x01) == 0x00) {
										plus++;
									}
									else {
										break;
									}
								}
							}
							if (plus > 0) {
							//	plus--;
							}
							dummy[dummycnt] = abs(b2 - b3) * 2 + 40 + plus;
							if((plus%2) == 0){
								endx -= plus/2;
								endy += plus/2;
								endh = endh;
							}
							else if((plus%2) == 1){
								endx -= (plus+1)/2;
								endy += (plus+1)/2;
								endh = 8;
							}
						}
						else {
							dummy[dummycnt] = 0;
						}
					}
					else if (c3 == 9) {
						if (c2 == 8) {
							if(dummycnt == 0){
								if(y+1 <= MA && x-1 >= MI && (maze[y+1][x-1]&0x02) == 0x00){
									goalslantnum = 2;
								}
								else{
									goalslantnum = 1;
								}
							}
							if ((b2 - b3) == 2) {
								dummy[dummycnt] = KOJIMAL;
							}
							else {
								dummy[dummycnt] = V90L;
							}
							//dummy[dummycnt] = V90L;
						}
						else if (c2 == 0) {
							dummy[dummycnt] = OUT45L;
							if(dummycnt == 0){
							endy++;
							}
						}
						else if (c2 == 3) {
							dummy[dummycnt] = OUT135L;
							if(dummycnt == 0){
							endx--;
							}
						}
						else if (c2 == 5) {
							plus = 0;
							if (dummycnt == 0) {
								while (1) {
									x++;
									y++;
									if (x <= MA && y <= MA && (maze[y][x] & 0x08) == 0x00) {
										plus++;
									}
									else {
										break;
									}
									if (x <= MA && y <= MA && (maze[y][x] & 0x01) == 0x00) {
										plus++;
									}
									else {
										break;
									}
								}
							}
							if (plus > 0) {
							//	plus--;
							}
							dummy[dummycnt] = abs(b2 - b3) * 2 + 40 + plus;
							if((plus%2) == 0){
								endx += plus/2;
								endy += plus/2;
								endh = endh;
							}
							else if((plus%2) == 1){
								endx += (plus-1)/2;
								endy += (plus+1)/2;
								endh = 9;
							}
						}
						else {
							dummy[dummycnt] = 0;
						}
					}
					else if (c3 == 10) {
						if (c2 == 11) {
							if(dummycnt == 0){
								if(y-1 >= MI && x-1 >= MI && (maze[y-1][x-1]&0x02) == 0x00){
									goalslantnum = 2;
								}
								else{
									goalslantnum = 1;
								}
							}
							if ((b3 - b2) == 2) {
								dummy[dummycnt] = KOJIMAR;
							}
							else {
								dummy[dummycnt] = V90R;
							}
							//dummy[dummycnt] = V90R;
						}
						else if (c2 == 2) {
							dummy[dummycnt] = OUT45R;
							if(dummycnt == 0){
							endy--;
							}
						}
						else if (c2 == 3) {
							dummy[dummycnt] = OUT135R;
							if(dummycnt == 0){
							endx--;
							}
						}
						else if (c2 == 6) {
							plus = 0;
							if (dummycnt == 0) {
								y++;
								while (1) {
									x++;
									y--;
									if (x <= MA && y >= MI && (maze[y][x] & 0x08) == 0x00) {
										plus++;
									}
									else {
										break;
									}
									if (x <= MA && y >= MI && (maze[y][x] & 0x04) == 0x00) {
										plus++;
									}
									else {
										break;
									}
								}
							}
							if (plus > 0) {
							//	plus--;
							}
							dummy[dummycnt] = abs(b3 - b2) * 2 + 38 + plus;
							if((plus%2) == 0){
								endx += plus/2;
								endy -= plus/2;
								endh = endh;
							}
							else if((plus%2) == 1){
								endx += (plus-1)/2;
								endy -= (plus-1)/2;
								endh = 10;
							}
						}
						else {
							dummy[dummycnt] = 0;
						}
					}
					else if (c3 == 11) {
						if (c2 == 10) {
							if(dummycnt == 0){
								if(y-1 >= MI && x+1 <= MA && (maze[y-1][x+1]&0x02) == 0x00){
									goalslantnum = 2;
								}
								else{
									goalslantnum = 1;
								}
							}
							if ((b3 - b2) == 2) {
								dummy[dummycnt] = KOJIMAL;
							}
							else {
								dummy[dummycnt] = V90L;
							}
							//dummy[dummycnt] = V90L;
						}
						else if (c2 == 2) {
							dummy[dummycnt] = OUT45L;
							if(dummycnt == 0){
							endy--;
							}
						}
						else if (c2 == 1) {
							dummy[dummycnt] = OUT135L;
							if(dummycnt == 0){
							endx++;
							}
						}
						else if (c2 == 7) {
							plus = 0;
							if (dummycnt == 0) {
								y++;
								while (1) {
									x--;
									y--;
									if (x >= MI && y >= MI && (maze[y][x] & 0x02) == 0x00) {
										plus++;
									}
									else {
										break;
									}
									if (x >= MI && y >= MI && (maze[y][x] & 0x04) == 0x00) {
										plus++;
									}
									else {
										break;
									}
								}
							}
							if (plus > 0) {
							//	plus--;
							}
							dummy[dummycnt] = abs(b3 - b2) * 2 + 38 + plus;
							if((plus%2) == 0){
								endx -= plus/2;
								endy -= plus/2;
								endh = endh;
							}
							else if((plus%2) == 1){
								endx -= (plus+1)/2;
								endy -= (plus-1)/2;
								endh = 11;
							}
						}
						else {
							dummy[dummycnt] = 0;
						}
					}
					else {
						dummy[dummycnt] = 0;
					}
				}
				a2 = a3;
				b2 = b3;
				c2 = c3;
				dummycnt++;
				if (a3 == 0 && b3 == 0) {
					saitantime = 0;
					//dummy[dummycnt] = 1234;
					for (a2 = 0; a2 < dummycnt; a2++) {
						path[a2] = dummy[dummycnt - a2 - 1];
						saitantime += runtime[dummy[dummycnt - a2 - 1]];
					}
					path[dummycnt] = SNODE;
					//
					if(endh == 4){
						if(goalslantnum == 1){
							endx--;
							endy++;
							endh = 8;
						}
						else if(goalslantnum == 2){
							endx--;
							endy++;
							endh = 4;
						}
					}
					else if(endh == 5){
						if(goalslantnum == 1){
							//endx--;
							endy++;
							endh = 9;
						}
						else if(goalslantnum == 2){
							endx++;
							endy++;
							endh = 5;
						}
					}
					else if(endh == 6){
						if(goalslantnum == 1){
							//endx--;
							//endy++;
							endh = 10;
						}
						else if(goalslantnum == 2){
							endx++;
							endy--;
							endh = 6;
						}
					}
					else if(endh == 7){
						if(goalslantnum == 1){
							endx--;
							//endy++;
							endh = 11;
						}
						else if(goalslantnum == 2){
							endx--;
							endy--;
							endh = 7;
						}
					}
					else if(endh == 8){
						if(goalslantnum == 1){
							//endx--;
							//endy++;
							endh = 4;
						}
						else if(goalslantnum == 2){
							endx--;
							endy++;
							endh = 8;
						}
					}
					else if(endh == 9){
						if(goalslantnum == 1){
							endx++;
							//endy++;
							endh = 5;
						}
						else if(goalslantnum == 2){
							endx++;
							endy++;
							endh = 9;
						}
					}
					else if(endh == 10){
						if(goalslantnum == 1){
							endx++;
							endy--;
							endh = 6;
						}
						else if(goalslantnum == 2){
							endx++;
							endy--;
							endh = 10;
						}
					}
					else if(endh == 11){
						if(goalslantnum == 1){
							//endx--;
							endy--;
							endh = 7;
						}
						else if(goalslantnum == 2){
							endx--;
							endy--;
							endh = 11;
						}
					}
					//
					break;
				}
			}
			break;
		}
	}
}

void infoup(unsigned char xx,unsigned char yy,unsigned char hh,signed char xxx,signed char yyy,unsigned char hhh){
	nodeinfo[xx+xxx][yy+yyy][hhh].from_x = (xx&0x0F);
	nodeinfo[xx+xxx][yy+yyy][hhh].from_y = (yy&0x0F);
	if(((xx&0x10) == 0x10) && ((yy&0x10) == 0x10)){
		nodeinfo[xx+xxx][yy+yyy][hhh].xy = 3;
	}
	else if(((xx&0x10) != 0x10) && ((yy&0x10) == 0x10)){
		nodeinfo[xx+xxx][yy+yyy][hhh].xy = 2;
	}
	else if(((xx&0x10) == 0x10) && ((yy&0x10) != 0x10)){
		nodeinfo[xx+xxx][yy+yyy][hhh].xy = 1;
	}
	else if(((xx&0x10) != 0x10) && ((yy&0x10) != 0x10)){
		nodeinfo[xx+xxx][yy+yyy][hhh].xy = 0;
	}
	nodeinfo[xx+xxx][yy+yyy][hhh].from_h = hh;
}

void path_reverse(char hs){
	volatile short num = 0,num2 = 0;
	volatile short i = 0;
	//char dum[1024];
	for (i = 0; i < 1024; i++) {
		dummy[i] = 0;
	}
	while(path[num] != SNODE){
		num++;
	}
	num--;
	while(1){
		if(num == -1){
			dummy[num2] = SNODE;
			break;
		}
		if(path[num] <= GO31/*GO15*/){
			dummy[num2] = path[num];
		}
		else if(path[num] <= DIA_GO63/*31*/){
			dummy[num2] = path[num];
		}
		else if(path[num] == BIG90R){
			if(num2 == 0 && hs == 0){
			dummy[num2] = GO1;
			num2++;
			}
			dummy[num2] = BIG90L;
		}
		else if(path[num] == BIG90L){
			if(num2 == 0 && hs == 0){
			dummy[num2] = GO1;
			num2++;
			}
			dummy[num2] = BIG90R;
		}
		else if(path[num] == BIG180R){
			if(num2 == 0 && hs == 0){
			dummy[num2] = GO1;
			num2++;
			}
			dummy[num2] = BIG180L;
		}
		else if(path[num] == BIG180L){
			if(num2 == 0 && hs == 0){
			dummy[num2] = GO1;
			num2++;
			}
			dummy[num2] = BIG180R;
		}
		else if(path[num] == IN45R){
			if(num2 == 0){
			dummy[num2] = 39+goalslantnum;
			num2++;
			}
			dummy[num2] = OUT45L;
		}
		else if(path[num] == IN45L){
			if(num2 == 0){
			dummy[num2] = 39+goalslantnum;
			num2++;
			}
			dummy[num2] = OUT45R;
		}
		else if(path[num] == IN135R){
			if(num2 == 0){
			dummy[num2] = 39+goalslantnum;
			num2++;
			}
			dummy[num2] = OUT135L;
		}
		else if(path[num] == IN135L){
			if(num2 == 0){
			dummy[num2] = 39+goalslantnum;
			num2++;
			}
			dummy[num2] = OUT135R;
		}
		else if(path[num] == OUT45R){
			if(num2 == 0 && hs == 0){
			dummy[num2] = GO1;
			num2++;
			}
			dummy[num2] = IN45L;
		}
		else if(path[num] == OUT45L){
			if(num2 == 0 && hs == 0){
			dummy[num2] = GO1;
			num2++;
			}
			dummy[num2] = IN45R;
		}
		else if(path[num] == OUT135R){
			if(num2 == 0 && hs == 0){
			dummy[num2] = GO1;
			num2++;
			}
			dummy[num2] = IN135L;
		}
		else if(path[num] == OUT135L){
			if(num2 == 0 && hs == 0){
			dummy[num2] = GO1;
			num2++;
			}
			dummy[num2] = IN135R;
		}
		else if(path[num] == V90R){
			if(num2 == 0){
			dummy[num2] = 39+goalslantnum;
			num2++;
			}
			dummy[num2] = V90L;
		}
		else if(path[num] == V90L){
			if(num2 == 0){
			dummy[num2] = 39+goalslantnum;
			num2++;
			}
			dummy[num2] = V90R;
		}
		else if(path[num] == KOJIMAR){
			if(num2 == 0){
			dummy[num2] = 39+goalslantnum;
			num2++;
			}
			dummy[num2] = KOJIMAL;
		}
		else if(path[num] == KOJIMAL){
			if(num2 == 0){
			dummy[num2] = 39+goalslantnum;
			num2++;
			}
			dummy[num2] = KOJIMAR;
		}
		num--;
		num2++;
	}
	for(num = 0; num <= num2; num++){
		path[num] = dummy[num];
	}
}
