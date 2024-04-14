#ifndef _MAPSET_
#define _MAPSET_

#include"Dataflash.h"

void queue_push(char,char);
unsigned short queue_pop(void);
void queue_push2(unsigned char,unsigned char,unsigned char);
void mapCtrlZ(unsigned char);
short numOfWalls(short,short);
short closeDeadEnd(short,short);
void clearmap(void);
void clearmap2(void);
void mapsave(void);
void recoverymap(void);
void makemap(void);
int getwalldata(void);
void makesmap(int,int,int);
void goalwallset(int,int,int);
void recovery_from_crash(short,short);
void make_map_known(void);

#define STEPMAX 65535

#define FWALL_CLOSE2 flash6[5]//70.0
#define FWALL_DIFF_BIG flash6[6]//7.5

#define COMP_SIZE_X flash3.compSizeX//32
#define COMP_SIZE_Y flash3.compSizeY//32


#define SEARCH_DATA_NUM 20

typedef struct
{
	volatile unsigned char qX[SEARCH_DATA_NUM];		//x
	volatile unsigned char qY[SEARCH_DATA_NUM];		//y
	volatile unsigned char qMap[SEARCH_DATA_NUM];	//壁データ
	//volatile unsigned char qHead;
	volatile unsigned char qTail;
}t_save;

t_save Saved;	//探索中保存データ

#endif
