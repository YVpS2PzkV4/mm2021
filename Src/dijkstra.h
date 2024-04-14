#ifndef __DIJKSTRA_H
#define __DIJKSTRA_H

#include <stdint.h>

//コマンドリスト（GO1は１区画前進を意味する）
#define GO1    1//必ず１である必要がある
#define GO31  31

#define DIA_GO1 40
#define DIA_GO63 102
#define SNODE 119//ストップを意味する

#define BIG90R 103
#define BIG90L 104
#define BIG180R 105
#define BIG180L 106
#define IN45R 107
#define IN45L 108
#define IN135R 109
#define IN135L 110
#define OUT45R 111
#define OUT45L 112
#define OUT135R 113
#define OUT135L 114
#define V90R 115
#define V90L 116
#define KOJIMAR 117
#define KOJIMAL 118

#define DIAGONAL 39

#define MA 31
#define MI 0

//void setRunTimeCost(void);
void runTimeCalc(void);

void map_to_maze(void);
void nodereset(void);
void shortest(int,int);
void infoup(unsigned char,unsigned char,unsigned char,signed char,signed char,unsigned char);
void path_reverse(char);

typedef struct
{
	unsigned char from_x:4;	//北の壁情報
	unsigned char from_y:4;	//東の壁情報
	unsigned char xy:2;
	unsigned char from_h:4;	//南の壁情報
	unsigned char checked:1;	//西の壁情報
	unsigned short node1:16;
}t_dijkstra;

//void inputmaze(void);

#endif /* __DIJKSTRA_H */
