
#ifndef _BEELINE_
#define _BEELINE_

#include"Dataflash.h"

#define ONESLANTING	63.64//127.27//127.2792204
#define SAITANWALLCUT_R flash6[0]//3.0//3.0			//ç≈íZï«êÿÇÍâE
#define SAITANWALLCUT_L flash6[1]//3.0//0.5			//ç≈íZï«êÿÇÍç∂

#define TURN_SHORT flash6[2]//3.0//3.0

#define FWALL_CLOSE_B flash6[3]//150

#define D_PLUS flash6[4]//0.0


void saitan_shortest(int,int,char,char);
void makesaitan(int,int);
void saitan_to_path(void);
void waitForEdge(char);
void turnWallEdge(void);
void setpara(int);
void edge_reverse(void);
//void check_lipo(void);

typedef struct
{
	unsigned char edge:2;
	unsigned char combsw:1;
}t_edge;

#endif
