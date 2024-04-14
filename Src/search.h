#ifndef _SEARCH_
#define _SEARCH_

#include"Dataflash.h"

void wallCtrlReset(void);
void slaadachi(int,int,int);
void setpara_search(int);

#define HALF_SECTION	(45.0f)			//”¼‹æ‰æ‚Ì‹——£
#define ONESECTION 		(90.0f)

#define STARTWAIT 1000

#define TURNWAIT 20//200
#define SEARCHBACK (15.0f)//15


#define WALLTOMIDDLE (8.0f)//48//45//43//53 //90 - 37 //40

#define SEARCHTIMEMAX flash3.searchTime//6.5 //60000*•ª
#define LEN_DOWN flash3.lenDown//25//1000*m

#define GOALSIZE flash3.goalSize

#endif
