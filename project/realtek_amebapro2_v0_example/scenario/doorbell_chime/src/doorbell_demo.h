#ifndef BDDORBELL_DEMO_H
#define BDDORBELL_DEMO_H

#define ISP_FLIP_ENABLE	0
#define FILP_NUM        0x03

#define SERVER_PORT 5000

#define FIREBASE_FLASH_ADDR  0x240000 - 0x1000

#define SUSPEND_TIME 180000//600000//120000 //ms
#define WIFI_CONNECT_TIMEOUT  120000
void doorbell_init_function(void *parm);

#define MAX_SPEAK_TIMES   3    //total time = SUSPEND_TIME * MAX_SPEAK_TIMES
#define MAX_STREAM_TIMES  3    //total time = SUSPEND_TIME * MAX_STREAM_TIMES

#endif