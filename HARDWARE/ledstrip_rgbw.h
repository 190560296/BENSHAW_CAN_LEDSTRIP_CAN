#ifndef __ledstrip_rgbw_h
#define __ledstrip_rgbw_h

#include "main.h"
#include "stm32f1xx_hal.h"
//#include "cmsis_os.h"
#include "color_rgbw.h"

#define BIT_0 28
#define BIT_1 67

#define lednumber1 110
#define ledcnt1 (lednumber1*32 + 43)
static uint16_t LED1_BYTE_BUFFER[ledcnt1] = {0x00};

#define lednumber2	110
#define ledcnt2 (lednumber2*32 + 43)
static uint16_t LED2_BYTE_BUFFER[ledcnt2] = {0x00};

//#define lednumber3 10
//#define ledcnt3 (lednumber3*32 + 43)
//static uint16_t LED3_BYTE_BUFFER[ledcnt3] = {0x00};

void ledstrip1_rgbw_perframe(int number,int len,uint8_t (*color_led)[4],uint8_t (*color_bg)[4]);
void ledstrip1_rgbw_alloff(void);
void ledstrip1_rgbw_forward(int len,int delayms,uint8_t (*color_led)[4],uint8_t (*color_bg)[4]);
void ledstrip1_rgbw_backward(int len,int delayms,uint8_t (*color_led)[4],uint8_t (*color_bg)[4]);

void ledstrip2_rgbw_perframe(int number,int len,uint8_t (*color_led)[4],uint8_t (*color_bg)[4]);
void ledstrip2_rgbw_alloff(void);
void ledstrip2_rgbw_brightset_WHITE(int len,int brightlevel);

//void ledstrip3_rgbw_perframe(int number,int len,uint8_t (*color_led)[4],uint8_t (*color_bg)[4]);
//void ledstrip3_rgbw_alloff(void);
//void ledstrip3_rgbw_brightset_WHITE(int len,int brightlevel);

//void ledgroup_stepon(int delayms);
//void ledgroup_stepdown(int delayms);

void ledgroup_rgbw_flow(int len,int delayms,uint8_t (*color_led)[4],uint8_t (*color_bg)[4]);

#endif
