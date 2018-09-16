#include "ledstrip_rgbw.h"

//extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

void ledstrip1_rgbw_perframe(int number,int len,uint8_t (*color_led)[4],uint8_t (*color_bg)[4])
{
  int cnt = 0;
  int lednum = 0;
  int i;

  for(; cnt < 3; cnt++)
  {
    LED1_BYTE_BUFFER[cnt] = 0;
  }

  while(lednum <= len)
  {
    if(lednum <= number)
    {
      for(i = 0; i < 8; i++)
      {
        LED1_BYTE_BUFFER[cnt] = ((*color_led)[0]<<i)&0x80?BIT_1:BIT_0;
        cnt++;
      }
      for(i = 0; i < 8; i++)
      {
        LED1_BYTE_BUFFER[cnt] = ((*color_led)[1]<<i)&0x80?BIT_1:BIT_0;
        cnt++;
      }
      for(i = 0; i < 8; i++)
      {
        LED1_BYTE_BUFFER[cnt] = ((*color_led)[2]<<i)&0x80?BIT_1:BIT_0;
        cnt++;
      }
      for(i = 0; i < 8; i++)
      {
        LED1_BYTE_BUFFER[cnt] = ((*color_led)[3]<<i)&0x80?BIT_1:BIT_0;
        cnt++;
      }
    }
    else
    {
      for(i = 0; i < 8; i++)
      {
        LED1_BYTE_BUFFER[cnt] = ((*color_bg)[0]<<i)&0x80?BIT_1:BIT_0;
        cnt++;
      }
      for(i = 0; i < 8; i++)
      {
        LED1_BYTE_BUFFER[cnt] = ((*color_bg)[1]<<i)&0x80?BIT_1:BIT_0;
        cnt++;
      }
      for(i = 0; i < 8; i++)
      {
        LED1_BYTE_BUFFER[cnt] = ((*color_bg)[2]<<i)&0x80?BIT_1:BIT_0;
        cnt++;
      }
      for(i = 0; i < 8; i++)
      {
        LED1_BYTE_BUFFER[cnt] = ((*color_bg)[3]<<i)&0x80?BIT_1:BIT_0;
        cnt++;
      }
    }

    lednum++;
  }

  for(;cnt < ledcnt1;cnt++)
  {
    LED1_BYTE_BUFFER[cnt] = 0;
  }

  HAL_TIM_PWM_Start_DMA(&htim3,TIM_CHANNEL_3,(uint32_t *)LED1_BYTE_BUFFER,cnt);
}

void ledstrip1_rgbw_alloff(void)
{
  ledstrip1_rgbw_perframe(lednumber1,lednumber1,&color_null,&color_null);
}

void ledstrip1_rgbw_forward(int len,int delayms,uint8_t (*color_led)[4],uint8_t (*color_bg)[4])
{
  int i = 0;

  for(; i <= len; i++)
  {
    ledstrip1_rgbw_perframe(i,lednumber1,color_led,color_bg);
    HAL_Delay(delayms);
  }
}

void ledstrip1_rgbw_backward(int len,int delayms,uint8_t (*color_led)[4],uint8_t (*color_bg)[4])
{
  int i = len;

  for(; i >= 0; i--)
  {
    ledstrip1_rgbw_perframe(i,lednumber1,color_led,color_bg);
    HAL_Delay(delayms);
  }
  
  ledstrip1_rgbw_alloff();
}

void ledstrip1_rgbw_brightset_WHITE(int len,int brightlevel)
{
  uint8_t color[4] = {0x00,0x00,0x00,0x00};
  color[3] = brightlevel;

  ledstrip1_rgbw_perframe(len,lednumber1,&color,&color_null);
}

void ledstrip2_rgbw_perframe(int number,int len,uint8_t (*color_led)[4],uint8_t (*color_bg)[4])
{
  int cnt = 0;
  int lednum = 0;
  int i;

  for(; cnt < 3; cnt++)
  {
    LED2_BYTE_BUFFER[cnt] = 0;
  }

  while(lednum <= len)
  {
    if(lednum <= number)
    {
      for(i = 0; i < 8; i++)
      {
        LED2_BYTE_BUFFER[cnt] = ((*color_led)[0]<<i)&0x80?BIT_1:BIT_0;
        cnt++;
      }
      for(i = 0; i < 8; i++)
      {
        LED2_BYTE_BUFFER[cnt] = ((*color_led)[1]<<i)&0x80?BIT_1:BIT_0;
        cnt++;
      }
      for(i = 0; i < 8; i++)
      {
        LED2_BYTE_BUFFER[cnt] = ((*color_led)[2]<<i)&0x80?BIT_1:BIT_0;
        cnt++;
      }
      for(i = 0; i < 8; i++)
      {
        LED2_BYTE_BUFFER[cnt] = ((*color_led)[3]<<i)&0x80?BIT_1:BIT_0;
        cnt++;
      }
    }
    else
    {
      for(i = 0; i < 8; i++)
      {
        LED2_BYTE_BUFFER[cnt] = ((*color_bg)[0]<<i)&0x80?BIT_1:BIT_0;
        cnt++;
      }
      for(i = 0; i < 8; i++)
      {
        LED2_BYTE_BUFFER[cnt] = ((*color_bg)[1]<<i)&0x80?BIT_1:BIT_0;
        cnt++;
      }
      for(i = 0; i < 8; i++)
      {
        LED2_BYTE_BUFFER[cnt] = ((*color_bg)[2]<<i)&0x80?BIT_1:BIT_0;
        cnt++;
      }
      for(i = 0; i < 8; i++)
      {
        LED2_BYTE_BUFFER[cnt] = ((*color_bg)[3]<<i)&0x80?BIT_1:BIT_0;
        cnt++;
      }
    }

    lednum++;
  }

  for(;cnt < ledcnt2; cnt++)
  {
    LED2_BYTE_BUFFER[cnt] = 0;
  }

	HAL_TIM_PWM_Start_DMA(&htim4,TIM_CHANNEL_1,(uint32_t *)LED2_BYTE_BUFFER,cnt);
}
	
void ledstrip2_rgbw_alloff(void)
{
	ledstrip2_rgbw_perframe(lednumber2,lednumber2,&color_null,&color_null);
}
	
void ledstrip2_rgbw_brightset_WHITE(int len,int brightlevel)
{
	uint8_t color[4] = {0x00,0x00,0x00,0x00};
	color[3] = brightlevel;

	ledstrip2_rgbw_perframe(len,lednumber2,&color,&color_null);
}

void ledgroup_rgbw_flow(int len,int delayms,uint8_t (*color_led)[4],uint8_t (*color_bg)[4])
{
	int i = 0;
	
	for(; i < lednumber1; i++)
	{
		ledstrip1_rgbw_perframe(i,lednumber1,color_led,color_bg);
		ledstrip2_rgbw_perframe(i,lednumber2,color_led,color_bg);
		
		HAL_Delay(delayms);
	}
}

/*
void ledstrip3_rgbw_perframe(int number,int len,uint8_t (*color_led)[4],uint8_t (*color_bg)[4])
{
  int cnt = 0;
  int lednum = 0;
  int i;

  for(; cnt < 3; cnt++)
  {
    LED3_BYTE_BUFFER[cnt] = 0;
  }

  while(lednum <= len)
  {
    if(lednum <= number)
    {
      for(i = 0; i < 8; i++)
      {
        LED3_BYTE_BUFFER[cnt] = ((*color_led)[0]<<i)&0x80?BIT_1:BIT_0;
        cnt++;
      }
      for(i = 0; i < 8; i++)
      {
        LED3_BYTE_BUFFER[cnt] = ((*color_led)[1]<<i)&0x80?BIT_1:BIT_0;
        cnt++;
      }
      for(i = 0; i < 8; i++)
      {
        LED3_BYTE_BUFFER[cnt] = ((*color_led)[2]<<i)&0x80?BIT_1:BIT_0;
        cnt++;
      }
      for(i = 0; i < 8; i++)
      {
        LED3_BYTE_BUFFER[cnt] = ((*color_led)[3]<<i)&0x80?BIT_1:BIT_0;
        cnt++;
      }
    }
    else
    {
      for(i = 0; i < 8; i++)
      {
        LED3_BYTE_BUFFER[cnt] = ((*color_bg)[0]<<i)&0x80?BIT_1:BIT_0;
        cnt++;
      }
      for(i = 0; i < 8; i++)
      {
        LED3_BYTE_BUFFER[cnt] = ((*color_bg)[1]<<i)&0x80?BIT_1:BIT_0;
        cnt++;
      }
      for(i = 0; i < 8; i++)
      {
        LED3_BYTE_BUFFER[cnt] = ((*color_bg)[2]<<i)&0x80?BIT_1:BIT_0;
        cnt++;
      }
      for(i = 0; i < 8; i++)
      {
        LED3_BYTE_BUFFER[cnt] = ((*color_bg)[3]<<i)&0x80?BIT_1:BIT_0;
        cnt++;
      }
    }

    lednum++;
  }

  for(;cnt < ledcnt3;cnt++)
  {
    LED3_BYTE_BUFFER[cnt] = 0;
  }

	HAL_TIM_PWM_Start_DMA(&htim2,TIM_CHANNEL_4,(uint32_t *)LED3_BYTE_BUFFER,cnt);
}

void ledstrip3_rgbw_alloff(void)
{
	ledstrip3_rgbw_perframe(lednumber3,lednumber3,&color_null,&color_null);
}
	
void ledstrip3_rgbw_brightset_WHITE(int len,int brightlevel)
{
	uint8_t color[4] = {0x00,0x00,0x00,0x00};
	color[3] = brightlevel;

	ledstrip3_rgbw_perframe(len,lednumber3,&color,&color_null);
}

void ledgroup_stepon(int delayms)
{
	int i = 0;
	
	for(; i <= lednumber1; i++)
	{
		ledstrip1_rgbw_perframe(i,lednumber1,&color_white,&color_null);
		ledstrip2_rgbw_brightset_WHITE(lednumber2,i*2);
		
		osDelay(delayms);
	}
}
	
void ledgroup_stepdown(int delayms)
{
	int i = lednumber1;
	
	for(; i >= 0; i--)
	{
		ledstrip1_rgbw_perframe(i,lednumber2,&color_white,&color_null);
		ledstrip2_rgbw_brightset_WHITE(lednumber2,i*2);
		
		osDelay(delayms);
	}
	
	ledstrip1_rgbw_alloff();
	ledstrip2_rgbw_alloff();
}
*/
