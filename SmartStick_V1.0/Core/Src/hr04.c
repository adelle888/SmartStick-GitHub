/*
 * @Author       : yzy
 * @Date         : 2021-05-31 17:03:23
 * @LastEditors  : yzy
 * @LastEditTime : 2021-06-21 22:16:28
 * @Description  : 
 * @FilePath     : \CSDN_HC-SR04_GPIO\BSP_HARDWARE\HC-SR04\hc-sr04.c
 */
#include "hr04.h"

Hcsr04InfoTypeDef Hcsr04Info;

/**
 * @description: 超声波模块的输入捕获定时器通道初始化
 * @param {TIM_HandleTypeDef} *htim
 * @param {uint32_t} Channel
 * @return {*}
 */
void Hcsr04Init(TIM_HandleTypeDef *htim, uint32_t Channel)
{
  /*--------[ Configure The HCSR04 IC Timer Channel ] */
  Hcsr04Info.ic_tim_ch = Channel;
  Hcsr04Info.prescaler = htim->Init.Prescaler; //  160-1
  Hcsr04Info.period = htim->Init.Period;       //  65535
  Hcsr04Info.edge_state = 0;
  Hcsr04Info.tim_overflow_counter = 0;

  Hcsr04Info.instance = htim->Instance;        
  if(Hcsr04Info.ic_tim_ch == TIM_CHANNEL_1)
  {
    Hcsr04Info.active_channel = HAL_TIM_ACTIVE_CHANNEL_1;             //  TIM_CHANNEL_1
  }
  else if(Hcsr04Info.ic_tim_ch == TIM_CHANNEL_2)
  {
    Hcsr04Info.active_channel = HAL_TIM_ACTIVE_CHANNEL_2;             //  TIM_CHANNEL_2
  }
  else if(Hcsr04Info.ic_tim_ch == TIM_CHANNEL_3)
  {
    Hcsr04Info.active_channel = HAL_TIM_ACTIVE_CHANNEL_3;             //  TIM_CHANNEL_3
  }
  else if(Hcsr04Info.ic_tim_ch == TIM_CHANNEL_4)
  {
    Hcsr04Info.active_channel = HAL_TIM_ACTIVE_CHANNEL_4;             //  TIM_CHANNEL_4
  }
  /*--------[ Start The ICU Channel ]-------*/
  HAL_TIM_Base_Start_IT(htim);
  HAL_TIM_IC_Start_IT(htim, Channel);//开启中断
}

/**
 * @description: HC-SR04触发
 * @param {*}
 * @return {*}
 */
void Hcsr04Start(void)
{
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
  HAL_Delay(1);  //  10us以上
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
}

/**
 * @description: 定时器计数溢出中断处理函数
 * @param {*}    main.c中重定义void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
 * @return {*}
 */
void Hcsr04TimOverflowIsr(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == Hcsr04Info.instance) //  
  {
    Hcsr04Info.tim_overflow_counter++;
  }
}

/**
 * @description: 输入捕获计算高电平时间->距离
 * @param {*}    main.c中重定义void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
 * @return {*}
 */
void Hcsr04TimIcIsr(TIM_HandleTypeDef* htim)
{
  if((htim->Instance == Hcsr04Info.instance) && (htim->Channel == Hcsr04Info.active_channel))
  {
    if(Hcsr04Info.edge_state == 0)      //  捕获上升沿
    {
      // 得到上升沿开始时间T1，并更改输入捕获为下降沿
      Hcsr04Info.t1 = HAL_TIM_ReadCapturedValue(htim, Hcsr04Info.ic_tim_ch);
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, Hcsr04Info.ic_tim_ch, TIM_INPUTCHANNELPOLARITY_FALLING);
      Hcsr04Info.tim_overflow_counter = 0;  //  定时器溢出计数器清零
      Hcsr04Info.edge_state = 1;        //  上升沿、下降沿捕获标志位
    }
    else if(Hcsr04Info.edge_state == 1) //  捕获下降沿
    {
      // 捕获下降沿时间T2，并计算高电平时间
      Hcsr04Info.t2 = HAL_TIM_ReadCapturedValue(htim, Hcsr04Info.ic_tim_ch);
      Hcsr04Info.t2 += Hcsr04Info.tim_overflow_counter * Hcsr04Info.period; //  需要考虑定时器溢出中断
      Hcsr04Info.high_level_us = Hcsr04Info.t2 - Hcsr04Info.t1; //  高电平持续时间 = 下降沿时间点 - 上升沿时间点
      // 计算距离
//      Hcsr04Info.distance = (Hcsr04Info.high_level_us / 1000000.0) * 340.0 / 2.0 * 100.0;
		 Hcsr04Info.distance = Hcsr04Info.high_level_us / 58.0;
      // 重新开启上升沿捕获
      Hcsr04Info.edge_state = 0;  //  一次采集完毕，清零
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, Hcsr04Info.ic_tim_ch, TIM_INPUTCHANNELPOLARITY_RISING);
    }
  }
}

/**
 * @description: 读取距离 
 * @param {*}
 * @return {*}
 */
float Hcsr04Read(void)
{
  // 测距结果限幅
  if(Hcsr04Info.distance >= 450)
  {
    Hcsr04Info.distance = 450;
  }
  if(Hcsr04Info.distance <= 2)
	{
    Hcsr04Info.distance = 0;
	}
  return Hcsr04Info.distance;
}

float Hcsr04ReadFiltered(void)
{
    float d[5];

    for(int i=0;i<5;i++)
    {
        Hcsr04Start();
        HAL_Delay(60);
        d[i] = Hcsr04Read();
    }

    // 中值滤波
    for(int i=0;i<4;i++)
        for(int j=i+1;j<5;j++)
            if(d[i] > d[j])
            {
                float t = d[i];
                d[i] = d[j];
                d[j] = t;
            }

    return d[2];
}


