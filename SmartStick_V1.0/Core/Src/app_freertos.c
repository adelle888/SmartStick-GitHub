/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "mpu6050.h"
#include "i2c.h"
#include "hr04.h"
#include "tim.h"
#include "usart.h"
#include "gps.h"
#include "oled.h"
#include "adc.h"
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>    /* atof */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile uint8_t GPS_RX_BUF;
volatile uint16_t GPS_RX_INDEX = 0;
volatile uint32_t adc_value = 0;
volatile uint8_t light_percent = 0;
char oled_buf[20];
float distance;
float pitch = 0.0f, roll = 0.0f;          // 融合角度
float pitch_off = 0.0f, roll_off = 0.0f;  // 校准偏移量
const float alpha = 0.98f;                // 滤波系数

float pitch_real = 0.0f,roll_real = 0.0f;
char gps_lat_str[16] = "0.000000";   /* 经度字符串 */
char gps_lon_str[16] = "0.000000";   /* 纬度字符串 */
uint8_t gps_valid = 0;               /* GPS是否有效 */
uint8_t sms_rx_buf[128];

#define RAD_TO_DEG 57.2957795f
#define ALPHA       0.98f
#define RAD_TO_DEG  57.2957795f
#define DT          0.01f        // 与 osDelay(10) 对应，单位秒

MPU6050_t MPU6050;
osThreadId lightTaskHandle;
osThreadId oledTaskHandle;
osThreadId buzzerTaskHandle;
osThreadId mpuTaskHandle;
osThreadId gpsTaskHandle;
osThreadId hcsr04TaskHandle;
osThreadId ledTaskHandle;
osSemaphoreId smsTriggerSemaphore;   /* PC6 触发信号量 */
osThreadId    smsTaskHandle;         /* SMS 任务句柄 */
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartMpuTask(void const * argument);
void Mpu6050_Function(I2C_HandleTypeDef* i2cHandle, MPU6050_t *DataStruct);
void StartGpsTask(void const * argument);
void StartHcsr04Task(void const * argument);
void StartLightTask(void const * argument);
void StartOledTask(void const * argument);
void StartBuzzerTask(void const * argument);
void StartLedTask(void const * argument);
void StartSmsTask(void const * argument);          /* SMS 任务 */
void A7670_SendCmd(const char *cmd, uint32_t delay_ms);
void A7670_SendSMS(const char *phone, const char *message);
void A7670_SendCmdWithResponse(const char *cmd, uint32_t delay_ms);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(mpuTask, StartMpuTask, osPriorityNormal, 0, 256);
  mpuTaskHandle = osThreadCreate(osThread(mpuTask), NULL);
    
  osThreadDef(gpsTask, StartGpsTask, osPriorityNormal, 0, 256);
  gpsTaskHandle = osThreadCreate(osThread(gpsTask), NULL);

  osThreadDef(hcsr04Task, StartHcsr04Task, osPriorityBelowNormal, 0, 256);
  hcsr04TaskHandle = osThreadCreate(osThread(hcsr04Task), NULL);

  osThreadDef(lightTask, StartLightTask, osPriorityNormal, 0, 256);
  lightTaskHandle = osThreadCreate(osThread(lightTask), NULL);

  osThreadDef(oledTask, StartOledTask, osPriorityBelowNormal, 0, 256);
  oledTaskHandle = osThreadCreate(osThread(oledTask), NULL);
	

  osThreadDef(buzzerTask, StartBuzzerTask, osPriorityBelowNormal, 0, 256);
  buzzerTaskHandle = osThreadCreate(osThread(buzzerTask), NULL);
  
  osThreadDef(ledTask, StartLedTask, osPriorityNormal, 0, 128);
  ledTaskHandle = osThreadCreate(osThread(ledTask), NULL);
  
  osSemaphoreDef(smsTrigger);
  smsTriggerSemaphore = osSemaphoreCreate(osSemaphore(smsTrigger), 1);
  osSemaphoreWait(smsTriggerSemaphore, 0);  /* 清空初始计数 */
	
  osThreadDef(smsTask, StartSmsTask, osPriorityNormal, 0, 256);
  smsTaskHandle = osThreadCreate(osThread(smsTask), NULL);
	 

  
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        Hcsr04TimIcIsr(htim);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        Hcsr04TimOverflowIsr(htim);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        if (GPS_RX_BUF == '$')
        {
            GPS_RX_INDEX = 0;
        }

        if (GPS_RX_INDEX < GPS_Buffer_Length - 1)
        {
            Save_Data.GPS_Buffer[GPS_RX_INDEX++] = GPS_RX_BUF;

            if (GPS_RX_BUF == '\n')
            {
                Save_Data.GPS_Buffer[GPS_RX_INDEX] = '\0';
                Save_Data.isGetData = true;
            }
        }
        else
        {
            GPS_RX_INDEX = 0;
        }

        HAL_UART_Receive_IT(&huart2, &GPS_RX_BUF, 1);
    }
}


/* ========== 校准函数 ========== */
void MPU6050_Calibrate(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu)
{
    float p = 0, r = 0;

    // 先预热滤波器50次，丢弃结果
    for (uint8_t i = 0; i < 50; i++)
    {
        MPU6050_Read_All(hi2c, mpu);
        float acc_p = atan2f(mpu->Ay,
                       sqrtf(mpu->Ax*mpu->Ax + mpu->Az*mpu->Az)) * RAD_TO_DEG;
        float acc_r = atan2f(-mpu->Ax, mpu->Az) * RAD_TO_DEG;

        //  正确互补滤波（陀螺仪积分 + 加速度计修正）
        float gyroPitch = mpu->Gy * DT;  // 陀螺仪角速度 × dt
        float gyroRoll  = mpu->Gx * DT;
        pitch = ALPHA * (pitch + gyroPitch) + (1.0f - ALPHA) * acc_p;
        roll  = ALPHA * (roll  + gyroRoll)  + (1.0f - ALPHA) * acc_r;
        osDelay(10);
    }

    // 再采集100次求平均，作为偏移量
    for (uint8_t i = 0; i < 100; i++)
    {
        MPU6050_Read_All(hi2c, mpu);
        float acc_p = atan2f(mpu->Ay,
                       sqrtf(mpu->Ax*mpu->Ax + mpu->Az*mpu->Az)) * RAD_TO_DEG;
        float acc_r = atan2f(-mpu->Ax, mpu->Az) * RAD_TO_DEG;

        float gyroPitch = mpu->Gy * DT;
        float gyroRoll  = mpu->Gx * DT;
        pitch = ALPHA * (pitch + gyroPitch) + (1.0f - ALPHA) * acc_p;
        roll  = ALPHA * (roll  + gyroRoll)  + (1.0f - ALPHA) * acc_r;

        p += pitch;
        r += roll;
        osDelay(10);
    }

    pitch_off = p / 100.0f;
    roll_off  = r / 100.0f;
    printf("校准完成！偏移：Pitch=%.2f° Roll=%.2f°\r\n", pitch_off, roll_off);
}

/* ========== 主运行函数 ========== */
void Mpu6050_Function(I2C_HandleTypeDef* i2cHandle, MPU6050_t *DataStruct)
{
    MPU6050_Read_All(i2cHandle, DataStruct);

    float accPitch = atan2f(DataStruct->Ay,
                      sqrtf(DataStruct->Ax*DataStruct->Ax +
                            DataStruct->Az*DataStruct->Az)) * RAD_TO_DEG;
    float accRoll  = atan2f(-DataStruct->Ax, DataStruct->Az) * RAD_TO_DEG;

    float gyroPitch = DataStruct->Gy * DT;
    float gyroRoll  = DataStruct->Gx * DT;
    pitch = ALPHA * (pitch + gyroPitch) + (1.0f - ALPHA) * accPitch;
    roll  = ALPHA * (roll  + gyroRoll)  + (1.0f - ALPHA) * accRoll;

    pitch_real = pitch - pitch_off;
    roll_real  = roll  - roll_off;

//    printf("Pitch: %.1f° | Roll: %.1f°\r\n", pitch_real, roll_real);

    // ========== 跌倒检测状态机 ==========
    static uint8_t  fall_state      = 0;
    static uint32_t fall_first_tick = 0;

    if (fabsf(roll_real) > 60.0f)
    {
        if (fall_state == 0)
        {
            fall_state      = 1;
            fall_first_tick = osKernelSysTick();
            printf("疑似跌倒，10s后二次确认...\r\n");
        }
        else if (fall_state == 1)
        {
            uint32_t elapsed = osKernelSysTick() - fall_first_tick;
            if (elapsed >= 10000)
            {
                printf("跌倒确认！正在通知监护人...\r\n");
                osSemaphoreRelease(smsTriggerSemaphore);  // ← 触发短信
                fall_state = 2;  // 进入锁定态，不再重复触发
            }
        }
    }
    else
    {
        if (fall_state == 1)
        {
            printf("角度已恢复，取消疑似跌倒。\r\n");
        }
        // 注意：fall_state==2 不重置，保持锁定
        if (fall_state != 2)
        {
            fall_state = 0;
        }
    }
}

/* ========== 任务入口 ========== */
void StartMpuTask(void const * argument)
{
    osDelay(2000);
    while (MPU6050_Init(&hi2c1) == 1)
    {
        printf("MPU6050 初始化失败\r\n");
        osDelay(500);
    }
    printf("MPU6050 初始化成功\r\n");

    printf("请平放设备，3秒后开始校准...\r\n");
    osDelay(3000);
    MPU6050_Calibrate(&hi2c1, &MPU6050);

    for (;;)
    {
        Mpu6050_Function(&hi2c1, &MPU6050);
        osDelay(10);   // 10ms = 100Hz 刷新率
    }
 }


/* GPS模块任务 */
void StartGpsTask(void const * argument)
{
    osDelay(100);

    printf("gps task start\r\n");
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&GPS_RX_BUF, 1);

    for(;;)
    {
        printf("gps task alive\r\n");
        parseGpsBuffer();
        printGpsBuffer();
        osDelay(1000);
    }
}
/* Hr04测距模块任务 */
void StartHcsr04Task(void const * argument)
{
    osDelay(100);

    Hcsr04Init(&htim2, TIM_CHANNEL_1);
    printf("hc-sr04 start!\r\n");

    for(;;)
    {
        osDelay(300);
    }
}
/* 光敏传感器任务 */
void StartLightTask(void const * argument)
{
  osDelay(100);

  for(;;)
  {
    if (HAL_ADC_Start(&hadc1) == HAL_OK)
    {
      if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
      {
        adc_value = HAL_ADC_GetValue(&hadc1);
        light_percent = 100 - (adc_value * 100 / 4095);
      }
      HAL_ADC_Stop(&hadc1);
    }

    osDelay(200);
  }
}

/* OLED显示任务 */
void StartOledTask(void const * argument)
{
    osDelay(200);

    for (;;)
    {
        OLED_Clear();

        /* 光照 */
        sprintf(oled_buf, "L:%d%%", light_percent);
        OLED_ShowString(0, 0, (uint8_t *)oled_buf, 16);

        /* GPS 显示经纬度，格式：22.07.48 113.30.26 */
        if (gps_valid)
            sprintf(oled_buf, "%s %s", gps_lat_str, gps_lon_str);
        else
            sprintf(oled_buf, "GPS: waiting...");
        OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);

        /* 距离 */
        distance = Hcsr04ReadFiltered();
        sprintf(oled_buf, "Dist:%.1f cm", distance);
        OLED_ShowString(0, 4, (uint8_t *)oled_buf, 16);

        /* Pitch / Roll */
        sprintf(oled_buf, "P:%.1f R:%.1f", pitch_real, roll_real);
        OLED_ShowString(0, 6, (uint8_t *)oled_buf, 16);

        osDelay(500);
    }
}


/* 蜂鸣器任务 */
void StartBuzzerTask(void const * argument)
{
    osDelay(200);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

    for(;;)
    {
        // 阈值：小于10cm报警
        if (distance < 10.0f && distance > 0) 
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);  // 响
        }
        else
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);// 不响
        }
        osDelay(100); 
    }
}

/* 照明灯任务 */
void StartLedTask(void const * argument)
{
    osDelay(100);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

    for(;;)
    {
        if (light_percent < 50)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
            printf("光照=%d%%, 照明灯打开\r\n", light_percent);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
            printf("光照=%d%%, 照明灯关闭\r\n", light_percent);
        }

        osDelay(500);
    }
}

/* ============================================================
 * SMS 模块实现
 * ============================================================ */

/* 发送 AT 指令并等待指定时间 */
void A7670_SendCmd(const char *cmd, uint32_t delay_ms)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)cmd, strlen(cmd), 1000);
    osDelay(delay_ms);
}

/* 发送短信（Text 模式） */
void A7670_SendSMS(const char *phone, const char *message)
{
    char cmd[64];
    uint8_t ctrlZ = 0x1A;

    printf("[SMS] 开始发送短信至 %s\r\n", phone);

    A7670_SendCmd("AT+CMGF=1\r\n", 500);
    A7670_SendCmd("AT+CSCS=\"UCS2\"\r\n", 500);   /* 改为 UCS2 支持中文 */

    sprintf(cmd, "AT+CMGS=\"%s\"\r\n", phone);
    A7670_SendCmd(cmd, 500);

    A7670_SendCmd(message, 100);

    HAL_UART_Transmit(&huart3, &ctrlZ, 1, 1000);
    osDelay(5000);

    printf("[SMS] 发送完成\r\n");
}


/* SMS FreeRTOS 任务 */

void StartSmsTask(void const * argument)
{
    osDelay(5000);
    printf("[SMS] SMS任务就绪，等待跌倒触发...\r\n");

    for (;;)
    {
        if (osSemaphoreWait(smsTriggerSemaphore, osWaitForever) == osOK)
        {
            printf("[SMS] 收到跌倒信号，正在发送短信...\r\n");

            char sms_content[320];

            if (gps_valid)
            {
                /* UCS2编码说明：
                 * 8001BACD75AEFFBF8ECC272C8BF7559059052001BACD662F5426AE895168FF01
                 * = "老人疑似跌倒超过10s，请留意老人是否安全！"
                 * 7ECF5EA6FF1A = "经度："
                 * 7EAF5EA6FF1A = "纬度："
                 * 经纬度数字直接转ASCII对应UCS2（前补00）
                 */

                /* 把经纬度字符串转为UCS2 */
                char lat_ucs2[64] = {0};
                char lon_ucs2[64] = {0};
                for (int i = 0; i < strlen(gps_lat_str); i++)
                    sprintf(lat_ucs2 + i*4, "00%02X", (uint8_t)gps_lat_str[i]);
                for (int i = 0; i < strlen(gps_lon_str); i++)
                    sprintf(lon_ucs2 + i*4, "00%02X", (uint8_t)gps_lon_str[i]);

                snprintf(sms_content, sizeof(sms_content),
                    "8001BACD75AEFFBF8ECC272C8BF7559059052001BACD662F5426AE895168FF01"
                    "7ECF5EA6FF1A%s"
                    "00200077EFBA5EA6FF1A%s",
                    lat_ucs2, lon_ucs2);
            }
            else
            {
                /* GPS无效时只发英文提示 */
                snprintf(sms_content, sizeof(sms_content),
                    "8001BACD75AEFFBF8ECC272C8BF7559059052001BACD662F5426AE895168FF01"
                    "0047005000530020004E0041");
            }

            printf("[SMS] 发送内容: %s\r\n", sms_content);
            A7670_SendSMS("123456789", sms_content); //修改为实际的求救电话

            printf("[SMS] 短信已发送，任务结束。\r\n");
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
            vTaskSuspend(NULL);
        }
    }
}


/* USER CODE END Application */

