/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "chassis.h"
#include "stdio.h"
#include "oled.h"
#include "serial.h"
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
osThreadId_t chassisTaskHandle;
const osThreadAttr_t chassisTask_attributes = {
    .name       = "chassisTask",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};
osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = {
    .name       = "oledTask",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};
osThreadId_t steerTaskHandle;
const osThreadAttr_t steerTask_attributes = {
    .name       = "steerTask",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};
osThreadId_t decodeTaskHandle;
const osThreadAttr_t decodeTask_attributes = {
    .name       = "decodeTask",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name       = "defaultTask",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void ChassisTask(void *argument);
void OLEDTask(void *argument);
void SteerTask(void *argument);
void DecodeTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
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
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    chassisTaskHandle = osThreadNew(ChassisTask, NULL, &chassisTask_attributes);
    oledTaskHandle    = osThreadNew(OLEDTask, NULL, &oledTask_attributes);
    steerTaskHandle   = osThreadNew(SteerTask, NULL, &steerTask_attributes);
    decodeTaskHandle  = osThreadNew(DecodeTask, NULL, &decodeTask_attributes);
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
    /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
    /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
float vx;
float vy;
float wc;

void ChassisTask(void *argument)
{
    // Init Motor
    Motor_Init(&motor1, 1, 70, 1, 0.1);
    Motor_Init(&motor2, 2, 70, 1, 0.1);
    Motor_Init(&motor3, 3, 70, 1, 0.1);
    Motor_Init(&motor4, 4, 70, 1, 0.1);
    Motor_Start(&motor1);
    Motor_Start(&motor2);
    Motor_Start(&motor3);
    Motor_Start(&motor4);
    Inverse_kinematic_equation(0.0, 0.0, 0.0);
    for (;;) {
        Forward_kinematic_equation(&vx, &vy, &wc);
        osDelay(10);
    }
}

void OLEDTask(void *argument)
{
    OLED_Init();
    for (;;) {

        OLED_ShowString(0, 0, (char *)("Speed:"), 6, 0);
        OLED_ShowNum(0, 1, (uint32_t)(ball_x.float_data), 3, 6, 0);
        osDelay(10);
    }
}

void DecodeTask(void *argument)
{
    for (;;) {
        Serial_Decode();
        osDelay(10);
    }
}

void SteerTask(void *argument)
{
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 1200);

    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 1600);

    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 1500);

    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 1000);

    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 1750);
    for (;;) {

        // __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 1000);
        // __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 1000);
        // __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 1000);
        osDelay(1);
    }
}
/* USER CODE END Application */
