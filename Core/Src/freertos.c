/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
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
#include "led.h"            /* LED_Red()      —— 亮红灯，留下"死因" */
#include "sys_timestamp.h"  /* Sys_Delay_US() —— 让红灯亮够时间再复位 */
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

/* USER CODE END Variables */
/* Definitions for TIM_1ms */
osThreadId_t TIM_1msHandle;
const osThreadAttr_t TIM_1ms_attributes = {
  .name = "TIM_1ms",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BMI088 */
osThreadId_t BMI088Handle;
const osThreadAttr_t BMI088_attributes = {
  .name = "BMI088",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void TIM_1ms_Task(void *argument);
void BMI088_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
/**
 * @brief 任务栈溢出钩子
 * @note  ★ 这个函数是在【中断上下文】里被调用的！
 *           FreeRTOS 在 PendSV 里做任务切换，切换前检查栈有没有被写穿，
 *           发现异常就跳到这里。所以：
 *             · 不能用阻塞式发送
 *             · 不能调用任何 FreeRTOS API（除了专门给中断用的 ...FromISR）
 *
 *        ★ 处理完为什么复位而不是原地死循环：
 *           栈都写穿了，那块 RAM 已经不可信。硬撑着跑只会产生
 *           "更难查的故障"（你这次遇到的 memchr 崩溃就是典型）。
 *           重启后系统至少是干净的。
 *
 *        ★ 怎么知道出过这事：串口只发波形、没有日志，所以只能靠"灯"——
 *           红灯常亮 50ms 然后复位；如果故障反复发生，看到的就是【红灯反复闪】。
 *           要进一步定位（哪个任务、栈用了多少），把它接进内存快照或临时恢复日志。
 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
  (void)xTask;                                                    /* 用不到，避免告警 */
  (void)pcTaskName;

  LED_Red();                                                      /* ① 红灯常亮 */
  Sys_Delay_US(50000.0f);                                         /* ② 等 50ms 让灯亮够 */
  NVIC_SystemReset();                                             /* ③ 复位 */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
/**
 * @brief FreeRTOS 堆耗尽钩子
 * @note  走到这里 = 32KB 的 ucHeap 用光了，说明有地方在运行中
 *        不停创建对象（任务/队列/信号量）却没有删。
 *        这个钩子是在【任务上下文】里调用的，比上面那个安全。
 *        同样：红灯亮 50ms 后复位（没有日志可打）。
 */
void vApplicationMallocFailedHook(void)
{
  LED_Red();
  Sys_Delay_US(50000.0f);
  NVIC_SystemReset();
}
/* USER CODE END 5 */

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
  /* creation of TIM_1ms */
  TIM_1msHandle = osThreadNew(TIM_1ms_Task, NULL, &TIM_1ms_attributes);

  /* creation of BMI088 */
  BMI088Handle = osThreadNew(BMI088_Task, NULL, &BMI088_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_TIM_1ms_Task */
/**
  * @brief  Function implementing the TIM_1ms thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_TIM_1ms_Task */
__weak void TIM_1ms_Task(void *argument)
{
  /* USER CODE BEGIN TIM_1ms_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TIM_1ms_Task */
}

/* USER CODE BEGIN Header_BMI088_Task */
/**
* @brief Function implementing the BMI088 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BMI088_Task */
__weak void BMI088_Task(void *argument)
{
  /* USER CODE BEGIN BMI088_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END BMI088_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

