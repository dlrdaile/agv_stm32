/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "freertos_setup.h"
#include "SEGGER_RTT.h"
#include "event_groups.h"
#include "timers.h"
#include "UserConfig.h"
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
xQueueHandle canSendQueue = NULL;
xQueueHandle canUrgentQueue = NULL;
SemaphoreHandle_t canMutex = NULL;
EventGroupHandle_t feedDogEvent = NULL;
/* USER CODE END Variables */
osThreadId rosTaskHandle;
osThreadId CanNormalTaskHandle;
osThreadId CanUrgentTaskHandle;
osThreadId feedDogTaskHandle;
osThreadId keyTaskHandle;
osThreadId PressTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void rosCallback(void const * argument);
void CanNormalTCallbk(void const * argument);
void CanUrgentCallbk(void const * argument);
void feedDogCallbk(void const * argument);
void keyCheckCallbk(void const * argument);
void PressCallbk(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void) {

}

__weak unsigned long getRunTimeCounterValue(void) {
    return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize) {
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize) {
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
    *ppxTimerTaskStackBuffer = &xTimerStack[0];
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
    /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
    feedDogEvent = xEventGroupCreate();
    if (feedDogEvent == NULL) {
#if JLINK_DEBUG == 1
        SEGGER_RTT_printf(0, "eventgroup create error!\n");
#endif
        while (1);
    }
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    canMutex = xSemaphoreCreateMutex();
    if (canMutex == NULL) {
#if JLINK_DEBUG == 1
        SEGGER_RTT_printf(0, "canMutex create error!\n");
#endif
        while (1);
    }
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    if(start_timer() != HAL_OK){
#if JLINK_DEBUG == 1
        SEGGER_RTT_printf(0, "time create error!\n");
#endif
        while (1);
    }
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    if (create_Queue() != HAL_OK) {
#if JLINK_DEBUG == 1
        SEGGER_RTT_printf(0, "queue create error!\n");
#endif
        while (1);
    }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of rosTask */
  osThreadDef(rosTask, rosCallback, osPriorityHigh, 0, 2048);
  rosTaskHandle = osThreadCreate(osThread(rosTask), NULL);

  /* definition and creation of CanNormalTask */
  osThreadDef(CanNormalTask, CanNormalTCallbk, osPriorityNormal, 0, 512);
  CanNormalTaskHandle = osThreadCreate(osThread(CanNormalTask), NULL);

  /* definition and creation of CanUrgentTask */
  osThreadDef(CanUrgentTask, CanUrgentCallbk, osPriorityAboveNormal, 0, 512);
  CanUrgentTaskHandle = osThreadCreate(osThread(CanUrgentTask), NULL);

  /* definition and creation of feedDogTask */
  osThreadDef(feedDogTask, feedDogCallbk, osPriorityRealtime, 0, 128);
  feedDogTaskHandle = osThreadCreate(osThread(feedDogTask), NULL);

  /* definition and creation of keyTask */
  osThreadDef(keyTask, keyCheckCallbk, osPriorityAboveNormal, 0, 128);
  keyTaskHandle = osThreadCreate(osThread(keyTask), NULL);

  /* definition and creation of PressTask */
  osThreadDef(PressTask, PressCallbk, osPriorityBelowNormal, 0, 128);
  PressTaskHandle = osThreadCreate(osThread(PressTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
#if JLINK_DEBUG == 1
    SEGGER_RTT_printf(0,"start!");
#endif
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_rosCallback */
/**
  * @brief  Function implementing the rosTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_rosCallback */
__weak void rosCallback(void const * argument)
{
  /* USER CODE BEGIN rosCallback */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
  /* USER CODE END rosCallback */
}

/* USER CODE BEGIN Header_CanNormalTCallbk */
/**
* @brief Function implementing the CanNormalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CanNormalTCallbk */
__weak void CanNormalTCallbk(void const * argument)
{
  /* USER CODE BEGIN CanNormalTCallbk */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
  /* USER CODE END CanNormalTCallbk */
}

/* USER CODE BEGIN Header_CanUrgentCallbk */
/**
* @brief Function implementing the CanUrgentTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CanUrgentCallbk */
__weak void CanUrgentCallbk(void const * argument)
{
  /* USER CODE BEGIN CanUrgentCallbk */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
  /* USER CODE END CanUrgentCallbk */
}

/* USER CODE BEGIN Header_feedDogCallbk */
/**
* @brief Function implementing the feedDogTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_feedDogCallbk */
__weak void feedDogCallbk(void const * argument)
{
  /* USER CODE BEGIN feedDogCallbk */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
  /* USER CODE END feedDogCallbk */
}

/* USER CODE BEGIN Header_keyCheckCallbk */
/**
* @brief Function implementing the keyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_keyCheckCallbk */
__weak void keyCheckCallbk(void const * argument)
{
  /* USER CODE BEGIN keyCheckCallbk */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
  /* USER CODE END keyCheckCallbk */
}

/* USER CODE BEGIN Header_PressCallbk */
/**
* @brief Function implementing the PressTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PressCallbk */
__weak void PressCallbk(void const * argument)
{
  /* USER CODE BEGIN PressCallbk */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END PressCallbk */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
