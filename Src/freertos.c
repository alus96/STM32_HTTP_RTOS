/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "MPU9250.h"
#include "spi.h"
#include "HTS221.h"
#include "GZY_LRF.h"
#include "usart.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId mpuListenerHandle;
osThreadId dataFilterHandle;
osThreadId dhtTaskHandle;
osMessageQId filterQueueHandle;
osMessageQId filteredImuQueueHandle;
osSemaphoreId filterDoneSemHandle;

/* USER CODE BEGIN Variables */
#define FILTER_SIZE 8

osMessageQId tempDataQueueHandle;
osThreadId gzyMessageSendeHandle;
osThreadId gzyPacketParserHandle;
osSemaphoreId lrfDataReceivedHandle;
osSemaphoreId lrfMeasureCommandHandle;
osMessageQId lrfDataQueueHandle;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void getMpuData(void const * argument);
void filterSomeData(void const * argument);
void readDht(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void filterThread(void const * argument);
uint16_t read_cycle(uint16_t cur_tics, uint8_t neg_tic);

void sendLRFMessage(void const * argument);
void parseGZYPacket(void const * argument);
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of filterDoneSem */
  osSemaphoreDef(filterDoneSem);
  filterDoneSemHandle = osSemaphoreCreate(osSemaphore(filterDoneSem), 3);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  osSemaphoreDef(lrfMeasureCommand);
   lrfMeasureCommandHandle = osSemaphoreCreate(osSemaphore(lrfMeasureCommand), 1);

   osSemaphoreDef(lrfDataReceived);
    lrfDataReceivedHandle = osSemaphoreCreate(osSemaphore(lrfDataReceived), 1);

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of mpuListener */
  osThreadDef(mpuListener, getMpuData, osPriorityHigh, 0, 1024);
  mpuListenerHandle = osThreadCreate(osThread(mpuListener), NULL);

  /* definition and creation of dataFilter */
  osThreadDef(dataFilter, filterSomeData, osPriorityAboveNormal, 0, 512);
  dataFilterHandle = osThreadCreate(osThread(dataFilter), NULL);

  /* definition and creation of dhtTask */
  osThreadDef(dhtTask, readDht, osPriorityRealtime, 0, 128);
  dhtTaskHandle = osThreadCreate(osThread(dhtTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
//  osThreadDef(gzyMessageSende, sendLRFMessage, osPriorityHigh, 0, 512);
//   gzyMessageSendeHandle = osThreadCreate(osThread(gzyMessageSende), NULL);

   /* definition and creation of gzyPacketParser */
//   osThreadDef(gzyPacketParser, parseGZYPacket, osPriorityHigh, 0, 128);
//   gzyPacketParserHandle = osThreadCreate(osThread(gzyPacketParser), NULL);

  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of filterQueue */
  osMessageQDef(filterQueue, 8, imuStruct);
  filterQueueHandle = osMessageCreate(osMessageQ(filterQueue), NULL);

  /* definition and creation of filteredImuQueue */
  osMessageQDef(filteredImuQueue, 8, imuStruct);
  filteredImuQueueHandle = osMessageCreate(osMessageQ(filteredImuQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  osMessageQDef(lrfDataQueue, 4, gzyDistanceStruct);
   lrfDataQueueHandle = osMessageCreate(osMessageQ(lrfDataQueue), NULL);

  osMessageQDef(tempDataQueue, 4, tempSensor);
  tempDataQueueHandle = osMessageCreate(osMessageQ(tempDataQueue), NULL);
  /* USER CODE END RTOS_QUEUES */
}

/* getMpuData function */
void getMpuData(void const * argument)
{

  /* USER CODE BEGIN getMpuData */
  /* Infinite loop */
  initMPU(&hspi1);
  for(;;)
  {
	MPU9250_TakeAndCalcData();
	MPU9250_CalcYPR();
	imuStruct rawMpu = imuGetData();
	xQueueSendToBack(filterQueueHandle, &rawMpu, 0);
    osDelay(1);
  }
  /* USER CODE END getMpuData */
}

/* filterSomeData function */
void filterSomeData(void const * argument)
{
  /* USER CODE BEGIN filterSomeData */
  /* Infinite loop */
  float filterYaw[FILTER_SIZE];
  float filterPitch[FILTER_SIZE];
  float filterRoll[FILTER_SIZE];

  uint8_t counter = 0;
  xSemaphoreTake(filterDoneSemHandle, 0);
  for(;;)
  {
	 imuStruct imuData;
	 xQueueReceive(filterQueueHandle, &imuData, portMAX_DELAY);

	 filterYaw[counter] = imuData.yaw;
	 filterPitch[counter] = imuData.pitch;
	 filterRoll[counter++] = imuData.roll;

	 if(counter == FILTER_SIZE){
		 uint8_t i, j;

		 sys_thread_new("FILTER_YAW", filterThread, &filterYaw, 128, osPriorityHigh);
		 sys_thread_new("FILTER_PITCH", filterThread, &filterPitch, 128, osPriorityHigh);
		 sys_thread_new("FILTER_ROLL", filterThread, &filterRoll, 128, osPriorityHigh);

		 xSemaphoreTake(filterDoneSemHandle, portMAX_DELAY);
		 xSemaphoreTake(filterDoneSemHandle, portMAX_DELAY);
		 xSemaphoreTake(filterDoneSemHandle, portMAX_DELAY);

		 imuData.yaw = filterYaw[FILTER_SIZE / 2];
		 imuData.pitch = filterPitch[FILTER_SIZE / 2];
		 imuData.roll = filterRoll[FILTER_SIZE / 2];
		 counter = 0;
		 xQueueSendToBack(filteredImuQueueHandle, &imuData, 0);
	 }


  }
  /* USER CODE END filterSomeData */
}

/* readDht function */
void readDht(void const * argument)
{
  /* USER CODE BEGIN readDht */
  /* Infinite loop */
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
  for(;;)
  {
    HTS221_Get_Humidity();
	HTS221_Get_Temperature();

	xQueueSendToBack(tempDataQueueHandle, &tempData, 0);
	vTaskDelayUntil( &xLastWakeTime, 1000);
  }
  /* USER CODE END readDht */
}

/* USER CODE BEGIN Application */
void filterThread(void const * argument){
	float * data = (float *) argument;

	uint8_t i, j;
	for(i = 0; i < FILTER_SIZE - 1; ++i){
		 for(j = i + 1; j < FILTER_SIZE; ++j){
			 if(data[i] < data[j]){
				 float tmp = data[i];
				 data[i] = data[j];
				 data[j] = tmp;
			 }
		 }
	 }

	xSemaphoreGive(filterDoneSemHandle);
	osThreadTerminate(NULL);
}

void sendLRFMessage(void const * argument)
{

  /* USER CODE BEGIN sendLRFMessage */
	/* Infinite loop */
//	initGZY(&huart1);
	xSemaphoreTake(lrfMeasureCommandHandle, 0);
	for (;;) {
		xSemaphoreTake(lrfMeasureCommandHandle, portMAX_DELAY);
		gzyMeasureCommand();
		osDelay(500);
	}
  /* USER CODE END sendLRFMessage */
}

void parseGZYPacket(void const * argument)
{
  /* USER CODE BEGIN parseGZYPacket */
	xSemaphoreTake(lrfDataReceivedHandle, 0);
	for (;;) {
		xSemaphoreTake(lrfDataReceivedHandle, portMAX_DELAY);
		gzyDistanceStruct gzy = parseGZYMessage();
		xQueueSendToBack(lrfDataQueueHandle, &gzy, 0);

	}
  /* USER CODE END parseGZYPacket */
}




/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
