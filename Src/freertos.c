/* USER CODE BEGIN Header */
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "MotorDriver.h"								// Pilote du moteur
#include "lsm6ds3.h"										// Pilote de la centrale inertielle
#include "mesn_uart.h"									// Pilote de la liaison UART
#include "libSBR_autom_obs-corr.h"			// Algo observateur et correcteur
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
osMessageQDef(MsgBox,60,int32_t);
osMessageQId MsgBox;
osMessageQDef(MsgBox2,1,int32_t);
osMessageQId MsgBox2;
osMutexId mut_handler;
osMutexId mut_handlerGPIO;
typedef struct {
	 int32_t buffer[BUFFERSIZE];
	 uint32_t indexW;
	 uint32_t indexR;
	 uint32_t eltNb;
} circularBufferTypeDef;
 circularBufferTypeDef CircBuff;

/* USER CODE END Variables */
//osThreadId defaultTaskHandle;
osThreadId GestionEquilibreHandle;
osThreadId EnregistrementHandle;
osThreadId UARTHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void GestionEquilibreFn(void const * argument);
void EnregistrementFn(void const * argument);
void UARTFn(void const * argument);
/* USER CODE END FunctionPrototypes */

//void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
	MESN_UART_PutString_Poll((uint8_t*)"\r\nERROR : stack overflow from task");
	MESN_UART_PutString_Poll((uint8_t*)pcTaskName);
	while(1);
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
__weak void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
	MESN_UART_PutString_Poll((uint8_t*)"\r\nERROR : Heap full!");
	while(1);
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
	CircBuff.eltNb = 0;
	CircBuff.indexR = 0;
	CircBuff.indexW = 0;
    for(int i=0;i<BUFFERSIZE;i++){
    	CircBuff.buffer[i]=0;
    }
  /* USER CODE BEGIN Init */


  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
	osMutexDef(mut_handler);
	mut_handler = osMutexCreate(osMutex(mut_handler));
	osMutexDef(mut_handlerGPIO);
	mut_handlerGPIO = osMutexCreate(osMutex(mut_handlerGPIO));


  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  //osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  //defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
  //


  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
    /* Task 1 */
	osThreadDef(GestionEquilibre,GestionEquilibreFn,osPriorityHigh , 0,1024);
	GestionEquilibreHandle = osThreadCreate (osThread(GestionEquilibre), NULL);

		/* Task 2 */
//
	osThreadDef(Enregistrement,EnregistrementFn,osPriorityNormal , 0,512);
	EnregistrementHandle = osThreadCreate (osThread(Enregistrement), NULL);
		/* Task 3 */
	//
 osThreadDef(UART,UARTFn,osPriorityLow, 0,512);
 UARTHandle = osThreadCreate (osThread(UART), NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
     MsgBox = osMessageCreate(osMessageQ(MsgBox),NULL);
     MsgBox2 = osMessageCreate(osMessageQ(MsgBox2),NULL);
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
//void StartDefaultTask(void const * argument)
//{

  /* USER CODE BEGIN StartDefaultTask */
	/*uint8_t LSM6DS3_Res = 0;
	uint8_t tempString[100];*/

	/* Init des periphs externes */
	/*MotorDriver_Init();
	MESN_UART_Init();
	if(LSM6DS3_begin(&LSM6DS3_Res) != IMU_SUCCESS){
		MESN_UART_PutString_Poll((uint8_t*)"\r\nIMU Error !");
		while(1);
	}*/

	/* Test des periphs externes */
	/*sprintf((char*)tempString, "\r\nInit done. LSM6DS3 reg = %02x", LSM6DS3_Res);
	MESN_UART_PutString_Poll(tempString);
	MotorDriver_Move(200);*/

	/* Test algo autom */
	/*sprintf((char*)tempString, "\r\nAngle = %ldmDeg", autoAlgo_angleObs(50,5));//autoAlgo_angleObs:permet l'envoie de l'angle estimé
	MESN_UART_PutString_Poll(tempString);*/

  /* Infinite loop */
 /* for(;;)
  {
    MESN_UART_GetString(tempString,osWaitForever);//attendre jusqu'a ce que l'utilisateur entre une valeur
    MESN_UART_PutString_Poll(tempString);//afficher la valeur dans le console série
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  }*/

void GestionEquilibreFn(void const * argument)
{
	uint32_t PreviousWakeTime=osKernelSysTick();
	uint8_t LSM6DS3_Res = 0;
	int32_t LSM6DS3_acc = 0;
	int32_t LSM6DS3_vit = 0;
	int32_t angle=0;
	int32_t speed =0;
	uint8_t tempString[100];

	/* Init des periphs externes */
	MotorDriver_Init();
    MESN_UART_Init();
    if(LSM6DS3_begin(&LSM6DS3_Res) != IMU_SUCCESS){
    		MESN_UART_PutString_Poll((uint8_t*)"\r\nIMU Error !");
    		while(1);
    	}

    PreviousWakeTime=osKernelSysTick();
	//uint8_t angle[100];
  /* Infinite loop */
	while(1)
	  {

		//LSM6DS3_readMgAccelX(&LSM6DS3_acc);//mesure l'acc sur l'axe X
		//LSM6DS3_readMdpsGyroY(&LSM6DS3_vit);//mesure la vitesse sur l'axe y
		if(LSM6DS3_readMgAccelX(&LSM6DS3_acc) != IMU_SUCCESS || LSM6DS3_readMdpsGyroY(&LSM6DS3_vit) != IMU_SUCCESS){//
			MESN_UART_PutString_Poll((uint8_t*)"\r\nIMU Error !");
			while(1);
		}

		//sprintf((char*)tempString, "\r\nInit done. LSM6DS3 reg = %02ld", LSM6DS3_acc);
		//MESN_UART_PutString_Poll(tempString);
		angle = autoAlgo_angleObs(LSM6DS3_acc,LSM6DS3_vit);//calcul angle
		//sprintf((char*)tempString,"\r\n %ld angle", angle);
		//MESN_UART_PutString_Poll(tempString);
		//MESN_UART_PutString_Poll((uint8_t*)"\r\ntask1- MsgSent");
		osMessagePut(MsgBox,angle,0);//envoie de l'angle par le biais d'un queue de msg
		/*sprintf((char*)tempString,"\r\nAngle = %ldmDeg", autoAlgo_angleObs(LSM6DS3_acc,LSM6DS3_vit));//autoAlgo_angleObs:permet l'envoie de l'angle estimé
	    MESN_UART_PutString_Poll(tempString);*/
		speed = autoAlgo_commandLaw(angle,LSM6DS3_vit);//commande moteur
	    MotorDriver_Move(speed);//controle moteur
	    osDelayUntil(&PreviousWakeTime,10);
	  }

}
void EnregistrementFn(void const * argument)
{
	osEvent evt;
	int32_t mess;
	int compt=0;

  /* Infinite loop */
	while(1)
	  {
		evt = osMessageGet(MsgBox,osWaitForever);//wait for msg

		if(evt.status == osEventMessage){
			mess=evt.value.v;
			osMutexWait(mut_handler,osWaitForever);//attendre le mutex
			if (CircBuff.indexW < BUFFERSIZE) {
				CircBuff.buffer[CircBuff.indexW] = mess;
			    CircBuff.indexW++;
				if (CircBuff.indexW >= BUFFERSIZE)
					CircBuff.indexW = 0;
			}
			osMutexRelease(mut_handler);//liberer le mutex
			//MESN_UART_PutString_Poll((uint8_t*)"\r\n -> task2 - MsgRcved");
			osMessagePut(MsgBox2,mess,0);//envoie de la dernier angle enregistrer par le biais d'un queue de msg
			compt++;
			if(mess > 25){
			   osMutexWait(mut_handlerGPIO,osWaitForever);//attendre le mutex
			   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			   osMutexRelease(mut_handlerGPIO);//liberer le mutex
			}
			else{
				if(compt>0 && compt<10){
					osMutexWait(mut_handlerGPIO,osWaitForever);//attendre le mutex
				    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
				    osMutexRelease(mut_handlerGPIO);//liberer le mutex
				}
				else{
					osMutexWait(mut_handlerGPIO,osWaitForever);//attendre le mutex
				    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
				    osMutexRelease(mut_handlerGPIO);//liberer le mutex
					if(compt==100) compt=0;
				}
			}
		}
		else{
			MESN_UART_PutString_Poll((uint8_t*)"\r\n -> task2 - timeout");
		}



 }
}
void UARTFn(void const * argument)
{
	char tempString[10],tempString2[10];
	osEvent evt2;
	uint32_t mess2;
	uint8_t val[2];
	int i=0;
	int32_t tab[100];
  /* Infinite loop */
	while(1)
	  {
		 MESN_UART_GetString((uint8_t*)tempString,osWaitForever);//attendre jusqu'a ce que l'utilisateur entre une valeur
		 MESN_UART_PutString_Poll((uint8_t*)"\r\n SelfBalancingRobot:~# ");
		 MESN_UART_PutString_Poll((uint8_t*)tempString);//afficher la valeur dans le console série

		 if(strcmp (tempString,"help") == 0){
			 MESN_UART_PutString_Poll((uint8_t*)"\r\n read                   return last measured angle value ");
			 MESN_UART_PutString_Poll((uint8_t*)"\r\n dump                   return last hundred angle values ");
			 MESN_UART_PutString_Poll((uint8_t*)"\r\n stream                 continiously return last measured angle value and update display press ENTER to quit stream mode ");
			 MESN_UART_PutString_Poll((uint8_t*)"\r\n help                   print this menu");
		 }

		 else if(strcmp (tempString,"read")== 0){
			 osMutexWait(mut_handler,osWaitForever);//attendre le mutex
			 sprintf(tempString,"\r\n %ld mdeg",CircBuff.buffer[CircBuff.indexW-1]);
			 osMutexRelease(mut_handler);//liberer le mutex
			 MESN_UART_PutString_Poll((uint8_t*)tempString);
		 }

		 else if(strcmp (tempString,"stream")== 0){
			 MESN_UART_GetString(val,10);
			 while(val[0]!=0x71){
			 evt2 = osMessageGet(MsgBox2,osWaitForever);//wait for msg
			 if(evt2.status == osEventMessage){
			 	mess2=evt2.value.v;
			 	sprintf(tempString2,"\r\n %ld  mdeg",mess2);
			 	MESN_UART_PutString_Poll((uint8_t*)tempString2);
			 	MESN_UART_GetString(val,10);
		    }
		}
	}
		 else if(strcmp(tempString,"dump")== 0){
			 while(i<100){
				 //while(CircBuff.indexW == 0 );
				    osMutexWait(mut_handler,osWaitForever);//attendre le mutex
					tab[i] = CircBuff.buffer[CircBuff.indexR];
					i++;
					CircBuff.indexR++;
					if (CircBuff.indexR >=BUFFERSIZE)
						CircBuff.indexR = 0;
				}

			 osMutexRelease(mut_handler);//liberer le mutex
			 i=0;
			for(int j=0;j<100;j++){
				sprintf(tempString2,"\r\n 0 %d : %ld  mdeg",j,tab[j]);
				MESN_UART_PutString_Poll((uint8_t*)tempString2);
			}

		 }
		 else{
			 MESN_UART_PutString_Poll((uint8_t*)"\r\n commande n'existe pas:~# ");
		 }
	  }
}


  /* USER CODE END StartDefaultTask */
//}




/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
