#include <stm32f30x.h>
#include <stm32f30x_conf.h>
#include <stm32f30x_it.h>
#include <stm32_configuration.h>

#include <Tasks.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <usart.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>
#include <croutine.h>

#include <IMU_algorithm.h>
#include <filter_Lib.h>
#include <vertibot_settings.h>
#include <xBee_Lib.h>
#include <L3GD20_Lib_SPI.h>
#include <ADXL345_Lib.h>
#include <LSM303DLHC_Lib_I2C.h>
#include <MotorPWM.h>
#include <PID_control.h>
//#include <fuzzy.h>

//#define DEG_TO_RAD 0.017453292f

/* Info Motorendstufen kalibrieren
 *
 */
//#define ESC_CALIBRATE		// Enstufen Kalibrieren
//#define DEBUG				// Motoren werden abgeschalten

#ifndef DEBUG
	#define enableX_AXIS
	#define enableY_AXIS
#endif





#define ACC_MEDIAN_VALUE 5

static void vLedCtrlCoRoutine( xCoRoutineHandle xHandle, unsigned portBASE_TYPE uxIndex );
static void vRemoteCtrlWatchdogCoRoutine( xCoRoutineHandle xHandle, unsigned portBASE_TYPE uxIndex );
static void IMU_Print_Values( void *pvParameters );
static void IMU_Task( void *pvParameters );
static void Control_Task( void *pvParameters );


int main()
{
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */

	RCC_Configuration();
	NVIC_Configuration();
	GPIO_Configuration();
	USART_Configuration();
	setup_xBeeS6();

	SPI_Configuration();
	I2C_Configuration();
	//TIM2_PWM_Configuration();
	TIM3_PWM_Configuration();
	TIM4_Configuration();
	ADC_Configuration();
	EXTI_Configuration();
	DMA_Configuration(&accBuffer[0]);
	//TIM4_PWM_INPUT_Configuration();

	SETTINGS_init(&GlobalSettings);
	CTRLSTATES_init(&CtrlStates);

	xCoRoutineCreate( vLedCtrlCoRoutine, 0, 0 );

	//xCoRoutineCreate( vRemoteCtrlWatchdogCoRoutine, 1, 0 );

	/* Motor Enstufe ausschalten */
	//GPIO_ResetBits(GPIOB, GPIO_Pin_1 | GPIO_Pin_2);
	Disable_Motor();


	/* Debug Fuzzy Controll */
/*
 	while(1){
		//GPIOC->ODR |= GPIO_Pin_11;	// Set Debug Pin


		GPIOC->ODR ^= GPIO_Pin_11;		// Toggle Pin
				//IMU_Calculation();

				PositionController();

	    //GPIOC->ODR &= ~GPIO_Pin_11;	// Clear Debug Pin

	}
*/

	xTaskCreate( IMU_Task, ( signed char * ) "IMU_Gyro", configMINIMAL_STACK_SIZE, NULL, 4, NULL );
	xTaskCreate( Control_Task, ( signed char * ) "Pos_Contr", configMINIMAL_STACK_SIZE, NULL, 3, NULL );
	//xTaskCreate( IMU_Print_Values, ( signed char * ) "Print_Euler", configMINIMAL_STACK_SIZE, NULL, 2, NULL );

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	while(1);
}

/*********************************
 *		IMU Task
 *********************************/
static void IMU_Task( void *pvParameters )
{
	portTickType xNextWakeTime;

	Disable_Motor();

	/*
	 * Wait here 2 Seconds to ensure,
	 * that all capacitors are charged
	 * and VCC is stable
	 * */
	vTaskDelay( 2000 );

	Init_IMU();

	Enable_Motor();

	xNextWakeTime = xTaskGetTickCount();

	while(1)
	{

		GPIOC->ODR |= GPIO_Pin_11;	// Set Debug Pin

		IMU_Calculation();

		//GPIOC->ODR ^= GPIO_Pin_11; // Toggle Debug Pin

		GPIOC->ODR &= ~GPIO_Pin_11; // Clear Debug Pin

		vTaskDelayUntil( &xNextWakeTime, DT );
	}
}

// GPIOC->ODR ^= GPIO_Pin_11; // Toggle Debug Pin


/*********************************
 *		Controller Task
 *********************************/
static void Control_Task( void *pvParameters )
{
	const portTickType xDelay = 10 / portTICK_RATE_MS;

	Init_PositionController();

	while(1)
	{
		//GPIOC->ODR |= GPIO_Pin_10;	// Set Debug Pin

		////Disable_Motor();
		PositionController();

		//GPIOC->ODR &= ~GPIO_Pin_10; // Clear Debug Pin


		vTaskDelay( xDelay );
	}
}


/*For debug purposes - the print task*/
static void IMU_Print_Values( void *pvParameters )
{
	const portTickType xDelay = 40 / portTICK_RATE_MS;

	while(1)
	{
		//printf("%d,%d,%d,%d %d  ",CtrlStates.gas, CtrlStates.nick, CtrlStates.roll, CtrlStates.yaw, CtrlStates.copterStatus);
		printf("%d,%d,%d\r\n",(int)XYZ.y, (int)XYZ.x, (int)XYZ.z);
		//printf("%d %d %d\n\r", xTemp, yTemp, zTemp);
		//printf("test:%d,%d,%d\r\n", (int)(XYZ.y), (int)(CtrlStates.roll) ,(int)(yTemp));

		// XYZ.y ?
		// XYZ.x roll
		// XYZ.z
		vTaskDelay( xDelay );
	}
}



void LED_TOGGLE_NOT_READY(void)
{
	GPIOE->ODR ^= GPIO_Pin_8;
	GPIOE->ODR ^= GPIO_Pin_10;
	GPIOE->ODR ^= GPIO_Pin_12;
	GPIOE->ODR ^= GPIO_Pin_14;
}

void LED_TOGGLE_READY(void)
{
	GPIOE->ODR ^= GPIO_Pin_9;
	GPIOE->ODR ^= GPIO_Pin_11;
	GPIOE->ODR ^= GPIO_Pin_13;
	GPIOE->ODR ^= GPIO_Pin_15;
}

void LED_TOGGLE_ALL(void)
{
	if(GPIOE->ODR & GPIO_Pin_9)
	{
		GPIOE->ODR &= ~GPIO_Pin_9;
		GPIOE->ODR &= ~GPIO_Pin_11;
		GPIOE->ODR &= ~GPIO_Pin_13;
		GPIOE->ODR &= ~GPIO_Pin_15;
		GPIOE->ODR |= GPIO_Pin_8;
		GPIOE->ODR |= GPIO_Pin_10;
		GPIOE->ODR |= GPIO_Pin_12;
		GPIOE->ODR |= GPIO_Pin_14;
	}
	else
	{
		GPIOE->ODR |= GPIO_Pin_9;
		GPIOE->ODR |= GPIO_Pin_11;
		GPIOE->ODR |= GPIO_Pin_13;
		GPIOE->ODR |= GPIO_Pin_15;
		GPIOE->ODR &= ~GPIO_Pin_8;
		GPIOE->ODR &= ~GPIO_Pin_10;
		GPIOE->ODR &= ~GPIO_Pin_12;
		GPIOE->ODR &= ~GPIO_Pin_14;
	}
}

void vLedCtrlCoRoutine( xCoRoutineHandle xHandle,
                         unsigned portBASE_TYPE uxIndex )
{
   // Co-routines must start with a call to crSTART().
   crSTART( xHandle );

   for( ;; )
   {
	   // Delay for a fixed period.
	   crDELAY( xHandle, 200 );

	   if(CtrlStates.copterStatus == IMU_READY_FLAG)
	   {
		   LED_TOGGLE_READY();
	   }
	   else if (CtrlStates.copterStatus & ARMED_FLAG)
	   {
		   LED_TOGGLE_ALL();
	   }
	   else
	   {
		   LED_TOGGLE_NOT_READY();
	   }
   }

   // Co-routines must end with a call to crEND().
   crEND();
}

void vRemoteCtrlWatchdogCoRoutine( xCoRoutineHandle xHandle,
                         unsigned portBASE_TYPE uxIndex )
{
   // Co-routines must start with a call to crSTART().

   crSTART( xHandle );

   for( ;; )
   {
	   // Delay for a fixed period.
	   crDELAY( xHandle, 1000 );
	   CtrlStates.watchDogTimVal++;
	   if (CtrlStates.watchDogTimVal >= 2)
	   {
		   CtrlStates.copterStatus &= ~ARMED_FLAG;
		   printf("Watchdog Error!!%d\n", 0);
	   }
	   GPIOC->ODR ^= GPIO_Pin_11; // Toggle Debug Pin
   }

   // Co-routines must end with a call to crEND().
   crEND();
}





void vApplicationMallocFailedHook( void )
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; )
	{
		GPIOC->ODR ^= GPIO_Pin_11; // Toggle Debug Pin
	}
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	for( ;; ){

		GPIOC->ODR ^= GPIO_Pin_11; // Toggle Debug Pin
	};
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{

	/* This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amout of FreeRTOS heap that
	remains unallocated. */
	vCoRoutineSchedule();
}


void vApplicationTickHook( void )
{


}

//if(CtrlStates.copterStatus & ARMED_FLAG )
//{
//	#ifdef ESC_CALIBRATE
//	MOT0 = MOT1 = GlobalSettings.stopSpeed + CtrlStates.gas;
//
//	PWM_Motor(0, (uint16_t)MOT0);
//	PWM_Motor(1, (uint16_t)MOT1);
//
//	#endif
//
//	//Mischen der Werte:
//
//	#ifdef VERTIBOT
//	//PID-Regler
//	calc_PID(-XYZ.y, -5.5, &PID_X1);						// Hier stellt er sich auf 6° ein
//
//	MOT0 = GlobalSettings.stopSpeed - PID_X1.y;
//	MOT1 = GlobalSettings.stopSpeed - PID_X1.y;
//
//	if(MOT0 < GlobalSettings.minSpeed) MOT0 = GlobalSettings.minSpeed;
//	if(MOT0 > GlobalSettings.maxSpeed) MOT0 = GlobalSettings.maxSpeed;
//
//	if(MOT1 < GlobalSettings.minSpeed) MOT1 = GlobalSettings.minSpeed;
//	if(MOT1 > GlobalSettings.maxSpeed) MOT1 = GlobalSettings.maxSpeed;
//
//	// Disable Interrupts
//	taskENTER_CRITICAL();
//
//	yTemp = MOT0;
//
//	#ifdef VERTIBOT
//	  PWM_Motor(0, (uint16_t)MOT0);
//	  PWM_Motor(1, (uint16_t)MOT1);
//	#endif
//
//	taskEXIT_CRITICAL();
//
//	#endif
//}
//else
//{
//	resetIntegralValues_PID(&PID_X1);
//
//	PWM_Motor(0, (uint16_t)GlobalSettings.stopSpeed);
//	PWM_Motor(1, (uint16_t)GlobalSettings.stopSpeed);
//}
//
////
//uint32_t Header = 0xFF7F3F1F;
//static uint32_t foo = 0;
//foo++;
//SendBufferedDataFrame(&It_Com ,&Header, &XYZ.y, &PID_X1.y, &PID_X1.esum, 4);
