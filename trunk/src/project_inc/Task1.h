/*
 * RT_Task1.h
 *
 *  Created on: Jan 4, 2014
 *      Author: user
 */

#ifndef RT_TASK1_H_
#define RT_TASK1_H_

#include <stm32f30x.h>
#include <stm32f30x_conf.h>
#include <stm32f30x_it.h>
#include <stm32_configuration.h>

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

#define DEG_TO_RAD 0.017453292f

//! Max Gyro Error when calibrating
#define MAX_GYRO_ERROR 0.15f

//! dt-Time of Main IMU Task
#define DT 2 // original 2
#define DT_S ((float)DT/1000)

uint8_t accBuffer[6];

int xTemp;
int yTemp;
int zTemp;

/************ COM ***************/
int loopcnt;


// Static Funktionen können nur in der Datei in der Sie stehen aufgerufen werden...

void Init_IMU();
void IMU_Calculation();
void Init_PositionController();
void PositionController();


#endif /* RT_TASK1_H_ */
