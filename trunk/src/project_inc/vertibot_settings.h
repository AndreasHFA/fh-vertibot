/*
 * multicopter_settings.h
 *
 *  Created on: 04.09.2012
 *      Author: Flasch Franz
 */

#ifndef MULTICOPTER_SETTINGS_H_
#define MULTICOPTER_SETTINGS_H_

#include <stm32f30x.h>
#include <IMU_algorithm.h>


/*
 * Define driver-libraries here:
 */
//#define ITG3200
//#include <ITG3200_Lib.h>
//#define L3G4200D
//#include <L3G4200D_Lib.h>

//#define ADXL345
//#include <ADXL345_Lib.h>

//#define HMC5883L
//#include <HMC5883L_Lib.h>

//#define PID-Controller
#define PIDSmithPredictor-Controller
//#define ForceMotorValues


#define MAX_GAS 	200
#define MAX_ROLL 	35
#define MAX_PITCH 	35
#define ROLL_PITCH_TRSHLD 	3
#define MAX_YAW 	35

#define sensX 	0
#define sensY 	1
#define sensZ 	2

#define STDEV_THRESHOLD 	80

/* USEFUL FLAGS */
#define ARMED_FLAG		0x01 	/* FLAG set by the RemoteControl*/
#define IMU_READY_FLAG	0x02 	/* FLAG set by the IMU Gyro Test*/

#define COPTER_STATUS_ERROR 	-1
#define COPTER_STATUS_OK 		 0

typedef struct _sensorDir_
{
	uint8_t sensOrder[3];
	int8_t sensDirection[3];
}sensorDir_s;

typedef struct _Controller_settings_
{
	float PID_kp;
	float PID_ki;
	float PID_kd;
	float PID_kn;
}Controller_settings_s;

typedef struct _SmithPredictor_settings_
{
	float a1;		// T1a
	float a2;		// T2a
	float b1;		// T1b
	float b2;
	float b3;
	float gain;
}SmithPredictor_settings_s;

typedef struct _GlobalSettings_
{
	uint16_t gyroSensTreshold;
	uint16_t gyroLowpassValue;
	sensorDir_s gyroDir;

	uint8_t enableDriftCorrection;

	uint8_t enableACC;
	sensorDir_s accDir;
	float accMaxGOverflow;

	uint8_t enableMAG;
	sensorDir_s magDir;
	uint16_t magLowpassValue;

	uint16_t stopSpeed;
	uint16_t minSpeed;
	uint16_t maxSpeed;

	Controller_settings_s PID_rollPitch;
	Controller_settings_s PID_yaw;

	SmithPredictor_settings_s SmithPrediktor_rollPitch;

}GlobalSettings_s;

typedef struct _VertibotCtrlStates_
{
	//int16_t gas;
	//int16_t roll;
	//int16_t pitch;
	//int16_t yaw;

	float pitch;
	float yaw;

	/*
	 * Global Values for the Motor Control
	 *TODO: describe copterStatus
	 */
	uint8_t copterStatus;
	uint8_t watchDogTimVal;

}CtrlStates_s;

void SETTINGS_init(GlobalSettings_s *settings);
void CTRLSTATES_init(CtrlStates_s *states);


/* Global variable declarations */
GlobalSettings_s GlobalSettings;

CtrlStates_s CtrlStates;

struct XYZ_angles XYZ;

#endif /* MULTICOPTER_SETTINGS_H_ */
