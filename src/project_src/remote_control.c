/*
 * remote_control.c
 *
 *  Created on: Apr 28, 2013
 *      Author: franz
 */

#include <filter_Lib.h>
#include <remote_control.h>
#include <vertibot_settings.h>

void CalcRXValues(unsigned char *msgPointer, int16_t adjValue)
{
	static intFilter gasFiltered = {0,0};
	static intFilter rollFiltered = {0,0};
	static intFilter pitchFiltered = {0,0};
	static intFilter yawFiltered = {0,0};
	static int16_t currentYaw = 0;

	/*Reset the Watchdog Timer value*/
	CtrlStates.watchDogTimVal = 0;

	currentYaw = ((uint8_t)msgPointer[4]-adjValue);

	update_intFilter(&gasFiltered, 1, (((uint8_t)msgPointer[5]-adjValue)*7.5f)+60);
	update_intFilter(&pitchFiltered, 1, ((uint8_t)msgPointer[3]-adjValue)*2.0f);
	update_intFilter(&rollFiltered, 1, ((uint8_t)msgPointer[2]-adjValue)*2.0f);
	update_intFilter(&yawFiltered, 1, currentYaw*0.3f);

	//CtrlStates.gas = gasFiltered.filtered;
	//CtrlStates.roll = rollFiltered.filtered;
	CtrlStates.pitch = pitchFiltered.filtered;

	/* integrating the yaw value */
	if(CtrlStates.copterStatus & ARMED_FLAG)
	{
		CtrlStates.yaw -= yawFiltered.filtered;
	}
	else
	{
		CtrlStates.yaw = XYZ.z;
	}


	/* Narrow down all ctrl values */
	/*
	//if(CtrlStates.gas > MAX_GAS) CtrlStates.gas = MAX_GAS;
	//if(CtrlStates.gas < 0) CtrlStates.gas = 0;

	//if(CtrlStates.roll > -ROLL_PITCH_TRSHLD && CtrlStates.roll < ROLL_PITCH_TRSHLD) CtrlStates.roll = 0;
	//if(CtrlStates.roll > MAX_ROLL) CtrlStates.roll = MAX_ROLL;
	//if(CtrlStates.roll < -MAX_ROLL) CtrlStates.roll = -MAX_ROLL;

    */
	if(CtrlStates.pitch > -ROLL_PITCH_TRSHLD && CtrlStates.pitch < ROLL_PITCH_TRSHLD) CtrlStates.pitch = 0;
	if(CtrlStates.pitch > MAX_PITCH) CtrlStates.pitch = MAX_PITCH;
	if(CtrlStates.pitch < -MAX_PITCH) CtrlStates.pitch = -MAX_PITCH;

	/* The yaw value needs a special handling */
	if(CtrlStates.yaw > 180) CtrlStates.yaw -= 360;
	if(CtrlStates.yaw < -180) CtrlStates.yaw += 360;

#ifdef USART_REMOTE
	if(msgPointer[1] == 'K')
	{
		COPTER_STATUS |= READY_FLAG;
	}
	else
	{
		COPTER_STATUS &= ~(READY_FLAG);
	}
#else
	/*
	if(currentYaw < -20 && CtrlStates.gas < 5)
	{
		if(CtrlStates.copterStatus & IMU_READY_FLAG)
		{
			CtrlStates.copterStatus |= ARMED_FLAG;
		}
	}
	else if((currentYaw*1.5) > 20 && CtrlStates.gas < 5)
	{
		CtrlStates.copterStatus &= ~(ARMED_FLAG);
	}
	*/
#endif
}

