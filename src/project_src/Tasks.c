/*
 *
 */

#include <Tasks.h>

uint16_t i = 0;

struct IMU_values IMU;

/************ GYRO **************/
/*TODO Put GyroValues together */
struct gyroValues gyroOffset = {0,0,0};
struct gyroValues gyroXYZ = {0,0,0};

floatFilter xGyroFiltered_float;
floatFilter yGyroFiltered_float;
floatFilter zGyroFiltered_float;


/************ ACC ***************/
struct accValues accXYZ = {0,0,0};
uint8_t accCount = 0;

floatFilter xAccFiltered_float;
floatFilter yAccFiltered_float;
floatFilter zAccFiltered_float;


/************ MAG ***************/
uint8_t magCount = 0;
struct magValues magXYZ = {0,0,0};
struct EULER_angles eulerTemp;
intFilter xMagFiltered = {0,0};
intFilter yMagFiltered = {0,0};

/************ Position Controller ***********/
uint16_t MOT0 = 0;
uint16_t MOT1 = 0;

PID_Control PID_Position;
SmitPredictor_Control Smith_Position;


void Init_PositionController(){

	init_PID(DT_S, 	GlobalSettings.PID_rollPitch.PID_kp,
					GlobalSettings.PID_rollPitch.PID_ki,
					GlobalSettings.PID_rollPitch.PID_kd,
					GlobalSettings.PID_rollPitch.PID_kn, &PID_Position);


	init_SmithPredictor(GlobalSettings.SmithPrediktor_rollPitch.a1, GlobalSettings.SmithPrediktor_rollPitch.a2,
						GlobalSettings.SmithPrediktor_rollPitch.b1, GlobalSettings.SmithPrediktor_rollPitch.b2,
						GlobalSettings.SmithPrediktor_rollPitch.b3, GlobalSettings.SmithPrediktor_rollPitch.gain,
						&Smith_Position);

	CtrlStates.pitch = 0.0f;

}

void Init_IMU(){


	memset(&xGyroFiltered_float, 0, sizeof(xGyroFiltered_float));
	memset(&yGyroFiltered_float, 0, sizeof(yGyroFiltered_float));
	memset(&zGyroFiltered_float, 0, sizeof(zGyroFiltered_float));


	memset(&xAccFiltered_float, 0, sizeof(xAccFiltered_float));
	memset(&yAccFiltered_float, 0, sizeof(yAccFiltered_float));
	memset(&zAccFiltered_float, 0, sizeof(zAccFiltered_float));


	/*Initialize the DCM-variables*/
	IMU_init(&IMU, DT_S);

	if(GlobalSettings.enableDriftCorrection)
	{
		IMU_init_drift_correction(&IMU, 0.00040f);  //0.00020f/*kp-value*/);
	}

	while(1)
	{
		Gyro_calibrate(&gyroOffset);
		for (i = 0;i<5000;i++)
		{
			Gyro_readValues(&gyroXYZ);
			gyroXYZ.gyroValueX = ((gyroXYZ.gyroValueX-gyroOffset.gyroValueX));
			gyroXYZ.gyroValueY = ((gyroXYZ.gyroValueY-gyroOffset.gyroValueY));
			gyroXYZ.gyroValueZ = ((gyroXYZ.gyroValueZ-gyroOffset.gyroValueZ));

			IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensX]] += ((float)gyroXYZ.gyroValueX*GYROCONVERT)*DT_S;
			IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensY]] += ((float)gyroXYZ.gyroValueX*GYROCONVERT)*DT_S;
			IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensZ]] += ((float)gyroXYZ.gyroValueX*GYROCONVERT)*DT_S;
		}
		//vTaskDelayUntil( &xNextWakeTime, DT );
		GPIOC->ODR ^= GPIO_Pin_11;
		vTaskDelay( 1000 );

		if((IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensX]] > -MAX_GYRO_ERROR) && (IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensX]] < MAX_GYRO_ERROR) &&
		   (IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensY]] > -MAX_GYRO_ERROR) && (IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensY]] < MAX_GYRO_ERROR) &&
		   (IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensZ]] > -MAX_GYRO_ERROR) && (IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensZ]] < MAX_GYRO_ERROR))
		{
			IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensX]] = 0;
			IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensY]] = 0;
			IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensZ]] = 0;
			break;
		}
		IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensX]] = 0;
		IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensY]] = 0;
		IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensZ]] = 0;
		printf("Error Gyro Test!\n\r");
	}

	if(GlobalSettings.enableACC)
	{
		//ADXL345_calibrate(GlobalSettings.accMaxGOverflow, &accXYZ);
		LSM303DLHC_ACC_calibrate(GlobalSettings.accMaxGOverflow, &accXYZ);
	}

	if(GlobalSettings.enableMAG)
	{
		Mag_calibrate();
		vTaskDelay( 200 );
		Mag_readValues(&magXYZ);
		eulerTemp.yaw = atan2((float)(-magXYZ.magValueX),(float)(magXYZ.magValueY))-1.5707;
		CtrlStates.yaw = -eulerTemp.yaw*(180/M_PI);
		/* The yaw value needs a special handling */
		if(CtrlStates.yaw > 180) CtrlStates.yaw -= 360;
		if(CtrlStates.yaw < -180) CtrlStates.yaw += 360;
		IMU_Euler_to_DCM(&IMU, &eulerTemp);
	}

	/* Set the START Flag! */
	CtrlStates.copterStatus |= IMU_READY_FLAG;

	//xNextWakeTime = xTaskGetTickCount();

	/* Set the DRIVE Flag! */
	CtrlStates.copterStatus |= ARMED_FLAG;

}


void IMU_Calculation()
{
	/*!
	 *  IMU - Calculation Process
	 */
		Gyro_readValues(&gyroXYZ);
		gyroXYZ.gyroValueX = ((gyroXYZ.gyroValueX-gyroOffset.gyroValueX));
		gyroXYZ.gyroValueY = ((gyroXYZ.gyroValueY-gyroOffset.gyroValueY));
		gyroXYZ.gyroValueZ = ((gyroXYZ.gyroValueZ-gyroOffset.gyroValueZ));

		/* Cut-off the lower values to reduce noise and fail-integration*/
		if(gyroXYZ.gyroValueX > -GlobalSettings.gyroSensTreshold && gyroXYZ.gyroValueX < GlobalSettings.gyroSensTreshold) gyroXYZ.gyroValueX = 0;
		if(gyroXYZ.gyroValueY > -GlobalSettings.gyroSensTreshold && gyroXYZ.gyroValueY < GlobalSettings.gyroSensTreshold) gyroXYZ.gyroValueY = 0;
		if(gyroXYZ.gyroValueZ > -GlobalSettings.gyroSensTreshold && gyroXYZ.gyroValueZ < GlobalSettings.gyroSensTreshold) gyroXYZ.gyroValueZ = 0;

		floatFilterGyro(&xGyroFiltered_float, gyroXYZ.gyroValueX);
		floatFilterGyro(&yGyroFiltered_float, gyroXYZ.gyroValueY);
		floatFilterGyro(&zGyroFiltered_float, gyroXYZ.gyroValueZ);

		IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensX]] = GlobalSettings.gyroDir.sensDirection[sensX]*(xGyroFiltered_float.currentValue*GYROCONVERT)*DEG_TO_RAD;
		IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensY]] = GlobalSettings.gyroDir.sensDirection[sensY]*(yGyroFiltered_float.currentValue*GYROCONVERT)*DEG_TO_RAD;
		IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensZ]] = GlobalSettings.gyroDir.sensDirection[sensZ]*(zGyroFiltered_float.currentValue*GYROCONVERT)*DEG_TO_RAD;

		/*Here the ACC-Compensation, we do this in every tenth round*/
		if(GlobalSettings.enableACC)
		{
			if(accCount >= 5)
			{
				//ADXL345_readValues(&accXYZ);
				//LSM303DLHC_ACC_readValues(&accXYZ);
				LSM303DLHC_ACC_readValuesDMA(&accXYZ, &accBuffer[0]);

				//ADC_AutoInjectedConvCmd(ADC2,ENABLE);
//				ADC_StartConversion(ADC2);
//				/* wait for ADRDY */
//				while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC));
//				ADC_StartInjectedConversion(ADC2);
//
//				accXYZ.accValueX = ADC_GetInjectedConversionValue(ADC2,ADC_InjectedSequence_2)-1700;
//				accXYZ.accValueZ = ADC_GetInjectedConversionValue(ADC2,ADC_InjectedSequence_1)-1700;
//				accXYZ.accValueY = ADC_GetInjectedConversionValue(ADC2,ADC_InjectedSequence_3)-1700;

				floatFilterAcc(&xAccFiltered_float, accXYZ.accValueX);
				floatFilterAcc(&yAccFiltered_float, accXYZ.accValueY);
				floatFilterAcc(&zAccFiltered_float, accXYZ.accValueZ);

				if((xAccFiltered_float.currentValue < accXYZ.accGValue) && (xAccFiltered_float.currentValue > -accXYZ.accGValue) &&
				   (yAccFiltered_float.currentValue < accXYZ.accGValue) && (yAccFiltered_float.currentValue > -accXYZ.accGValue) &&
				   (zAccFiltered_float.currentValue < accXYZ.accGValue) && (zAccFiltered_float.currentValue > -accXYZ.accGValue))
				{
					/*Direction setup is the same as above*/
					IMU.accel_vector[GlobalSettings.accDir.sensOrder[sensX]] = GlobalSettings.accDir.sensDirection[sensX]*xAccFiltered_float.currentValue;
					IMU.accel_vector[GlobalSettings.accDir.sensOrder[sensY]] = GlobalSettings.accDir.sensDirection[sensY]*yAccFiltered_float.currentValue;
					IMU.accel_vector[GlobalSettings.accDir.sensOrder[sensZ]] = GlobalSettings.accDir.sensDirection[sensZ]*zAccFiltered_float.currentValue;

					xTemp = (int)IMU.accel_vector[GlobalSettings.accDir.sensOrder[sensX]];
					yTemp = (int)IMU.accel_vector[GlobalSettings.accDir.sensOrder[sensY]];
					zTemp = (int)IMU.accel_vector[GlobalSettings.accDir.sensOrder[sensZ]];
				}
				else
				{
					IMU.kp_vector_ACC[0] = 0;
					IMU.kp_vector_ACC[1] = 0;
					IMU.kp_vector_ACC[2] = 0;

					xTemp = 0;
					yTemp = 0;
					zTemp = 0;
				}

				//printf("%d %d %d\n\r", (int)(IMU.accel_vector[0]*30),(int)(IMU.accel_vector[1]*30), (int)(IMU.accel_vector[2]*30));

				if(GlobalSettings.enableDriftCorrection)
				{
					IMU_drift_correction_ACC(&IMU);
				}
				accCount = 0;
			}
			accCount++;
		}

		/*Here the Mag-Compensation, we do this in every 45 round*/
		if(GlobalSettings.enableMAG)
		{
			if(magCount >= 45)
			{
				Mag_readValues(&magXYZ);
				update_intFilter(&xMagFiltered, GlobalSettings.magLowpassValue, (magXYZ.magValueX));
				update_intFilter(&yMagFiltered, GlobalSettings.magLowpassValue, (magXYZ.magValueY));

				/*Direction setup is the same as above*/
				IMU.mag_vector[GlobalSettings.magDir.sensOrder[sensX]] = GlobalSettings.magDir.sensDirection[sensX]*xMagFiltered.filtered;
				IMU.mag_vector[GlobalSettings.magDir.sensOrder[sensY]] = GlobalSettings.magDir.sensDirection[sensY]*yMagFiltered.filtered;
				//IMU.mag_vector[GlobalSettings.magDir.sensOrder[sensZ]] = GlobalSettings.magDir.sensDirection[sensZ]*zMagFiltered;
				IMU.mag_vector[GlobalSettings.magDir.sensOrder[sensZ]] = 0;

				if(GlobalSettings.enableDriftCorrection)
				{
					IMU_drift_correction_MAG(&IMU);
				}
				magCount = 0;
				//printf("%d,%d\n\r", (int)xMagFiltered.filtered, (int)yMagFiltered.filtered);
			}
			magCount++;
		}

		/*This functions here do the DCM-Algorithm stuff*/
		IMU_update(&IMU);
		IMU_ortho_adjust(&IMU);
		IMU_normalize(&IMU);
		//Euler_angles(&DCM);
		//IMU_DCM_to_Euler_deg(&IMU, &euler);
		IMU_DCM_to_XYZ(&IMU, &XYZ);


}

float InPut[4];
void PositionController()
{
	//** PID Calculation Test

	//if(CtrlStates.copterStatus & ARMED_FLAG )
	{
		float MiddelPos = 4.8f;
		float Setpoint;


		/* Get Controll Values */
		if(It_Com.rxready == 1)
		{
			It_Com.rxready = 0;
			memcpy(&CtrlStates.pitch, &It_Com.rxbuffer[4], 4);
			memcpy(&CtrlStates.yaw, &It_Com.rxbuffer[8], 4);
		}
		/* Calc Setpoint */
		Setpoint = MiddelPos + CtrlStates.pitch;

#ifdef  PID-Controller

		calc_PID(XYZ.y, 4.8f, &PID_Position); // Hier stellt er sich auf 6° ein

		MOT0 = GlobalSettings.stopSpeed - PID_Position.y;
		MOT1 = GlobalSettings.stopSpeed - PID_Position.y;

		if(MOT0 < GlobalSettings.minSpeed) MOT0 = GlobalSettings.minSpeed;
		if(MOT0 > GlobalSettings.maxSpeed) MOT0 = GlobalSettings.maxSpeed;

		if(MOT1 < GlobalSettings.minSpeed) MOT1 = GlobalSettings.minSpeed;
		if(MOT1 > GlobalSettings.maxSpeed) MOT1 = GlobalSettings.maxSpeed;

		// Disable Interrupts
		taskENTER_CRITICAL();

		yTemp = MOT0;

		PWM_Motor(0, MOT0);
		PWM_Motor(1, MOT1);

		taskEXIT_CRITICAL();
#endif

#ifdef PIDSmithPredictor-Controller

		//calc_PID_SmithPredictor(XYZ.y, 4.8f, &PID_Position, &Smith_Position);

		//calc_PID(XYZ.y, 4.8f, &PID_Position);



		/* Get Angle */
		InPut[0] = XYZ.y;
		/* Get Angle Acceleration */
		InPut[1] = xGyroFiltered_float.currentValue;

		float OutPut[1];

		//MyFirstFuzzy(&InPut, &OutPut);
		//fuzzy(&InPut, &OutPut);


		if  (PID_Position.y > -5.0f && PID_Position.y < 5.0f )
		//	NotLinear = 0.0f;


		MOT0 = GlobalSettings.stopSpeed - PID_Position.y;
		MOT1 = GlobalSettings.stopSpeed - PID_Position.y;

		if(MOT0 < GlobalSettings.minSpeed) MOT0 = GlobalSettings.minSpeed;
		if(MOT0 > GlobalSettings.maxSpeed) MOT0 = GlobalSettings.maxSpeed;

		if(MOT1 < GlobalSettings.minSpeed) MOT1 = GlobalSettings.minSpeed;
		if(MOT1 > GlobalSettings.maxSpeed) MOT1 = GlobalSettings.maxSpeed;


		if ( MOT0 < 100 ) MOT0 -= 10;
		if ( MOT0 > 100 ) MOT0 += 10;

		MOT1 = MOT0;

		// Disable Interrupts
		// taskENTER_CRITICAL();

		PWM_Motor(0, MOT0);
		PWM_Motor(1, MOT1);

		// taskEXIT_CRITICAL();
#endif

#ifdef ForceMotorValues

			if(It_Com.rxready == 1)
				{
					It_Com.rxready = 0;
					//SendBufferedDataFrame(&It_Com ,&It_Com.rxbuffer[0], &It_Com.rxbuffer[4], &It_Com.rxbuffer[8], &It_Com.rxbuffer[12], 1);
					//float fValue;
					memcpy(&CtrlStates.pitch, &It_Com.rxbuffer[4], 4);
					//CtrlStates.pitch = fValue;
					memcpy(&CtrlStates.yaw, &It_Com.rxbuffer[8], 4);
					//CtrlStates.yaw = fValue;
				}


			MOT0 = MOT1= GlobalSettings.stopSpeed + (int16_t) CtrlStates.pitch;

				PWM_Motor(0, (uint16_t)MOT0);
				PWM_Motor(1, (uint16_t)MOT1);

#endif

	}
//	else
	{
		resetIntegralValues_PID(&PID_Position);

		PWM_Motor(0, (uint16_t)GlobalSettings.stopSpeed);
		PWM_Motor(1, (uint16_t)GlobalSettings.stopSpeed);
	}



	uint32_t Header = 0xFF7F3F1F;

	static uint16_t foo = 0;
	foo++;

	uint32_t foobar = MOT0;

	// SendBufferedDataFrame(&It_Com ,&Header, &XYZ.y, &PID_Position.Isum, &foobar, 12);
	//SendBufferedDataFrame(&It_Com ,&Header, &XYZ.y, &PID_Position.e, &foobar, 12);
	SendBufferedDataFrame(&It_Com, &Header, &InPut[1], &InPut[0], &foobar, 1);



/* CalcVirtualPlant Function Checked */
/*
	float plantvalue = CalcVirtualPlant(&Smith_Position, 1.0f);
	float tmp = 1.0f;
	SendBufferedDataFrame(&It_Com ,&Header, &plantvalue, &tmp, &foo, 12);
*/

/* Delay Function Checked */
/*
	static float tmp;

	if (tmp >= 20) tmp = 0.0f;
	else tmp += 1.0f;

	float delayoutvalue;

	if (tmp > 10 && tmp <= 20 ) {
		delayoutvalue = CalcDelay(&Smith_Position, 0.0f);
		foo = 0;
	} else {
		delayoutvalue = CalcDelay(&Smith_Position, 1.0f);
		foo = 1;
	}
	SendBufferedDataFrame(&It_Com ,&Header, &delayoutvalue, &tmp, &foo, 12);
*/

	// Debug
	//if(It_Com.rxready == 1)
	//{
	//	It_Com.rxready = 0;
	//	SendBufferedDataFrame(&It_Com ,&It_Com.rxbuffer[0], &It_Com.rxbuffer[4], &It_Com.rxbuffer[8], &It_Com.rxbuffer[12], 1);
	//}

	//float delayoutvalue = CalcDelay(&Smith_Position, 1.0f);

	// **** End Test
}

