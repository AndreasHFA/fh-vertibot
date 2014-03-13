#include <vertibot_settings.h>
#include <stm32_configuration.h>
#include <string.h>


void SETTINGS_init(GlobalSettings_s *settings)
{
	settings->gyroSensTreshold = 20;
	settings->gyroDir.sensDirection[sensX] = -1;
	settings->gyroDir.sensDirection[sensY] = -1;
	settings->gyroDir.sensDirection[sensZ] = 1;
	settings->gyroDir.sensOrder[sensX] = 0;
	settings->gyroDir.sensOrder[sensY] = 2;
	settings->gyroDir.sensOrder[sensZ] = 1;

	settings->enableDriftCorrection = 1;

	settings->enableACC = 1;
	settings->accDir.sensDirection[sensX] = -1;
	settings->accDir.sensDirection[sensY] = -1;
	settings->accDir.sensDirection[sensZ] = 1;
	settings->accDir.sensOrder[sensX] = 2;
	settings->accDir.sensOrder[sensY] = 0;
	settings->accDir.sensOrder[sensZ] = 1;
	settings->accMaxGOverflow = 1.3;

	settings->enableMAG = 0;
	settings->magLowpassValue = 5;//20;
	settings->magDir.sensDirection[sensX] = 1;
	settings->magDir.sensDirection[sensY] = -1;
	settings->magDir.sensDirection[sensZ] = 1;
	settings->magDir.sensOrder[sensX] = 0;
	settings->magDir.sensOrder[sensY] = 1;
	settings->magDir.sensOrder[sensZ] = 2;

	settings->stopSpeed = 100;	//*PRESCALEVALUE; 	//PRESCALEVALUE=6
	settings->minSpeed = 0; 	//*PRESCALEVALUE;
	settings->maxSpeed = 200;	//*PRESCALEVALUE;

	settings->PID_yaw.PID_kp = 2.0f;
	settings->PID_yaw.PID_ki = 1.0f;
	settings->PID_yaw.PID_kd = 0.0f;

/* PID Only */
#ifdef PIDController										// Matlab rob 0.6			//
	settings->PID_rollPitch.PID_kp = 19.847615688639f;		// 23.8526967116026  		// 22.7f;
	settings->PID_rollPitch.PID_ki = 52.4488131557352f;		//    	// 0.8f;
	settings->PID_rollPitch.PID_kd = 1.2881940564086f; 	//		// 0.40f;
	settings->PID_rollPitch.PID_kn = 108.348596776164f;
#endif


#ifdef PIDSmithPredictorController

	/* PID */
	settings->PID_rollPitch.PID_kp = 18.115707268541f; //14.5478771127435f;    	//22.7f;
	settings->PID_rollPitch.PID_ki = 3.8405091086306f; //28.1211353951591f;  	//0.8f;
	settings->PID_rollPitch.PID_kd = 0.4392517058174756f; //1.84575662912126f; 	//0.40f;
	settings->PID_rollPitch.PID_kn = 80.8259704631948f; //683.74924793257f;

	/* Smith Predictor */
	settings->SmithPrediktor_rollPitch.b1 = 0.9753f;	// T1b
    settings->SmithPrediktor_rollPitch.b2 = 0.0f;
	settings->SmithPrediktor_rollPitch.b3 = 0.0f;
	settings->SmithPrediktor_rollPitch.a1 = 0.02469f;	// T1a
	settings->SmithPrediktor_rollPitch.a2 = 0.007f;		// T2a
	settings->SmithPrediktor_rollPitch.gain = 0.0f;


	/* Test Controller */
	/*

  Columns 1 through 5

   1.000000000000000   0.024690087971871  -0.975309912028128   1.000000000000000   0.995023590034886

  Column 6

   0.003294688806972


G =

  38.285739551944431
	*/
	/*
	 	 SOS =
         	 0    1.0000    0.9917    1.0000   -1.9753    0.9753
		 G =
   	   	   	 8.6775e-05
	 */

	/* Info zur Matlabausgabe
    SOS is an L by 6 matrix with the following structure:
        SOS = [ b01 b11 b21  1 a11 a21
                b02 b12 b22  1 a12 a22
                ...
                b0L b1L b2L  1 a1L a2L ]

    Each row of the SOS matrix describes a 2nd order transfer function:
                  b0k +  b1k z^-1 +  b2k  z^-2
        Hk(z) =  ----------------------------
                   1 +  a1k z^-1 +  a2k  z^-2
    where k is the row index.
	 */


#endif

}

void CTRLSTATES_init(CtrlStates_s *states)
{
	memset(states, 0, sizeof(CtrlStates_s));
}

