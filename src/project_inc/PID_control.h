#ifndef PID_CONTROL_H
#define PID_CONTROL_H


typedef struct{

	//Variables for the PID-Control
	float sampling_rate;	/*sample rate in HZ*/
	float y;
	float kp;
	float ki;
	float kd;
	float kn;
	float e;
	float Isum;
	float Dsum;
	float ealt;
}PID_Control;

#define max_delays 8

typedef struct{
	float w[3];
	float vPlantyT1x[2];
	float vPlantyT1y[2];
	float vPlantyT2x[2];
	float vPlantyT2y[2];
	float a1;
	float a2;
	float b1;
	float b2;
	float b3;
	float gain;
	float fifo[max_delays];
}SmitPredictor_Control;


/*function prototypes*/

float CalcVirtualPlant(SmitPredictor_Control *value, float Input);
float CalcDelay(SmitPredictor_Control *value, float Input);
float CalcDirektForm2(float istwert, float sollwert, SmitPredictor_Control *value);

void init_PID(float sample_rate, float kp, float ki, float kd, float kn, PID_Control *values);
void init_SmithPredictor(float a1, float a2, float b1, float b2, float b3, float gain, SmitPredictor_Control *values);

void calc_PID_SmithPredictor(float istwert, float sollwert, PID_Control *PID_Values, SmitPredictor_Control *Plant_Values);
void resetIntegralValues_PID(PID_Control *values);
void calc_PID(float istwert, float sollwert, PID_Control *values);
void calc_PID_YAW(float istwert, float sollwert, PID_Control *values);

#endif /* PID_CONTROL_H */
