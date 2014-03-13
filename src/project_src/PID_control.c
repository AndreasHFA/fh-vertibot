#include <PID_control.h>

//float CalcVirtualPlant(SmitPredictor_Control *value, float Input);
//float CalcDelay(SmitPredictor_Control *value, float Input);


void init_PID(float sample_rate, float kp, float ki, float kd, float kn, PID_Control *values)
{
	values->sampling_rate = sample_rate;
	values->kp = kp;
	values->ki = ki;
	values->kd = kd;
	values->kn = kn;
}

void init_SmithPredictor(float a1, float a2, float b1, float b2, float b3, float gain, SmitPredictor_Control *values)
{
	values->a1 = a1;
	values->a2 = a2;
	values->b1 = b1;
	values->b2 = b2;
	values->b3 = b3;
	values->gain = gain;
}

void resetIntegralValues_PID(PID_Control *values)
{
	values->ealt = 0;
	values->Isum = 0;
	values->Dsum = 0;
	values->y = 0;
}

/* Originaler Regler */
// float P = (values->kp*values->e);
// float I = (values->esum*(values->ki*values->sampling_rate));
// float D = (values->kd*values->e -values->Dsum) * values->kn;
// values->Dsum += D * values->sampling_rate;

// D-Anteil:
// values->ealt = values->e;


// Error
//values->e = istwert - sollwert;
//values->e = sollwert - istwert;

// I-Anteil aufsummieren:
//values->Isum += (values->e) * values->sampling_rate;

// Anti-Windup
//if (values->esum > xx) values->esum = xx;
//if (values->esum < xx) values->esum = xx;
//float P = (values->kp*values->e);
//float I = (values->Isum*(values->ki));
//float D = (values->kd*values->e - values->Dsum ) * values->kn;
//values->Dsum += D * values->sampling_rate;


void calc_PID(float istwert, float sollwert, PID_Control *PID_Values)
{
	float P ,I ,D;

#define Isum_max 50.0f

	/* */
	PID_Values->e = sollwert - istwert;

	//if(PID_Values->e < 1.0f && PID_Values->e > -1.0f){
	//	PID_Values->e = 0.0f;
	//}

	/* Nichtlinearität hinzufügen (x^3) */
	//PID_Values->e = PID_Values->e * PID_Values->e * PID_Values->e;


	// I-Anteil aufsummieren:
	PID_Values->Isum += (PID_Values->e*PID_Values->ki) * PID_Values->sampling_rate;

	// Anti-Windup
	if (PID_Values->Isum > Isum_max) PID_Values->Isum = Isum_max;
	if (PID_Values->Isum < -Isum_max) PID_Values->Isum = -Isum_max;

	P = (PID_Values->kp * PID_Values->e);
	//I = (PID_Values->Isum * PID_Values->ki);
	I = PID_Values->Isum;
	D = (PID_Values->kd * PID_Values->e - PID_Values->Dsum ) * PID_Values->kn;
	PID_Values->Dsum += D * PID_Values->sampling_rate;

	// PID-Regler
	PID_Values->y = P + I + D;
	
}

void calc_PID_SmithPredictor(float istwert, float sollwert, PID_Control *PID_Values, SmitPredictor_Control *Plant_Values)
{
	static float S, E;
	float P ,I ,D;

#define Isum_max 50.0f

	/* SmithPrädiktor */
	PID_Values->e = sollwert - ( S - E + istwert);

	// I-Anteil aufsummieren:
	PID_Values->Isum += (PID_Values->e*PID_Values->ki) * PID_Values->sampling_rate;

	// Anti-Windup
	if (PID_Values->Isum > Isum_max) PID_Values->Isum = Isum_max;
	if (PID_Values->Isum < -Isum_max) PID_Values->Isum = -Isum_max;

	P = (PID_Values->kp * PID_Values->e);
	//I = (PID_Values->Isum * PID_Values->ki);
	I = PID_Values->Isum;
	D = (PID_Values->kd * PID_Values->e - PID_Values->Dsum ) * PID_Values->kn;
	PID_Values->Dsum += D * PID_Values->sampling_rate;

	// PID-Regler
	PID_Values->y = P + I + D;

	/* Get Virtual Plant */
	S = CalcVirtualPlant(Plant_Values, PID_Values->y);

	/* Delay */
	E = CalcDelay(Plant_Values, S);
}

/* OLD */
/* Calc PID */
//	PID_Values->Isum = PID_Values->Isum + (PID_Values->e);
//if (values->esum > xx) values->esum = xx;
//if (values->esum < xx) values->esum = xx;

//	PID_Values->y = (PID_Values->kp*PID_Values->e) + (PID_Values->Isum*PID_Values->ki*PID_Values->sampling_rate) +
//			(PID_Values->kd*(PID_Values->e - PID_Values->ealt)/PID_Values->sampling_rate);
//	PID_Values->ealt = PID_Values->e;


/* Simulate Virtual Plant */
/* checked */
float CalcVirtualPlant(SmitPredictor_Control *value, float Input){

	/* Calculate first Section */
	value->vPlantyT1y[0] = value->b1 * value->vPlantyT1y[1] + value->a1 * value->vPlantyT1x[1];
	value->vPlantyT1x[1] = Input;
	value->vPlantyT1y[1] = value->vPlantyT1y[0];

	/* Calculate second Section */
	value->vPlantyT2y[0] = value->vPlantyT2y[1] + value->a2 * value->vPlantyT2x[1];
	value->vPlantyT2x[1] = value->vPlantyT1y[0];
	value->vPlantyT2y[1] = value->vPlantyT2y[0];

	return value->vPlantyT2y[0];
}

/* Simulate Virtual Plant */
/* Fehler */
float CalcVirtualPlant_xxx(SmitPredictor_Control *value, float Input){
	/* shift */
	value->w[2] = 	value->w[1];
	value->w[1] =	value->w[0];
	value->w[0] = 	(value->gain*Input) + (value->a1*value->w[1]) + (value->a2*value->w[2]);
	return 		(value->b1*value->w[0]) + (value->b2*value->w[1]) + (value->b3*value->w[2]);
}

/* Fehler */
float CalcDirektForm2(float istwert, float sollwert, SmitPredictor_Control *value){
	/* shift */
	float E = sollwert - istwert;

	value->w[2] = 	value->w[1];
	value->w[1] =	value->w[0];
	value->w[0] = 	(value->gain*E) + (-value->a1*value->w[1]) + (-value->a2*value->w[2]);
	return 		(value->b1*value->w[0]) + (value->b2*value->w[1]) + (value->b3*value->w[2]);
}

/* FIFO Buffer */
/* Checked */
float CalcDelay(SmitPredictor_Control *value, float Input){

	static unsigned char dlyindex = 0;

	dlyindex++;
	if ( dlyindex >= (sizeof(value->fifo)/sizeof(float))) dlyindex = 0;
	value->fifo[dlyindex] = Input;
	if ( dlyindex < ( sizeof(value->fifo)/sizeof(float)-1 ) )
		return value->fifo[(dlyindex + 1)];
	else
		return value->fifo[0];
}


/*
 * The yaw PID calculation needs a special handling as the input value has
 * an integrating character
 * */
void calc_PID_YAW(float istwert, float sollwert, PID_Control *values)
{
	values->e = sollwert - istwert;

	if(values->e >= 180) values->e -= 360;
	else if (values->e <= -180) values->e += 360;

	if(values->e > 50) values->e = 50;
	if(values->e < -50) values->e = -50;

	values->Isum = values->Isum + (values->e);

	if(values->Isum < Isum_max) values->Isum = Isum_max;
	if(values->Isum > Isum_max) values->Isum = Isum_max;

	//PID-Regler
	values->y = (values->kp*values->e) + (values->Isum*values->ki*values->sampling_rate) + (values->kd*(values->e - values->ealt)/values->sampling_rate);

	values->ealt = values->e;
}


