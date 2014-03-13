#include "fuzzy.h"
// Not Function
float _not(float x)
{
	return (float)(1.0-x);
}
// Generates the values in the range [min,max] with increments
// defined by 'step'.
void generate(float minimum, float maximum, float st, float* values,int length)
{
	float x = minimum;
	int i;
	for(i=0;i<length;i++,x+=st)
		values[i] = x;
}
// Sets all elements in the array with the same value
void set(float* values,int length,float value)
{
	int i;
	for(i=0;i<length;i++)
		values[i] = value;
}
// Returns the smallest element of an array
float min_array(float* x, int length)
{
    if(length <= 0) return (float)0;
	float m = x[0];
	int i=1;
	for(;i<length;i++)
		m = min(m,x[i]);
	return m;
}
// Returns the largest element of an array
float max_array(float* x, int length)
{
    if(length <= 0) return (float)0;
	float m = x[0];
	int i=1;
	for(;i<length;i++)
		m = max(m,x[i]);
	return m;
}
// Centroid of points
void centroid(float* x, float* y, int length, float* Cx, float* Cy)
{
	float a = 0.0;
	float t = 0.0;
	int limit = length - 1;
	float xmin = x[limit];
	*Cx = 0;
	// *Cy = 0;

	// For all vertices except last
	int i = 0, j = 1;
	for ( ; i < limit; ++i)
	{
		j = (i + 1);

		if (y[i] > 0 && y[j] > 0)
		{
			if ((xmin > x[i]) || (i == 0)) xmin = x[i];
			t = (((x[i] - xmin) * y[j]) - ((x[j] - xmin) * y[i]));
			a += t;
			*Cx += (x[i] + x[j] - xmin - xmin) * t;
			// *Cy += (y[i] + y[j]) * t;
		}
	}

	// Do last vertex
	//t = (((x[0] - xmin) * y[i]) - ((x[i] - xmin) * y[0]));
	//a += t;
	//*Cx += (x[0] + x[i] - (2 * xmin)) * t;
	// *Cy += (y[0] + y[i]) * t;

	a *= 0.5;
	*Cx = (*Cx / (6.0 * a)) + xmin;
	// *Cy /= (6.0 * a);
}
// Trapezoidal Member Function
float trapmf(float x, float a, float b, float c, float d)
{
    return (float)max(min((x - a) / (b - a), min(1.0, (d - x) / (d - c))), 0);
}
// Triangular Member Function
float trimf(float x, float a, float b, float c)
{
    return (float)max(min((x - a) / (b - a), (c - x) / (c - b)), 0);
}
// Aggregation code for membership functions that require 3 setting values
void aggregate3(float* rules, int* ruleIndexes, int ruleIndexLength, float* xArr,
			    float *yArr, int resultLength,_mffloat3 mf,float a,float b,float c)
{
	int i=0, j=0;
	for(;i<resultLength;i++)
	{
		for(j=0;j<ruleIndexLength;j++)
		{yArr[i] = max(yArr[i],min(rules[ruleIndexes[j]],mf(xArr[i],a,b,c)));}
	}
}

// Aggregation code for membership functions that require 4 setting values
void aggregate4(float* rules, int* ruleIndexes, int ruleIndexLength, float* xArr, float *yArr, int resultLength,_mffloat4 mf,float a,float b,float c,float d)
{
	int i=0, j=0;
	for(;i<resultLength;i++)
	{
		for(j=0;j<ruleIndexLength;j++)
		{yArr[i] = max(yArr[i],min(rules[ruleIndexes[j]],mf(xArr[i],a,b,c,d)));}
	}
}

void fuzzy(float* inputs, float* outputs)
{	// Transformation of inputs to fuzzy inputs.
	// Input variable angular
	float Inputs1[3];
	// Input term 'N'for variable 'angular'
	Inputs1[0] = trapmf(inputs[0],-13,-10,-8,-0.5);
	// Input term 'Z'for variable 'angular'
	Inputs1[1] = trimf(inputs[0],-4,0,4);
	// Input term 'P'for variable 'angular'
	Inputs1[2] = trapmf(inputs[0],0.5,8,10,13);
	// Input variable angularvelocity
	float Inputs2[3];
	// Input term 'N'for variable 'angularvelocity'
	Inputs2[0] = trimf(inputs[1],-9,-5,-1);
	// Input term 'Z'for variable 'angularvelocity'
	Inputs2[1] = trimf(inputs[1],-4,0,4);
	// Input term 'P'for variable 'angularvelocity'
	Inputs2[2] = trimf(inputs[1],1,5,9);
	// Build all the conditions.
	// Variable to store the condition result
	float R[9];
	float CondR[2];
	// Handle conditions for rule 1
	CondR[0] = Inputs1[0];
	CondR[1] = Inputs2[0];
	R[0] = 1 * min_array(CondR,2);
	// Handle conditions for rule 2
	CondR[0] = Inputs1[1];
	CondR[1] = Inputs2[0];
	R[1] = 1 * min_array(CondR,2);
	// Handle conditions for rule 3
	CondR[0] = Inputs1[2];
	CondR[1] = Inputs2[0];
	R[2] = 1 * min_array(CondR,2);
	// Handle conditions for rule 4
	CondR[0] = Inputs1[0];
	CondR[1] = Inputs2[1];
	R[3] = 1 * min_array(CondR,2);
	// Handle conditions for rule 5
	CondR[0] = Inputs1[1];
	CondR[1] = Inputs2[1];
	R[4] = 1 * min_array(CondR,2);
	// Handle conditions for rule 6
	CondR[0] = Inputs1[2];
	CondR[1] = Inputs2[1];
	R[5] = 1 * min_array(CondR,2);
	// Handle conditions for rule 7
	CondR[0] = Inputs1[0];
	CondR[1] = Inputs2[2];
	R[6] = 1 * min_array(CondR,2);
	// Handle conditions for rule 8
	CondR[0] = Inputs1[1];
	CondR[1] = Inputs2[2];
	R[7] = 1 * min_array(CondR,2);
	// Handle conditions for rule 9
	CondR[0] = Inputs1[2];
	CondR[1] = Inputs2[2];
	R[8] = 1 * min_array(CondR,2);
	// Basic output processing
	int res = 100;
	float step;
	float xMin;
	float xMax;
	float x[res];
	float y[res];
	float t;
	int value[9];
	// Output variable PWM
	xMin = -100;
	xMax = 100;
	t=0;
	step = (xMax - xMin) / (float)(res - 1);
	generate(xMin, xMax, step, x, res);
	set(y, res, 0);
	value[0] = 5;
	value[1] = 7;
	value[2] = 8;
	aggregate4(R,value,3,x,y,res,trapmf,-120,-100,-80,-20);
	value[0] = 4;
	aggregate3(R,value,1,x,y,res,trimf,-40,0,40);
	value[0] = 0;
	value[1] = 1;
	value[2] = 3;
	aggregate4(R,value,3,x,y,res,trapmf,20,80,100,120);
	value[0] = 6;
	aggregate3(R,value,1,x,y,res,trimf,-80,-40,-5);
	value[0] = 2;
	aggregate3(R,value,1,x,y,res,trimf,5,40,80);
	centroid(x, y, res, &outputs[0], &t);
}
