#include <helperFunctions.h>


void Delay(volatile uint32_t delayCount)
{
	while (delayCount > 0)
	{
		delayCount--;
	}
}
//max( a, b ) ( ((a) > (b)) ? (a) : (b) )
float min(float a, float b)
{
	if( a < b ) return a;
	else return b;
}

float max(float a, float b)
{
	if( a > b ) return a;
	else return b;
}
