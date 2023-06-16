#ifndef _LOW_PASS_FILTER_H_
#define _LOW_PASS_FILTER_H_

#ifdef ARDUINO && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
class LowPassFilter
{
	private:
		double Tf;
		static unsigned long timestamp_prev;
		static double y_prev;
	public:
		
		double filter( double signal);

		LowPassFilter(double Tf = 0.01);

};

#endif
