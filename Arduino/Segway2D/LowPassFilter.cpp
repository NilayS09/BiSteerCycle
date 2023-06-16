#include "LowPassFilter.h"

unsigned long LowPassFilter::timestamp_prev {0};
double LowPassFilter::y_prev {0};

LowPassFilter::LowPassFilter(double Tf_val)
	:Tf{Tf_val} {
	}

double LowPassFilter::filter(double x){
	
	  unsigned long timestamp = micros();
	  float dt = (timestamp - timestamp_prev)*1e-6f;
	  if (dt < 0.0f || dt > 0.5f) dt = 1e-3f;
    //Serial.println(dt);
	  float alpha = Tf/(Tf + dt);
	  float y = alpha*y_prev + (1.0f - alpha)*x;
	  y_prev = y;
	  timestamp_prev = timestamp;
	  return y;	

}
