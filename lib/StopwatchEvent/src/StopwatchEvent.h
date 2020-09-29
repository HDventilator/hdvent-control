/*
StopwatchEvent 0.4 For Arduino by cygig
StopwatchEvent is based on TimedAction 1.6 by Alexander Brevig (alexanderbrevig@gmail.com).
It is updated to work with Arduino IDE 1.8.5.

StopwatchEvent provides an easy way to trigger functions every set time and is a non-blocking alternative
to delay() function.

*/

#ifndef STOPWATCHEVENT
#define STOPWATCHEVENT

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Stopwatch.h>


class StopwatchEvent {
  
  public:
    StopwatchEvent(Stopwatch& stopwatch);
  	void set(Stopwatch & stopwatch, unsigned long myPeriod, void (*myFunction)());
  	void reset();
  	void disable();
  	void enable();

  	void update();
	  void setInterval( unsigned long myPeriod );

  private:
    Stopwatch& _stopwatch;
    bool _active;
    void (*_function)();
    unsigned int _period;
		
};

#endif
