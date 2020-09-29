#include "StopwatchEvent.h"

StopwatchEvent::StopwatchEvent(Stopwatch& stopwatch): _stopwatch(stopwatch){
// Nothing inside constructor.
}

void StopwatchEvent::set(Stopwatch& stopwatch, unsigned long myPeriod, void (*myFunction)()){
    _active = true;
    _stopwatch = stopwatch;
    _stopwatch.safeStart();
	_period = myPeriod;
	_function = myFunction;
}



void StopwatchEvent::reset(){
    _stopwatch.start();
}

void StopwatchEvent::disable(){
    _active = false;
}

void StopwatchEvent::enable(){
    _active = true;
}

void StopwatchEvent::update(){
  if (_active && ((unsigned long) (millis() - _stopwatch.getElapsedTime()) >= _period) ) {
    _function();
    _stopwatch.start();
  }
}

void StopwatchEvent::setInterval(unsigned long myPeriod){
	_period = myPeriod;
}


