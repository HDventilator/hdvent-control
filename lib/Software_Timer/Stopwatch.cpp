//
// Created by david on 18.04.20.
//

#include "Stopwatch.h"

Stopwatch::Stopwatch(){
    _startTime = 0;
    isRunning = false;
    _elapsedTime = 0;
}

unsigned long Stopwatch::stop(){
    _elapsedTime = millis()-_startTime;
    isRunning = false;
    return _elapsedTime;
}

void Stopwatch::start() {
    _startTime = millis();
    isRunning = true;
}

unsigned long Stopwatch::getElapsedTime() {
    if (isRunning){
        _elapsedTime = millis()-_startTime;
    }
    else {
        _elapsedTime =0;
    }
    return(_elapsedTime);
}

void Stopwatch::reset() {
    _elapsedTime=0;
    isRunning=false;

}
