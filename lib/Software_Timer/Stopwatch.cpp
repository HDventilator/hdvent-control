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
    getElapsedTime();
    isRunning = false;
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