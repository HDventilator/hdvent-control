//
// Created by david on 18.04.20.
//

#include "Stopwatch.h"

Stopwatch::Stopwatch(){
    _startTime = 0;
    _isRunning = false;
    _elapsedTime = 0;
}

unsigned long Stopwatch::stop(){
    getElapsedTime();
    _isRunning = false;
}

void Stopwatch::start() {
    _startTime = millis();
    _isRunning = true;
}

unsigned long Stopwatch::getElapsedTime() {
    if (_isRunning){
        _elapsedTime = millis();
    }
    else {
        _elapsedTime =0;
    }
    return(_elapsedTime);
}