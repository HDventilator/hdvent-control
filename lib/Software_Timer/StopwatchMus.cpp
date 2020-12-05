//
// Created by david on 05.12.20.
//

#include "StopwatchMus.h"

unsigned long StopwatchMus::stop(){
    _elapsedTime = micros()-_startTime;
    isRunning = false;
    return _elapsedTime;
}

void StopwatchMus::start() {
    _startTime = micros();
    isRunning = true;
}

unsigned long StopwatchMus::getElapsedTime() {
    if (isRunning){
        _elapsedTime = micros()-_startTime;
    }
    else {
        _elapsedTime =0;
    }
    return(_elapsedTime);
}

StopwatchMus::StopwatchMus() {

        _startTime = 0;
        isRunning = false;
        _elapsedTime = 0;

};


void StopwatchMus::reset() {
    _elapsedTime=0;
    isRunning=false;

}

void StopwatchMus::safeStart() {
    if (!isRunning){
        start();
    }
}