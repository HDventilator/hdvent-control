//
// Created by david on 18.04.20.
//

#ifndef HDVENT_CONTROL_STOPWATCH_H
#define HDVENT_CONTROL_STOPWATCH_H

#include <Arduino.h>

class Stopwatch {
public:
    Stopwatch();
    void start();
    unsigned long stop();
    unsigned long getElapsedTime();

private:
    unsigned long _startTime;
    unsigned long _elapsedTime;
    bool _isRunning;
};


#endif //HDVENT_CONTROL_STOPWATCH_H
