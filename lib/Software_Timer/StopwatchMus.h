//
// Created by david on 05.12.20.
//

#ifndef HDVENT_CONTROL_STOPWATCHMUS_H
#define HDVENT_CONTROL_STOPWATCHMUS_H
#include <Stopwatch.h>

class StopwatchMus  {
public:
    StopwatchMus();
    void start();

    unsigned long stop();
    unsigned long getElapsedTime();
    void reset();
    void safeStart();
    bool isRunning;
private:
    unsigned long _startTime;
    unsigned long _elapsedTime;

};


#endif //HDVENT_CONTROL_STOPWATCHMUS_H
