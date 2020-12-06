//
// Created by david on 29.09.20.
//

#ifndef HDVENT_CONTROL_TIMEDTOGGLER_H
#define HDVENT_CONTROL_TIMEDTOGGLER_H

#include <Arduino.h>

class TimedToggler {

public:
    TimedToggler();
    TimedToggler(unsigned long myPeriod);
    void set(unsigned long myPeriod);
    void reset();
    void disable(bool state);
    void enable();
    void update();
    void setInterval( unsigned long myPeriod );

    bool getEvent();

    bool getState();

private:
    bool active;
    unsigned long timer;
    unsigned long period;
    void (*function)();
    bool _changed;
    bool _state;

};


#endif //HDVENT_CONTROL_TIMEDTOGGLER_H
