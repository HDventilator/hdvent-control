//
// Created by david on 29.09.20.
//

#include "TimedToggler.h"


TimedToggler::TimedToggler() {
// Nothing inside constructor.
}

void TimedToggler::set(unsigned long myPeriod) {
    active = true;
    timer = 0;
    period = myPeriod;
}


void TimedToggler::reset(){
    timer = millis();
}

void TimedToggler::disable(){
    active = false;
}

void TimedToggler::enable(){
    active = true;
}

void TimedToggler::update(){
    if ( active && ((unsigned long) (millis()-timer) >= period) ) {
        _changed = true;
        _state = !_state;
        timer = millis();
    }
}

void TimedToggler::setInterval( unsigned long myPeriod){
    period = myPeriod;
}

bool TimedToggler::getEvent() {
    update();
    if (_changed){
        _changed = false;
        return true;
    }
    else{
        return false;
    }
}

bool TimedToggler::getState() {
    update();
    return _state;
}
