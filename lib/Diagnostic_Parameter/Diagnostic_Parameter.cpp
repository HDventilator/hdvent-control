//
// Created by david on 17.04.20.
//

#include "Diagnostic_Parameter.h"
Diagnostic_Parameter::Diagnostic_Parameter(float initialValue, float maxAlarm, float minAlarm){
    _maxAlarm = maxAlarm;
    _minAlarm = minAlarm;
    _value = initialValue;
}

float Diagnostic_Parameter::getMaxAlarm() const {
    return _maxAlarm;
}

float Diagnostic_Parameter::getMinAlarm() const {
    return _minAlarm;
}

void Diagnostic_Parameter::setMaxAlarm(float maxAlarm) {
    _maxAlarm = maxAlarm;
}

void Diagnostic_Parameter::setMinAlarm(float minAlarm) {
    _minAlarm = minAlarm;
}

float Diagnostic_Parameter::getValue() const {
    return _value;
}

Diagnostic_Parameter::Alarm Diagnostic_Parameter::checkAlarm(){
    if (_value > _maxAlarm){
        return TOO_HIGH;
    }
    else if (_value < _minAlarm){
        return TOO_LOW;
    }
    else {
        return OK;
    }
}
