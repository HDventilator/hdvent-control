//
// Created by david on 17.04.20.
//

#ifndef HDVENT_CONTROL_DIAGNOSTIC_PARAMETER_H
#define HDVENT_CONTROL_DIAGNOSTIC_PARAMETER_H
#include <Arduino.h>

class Diagnostic_Parameter {
public:
    enum Alarm {TOO_LOW, OK, TOO_HIGH};
    Diagnostic_Parameter(float initialValue, float maxAlarm, float minAlarm=0);

    float getMaxAlarm() const;

    float getMinAlarm() const;

    float getValue() const;

    void setMaxAlarm(float maxAlarm);

    void setMinAlarm(float minAlarm);

    Alarm checkAlarm();

private:
    float _value;
    float _maxAlarm;
    float _minAlarm;
};


#endif //HDVENT_CONTROL_DIAGNOSTIC_PARAMETER_H
