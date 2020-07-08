//
// Created by david on 17.04.20.
//

#ifndef HDVENT_CONTROL_DIAGNOSTIC_PARAMETER_H
#define HDVENT_CONTROL_DIAGNOSTIC_PARAMETER_H
#include <Arduino.h>
#include "../Serial_Protocol/Serial_Protocol.h"

const char DIAGNOSTIC_PARAMETER_ID_PREFIX[] = {"DM"};

class Diagnostic_Parameter {
public:
    enum Alarm {TOO_LOW, OK, TOO_HIGH};
    Diagnostic_Parameter(float initialValue, float minAlarm, float maxAlarm, char *identifier);
    Diagnostic_Parameter(char *identifier);
    Diagnostic_Parameter();

    float getMaxAlarm() const;

    float getMinAlarm() const;

    float getValue() const;

    void setValue(float value);

    void setMaxAlarm(float maxAlarm);

    void setMinAlarm(float minAlarm);


    Alarm checkAlarm();

    package_struct_float_t getPackageStruct();

private:
    float _value;
    bool _maxAlarmSet;
    bool _minAlarmSet;
    float _maxAlarm;
    float _minAlarm;
    char* _identifier;
};

#endif //HDVENT_CONTROL_DIAGNOSTIC_PARAMETER_H
