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
    Diagnostic_Parameter();

    enum Alarm {TOO_LOW, OK, TOO_HIGH};
    enum AlarmSetting {ACTIVE, INACTIVE, PERMITTED};
    Diagnostic_Parameter(float initialValue, float minAlarm, float maxAlarm, char *identifier, char string[]);
    Diagnostic_Parameter(char *identifier, char string[], AlarmSetting hiAlarmSet, AlarmSetting loAlarmSet);
    Diagnostic_Parameter(float initialValue, float minAlarm, float maxAlarm, char *identifier);


    float getMaxAlarm() const;

    float getMinAlarm() const;

    float getValue() const;

    void setValue(float value);

    void setMaxAlarm(float maxAlarm);

    void setMinAlarm(float minAlarm);


    Alarm checkAlarm();
    char* lcdString;

    package_struct_float_t getPackageStruct();


private:
    float _value;
    AlarmSetting _hiAlarmSet;
public:
    AlarmSetting getHiAlarmSet() const;

private:
    AlarmSetting _loAlarmSet;
public:
    AlarmSetting getLoAlarmSet() const;

private:
    float _hiAlarm;
    float _loAlarm;
    char* _identifier;
public:
    char *getIdentifier() const;
};

typedef union  {
    struct mytype_t {
        Diagnostic_Parameter peep, tidalVolume, volume, flow, airwayPressure, respiratoryRate, plateauPressure, meanPressure, pressureChange;
        //Aggregated_Parameter minuteVolume= Aggregated_Parameter(0, 0, 0, "mvol", 60);
    } s;
    Diagnostic_Parameter arr[9];} diagnosticParameters_t;

#endif //HDVENT_CONTROL_DIAGNOSTIC_PARAMETER_H
