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
    Diagnostic_Parameter(char *identifier, char string[], AlarmSetting hiAlarmSet, AlarmSetting loAlarmSet, float minAlarm, float maxAlarm);
    Diagnostic_Parameter(float initialValue, float minAlarm, float maxAlarm, char *identifier);
    Diagnostic_Parameter(char *identifier, char string[],  float minAlarm, float maxAlarm);
    Diagnostic_Parameter(char *identifier, char string[]);


    float getMinAlarm() const;

    void setMinAlarm(float minAlarm);

    float getMaxAlarm() const;

    void setMaxAlarm(float maxAlarm);

    float getValue() const;

    void setValue(float value);

    Alarm checkAlarm();
    char* lcdString;

    package_struct_float_t getPackageStruct();

    void setHiAlarmSet(AlarmSetting hiAlarmSet);

    void setLoAlarmSet(AlarmSetting loAlarmSet);

    AlarmSetting getHiAlarmSet() const;
    AlarmSetting getLoAlarmSet() const;
    char *getIdentifier() const;

    float getIncrement() const;

private:
    float _increment;
    float _minAlarm;
    float _maxAlarm;
    AlarmSetting _loAlarmSet;
    float _value;
    AlarmSetting _hiAlarmSet;
    float _hiAlarm;
    float _loAlarm;
    char* _identifier;

};

typedef union  {
    struct mytype_t {
        Diagnostic_Parameter peep, tidalVolume, volume, flow, airwayPressure, respiratoryRate, plateauPressure, meanPressure, pressureChange;
        //Aggregated_Parameter minuteVolume= Aggregated_Parameter(0, 0, 0, "mvol", 60);
    } s;
    Diagnostic_Parameter arr[9];} diagnosticParameters_t;

#endif //HDVENT_CONTROL_DIAGNOSTIC_PARAMETER_H
