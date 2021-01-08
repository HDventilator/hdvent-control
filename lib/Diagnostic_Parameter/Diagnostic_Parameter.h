//
// Created by david on 17.04.20.
//

#ifndef HDVENT_CONTROL_DIAGNOSTIC_PARAMETER_H
#define HDVENT_CONTROL_DIAGNOSTIC_PARAMETER_H
#include <Arduino.h>
#include "../Serial_Protocol/Serial_Protocol.h"
#include "Aggregated_Parameter.h"

const char DIAGNOSTIC_PARAMETER_ID_PREFIX[] = {"DM"};
const char DIAGNOSTIC_PARAMETER_LOW_ALARM_PREFIX[] = {"LA"};
const char DIAGNOSTIC_PARAMETER_HIGH_ALARM_PREFIX[] = {"HA"};
const char DIAGNOSTIC_PARAMETER_SETTINGS_ALARM_PREFIX[] = {"SA"};
const char DIAGNOSTIC_PARAMETER_ALARM_TRIGGERED_PREFIX[] = {"TA"};

class Diagnostic_Parameter {
public:
    Diagnostic_Parameter();

    enum Alarm {OK=0, TOO_LOW=1, TOO_HIGH=2};
    enum AlarmSetting {ACTIVE, INACTIVE, DISABLED};
    Diagnostic_Parameter(float initialValue, float minAlarm, float maxAlarm, char *identifier, char string[]);
    Diagnostic_Parameter(char *identifier, char string[], AlarmSetting hiAlarmSet, AlarmSetting loAlarmSet, float minAlarm, float maxAlarm);
    Diagnostic_Parameter(float initialValue, float minAlarm, float maxAlarm, char *identifier);
    Diagnostic_Parameter(char *identifier, char string[],  float minAlarm, float maxAlarm);
    Diagnostic_Parameter(char *identifier, char string[]);
    void checkAlarm();


    float getMinAlarm() const;

    void setMinAlarm(float minAlarm);

    float getMaxAlarm() const;

    void setMaxAlarm(float maxAlarm);

    float getValue() const;

    void setValue(float value);

    void setHiAlarm(float hiAlarm);

    void setLoAlarm(float loAlarm);
    void resetPersistentAlarm();
    float getHiAlarm() const;

    float getLoAlarm() const;

    Alarm getState();
    char* lcdString;

    package_struct_float_t getPackageStruct();
    package_struct_float_t preparePackage(const char *prefix, float value);
    package_struct_float_t getLoAlarmThresholdPackage();
    package_struct_float_t getHiAlarmThresholdPackage();
    package_struct_float_t getSettingsAlarmPackage();


    void setHiAlarmSet(AlarmSetting hiAlarmSet);

    void setLoAlarmSet(AlarmSetting loAlarmSet);

    AlarmSetting getHiAlarmSet() const;
    AlarmSetting getLoAlarmSet() const;
    char *getIdentifier() const;

    Alarm getPersistentState() const;

    float getIncrement() const;

    bool isAlarmSettingChanged() const;

    void setAlarmSettingChanged(bool alarmSettingChanged);
    package_struct_float_t getAlarmTriggeredPackage();

private:
    Alarm _state=Alarm::OK;
    Alarm _persistentState = Alarm::OK;
    bool _alarmSettingChanged;
    float _increment;
    float _minAlarm;
    float _maxAlarm;
    AlarmSetting _loAlarmSet=DISABLED;
    float _value;
    AlarmSetting _hiAlarmSet=DISABLED;
    float _hiAlarm;
    float _loAlarm;
    char* _identifier;


};

typedef union  {
    struct mytype_t {
        Diagnostic_Parameter tidalVolume, peep, volume, flow, airwayPressure, plateauPressure, minuteVolume;
        //Aggregated_Parameter minuteVolume= Aggregated_Parameter(0, 0, 0, "mvol", 60);
    } s;
    Diagnostic_Parameter arr[9];
    Diagnostic_Parameter& operator[](int idx)       { return arr[idx]; }
    const Diagnostic_Parameter& operator[](int idx)      const  { return arr[idx]; }
} diagnosticParameters_t;


#endif //HDVENT_CONTROL_DIAGNOSTIC_PARAMETER_H
