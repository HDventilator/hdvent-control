//
// Created by david on 17.04.20.
//

#include "Diagnostic_Parameter.h"
#include <CRC32.h>

Diagnostic_Parameter::Diagnostic_Parameter(float initialValue, float minAlarm, float maxAlarm, char *identifier, char string[]){
    _loAlarm = minAlarm;
    _hiAlarm = maxAlarm;
    _value = initialValue;
    _identifier = identifier;
    lcdString = string;
}

Diagnostic_Parameter::Diagnostic_Parameter(float initialValue, float minAlarm, float maxAlarm, char *identifier){
    _loAlarm = minAlarm;
    _hiAlarm = maxAlarm;
    _value = initialValue;
    _identifier = identifier;
    lcdString = "none";
}


Diagnostic_Parameter::Diagnostic_Parameter(char *identifier, char *string,
                                           Diagnostic_Parameter::AlarmSetting hiAlarmSet,
                                           Diagnostic_Parameter::AlarmSetting loAlarmSet, float minAlarm,
                                           float maxAlarm) {
    _hiAlarmSet=INACTIVE;//_hiAlarmSet;
    _loAlarmSet=INACTIVE;//_loAlarmSet;
    lcdString=string;
    _identifier=identifier;
    _increment=(maxAlarm-minAlarm)/40;
    _value =0;
}

Diagnostic_Parameter::Diagnostic_Parameter(char *identifier, char *string, float minAlarm, float maxAlarm) {
    _hiAlarmSet=INACTIVE;//_hiAlarmSet;
    _loAlarmSet=INACTIVE;//_loAlarmSet;
    _minAlarm = minAlarm;
    _maxAlarm = maxAlarm;
    lcdString=string;
    _identifier=identifier;
    _increment=(maxAlarm-minAlarm)/40;
    _value =0;
}

Diagnostic_Parameter::Diagnostic_Parameter(char *identifier, char *string) {
    _hiAlarmSet=DISABLED;//_hiAlarmSet;
    _loAlarmSet=DISABLED;//_loAlarmSet;
    lcdString=string;
    _identifier=identifier;
    _value =0;

}

/*
Diagnostic_Parameter::Diagnostic_Parameter(char *identifier, char string[], AlarmSetting _hiAlarmSet, AlarmSetting _loAlarmSet):
_hiAlarmSet(_hiAlarmSet),
_loAlarmSet(_loAlarmSet),
lcdString(string),
_identifier(identifier)
{
    _value =0;
}
*/




float Diagnostic_Parameter::getValue() const {
    return _value;
}

Diagnostic_Parameter::Alarm Diagnostic_Parameter::getState(){
    return _state;
}

void Diagnostic_Parameter::setValue(float value) {
    _value = value;
    checkAlarm();
}

package_struct_float_t Diagnostic_Parameter::preparePackage(const char *prefix, float value) {
    package_struct_float_t dataPackage{};
    dataPackage.identifier[0]=prefix[0];
    dataPackage.identifier[1]=prefix[1];
    //memcpy(&dataPackage.identifier+2,_identifier,4);
    dataPackage.identifier[2]=_identifier[0];
    dataPackage.identifier[3]=_identifier[1];
    dataPackage.identifier[4]=_identifier[2];
    dataPackage.identifier[5]=_identifier[3];
    dataPackage.value = value;
    CRC32 crc;
    crc.update((uint8_t*) &dataPackage, IDENTIFIER_LENGTH+4);
    dataPackage.checksum = crc.finalize();

    return dataPackage;
}



package_struct_float_t Diagnostic_Parameter::getPackageStruct() {
    package_struct_float_t dataPackage{};
    dataPackage.identifier[0]=DIAGNOSTIC_PARAMETER_ID_PREFIX[0];
    dataPackage.identifier[1]=DIAGNOSTIC_PARAMETER_ID_PREFIX[1];
    //memcpy(&dataPackage.identifier+2,_identifier,4);
    dataPackage.identifier[2]=_identifier[0];
    dataPackage.identifier[3]=_identifier[1];
    dataPackage.identifier[4]=_identifier[2];
    dataPackage.identifier[5]=_identifier[3];

    dataPackage.value = _value;
    CRC32 crc;
    crc.update((uint8_t*) &dataPackage, IDENTIFIER_LENGTH+4);
    dataPackage.checksum = crc.finalize();
    return dataPackage;
}


char* Diagnostic_Parameter::getIdentifier() const {
    return _identifier;
}

Diagnostic_Parameter::Diagnostic_Parameter() {

}

Diagnostic_Parameter::AlarmSetting Diagnostic_Parameter::getHiAlarmSet() const {
    return _hiAlarmSet;
}

Diagnostic_Parameter::AlarmSetting Diagnostic_Parameter::getLoAlarmSet() const {
    return _loAlarmSet;
}

void Diagnostic_Parameter::setHiAlarmSet(Diagnostic_Parameter::AlarmSetting hiAlarmSet) {
    _hiAlarmSet = hiAlarmSet;
}

void Diagnostic_Parameter::setLoAlarmSet(Diagnostic_Parameter::AlarmSetting loAlarmSet) {
    _loAlarmSet = loAlarmSet;
}

float Diagnostic_Parameter::getMinAlarm() const {
    return _minAlarm;
}

void Diagnostic_Parameter::setMinAlarm(float minAlarm) {
    _minAlarm = minAlarm;
}

float Diagnostic_Parameter::getMaxAlarm() const {
    return _maxAlarm;
}

void Diagnostic_Parameter::setMaxAlarm(float maxAlarm) {
    _maxAlarm = maxAlarm;
}

float Diagnostic_Parameter::getIncrement() const {
    return _increment;
}

void Diagnostic_Parameter::setHiAlarm(float hiAlarm) {
    _hiAlarm = hiAlarm;
    checkAlarm();
}

void Diagnostic_Parameter::setLoAlarm(float loAlarm) {
    _loAlarm = loAlarm;
    checkAlarm();
}

float Diagnostic_Parameter::getHiAlarm() const {
    return _hiAlarm;
}

float Diagnostic_Parameter::getLoAlarm() const {
    return _loAlarm;
}

void Diagnostic_Parameter::resetPersistentAlarm() {
    _persistentState = Alarm::OK;
}

void Diagnostic_Parameter::checkAlarm() {
    //Serial.println("checking...");
    if ((_loAlarmSet==ACTIVE) && (_value<_loAlarm)){
        _state = TOO_LOW;
        _persistentState = _state;
    }
    else if ((_hiAlarmSet==ACTIVE) && (_value>_hiAlarm)){
        //Serial.println("To high");
        _state = TOO_HIGH;
        _persistentState = _state;
    }
    else {
        _state = OK;
    }
}

Diagnostic_Parameter::Alarm Diagnostic_Parameter::getPersistentState() const {
    return _persistentState;
}

package_struct_float_t Diagnostic_Parameter::getLoAlarmThresholdPackage() {
    package_struct_float_t package= preparePackage(DIAGNOSTIC_PARAMETER_LOW_ALARM_PREFIX, _loAlarm);
    return package;
}

package_struct_float_t Diagnostic_Parameter::getHiAlarmThresholdPackage() {
    package_struct_float_t package= preparePackage(DIAGNOSTIC_PARAMETER_HIGH_ALARM_PREFIX, _hiAlarm);
    return package;
}

package_struct_float_t Diagnostic_Parameter::getAlarmTriggeredPackage() {
    package_struct_float_t package= preparePackage(DIAGNOSTIC_PARAMETER_ALARM_TRIGGERED_PREFIX, _persistentState);
    return package;
}



package_struct_float_t Diagnostic_Parameter::getSettingsAlarmPackage() {
    float value =0;
    if (_hiAlarmSet==ACTIVE&&_loAlarmSet==ACTIVE){
        value=3;
    }
    else if (_hiAlarmSet==ACTIVE){
        value=2;
    }
    else if (_loAlarmSet==ACTIVE){
        value=1;
    }
    else {
        value=0;
    }
    package_struct_float_t package= preparePackage(DIAGNOSTIC_PARAMETER_SETTINGS_ALARM_PREFIX, value);
    return package;
}

bool Diagnostic_Parameter::isAlarmSettingChanged() const {
    return _alarmSettingChanged;
}

void Diagnostic_Parameter::setAlarmSettingChanged(bool alarmSettingChanged) {
    _alarmSettingChanged = alarmSettingChanged;
}




