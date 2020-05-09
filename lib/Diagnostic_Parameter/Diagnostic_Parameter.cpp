//
// Created by david on 17.04.20.
//

#include "Diagnostic_Parameter.h"
#include <CRC32.h>

Diagnostic_Parameter::Diagnostic_Parameter(float initialValue, float minAlarm, float maxAlarm, char *identifier){
    _minAlarm = minAlarm;
    _minAlarmSet = true;
    _maxAlarm = maxAlarm;
    _maxAlarmSet = true;
    _value = initialValue;
    _identifier = identifier;
}

Diagnostic_Parameter:: Diagnostic_Parameter(){
    _value=0;
};

float Diagnostic_Parameter::getMaxAlarm() const {
    return _maxAlarm;
}

float Diagnostic_Parameter::getMinAlarm() const {
    return _minAlarm;
}

void Diagnostic_Parameter::setMaxAlarm(float maxAlarm) {
    _maxAlarm = maxAlarm;
    _maxAlarmSet = true;
}

void Diagnostic_Parameter::setMinAlarm(float minAlarm) {
    _minAlarm = minAlarm;
    _minAlarmSet = true;
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

void Diagnostic_Parameter::setValue(float value) {
    _value = value;
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


