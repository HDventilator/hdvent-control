//
// Created by david on 16.04.20.
//

#include <Serial_Protocol.h>
#include "User_Parameter.h"
#include <CRC32.h>
User_Parameter::User_Parameter(float initialValue, float minOutValue, float maxOutValue, char *string)
: wasEdited(false)
,_min(minOutValue)
, _max(maxOutValue)

{
    _value = min(initialValue, maxOutValue) ;
    _value = max(_value, minOutValue) ;
    _dialValue = _value;
    lcdString = string;
    increment = (_max-_min)/100;
    resetDialValue();
}

float User_Parameter::getValue() const {
    return _value;
}

void User_Parameter::setValue(float value) {
    _value = value;
    _dialValue =value;
}


float User_Parameter::getDialValue() {
    return _dialValue;
}

bool User_Parameter::hasChanged() const {
    return _valueChanged;
}

void User_Parameter::saveValue() {
    _value = _dialValue;

}

void User_Parameter::setMin(float min) {
    _min = min;
}

void User_Parameter::setMax(float max) {
    _max = max;
}

void User_Parameter::setDialValue(float dialValue) {
    _oldDialValue = _dialValue;
    _dialValue = dialValue;
}

void User_Parameter::resetDialValue() {
    _dialValue = _value;

}

float User_Parameter::getMin() const {
    return _min;
}

float User_Parameter::getMax() const {
    return _max;
}

int User_Parameter::getEeAddress() const {
    return eeAddress;
}

void User_Parameter::setEeAddress(int eeAddress) {
    User_Parameter::eeAddress = eeAddress;
}

float User_Parameter::getValueEEPROM(int &index) {
    eeAddress = index;
    index += sizeof(float);
    return _value;
}