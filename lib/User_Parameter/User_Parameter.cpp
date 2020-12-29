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

package_struct_float_t User_Parameter::getValuePackage() {
    _package.identifier[0]=USER_PARAMETER_VALUE_ID_PREFIX[0];
    _package.identifier[1]=USER_PARAMETER_VALUE_ID_PREFIX[1];
    _packStruct(_value);
    return _package;
}

package_struct_float_t User_Parameter::getMinPackage() {
    _package.identifier[0]=USER_PARAMETER_MIN_ID_PREFIX[0];
    _package.identifier[1]=USER_PARAMETER_MIN_ID_PREFIX[1];
    _packStruct(_min);
    return _package;
}

package_struct_float_t User_Parameter::getMaxPackage() {
    _package.identifier[0]=USER_PARAMETER_MAX_ID_PREFIX[0];
    _package.identifier[1]=USER_PARAMETER_MAX_ID_PREFIX[1];
    _packStruct(_max);
    return _package;
}

package_struct_float_t User_Parameter::getDialValuePackage() {
    _package.identifier[0]=USER_PARAMETER_DISPLAY_ID_PREFIX[0];
    _package.identifier[1]=USER_PARAMETER_DISPLAY_ID_PREFIX[1];
    _packStruct(_dialValue);
    return _package;
}

void User_Parameter::_packStruct(float value) {
    _package.identifier[2]=lcdString[0];
    _package.identifier[3]=lcdString[1];
    _package.identifier[4]=lcdString[2];
    _package.identifier[5]=lcdString[3];// lcdString contains only three characters
    _package.value = value;
    CRC32 crc;
    crc.update((uint8_t*) &_package, IDENTIFIER_LENGTH+4);
    _package.checksum = crc.finalize();
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