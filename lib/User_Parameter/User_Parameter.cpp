//
// Created by david on 16.04.20.
//

#include <Serial_Protocol.h>
#include "User_Parameter.h"
#include <CRC32.h>

User_Parameter::User_Parameter(float initialValue, float minOutValue, float maxOutValue, char string[], float minInValue, float maxInValue) {
    _min = minOutValue;
    _max = maxOutValue;
    _value = initialValue;
    _displayValue = initialValue;
    _minIn = minInValue;
    _maxIn = maxInValue;
    lcdString = string;
    step = (_max-_min)/200;

}

float User_Parameter::getValue() const {
    return _value;
}

void User_Parameter::setValue(float value) {
    _value = value;
}

float User_Parameter::transferFunction(int valueIn){
    return ((float)valueIn-_minIn) / (_maxIn-_minIn) * (_max-_min) + _min;
}

float User_Parameter::getDisplayValue() const {
    return _displayValue;
}

void User_Parameter::setDisplayValue(float displayValue) {
    _displayValue = displayValue;
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

package_struct_float_t User_Parameter::get_displayValuePackage() {
    _package.identifier[0]=USER_PARAMETER_DISPLAY_ID_PREFIX[0];
    _package.identifier[1]=USER_PARAMETER_DISPLAY_ID_PREFIX[1];
    _packStruct(_displayValue);
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