//
// Created by david on 16.04.20.
//

#include "User_Parameter.h"

User_Parameter::User_Parameter(float initialValue, float minOutValue, float maxOutValue, char string[], float minInValue, float maxInValue) {
    _min = minOutValue;
    _max = maxOutValue;
    _value = initialValue;
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
