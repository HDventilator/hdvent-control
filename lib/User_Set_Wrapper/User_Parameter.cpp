//
// Created by david on 16.04.20.
//

#include "User_Parameter.h"

User_Parameter::User_Parameter(float minOutValue, float maxOutValue, float minInValue, float maxInValue) {
    _min = minOutValue;
    _max = maxOutValue;
    _value = 0;
    _minIn = minInValue;
    _maxIn = maxInValue;
}

float User_Parameter::getValue() const {
    return _value;
}

void User_Parameter::setValue(float value) {
    _value = value;
}

