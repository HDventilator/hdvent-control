//
// Created by david on 16.04.20.
//

#ifndef HDVENT_CONTROL_USER_PARAMETER_H
#define HDVENT_CONTROL_USER_PARAMETER_H

const char USER_PARAMETER_VALUE_ID_PREFIX[] = "Pv";
const char USER_PARAMETER_MIN_ID_PREFIX[] = "P_";
const char USER_PARAMETER_MAX_ID_PREFIX[] = "P^";
const char USER_PARAMETER_DISPLAY_ID_PREFIX[] = "DM";

class User_Parameter {
public:
    User_Parameter(){};
    User_Parameter(float initialValue, float minOutValue, float maxOutValue, char string[], float minInValue=0, float maxInValue=1);

    float getValue() const;

    void setValue(float value);
    char* lcdString;
    float step;
    float getDisplayValue() const;

    void setDisplayValue(float displayValue);
    package_struct_float_t getValuePackage();
    package_struct_float_t getMinPackage();
    package_struct_float_t getMaxPackage();
    package_struct_float_t get_displayValuePackage();

private:
    float _displayValue;
    float _value;
    float _min;
    float _max;
    float _minIn;
    float _maxIn;
    float transferFunction(int data);
    package_struct_float_t _package;
    void _packStruct(float value);

};



#endif //HDVENT_CONTROL_USER_PARAMETER_H
