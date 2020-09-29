//
// Created by david on 16.04.20.
//

#ifndef HDVENT_CONTROL_USER_PARAMETER_H
#define HDVENT_CONTROL_USER_PARAMETER_H

#include <Serial_Protocol.h>

const char USER_PARAMETER_VALUE_ID_PREFIX[] = "Pv";
const char USER_PARAMETER_MIN_ID_PREFIX[] = "P_";
const char USER_PARAMETER_MAX_ID_PREFIX[] = "PM";
const char USER_PARAMETER_DISPLAY_ID_PREFIX[] = "DM";

class User_Parameter {
public:
    User_Parameter(){};
    User_Parameter(float initialValue, float minOutValue, float maxOutValue, char string[], float minInValue=0, float maxInValue=1024, bool invert=false);

    float getValue() const;

    void setValue(float value);
    char* lcdString;
    float getDialValue();
    void loadValue(int data);
    void saveValue();
    package_struct_float_t getValuePackage();
    package_struct_float_t getMinPackage();
    package_struct_float_t getMaxPackage();
    package_struct_float_t getDialValuePackage();

    void setDialValue(float dialValue);
    void resetDialValue();
    bool hasChanged() const;
    void setMin(float min);
    void setMax(float max);
    float increment;

private:
    float _dialValue;
    float _value;
    float _min;
    float _max;
    float _minIn;
    float _maxIn;
    float _oldIn;
    bool _valueChanged;
    float _toleranceInputChange;
    float transferFunction(float valueIn);
    package_struct_float_t _package;
    void _packStruct(float value);
    bool _invert;


};



#endif //HDVENT_CONTROL_USER_PARAMETER_H
