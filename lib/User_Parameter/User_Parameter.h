//
// Created by david on 16.04.20.
//

#ifndef HDVENT_CONTROL_USER_PARAMETER_H
#define HDVENT_CONTROL_USER_PARAMETER_H


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

private:
    float _displayValue;
    float _value;
    float _min;
    float _max;
    float _minIn;
    float _maxIn;
    float transferFunction(int data);

};



#endif //HDVENT_CONTROL_USER_PARAMETER_H
