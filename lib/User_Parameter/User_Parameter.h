//
// Created by david on 16.04.20.
//

#ifndef HDVENT_CONTROL_USER_PARAMETER_H
#define HDVENT_CONTROL_USER_PARAMETER_H

#include <Serial_Protocol.h>
#include <Ventilation_Modes.h>

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

    float getValueEEPROM(int &index);
    void setDialValue(float dialValue);
    void resetDialValue();
    bool hasChanged() const;
    void setMin(float min);
    void setMax(float max);
    float increment;
    bool isGettingEdited;
    bool wasEdited=false;

    float getMin() const;

    float getMax() const;

    int getEeAddress() const;

    void setEeAddress(int eeAddress);


private:
    int eeAddress;
    float _oldDialValue;
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

template<uint8_t N>
class Parameter_Container {
public:
    Parameter_Container(User_Parameter *params) : nActive(N), params(params), _activeIndexes(nullptr) {}

    Parameter_Container() {

    }

    User_Parameter & operator[](int idx)       { return params[idx]; }
    const User_Parameter& operator[](int idx)      const  { return params[idx]; }
    void update(int activeIndexes[], int n){
        _activeIndexes = activeIndexes;
        nActive = n;
    };
    void update(UP* activeIndexes, int n){
        _activeIndexes = reinterpret_cast<int*>(activeIndexes);
        nActive = n;
    };

    User_Parameter& getActive(int i){
        return params[_activeIndexes[i]];
    }
    int nActive{};
    User_Parameter params[N];
private:

    int* _activeIndexes;

};


#endif //HDVENT_CONTROL_USER_PARAMETER_H
