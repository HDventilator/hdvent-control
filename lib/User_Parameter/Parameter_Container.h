//
// Created by david on 29.12.20.
//

#ifndef HDVENT_CONTROL_PARAMETER_CONTAINER_H
#define HDVENT_CONTROL_PARAMETER_CONTAINER_H

#include <User_Parameter.h>
#include <Ventilation_Modes.h>

template<uint8_t N, typename T>
class Parameter_Container {
public:
    Parameter_Container(User_Parameter *params) : nActive(N), params(params), _activeIndexes(nullptr) {}

    Parameter_Container() {

    }

    User_Parameter & operator[](T idx)       { return params[idx]; }
    const User_Parameter& operator[](T idx)      const  { return params[idx]; }

    void update(int activeIndexes[], int n){
        _activeIndexes = activeIndexes;
        nActive = n;
    };

    void update(T* activeIndexes, int n){
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

#endif //HDVENT_CONTROL_PARAMETER_CONTAINER_H
