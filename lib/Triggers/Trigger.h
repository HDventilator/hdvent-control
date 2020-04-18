//
// Created by david on 18.04.20.
//

#ifndef HDVENT_CONTROL_TRIGGER_H
#define HDVENT_CONTROL_TRIGGER_H

#include <Arduino.h>

class Trigger{
public:
    Trigger();

private:
    bool _high;
    bool _active;
};

template<typename T, size_t N>
bool anyTrue(T (&array)[N])
{
    bool result = array[0];
    for (int a = 1; ~result && a < N; ++a)
    {
        result = result || array[a];
    }
    return result;
}

#endif //HDVENT_CONTROL_TRIGGER_H
