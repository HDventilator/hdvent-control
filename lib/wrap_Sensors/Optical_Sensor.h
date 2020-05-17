//
// Created by david on 17.05.20.
//

#ifndef HDVENT_CONTROL_OPTICAL_SENSOR_H
#define HDVENT_CONTROL_OPTICAL_SENSOR_H
#include "SensorWrapper.h"
#include <Arduino.h>

class Optical_Sensor : public Sensor {
public:
    Optical_Sensor(int pin);
    typedef struct{
        bool isBlocked;
    } sensor_values_t;
    bool readSensor() override ;

    void interruptRoutine();

private:
    sensor_values_t _data;
    uint8_t _pin;
};


#endif //HDVENT_CONTROL_OPTICAL_SENSOR_H
