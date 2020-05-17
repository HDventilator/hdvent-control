//
// Created by david on 17.05.20.
//

#ifndef HDVENT_CONTROL_STEPPER_MONITOR_H
#define HDVENT_CONTROL_STEPPER_MONITOR_H

#include <SensorWrapper.h>
#include <Arduino.h>
#include <powerSTEP01ArduinoLibrary.h>

class Stepper_Monitor : public Sensor {
public:
    typedef struct{
        uint16_t relativePosition;
        uint16_t absolutePosition;
        bool isHome;
    } sensor_values_t;
    Stepper_Monitor(powerSTEP* stepper);
    bool readSensor() override ;

    const sensor_values_t &getData() const;

private:
    powerSTEP* _stepper;
    sensor_values_t _data;
    uint16_t _homeTolerance;
};


#endif //HDVENT_CONTROL_STEPPER_MONITOR_H
