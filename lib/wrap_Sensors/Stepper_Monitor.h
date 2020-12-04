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
        int16_t relativePosition;
        int16_t absolutePosition;
        bool isHome;
    } sensor_values_t;
    Stepper_Monitor(powerSTEP *stepper, int homeTolerance, uint8_t positiveDir);
    bool readSensor() override ;

    const sensor_values_t &getData() const;

private:
    powerSTEP* _stepper;
    sensor_values_t _data;
    uint16_t _homeTolerance;
    int8_t _positionSign;
};


#endif //HDVENT_CONTROL_STEPPER_MONITOR_H
