//
// Created by david on 17.05.20.
//

#include "Optical_Sensor.h"

Optical_Sensor::Optical_Sensor(int pin) {
    _pin = pin;
}

bool Optical_Sensor::readSensor() {
    _data.isBlocked = digitalRead(_pin);
    return true;
}

void Optical_Sensor::interruptRoutine() {
    _data.isBlocked = digitalRead(_pin);
}

const Optical_Sensor::sensor_values_t &Optical_Sensor::getData() const {
    return _data;
}
