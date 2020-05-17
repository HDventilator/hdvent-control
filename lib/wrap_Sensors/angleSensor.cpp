//
// Created by david on 14.04.20.
//

#include "angleSensor.h"

Angle_Sensor::Angle_Sensor(int pin, int stepRange, int angleRange, int analogMin, int analogMax, int homeTolerance){
    _pin = pin;
    _homeTolerance = homeTolerance;
    _stepRange = stepRange;
    _angleRange = angleRange;
    _analogMin = analogMin;
    _analogMax = analogMax;
    _homeTolerance = homeTolerance;
}

bool Angle_Sensor::readSensor(){
    int16_t analogValue = analogRead(_pin);
    int16_t stepValue = transferFunction(analogValue);

    _data.absolutePosition = stepValue;
    _data.relativePosition = stepValue-_home;
    _data.isHome = checkHome(stepValue);
    return (true);
}

bool Angle_Sensor::checkHome(int16_t stepValue){
    return abs(_home - stepValue) < _homeTolerance;
}

int Angle_Sensor::transferFunction(uint16_t data) {
    float stepPosition = (float)(data-_analogMin) / (float)(_analogMax-_analogMin) * (float)_angleRange / 360 * (float)_stepRange;
    return (int)stepPosition;
}

void Angle_Sensor::markPos() {
    _mark = _data.absolutePosition;
}

void Angle_Sensor::resetPos() {
    _home = _data.absolutePosition;
}

const Angle_Sensor::sensor_values_t &Angle_Sensor::getData() const {
    return _data;
}
