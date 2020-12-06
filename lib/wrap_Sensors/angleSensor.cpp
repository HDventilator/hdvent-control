//
// Created by david on 14.04.20.
//

#include "angleSensor.h"

Angle_Sensor::Angle_Sensor(int pin, int stepRange, int angleRange, float analogMin, float analogMax, int homeTolerance){
    _pin = pin;
    _homeTolerance = homeTolerance;
    _stepsFullTurn = stepRange;
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

float Angle_Sensor::transferFunction(uint16_t data) {
    float stepPosition = (float)(data-_analogMin) / (float)(_analogMax-_analogMin) * (float)_angleRange / 360 * (float)_stepsFullTurn;
    return stepPosition;
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

int Angle_Sensor::getEeAddress() const {
    return _eeAddress;
}

void Angle_Sensor::setEeAddress(int eeAddress) {
    _eeAddress = eeAddress;
}

float Angle_Sensor::getHome() const {
    return _home;
}

void Angle_Sensor::setHome(int home) {
    _home = home;
}
