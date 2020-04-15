//
// Created by david on 11.04.20.
//

#include "Honeywell_SSC.h"
#include <Arduino.h>
#include <Wire.h>
#include "SensorWrapper.h"
#include "angleSensor.h"
Honeywell_SSC::Honeywell_SSC(int i2cAddress, int sensorID, float pressureMin, float pressureMax, float outputMin,
                             float outputMax) {

    _outputMin = outputMin;
    _outputMax = outputMax;
    _pressureMax = pressureMax;
    _pressureMin = pressureMin;
    setSensorId(sensorID);
    setI2CAddress(i2cAddress);
}

bool Honeywell_SSC::begin()
{
    // Enable I2C
    Wire.begin();
}

float Honeywell_SSC::transferFunction(uint16_t data) {
    float pressure;
    pressure = ((float) data - _outputMin) /(_outputMax - _outputMin)* (_pressureMax - _pressureMin) + _pressureMin;
    return (pressure);
}

bool Honeywell_SSC::readSensor( int timestamp)
{
    int i2c_address = getI2CAddress();
    Wire.requestFrom(i2c_address, 4);
    while(Wire.available() == 0);

    byte a     = Wire.read(); // first received byte stored here ....Example bytes one: 00011001 10000000
    byte b     = Wire.read(); // second received byte stored here ....Example bytes two: 11100111 00000000
    byte c     = Wire.read(); // third received byte stored here
    byte d     = Wire.read(); // fourth received byte stored here

    byte status = (a & 0xc0) >> 6;
    int bridge_data = ((a & 0x3f) << 8) + b;
    int temperature_data = ((c << 8) + (d & 0xe0)) >> 5;

    float pressure = transferFunction(bridge_data);
    auto temperature = (float) temperature_data* 0.0977 - 50.0;

    _data.temperature = temperature;
    _data.pressure = pressure;
    return true;
}

const Honeywell_SSC::sensor_values_t &Honeywell_SSC::getDataMin() const {
    return _dataMin;
}

const Honeywell_SSC::sensor_values_t &Honeywell_SSC::getData() const {
    return _data;
}

const Honeywell_SSC::sensor_values_t &Honeywell_SSC::getDataMax() const {
    return _dataMax;
}


