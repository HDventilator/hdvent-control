//
// Created by david on 11.04.20.
//

#include "Honeywell_SSC.h"
#include <Arduino.h>
#include <Wire.h>
#include "SensorWrapper.h"

Honeywell_SSC::Honeywell_SSC(int i2cAddress, int sensorID, float pressureMin, float pressureMax, float outputMin,
                             float outputMax) {
    _i2cAddress = i2cAddress;
    _outputMin = outputMin;
    _outputMax = outputMax;
    _pressureMax = pressureMax;
    _pressureMin = pressureMin;
    _sensorID = sensorID;
}

bool Honeywell_SSC::begin()
{
    // Enable I2C
    Wire.begin();
}

int Honeywell_SSC::getStatus(){
    return _status;
}

int Honeywell_SSC::getI2CAddress() const {
    return _i2cAddress;
}

float Honeywell_SSC::transferFunction(uint16_t data) {
    float pressure;
    pressure = ((float) data - _outputMin) /(_outputMax - _outputMin)* (_pressureMax - _pressureMin) + _pressureMin;
    return (pressure);
}

void Honeywell_SSC::getSensor(sensor_t *sensor)
{

    /* Clear the sensor_t object */
    memset(sensor, 0, sizeof(sensor_t));

    /* Insert the sensor name in the fixed length char array */
    strncpy (sensor->name, "BMP085", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name)- 1] = 0;
    sensor->version     = 1;
    sensor->sensor_id   = _sensorID;
    sensor->min_delay   = 0;
    sensor->max_value   = _pressureMax;
    sensor->min_value   = _pressureMin;
    sensor->resolution  = 0;
}

bool Honeywell_SSC::readSensor(sensors_event_t *event, int timestamp)
{

    int i2c_address = _i2cAddress;
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

    /* Clear the event */
    memset(event, 0, sizeof(sensors_event_t));
    event->version = sizeof(sensors_event_t);

    event->sensor_id = _sensorID;
    (event->pt).temperature = temperature;
    (event->pt).pressure = pressure;
    return true;
}