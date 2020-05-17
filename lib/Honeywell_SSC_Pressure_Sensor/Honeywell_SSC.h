//
// Created by david on 11.04.20.
//

#ifndef _HONEYWELL_SSC
#define _HONEYWELL_SSC
#include "Arduino.h"
#include "SensorWrapper.h"


class Honeywell_SSC: public Sensor
{
public:
    typedef struct{
        float pressure;
        float temperature;
    } sensor_values_t;

    Honeywell_SSC(int i2cAddress, int sensorID,
                  float pressureMin, float pressureMax, float outputMin, float outputMax);
    bool  begin() override;
    bool readSensor() override;

    const sensor_values_t &getDataMin() const;
    const sensor_values_t &getDataMax() const;
    const sensor_values_t &getData() const;

private:
    sensor_values_t  _data;
    sensor_values_t _dataMin;
    sensor_values_t _dataMax;

    float transferFunction(uint16_t data);

    float _outputMin;
    float _outputMax;
    float _pressureMin;
    float  _pressureMax;

};

#endif

