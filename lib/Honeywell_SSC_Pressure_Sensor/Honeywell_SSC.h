//
// Created by david on 11.04.20.
//

#ifndef _HONEYWELL_SSC
#define _HONEYWELL_SSC
#include "Arduino.h"
#include "SensorWrapper.h"

const int TCAADDR = 0x70;
void tcaselect(uint8_t i);

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
    uint8_t _tca_id;

    float _outputMin;
    float _outputMax;
    float _pressureMin;
    float  _pressureMax;

};

class SSC_100MD4A3: public Honeywell_SSC
{
public:
    SSC_100MD4A3(int sensorID):Honeywell_SSC(0x48, sensorID, -100, 100, 0.1*16383, 0.9*16383) {}
};

class SSC_016MD2A5: public Honeywell_SSC
{
public:
    SSC_016MD2A5(int sensorID):Honeywell_SSC(0x28, sensorID, -16, 16, 0.1*16383, 0.9*16383) {}
};

#endif

