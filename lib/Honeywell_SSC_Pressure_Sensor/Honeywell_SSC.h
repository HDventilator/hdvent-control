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
    Honeywell_SSC(int i2cAddress, int sensorID,
                  float pressureMin, float pressureMax, float outputMin, float outputMax);
    bool  begin();
    bool  readSensor(sensors_event_t*, int timestamp = 0);
    void  getSensor(sensor_t*);


private:
    int getI2CAddress() const;
    int getStatus();
    int _i2cAddress;
    float _pressureMin;
    float  _pressureMax;
    float  _outputMin;
    float  _outputMax;
    int _sensorID;
    int _status;
    float transferFunction(uint16_t);
    float getData();
};

#endif

