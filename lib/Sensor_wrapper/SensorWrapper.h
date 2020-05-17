//
// Created by david on 11.04.20.
//

#ifndef _SENSORWRAPPER_H
#define _SENSORWRAPPER_H

#include <Arduino.h>

class Sensor {
    // TODO averaging
public:
    // Constructor(s)
    Sensor() {}
    virtual ~Sensor(){}
    virtual bool readSensor() = 0;
    enum state_t {OK=0, TIME_OUT, FAULTY, DISCONNECTED};
    virtual bool begin(){};

    state_t getState() const {
        return _state;
    }

    int getI2CAddress() const {
        return _i2cAddress;
    }

    void setI2CAddress(int i2CAddress) {
        _i2cAddress = i2CAddress;
    }

    int getSensorId() const {
        return _sensorID;
    }

    void setSensorId(int sensorId) {
        _sensorID = sensorId;
    }

    void setState(state_t state) {
        _state = state;
    }

private:
    float resolution; /**< smallest difference between two values reported by this
                       sensor */
    int32_t timeout; /**< min delay in microseconds between events. zero = not a
                        constant rate */
    int _i2cAddress;
    int _sensorID;
    state_t _state;
};

#endif //_SENSORWRAPPER_H