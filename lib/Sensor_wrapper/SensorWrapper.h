//
// Created by david on 11.04.20.
//

#ifndef _SENSORWRAPPER_H
#define _SENSORWRAPPER_H

#include <Arduino.h>

class Sensor {
public:
    // Constructor(s)
    Sensor() {}
    virtual ~Sensor() {}
    virtual bool readSensor(int timestamp) = 0;
   // virtual void getSensor() = 0;

    //void printSensorDetails(void);

    virtual bool begin();
    char name[12];     /**< sensor name */
    int32_t version;   /**< version of the hardware + driver */
    int32_t sensor_id; /**< unique sensor identifier */
    int32_t type;      /**< this sensor's type */
    float resolution; /**< smallest difference between two values reported by this
                       sensor */
    int32_t min_delay; /**< min delay in microseconds between events. zero = not a
                        constant rate */
};


#endif _SENSORWRAPPER_H
