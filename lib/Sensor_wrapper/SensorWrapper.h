//
// Created by david on 11.04.20.
//

#ifndef _SENSORWRAPPER_H
#define _SENSORWRAPPER_H

#include <Arduino.h>

/* Sensor details (40 bytes) */
/** struct sensor_s is used to describe basic information about a specific
 * sensor. */
typedef struct {
    char name[12];     /**< sensor name */
    int32_t version;   /**< version of the hardware + driver */
    int32_t sensor_id; /**< unique sensor identifier */
    int32_t type;      /**< this sensor's type (ex. SENSOR_TYPE_LIGHT) */
    float max_value;   /**< maximum value of this sensor's value in SI units */
    float min_value;   /**< minimum value of this sensor's value in SI units */
    float resolution; /**< smallest difference between two values reported by this
                       sensor */
    int32_t min_delay; /**< min delay in microseconds between events. zero = not a
                        constant rate */
} sensor_t;

/* Sensor event (36 bytes) */
/** struct sensor_event_s is used to provide a single sensor event in a common
 * format. */
typedef struct {
    int32_t version;
    int32_t sensor_id; /**< unique sensor identifier */
    int32_t type;      /**< sensor type */
    int32_t timestamp; /**< time is in milliseconds */

    union {
        byte data[4];              ///< Raw data
        float angle;    /**< distance in centimeters */
        float light;       /**< light in SI lux units */
        float pressure;    /**< pressure in hectopascal (hPa) */
        float temperature;
        float current;           /**< current in milliamps (mA) */
        float voltage;           /**< voltage in volts (V) */
        struct {
            float pressure;
            float temperature;
        } pt;
    };                         ///< Union for the wide ranges of data we can carry
} sensors_event_t;


class Sensor {
public:
    // Constructor(s)
    Sensor() {}
    virtual ~Sensor() {}
    virtual bool readSensor(sensors_event_t *, int timestamp) = 0;
    virtual void getSensor(sensor_t *) = 0;

    void printSensorDetails(void);

    virtual bool begin();
};


#endif _SENSORWRAPPER_H
