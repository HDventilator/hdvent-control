//
// Created by david on 14.04.20.
//

#ifndef HDVENT_CONTROL_ANGLESENSOR_H
#define HDVENT_CONTROL_ANGLESENSOR_H
#include "SensorWrapper.h"
#include <Arduino.h>

class Angle_Sensor : public Sensor {
public:
    typedef struct{
        float relativePosition;
        float absolutePosition;
        bool isHome;
    } sensor_values_t;

    /// @brief Initialise analog angle sensor
    /// @param pin Analog pin, the sensor output is connected to
    /// @param stepRange Number of (micro-)steps in one full turn of the Stepper Motor (e.g. 200*STEP_DIVIDER)
    /// @param angleRange Angle interval in deg that is mapped on the [analogMin,analogMax] range
    /// @param analogMin
    /// @param analogMax
    /// @param homeTolerance Number of (micro-) steps, that the relativePosition can deviate from 0
    /// before isHome becomes false
    Angle_Sensor(int pin,  int stepRange, int angleRange=360, int analogMin=0, int analogMax=1024, int homeTolerance=10);


    /// @brief update the position data, by calling analogRead on the initialised pin
    /// @param timestamp Timestamp associated to the current read out
    /// @return true if read succeed, else:false
    bool readSensor () override;

    /// @brief Make current position home.
    void resetPos();

    /// @brief Mark current position.
    void markPos();




private:
    int transferFunction(uint16_t data);
    int _home; // step value that corresponds to home position
    int _mark; // step value that corresponds to mark position
    int _pin;
    int _homeTolerance;
    int _stepRange;
    int _angleRange;
    int _analogMin;
    int _analogMax;
    sensor_values_t _data;
    bool checkHome(int16_t stepValue);
};


#endif //HDVENT_CONTROL_ANGLESENSOR_H
