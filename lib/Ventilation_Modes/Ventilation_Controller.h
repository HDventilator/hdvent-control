//
// Created by david on 21.04.20.
//

#ifndef HDVENT_CONTROL_VENTILATION_CONTROLLER_H
#define HDVENT_CONTROL_VENTILATION_CONTROLLER_H

#include <PID_v1.h>
#include <Stopwatch.h>
#include "Ventilation_Modes.h"

class VentilationController {
public:
    VentilationController(VentilationMode mode, double kp, double ki, double kd);
    bool expirationTrigger();
    bool inspirationTrigger();
    float calcSpeed(double input);
    float calcSetPoint();
    void startRamp(float slope, float level);
private:
    VentilationMode _mode;
    PID _pid;
    double _pidIn;
    double _pidOut;
    double _pidSetpoint;
    float _slope;
    float _level;
    unsigned long _slopeTime;
    Stopwatch _timer;
};



#endif //HDVENT_CONTROL_VENTILATION_CONTROLLER_H
