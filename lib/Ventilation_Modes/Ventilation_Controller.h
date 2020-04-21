//
// Created by david on 21.04.20.
//

#ifndef HDVENT_CONTROL_VENTILATION_CONTROLLER_H
#define HDVENT_CONTROL_VENTILATION_CONTROLLER_H

#include <PID_v1.h>
#include <Stopwatch.h>
#include <Diagnostic_Parameter.h>
#include "Ventilation_Modes.h"

typedef float (*input_func_t)();

class VentilationController {
public:
    VentilationController(VentilationMode mode, double kp, double ki, double kd, Diagnostic_Parameter &pressure, Diagnostic_Parameter &flow);
    bool expirationTrigger();
    bool inspirationTrigger();
    float calcSpeed();
    float calcSetPoint();
    void startRamp(float slope, float level);

private:
    VentilationMode _mode;
    PID _pid;
    double _pidIn;
    double _pidOut;
    double _pidSetpoint;
    input_func_t _getInput;
    float _slope;
    float _level;
    unsigned long _slopeTime;
    Stopwatch _timer;
    Diagnostic_Parameter _param;
};



#endif //HDVENT_CONTROL_VENTILATION_CONTROLLER_H
