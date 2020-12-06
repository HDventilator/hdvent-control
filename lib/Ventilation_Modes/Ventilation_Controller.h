//
// Created by david on 21.04.20.
//

#ifndef HDVENT_CONTROL_VENTILATION_CONTROLLER_H
#define HDVENT_CONTROL_VENTILATION_CONTROLLER_H


#include <Stopwatch.h>
#include <Diagnostic_Parameter.h>
#include "Ventilation_Modes.h"
#include "PID_v1.h"
#include <Subset.h>
#include "User_Parameter.h"

typedef float (*input_func_t)();

class VentilationController {
public:
    VentilationController(VentilationMode mode, Diagnostic_Parameter &pressure, Diagnostic_Parameter &flow,
                          User_Parameter* allUserParams);
    bool expirationTrigger();
    bool inspirationTrigger();
    float calcSpeed();
    float calcSetPoint();
    void startRamp(float slope, float level);
    Subset<User_Parameter, UP> userParams;
    VentilationMode _mode;
    float calcSetPointTrapezoid();

    void startTrapezoid(float slope, float level, float time);
private:


    PID _pid;
    bool _bypass;
    double _pidIn;
    double _pidOut;
    double _pidSetpoint;
    input_func_t _getInput;
    float _slope;
    float _level;
    float _slopeTime;
    Stopwatch _timer;
    Diagnostic_Parameter _param;
    float _holdTime;


};



#endif //HDVENT_CONTROL_VENTILATION_CONTROLLER_H
