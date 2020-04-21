//
// Created by david on 21.04.20.
//

#include "Ventilation_Controller.h"
#include <PID_v1.h>
#include <Stopwatch.h>

VentilationController::VentilationController(VentilationMode mode, double kp, double ki, double kd) : _pid(_pidIn,_pidOut,_pidSetpoint,kp,ki,kd){
    _pidIn =0;
    _pidOut = 0;
    _slope=0;
    _level=0;
    _slopeTime=0;
    _timer = Stopwatch();
_mode = mode;
}

bool VentilationController::expirationTrigger(){
    return anyTrue(_mode.expirationTriggers);
}

bool VentilationController::inspirationTrigger(){
    return anyTrue(_mode.inspirationTriggers);
}

void VentilationController::startRamp(float slope, float level) {
    _slope = slope;
    _level = level;
    _slopeTime = level / slope;
    _timer.start();
}

float VentilationController::calcSetPoint() {
    unsigned long time = _timer.getElapsedTime();
    if (time < _slopeTime){
        return _slope*(float)time/1000;
    }
    else {
        return _level;
    }
}

float VentilationController::calcSpeed(double input) {
    _pidSetpoint = calcSetPoint();
    _pidIn = input;
    _pid.Compute();
    return _pidOut;
}