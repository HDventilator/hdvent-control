//
// Created by david on 21.04.20.
//

#include "Ventilation_Controller.h"

VentilationController::VentilationController(VentilationMode mode, Diagnostic_Parameter &pressure,
                                             Diagnostic_Parameter &flow,
                                             User_Parameter* allUserParams)
: _pid(&_pidIn, &_pidOut, &_pidSetpoint, mode.pidParameters.k_p, mode.pidParameters.k_i, mode.pidParameters.k_d, DIRECT),
userParams (allUserParams, mode.parameters),
_mode(mode)
{
    switch (_mode.controlMode){
        case ControlMode::PC:
            _param = pressure;
            _bypass = false;
            break;
        case ControlMode::VC:
            _param = flow;
            _bypass = false;
            break;
        case ControlMode::VN:
            _param = pressure;
            _bypass = true;
            break;
    }
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

void VentilationController::startTrapezoid(float slope, float level, float time) {
    _slope = slope;
    _level = level;
    _slopeTime = level / slope;
    _holdTime = time-2*_slopeTime;
    _timer.start();
}


float VentilationController::calcSetPoint() {
    float time = (float)_timer.getElapsedTime()/1000;
    if (time < _slopeTime){
        return _slope*(float)time;
    }
    else {
        return _level;
    }
}

float VentilationController::calcSetPointTrapezoid() {
    float time = (float)_timer.getElapsedTime()/1000;
    Serial.print("time: ");Serial.println(time);
    if (time < _slopeTime){
        return _slope*(float)time;
    }
    else if (time < (_slopeTime + _holdTime)){
        return _level;
    }
    else {
        return _level-_slope*(float)(time-_holdTime-_slopeTime);
    }
}

float VentilationController::calcSpeed() {
    _pidSetpoint = calcSetPointTrapezoid();
    _pidIn = _param.getValue();
    _pid.Compute();
    if (_bypass){
        return _pidSetpoint;
    }
    else {
        return _pidOut;
    }
}