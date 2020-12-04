//
// Created by david on 17.05.20.
//

#include "Stepper_Monitor.h"

Stepper_Monitor::Stepper_Monitor(powerSTEP *stepper, int homeTolerance, uint8_t positiveDir) {
    _stepper = stepper;
    _homeTolerance=homeTolerance;
    _positionSign = (2*positiveDir-1);

}

bool Stepper_Monitor::readSensor() {
    int status = _stepper->getStatus();
    // TODO get step loss from status register
    setState(OK);

    _data.relativePosition = _stepper->getPos()*_positionSign;
    _data.isHome = abs(_data.relativePosition) < _homeTolerance;

    return true;
}

const Stepper_Monitor::sensor_values_t &Stepper_Monitor::getData() const {
    return _data;
}
