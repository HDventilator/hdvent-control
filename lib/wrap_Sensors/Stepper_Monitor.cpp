//
// Created by david on 17.05.20.
//

#include "Stepper_Monitor.h"

Stepper_Monitor::Stepper_Monitor(powerSTEP *stepper, int homeTolerance) {
    _stepper = stepper;
    _homeTolerance=homeTolerance;

}

bool Stepper_Monitor::readSensor() {
    int status = _stepper->getStatus();
    // TODO get step loss from status register
    setState(OK);

    _data.relativePosition = _stepper->getPos();
    _data.isHome = abs(_data.relativePosition) < _homeTolerance;

    return true;
}

const Stepper_Monitor::sensor_values_t &Stepper_Monitor::getData() const {
    return _data;
}
