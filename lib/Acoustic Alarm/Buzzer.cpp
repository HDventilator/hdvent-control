//
// Created by david on 26.09.20.
//

#include "Buzzer.h"

Buzzer::Buzzer(uint8_t pin, uint16_t onInterval, uint16_t offInterval) : _pin(pin), _hiInterval(
        onInterval), _loInterval(offInterval) {
    _isHigh = false;
    _on = false;
}

void Buzzer::service() {
    if (_on){
        if (_isHigh) {
            if (_stopwatch.getElapsedTime() > _hiInterval) {
                noTone(_pin);
                _stopwatch.start();
                _isHigh = false;
            }
        }
        else if ((_stopwatch.getElapsedTime() > _loInterval)){
            //tone(_pin, _frequency);
            digitalWrite(_pin, HIGH);
            _stopwatch.start();
            _isHigh = true;
        }

    }
}

bool Buzzer::isOn() const {
    return _on;
}

void Buzzer::turnOn() {
    //tone(_pin, _frequency);
    digitalWrite(_pin, HIGH);
    _stopwatch.start();
    _isHigh = true;
    _on = true;
}
void Buzzer::turnOff() {
    _on = false;
}

