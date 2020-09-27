//
// Created by david on 26.09.20.
//

#ifndef HDVENT_CONTROL_BUZZER_H
#define HDVENT_CONTROL_BUZZER_H


#include <Arduino.h>
#include <Stopwatch.h>

class Buzzer {
public:
    Buzzer(uint8_t pin, uint16_t onInterval, uint16_t offInterval);
    void service();

    bool isOn() const;

    void turnOn();
    void turnOff();

private:
    uint8_t _pin;
    uint16_t _hiInterval;
    uint16_t _loInterval;

    Stopwatch _stopwatch;
    bool _on;
    bool _isHigh;

};


#endif //HDVENT_CONTROL_BUZZER_H
