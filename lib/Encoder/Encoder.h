//
// Created by david on 23.09.20.
//

#ifndef HDVENT_CONTROL_ENCODER_H
#define HDVENT_CONTROL_ENCODER_H

#include <Stopwatch.h>

class Encoder{
public:
    Encoder(int pinA, int pinB);
    bool wasTurned=false;
    bool shortPressDetected=false;
    bool sense=false;
    bool wasPressed=true;
    Stopwatch pressingTime;
    bool longPressDetected=false;
    void incrementEncoder();
    void service();
    void reset();
    int getPosition() const;
    int8_t getDelta();
    int _position;

private:
    int _oldPosition;
    int _pinA;
    int _pinB;
    int8_t _delta;
    bool _aLast;
    bool _bLast;

};

#endif //HDVENT_CONTROL_ENCODER_H
