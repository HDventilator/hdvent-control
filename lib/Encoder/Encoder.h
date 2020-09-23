//
// Created by david on 23.09.20.
//

#ifndef HDVENT_CONTROL_ENCODER_H
#define HDVENT_CONTROL_ENCODER_H

#include <Stopwatch.h>

struct Encoder{
    bool wasTurned=false;
    bool shortPressDetected=false;
    bool sense=false;
    bool wasPressed=true;
    int _increment=0;
    Stopwatch pressingTime;
    bool longPressDetected=false;
    void incrementEncoder();

};


#endif //HDVENT_CONTROL_ENCODER_H
