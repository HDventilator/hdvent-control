//
// Created by david on 08.07.20.
//

#ifndef HDVENT_CONTROL_AGGREGATED_PARAMETER_H
#define HDVENT_CONTROL_AGGREGATED_PARAMETER_H

#include "Diagnostic_Parameter.h"

class Aggregated_Parameter
        {
public:
    Aggregated_Parameter(int nSamples);

/*Aggregated_Parameter(float initialValue, float minAlarm, float maxAlarm, char *identifier, unsigned  int nSamples);*/

    void enqueue(float item, unsigned long time);
    float sumN(unsigned int n);
    float sumFromTimestamp(unsigned long timestamp);
    float sumFromInterval(unsigned long dt);
private:
    unsigned int _nSamples;
    float* _dataQueue;
    unsigned long* _timeQueue;
    unsigned int _rear;
    unsigned int _front;
    unsigned int _index;

};


#endif //HDVENT_CONTROL_AGGREGATED_PARAMETER_H
