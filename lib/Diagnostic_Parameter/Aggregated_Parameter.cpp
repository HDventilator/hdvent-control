//
// Created by david on 08.07.20.
//

#include <string.h>
#include "Aggregated_Parameter.h"
/*
Aggregated_Parameter::Aggregated_Parameter(float initialValue, float minAlarm, float maxAlarm, char *identifier,
                                           unsigned int nSamples)
        : Diagnostic_Parameter(initialValue, minAlarm, 0) {
    _nSamples = nSamples;
    _dataQueue = new float[nSamples];
    memset(_dataQueue, 0, nSamples);
    _timeQueue = new unsigned long[nSamples];
    memset(_timeQueue, 0, nSamples);
    _rear = _nSamples;
    _front = 0;
    _index = 0;
}
*/
void Aggregated_Parameter::enqueue(float item, unsigned long time)
{
    _index = (_index+1) % _nSamples;
    _dataQueue[_index] = item;
    _timeQueue[_index] = time;
    Serial.print("enqueued:"); Serial.println(item);
}

float Aggregated_Parameter::sumN(unsigned int n) {
    float sum =0;
    for (unsigned int i=0; i<n; i++){
        Serial.print('i');Serial.println(i);
        unsigned int ix = (_index-i)%_nSamples;
        sum = sum + _dataQueue[ix];
    }
//setValue(sum);
    return sum;
}

float Aggregated_Parameter::sumFromTimestamp(unsigned long timestamp) {
    float sum =0;
    for (unsigned int i=0; (i<_nSamples)&&(_timeQueue[(_index-i)%_nSamples] > timestamp); i++){
        unsigned int ix = (_index-i)%_nSamples;
        Serial.print("ix:");Serial.println(ix);
        sum = sum + _dataQueue[ix];
    }
  //  setValue(sum);
    return sum;
}

Aggregated_Parameter::Aggregated_Parameter(int nSamples)
    {
    _nSamples = nSamples;
    _dataQueue = new float[nSamples];
    memset(_dataQueue, 0, nSamples);
    _timeQueue = new unsigned long[nSamples];
    memset(_timeQueue, 0, nSamples);
    _rear = _nSamples;
    _front = 0;
    _index = 0;
}
