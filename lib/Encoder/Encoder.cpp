//
// Created by david on 23.09.20.
//

#include "Encoder.h"
Encoder::Encoder(int pinA, int pinB){
    _pinA = pinA;
    _pinB = pinB;
}
/*
void Encoder::incrementEncoder(){
    if (wasTurned){
        if (sense){
            _increment++;
        }
        if (!sense){
            _increment--;
        }
    }
    wasTurned = false;
}
*/
void Encoder::service() {
    bool a = digitalRead(_pinA);
    bool b = digitalRead(_pinB);

    //int A =0;
    int B=0;

    /*
    if (a==_aLast){
        A = 0;
    }
    else if (a<_aLast){
        A = -1;
    }
    else{
        A = 1;
    }
     */

    if (b==_bLast){
        B = 0;
    }
    else if (b<_bLast){
        B = -1;
    }
    else{
        B = 1;
    }

    if (a == HIGH) {
        _position += B;
        _delta = B;
    }
    else if (a == LOW) {
        _position -= B;
        _delta = -B;
    }

   /* if (b == HIGH) {
        _position -= A;
    }
    else if (b == LOW) {
        _position += A;
    }*/

    _aLast = a;
    _bLast = b;
}

int Encoder::getPosition() const {
    return _position;
}

void Encoder::reset() {
    _position=0;
}

int8_t Encoder::getDelta() {
    service();
    return _delta;
}
