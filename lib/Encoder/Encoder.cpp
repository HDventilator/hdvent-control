//
// Created by david on 23.09.20.
//

#include "Encoder.h"

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