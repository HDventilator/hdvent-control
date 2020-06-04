//
// Created by david on 04.06.20.
//

#ifndef HDVENT_CONTROL_USER_INPUT_H
#define HDVENT_CONTROL_USER_INPUT_H

#include "Display.h"
#include <Arduino.h>
#include <LiquidCrystal.h>
#include "Stopwatch.h"

const unsigned int ALLOWED_USER_EDIT_TIME = 30000; //milliseconds

class User_Input {
public:
    User_Input(User_Parameter *allUserParameters, VentilationMode *mode, bool *do_save);
    enum input_state_t {EDIT_SETTINGS, VIEW_SETTINGS, SAVE_SETTINGS, ENTER_EDIT};
    void update();
private:
    input_state_t _inputState;
    User_Parameter* _allUserParameters;
    VentilationMode* _mode;
    bool* _do_save;
    Stopwatch _stopwatch;

};


#endif //HDVENT_CONTROL_USER_INPUT_H
