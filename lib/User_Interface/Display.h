//
// Created by david on 24.04.20.
//

#ifndef HDVENT_CONTROL_DISPLAY_H
#define HDVENT_CONTROL_DISPLAY_H

#include <LiquidCrystal.h>
#include <User_Parameter.h>
#include <Ventilation_Modes.h>
/*
float roundNumber(float a, uint8_t n){
    static int pow10[5] = {1, 10, 100, 1000, 10000};
    a = a*pow10[n];
    a = round(a);
    a = a/pow10[n];
    return a;
}
*/

class Display {
public:
    Display(LiquidCrystal &lcd, User_Parameter* allUserParameters, const VentilationMode* mode, int* cursorIncrementer, int* valueIncrementer, bool* toggleEditState, bool* toggleMenuState);
    enum editState_t {EDIT_ENTRY, NAVIGATE, VIEW_ONLY};
    enum menuState_t {EDIT_SETTINGS, VIEW};

    void updateDisplay();
    void printParameterValue();
    void displayParameterValue();

    uint8_t getParameterIndex();
    void safeParams();
    void loadParams();
    void printStaticText();


    float _parametersMemory[4];

    uint8_t _valueColumnPos=5;
    const VentilationMode *_mode;
    uint8_t columnValues;
    uint8_t _activeParamIndex;
    editState_t _editState;
    menuState_t _menuState;
     uint8_t  _markerPosition;
    int *markerIncrementer;
    int *_valueIncrementer;
    bool *_toggleEditState;
    bool *_toggleMenuState;
    uint8_t  _markerPositionMax;
   uint8_t _markerPositionMin;
   void moveMarker();
   LiquidCrystal _lcd;
   User_Parameter* _allUserParameters;



};


#endif //HDVENT_CONTROL_DISPLAY_H
