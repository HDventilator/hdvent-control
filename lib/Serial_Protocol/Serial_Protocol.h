//
// Created by david on 06.05.20.
//

#ifndef MON_ARDUINO_SERIAL_SERIAL_PROTOCOL_H
#define MON_ARDUINO_SERIAL_SERIAL_PROTOCOL_H

#include <Arduino.h>

const uint8_t PACKAGE_SIZE = 14;
const uint8_t IDENTIFIER_LENGTH = 6;
const uint8_t IDENTIFIER_PREFIX_LENGTH = 2;
// struct with components of single data package
struct __attribute__ ((packed)) package_struct_float_t{
    // if you change identifier length, don't forget to change PACKAGE_SIZE too!!
    char identifier[IDENTIFIER_LENGTH]; // IDENTIFIER_LENGTH bytes
    float value; // 4 bytes
    uint32_t checksum; // 4 bytes
};

// struct with components of single data package
struct __attribute__ ((packed)) package_struct_4char_t{
    // if you change identifier length, don't forget to change PACKAGE_SIZE too!!
    char identifier[4]; // IDENTIFIER_LENGTH bytes
    char value[4]; // 4 bytes
    uint32_t checksum; // 4 bytes
};

#endif //MON_ARDUINO_SERIAL_SERIAL_PROTOCOL_H
