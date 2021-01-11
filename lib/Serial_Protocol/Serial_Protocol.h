//
// Created by david on 06.05.20.
//

#ifndef MON_ARDUINO_SERIAL_SERIAL_PROTOCOL_H
#define MON_ARDUINO_SERIAL_SERIAL_PROTOCOL_H

#include <Arduino.h>
#include <PacketSerial.h>

const uint8_t PACKAGE_SIZE = 14;
const uint8_t IDENTIFIER_PREFIX_LENGTH = 2;
const uint8_t IDENTIFIER_LENGTH = 6;

//! @brief struct with components of single data package
struct __attribute__ ((packed)) package_struct_float_t{
    // if you change identifier length, don't forget to change PACKAGE_SIZE too!!
    char identifier[IDENTIFIER_LENGTH]; // IDENTIFIER_LENGTH bytes
    float value; // 4 bytes
    uint32_t checksum; // 4 bytes
};


//! @brief struct with components of single data package
struct __attribute__ ((packed)) package_struct_4char_t{
    // if you change identifier length, don't forget to change PACKAGE_SIZE too!!
    char identifier[IDENTIFIER_LENGTH]; // IDENTIFIER_LENGTH bytes
    char value[4]; // 4 bytes
    uint32_t checksum; // 4 bytes
};

//! @brief write a packed struct with the PACKAGE_SIZE to the serial port using PacketSerial (COBS)
//! @tparam T
//! @param packetSerial pointer to the PacketSerial instance
//! @param package packed struct to transmit
template <typename T>
void serialWritePackage(PacketSerial *packetSerial, T package) {

    // Use either this to get byte string
    uint8_t tmp[PACKAGE_SIZE];
    memcpy(tmp, &package, PACKAGE_SIZE);
    // or this:
    //const char* tmp = reinterpret_cast<char*>(&package);

    // Write data to serial port
    packetSerial->send(tmp, PACKAGE_SIZE);
}


#endif //MON_ARDUINO_SERIAL_SERIAL_PROTOCOL_H
