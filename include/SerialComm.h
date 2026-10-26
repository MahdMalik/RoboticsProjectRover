#pragma once
#include "Utils.h"

enum PacketType : uint8_t {
    SAY_HELLO = 0x00
};

const uint8_t MAX_PAYLOAD = 64;

class SerialComm
{
    public:
        SerialComm(int baudRate);
        const byte buffer[MAX_PAYLOAD];
        uint8_t bufferSize;
};