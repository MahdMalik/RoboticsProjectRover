#pragma once
#include "Utils.h"

enum class ReceivedPacketType : uint8_t {
    HANDSHAKE = 0x00,
    MOVE_FORWARD = 0x01,
    MOVE_BACKWARD = 0x02,
    TURN_STATIONARY = 0x03,
    STOP = 0x04
};

enum class SentPacketType: uint8_t
{
    HANDSHAKE_RESPONSE = 0x00,
    ACTION_SUCCESS = 0x01,
    ACTION_FAILED = 0x02
};

struct SerialReturnType
{
    bool dataReturned = false;
    ReceivedPacketType messageType = (ReceivedPacketType) 0xFF;
    byte parameterValue = 0xFF;
};

/*
i'm thinking for handling send commands from the jetson to arduino, do something like:

one message type being MOVE_FORWARD, with also the desired angle to move forward towards between -90 and 90.

Then another one like MOVE_BACKWARD, with also the desire dangle to back up at between -90 and 90.

Then one like TURN_STATIONARY, with also the angle to turn towards.

Then finally one like STOP? with no angle so just garbage for that byte value.

Here it looks like i only need messages of 2 bytes?
*/

const uint8_t SENT_MESSAGE_SIZE = 2; 
const uint8_t RECEIVED_MESSAGE_SIZE = 2;

class SerialComm
{
    public:
        void init(unsigned long baudRate);
        byte buffer[RECEIVED_MESSAGE_SIZE];
        uint8_t currentInBuffer = 0;
        SerialReturnType CheckForData();
        void SendData(SentPacketType packetType, byte otherData);
};