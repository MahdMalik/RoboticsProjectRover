#include "SerialComm.h"

void SerialComm::init(unsigned long theBaudRate)
{
    Serial.begin(theBaudRate);
}

SerialReturnType SerialComm::CheckForData()
{
    while (Serial.available() > 0 && currentInBuffer != RECEIVED_MESSAGE_SIZE)
    {
        buffer[currentInBuffer] = Serial.read();
        currentInBuffer++;
    }

    if(currentInBuffer == RECEIVED_MESSAGE_SIZE)
    {
        SerialReturnType returnedVal;
        returnedVal.dataReturned = true;
        returnedVal.messageType = (ReceivedPacketType) buffer[0];
        returnedVal.parameterValue = buffer[1];
        currentInBuffer = 0;
        return returnedVal;
    }

    return {};
}

void SerialComm::SendData(SentPacketType packetType, byte otherData)
{
    Serial.write((byte) packetType);
    Serial.write(otherData);
}