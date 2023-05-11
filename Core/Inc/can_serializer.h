#ifndef CANSERIALIZER_H
#define CANSERIALIZER_H

#include "main.h"

// a library independant way of inputting and outputting CAN data.
// input all values other than the checksum.
// use a pointer to an instance of this struct to send data.
// struct is packed to ensure no padding bytes are added by GCC.
typedef struct __attribute__((__packed__))
{
  uint16_t syncWord;        // useful when using non framed transport layer
  uint8_t extended;            // 1 if 29 bit ID, 0 if 11 bit
  uint32_t id;              // CAN frame ID
  uint8_t dlc;              // CAN frame data length code (0 - 8).
  uint8_t data[8];          // CAN frame data array.
  uint8_t checksum;         // DO NOT EDIT. This is filled by fillChecksum()
} CANFrame;


// Calcuates a checksum
uint8_t calculateChecksum(CANFrame *pFrame)
{
    uint8_t computedChecksum = 0;                                           // will hold our checksum as we calculate
    uint8_t bytesToSum = sizeof(*pFrame) - sizeof(pFrame->checksum);        // do not include the checksum when calculating the checksum

    uint8_t *bytePtr = (uint8_t *)pFrame;                                   // get a pointer to struct so we can access individual bytes

    for (int i = 0; i < bytesToSum; i++) {
      computedChecksum += *(bytePtr + i);                                   // dereference and sum each byte
    }

    return computedChecksum;
}

/**
 * @brief adds checksum and syncWord to packet. 
 * @param pFrame: Pointer to a CAN frame.
 * @note: Ensure all other data of frame is filled before calling fillChecksum().
*/
void fillChecksum(CANFrame *pFrame)
{
    pFrame->syncWord = 0xA55A;                                              // set the sync word before summing
    pFrame->checksum = calculateChecksum(pFrame);                           // append the checksum to the packet
}

/**
 * @param pFrame: Pointer to a CAN frame.
 * Returns true if the pFrame struct is valid, false if it is invalid.
*/
uint8_t validateChecksum(CANFrame *pFrame)
{
    return pFrame->checksum == calculateChecksum(pFrame);
}

#endif // CANSERIALIZER_H