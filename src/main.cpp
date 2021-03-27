// TRANSMITTER

#include <canLink_TX.h>
#include <canSerializer.h>
#include <mbed.h>

#include <cstring>
#include <map>

#include "board.h"

// External variables
extern CAN can;
extern UnbufferedSerial xbee;
extern DigitalOut led;

// Prototypes
void initCanMap();
void updateCanMap(CANMessage *msg);

// Main data store
map<uint32_t, CANFrame> canData;

// Timer for spacing out rf transmissions
Timer timer;

// All incoming CAN frames go here
CANMessage inmsg;

// Buffer for holding multiple frames
// Statically sized for the amount of data in the trackedCanIds array
uint8_t packetBuf[sizeof(CANFrame) *
                  (sizeof(trackedCanIds) / sizeof(trackedCanIds[0]))];

int main(void) {
  initBoard();
  initCanMap();

  timer.reset();
  timer.start();

  while (1) {
    // if a message is available, log it.
    if (can.read(inmsg)) {
      updateCanMap(&inmsg);
    }

    // time to transmit data
    if (timer.read_ms() >= TRANSMIT_INTERVAL_MS) {
      timer.reset();

      // light LED while transmitting a packet
      led.write(1);

      // iterate through entire map
      uint8_t i = 0;
      for (const auto &[key, value] : canData) {
        // copy each CANFrame struct to the output buffer
        memcpy(packetBuf + i, &value, sizeof(value));
        // offset output buffer destination by the size of the frame we just
        // added
        i += sizeof(CANFrame);
      }
      xbee.write(packetBuf, sizeof(packetBuf));

      led.write(0);
    }
  }
}

// Initialize with the values that we care about
void initCanMap() {
  // create a valid dummy frame (this way RX LED lights up even when car is off)
  CANFrame dummyFrame = {0};
  dummyFrame.syncWord = 0xA55A;
  fillChecksum(&dummyFrame);

  // make volatile for debugger viewing
  volatile int numElements = sizeof(trackedCanIds) / sizeof(trackedCanIds[0]);
  for (int i = 0; i < numElements; i++) {
    canData[trackedCanIds[i]] = dummyFrame;
  }
}

// Takes an MBed style CAN message and updates the map if we care about the ID
void updateCanMap(CANMessage *msg) {
  CANFrame tempFrame = {0};

  // Don't add any new values that weren't in the map when we initialized it
  if (canData.find(uint32_t(msg->id)) != canData.end()) {
    // Copy data to serializable frame format
    if (msg->format == CANExtended) {
      tempFrame.extended = true;
    } else {
      tempFrame.extended = false;
    }

    tempFrame.id = msg->id;
    tempFrame.dlc = msg->len;
    memcpy(tempFrame.data, msg->data, msg->len);

    fillChecksum(&tempFrame);
    canData[tempFrame.id] = tempFrame;
  }
}
