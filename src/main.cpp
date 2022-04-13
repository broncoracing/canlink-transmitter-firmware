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
map <uint32_t, CANFrame> canData;

// Timer for spacing out rf transmissions
Timer timer;

// All incoming CAN frames go here
CANMessage inmsg;

// Buffer for holding multiple frames
// Statically sized for the amount of data in the trackedCanIds array
uint8_t packetBuf[sizeof(CANFrame) *
                  (sizeof(trackedCanIds) / sizeof(trackedCanIds[0]))];

// Outgoing can messages go here
CANMessage outmsg;

char xbeeBuf = 0;       // UART bytes go here
Queue<char, 512> xbeeQueue;

// holds trial CAN frame before it is converted to an Mbed frame
uint8_t frameBuf[sizeof(CANFrame)] = {0};


// Prototypes
void xbeeISR();

void buildFrame(CANFrame *tempFrame);


int main(void) {
    initBoard();
    initCanMap();

    timer.reset();
    timer.start();


    xbee.attach(xbeeISR, SerialBase::RxIrq); // dynamically set receive interrupt for XBEE transceiver


    while (1) {
        // TRANSMIT FUNCTIONALITY
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
            for (const auto &[key, value]: canData) {
                // copy each CANFrame struct to the output buffer
                memcpy(packetBuf + i, &value, sizeof(value));
                // offset output buffer destination by the size of the frame we just
                // added
                i += sizeof(CANFrame);
            }
            xbee.write(packetBuf, sizeof(packetBuf));

            led.write(0);
        }

        // RECEIVE FUNCTIONALITY
        if (xbeeQueue.full())                                               // if we fall this far behind there is an issue
        {
            char *dummy_out;
            while(!xbeeQueue.empty()) {
                xbeeQueue.try_get(&dummy_out);
            }                                                 // wipe the buffer and attempt to recover
        }

        while (!xbeeQueue.empty())                       // make sure we have enough data to build a packet
        {
            CANFrame tempFrame = {0};                                       // empty trial frame

            for (int i = 0; i < sizeof(CANFrame) - 1; i++)
            {
                frameBuf[i] = frameBuf[i + 1];
            }

            char *last; // new last byte for frame buf
            if(xbeeQueue.try_get(&last)) {                                     // pull the latest value off the queue
                frameBuf[sizeof(frameBuf) - 1] = *last;
                memcpy(&tempFrame, frameBuf, sizeof(CANFrame));                 // build trial frame
                buildFrame(&tempFrame);                                         // build output frame if the checksum is valid
            } else {
                break;
            }

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


void xbeeISR()
{
    xbee.read((void*)&xbeeBuf, sizeof(xbeeBuf));
    xbeeQueue.try_put(&xbeeBuf);
}


void buildFrame(CANFrame *tempFrame)
{
    if (tempFrame->syncWord == 0xA55A && validateChecksum(tempFrame))       // got a valid frame
    {
        outmsg.id = tempFrame->id;                                          // build an Mbed CAN frame and send it
        outmsg.len = tempFrame->dlc;
        memcpy(outmsg.data, tempFrame->data, tempFrame->dlc);

        if (tempFrame->extended)
        {
            outmsg.format = CANExtended;
        }
        else
        {
            outmsg.format = CANStandard;
        }

        can.write(outmsg);
    }
}
