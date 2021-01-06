#ifndef CANLINK_TX_H
#define CANLINK_TX_H

#include <BR_CAN_IDs.h>
#include <stdint.h>

#define TRANSMIT_INTERVAL_MS 100
#define INTERFRAME_PERIOD_US 8000

// List of tracked CAN IDs that we want to transmit over rf
// Ensure there are 15 or less IDs in the list (XBEE max frame size is 256 bytes)
uint32_t trackedCanIds[] =
{
    BCM_STATUS_ID,

    STEERING_WHEEL_ID,

    THERMOCOUPLE_1_ID,
    THERMOCOUPLE_2_ID,

    PE1_ID,
    PE6_ID,

    LAP_TIME_ID,
    GPS_LOCATION_ID
};

#endif //CANLINK_TX_H
