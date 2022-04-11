#ifndef CANLINK_TX_H
#define CANLINK_TX_H

#include <CAN_IDS.h>
#include <stdint.h>

#define TRANSMIT_INTERVAL_MS 100
#define INTERFRAME_PERIOD_US 8000

// List of tracked CAN IDs that we want to transmit over rf
// Ensure there are 15 or less IDs in the list (XBEE max frame size is 256 bytes)
uint32_t trackedCanIds[] =
{
    BCM_STATUS_ID,

    STEERING_WHEEL_ID,

    THERMOCOUPLE1_ID,
    THERMOCOUPLE2_ID,

    ECU1_ID,
    ECU2_ID,
    ECU3_ID,

    LAP_TIME_ID,
    GPS_LAT_ID,
    GPS_LONG_ID,

    DBW_SENSORS_ID,
    BRAKE_PRESSURE_ID,
};

#endif //CANLINK_TX_H
