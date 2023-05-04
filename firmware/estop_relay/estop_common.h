#ifndef COMMON_H
#define COMMON_H

#include <stdio.h>
#include <stdint.h>

#define GLOBAL_PASSWORD "CCAKES"

// Message IDs
static const uint8_t MSG_NONE_ID = 99;

static const uint8_t MSG_SIGNAL_ID = 0;
static const uint8_t MSG_SIGNAL_REPLY_ID = 1;

static const uint8_t MSG_INIT_HANDSHAKE_ID = 2;
static const uint8_t MSG_HANDSHAKE_REPLY_ID = 3;

static const uint8_t MSG_INIT_HEARTBEAT_ID = 4;
static const uint8_t MSG_HEARTBEAT_REPLY_ID = 5;

// Signals
static const int NONE_SIGNAL = -1;
static const int ESTOP_SIGNAL = 0;
static const int MOB_STOP_SIGNAL = 1;
static const int MOB_START_SIGNAL = 9;

typedef struct RadioPacket { 
    uint8_t id;
    char password[sizeof(GLOBAL_PASSWORD)];
    uint8_t signal;
} RadioPacket;

#endif