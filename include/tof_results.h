#pragma once
#include <stdint.h>
#include <stdbool.h>

#define TOF_MAX_ZONES 1536  // enough for largest mode: 48x32 = 1536 zones

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t distance_mm[TOF_MAX_ZONES];
    uint8_t  confidence[TOF_MAX_ZONES];
    uint16_t num_zones;
    uint8_t  layout_byte;   // copied from frame header — used to compute pixel size
    volatile bool ready;
} TofResults;

extern TofResults tofResults;

#ifdef __cplusplus
}
#endif
