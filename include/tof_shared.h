#ifndef TOF_SHARED_H
#define TOF_SHARED_H

#include <stdint.h>

struct ToFData
{
    uint64_t timestamp_us;
    uint16_t distance_mm[64];
};

#endif