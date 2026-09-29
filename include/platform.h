#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int fd;
    uint16_t address;

} VL53L5CX_Platform;

/* -------------------- I2C functions -------------------- */

uint8_t WrMulti(
    VL53L5CX_Platform *p_platform,
    uint16_t reg,
    uint8_t *data,
    uint32_t size);

uint8_t RdMulti(
    VL53L5CX_Platform *p_platform,
    uint16_t reg,
    uint8_t *data,
    uint32_t size);

uint8_t WrByte(
    VL53L5CX_Platform *p_platform,
    uint16_t reg,
    uint8_t value);

uint8_t RdByte(
    VL53L5CX_Platform *p_platform,
    uint16_t reg,
    uint8_t *value);

uint8_t WaitMs(
    VL53L5CX_Platform *p_platform,
    uint32_t ms);

/* -------------------- wrapper macros -------------------- */

#define VL53L5CX_RdMulti RdMulti
#define VL53L5CX_WrMulti WrMulti
#define VL53L5CX_RdByte  RdByte
#define VL53L5CX_WrByte  WrByte
#define VL53L5CX_WaitMs  WaitMs

/* -------------------- utility -------------------- */

void VL53L5CX_SwapBuffer(
    uint8_t *buffer,
    uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_H */