#include "platform.h"

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <cstring>
#include <iostream>
#include <stdint.h>

using namespace std;

// ======================================================
// Write Multiple Bytes
// ======================================================

extern "C" void VL53L5CX_SwapBuffer(
    uint8_t *buffer,
    uint16_t size)
{
    for (uint16_t i = 0; i < size; i += 4)
    {
        uint8_t tmp = buffer[i];
        buffer[i] = buffer[i + 3];
        buffer[i + 3] = tmp;

        tmp = buffer[i + 1];
        buffer[i + 1] = buffer[i + 2];
        buffer[i + 2] = tmp;
    }
}

extern "C" uint8_t WrMulti(
    VL53L5CX_Platform *p_platform,
    uint16_t reg,
    uint8_t *data,
    uint32_t size)
{
    const uint32_t CHUNK_SIZE = 1024;

    uint32_t offset = 0;

    while (offset < size)
    {
        uint32_t chunk =
            (size - offset > CHUNK_SIZE)
            ? CHUNK_SIZE
            : (size - offset);

        uint8_t buffer[CHUNK_SIZE + 2];

        uint16_t current_reg = reg + offset;

        buffer[0] = (current_reg >> 8) & 0xFF;
        buffer[1] = current_reg & 0xFF;

        memcpy(
            &buffer[2],
            &data[offset],
            chunk);

        if (write(
                p_platform->fd,
                buffer,
                chunk + 2)
            < 0)
        {
            return 1;
        }

        offset += chunk;
    }

    return 0;
}

// ======================================================
// Read Multiple Bytes
// ======================================================

extern "C" uint8_t RdMulti(
    VL53L5CX_Platform *p_platform,
    uint16_t reg,
    uint8_t *data,
    uint32_t size)
{
    uint8_t addr[2];

    addr[0] = (reg >> 8) & 0xFF;
    addr[1] = reg & 0xFF;

    if (write(
            p_platform->fd,
            addr,
            2)
        < 0)
    {
        return 1;
    }

    uint32_t offset = 0;

    const uint32_t CHUNK_SIZE = 1024;

    while (offset < size)
    {
        uint32_t chunk =
            (size - offset > CHUNK_SIZE)
            ? CHUNK_SIZE
            : (size - offset);

        if (read(
                p_platform->fd,
                &data[offset],
                chunk)
            < 0)
        {
            return 1;
        }

        offset += chunk;
    }

    return 0;
}

// ======================================================
// Write Byte
// ======================================================

extern "C" uint8_t WrByte(
    VL53L5CX_Platform *p_platform,
    uint16_t reg,
    uint8_t value)
{
    return WrMulti(
        p_platform,
        reg,
        &value,
        1);
}

// ======================================================
// Read Byte
// ======================================================

extern "C" uint8_t RdByte(
    VL53L5CX_Platform *p_platform,
    uint16_t reg,
    uint8_t *value)
{
    return RdMulti(
        p_platform,
        reg,
        value,
        1);
}

// ======================================================
// Delay
// ======================================================

extern "C" uint8_t WaitMs(
    VL53L5CX_Platform *p_platform,
    uint32_t ms)
{
    usleep(ms * 1000);

    return 0;
}
