#ifndef LIVELYBOT_SERIAL_CRC_CRC16_H
#define LIVELYBOT_SERIAL_CRC_CRC16_H

#include <cstdint>

uint16_t crc_ccitt(uint16_t crc, const uint8_t *buffer, uint16_t len);

#endif
