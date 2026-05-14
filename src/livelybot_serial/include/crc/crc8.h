#ifndef LIVELYBOT_SERIAL_CRC_CRC8_H
#define LIVELYBOT_SERIAL_CRC_CRC8_H

#include <cstdint>


uint8_t Get_CRC8_Check_Sum(const uint8_t *message, unsigned int length, uint8_t crc8);


#endif
