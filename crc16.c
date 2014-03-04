#include "crc16.h"

static const uint16_t crc16CCITTTable[256] = {
#include "crc16CCITTTable_x16_x12_x5_1.txt"
};


uint16_t crc16Compute(const uint8_t *data, uint32_t size)
{
  uint32_t i;
  uint16_t crc = 0xFFFF;
  for (i = 0; i < size; i++)
    crc = crc16CCITTTable[((crc >> 8) ^ data[i])] ^ (crc << 8);
  return crc;
}
