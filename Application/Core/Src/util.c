#include "util.h"

uint16_t Util_CalculateCRC16(const uint8_t *data, int length)
{
  uint16_t checksum = 0xFFFF;
  for (int i = 0; i < length; i++) {
    uint8_t value = checksum >> 8 ^ data[i];
    value ^= value >> 4;
    checksum = (checksum << 8) ^ (uint16_t)(value << 12) ^ (uint16_t)(value << 5) ^ (uint16_t)value;
  }
  return checksum;
}
