#pragma once

#include <LoRaMac.hpp>

class LoRaMacFragmentFrame : public LoRaMacFrame {
public:
  LoRaMacFragmentFrame(const uint8_t *id, uint16_t offset, uint8_t len, const uint8_t *src = nullptr) :
  LoRaMacFrame(4 + 2 + len) {
    if (this->buf) {
      this->buf[0] = id[0];
      this->buf[1] = id[1];
      this->buf[2] = id[2];
      this->buf[3] = id[3];
      this->buf[4] = (offset >> 0) & 0xFF;
      this->buf[5] = (offset >> 8) & 0xFF;
      if (src) {
        memcpy(&this->buf[6], src, len);
      }
    }
  }

  void setBuffer(uint8_t offset, const uint8_t *src, uint8_t len) {
    memcpy(&this->buf[6 + offset], src, len);
  }
};
