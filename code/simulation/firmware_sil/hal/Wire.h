// firmware_sil/hal/Wire.h
//
// Empty host stub for <Wire.h>. The firmware control files #include it but never
// use the I2C bus directly (the PCA9685 driver mock owns the "hardware"). Just
// enough to satisfy the include.
#pragma once

#include <cstdint>

class TwoWire {
   public:
    void begin() {}
    void begin(int, int) {}
    void setClock(uint32_t) {}
};

inline TwoWire Wire;
