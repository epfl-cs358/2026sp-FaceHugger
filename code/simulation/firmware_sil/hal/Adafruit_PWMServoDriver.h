// firmware_sil/hal/Adafruit_PWMServoDriver.h
//
// Recording host mock for the PCA9685 driver. The real firmware funnels every
// servo write through setPWM(channel, 0, pulse); here we just store the last
// pulse per channel so the Python side can read the electrical-layer values if
// it wants. (The primary readout for the sim is Servo::getServoAngle() via
// SpinalCord::snapshot(), which is the post-clamp servo *degrees* — no PWM
// truncation. This buffer is the alternate, lets-you-verify-the-map seam.)
#pragma once

#include <array>
#include <cstdint>

class Adafruit_PWMServoDriver {
   public:
    Adafruit_PWMServoDriver() = default;
    explicit Adafruit_PWMServoDriver(uint8_t addr) : addr_(addr) {}

    void begin() {}
    void setPWMFreq(float) {}
    void setOscillatorFrequency(uint32_t) {}

    void setPWM(uint8_t channel, uint16_t /*on*/, uint16_t off) {
        if (channel < pulses_.size()) pulses_[channel] = off;
    }

    // SIL read-back: last 'off' pulse count written to a channel.
    uint16_t lastPulse(uint8_t channel) const {
        return channel < pulses_.size() ? pulses_[channel] : 0;
    }

   private:
    uint8_t addr_ = 0x40;
    std::array<uint16_t, 16> pulses_{};
};
