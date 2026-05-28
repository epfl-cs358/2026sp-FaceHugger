#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "../shared/config.h"
#include "servo.h"
#include "motion_math.h"  // easeFraction (smoothstep)

Servo::Servo(Adafruit_PWMServoDriver &pwm, uint8_t pcaChannel, uint16_t servoDefaultAngle)
    : pwm(pwm),
      pcaChannel(pcaChannel),
      servoAngle(servoDefaultAngle),
      servoDefaultAngle(servoDefaultAngle){
    }

void Servo::applyAngle(double angle){
    // Electrical backstop. Every motion path (IK, pose, calibrate, clip player)
    // funnels through here, and map() does NOT clamp — an out-of-range angle would drive
    // the servo past MIN/MAX_PULSE. constrain here protects them all at one chokepoint.
    double clamped = constrain(angle, 0.0, 180.0);
    if (clamped != angle) {
        // Out-of-range request (e.g. a direct calibrate). Same [OOR] line as the
        // gait/clip guard in Leg::setJointAngles so one report format covers all
        // paths (serial monitor on hardware; captured by the sim).
        Serial.printf("[OOR] servo %d requested %.2f\n", this->pcaChannel, angle);
    }
    uint16_t pulse = map(clamped, 0, 180, MIN_PULSE, MAX_PULSE);
    pwm.setPWM(this->pcaChannel, 0, pulse);
    this->servoAngle = clamped;
}

void Servo::setServoAngle(double angle){
    // A direct write supersedes any in-progress ease, so cancel it. (tickEase
    // uses applyAngle() instead, so advancing an ease doesn't cancel itself.)
    applyAngle(angle);
    easeActive = false;
}

void Servo::setServoAngleTimed(double angle, uint32_t ms) {
    double target = constrain(angle, 0.0, 180.0);
    if (ms == 0) { easeActive = false; setServoAngle(target); return; }
    easeStartAngle  = (double)servoAngle;  // ease from where we are now
    easeTargetAngle = target;
    easeStartMs     = millis();
    easeDurMs       = ms;
    easeActive      = true;
}

void Servo::tickEase() {
    if (!easeActive) return;
    uint32_t t = millis() - easeStartMs;
    if (t >= easeDurMs) {
        applyAngle(easeTargetAngle);
        easeActive = false;
        return;
    }
    double frac = easeFraction(t, easeDurMs);  // smoothstep ease-in-out
    applyAngle(easeStartAngle + (easeTargetAngle - easeStartAngle) * frac);
}

bool Servo::easing() const { return easeActive; }

void Servo::returnToDefaultAngleTimed(uint32_t ms) {
    setServoAngleTimed((double)servoDefaultAngle, ms);
}

uint8_t Servo::getChannel() const {
    return pcaChannel;
}

void Servo::returnToDefaultAngle(){
    setServoAngle(this->servoDefaultAngle);
}

double Servo::getServoAngle() const {
    // Whole-degree precision: setServoAngle() truncates double → uint16_t storage.
    return (double)servoAngle;
}

