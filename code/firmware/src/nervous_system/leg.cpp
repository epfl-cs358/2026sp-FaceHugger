#include "leg.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "../shared/config.h"

Leg::Leg(Adafruit_PWMServoDriver &pwm,
         int legID,
         uint8_t hipPCAChannel,
         uint8_t thighPCAChannel,
         uint8_t kneePCAChannel,
         uint16_t hipDefaultAngle,
         uint16_t thighDefaultAngle,
         uint16_t kneeDefaultAngle)
    : pwm(pwm),   
      id(legID),
      hipPCAChannel(hipPCAChannel),
      thighPCAChannel(thighPCAChannel),
      kneePCAChannel(kneePCAChannel),
      hipDefaultAngle(hipDefaultAngle),
      thighDefaultAngle(thighDefaultAngle),
      kneeDefaultAngle(kneeDefaultAngle),
      hipAngle(hipDefaultAngle),
      thighAngle(thighDefaultAngle),
      kneeAngle(kneeDefaultAngle)
{
    updateServos();
}

void Leg::setPose(float x, float y, float z) {
    Serial.print("Leg ");
    Serial.print(id);
    Serial.printf(" moving to target: %.1f, %.1f, %.1f\n", x, y, z);
    updateServos();
}

void Leg::updateServos(){
    setServoAngle(hipPCAChannel, hipAngle);
    setServoAngle(thighPCAChannel, thighAngle);
    setServoAngle(kneePCAChannel, kneeAngle);
}

void Leg::returnToDefaultAngles(){
    hipAngle = hipDefaultAngle;
    thighAngle = thighDefaultAngle;
    kneeAngle = kneeDefaultAngle;
    updateServos();
}

void Leg::setServoAngle(uint8_t channel, double angle){
    uint16_t pulse = map(angle, 0, 270, MIN_PULSE, MAX_PULSE);
    pwm.setPWM(channel, 0, pulse);
}