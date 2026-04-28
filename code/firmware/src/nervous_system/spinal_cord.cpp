#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "leg.h"
#include "spinal_cord.h"
#include "servo.h"
#include "../shared/config.h"

//Robot starts in idle state
SpinalCord::SpinalCord(uint8_t pwm):
    robotState(STATE_IDLE),
    driver(Adafruit_PWMServoDriver(ADDR_SERVO_DRIVER)),
    leg1(Leg(driver, 0,
        Servo(driver, FRONT_RIGHT_LEG_HIP_PCA_CHANNEL, FRONT_RIGHT_LEG_HIP_DEFAULT_ANGLE),
        Servo(driver, FRONT_RIGHT_LEG_THIGH_PCA_CHANNEL, FRONT_RIGHT_LEG_THIGH_DEFAULT_ANGLE),
        Servo(driver, FRONT_RIGHT_LEG_KNEE_PCA_CHANNEL, FRONT_RIGHT_LEG_KNEE_DEFAULT_ANGLE)
    )),
    leg2(Leg(driver, 1,
        Servo(driver, FRONT_LEFT_LEG_HIP_PCA_CHANNEL, FRONT_LEFT_LEG_HIP_DEFAULT_ANGLE),
        Servo(driver, FRONT_LEFT_LEG_THIGH_PCA_CHANNEL, FRONT_LEFT_LEG_THIGH_DEFAULT_ANGLE),
        Servo(driver, FRONT_LEFT_LEG_KNEE_PCA_CHANNEL, FRONT_LEFT_LEG_KNEE_DEFAULT_ANGLE)
        )),
    leg3(Leg(driver, 2,
        Servo(driver, BOTTOM_RIGHT_LEG_HIP_PCA_CHANNEL, BOTTOM_RIGHT_LEG_HIP_DEFAULT_ANGLE),
        Servo(driver, BOTTOM_RIGHT_LEG_THIGH_PCA_CHANNEL, BOTTOM_RIGHT_LEG_THIGH_DEFAULT_ANGLE),
        Servo(driver, BOTTOM_RIGHT_LEG_KNEE_PCA_CHANNEL, BOTTOM_RIGHT_LEG_KNEE_DEFAULT_ANGLE)
        )),
    leg4(Leg(driver, 3,
        Servo(driver, BOTTOM_LEFT_LEG_HIP_PCA_CHANNEL, BOTTOM_LEFT_LEG_HIP_DEFAULT_ANGLE),
        Servo(driver, BOTTOM_LEFT_LEG_THIGH_PCA_CHANNEL, BOTTOM_LEFT_LEG_THIGH_DEFAULT_ANGLE),
        Servo(driver, BOTTOM_LEFT_LEG_KNEE_PCA_CHANNEL, BOTTOM_LEFT_LEG_KNEE_DEFAULT_ANGLE)
        ))
{
}

void SpinalCord::begin() {
    driver.begin();
    driver.setPWMFreq(60); // Standard for servos
    
    // Now that the driver is alive, set the initial pose
    leg1.returnToDefaultAngles();
    leg2.returnToDefaultAngles();
    leg3.returnToDefaultAngles();
    leg4.returnToDefaultAngles();
}

void SpinalCord::walk(){
    robotState = STATE_WALK;
}

void SpinalCord::rest(){
    robotState = STATE_IDLE;
}

void SpinalCord::wallFlip(){
    robotState = STATE_ACTION;
}

void SpinalCord::applyCalibration(int channel, int angle) {
    // We call identifyAndMove on every leg. 
    // Leg class logic ensures only the correct leg reacts.
    leg1.identifyAndMove(channel, (double)angle);
    leg2.identifyAndMove(channel, (double)angle);
    leg3.identifyAndMove(channel, (double)angle);
    leg4.identifyAndMove(channel, (double)angle);
}

void SpinalCord::update(){
    switch(robotState){
        case STATE_WALK:
            break;
        case STATE_ACTION:
            break;
        case STATE_IDLE:
            break;
        case STATE_FAILSAFE:
            leg1.returnToDefaultAngles();
            leg2.returnToDefaultAngles();
            leg3.returnToDefaultAngles();
            leg4.returnToDefaultAngles();
            break;
    //TODO implement the gait pattern and overall functionality of this function
    }
}

