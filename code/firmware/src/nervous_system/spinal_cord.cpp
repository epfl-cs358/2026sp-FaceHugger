#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "leg.h"
#include "spinal_cord.h"
#include "../shared/config.h"

//Robot starts in idle state
SpinalCord::SpinalCord(uint8_t pwm):
    robotState(STATE_IDLE),
    driver(Adafruit_PWMServoDriver(ADDR_SERVO_DRIVER)),
    leg1(Leg(driver, 0,0,1,2,90,90,90)),
    leg2(Leg(driver, 1,3,4,5,90,90,90)),
    leg3(Leg(driver, 2,6,7,8,90,90,90)),
    leg4(Leg(driver, 3, 9,10,11, 90,90,90))
{
}

void SpinalCord::walk(){
    robotState = STATE_WALK;
}

void SpinalCord::rest(){
    robotState = STATE_IDLE;
}

void SpinalCord::jump(){
    robotState = STATE_ACTION;
}

void SpinalCord::update(){
    switch(robotState){
        case STATE_WALK:
            break;
        case STATE_ACTION:
            break;
        case STATE_IDLE:
            leg1.returnToDefaultAngles();
            leg2.returnToDefaultAngles();
            leg3.returnToDefaultAngles();
            leg4.returnToDefaultAngles();
            break;
        case STATE_FAILSAFE:
            break;
    //TODO implement the gait pattern and overall functionality of this function
    }
}

