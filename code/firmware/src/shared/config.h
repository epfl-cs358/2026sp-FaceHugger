#ifndef CONFIG_H
#define CONFIG_H

// I2C Addresses
#define ADDR_SERVO_DRIVER 0x40
#define ADDR_TOF_FRONT    0x29  // Default VL53L0X

// Pins
#define PIN_SDA 21
#define PIN_SCL 22

//servos work with pulses not angles
#define MIN_PULSE 500
#define MAX_PULSE 2500

#endif