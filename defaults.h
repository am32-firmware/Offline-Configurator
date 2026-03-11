#ifndef DEFAULTS_H
#define DEFAULTS_H

#include <QWidget>

uint8_t air_starteeprom[48] ={

    0x01,              // eeprom start byte, must be 1
    0x03,              // eeprom version 0-255
    0x01,              // bootloader version 0-255
    0x01,              // firmware version major
    0x23,              // firmware version minor 50
    0xA0,              // max ramp speed value / 10. percent per millisecond default 160 = 16%/ms
    0x04,              //miniumum duty cycle default value/2  default 4 = 2%
    0x00,              //disable stick based calibration defaut 0
    0x0A,              //absolute voltage cut off default 10
    0x64,              //current PID P value x 2 default 100 = 200
    0x00,              //current control I value default 0
    0x32,              //current control D value x 10 default 50 = 500
    0x02,              // active brake power
    0x30, 0x35, 0x31, 0x20,       // reserved 4 bytes
    0x00,      // direction reversed byte 17
    0x00,      // bidectional mode 1 for on 0 for off byte 18
    0x00,      // sinusoidal startup  byte 19
    0x01,      //complementary pwm byte 20
    0x01,      //variable pwm frequency 21
    0x01,      //stuck rotor protection 22
    0x1A,      // timing advance x0.9375, ei 16 = 15 degrees byte 23
    0x18,      // pwm frequency mutiples of 1k..  byte 24 default 24khz 0x18
    0x64,      // startup power 50-150 percent default 100 percent 0x64 byte 25
    0x37,      // motor KV in increments of 40 default 55 = 2200kv
    0x0e,      // motor poles default 14 byte 27
    0x00,      // brake on stop default 0 byte 28
    0x00,      // anti stall protection, throttle boost at low rpm.
    0x05,      // beep volume     byte 30            range 0 to 11
    // eeprom version 0x01 and later
    0x00,      // 30 Millisecond telemetry output byte 31      0 or 1
    0x80,      //servo low value =  (value*2) + 750us byte 32
    0x80,      //servo high value = (value *2) + 1750us    byte 33
    0x80,      //servo neutral base 1374 + value Us. IE 128 = 1500 us. deafault 128 (0x80)
    0x32,      //servo dead band 0-100, applied to either side of neutral.      byte 35
    0x00,      //low voltage cuttoff                                         byte 36
    0x32,      //low voltage threshold value plus 250 /10 volts. default 50 3.0volts  byte 37
    0x00,      // rc car type reversing, brake on first aplication return to center to reverse default 0  byte 38
    0x00,      // options byte. Hall sensors if equipped  byte 39
    0x0f,      // Sine Mode Range 5-25 percent of throttle default 15   byte 40
    0x0a,      // Drag Brake Strength 1-10 , 10 being full strength, default 10   byte 41
    0x0a,      // amount of brake to use when the motor is running byte 42
    0x8d,      // temperature limit 70-140 degrees C. above 140 disables, default 141. (byte 43)
    0x66,      // current protection level (value x 2) above 100 disables, default 102 (204 degrees) (byte 44)
    0x06,      // sine mode strength 1-10 default 6 byte 45
    //eeprom version 2 or later
    0x00,      // input type selector 1)Auto 2)Dshot only 3)Servo only 4)PWM 5)Serial 6)BetaFlight Safe Arming
    0x00,      // auto_timing
};


uint8_t crawler_starteeprom[48] ={
    0x01,              // eeprom start byte, must be 1
    0x02,              // eeprom version 0-255
    0x01,              // bootloader version 0-255
    0x01,              // firmware version major
    0x23,              // firmware version minor 50
    0x4e, 0x45, 0x4f, 0x45, 0x53, 0x43, 0x20, 0x66, 0x30, 0x35, 0x31, 0x20,       // ESC name, 12 bytes
    0x00,      // direction reversed byte 17
    0x01,      // bidectional mode 1 for on 0 for off byte 18
    0x01,      // sinusoidal startup  byte 19
    0x01,      //complementary pwm byte 20
    0x01,      //variable pwm frequency 21
    0x00,      //stuck rotor protection 22
    0x03,      // timing advance x7.5, ei 2 = 22.5 degrees byte 23
    0x18,      // pwm frequency mutiples of 1k..  byte 24 default 24khz 0x18
    0x68,      // startup power 50-150 percent default 100 percent 0x68 byte 25
    0x30,      // motor KV in increments of 40 default 55 = 2200kv
    0x0e,      // motor poles default 14 byte 27
    0x01,      // brake on stop default 0 byte 28
    0x01,      // anti stall protection, throttle boost at low rpm.
    0x05,      // beep volume     byte 30            range 0 to 11
    // eeprom version 0x01 and later
    0x00,      // 30 Millisecond telemetry output byte 31      0 or 1
    0x80,      //servo low value =  (value*2) + 750us byte 32
    0x80,      //servo high value = (value *2) + 1750us    byte 33
    0x80,      //servo neutral base 1374 + value Us. IE 128 = 1500 us. deafault 128 (0x80)
    0x32,      //servo dead band 0-100, applied to either side of neutral.      byte 35
    0x01,      //low voltage cuttoff                                         byte 36
    0x50,      //low voltage threshold value plus 250 /10 volts. default 80 3.3volts  byte 37
    0x00,      // rc car type reversing, brake on first aplication return to center to reverse default 0  byte 38
    0x00,      // options byte. Hall sensors if equipped  byte 39
    0x0f,      // Sine Mode Range 5-25 percent of throttle default 15   byte 40
    0x0a,      // Drag Brake Strength 1-10 , 10 being full strength, default 10   byte 41
    0x0a,      // amount of brake to use when the motor is running byte 42
    0x65,      // temperature limit 70-140 degrees C. above 140 disables, default 141. (byte 43)
    0x14,      // current protection level (value x 2) above 100 disables, default 20 (40 AMPS) (byte 44)
    0x05,      // sine mode strength 1-10 default 5
    //eeprom version 2 or later
     0x00,      // input type selector 1)Auto 2)Dshot only 3)Servo only 4)PWM 3)Serial
     0x31,  // up to byte 47 for future use
 };










#endif // DEFAULTS_H
