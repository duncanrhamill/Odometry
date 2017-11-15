/*
 *  Defines for odometry task
 */

// include guard
#ifndef DEFINES_H
#define DEFINES_H

// constant definitions
#define MAXLOOPCOUNT 5          // maximum times to loop while correcting steer/drive
#define WHEELDIST 125           // distance between centre of robot and centre of wheels
#define PIEZOFREQ 1000          // frequency to sound the buzzer at
#define NOTIFYPAUSE 200         // time to sound buzzer and flash light if not dropping M&M
#define SERVOINIT 0             // initial angle for servo to sit at (the empty hole)
#define SERVOSTEP 34            // angle to rotate servo by in order to move to next hole
#define SERVOPAUSE 400          // time to wait to ensure M&M drops cleanly
#define DUALSPEED 50            // speed of the motors in dual mode
#define FORWARD 1               // multiplier to move forward
#define BACKWARD -1             // backwards multiplier
#define LINEARTOL 1             // linear tolerance for accuracy in straight line
#define ANGULARTOL 0.75
#define ARCTOL 0.2
#define CLICKSTOMM 0.890        // conversion factor from clicks to mm

// MD25 I2C codes
#define MD25ADDR 0x58           // I2C MD25 address
#define SPEEDLEFT 0x00          // MD25 register for speed #1 (left)
#define SPEEDRIGHT 0x01         //   "      "       "      #2 (right)
#define ENCODELEFT 0x02         // encoder address left
#define ENCODERIGHT 0x06        // "        "      right
#define ACCEL 0x0E              // Acceleration encoder
#define MODE 0x0F               // mode register
#define CMD 0x10                // command register

// MD25 command codes
#define CLEARENCODERREGISTERS 0x20  // code to clear encoder values

// MD25 acceleration modes
#define ACCELDEFAULT 2          // acceleration mode

// MD25 modes
#define MODEUNSIGNEDDUAL 2
#define MODEDUAL 3              // dual motor mode, all off speed 1
#define MODESEPERATE 0          // seperate motor speeds

// pin definitions
#define LEDPIN 8                // led pin
#define PIEZOPIN 9              // buzzer spin
#define SERVOPIN 10             // servo pin

#endif
