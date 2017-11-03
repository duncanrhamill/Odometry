/*
 *      Odometry Task - Aero 2 Group 2
 * 
 *          Code written by Duncan R Hamill - 28262174
 */

// includes
#include <Wire.h>
#include <Servo.h>

// constant definitions
#define MAXLOOPCOUNT 5
#define WHEELDIST 40
#define FULLCIRCLE 360
#define PI 3.14159
#define PIEZOFREQ 1000
#define NOTIFYPAUSE 200
#define SERVOINIT 15
#define SERVOSTEP 25
#define SERVOPAUSE 200

// MD25 I2C codes
#define SPEED1 0x00
#define SPEED2 0x01
#define ENCODE1A 0x02
#define ENCODE2A 0x06
#define VOLTS 0x0A
#define MOTOR1I 0x0B
#define MOTOR2I 0x0C
#define ACCEL 0x0E
#define MODE 0x0F
#define CMD 0x10

// pin definitions
#define LEDPIN 8
#define PIEZOPIN 9
#define SERVOPIN 10

Servo servo;
int ServoPosition;


/*
 *  Base leg class, contains run method that is implamented by the Line and Cirlce classes
 */
class Leg
{
  public:
    // Should we drop an M&M?, leg finished successfully?
    bool drop, success;
    
    // Virtual function that will be called to run this leg of the course.
    virtual void run();

    // turn on the LED and sound the buzzer
    void notifyOn() {
        digitalWrite(LEDPIN, HIGH);
        tone(PIEZOPIN, PIEZOFREQ);
    }

    // turn off LED and sound buzzer
    void notifyOff() {
        digitalWrite(LEDPIN, LOW);
        noTone(PIEZOPIN);
    }

    // dispense an M&M
    void dispense() {
        ServoPosition += SERVOSTEP;
        servo.write(ServoPosition);
        delay(SERVOPAUSE);
    }

    // rotate by the given angle (+ve clockwise), returning the actual angle rotated
    int rotate(int d) {
        return d;
    }

    int distanceLeft() {
        
    }
};


/*
 *  Line class, for driving the robot down a straight line.
 */
class Line: public Leg {
    // count how many times we loop over the drive sections, so we don't get stuck.
    int loopCount;
  public:
    // distance to travel, and how far to rotate to be pointing in correct direction at end of the leg
    int dist, endRot;

    // constructor
    Line(int d, int r, bool m) {
        this->dist = d;
        this->endRot = r;
        this->drop = m;
        this->loopCount = 0;
    }

    // implement the run function
    void run() {
        // run drive, get how far we actually drove
        int driven = this->drive(this->dist);

        // calculate distance left to drive
        int shortfall = this->dist - driven;

        // aim to get within 2mm of the target waypoint, without going over MAXLOOPCOUNT
        while (abs(shortfall) > 2 && loopCount < MAXLOOPCOUNT) {
            // if we aren't on target, drive the shortfall again, looping over to check we reached it
            driven = this->drive(shortfall);
            shortfall -= driven;
            this->loopCount++;
        } 
        this->loopCount = 0;

        // now repeat this for rotation
        int rotated = this->rotate(this->endRot);

        shortfall = this->endRot - rotated;

        while (abs(shortfall) > 2 && this->loopCount < MAXLOOPCOUNT) {
            rotated = this->rotate(shortfall);
            shortfall -= rotated;
            this->loopCount++;
        }
        this->loopCount = 0;

        // blink the led and sound buzzer to show we've reached the waypoint
        this->notifyOn();

        // drop the M&M if we should, if not delay for NOTIFYPAUSE to keep the LED on for a bit
        if (this->drop) { this->dispense(); }
        else { delay(NOTIFYPAUSE); }

        this->notifyOff();
    }

    // move the wheels the desired distance, and return the actual distance driven
    int drive(int d) {
        return d;
    }
};


/*
 *  Circle class, for driving the robot along a circular path
 */
class Circle: public Leg {
    // loop counter to ensure we don't get stuck in a loop, distance the outer wheel has to rotate
    int loopCount, outerDist, innerDist;
  public:
    // radius of the circle, angular distance to travel, final rotation for next leg
    int radius, theta, endRot;

    // constructor
    Circle(int r, int t, int eR, bool m) {
        this->radius = r;
        this->theta = t;
        this->endRot = eR;
        this->drop = m;
        this->loopCount = 0;
        
        // compute the outerDist as 2*pi*(radius of circle + distance to outer wheel from center of robot)*(theta/360), and parse to int
        this->outerDist = (int)(2 * PI * (this->radius + WHEELDIST) * (this->theta / FULLCIRCLE));
        
        // similar procedure for innerDist, but subtract the wheel distance instead
        this->innerDist = (int)(2 * PI * (this->radius - WHEELDIST) * (this->theta / FULLCIRCLE));
    }

    // implement the run function
    void run() {
        
    }
};


void setup()
{
    pinMode(LEDPIN, OUTPUT);

    servo.attach(SERVOPIN);
    ServoPosition = SERVOINIT;
    servo.write(ServoPosition);
    delay(3000);

    bool dropPoints[] = {false, true, false, true, false, true, false, true, false, true, false, false, false};
    for (int i = 0; i < 13; i++) {
        Line testLine = Line(100, 10, dropPoints[i]);
        testLine.run();
        delay(1000);
    }
}

void loop()
{
    
}
