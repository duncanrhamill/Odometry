/*
 *      Odometry Task - Aero 2 Group 2
 * 
 *          Code written by Duncan R Hamill - 28262174
 * 
 *          All distances in mm, all angles in degrees
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
#define DUALSPEED 50

// MD25 I2C codes
#define MD25ADDR 0x58
#define SPEEDLEFT 0x00
#define SPEEDRIGHT 0x01
#define ENCODELEFT 0x02
#define ENCODERIGHT 0x06
#define VOLTS 0x0A
#define MOTOR1I 0x0B
#define MOTOR2I 0x0C
#define ACCEL 0x0E
#define MODE 0x0F
#define CMD 0x10

// MD25 command codes
#define CLEARENCODERREGISTERS 0x20

// MD25 acceleration modes
#define ACCELDEFAULT 5

// MD25 modes
#define MODEDUAL 3
#define MODESEPERATE 1

// pin definitions
#define LEDPIN 8
#define PIEZOPIN 9
#define SERVOPIN 10

// Servo global vars
Servo servo;
int ServoPosition;


/*
 *  Encoder interaction -------------------------------------------------------
 */

// find the average distance travelled
int averageDistance() {
    // get individual wheel distances
    int distLeft = individualDistance(ENCODELEFT);
    int distRight = individualDistance(ENCODERIGHT);

    // return the average
    return (int)(distLeft + distRight / 2);
}

// find distance left wheel has moved
int individualDistance(char side) {
    // set MD25 to send the encoder for the given side
    Wire.beginTransmission(MD25ADDR);
    Wire.write(side);
    Wire.endTransmission();

    // request 4 bytes from the MD25
    Wire.requestFrom(MD25ADDR, 4);

    // the raw click value from the encoder
    long clicks;

    // wait for first 4 bytes back
    while (Wire.available() < 4)

    // get all bytes of the click var
    clicks = Wire.read();
    for (int i = 0; i < 3; i++) {
        clicks <<= 8;
        clicks += Wire.read();
    }

    // delay to ensure data is correct
    delay(5);

    // return a value in mm
    return (int)(clicks * 0.009);
}

// reset distance encoders between legs
void resetEncoders() {
    Wire.beginTransmission(MD25ADDR);
    Wire.write(CMD);
    Wire.write(CLEARENCODERREGISTERS);
    Wire.endTransmission();
    delay(10);
}

// ----------------------------------------------------------------------------

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
            resetEncoders();
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

        // turn off light and buzzer
        this->notifyOff();
    }

    // move the wheels the desired distance, and return the actual distance driven
    int drive(int d) {
        while(averageDistance() <= d) {
            // Set both wheels to spin at the same rate
            Wire.beginTransmission(MD25ADDR);
            Wire.write(MODE);
            Wire.write(MODEDUAL);
            Wire.endTransmission();

            // set the acceleration mode to fast
            Wire.beginTransmission(MD25ADDR);
            Wire.write(ACCEL);
            Wire.write(ACCELDEFAULT);
            Wire.endTransmission();

            // set the speed
            Wire.beginTransmission(MD25ADDR);
            Wire.write(SPEEDLEFT);
            Wire.write(DUALSPEED);
            Wire.endTransmission();
        }

        // return the read distance
        return averageDistance();
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

/*
 *  Arduino logic -------------------------------------------------------------
 */

void setup()
{
    // Set LED pin mode
    pinMode(LEDPIN, OUTPUT);

    // setup servo
    servo.attach(SERVOPIN);
    ServoPosition = SERVOINIT;
    servo.write(ServoPosition);

    // delay before starting test sequence;
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
