/*
 *  Leg class - controls the robot for one 'leg' (segment) of the course
 *              includes logic for the action, rotate, and stop functions
 */

#include "defines.h"
#include "encoderInteraction.cpp"

#include <Servo.h>
#include <Wire.h>

// include guard
#ifndef LEG_CPP
#define LEG_CPP

class Leg
{
  public:
    // pointer to serveo position variable
    int* servoPosition;
    Servo* servo;

    // Should we drop an M&M?, leg finished successfully?
    bool drop, direction;
    
    // Virtual function that will be called to run this leg of the course.
    virtual void run();

    // perform actions at waypoint, including dropping M&M if needed
    void action() {
        // turn on LED and buzzer
        digitalWrite(LEDPIN, HIGH);
        tone(PIEZOPIN, PIEZOFREQ);
        
        // if need to drop M&M, drop one, if not delay so we can see and hear buzzer
        if (this->drop) { this->dispense(); }
        else { delay(NOTIFYPAUSE); }

        // turn off led & buzzer
        digitalWrite(LEDPIN, LOW);
        noTone(PIEZOPIN);
    }

    // dispense an M&M
    void dispense() {
        // increase servo position
        *servoPosition += SERVOSTEP;

        // make sure we don't accidentally run through all positions
        if (*servoPosition >= 179) {
            *servoPosition = 179;
        }
        
        // write the servo position and wait to ensure clean drop
        *servo.write(*servoPosition);
        delay(SERVOPAUSE);
    }

    // rotate by the given angle (+ve clockwise), returning the actual angle rotated
    int rotate(int t, bool correction) {
        Serial.print("Rotate ");

        // find distance needed to rotate
        int dist = (int)(2 * PI * WHEELDIST * ((float)abs(t) / (float)360));

        Serial.print(dist);
        Serial.print(" ");

        // speeds of each wheel
        int leftWheel, rightWheel, rotateSpeed;

        // if we're in correction mode rotate slower
        if (correction) {
            rotateSpeed = DUALSPEED * 0.1;
        } else {
            rotateSpeed = DUALSPEED * 0.5;
        }
        
        Serial.print((int)rotateSpeed);

        // set speeds of each wheel depending on direction (+ve -> left goes forwards)
        if (t > 0) {
            leftWheel = (128 + rotateSpeed);
            rightWheel = (128 - rotateSpeed);
        } else {
            leftWheel = 128 - rotateSpeed;
            rightWheel = 128 + rotateSpeed;
        }

        Serial.print(" ");
        Serial.print((int)leftWheel);
        Serial.print(" ");
        Serial.println((int)rightWheel);
        
        // while we've not rotated less that the required distance
        while (averageDistance() <= dist) {
            Serial.println(averageDistance());
          
            // set wheels to spin at different speeds
            Wire.beginTransmission(MD25ADDR);
            Wire.write(MODE);
            Wire.write(MODESEPERATE);
            Wire.endTransmission();

            // set the acceleration mode to fast
            Wire.beginTransmission(MD25ADDR);
            Wire.write(ACCEL);
            Wire.write(ACCELDEFAULT);
            Wire.endTransmission();

            // Set left wheel speed
            Wire.beginTransmission(MD25ADDR);
            Wire.write(SPEEDLEFT);
            Wire.write((char)leftWheel);
            Wire.endTransmission();

            // set right wheel speed
            Wire.beginTransmission(MD25ADDR);
            Wire.write(SPEEDRIGHT);
            Wire.write((char)rightWheel);
            Wire.endTransmission();
        }

        int avg = averageDistance();

        // return the angle rotated
        int ang = (int)((float)(360 * avg)/(float)(2 * PI * WHEELDIST));

        resetEncoders();
        this->stop();
        delay(50);
        return ang;
    }

    // stop the vehicle
    void stop() {
        // allow both registers to be set to stop
        Wire.beginTransmission(MD25ADDR);
        Wire.write(MODE);
        Wire.write(MODESEPERATE);
        Wire.endTransmission();

        // high acceleration mode
        Wire.beginTransmission(MD25ADDR);
        Wire.write(ACCEL);
        Wire.write(10);
        Wire.endTransmission();

        // set left to stop
        Wire.beginTransmission(MD25ADDR);
        Wire.write(SPEEDLEFT);
        Wire.write(128);
        Wire.endTransmission();
        
        // set right to stop
        Wire.beginTransmission(MD25ADDR);
        Wire.write(SPEEDRIGHT);
        Wire.write(128);
        Wire.endTransmission();
        delay(50);
    }
};

#endif