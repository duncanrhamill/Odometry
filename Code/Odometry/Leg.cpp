/*
 *  Leg class - controls the robot for one 'leg' (segment) of the course
 *              includes logic for the action, rotate, and stop functions
 */

#include "defines.h"
#include "encoderInteraction.cpp"

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>

// include guard
#ifndef LEG_CPP
#define LEG_CPP

class Leg
{
  public:

    // Should we drop an M&M?, leg finished successfully?
    bool drop, dir;
    
    // Virtual function that will be called to run this leg of the course.
    virtual void run();

    // rotate by the given angle (+ve clockwise), returning the actual angle rotated
    float rotate(float t, bool correction) {
        resetEncoders();
      
        if (t == 0) {
            return 0;
        }

        // find distance needed to rotate as an arc length of the required angle
        float dist = (2 * PI * WHEELDIST * ((float)fabs(t) / (float)360));

        // speeds of each wheel
        int leftWheel, rightWheel, rotateSpeed;

        // if we're in correction mode rotate slower
        if (correction) {
            rotateSpeed = DUALSPEED * 0.1;
        } else {
            rotateSpeed = DUALSPEED * 0.4;
        }

        // set speeds of each wheel depending on direction (+ve -> left goes forwards)
        if (t > 0) {
            leftWheel = (128 + rotateSpeed);
            rightWheel = (128 - rotateSpeed);
        } else {
            leftWheel = 128 - rotateSpeed;
            rightWheel = 128 + rotateSpeed;
        }

        // while we've not rotated less that the required distance
        while (averageDistance() <= dist) {
            
            // set wheels to spin at different speeds
            Wire.beginTransmission(MD25ADDR);
            Wire.write(MODE);
            Wire.write(MODESEPERATE);
            Wire.endTransmission();

            // Set left wheel speed
            Wire.beginTransmission(MD25ADDR);
            Wire.write(SPEEDLEFT);
            Wire.write((unsigned char)leftWheel);
            Wire.endTransmission();

            // set right wheel speed
            Wire.beginTransmission(MD25ADDR);
            Wire.write(SPEEDRIGHT);
            Wire.write((unsigned char)rightWheel);
            Wire.endTransmission();
        }

        // get the average distance we travelled
        long avg = (long)averageDistance();

        // convert that to an angle
        float ang =((float)(360 * avg)/((float)(2 * PI * WHEELDIST)));

        // negate it if we went backwards
        if (t < 0) {
            ang *= -1;
        }

        // reset encoders, stop, and delay slightly
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
