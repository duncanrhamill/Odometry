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

        // find distance needed to rotate
        float dist = (2 * PI * WHEELDIST * ((float)fabs(t) / (float)360));

        Serial.print("dist: ");
        Serial.println(dist);

        // speeds of each wheel
        int leftWheel, rightWheel, rotateSpeed;

        // if we're in correction mode rotate slower
        if (correction) {
            rotateSpeed = DUALSPEED * 0.05;
        } else {
            rotateSpeed = DUALSPEED * 0.2;
        }

        // set speeds of each wheel depending on direction (+ve -> left goes forwards)
        if (t > 0) {
            leftWheel = (128 + rotateSpeed);
            rightWheel = (128 - rotateSpeed);
        } else {
            leftWheel = 128 - rotateSpeed;
            rightWheel = 128 + rotateSpeed;
        }

        Serial.print("LeftWheel spd: ");
        Serial.print((unsigned char)leftWheel, DEC);
        Serial.print(", RightWheel spd: ");
        Serial.println((unsigned char)rightWheel, DEC);

        int avgD = averageDistance();

        Serial.print("avg: ");
        Serial.print(avgD);
        
        // while we've not rotated less that the required distance
        while (averageDistance() <= dist) {
            Serial.print("l: ");
            Serial.print(individualDistance(ENCODELEFT));
            Serial.print(", r: ");
            Serial.println(individualDistance(ENCODERIGHT));
            
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

        long avg = (long)averageDistance();
        
        float ang =((float)(360 * avg)/((float)(2 * PI * WHEELDIST)));

        if (t < 0) {
            ang *= -1;
        }

        Serial.print("Rotate t: ");
        Serial.print(t);
        Serial.print(", Actual t: ");
        Serial.println(ang);

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
