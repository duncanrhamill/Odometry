/*
 *  Circle class - contains logic for driving a circular section of the course
 */

#include "defines.h"
#include "encoderInteraction.cpp"
#include "Leg.cpp"

#include <Arduino.h>
#include <Wire.h>

// include guard
#ifndef CIRCLE_CPP
#define CIRCLE_CPP

class Circle: public Leg {
    // loop counter to ensure we don't get stuck in a loop, distance the outer wheel has to rotate
    int loopCount, dir;
    float outerDist;

  public:
    // radius of the circle, angular distance to travel, final rotation for next leg
    int radius, theta, endRot;

    // constructor
    Circle(int r, int t, int _dir, int eR, bool m) {
        this->radius = r;
        this->theta = t;
        this->dir = _dir;
        this->endRot = eR;
        this->drop = m;
        this->loopCount = 0;
        
        // compute the outerDist as 2*pi*(radius of circle + distance to outer wheel from center of robot)*(theta/360), and parse to int
        this->outerDist = (float)(2 * PI * (this->radius + WHEELDIST) * ((float)abs(this->theta) / 360));
    }

    // implement the run function
    void run() {
      
        // set the robot to drive an arc in the specified direction, and at the given angle. Don't do corrective speeds
        float driven = this->drive(this->dir * this->theta, false);

        // get angular shortfall
        float angShortfall = (this->dir * this->theta) - driven;

        // call the drive function again with corrective speeds to solve any drive issues
        while (fabs(angShortfall) > ARCTOL && this->loopCount < MAXLOOPCOUNT) {
            driven = this->drive(angShortfall, true);
            this->stop();

            // subtract how far we moved from angShortfall so we get progressively closer to the target
            angShortfall = angShortfall - driven;
            this->loopCount++;
        }
        // reset the loop counter
        this->loopCount = 0;

        // rotate to start of next leg
        float rotated = this->rotate(this->endRot, false);

        // now correct rotation in a similar way to the arc drive
        float rotShortfall = this->endRot - rotated;

        while (fabs(rotShortfall) > ANGULARTOL && loopCount < MAXLOOPCOUNT) {
            Serial.println("Correcting rotation");
            rotated = this->rotate(rotShortfall, true);
            this->stop();
            rotShortfall = rotShortfall - rotated;
            this->loopCount++;
        }
        this->loopCount = 0;
    }

    // function to drive in an arc
    float drive(float t, bool correction) {
        // variables to store the encoders so we can drive clockwise and anti clockwise
        char innerWheel, outerWheel, innerSpeed, outerSpeed;

        // if we're given a zero angle don't do any driving
        if (t == 0) {
            return 0;
        }
        
        // set outerDistance to the arclength for the required theta
        this->outerDist = (float)(2 * PI * (this->radius + WHEELDIST) * ((float)fabs(t) / 360));

        // if we're going forward, the left wheel is on the inside, else its the outside wheel
        if (this->dir == FORWARD) {
            innerWheel = ENCODELEFT;
            outerWheel = ENCODERIGHT;
            innerSpeed = SPEEDLEFT;
            outerSpeed = SPEEDRIGHT;
        } else {
            innerWheel = ENCODERIGHT;
            outerWheel = ENCODELEFT;
            innerSpeed = SPEEDRIGHT;
            outerSpeed = SPEEDLEFT;
        }

        // angular velocity from dual speed
        float omega = ((float)DUALSPEED * 0.5 / (float)this->radius);

        // if have a negative angle, need to drive backward
        if (t < 0) {
            omega *= -1;
        }

        // if correction, reduce the speed for greater accuracy
        if (correction) {
            omega *= 0.3;
        }

        // set an unsigned char storing the velocity of each wheel
        unsigned char outerVel = 128 + (this->radius + WHEELDIST) * omega;
        unsigned char innerVel = 128 + (this->radius - WHEELDIST) * omega;

        // variable to store the distance moved by the outer wheel
        long outerDriven;

        // reset encoders so we have an accurate first reading.
        resetEncoders();
        
        // loop through driving until one of the outer distance is over it's limit
        do  {
            // Set wheels to spin at different rates
            Wire.beginTransmission(MD25ADDR);
            Wire.write(MODE);
            Wire.write(MODESEPERATE);
            Wire.endTransmission();

            // Set outer wheel speed
            Wire.beginTransmission(MD25ADDR);
            Wire.write(outerSpeed);
            Wire.write((unsigned char)outerVel);
            Wire.endTransmission();

            // set inner wheel speed
            Wire.beginTransmission(MD25ADDR);
            Wire.write(innerSpeed);
            Wire.write((unsigned char)innerVel);
            Wire.endTransmission();
            
            outerDriven = individualDistance(outerWheel);
            
        } while (outerDriven <= this->outerDist);

        // get the angle driven through
        float ang = (float)(360 * outerDriven)/ (float)(2 * PI * (this->radius + WHEELDIST));

        // if we were going to drive backwards negate the angle so correction doesn't go on for ever
        if (t < 0) {
            ang *= -1;
        }

        // reset the encoders, stop the robot, and return the angle traversed.
        resetEncoders();
        this->stop();
        return ang;
    }
    
};

#endif
