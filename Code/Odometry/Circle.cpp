/*
 *  Circle class - contains logic for driving a circular section of the course
 */

#include "defines.h"
#include "encoderInteraction.cpp"
#include "Leg.cpp"

#include <Wire.h>

// include guard
#ifndef CIRCLE_CPP
#define CIRCLE_CPP

class Circle: public Leg {
    // loop counter to ensure we don't get stuck in a loop, distance the outer wheel has to rotate
    int loopCount, direction, outerDist, innerDist;
  public:
    // radius of the circle, angular distance to travel, final rotation for next leg
    int radius, theta, endRot;

    // constructor
    Circle(int r, int t, int dir, int eR, bool m) {
        this->radius = r;
        this->theta = t;
        this->direction = dir;
        this->endRot = eR;
        this->drop = m;
        this->loopCount = 0;
        
        // compute the outerDist as 2*pi*(radius of circle + distance to outer wheel from center of robot)*(theta/360), and parse to int
        this->outerDist = (int)(2 * PI * (this->radius + WHEELDIST) * ((float)abs(this->theta) / 360));
        
        // similar procedure for innerDist, but subtract the wheel distance instead
        this->innerDist = (int)(2 * PI * (this->radius - WHEELDIST) * ((float)abs(this->theta) / 360));
    }

    // implement the run function
    void run() {
        Serial.print("Circle ");

        Serial.print(this->radius);
        Serial.print(" ");
        Serial.print(this->outerDist);
        Serial.print(" ");
        Serial.print(this->innerDist);
        Serial.print(" ");
        
        // drive round in a circle
        int driven = this->drive(this->radius, false);

        // get angular shortfall
        int angShortfall = this->theta - driven;

        while (abs(innerShortfall) <= ANGULARTOL && this->loopCount < MAXLOOPCOUNT) {
            driven = this->drive(innerShortfall, true);
            shortfall = abs(shortfall) - driven;
            this->loopCount++;
        }
        this->loopCount = 0;

        // rotate to start of next leg
        this->rotate(endRot, false);

        // perform any actions needed
        this->action();
    }

    int drive(int t, bool correction) {
        char innerWheel, outerWheel, innerSpeed, outerSpeed;

        // reset inner distance to theta
        this->innerDist = (int)(2 * PI * (this->radius - WHEELDIST) * ((float)abs(t) / 360));

        // if we're going forward, the left wheel is on the inside, else its the outside wheel
        if (this->direction == FORWARD) {
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

        // angular velocity from dual speed, with direction
        float omega = this->direction * ((float)DUALSPEED * 0.5 / (float)this->radius);

        // if correction, reduce the speed for greater accuracy
        if (correction) {
            omega *= 0.2;
        }

        Serial.print("Omega: ");
        Serial.println(omega);

        // loop through driving until one of the distances is over it's limit
        while (individualDistance(innerWheel) <= this->innerDist) {
            // Set wheels to spin at different rates
            Wire.beginTransmission(MD25ADDR);
            Wire.write(MODE);
            Wire.write(MODESEPERATE);
            Wire.endTransmission();

            // set the acceleration mode to fast
            Wire.beginTransmission(MD25ADDR);
            Wire.write(ACCEL);
            Wire.write(ACCELDEFAULT);
            Wire.endTransmission();

            // Set outer wheel speed
            Wire.beginTransmission(MD25ADDR);
            Wire.write(outerSpeed);
            Wire.write((char)(128 + (this->radius + WHEELDIST) * omega));
            Wire.endTransmission();

            // set inner wheel speed
            Wire.beginTransmission(MD25ADDR);
            Wire.write(innerSpeed);
            Wire.write((char)(128 + (this->radius - WHEELDIST) * omega));
            Wire.endTransmission();
        }

        int innerDriven = individualDistance(innerWheel);
        // get the angle driven through
        int ang = 360 * innerDriven / (2 * PI * (this->radius - WHEELDIST));
        resetEncoders();
        this->stop();
        return ang;
    }
    
};

#endif