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
    float outerDist, innerDist;
  public:
    // radius of the circle, angular distance to travel, final rotation for next leg
    int radius, theta, endRot;

    // constructor
    Circle(int r, int t, int dir, int eR, bool m) {
        this->radius = r;
        this->theta = t;
        this->dir = dir;
        this->endRot = eR;
        this->drop = m;
        this->loopCount = 0;
        
        // compute the outerDist as 2*pi*(radius of circle + distance to outer wheel from center of robot)*(theta/360), and parse to int
        this->outerDist = (float)(2 * PI * (this->radius + WHEELDIST) * ((float)abs(this->theta) / 360));
        
        // similar procedure for innerDist, but subtract the wheel distance instead
        this->innerDist = (float)(2 * PI * (this->radius - WHEELDIST) * ((float)abs(this->theta) / 360));
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
        float driven = this->drive(this->dir * this->theta, false);

        Serial.println(driven);
        Serial.print(" ");

        // get angular shortfall
        float angShortfall = this->theta - driven;

        Serial.println(angShortfall);
        Serial.print(" ");

        while (abs(angShortfall) > ARCTOL && this->loopCount < MAXLOOPCOUNT) {
            driven = this->drive(angShortfall, true);
            this->stop();
            angShortfall = abs(angShortfall) - driven;
            Serial.println(angShortfall);
            Serial.print(" ");
            this->loopCount++;
        }
        this->loopCount = 0;

        // rotate to start of next leg
        float rotated = this->rotate(this->endRot, false);

        float rotShortfall = this->endRot - rotated;

        while (abs(rotShortfall) > ANGULARTOL && loopCount < MAXLOOPCOUNT) {
            Serial.println("Correcting rotation");
            rotated = this->rotate(rotShortfall, true);
            this->stop();
            rotShortfall = abs(rotShortfall) - rotated;
            this->loopCount++;
        }
        this->loopCount = 0;

        // perform any actions needed
        this->action();
    }

    float drive(int t, bool correction) {
        char innerWheel, outerWheel, innerSpeed, outerSpeed;

<<<<<<< HEAD
        // reset inner distance to theta
        this->innerDist = (int)(2 * PI * (this->radius - WHEELDIST) * ((float)fabs(t) / 360));
=======
        float test = (float)(abs(t)/360);
        Serial.println(test, 10);
        
        // reset outer distance to theta
        this->outerDist = (float)(2 * PI * (this->radius + WHEELDIST) * ((float)abs(t) / 360));
>>>>>>> ali-dev

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

        // angular velocity from dual speed, with direction
        float omega = ((float)DUALSPEED * 0.5 / (float)this->radius);

        if (t < 0) {
            omega *= -1;
        }

        // if correction, reduce the speed for greater accuracy
        if (correction) {
            omega *= 0.3;
        }

        Serial.print("Omega: ");
        Serial.println(omega);

        // loop through driving until one of the distances is over it's limit
        while (individualDistance(outerWheel) <= this->outerDist) {
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

        long outerDriven = individualDistance(outerWheel);

        Serial.print(this->outerDist);
        Serial.print(" ");
        Serial.println(outerDriven);
        // get the angle driven through
        float ang = (float)(360 * outerDriven)/ (float)(2 * PI * (this->radius + WHEELDIST));
        resetEncoders();
        this->stop();
        return ang;
    }
    
};

#endif
