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
    Circle(int r, int t, int _dir, int eR, bool m) {
        this->radius = r;
        this->theta = t;
        this->dir = _dir;
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
        float angShortfall = (this->dir * this->theta) - driven;

        Serial.println(angShortfall);
        Serial.print(" ");

        while (fabs(angShortfall) > ARCTOL && this->loopCount < MAXLOOPCOUNT) {
            Serial.print(angShortfall);
            Serial.print(" ");
            driven = this->drive(angShortfall, true);
            this->stop();
            angShortfall = angShortfall - driven;
            this->loopCount++;
        }
        this->loopCount = 0;

        // rotate to start of next leg
        float rotated = this->rotate(this->endRot, false);

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

    float drive(float t, bool correction) {
        char innerWheel, outerWheel, innerSpeed, outerSpeed;

        if (t == 0) {
            return 0;
        }
        
        // reset outer distance to theta
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

        unsigned char outerVel = 128 + (this->radius + WHEELDIST) * omega;
        unsigned char innerVel = 128 + (this->radius - WHEELDIST) * omega;

        Serial.print("outerVel: ");
        Serial.print(outerVel, DEC);
        Serial.print("innerVel: ");
        Serial.println(innerVel, DEC);

        long outerDriven;

        // loop through driving until one of the distances is over it's limit
        resetEncoders();
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

        Serial.print("OuterDist: ");
        Serial.println(this->outerDist);
        Serial.print("OuterDriven: ");
        Serial.println(outerDriven);
        // get the angle driven through
        float ang = (float)(360 * outerDriven)/ (float)(2 * PI * (this->radius + WHEELDIST));

        if (t < 0) {
            ang *= -1;
        }

        Serial.print("ang: ");
        Serial.println(ang);
        
        resetEncoders();
        this->stop();
        return ang;
    }
    
};

#endif
