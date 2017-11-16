/*
 *  Line class - drives a straight leg of the course
 */

#include "defines.h"
#include "encoderInteraction.cpp"
#include "Leg.cpp"

#include <Arduino.h>
#include <Wire.h>

// include guard
#ifndef LINE_CPP
#define LINE_CPP

class Line: public Leg {
    // count how many times we loop over the drive sections, so we don't get stuck.
    int loopCount, dir;

    // ramp function to increase speed over course of a line
    int ramp(int m, int dist, int x) {
        int offset = (2 * (float)m / dist)*(x - ((float)dist / 2));
        int spd = m - abs(offset);
        return spd;
    }

  public:
    // distance to travel, and how far to rotate to be pointing in correct direction at end of the leg
    int dist, endRot;

    // constructor
    Line(int d, int _dir, int r, bool m) {
        this->dist = d;
        this->dir = _dir;
        this->endRot = r;
        this->drop = m;
        this->loopCount = 0;
    }

    // implement the run function
    void run() {

        if (this->dir == BACKWARD) {
            this->dist *= 0.9;
        }
        
        // run drive, get how far we actually drove
        int driven = this->drive(this->dir * this->dist, false);

        // calculate distance left to drive
        int shortfall = (this->dir * this->dist) - driven;

        shortfall *= 1.5;

        // aim to get within LINEARTOL of the target waypoint, without going over MAXLOOPCOUNT
        while (abs(shortfall) > LINEARTOL && loopCount < MAXLOOPCOUNT) {
            // if we aren't on target, drive the shortfall again, looping over to check we reached it
            driven = this->drive(shortfall, true);
            this->stop();
            shortfall = shortfall - driven;
            shortfall *= 1.5;
            this->loopCount++;
            
        } 
        this->loopCount = 0;

        // now repeat this for rotation
        float rotated = this->rotate(this->endRot, false);

        float rotShortfall = this->endRot - rotated;

        while (fabs(rotShortfall) > ANGULARTOL && loopCount < MAXLOOPCOUNT) {
            Serial.print("Correcting shortfall ");
            Serial.println(rotShortfall);
            rotated = this->rotate(rotShortfall, true);
            this->stop();
            rotShortfall = rotShortfall - rotated;
            this->loopCount++;
        }
        this->loopCount = 0;
    }

    // move the wheels the desired distance, and return the actual distance driven
    int drive(int d, bool correction) {        
        Serial.print("Line ");
        Serial.print(this->dir);
        Serial.print(" ");
        Serial.println(d);

        if (d == 0) {
            return 0;
        }

        int avgDist;
        resetEncoders();
        do {
            avgDist = averageDistance();
            
            int spd;

            // if in a correction, go slowly for more accuracy
            if (correction) {
                spd = DUALSPEED * 0.1;
            } else {
                spd = 1 + ramp(DUALSPEED, abs(d), avgDist);
            }

            // Set both wheels to spin at the same rate
            Wire.beginTransmission(MD25ADDR);
            Wire.write(MODE);
            Wire.write(MODEUNSIGNEDDUAL);
            Wire.endTransmission();

            // set the acceleration mode to fast
            Wire.beginTransmission(MD25ADDR);
            Wire.write(ACCEL);
            Wire.write(ACCELDEFAULT);
            Wire.endTransmission();

            // set the speed
            Wire.beginTransmission(MD25ADDR);
            Wire.write(SPEEDLEFT);
            
            // if we're given a negative distance, drive backwards
            if (d < 0) {
                Wire.write((unsigned char)(128 - spd));
            } else {
                Wire.write((unsigned char)(128 + spd));
            }
            Wire.endTransmission();
        } while (avgDist <= abs(d));

        // return the read distance
        int avg = averageDistance();
        
        Serial.print("Distance required: ");
        Serial.print(d);
        Serial.print(", Distance traveled: ");
        Serial.println(avg);

        if (d < 0) {
            avg *= -1;
        }
        
        resetEncoders();
        this->stop();
        delay(50);
        return avg;
    }
};

#endif
