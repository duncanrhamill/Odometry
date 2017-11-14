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
    int loopCount;
  public:
    // distance to travel, and how far to rotate to be pointing in correct direction at end of the leg
    int dist, endRot;

    // constructor
    Line(int d, int dir, int r, bool m) {
        this->dist = d;
        this->direction = dir;
        this->endRot = r;
        this->drop = m;
        this->loopCount = 0;
    }

    // implement the run function
    void run() {
       
        // run drive, get how far we actually drove
        int driven = this->drive(this->direction * this->dist, false);

        // calculate distance left to drive
        int shortfall = this->dist - driven;

        // aim to get within 2mm of the target waypoint, without going over MAXLOOPCOUNT
        while (abs(shortfall) > LINEARTOL && loopCount < MAXLOOPCOUNT) {
            // if we aren't on target, drive the shortfall again, looping over to check we reached it
            driven = this->drive(shortfall, true);
            this->stop();
            shortfall = abs(shortfall) - driven;
            this->loopCount++;
            
        } 
        this->loopCount = 0;

        // now repeat this for rotation
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

        // blink light, sound buzzer, and drop M&M if needed
        this->action();
    }

    // move the wheels the desired distance, and return the actual distance driven
    int drive(int d, bool correction) {
        Serial.print("Line ");

        if (d == 0) {
            return 0;
        }
        
        while(averageDistance() <= abs(d)) {
            int spd;

            // if in a correction, go slowly for more accuracy
            if (correction) {
                spd = DUALSPEED * 0.2;
            } else {
                spd = DUALSPEED;
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
                Wire.write((char)(128 - spd));
            } else {
                Wire.write((char)(128 + spd));
            }
            Wire.endTransmission();
        }

        // return the read distance
        int avg = averageDistance();
        Serial.println(avg);
        resetEncoders();
        this->stop();
        delay(50);
        return avg;
    }
};

#endif
