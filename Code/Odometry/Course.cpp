/*
 *  Course list - includes Legs variable that stores how to run the course
 */

#include "defines.h"
#include "Leg.cpp"
#include "Line.cpp"
#include "Circle.cpp"

#include <Servo.h>

// include guard
#ifndef COURSE_CPP
#define COURSE_CPP

class Course {
  public:
    Leg** legs;

    Servo servo;
    int servoPosition;

    Course() {
        Serial.println("Constructing course");
      
        ServoSetup();

        legs = new Leg*[13];

        /*
        * ---- THE LEG CODE ----
        * 
        * Each leg represents a part of the course, with the following parameters
        *      Line - Distance, Direction, Angle to turn at end, drop M&M
        *    Circle - Radius, angle to move through, direction, angle to turn at end, drop M&M 
        */
        legs[0] = new Line(425, BACKWARD, 0, false);
        legs[1] = new Line(455, BACKWARD,-37, false);
        legs[2] = new Circle(180, 260, FORWARD, 90, true);
        legs[3] = new Line(170, BACKWARD, -35, false);
        legs[4] = new Line(595, FORWARD, 36, true);
        legs[5] = new Line(400, FORWARD, 87, false);
        legs[6] = new Line(400, FORWARD, 84, true);
        legs[7] = new Line(400, FORWARD, 84, false);
        legs[8] = new Line(660, FORWARD, -90, true);
        legs[9] = new Circle(260, 85, FORWARD, -85, false);
        legs[10] = new Line(500, BACKWARD, 90, true);
        legs[11] = new Line(256, BACKWARD, 90, false);
        legs[12] = new Line(335, FORWARD, 143, false);

    }

    void ServoSetup() {
        servo.attach(SERVOPIN);
        servoPosition = SERVOINIT;
        servo.write(servoPosition);
    }

    void run() {
        for (int i = 0; i < 13; i++) {
            Serial.print("Running leg ");
            Serial.println(i);

            legs[i]->run();
            action(legs[i]->drop);
        }
    }

    // perform actions at waypoint, including dropping M&M if needed
    void action(bool drop) {
        // turn on LED and buzzer
        digitalWrite(LEDPIN, HIGH);
        tone(PIEZOPIN, PIEZOFREQ);
        
        // if need to drop M&M, drop one, if not delay so we can see and hear buzzer
        if (drop) { 
            dispense(); 
        } else { 
            delay(NOTIFYPAUSE); 
        }

        // turn off led & buzzer
        digitalWrite(LEDPIN, LOW);
        noTone(PIEZOPIN);
    }

    // dispense an M&M
    void dispense() {
        // increase servo position
        servoPosition += SERVOSTEP;

        // make sure we don't accidentally run through all positions
        if (servoPosition >= 179) {
            servoPosition = 179;
        }
        
        // write the servo position and wait to ensure clean drop
        servo.write(servoPosition);
        delay(SERVOPAUSE);
    }
};

#endif
