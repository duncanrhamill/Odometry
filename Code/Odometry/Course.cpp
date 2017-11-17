/*
 *  Course list - includes Legs variable that stores how to run the course
 *  Also includes servo level logic
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
    // pointer to the leg pointer array
    Leg** legs;

    // global servo and servo position
    Servo servo;
    int servoPosition;

    // constructor
    Course() {
        Serial.println("Constructing course");

        // set the servo to it's initial position
        ServoSetup();
        
        // initialise the legs pointer
        legs = new Leg*[13];

        /*
        * ---- THE LEG CODE ----
        * 
        * Each leg represents a part of the course, with the following parameters
        *      Line - Distance, Direction, Angle to turn at end, drop M&M
        *    Circle - Radius, angle to move through, direction, angle to turn at end, drop M&M 
        *    
        *    Many of these distances and angles found empirically
        */
        legs[0] = new Line(425, BACKWARD, 0, false);
        legs[1] = new Line(455, BACKWARD,-37, false);
        legs[2] = new Circle(170, 260, FORWARD, 88, true);
        legs[3] = new Line(170, BACKWARD, -33, false);
        legs[4] = new Line(605, FORWARD, 38, true);
        legs[5] = new Line(400, FORWARD, 87, false);
        legs[6] = new Line(420, FORWARD, 89, true);
        legs[7] = new Line(410, FORWARD, 89, false);
        legs[8] = new Line(665, FORWARD, -90, true);
        legs[9] = new Circle(260, 85, FORWARD, -82, false);
        legs[10] = new Line(535, BACKWARD, 86, true);
        legs[11] = new Line(266, BACKWARD, 88, false);
        legs[12] = new Line(355, FORWARD, 37, false);

    }

    // attach the servo pin to the servo object and set it to it's initial position
    void ServoSetup() {
        servo.attach(SERVOPIN);
        servoPosition = SERVOINIT;
        servo.write(servoPosition);
    }

    // run the whole course by looping through each leg, calling it's run function, and then calling the action
    void run() {
        for (int i = 0; i < 13; i++) {
            Serial.print("Running leg ");
            Serial.println(i);

            legs[i]->run();

            // pass the leg's drop variable into the action function
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
