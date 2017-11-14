/*
 *  Course list - includes Legs variable that stores how to run the course
 */

#include "defines.h"
#include "Leg.cpp"
#include "Line.cpp"
#include "Circle.cpp"

// include guard
#ifndef COURSE_CPP
#define COURSE_CPP

class Course {
  public:
    Leg** legs;

    Course(Servo* servo_ptr, int* servoPosition_ptr) {
        legs = new Leg*[13];

        /*
        * ---- THE LEG CODE ----
        * 
        * Each leg represents a part of the course, with the following parameters
        *      Line - Distance, Direction, Angle to turn at end, drop M&M
        *    Circle - Radius, angle to move through, direction, angle to turn at end, drop M&M 
        */
        legs[0] = new Line(428, FORWARD, 0, true);
        legs[1] = new Line(504, FORWARD,-37, false);
        legs[2] = new Circle(180, 270, BACKWARD, 90, true);
        legs[3] = new Line(180, FORWARD, -40, false);
        legs[4] = new Line(622, BACKWARD,50, true);
        legs[5] = new Line(400, BACKWARD,-90, false);
        legs[6] = new Line(400, FORWARD, 90, true);
        legs[7] = new Line(400, FORWARD, 90, false);
        legs[8] = new Line(660, FORWARD, 90, true);
        legs[9] = new Circle(260, 90, FORWARD, -90, false);
        legs[10] = new Line(500, FORWARD, 90, false);
        legs[11] = new Line(260, FORWARD, 90, false);
        legs[12] = new Line(340, FORWARD, 143, false);

        for (int i = 0; i < 13; i++) {
            legs[i]->servo = servo_ptr;
            legs[i]->servoPosition = servoPosition_ptr;
        }
    }

    void run() {
        for (int i = 0; i < 13; i++) {
            Serial.print("Running leg ");
            Serial.println(i);

            legs[i]->run();
        }
    }
};

#endif