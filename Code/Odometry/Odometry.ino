/*
 *      Odometry Task - Aero 2 Group 2
 * 
 *          Code written by Duncan R Hamill - 28262174
 *          Tested & verified by Tom Griffiths - 28290771, Ali Hajizadah - 29053056, Robin Hannaford - , Felix Harris - 28611969
 * 
 *          All distances in mm, all angles in degrees
 */

#include "defines.h"
#include "encoderInteraction.cpp"
#include "Course.cpp"

#include <Wire.h>

void setup() {
    // start serial for monitoring
    Serial.begin(9600);
    
    // setup the I2C and wait 100ms
    Wire.begin();
    delay(100);

    // zero the encoders
    resetEncoders();

    // initialise the course
    Course course = Course();

    // wait a bit before we start
    delay(1000);

    // TESTING - loop through legs and run their action
    course.run();

    /*Line l1 = Line(500, FORWARD, 0, false);
    Line l2 = Line(500, FORWARD, 0, false);
    Line l3 = Line(500, BACKWARD, 0, false);
    Line l4 = Line(500, BACKWARD, 0, false);
    l1.run();
    delay(1000);
    l2.run();
    delay(1000);
    l3.run();
    delay(1000);
    l4.run();*/

    // turn on for event
    //finished();

}

// just a bit of fun
void finished() {
    tone(9,660,100);
    delay(150);
    tone(9,660,100);
    delay(300);
    tone(9,660,100);
    delay(300);
    tone(9,510,100);
    delay(100);
    tone(9,660,100);
    delay(300);
    tone(9,770,100);
    delay(550);
    tone(9,380,100);
    delay(575);
}

void loop() {
    
}
