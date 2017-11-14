/*
 *      Odometry Task - Aero 2 Group 2
 * 
 *          Code written by Duncan R Hamill - 28262174
 *          Tested & verified by Tom Griffiths - 28290771, Ali Hajizadah - 29053056, Robin Hannaford - , Felix Harris - 28611969
 * 
 *          All distances in mm, all angles in degrees
 */

#include "defines.h"
#include "Course.cpp"

#include <Wire.h>

void setup() {
    Serial.begin(9600);
    
    Wire.begin();


    Course course = new Course();

    // TESTING - loop through legs and run their action
    for (int i = 0; i < 13; i++) {
        Serial.print("Running ");
        Serial.println(i);
        course.legs[i]->run();
    }
    
    /*Line test = Line(500, FORWARD, 90, false);
    test.run();
    Circle testCircle = Circle(180, 270, BACKWARD, 90, true);
    testCircle.run();*/

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
