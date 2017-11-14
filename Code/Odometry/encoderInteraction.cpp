/*
 *  Encoder interaction file, contains functions to read and clear MD25 encoders
 *          Uses inline functions to prevent multiple definitions
 */

#include "defines.h"

#include <Arduino.h>
#include <Wire.h>

// include guard
#ifndef ENCODERINTERACTION_CPP
#define ENCODERINTERACTION_CPP

// find distance a specific wheel has moved
inline int individualDistance(char side) {
    // set MD25 to send the encoder for the given side
    Wire.beginTransmission(MD25ADDR);
    Wire.write(side);
    Wire.endTransmission();

    // request 4 bytes from the MD25
    Wire.requestFrom(MD25ADDR, 4);

    // wait for first 4 bytes back
    while (Wire.available() < 4);

    // get all bytes of the click var
    long clicks = Wire.read();
    clicks <<= 8;
    clicks += Wire.read();
    clicks <<= 8;
    clicks += Wire.read();
    clicks <<= 8;
    clicks += Wire.read();

    delay(5);

    // convert clicks to mm
    int dist = clicks * CLICKSTOMM;
    
    // return absolute distance moved
    return abs(dist);
}

// reset distance encoders between legs
inline void resetEncoders() {
    Wire.beginTransmission(MD25ADDR);
    Wire.write(CMD);
    Wire.write(CLEARENCODERREGISTERS);
    Wire.endTransmission();
    delay(50);
}

// find the average distance travelled
inline int averageDistance() {
    // get individual wheel distances
    int distLeft = individualDistance(ENCODELEFT);
    int distRight = individualDistance(ENCODERIGHT);

    // find the absolute distance
    distLeft = abs(distLeft);
    distRight = abs(distRight);

    // return the average
    return (int)((distLeft + distRight)/ 2);
}

#endif
