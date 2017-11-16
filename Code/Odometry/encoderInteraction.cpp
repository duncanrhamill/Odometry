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
    float dist = clicks * CLICKSTOMM;
    
    // return absolute distance moved
    return fabs(dist);
}

// reset distance encoders between legs
inline void resetEncoders() {
    Wire.beginTransmission(MD25ADDR);
    Wire.write(CMD);
    Wire.write(CLEARENCODERREGISTERS);
    Wire.endTransmission();
}

// find the average distance travelled
inline float averageDistance() {
    // get individual wheel distances
    float distLeft = individualDistance(ENCODELEFT);
    float distRight = individualDistance(ENCODERIGHT);

    // find the absolute distance
    distLeft = fabs(distLeft);
    distRight = fabs(distRight);

    // return the average
    return ((distLeft + distRight)/ 2);
}

// turns out abs doesn't like floats
/*inline float fabs(float f) {
    if (f < 0) {
        return -f;
    }
    return f;
}*/

#endif
