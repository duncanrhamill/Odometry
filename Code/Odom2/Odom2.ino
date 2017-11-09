#include <Wire.h>

void resetEncoders() {
    Wire.beginTransmission(0x58);
    Wire.write(0x10);
    Wire.write(0x20);
    Wire.endTransmission();

    delay(50);
}

float encoderLeft() {
    Wire.beginTransmission(0x58);
    Wire.write(0x02);
    Wire.endTransmission();

    Wire.requestFrom(0x58, 4);

    while (Wire.available() < 4);

    long clicks = Wire.read();
    clicks <<= 8;
    clicks += Wire.read();
    clicks <<= 8;
    clicks += Wire.read();
    clicks <<= 8;
    clicks += Wire.read();

    delay(5);

    Serial.println(clicks);

    float dist = clicks * 0.093;
    return dist;
}

float encoderRight() {
    Wire.beginTransmission(0x58);
    Wire.write(0x06);
    Wire.endTransmission();

    Wire.requestFrom(0x58, 4);

    while (Wire.available() < 4);

    long clicks = Wire.read();
    clicks <<= 8;
    clicks += Wire.read();
    clicks <<= 8;
    clicks += Wire.read();
    clicks <<= 8;
    clicks += Wire.read();

    delay(5);

    float dist = clicks * 0.093;

    return dist;
}

void drive_forward(int dist) {
    Serial.println("Drive");
    while (encoderLeft() <= dist && encoderRight() <= dist) {
        Serial.print("Left: ");
        Serial.println(encoderLeft());
        Serial.print("Right: ");
        Serial.println(encoderRight());
        
        Wire.beginTransmission(0x58);
        Wire.write(0x0F);
        Wire.write(0x02);
        Wire.endTransmission();

        Wire.beginTransmission(0x58);
        Wire.write(0x00);
        Wire.write(150);
        Wire.endTransmission();
    }

    Wire.beginTransmission(0x58);
    Wire.write(0x00);
    Wire.write(128);
    Wire.endTransmission();

    resetEncoders();
}

void stop() {
    Wire.beginTransmission(0x58);
    Wire.write(0x00);
    Wire.write(128);
    Wire.endTransmission();
}

void setup() {
  
    Serial.begin(9600);
    Serial.println("Begun");
  
    Wire.begin();

    delay(100);

    resetEncoders();
    
}

void loop() {
    drive_forward(100);

    delay(3000);

    drive_forward(50);

    delay(1500);
}
