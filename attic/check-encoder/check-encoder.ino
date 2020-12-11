/* Encoder Library - TwoKnobs Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder knobLeft(9, 10);
//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
  Serial.println("TwoKnobs Encoder Test:");
}

long positionLeft  = -999;

void loop() {
  long newLeft;
  newLeft = knobLeft.read();
  if (newLeft != positionLeft) {
    Serial.print("Left = ");
    Serial.print(newLeft);
    Serial.println();
    positionLeft = newLeft;
  }
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    knobLeft.write(0);
  }
}

// // Encoder Counts Per Revolution (CPR) = 4x Pulse Per Revolution (PPR)
// const unsigned int ENCODER_CPR = 500*2;
// inline float rad2motorpos(float angle) {
//   return angle * 180.0f / 3.14f * 2000.0f;
// }

// // Encoder ****************************************************
// // https://www.pjrc.com/teensy/td_libs_Encoder.html
// #include <Encoder.h>

// const uint8_t ENCODER_A = 9;
// const uint8_t ENCODER_B = 10;
// const uint8_t ENCODER_Z = 11;

// Encoder encoder(ENCODER_A, ENCODER_B);

// // setup ******************************************************
// void setup() {
//   // Serial to PC
//   Serial.begin(115200);
//   while (!Serial.availableForWrite()) ;  // wait for Serial Monitor to open
//   Serial << "angle" << '\n';
//   delay((uint32_t) 1000);
// }

// // loop ********************************************************
// void loop() {
//     // this is how you'd read the encoders and log to serial:
//     Serial << encoder.read()/2000.0f << '\n';
//     delay((uint32_t) 1000);
// }

// // // printing with stream operator
// // template<class T> inline Print& operator <<(Print &obj,     T arg) {
// //   obj.print(arg);
// //   return obj;
// // }

// // template<>        inline Print& operator <<(Print &obj, float arg) {
// //   obj.print(arg, 4);
// //   return obj;
// // }
