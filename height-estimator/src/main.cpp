// originally written by Josh van Zyl and Callen Fisher
// refactored and modified for this project by Alexander Knemeyer
#include <BasicLinearAlgebra.h>
#include <TeenHead.h>
#include <math.h>

void init_params();
void wait_control_loop();

bool doneinit = false;

// control loop variables -> 2kHz control loop
// if you enconter problems, slow down
unsigned long looptime_us = 500;
unsigned long prevtime_us = 0;
unsigned long evaltime_us = 0;

// main arduino setup() function
void setup() {
  setup_laptop_comms();
  // while (true) { laptop_comm(); delay(1000); }
  setup_encoders();
  setup_arduino_comms();
}

// main arduino loop() function
void loop() {
  // hold until initalise button is pressed
  while (doneinit == false) {
    float lidarInit = 0;
    init_params();

    for (int h = 0; h < 10; h++) {
      while (X_bar(0) < -0.001 || X_bar(0) > 0.001) {
        init_params();
        for (int k = 0; k < 100; k++) {
          int readstat = -1;
          while (readstat == -1) {
            readstat = read_arduino();
          }
          predictKF();
          updateKF(readstat);
          wait_control_loop();
        }
      }
      lidarInit = lidarInit + LidarOffset;
    }
    LidarOffset = lidarInit / 10;
    doneinit = true;
  }

  // read data from encoder and arduino
  get_encoder_values(0.0005);

  int readstat = -1;
  uint32_t timeread = micros();
  while (readstat == -1 && ((micros() - timeread) < 450)) {
    readstat = read_arduino();
  }

  //-------------------Run KalmanFilter--------------------------
  predictKF();
  updateKF(readstat);

  //----------------------Controller-----------------------------
  laptop_comm();

  // wait for control loop to be finished
  wait_control_loop();
}

//-----------------------------------------------------------------------------------------
//----------------------------------Auxiliary Functions------------------------------------
//-----------------------------------------------------------------------------------------

void wait_control_loop() {
  evaltime_us = prevtime_us + looptime_us;
  while (micros() < evaltime_us) {}
  prevtime_us = micros();
}

void init_params() {
  boom_X_enc.write(0);

  Serial.flush();
  ArdSerial.flush();

  setup_KF();
}
