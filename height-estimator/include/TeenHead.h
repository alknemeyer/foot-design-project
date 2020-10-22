#include <Arduino.h>
#include <QuadEncoder.h>
#include <math.h>

#include <BasicLinearAlgebra.h>
using namespace BLA;

#define PPR 4096
#define XConversion 0.64
#define LBoom 2.045 // TODO: update to actual length!

#define loopTime 500

#define Ard_baud 500000

// PIN MAPPING
#define X_enc_A 2 // 2
#define X_enc_B 3 // 3

#define TX_motor 14 // 14-A0    SERIAL 3 motor comms TX
#define RX_motor 15 // 15-A1    SERIAL 3 motor comms RX

#define TX_KF 20 // 20-A6
#define RX_KF 21 // 21-A7

#define KILL 9   // 9
#define INIT 10  // 10
#define START 19 // 19-A5

// comms to motor
#define motor_baud 250000
int motor_pos;
int motor_vel;
int motor_current;
bool commsWait();

// encode variables
float boom_X_current_pos;
float boom_X_old_pos = 0;
int32_t boom_X_CTS;

QuadEncoder
    boom_X_enc(1, X_enc_A, X_enc_B,
               0); // Enc 1, Phase A (pin0), PhaseB(pin1), Pullups Req(0)
float boom_X_current_vel;

// LiDAR variables
float Lidar;
float LidarOffset;

// MPU Variables
float Ax;
float Az;
uint16_t timeMPULast;

byte bufTemp[13] = {0};
byte message[7] = {0};

// KF Vars
#define R 0.000625
#define dAmax 3000
int KFcnt = 0;
int i = 0;
int temp, tempP;
uint16_t timePrev, timeNow;
float deltaT = (1 / 2000);

boolean AzUP = false;

BLA::Matrix<3, 3> F;
BLA::Matrix<3, 3> Q;
BLA::Matrix<3> X_bar;
BLA::Matrix<3> X_bartemp;
BLA::Matrix<1> B;
BLA::Matrix<3> u;
BLA::Matrix<3, 3> P;
BLA::Matrix<3, 3> P_bar;
BLA::Matrix<1, 3> H;
BLA::Matrix<3> Xm;

float Position = 0;

void updateF(float Fup);
void updateQ(float azCovA, float azCovB, float azCovC);

void writeBytes(int32_t data);

//-----------------------------   Encoder Functions
//--------------------------------------
void setup_encoders() {
  boom_X_enc.setInitConfig(); //
  boom_X_enc.EncConfig.revolutionCountCondition = ENABLE;
  boom_X_enc.EncConfig.enableModuloCountMode = ENABLE;
  boom_X_enc.EncConfig.positionModulusValue = PPR;
  boom_X_enc.EncConfig.positionInitialValue = 0;
  boom_X_enc.init();
}

void get_encoder_values(float deltaT) {
  // get encoder values
  boom_X_old_pos = boom_X_current_pos;

  boom_X_CTS = boom_X_enc.read();

  // Check direction
  if (boom_X_CTS > 2048) {
    boom_X_CTS = boom_X_CTS - 4096;
  }

  // Convert to planar
  boom_X_current_pos =
      ((LBoom) * (boom_X_CTS * M_PI * (44 / 28) / PPR)) / XConversion;

  // calculate encoder vel
  boom_X_current_vel = (boom_X_current_pos - boom_X_old_pos) / deltaT;
}

// ------------------------------- KF Data Functions
// ------------------------------------------- Check what serial port on the
// teensy you are using for arduino comms
void setup_Ard_comms() {

  Serial5.begin(Ard_baud);
  Serial5.flush();
}

// Reads data from arduino to global vars, returns 1 for lidar, 2 for Az,
// 3 for Ax with preference. Reads till serial buffer is empty
int read_Ard() {

  int dat = -1;
  while (Serial5.available()) {
    tempP = temp;
    temp = Serial5.read();
    bufTemp[i] = temp;
    if (temp == 88 && tempP == 88) {
      if (i - 7 < 0) {
        memcpy(message, bufTemp + 13 + (i - 7), 7 - i);
        memcpy(message + 7 - i, bufTemp, i);
      } else {
        memcpy(message, bufTemp + (i - 7), 7);
      }

      byte buftemp[4] = {0};
      memcpy(buftemp, message + 1, sizeof(buftemp));
      float x = *((float *)(buftemp));

      if (message[0] == 0) {
        if (x < 300 && x > -0.1) {
          Lidar = x;
          Lidar = Lidar / 100;
          dat = 0;
        } else {
          Serial5.flush();
        }
      } else if (message[0] == 1) {
        if (Az < 100) {
          Az = x;
          if (dat != 0) {
            dat = 1;
          }
          AzUP = true;
        }
      } else if (message[0] == 2) {
        if (Ax < 16.5) {
          Ax = x;
          if (dat != 0 || dat != 1) {
            dat = 2;
          }
        }
      }
      int count = 0;
      for (uint8_t j = 0; j < sizeof(message); j++) {
        if (message[j] & 0b1) {
          count++;
        }
      }
    }
    i++;
    if (i > 13) {
      i = 0;
    }
  }
  return dat;
}

// ---------------------------------- KF Compute Functions
// -------------------------------------- Sets up and initalised KF matrices,
// Also computes the Lidar offset When using the KF data:
//          X_bar(0) -> Zpos
//          X_bar(1) -> Zvel
//          X_bar(2) -> Zacc

void setup_KF() {
  F.Fill(0);
  Q.Fill(0);
  X_bar.Fill(0);
  X_bartemp.Fill(0);
  B.Fill(0);
  P.Fill(0);
  H.Fill(0);
  Xm.Fill(0);

  B(0) = 1;

  P << 0.002, 0, 0, 0, 0.001, 0, 0, 0, 0.001;

  H(0, 0) = 1;
  H(0, 1) = 0;
  H(0, 2) = 0;

  int temp = read_Ard();
  float LidarTot = 0;

  for (int j = 0; j < 10; j++) {
    while (temp != 0) {
      temp = read_Ard();
    }
  }
  for (int k = 0; k < 100; k++) {
    while (temp != 0) {
      temp = read_Ard();
    }
    LidarTot += Lidar;
  }
  LidarOffset = LidarTot / 100;

  // LidarOffset = 0.25;
  Xm(0) = (Lidar - LidarOffset);
  Xm(1) = 0;
  Xm(2) = 0;

  updateF(0);
  updateQ(0, 0, 0);
}

// Internal KF functions Values precomputed for efficency see comment for maths
void updateQ(float azCovA, float azCovB, float azCovC) {
  /*Q <<  pow(deltaT,4)/4,  pow(deltaT,3)/2, pow(deltaT,2)/2,
        pow(deltaT,3)/2,  pow(deltaT,2),   deltaT,
        azCovA,           azCovB,          azCovC;*/
  Q << 1.5625e-14, 6.25e-11, 1.25e-7, 6.25e-11, 1.25e-7, 0.0005, azCovA, azCovB,
      azCovC;
}
void updateF(float Fup) {
  /*F <<  1,  deltaT, pow(deltaT,2)/2,
        0,  1,      deltaT,
        0,  0,      Fup;*/
  F << 1, 0.0005, 1.25e-7, 0, 1, 0.0005, 0, 0, Fup;
}

void predictKF() {
  //----------------------------Predict--------------------------
  float AZtemp = -Az * 9.81;
  u(0) = 0;
  u(1) = 0;
  u(2) = AZtemp;
  X_bar = F * Xm + u;

  BLA::Matrix<3, 3> P_bartemp;
  BLA::Matrix<3, 3> Ft = ~F;

  P_bar = F * P * Ft;
  float daTemp = 0;
  float azCovA, azCovB, azCovC;
  if (Az > 14 || AZtemp > -5) {
    azCovA = pow(deltaT, 2) / 2;
    azCovB = deltaT;
    azCovC = 1;
    daTemp = 3000; // 100000;
  } else {
    azCovA = 0;
    azCovB = 0;
    azCovC = 1.26736e-7 / 3000;
    daTemp = 500; // 3000;
  }
  updateQ(azCovA, azCovB, azCovC);
  Q = Q * daTemp; // dAmax;
  P_bar += Q;
}

void updateKF(int dat) {
  //--------------------------------Update------------------------------
  BLA::Matrix<3, 3> Eye3 = {1, 0, 0, 0, 1, 0, 0, 0, 1};

  if (dat == 0 && (Lidar - LidarOffset) > -0.2) // KFcnt<4)
  {
    BLA::Matrix<1> y = {Lidar - LidarOffset};
    y -= H * (X_bar);
    BLA::Matrix<3> K;
    BLA::Matrix<3> Ht = ~H;

    BLA::Matrix<1> temp = H * P_bar * Ht;
    temp += R;
    temp = temp.Inverse();
    K = P_bar * Ht;
    K = K * temp;

    P = (Eye3 - K * H) * P_bar;
    Xm = X_bar + K * y;
    KFcnt = 0;
  } else {
    Xm = X_bar;
    P = Eye3 * P_bar;

    KFcnt++;
  }
}

// ------------------------------ Motor Comms ---------------------------
void setup_motor_comms() {
  Serial3.begin(motor_baud);
  Serial3.flush();
}

// Motor comms
// (0, PosCommand)    -> Position Control ignore return its legacy
// (1, VelCommand)    -> Velocity Control (untested) ignore return its legacy
// (2, TorqueCommand) -> Torque Control ignore return its legacy
// (3)                -> get Pos

float motorCom(uint8_t cmdType) {
  Serial3.write(cmdType);

  bool readtest = commsWait();

  byte bufTemp[4] = {0};
  if (readtest == 0) {
    for (int i = 0; i < 4; i++) {
      bool readtest = commsWait();
      if (readtest == 0) {
        bufTemp[i] = Serial3.read();
      } else {
        break;
      }
    }
  }

  float x = *((float *)(bufTemp));
  return x;
}

float motorCom(uint8_t cmdType, float cmdVal) {
  Serial3.write(cmdType);
  float tempSend = cmdVal;
  byte *dataByte = (byte *)&tempSend;
  byte buffer[4] = {0};

  memcpy(buffer, dataByte, 4);

  Serial3.write(buffer, 4);
  return 1;
}

bool commsWait() {
  uint16_t currtime = micros();
  bool comTimeout = false;
  while (Serial3.available() == 0 && comTimeout == false) {
    if (micros() - currtime > 2000) {
      comTimeout = true;
    }
  }
  return comTimeout;
}
