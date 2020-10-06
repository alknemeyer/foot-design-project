#include <BasicLinearAlgebra.h>
#include <QuadEncoder.h>
#include <TeenHeadAlex.h>
#include <math.h>


//---------------------------------------------------------------------------------------------------
//----------------------------------Defines for useful constants-------------------------------------
//---------------------------------------------------------------------------------------------------
void init_params();
void wait_control_loop();
void run_controller();
void horzontal_controller();
void testPos();
//-------------------------------------control loop variables----------------------------------------
#define loopTime 500 // Gives control loop 2kHz Frequency
unsigned long previousTime = 0;
unsigned long evalTime = 500;

bool doneInit = false;
bool started = false;
bool dead = false;

int MotorCnt = 0;
float motorPosition;
//-------------------------------------------------------------------------------------
//----------------------------------SetUp Functions------------------------------------
//-------------------------------------------------------------------------------------

void setup() {
  setup_motor_comms();
  setup_encoders();
  setup_Ard_comms();
}

//-------------------------------------------------------------------------------
//----------------------------------Main Loop------------------------------------
//-------------------------------------------------------------------------------

void loop() {
  //------------------Wait for initialisation-------------------------
  // Holds till initalise button is pressed and then initialise the system
  while (doneInit == false) {
    if (digitalRead(INIT) == LOW) {
      float lidarInit = 0;
      init_params();

      for (int h = 0; h < 10; h++) {
        while (X_bar(0) < -0.001 || X_bar(0) > 0.001) {
          init_params();
          for (int k = 0; k < 100; k++) {
            int ReadStat = -1;
            while (ReadStat == -1) {
              ReadStat = read_Ard();
            }
            predictKF();
            updateKF(ReadStat);
            wait_control_loop();
          }
        }
        lidarInit = lidarInit + LidarOffset;
      }
      LidarOffset = lidarInit / 10;

      doneInit = true;
    }
  }

  //--------------------Hold for system start-------------------------
  // Holds in the while loop till the start button is pressed
  while (started == false) {
    if (digitalRead(START) == LOW) {
      started = true;
    }
  }

  //---------------------Get Data--------------------------------
  // Reads data from the various sensors+
  get_encoder_values(0.0005);
  int ReadStat = -1;
  uint32_t timeread = micros();
  while (ReadStat == -1 && ((micros() - timeread) < 450)) {
    ReadStat = read_Ard();
  }

  //-------------------Run KalmanFilter--------------------------
  predictKF();
  updateKF(ReadStat);
  //----------------------Controller-----------------------------
  // check if control killed:
  if (digitalRead(KILL) == LOW) {
    dead = true;
    motorCom(2, 0);
  } else {
    run_controller();
  }
  while (dead) {
    // Hang in dead state
  }

  // comms to motor runs at 500Hz if you enconter problems slow down
  if (MotorCnt >= 3) {
    motorCom(0, motorPosition);
    float tempPosition = motorCom(3);
    if (-1.5 < tempPosition < 1.5) {
      tempPosition = -tempPosition;
    } else {
      tempPosition = Position;
    }
    Position = tempPosition;
    MotorCnt = 0;
  } else {
    MotorCnt++;
  }

  // wait for control loop to be finished
  wait_control_loop();
}

//---------------------------------------------------------------------------------
//----------------------------------Conteroller------------------------------------
//---------------------------------------------------------------------------------
void run_controller() {}

//-----------------------------------------------------------------------------------------
//----------------------------------Auxiliary Functions------------------------------------
//-----------------------------------------------------------------------------------------

void wait_control_loop() {
  evalTime = previousTime + loopTime;
  while (micros() < evalTime) {
  }
  previousTime = micros();
}
void init_params() {
  boom_X_enc.write(0);

  Serial5.flush();
  Serial3.flush();

  setup_KF();
}
