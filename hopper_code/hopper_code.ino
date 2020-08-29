// constants *************************************************
// left and right motors
// these aren't pins, just the motor numbers
const int motor_0 = 0;
const int motor_1 = 1;

// Encoder Counts Per Revolution (CPR) = 4x Pulse Per Revolution (PPR)
const unsigned int ENCODER_CPR = 4096;
inline float rad2motorpos(float angle) {
  return angle/3.14f * 4096.0f;
}

// waypoints (in radians) that the motors should follow
// these vectors should all be the same length
// at timestep[i] (measured in milliseconds), motor_0 should be
// at waypts_0[i] rads
//const float waypts_0[] =        {  0.5,  1.0,  0.0, -0.5,  0.0, 3.14 };
const float waypts_0[] =        {  0.0,  0.5,  0.0, -0.5,  0.0,  0.5 };
const float waypts_1[] =        {  0.0,  0.0,  0.5, -0.5,  0.0,  0.5 };
const unsigned long timepts[] = { 1000, 2000, 3000, 4000, 5000, 6000 };
const unsigned int num_waypts = 6;



// pins *****************************************************
// https://docs.odriverobotics.com/interfaces#uart
const uint8_t ODRIVE_RX = 0; // odrive GPIO 1 = Tx
const uint8_t ODRIVE_TX = 1; // odrive GPIO 2 = Rx

const uint8_t ENCODER_YAW_A = 9;
const uint8_t ENCODER_YAW_B = 10;
const uint8_t ENCODER_PITCH_A = 11;
const uint8_t ENCODER_PITCH_B = 12;

const uint8_t STOP_BUTTON = 3;
volatile bool should_stop = false;
void emergency_stop() { should_stop = true; }



// ODrive ****************************************************
#include <SoftwareSerial.h>  // TODO: why am I using this instead of Serial?
#include <ODriveArduino.h>

// RX (ODrive TX), TX (ODrive RX)
SoftwareSerial odrive_serial(ODRIVE_RX, ODRIVE_TX);
ODriveArduino odrive(odrive_serial);


// Encoder ****************************************************
// https://www.pjrc.com/teensy/td_libs_Encoder.html
#include <Encoder.h>
Encoder global_yaw(ENCODER_YAW_A, ENCODER_YAW_B);
Encoder global_pitch(ENCODER_PITCH_A, ENCODER_PITCH_B);


// vs code arduino complains about not knowing what serial is otherwise
#ifdef USBCON
#define Serial Serial1
#endif

// setup ******************************************************
void setup() {
  // Serial to PC
  Serial.begin(115200);
  while (!Serial.availableForWrite()) ;  // wait for Serial Monitor to open

  pinMode(STOP_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(STOP_BUTTON), emergency_stop, LOW);
  
  odrive_serial.begin(115200);
  // Serial << "axis0: requesting encoder index search...";
  // run_state(motor_0, ODriveArduino::AXIS_STATE_ENCODER_INDEX_SEARCH);
  // Serial << "axis1: requesting encoder index search...";
  // run_state(motor_1, ODriveArduino::AXIS_STATE_ENCODER_INDEX_SEARCH);
  // delay(1000);

  // odrv0.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL??
  Serial << "Setting motors to closed loop control" << '\n';
  odrive.run_state(motor_0, ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL, false);
  odrive.run_state(motor_1, ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL, false);

  Serial << "BEGIN LOGGING" << '\n';
  Serial << "global_yaw,global_pitch" << '\n';

  // TODO: replace delay with waiting for a button event or something?
  delay(0.5);

  odrive.SetPosition(motor_0, rad2motorpos(waypts_0[0]), 1024.0f);
  odrive.SetPosition(motor_1, rad2motorpos(waypts_1[0]), 1024.0f);
}

// loop ********************************************************
void loop() {
  static unsigned int ptr = 0;             // pointer into waypts
  static unsigned long t_start = millis(); // time since first loop
  unsigned long t_loop_start = millis();   // time since loop started
  
  // read encoders and log to serial
  //Serial << global_yaw.read()/4096.0f << ',' << global_pitch.read()/4096.0f << '\n';

  // read esimated motor positions
  odrive_serial << "r axis0.encoder.pos_estimate\n";
  Serial << odrive.readFloat();
  odrive_serial << "r axis1.encoder.pos_estimate\n";
  Serial << ',' << odrive.readFloat() << '\n';
  
  // inc array pointer if past time
  if (millis() > timepts[ptr] + t_start) {
    if (ptr + 1 >= num_waypts) {
      should_stop = true;
    } else {
      ptr += 1;

      // calculate correct angle
      // ...........
      // ...........
  
      // set next position
      odrive.SetPosition(motor_0, rad2motorpos(waypts_0[ptr]), 1024.0f);
      odrive.SetPosition(motor_1, rad2motorpos(waypts_1[ptr]), 1024.0f);
    }
  }

  if (should_stop) {
    Serial << "END LOGGING" << '\n';
    Serial << "Setting motors to idle" << '\n';
    odrive.run_state(motor_0, ODriveArduino::AXIS_STATE_IDLE, false);
    odrive.run_state(motor_1, ODriveArduino::AXIS_STATE_IDLE, false);
    for (;;) {};
  }

  // 50 Hz control loop - wait for correct time
  unsigned int loop_time = millis() - t_loop_start;
  if (loop_time < 50) {
    delay(50 - loop_time);
  }
}


// other functions *******************************************
void setup_motor(int motornum) {
  //Serial << "axis" << motornum << ": requesting full calibration sequence...";
  //_run_state(motornum, ODriveArduino::AXIS_STATE_FULL_CALIBRATION_SEQUENCE);

  // Serial << "axis" << motornum << ": requesting encoder offset calibration...";
  // _run_state(motornum, ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION);

  // Serial << "axis" << motornum << ": requesting motor calibration...";
  // _run_state(motornum, ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION);

  Serial << "axis" << motornum << ": requesting encoder index search...";
  run_state(motornum, ODriveArduino::AXIS_STATE_ENCODER_INDEX_SEARCH);
  
  Serial << "axis" << motornum << ": requesting closed loop control...";
  run_state(motornum, ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL);
}

void run_state(int motornum, int requested_state) {
  int success = odrive.run_state(motornum, requested_state, true);
  if (success) {
    Serial << " success :)" << '\n';
  } else {
    Serial << " failed :(" << '\n';
  }
}

// printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) {
  obj.print(arg);    return obj;
}
template<>        inline Print& operator <<(Print &obj, float arg) {
  obj.print(arg, 4); return obj;
}

// attic ******************************************************
/*
odrive_serial << "r axis" << motor_0 << ".encoder.config.cpr\n";
Serial << "debug: " << odrive.readInt() << '\n';
*/
/*
  Serial << "Setting to position " << rad2motorpos(waypts_0[ptr])
          << " via ptr = " << ptr
          << " and angle = " << waypts_0[ptr] << "rad" << '\n';
*/
