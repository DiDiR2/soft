/*
 * See documentation at https://nRF24.github.io/RF24
 * See License information at root directory of this library
 * Author: Brendan Doherty (2bndy5)
 */

#define s_version "M.2025.07.05.3"

#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>
#include <SPI.h>
#include <Adafruit_LSM6DSOX.h>
#include <utility/imumaths.h>

#include "printf.h"
#include "RF24.h"


// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }
//-------------------------------------------------------------------
// WIFI variables
#define CE_PIN 9
#define CSN_PIN 53

// instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);
 
// Let these addresses be used for the pair
uint8_t address[][6] = { "1Node", "2Node" };
// It is very helpful to think of an address as a path instead of as
// an identifying device destination
 
// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 1;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit
 
// Used to control whether this node is sending or receiving
//bool wifi_role = false;  // true = TX role, false = RX role
 
// For this example, we'll be using a payload containing
// a single float number that will be incremented
// on every successful transmission
//int joy_Y = 0;
char c_payload;
//-------------------------------------------------------------------
// ODRIVE variables
HardwareSerial& odrive_serial = Serial1;

ODriveArduino odrive(odrive_serial);
//-------------------------------------------------------------------
// SENSORS variables
Adafruit_LSM6DSOX sox;
//-------------------------------------------------------------------
// OTHERS
#define robot_state_stopped 0
#define robot_state_moving_forward 1
#define robot_state_moving_backward 2

int robot_state = robot_state_stopped;

//-------------------------------------------------------------------
void setup_sensors()
{
    Serial.print("Accelerometer range set to: ");

  switch (sox.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  Serial.print("Accelerometer data rate set to: ");

  switch (sox.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }
}
//-------------------------------------------------------------------
void setup_wifi()
{
    Serial.print(F("radioNumber = "));
  Serial.println((int)radioNumber);
 
  // role variable is hardcoded to RX behavior, inform the user of this
  Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));
 
  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
 
  // save on transmission time by setting the radio to only transmit the
  // number of bytes we need to transmit a float
  radio.setPayloadSize(sizeof(c_payload));  // float datatype occupies 4 bytes
 
  // set the TX address of the RX node for use on the TX pipe (pipe 0)
  radio.stopListening(address[radioNumber]);  // put radio in TX mode
 
  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[!radioNumber]);  // using pipe 1
 
  // additional setup specific to the node's RX role
  radio.startListening();  // put radio in RX mode
}
//-------------------------------------------------------------------
void setup_odrive()
{
    // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters

  for (int axis = 0; axis < 2; ++axis) {
    odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << 2.0f << '\n';
    odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 11.0f << '\n';
    // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
  }

  Serial.println("Ready!");
  Serial.println("Send the character '0' or '1' to calibrate respective motor (you must do this before you can command movement)");
  Serial.println("Send the character 'f' to move forward");
  Serial.println("Send the character 'b' to move backward");
  Serial.println("Send the character 'z' to move to 0");
  Serial.println("Send the character 'v' to read bus voltage");
  Serial.println("Send the character 's' to read speed");
  Serial.println("Send the character 'p' to read motor position");
  Serial.println("Send the character 'i' to idle motor");
}
//-------------------------------------------------------------------
void setup() {
 
  Serial.begin(115200);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }
 
  Serial.print("Version: ");
  Serial.println(s_version);

  // sensors setup
  Serial.println("Connecting to LSM6DSOX sensor ...");
   if (!sox.begin_I2C()) {
    // if (!sox.begin_SPI(LSM_CS)) {
    // if (!sox.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    // Serial.println("Failed to find LSM6DSOX chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("LSM6DSOX alive!");

  setup_sensors();

// wifi setup
Serial.println("Connecting to wifi ...");
 // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    while (1) {}  // hold in infinite loop
  }
  Serial.println(F("wifi alive!"));

  setup_wifi();

  Serial.println("Connecting to ODrive ...");
  odrive_serial.begin(115200);
  while (!odrive_serial) {
    // some boards need to wait to ensure access to serial over USB
  }
  Serial.println("ODrive alive ...");

  Serial.println("Setting ODRIVE parameters...");
  setup_odrive();

  robot_state = robot_state_stopped;
/*
      int motornum = 0;
      int requested_state;
      requested_state = AXIS_STATE_MOTOR_CALIBRATION;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(motornum, requested_state, true)) return;

      requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(motornum, requested_state, true, 25.0f)) return;
*/

}  // setup
//-------------------------------------------------------------------
void handle_motor_command(char c)
{
    // Run calibration sequence
    if (c == '0') {
      int motornum = c-'0';
      int requested_state;

      requested_state = AXIS_STATE_MOTOR_CALIBRATION;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(motornum, requested_state, true)) return;

      requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(motornum, requested_state, true, 25.0f)) return;
    }

    if ((c == 'f') && (robot_state != robot_state_moving_forward)){
      robot_state = robot_state_moving_forward;
      
      int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;

      Serial.println("Executing forward");
      //odrive.SetPosition(0, 1);
      odrive.SetVelocity(0, 1);
      delay(5);
    }

    if ((c == 'b') && (robot_state != robot_state_moving_backward)) {
      robot_state = robot_state_moving_backward;

      int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;

      Serial.println("Executing backward");
      //odrive.SetPosition(0, -1);
      odrive.SetVelocity(0, -1);
      delay(5);
    }

    if ((c == 'z') && (robot_state != robot_state_stopped)) {
      robot_state = robot_state_stopped;

      int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;

      Serial.println("Executing zero");
      //odrive.SetPosition(0, 0);
      odrive.SetVelocity(0, 0);
      delay(5);
    }

    // Read bus voltage
    if (c == 'v') {
      odrive_serial << "r vbus_voltage\n";
      Serial << "Vbus voltage: " << odrive.readFloat() << '\n';
    }

    if (c == 's') {
      Serial << "Speed= " << odrive.GetVelocity(0) << '\n';
    }
    // print motor position
    if (c == 'p') {
          Serial << "Position=" << odrive.GetPosition(0) << '\n';
    }

    if (c == 'i') {
      int requested_state = AXIS_STATE_IDLE;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;
    }
}
//-------------------------------------------------------------------
void loop() {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    sox.getEvent(&accel, &gyro, &temp);
    /* Display the results (acceleration is measured in m/s^2) */
    /*
    Serial.print("\t\tAccel X: ");
    Serial.print(accel.acceleration.x);
    Serial.print(" \tY: ");
    Serial.print(accel.acceleration.y);
    Serial.print(" \tZ: ");
    Serial.print(accel.acceleration.z);
    Serial.println(" m/s^2 ");
*/
    if (accel.acceleration.x > 2)
      handle_motor_command('f');
    else
      if (accel.acceleration.x < -2)
        handle_motor_command('b');
      else
        handle_motor_command('z');
    // This device is a RX node
 
    uint8_t pipe;
    if (radio.available(&pipe)) {              // is there a payload? get the pipe number that received it
      uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
      radio.read(&c_payload, bytes);             // fetch payload from FIFO
      Serial.print(F("Received "));
      Serial.print(bytes);  // print the size of the payload
      Serial.print(F(" bytes on pipe "));
      Serial.print(pipe);  // print the pipe number
      Serial.print(F(": "));
      Serial.println(c_payload);  // print the payload's value
      handle_motor_command(c_payload);
    }

    delay (200);
 
}  // loop