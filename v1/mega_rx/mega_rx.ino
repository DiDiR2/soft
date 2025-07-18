#define s_version "M.2025.07.18.1"
//-------------------------------------------------------------------
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <SPI.h>

#include <Adafruit_LSM6DSOX.h>
#include "ODriveArduino_my.h"
#include "RF24.h"

#include "motor_commands.h"
#include "setup_commands.h"
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
//bool radioNumber = 1;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit
 
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

t_ODriveArduino odrive(odrive_serial);
//-------------------------------------------------------------------
// SENSORS variables
Adafruit_LSM6DSOX sox;
//-------------------------------------------------------------------
// OTHERS
int robot_state;

float max_current;
//-------------------------------------------------------------------
void setup() {
 
  Serial.begin(115200);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }
 
  Serial.print("Version: ");
  Serial.println(s_version);
/*
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

  setup_sensors(sox);
*/

// wifi setup
Serial.println("Connecting to wifi ...");
 // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    while (1) {}  // hold in infinite loop
  }
  Serial.println(F("wifi alive!"));

  setup_wifi(radio, address);
// ODrive setup
  Serial.println("Connecting to ODrive ...");
  odrive_serial.begin(115200);
  while (!odrive_serial) {
    // some boards need to wait to ensure access to serial over USB
  }
  Serial.println("ODrive alive ...");

  Serial.println("Setting ODRIVE parameters...");
  setup_odrive(odrive);
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
  robot_state = robot_state_stopped;

  max_current = init_torque_or_speed;

  odrive.SetVelocity(0, 0);// stop the motors
  odrive.SetVelocity(1, 0);

  Serial.println("Ready!");
  Serial.println("Send the character '0' or '1' to calibrate respective motor (you must do this before you can command movement)");

  Serial.println("Send the character 'f' to move forward");
  Serial.println("Send the character 'b' to move backward");
  Serial.println("Send the character 'l' to spin to the left");
  Serial.println("Send the character 'r' to spin to the right");
  Serial.println("Send the character 'q' to drive to forward left");
  Serial.println("Send the character 'w' to drive to forward right");
  Serial.println("Send the character 'a' to drive to backward left");
  Serial.println("Send the character 's to drive to backward right");

  Serial.println("Send the character 'z' to move to 0");
  Serial.println("Send the character 'i' to idle motor");

  Serial.println("Send the character 'g' to read bus voltage, position, speed, current");
  Serial.println("Send the character 'u' to increase power");
  Serial.println("Send the character 'd' to decrease power");
}  // setup
//-------------------------------------------------------------------
void loop() 
{
  /*
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    sox.getEvent(&accel, &gyro, &temp);
    */
// Display the results (acceleration is measured in m/s^2) 
/*
    Serial.print("\t\tAccel X: ");
    Serial.print(accel.acceleration.x);
    Serial.print(" \tY: ");
    Serial.print(accel.acceleration.y);
    Serial.print(" \tZ: ");
    Serial.print(accel.acceleration.z);
    Serial.println(" m/s^2 ");
*/
/*
    if (accel.acceleration.x > 2)
      handle_motor_command('f');
    else
      if (accel.acceleration.x < -2)
        handle_motor_command('b');
      else
        handle_motor_command('z');
*/ 
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

      handle_motor_command_velocity_control(odrive, c_payload);
    }

    delay (10);
}  // loop