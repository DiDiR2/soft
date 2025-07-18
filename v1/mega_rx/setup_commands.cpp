#include "setup_commands.h"
#include "print_commands.h"
#include "motor_commands.h"
//-------------------------------------------------------------------
void setup_sensors(Adafruit_LSM6DSOX &sox)
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
void setup_wifi(RF24 &radio, uint8_t address[][6])
{
  Serial.print(F("radioNumber = 1"));
  //Serial.println((int)radioNumber);
 
  // role variable is hardcoded to RX behavior, inform the user of this
  Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));
 
  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
 
  // save on transmission time by setting the radio to only transmit the
  // number of bytes we need to transmit a float
  radio.setPayloadSize(sizeof(char));  // float datatype occupies 4 bytes
 
  // set the TX address of the RX node for use on the TX pipe (pipe 0)
  radio.stopListening(address[1]);  // put radio in TX mode
 
  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[0]);  // using pipe 1
 
  // additional setup specific to the node's RX role
  radio.startListening();  // put radio in RX mode
}
//-------------------------------------------------------------------
void setup_odrive(t_ODriveArduino &odrive)
{
    // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters

  for (int m = 0; m < 2; ++m) {
    odrive.serial_ << "w axis" << m << ".controller.config.vel_limit " << 3.0 << '\n';
    odrive.serial_ << "w axis" << m << ".motor.config.current_lim " << 30.0f << '\n';

    odrive.serial_ << "w axis" << m << ".controller.config.input_mode " << INPUT_MODE_PASSTHROUGH << '\n';
    odrive.serial_ << "w axis" << m << ".controller.config.control_mode " << CONTROL_MODE_VELOCITY_CONTROL << '\n';

    odrive.serial_ << "w axis" << m << ".controller.config.enable_overspeed_error " << false << '\n';
  }
}
//-------------------------------------------------------------------
