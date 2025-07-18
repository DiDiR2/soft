#define s_version "M.2025.07.17.0"
//-------------------------------------------------------------------
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>
//-------------------------------------------------------------------
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }
// ODRIVE variables
HardwareSerial& odrive_serial = Serial1;

ODriveArduino odrive(odrive_serial);
//-------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }
 
  Serial.print("Version: ");
  Serial.println(s_version);

  Serial.println("Connecting to ODrive ...");
  odrive_serial.begin(115200);
  while (!odrive_serial) {
    // some boards need to wait to ensure access to serial over USB
  }
  Serial.println("ODrive alive ...");
  for (int m = 0; m < 2; ++m) {
      int requested_state = AXIS_STATE_IDLE;
      Serial << "Axis" << m << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(m, requested_state, false /*don't wait*/)) 
        return;
      odrive_serial << "w axis" << m << ".controller.config.input_mode " << INPUT_MODE_PASSTHROUGH << '\n';
      odrive_serial << "w axis" << m << ".controller.config.control_mode " << CONTROL_MODE_VELOCITY_CONTROL << '\n';
      odrive_serial << "w axis" << m << ".controller.config.vel_limit " << 3.0 << '\n';
      odrive_serial << "w axis" << m << ".motor.config.current_lim " << 30.0f << '\n';
  }
}

void loop() 
{
  if (Serial.available()){
    char c = Serial.read();
    if (c =='f'){// go forward
      int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;
      odrive.SetVelocity(0, 1.0);
      if(!odrive.run_state(1, requested_state, false /*don't wait*/)) return;
      odrive.SetVelocity(1, 1.0);
    }
    else
    if (c =='z'){// stop
      int requested_state = AXIS_STATE_IDLE;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;
      odrive.SetVelocity(0, 0);
      if(!odrive.run_state(1, requested_state, false /*don't wait*/)) return;
      odrive.SetVelocity(1, 0);
    }
    else
    if (c =='c'){// clear errors
      odrive_serial << "sc\n";
    }
    else
      Serial << "Invalid command " << c << '\n';
  }
// read control mode
  for (int m = 0; m < 2; ++m) {
    odrive_serial << "r axis" << m << ".controller.config.control_mode\n";
    Serial << "Control mode of axis" << m << " " << odrive.readInt() << '\n';
  }
  Serial.println(" ");
  delay (100);
}