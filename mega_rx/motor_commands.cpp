#include "motor_commands.h"
//-------------------------------------------------------------------
void handle_motor_command(t_ODriveArduino &odrive, char c)
{
    // Run calibration sequence
    if (c == '0' || c == '1') {
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
      odrive.SetCurrent(0, M0_DIR * max_current);
      if(!odrive.run_state(1, requested_state, false /*don't wait*/)) return;
      odrive.SetCurrent(1, M1_DIR * max_current);

      Serial.println("Executing forward");
      //odrive.SetPosition(0, 1);
      //odrive.SetVelocity(0, 1);
      delay(5);
    }

    if ((c == 'q') && (robot_state != robot_state_moving_forward_left)){
      robot_state = robot_state_moving_forward_left;
      
      int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;
      odrive.SetCurrent(0, M0_DIR * max_current);// +  CURVE_DIFF));
      if(!odrive.run_state(1, requested_state, false /*don't wait*/)) return;
      odrive.SetCurrent(1, 0);//M1_DIR * max_current);

      Serial.println("Moving forward left");
      delay(5);
    }

    if ((c == 'w') && (robot_state != robot_state_moving_forward_right)){
      robot_state = robot_state_moving_forward_right;
      
      int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;
      odrive.SetCurrent(0, 0);//M0_DIR * max_current);
      if(!odrive.run_state(1, requested_state, false /*don't wait*/)) return;
      odrive.SetCurrent(1, M1_DIR * max_current);// + CURVE_DIFF));

      Serial.println("Moving forward right");
      delay(5);
    }

    if ((c == 'b') && (robot_state != robot_state_moving_backward)) {
      robot_state = robot_state_moving_backward;

      int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;
      odrive.SetCurrent(0, -M0_DIR*max_current);
      if(!odrive.run_state(1, requested_state, false /*don't wait*/)) return;
      odrive.SetCurrent(1, -M1_DIR*max_current);

      Serial.println("Executing backward");
      delay(5);
    }

    if ((c == 'a') && (robot_state != robot_state_moving_back_left)){
      robot_state = robot_state_moving_back_left;
      
      int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;
      odrive.SetCurrent(0, -M0_DIR*max_current);// + CURVE_DIFF));
      if(!odrive.run_state(1, requested_state, false /*don't wait*/)) return;
      odrive.SetCurrent(1, 0);//-M1_DIR*max_current);

      Serial.println("Moving back left");
      delay(5);
    }

    if ((c == 's') && (robot_state != robot_state_moving_back_right)){
      robot_state = robot_state_moving_back_right;
      
      int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;
      odrive.SetCurrent(0, 0);//-M0_DIR * max_current);
      if(!odrive.run_state(1, requested_state, false /*don't wait*/)) return;
      odrive.SetCurrent(1, -M1_DIR * max_current);// + CURVE_DIFF));

      Serial.println("Moving back right");
      delay(5);
    }

    if ((c == 'l') && (robot_state != robot_state_spin_left)){
      robot_state = robot_state_spin_left;
      
      int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;
      odrive.SetCurrent(0, M0_DIR*max_current);
      if(!odrive.run_state(1, requested_state, false /*don't wait*/)) return;
      odrive.SetCurrent(1, -M1_DIR*max_current);

      Serial.println("Rotating left");
      delay(5);
    }

    if ((c == 'r') && (robot_state != robot_state_spin_right)){
      robot_state = robot_state_spin_right;
      
      int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;
      odrive.SetCurrent(0, -M0_DIR*max_current);
      if(!odrive.run_state(1, requested_state, false /*don't wait*/)) return;
      odrive.SetCurrent(1, M1_DIR*max_current);

      Serial.println("Rotating right");
      delay(5);
    }


    if ((c == 'z') && (robot_state != robot_state_stopped) && (robot_state != robot_state_idle)) {
      robot_state = robot_state_stopped;

      int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;
      odrive.SetCurrent(0, 0.0);
      if(!odrive.run_state(1, requested_state, false /*don't wait*/)) return;
      odrive.SetCurrent(1, 0.0);

      Serial.println("Stop");
      delay(5);
    }

    if ((c == 'h') && (robot_state != robot_state_hold_position)) {
      // stop first
      int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;
      odrive.SetCurrent(0, 0.0);
      if(!odrive.run_state(1, requested_state, false /*don't wait*/)) return;
      odrive.SetCurrent(1, 0.0);
      delay(5);

// move to position control
      robot_state = robot_state_hold_position;
      for (int m = 0; m < 2; ++m){
        odrive.serial_ << "w axis" << m << ".controller.control_mode " << CONTROL_MODE_POSITION_CONTROL << '\n';
        delay(5);
      }

      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;
      float pos = odrive.GetPosition(0);
      odrive.SetPosition(0, pos);
      if(!odrive.run_state(1, requested_state, false /*don't wait*/)) return;
      pos = odrive.GetPosition(1);
      odrive.SetPosition(1, pos);

      Serial.println("Hold position");
      delay(5);
    }

    if ((c == 'i') && (robot_state != robot_state_idle) && (robot_state != robot_state_stopped)) {
      int requested_state = AXIS_STATE_IDLE;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;
      if(!odrive.run_state(1, requested_state, false /*don't wait*/)) return;
     // odrive.SetCurrent(0, 0.0);
      robot_state = robot_state_idle;
    }

    // Read bus voltage
    if (c == 'g') {
      odrive.serial_ << "r vbus_voltage\n";
      Serial << "Vbus voltage: " << odrive.readFloat() << '\n';
      Serial << "Speed= " << odrive.GetVelocity(0) << '\n';
      Serial << "Position=" << odrive.GetPosition(0) << '\n';
      Serial << "Max current=" << max_current << '\n';
      Serial << "Robot state=" << robot_state_stopped << '\n';
    }

    if (c == 'c') {
      odrive.clear_errors();
      max_current = default_max_power;
    }

    if (c == 'u') {
        if (max_current < 3)
          max_current+= 0.1;
        Serial << "Current=" << max_current << '\n';
    }

    if (c == 'd') {
        if (max_current >= 0.1)
          max_current -= 0.1;
        Serial << "Current=" << max_current << '\n';
    }

}
//-------------------------------------------------------------------
