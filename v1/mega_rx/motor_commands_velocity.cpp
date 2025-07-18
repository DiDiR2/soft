#include "motor_commands.h"
#include "print_commands.h"
//-------------------------------------------------------------------
void handle_motor_command_velocity_control(t_ODriveArduino &odrive, char c)
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

      return;
    }

    if ((c == 'f') && (robot_state != robot_state_moving_forward)){
/*
      for (int m = 0; m < 2; ++m){
        odrive.serial_ << "w axis" << m << ".controller.config.control_mode " << CONTROL_MODE_VELOCITY_CONTROL << '\n';
        delay(5);
      }
*/
      robot_state = robot_state_moving_forward;
      
      int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;
      odrive.SetVelocity(0, M0_DIR * max_current);
      if(!odrive.run_state(1, requested_state, false /*don't wait*/)) return;
      odrive.SetVelocity(1, M1_DIR * max_current);

      Serial.println("Executing forward");
      delay(5);
      return;
    }

    if ((c == 'q') && (robot_state != robot_state_moving_forward_left)){
      robot_state = robot_state_moving_forward_left;
      delay(10);
      for (int m = 0; m < 2; ++m)
        odrive.serial_ << "w axis" << m << ".controller.config.control_mode " << CONTROL_MODE_VELOCITY_CONTROL << '\n';

      int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;
      odrive.SetVelocity(0, M0_DIR * max_current);
      if(!odrive.run_state(1, requested_state, false /*don't wait*/)) return;
      odrive.SetVelocity(1, M1_DIR * max_current * VELOCITY_ROTATION_FACTOR);

      Serial.println("Moving forward left");
      delay(5);
      return;
    }

    if ((c == 'w') && (robot_state != robot_state_moving_forward_right)){
      for (int m = 0; m < 2; ++m)
        odrive.serial_ << "w axis" << m << ".controller.config.control_mode " << CONTROL_MODE_VELOCITY_CONTROL << '\n';

      robot_state = robot_state_moving_forward_right;
      
      int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;
      odrive.SetVelocity(0, M0_DIR * max_current * VELOCITY_ROTATION_FACTOR);
      if(!odrive.run_state(1, requested_state, false /*don't wait*/)) return;
      odrive.SetVelocity(1, M1_DIR * max_current);

      Serial.println("Moving forward right");
      delay(5);
      return;
    }

    if ((c == 'b') && (robot_state != robot_state_moving_backward)) {

      for (int m = 0; m < 2; ++m)
        odrive.serial_ << "w axis" << m << ".controller.config.control_mode " << CONTROL_MODE_VELOCITY_CONTROL << '\n';

      robot_state = robot_state_moving_backward;

      int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;
      odrive.SetVelocity(0, -M0_DIR*max_current);
      if(!odrive.run_state(1, requested_state, false /*don't wait*/)) return;
      odrive.SetVelocity(1, -M1_DIR*max_current);

      Serial.println("Executing backward");
      delay(5);
      return;
    }

    if ((c == 'a') && (robot_state != robot_state_moving_back_left)){

      robot_state = robot_state_moving_back_left;
      for (int m = 0; m < 2; ++m)
        odrive.serial_ << "w axis" << m << ".controller.config.control_mode " << CONTROL_MODE_VELOCITY_CONTROL << '\n';

      int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;
      odrive.SetVelocity(0, -M0_DIR*max_current);
      if(!odrive.run_state(1, requested_state, false /*don't wait*/)) return;
      odrive.SetVelocity(1, -M1_DIR*max_current * VELOCITY_ROTATION_FACTOR);

      Serial.println("Moving back left");
      delay(5);
      return;
    }

    if ((c == 's') && (robot_state != robot_state_moving_back_right)){

      robot_state = robot_state_moving_back_right;
      for (int m = 0; m < 2; ++m)
        odrive.serial_ << "w axis" << m << ".controller.config.control_mode " << CONTROL_MODE_VELOCITY_CONTROL << '\n';

      int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;
      odrive.SetVelocity(0, -M0_DIR * max_current * VELOCITY_ROTATION_FACTOR);
      if(!odrive.run_state(1, requested_state, false /*don't wait*/)) return;
      odrive.SetVelocity(1, -M1_DIR * max_current);// + CURVE_DIFF));

      Serial.println("Moving back right");
      delay(5);
      return;
    }

    if ((c == 'l') && (robot_state != robot_state_spin_left)){
      robot_state = robot_state_spin_left;
      for (int m = 0; m < 2; ++m)
        odrive.serial_ << "w axis" << m << ".controller.config.control_mode " << CONTROL_MODE_VELOCITY_CONTROL << '\n';
      
      int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;
      odrive.SetVelocity(0, M0_DIR*max_current);
      if(!odrive.run_state(1, requested_state, false /*don't wait*/)) return;
      odrive.SetVelocity(1, -M1_DIR*max_current);

      Serial.println("Rotating left");
      delay(5);
      return;
    }

    if ((c == 'r') && (robot_state != robot_state_spin_right)){
      robot_state = robot_state_spin_right;
      for (int m = 0; m < 2; ++m)
        odrive.serial_ << "w axis" << m << ".controller.config.control_mode " << CONTROL_MODE_VELOCITY_CONTROL << '\n';
      
      int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;
      odrive.SetVelocity(0, -M0_DIR*max_current);
      if(!odrive.run_state(1, requested_state, false /*don't wait*/)) return;
      odrive.SetVelocity(1, M1_DIR*max_current);

      Serial.println("Rotating right");
      delay(5);
      return;
    }

    if ((c == 'z') && (robot_state != robot_state_stopped) && (robot_state != robot_state_idle) && (robot_state != robot_state_hold_position)) {
      robot_state = robot_state_stopped;

      int requested_state = AXIS_STATE_IDLE;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;
      odrive.SetVelocity(0, 0.0);
      if(!odrive.run_state(1, requested_state, false /*don't wait*/)) return;
      odrive.SetVelocity(1, 0.0);

      Serial.println("Stop");
      delay(5);
      return;
    }

    if ((c == 'h') && (robot_state != robot_state_hold_position)) {
      // stop first
      int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false )) return;
      odrive.SetVelocity(0, 0.0);
      if(!odrive.run_state(1, requested_state, false )) return;
      odrive.SetVelocity(1, 0.0);
      delay(5);
/*
// move to position control
      robot_state = robot_state_hold_position;
      for (int m = 0; m < 2; ++m){
        odrive.serial_ << "w axis" << m << ".controller.config.control_mode " << CONTROL_MODE_POSITION_CONTROL << '\n';
        delay(5);
      }

      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false )) return;
      float pos = odrive.GetPosition(0);
      odrive.SetPosition(0, pos);
      if(!odrive.run_state(1, requested_state, false )) return;
      pos = odrive.GetPosition(1);
      odrive.SetPosition(1, pos);

      Serial.println("Hold position");
      delay(5);
  */
      return;
    }

    if ((c == 'i') && (robot_state != robot_state_idle)) {
      int requested_state = AXIS_STATE_IDLE;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(0, requested_state, false /*don't wait*/)) return;
      if(!odrive.run_state(1, requested_state, false /*don't wait*/)) return;
      odrive.SetVelocity(0, 0.0);
      odrive.SetVelocity(1, 0.0);
      robot_state = robot_state_idle;
      return;
    }

    // Read bus voltage
    if (c == 'g') {
      odrive.serial_ << "r vbus_voltage\n";
      Serial << "Vbus voltage: " << odrive.readFloat() << '\n';
      Serial << "Speed= " << odrive.GetVelocity(0) << '\n';
      Serial << "Position=" << odrive.GetPosition(0) << '\n';
      Serial << "Max current=" << max_current << '\n';
      Serial << "Robot state=" << robot_state_stopped << '\n';
      odrive.serial_ << "r axis" << 0 << ".controller.config.control_mode\n";
      Serial << "Control mode=" << odrive.readInt() << '\n';
      return;
    }

    if (c == 'c') {
      odrive.clear_errors();
      max_current = init_torque_or_speed;
      for (int m = 0; m < 2; ++m){
        odrive.serial_ << "w axis" << m << ".controller.config.control_mode " << CONTROL_MODE_VELOCITY_CONTROL << '\n';
        delay(5);
      }
      return;
    }

    if (c == 'u') {
        if (max_current < 3)
          max_current+= 0.1;
        Serial << "Current=" << max_current << '\n';
        return;
    }

    if (c == 'd') {
        if (max_current >= 0.1)
          max_current -= 0.1;
        Serial << "Current=" << max_current << '\n';
        return;
    }
}
//-------------------------------------------------------------------
