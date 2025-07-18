#ifndef handle_motor_commands_H
#define handle_motor_commands_H
//-------------------------------------------------------------------
#include "ODriveArduino_my.h"
//-------------------------------------------------------------------
#define M0_DIR 1
#define M1_DIR -1

#define init_torque_or_speed 0.5

#define VELOCITY_ROTATION_FACTOR 0.75
//-------------------------------------------------------------------
void handle_motor_command_velocity_control(t_ODriveArduino &odrive, char command);
//-------------------------------------------------------------------
#define robot_state_stopped 0
#define robot_state_moving_forward 1
#define robot_state_moving_backward 2
#define robot_state_spin_left 3
#define robot_state_spin_right 4
#define robot_state_moving_forward_left 5
#define robot_state_moving_forward_right 6
#define robot_state_moving_back_left 7
#define robot_state_moving_back_right 8
#define robot_state_idle 9
#define robot_state_hold_position 10
//-------------------------------------------------------------------
extern int robot_state;

extern float max_current;
//-------------------------------------------------------------------
#endif
