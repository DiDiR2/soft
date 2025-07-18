#ifndef setup_commands_H 
#define setup_commands_H
//-------------------------------------------------------------------
#include <Adafruit_LSM6DSOX.h>
#include "ODriveArduino_my.h"
#include "RF24.h"
//-------------------------------------------------------------------
void setup_sensors(Adafruit_LSM6DSOX &sox);
void setup_wifi(RF24 &radio, uint8_t address[][6]);
void setup_odrive(t_ODriveArduino &);
//-------------------------------------------------------------------
#endif setup_commands_H
