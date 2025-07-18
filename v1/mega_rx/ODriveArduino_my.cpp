#include "Arduino.h"
#include "ODriveArduino_my.h"
#include "print_commands.h"
// ----------------------------------------------------------------------------------
// Print with stream operator
//template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
//template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }
// ----------------------------------------------------------------------------------
t_ODriveArduino::t_ODriveArduino(Stream& serial): serial_(serial) 
{
//		
}
// ----------------------------------------------------------------------------------
void t_ODriveArduino::SetPosition(int motor_number, float position) 
{
    SetPosition(motor_number, position, 0.0f, 0.0f);
}
// ----------------------------------------------------------------------------------
void t_ODriveArduino::SetPosition(int motor_number, float position, float velocity_feedforward) 
{
    SetPosition(motor_number, position, velocity_feedforward, 0.0f);
}
// ----------------------------------------------------------------------------------
void t_ODriveArduino::SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward) 
{
    serial_ << "p " << motor_number  << " " << position << " " << velocity_feedforward << " " << current_feedforward << "\n";
}
// ----------------------------------------------------------------------------------
void t_ODriveArduino::SetVelocity(int motor_number, float velocity) 
{
    SetVelocity(motor_number, velocity, 0.0f);
}
// ----------------------------------------------------------------------------------
void t_ODriveArduino::SetVelocity(int motor_number, float velocity, float current_feedforward) 
{
    serial_ << "v " << motor_number  << " " << velocity << " " << current_feedforward << "\n";
}
// ----------------------------------------------------------------------------------
void t_ODriveArduino::SetCurrent(int motor_number, float current) 
{
    serial_ << "c " << motor_number << " " << current << "\n";
}
// ----------------------------------------------------------------------------------
void t_ODriveArduino::TrapezoidalMove(int motor_number, float position) 
{
    serial_ << "t " << motor_number << " " << position << "\n";
}
// ----------------------------------------------------------------------------------
float t_ODriveArduino::readFloat() 
{
    return readString().toFloat();
}
// ----------------------------------------------------------------------------------
float t_ODriveArduino::GetVelocity(int motor_number) 
{
	serial_<< "r axis" << motor_number << ".encoder.vel_estimate\n";
	return t_ODriveArduino::readFloat();
}
// ----------------------------------------------------------------------------------
float t_ODriveArduino::GetPosition(int motor_number)
{
    serial_ << "r axis" << motor_number << ".encoder.pos_estimate\n";
    return t_ODriveArduino::readFloat();
}
// ----------------------------------------------------------------------------------
int32_t t_ODriveArduino::readInt() 
{
    return readString().toInt();
}
// ----------------------------------------------------------------------------------
bool t_ODriveArduino::run_state(int axis, int requested_state, bool wait_for_idle, float timeout) 
{
    int timeout_ctr = (int)(timeout * 10.0f);
    serial_ << "w axis" << axis << ".requested_state " << requested_state << '\n';
    if (wait_for_idle) {
        do {
            delay(100);
            serial_ << "r axis" << axis << ".current_state\n";
        } while (readInt() != AXIS_STATE_IDLE && --timeout_ctr > 0);
    }

    return timeout_ctr > 0;
}
// ----------------------------------------------------------------------------------
String t_ODriveArduino::readString() 
{
    String str = "";
    static const unsigned long timeout = 1000;
    unsigned long timeout_start = millis();
    for (;;) {
        while (!serial_.available()) {
            if (millis() - timeout_start >= timeout) {
                return str;
            }
        }
        char c = serial_.read();
        if (c == '\n')
            break;
        str += c;
    }
    return str;
}
// ----------------------------------------------------------------------------------
void t_ODriveArduino::clear_errors(void)
{
	serial_ << "sc\n";
}
// ----------------------------------------------------------------------------------