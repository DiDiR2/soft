#ifndef print_commands_H
#define print_commands_H
//-------------------------------------------------------------------
//#include "printf.h"
//-------------------------------------------------------------------
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>	      inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }
//-------------------------------------------------------------------
#endif print_commands_H