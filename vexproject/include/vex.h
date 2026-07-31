#ifndef VEX_H_
#define VEX_H_

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "iq2_cpp.h"

// defined in main.cpp. the only brain in the program: main.cpp draws the map
// through it and robot.cpp writes its log lines to it.
extern vex::brain Brain;

#endif


