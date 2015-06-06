#ifndef IR_H
#define IR_H

#include "Arduino.h"

const int num_pts = 13;

void IR_setup(int ir);

int IR_get_dist(int ir);

#endif
