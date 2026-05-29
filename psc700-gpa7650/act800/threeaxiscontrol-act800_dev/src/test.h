#ifndef __TEST_H__
#define __TEST_H__

#include "motor_ctrl.h"

#define TAN_30DEG (0.57735026918962576450914878050196)

typedef struct{
    float x;
    float y;
    float z;
}VECTOR3F;

typedef struct{
    char model[32];
    VECTOR3F range;
}DESIGN_RANGE;

static float step_to_mm(int axis, int step);
void set_driving_current(CURRENT_SETTING cur_set);
int burn_in_loop(int count, CURRENT_SETTING current_setting);

#endif //__TEST_H__