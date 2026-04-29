#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "movements.h"

struct JointAngles {
    float shoulder;
    float hip;
    float knee;
};

JointAngles legIK(LegId leg, float x, float y, float z);

#endif
