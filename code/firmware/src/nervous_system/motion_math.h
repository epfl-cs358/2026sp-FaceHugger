#ifndef FH_MOTION_MATH_H
#define FH_MOTION_MATH_H
#include <stdint.h>

/* Pure, Arduino-free motion math shared by the gait engine and the clip
 * player. Compiled in the `native` env so it is unit-tested on the host. */

typedef struct { double hip; double thigh; double knee; } ServoTriple;

/* Math-space (sh,th,kn) -> servo-space (0..180) for one leg. The exact
 * per-leg mounting transform lifted verbatim from tickGait's switch.
 * NOT a clamp and NOT IK. legId is the firmware LegId (0=FR,1=FL,2=RR,3=RL). */
ServoTriple translateToServo(uint8_t legId, double sh, double th, double kn);

#endif /* FH_MOTION_MATH_H */
