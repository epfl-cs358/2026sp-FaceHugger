#include "kinematics.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

constexpr float L1 = 0.080f;
constexpr float L2 = 0.075f;
constexpr float L3 = 0.077f;

struct LegGeometry {
    float mount_x;
    float mount_y;
    float mount_z;
    float yaw_offset;
};

constexpr LegGeometry GEOMETRY[LEG_COUNT] = {
    { +0.040f,  0.050f, 0.025f, (float)M_PI },
    { -0.040f,  0.050f, 0.025f, 0.0f         },
    { +0.040f, -0.050f, 0.025f, (float)M_PI },
    { -0.040f, -0.050f, 0.025f, 0.0f         },
};

constexpr float SHOULDER_LIMIT_LO = -2.356f;
constexpr float SHOULDER_LIMIT_HI =  2.356f;
constexpr float HIP_LIMIT_LO      = -1.5708f;
constexpr float HIP_LIMIT_HI      =  1.5708f;
constexpr float KNEE_LIMIT_LO     = -2.356f;
constexpr float KNEE_LIMIT_HI     =  0.5f;

inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

inline float wrapPi(float a) {
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

}

JointAngles legIK(LegId leg, float x, float y, float z) {
    const LegGeometry& g = GEOMETRY[leg];

    const float dx = x - g.mount_x;
    const float dy = y - g.mount_y;
    const float dz = z - g.mount_z;

    const float r_xy = sqrtf(dx * dx + dy * dy);
    const float theta_total = atan2f(-dy, -dx);
    float theta_s = wrapPi(theta_total - g.yaw_offset);

    float a = r_xy - L1;
    float b = -dz;

    float c = sqrtf(a * a + b * b);
    const float c_max = L2 + L3 - 1e-4f;
    if (c > c_max) {
        const float scale = c_max / fmaxf(sqrtf(a * a + b * b), 1e-9f);
        a *= scale;
        b *= scale;
        c = c_max;
    }

    const float cos_alpha = (L2 * L2 + L3 * L3 - c * c) / (2.0f * L2 * L3);
    const float alpha = acosf(clampf(cos_alpha, -1.0f, 1.0f));
    float theta_k = alpha - (float)M_PI;

    const float cos_beta = (L2 * L2 + c * c - L3 * L3) / (2.0f * L2 * c + 1e-12f);
    const float beta = acosf(clampf(cos_beta, -1.0f, 1.0f));
    float theta_h = atan2f(a, b) + beta;

    JointAngles out;
    out.shoulder = clampf(theta_s, SHOULDER_LIMIT_LO, SHOULDER_LIMIT_HI);
    out.hip      = clampf(theta_h, HIP_LIMIT_LO,      HIP_LIMIT_HI);
    out.knee     = clampf(theta_k, KNEE_LIMIT_LO,     KNEE_LIMIT_HI);
    return out;
}
