#ifndef LEG_H
#define LEG_H

class Leg {
  public:
    int id;
    Leg(int legID); // Constructor
    void setPose(float x, float y, float z); // Inverse Kinematics target
    void updateServos(); // Sends the angles to the PCA9685
};

#endif