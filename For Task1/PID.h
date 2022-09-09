#include <util/atomic.h>
#include "Arduino.h"
// A class to compute the control signal
class SimplePID{
  private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral; // Storage
  public:
  // Constructor
  SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0){};
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn,float &einte);
  void setParams1(float kpIn, float kdIn, float kiIn, float umaxIn);
  void evalu(float value, float target, float deltaT, int pwr[2], int &dir,float &m_inte);
  void evaludis(float value, float target, float deltaT, int &r_rpm, int &dir);
  // A function to set the parameters
};
