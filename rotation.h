#include "Arduino.h"
#include <util/atomic.h>
#include "rotationpid.h"
#ifndef NMOTORS
#define NMOTORS 2
#endif
//const int NMOTORS=2;
class Rotation{
  public:
  // Pins
  const int enca[NMOTORS] = {19,20};
  const int encb[NMOTORS]= {18,21};
  int pwm[NMOTORS] = {6,4};
  const int in1[NMOTORS] = {49,51};
  const int in2[NMOTORS] = {48,52};
  const int STBY=50;

  // Globals
  long prevT = 0;
  long lastT=0;
  float current_rpm[NMOTORS]={0};
  float delta_rpm[NMOTORS]={0};
  float pre_rpm[NMOTORS]={0};
  float required_rpm[NMOTORS]={0};
  boolean enable=0;
  int pwr[NMOTORS]={0};
  int m_stop=0;
  // PID class instances
  Rotationpid pid[NMOTORS];
  Rotation();
  ~Rotation();
  void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
  void task(float target_left,float target_right);
};
  static void readEncoder2();
  static void readEncoder3();
  static int posi1[NMOTORS] = {0};
  static int m_posi1[NMOTORS] = {0};
