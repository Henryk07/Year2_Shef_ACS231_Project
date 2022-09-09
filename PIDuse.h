#include "Arduino.h"
#include <util/atomic.h>
#include "PID.h"
#ifndef NMOTORS
#define NMOTORS 2
#endif
//const int NMOTORS=2;
class PIDuse{
  public:
  // Pins
  const int enca[NMOTORS] = {19,20};
  const int encb[NMOTORS]= {18,21};
  int pwm[NMOTORS] = {6,4};
  const int in1[NMOTORS] = {49,51};
  const int in2[NMOTORS] = {48,52};
  const int STBY=50;
  const float pi=3.1415;
  float system_T_s=0;
  float system_T_ms=0;
  const float m_carlength=10.5; 
  const float wheel_diameter=0.065;
  const float wheel_diameter_cm=6.5*pi;
  const float wheel_diameter_mm=0.065;
  // Globals
  long prevT = 0;
  long power_not_enough_flag=0;
  double lastT=0;
  //this from ultr sensor
  float original_dis=0;
  float current_dis=0;
  int car_dir_four_axis[2]={0};
  float rva_car_center_angle_car_target=0;
  float rva_car_dir_angle_car_target=0;
  float current_rpm[NMOTORS]={0};
  float current_vel[NMOTORS]={0};
  float required_rpm[NMOTORS]={0};
  float required_vel[NMOTORS]={0};
  boolean enable=0;
  float m_deltarpm=0;
  int pwr[NMOTORS]={0};
  int m_stop=0;
  int endflag=0;
  int cleanflage=0;
  // PID class instances
  SimplePID pid[NMOTORS];
  PIDuse(int task,float &m_inte);
  ~PIDuse();
  void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
  float calangle(float Right_rpm,float Left_rpm);
  float cm_to_signals(float m_cm);
  float rpm_m_to_velocity_cm_s(float base_rpm);
  float velocity_m_s_to_rpm_m(float velocity);
  float radians_to_degree(float m_radians);
  float degree_to_radians(float m_degree);
  int task(volatile float *current_angle,volatile float *current_x,volatile float *current_y,volatile float target_x,volatile float target_y,float pathl,float pathr,float *m_inte,float center_path_curve=0);
};
  static void readEncoder0();
  static void readEncoder1();
  static int posi[NMOTORS] = {0};
  static int m_posi[NMOTORS] = {0};
