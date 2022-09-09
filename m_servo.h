#include <Servo.h>

#define M_NMOTOR 2
class m_servo{
  public:
  const int S1=27;
  const int S2=31;
  const int S3=37;
  const int S4=33;
  const int m_coutinue_servo=22;
  int angle1=14;
  int angle2=90;
  int angle3=90;
  int angle4=90;
  double lasttime=0;
  const int catch_S1=14;
  const int catch_S2=90;
  const int catch_S3=90;
  const int catch_S4=90;
  float target1=45;
  float target2=90;
  float target3=10;
  float target4=90;
  const int circle_point_number=50;
  float circeangle2[50]={0};
  float circeangle3[50]={0};
  float circeangle4[50]={0};
  Servo m_S1;
  Servo m_S2;
  Servo m_S3;
  Servo m_S4;
  Servo m_conservo;
  m_servo();
  ~m_servo();
  float start();
  float catch_pen();
  float bringback_pen();
  float catch_pen_process();
  float place_pen();
  float draw_circuit();
  float safe_path_grapper();
  //this part give low voltage to dc motor in order to keep position
  //const int M_NMOTOR=2;
  const int STBY=50;
  const int enca[M_NMOTOR] = {19,20};
  const int encb[M_NMOTOR]= {18,21};
  int pwm[M_NMOTOR] = {6,4};
  const int in1[M_NMOTOR] = {49,51};
  const int in2[M_NMOTOR] = {48,52};
  float keepposition();
  void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
};
