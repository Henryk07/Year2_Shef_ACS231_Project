#include "PIDuse.h"
#include "Rotation.h"
//#include <Servo.h>
#include "m_servo.h"
#include "m_dis.h"
m_servo *m_servotask;
PIDuse *m_task1;
Rotation *m_task2;
PIDuse *m_task3;
m_dis *nm_dis;
//10,48,46禁用
volatile float current_angle=0;
volatile float current_angle1=0;
volatile float current_angle2=0;
volatile float current_x=0;
volatile float current_y=0;
volatile float current_x1=0;
volatile float current_y1=0;
float target_x=0;
float target_y=10;
float eintegral=0;
float eintegral1=0;
void setup() {
  nm_dis=new m_dis();
  // put your setup code here, to run once:
  //m_task=new PIDuse();
  Serial.begin(9600);
  /*//四个伺服电机
  m_servotask=new m_servo();
  //直行
  m_task1=new PIDuse(1,eintegral);
  m_task1->task(&current_angle,&current_x,&current_y,0,24,24,24,&eintegral);
  delete m_task1;
  delay(1000);
  //抓笔
  m_servotask->catch_pen_process();
  delay(1000);
  eintegral=0;
  current_x=0;
  current_y=0;
  //直行
  m_task1=new PIDuse(1,eintegral);
  m_task1->task(&current_angle,&current_x,&current_y,0,15,15,15,&eintegral);
  delete m_task1;
  delay(1000);
  
  //m_servotask->bringback_pen();
  //第一次转弯
  m_task2=new Rotation();
  m_task2->task(485,-485);
  delete m_task2;
  delay(1000);
  //直行
  eintegral=0;
  current_x=0;
  current_y=0;
  m_task1=new PIDuse(1,eintegral);
  m_task1->task(&current_angle,&current_x,&current_y,0,48,48,48,&eintegral);
  delete m_task1;
  delay(1000);
  //放笔
  m_servotask->place_pen();
  m_servotask->bringback_pen();
  //转180度
  m_task2=new Rotation();
  m_task2->task(485,-485);
  delete m_task2;
  m_task2=new Rotation();
  m_task2->task(485,-485);
  delete m_task2;
  delay(1000);
  //直行
  eintegral=0;
  current_x=0;
  current_y=0;
  m_task1=new PIDuse(1,eintegral);
  m_task1->task(&current_angle,&current_x,&current_y,0,48,48,48,&eintegral);
  delete m_task1;
  delay(1000);
  //左转
  m_task2=new Rotation();
  m_task2->task(-485,485);
  delete m_task2;
  delay(1000);
  //直行
  eintegral=0;
  current_x=0;
  current_y=0;
  m_task1=new PIDuse(1,eintegral);
  m_task1->task(&current_angle,&current_x,&current_y,0,38,38,38,&eintegral);
  delete m_task1;
  delay(1000);
  //掉头
  m_task2=new Rotation();
  m_task2->task(485,-485);
  delete m_task2;
  m_task2=new Rotation();
  m_task2->task(485,-485);
  delete m_task2;
  delay(1000);
  delete m_servotask;*/
  
  //task2
  m_servotask=new m_servo();
  //直行
  eintegral=0;
  current_x=0;
  current_y=0;
  m_task1=new PIDuse(1,eintegral);
  do
  {
    eintegral=0;
    current_x=0;
    current_y=0;
    m_task1->task(&current_angle,&current_x,&current_y,0,(nm_dis->return_dis()-5),(nm_dis->return_dis()-5),(nm_dis->return_dis()-5),&eintegral);
    delay(1000);
  }while(nm_dis->return_dis()>5);
  delete m_task1;
  delay(1000);
  //houzhi(未销毁)
  m_servotask->bringback_pen();
  //左转90度
  m_task2=new Rotation();
  m_task2->task(-485,485);
  delete m_task2;
  delay(1000);
  //直行
  eintegral=0;
  current_x=0;
  current_y=0;
  PIDuse m_task12(1,eintegral);
  m_dis new_dis;
  m_task1=new PIDuse(1,eintegral);
  do
  {
    eintegral=0;
    current_x=0;
    current_y=0;
    m_task12.task(&current_angle,&current_x,&current_y,0,(new_dis.return_dis()-17.5),(new_dis.return_dis()-17.5),(new_dis.return_dis()-17.5),&eintegral);
    delay(1000);
  }while(new_dis.return_dis()>17.5);
  delete m_task1;
  delay(1000);
  m_servotask->draw_circuit();
  m_servotask->bringback_pen();
  //左转180度
  m_task2=new Rotation();
  m_task2->task(-485,485);
  delete m_task2;
  delay(2000);
  m_task2=new Rotation();
 m_task2->task(-485,485);
  delete m_task2;
  delay(2000);
  //直行
  m_task1=new PIDuse(1,eintegral);
  do
  {
    eintegral=0;
    current_x=0;
    current_y=0;
    m_task12.task(&current_angle,&current_x,&current_y,0,(new_dis.return_dis()-10),(new_dis.return_dis()-10),(new_dis.return_dis()-10),&eintegral);
    delay(1000);
  }while(new_dis.return_dis()>12);
  delete m_task1;
  delay(1000);
  //右转90度
  m_task2=new Rotation();
  m_task2->task(485,-485);
  delete m_task2;
  //直行
  eintegral=0;
  current_x=0;
  current_y=0;
  m_task1=new PIDuse(1,eintegral);
  do
  {
    eintegral=0;
    current_x=0;
    current_y=0;
    m_task12.task(&current_angle,&current_x,&current_y,0,(new_dis.return_dis()-30),(new_dis.return_dis()-30),(new_dis.return_dis()-30),&eintegral);
    delay(1000);
  }while(new_dis.return_dis()>35);
  delete m_task1;
  delay(1000);
  //delete m_servotask;
  //右转180度
  m_task2=new Rotation();
  m_task2->task(485,-485);
  delete m_task2;
  delay(1000);
  m_task2=new Rotation();
  m_task2->task(485,-485);
  delete m_task2;














  
  /*//四个伺服电机
  m_servotask=new m_servo();
  //直行
  m_task1=new PIDuse(1,eintegral);
  m_task1->task(&current_angle,&current_x,&current_y,0,25,25,25,&eintegral);
  delete m_task1;
  delay(1000);
  //抓笔
  m_servotask->catch_pen_process();
  delay(1000);
  eintegral=0;
  current_x=0;
  current_y=0;
  //直行
  m_task1=new PIDuse(1,eintegral);
  m_task1->task(&current_angle,&current_x,&current_y,0,15,15,15,&eintegral);
  delete m_task1;
  delay(1000);
  
  //m_servotask->bringback_pen();
  //第一次转弯
  m_task2=new Rotation();
  m_task2->task(485,-485);
  delete m_task2;
  delay(1000);
  //直行
  eintegral=0;
  current_x=0;
  current_y=0;
  m_task1=new PIDuse(1,eintegral);
  m_task1->task(&current_angle,&current_x,&current_y,0,48,48,48,&eintegral);
  delete m_task1;
  delay(1000);
  //放笔
  m_servotask->place_pen();
  m_servotask->bringback_pen();
  //转180度
  m_task2=new Rotation();
  m_task2->task(485,-485);
  delete m_task2;
  m_task2=new Rotation();
  m_task2->task(485,-485);
  delete m_task2;
  delay(1000);
  //直行
  eintegral=0;
  current_x=0;
  current_y=0;
  m_task1=new PIDuse(1,eintegral);
  m_task1->task(&current_angle,&current_x,&current_y,0,48,48,48,&eintegral);
  delete m_task1;
  delay(1000);
  //左转
  m_task2=new Rotation();
  m_task2->task(-485,485);
  delete m_task2;
  delay(1000);
  //直行
  eintegral=0;
  current_x=0;
  current_y=0;
  m_task1=new PIDuse(1,eintegral);
  m_task1->task(&current_angle,&current_x,&current_y,0,38,38,38,&eintegral);
  delete m_task1;
  delay(1000);
  //掉头
  m_task2=new Rotation();
  m_task2->task(485,-485);
  delete m_task2;
  m_task2=new Rotation();
  m_task2->task(485,-485);
  delete m_task2;
  delay(1000);*/
}

void loop() {
  
}
