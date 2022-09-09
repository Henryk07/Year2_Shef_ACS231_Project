#include "m_servo.h"
#include "Arduino.h"
m_servo::m_servo()
{
  this->m_S1.attach(S1);
  this->m_S2.attach(S2);
  this->m_S3.attach(S3);
  this->m_S4.attach(S4);
  this->m_conservo.attach(m_coutinue_servo);
  //this->m_conservo.write(90);
  this->m_S1.write(angle1);
  this->m_S2.write(angle2);
  this->m_S3.write(angle3);
  this->m_S3.write(angle4);
}
m_servo::~m_servo()
{
  target1=(13);
  target2=(90);
  target3=(90);
  target4=(90);
  start();
  /*this->m_S1.detach();
  this->m_S2.detach();
  this->m_S3.detach();
  this->m_S4.detach();*/
}
float m_servo::start()
{
  do{
    if(millis()-lasttime>50)
    {
      lasttime=millis();
    if(angle1<target1)
    {
      angle1=angle1+1;
    }else if(angle1>target1)
    {
      angle1=angle1-1;
    }
    if(angle2<target2)
    {
      angle2=angle2+1;
    }else if(angle2>target2)
    {
      angle2=angle2-1;
    }
    if(angle3<target3)
    {
      angle3=angle3+1;
    }else if(angle3>target3)
    {
      angle3=angle3-1;
    }
    if(angle4<target4)
    {
      angle4=angle4+1;
    }else if(angle4>target4)
    {
       angle4=angle4-1;
    }
      m_S1.write(angle1);
      //Serial.print(angle1);
      //Serial.print(" angle1 ");
      m_S2.write(angle2);
      //Serial.print(target1);
      //Serial.print(" target2 ");
      m_S3.write(angle3);
      //Serial.print(angle3);
      //Serial.print(" angle3 ");
      m_S4.write(angle4);
      //Serial.print(angle4);
      //Serial.print(" angle4 ");
      //Serial.println();
    }
  }while((angle1!=target1)||(angle2!=target2)||(angle3!=target3)||(angle4!=target4));
  return 1;
}
float m_servo::catch_pen()
{
  target1=13;
  return 1;
}
float m_servo::bringback_pen()
{
   target1=(13);
   target2=(70);
   target3=(150);
   target4=(90);
   start();
}
float m_servo::catch_pen_process()
{
  start();
  catch_pen();
  start();
  bringback_pen();
}
float m_servo::place_pen(){
  target1=13;
  target2=180;
  target3=105;
  target4=90;
  start();
  target4=92;
  start();
  target4=(87);
  start();
  target1=(45);
  start();
}
float m_servo::draw_circuit()
{
  target1=70;
  target2=145;
  target3=70;
  target4=85;
  start();
  delay(1000);
  m_conservo.attach(m_coutinue_servo);
  m_conservo.write(100);
  delay(5000);
  m_conservo.write(80);
  delay(5000);
  m_conservo.write(90);
  return 1;
}

float m_servo::safe_path_grapper()
{
  target1=100;
  start();
}

void m_servo::setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  if(pwmVal>0)
  {
   analogWrite(STBY,HIGH);
  }else
  {
    analogWrite(STBY,LOW);
  }
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
};
