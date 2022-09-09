#include "rotationpid.h"
void Rotationpid::setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
    this->kp = kpIn; this->kd = kdIn; this->ki = kiIn; this->umax = umaxIn;
};

  // A function to compute the control signal
void Rotationpid::evalu(float value, float target, float deltaT, int pwr[2], int &dir){
    // error
    float e = target - value;
    // derivative
    float dedt = (e-eprev)/(deltaT);
  
    // integral
    eintegral = eintegral + e*deltaT;
  
    // control signal
    float u = kp*e;
    // motor power
    /*Serial.print(value);
    Serial.print(" value ");
    Serial.print(target);
    Serial.print(" target ");*/
    pwr[0] = pwr[0]+(int)(u);
    pwr[1] = pwr[1]-(int)(u);
    if( pwr[0] > 100 ){
      pwr[0] = 100;
    }else if(pwr[0]<0)
    {
      pwr[0]= 0;
    }
    if( pwr[1] > 100 ){
      pwr[1] = 100;
    }else if(pwr[1]<0)
    {
      pwr[1]= 0;
    }
    // motor direction
    /*dir = 1;
    if(pwr<0){
      dir = -1;
    }*/
  
    // store previous error
    eprev = e;
};
void Rotationpid::evaludis(float value, float target, float deltaT,int &r_rpm, int &dir){
    // error
    float e = target - value;
    // derivative
    float dedt = (e-eprev)/(deltaT);
  
    // integral
    eintegral = eintegral + e*deltaT;
  
    // control signal
    float u = 50*e;
    // motor power
    //Serial.print(e);
    //Serial.print(" ");
    r_rpm = fabs((int)(u));
    if( r_rpm > 70 ){
      r_rpm = 70;
    }else if(r_rpm<40)
    {
      r_rpm = 40;
    }
    // motor direction
    dir = 1;
    if(u<0){
      dir = -1;
    }else if(value>target)
    {
      dir = 0;
    }
    // store previous error
    eprev = e;
};
  
