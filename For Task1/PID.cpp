#include "PID.h"
void SimplePID::setParams(float kpIn, float kdIn, float kiIn, float umaxIn,float &einte){
    this->kp = kpIn; this->kd = kdIn; this->ki = kiIn; this->umax = umaxIn,this->eintegral=einte;
};
void SimplePID::setParams1(float kpIn, float kdIn, float kiIn, float umaxIn){
    this->kp = kpIn; this->kd = kdIn; this->ki = kiIn; this->umax = umaxIn;
};

// A function to compute the control signal
void SimplePID::evalu(float value, float target, float deltaT, int pwr[2], int &dir,float &m_inte){
    // error
    float e = target - value;
    // derivative
    float dedt = (e-eprev)/(deltaT);
  
    // integral
    eintegral = m_inte + e*deltaT;
  
    // control signal
    float u = kp*e + ki*(eintegral);
    //float u = kp*e;
    // motor power
    if(dir==1)
    {
      pwr[0] = pwr[0]+(int)(u);
      pwr[1] = pwr[1]-(int)(u);
    }
    if(dir==-1)
    {
      pwr[0] = pwr[0]-(int)(u);
      pwr[1] = pwr[1]+(int)(u);
    }

    
    if( pwr[0] > 200 ){
      pwr[0] = 200;
    }else if(pwr[0]<0)
    {
      pwr[0] = 0;
    }
     if( pwr[1] > 200 ){
      pwr[1] = 200;
    }else if(pwr[0]<0)
    {
      pwr[1] = 0;
    }
    /*Serial.print(e);
    Serial.print(" e ");
    Serial.print(eintegral);
    Serial.print(" eintegral ");
    Serial.print(pwr[0]);
    Serial.print(pwr[1]);
    Serial.print(" ");*/
    // store previous error
    eprev = e;
};

void SimplePID::evaludis(float value, float target, float deltaT,int &r_rpm, int &dir){
    // error
    float e = target - value;
  
    // control signal
    float u = 40*e;
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
};
  
