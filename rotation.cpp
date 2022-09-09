#include "rotation.h"
Rotation::Rotation()
{
    digitalWrite(this->STBY,HIGH);
    for(int k = 0; k < NMOTORS; k++){
    pinMode(this->enca[k],INPUT);
    pinMode(this->encb[k],INPUT);
    pinMode(this->pwm[k],OUTPUT);
    pinMode(this->in1[k],OUTPUT);
    pinMode(this->in2[k],OUTPUT);
    pid[k].setParams(0.9,0,0,255);
  }
  attachInterrupt(digitalPinToInterrupt(this->enca[0]),readEncoder2,RISING);
  attachInterrupt(digitalPinToInterrupt(this->enca[1]),readEncoder3,RISING);
  //interrupts();
};
Rotation::~Rotation()
{
 //detachInterrupt(digitalPinToInterrupt(this->enca[0]));
  //detachInterrupt(digitalPinToInterrupt(this->enca[1]));
  //noInterrupts();
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    for(int k = 0; k < NMOTORS; k++){
      posi1[k]=0;
    }
  };
};

void Rotation::setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
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
void Rotation::task(float target_left,float target_right)
{
  int dir[2]={1};
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  this->prevT = currT;
  float target[NMOTORS];
  //target[0]=-864;
  //target[1]=864;
  target[0]=target_left;
  target[1]=target_right;
  int pos1[NMOTORS];
  do{
    
    // Read the posi1tion in an atomic block to avoid a potential misread
    
    if((millis()-lastT)==100||(millis()-lastT)>100)
    {
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k=0;k<2;k++)
      {
        pos1[k] = posi1[k];
      }
      for(int i = 0; i < NMOTORS; i++)
      {
        this->current_rpm[i]=(float)(m_posi1[i])/0.1/960*60;
        delta_rpm[i]=current_rpm[i]-pre_rpm[i];
        pre_rpm[i]=current_rpm[i];
        m_posi1[i]=0;
        lastT=millis();
     }
     for(int k = 0; k < NMOTORS; k++){
      this->pid[k].evaludis((float)pos1[k],(float)target[k],deltaT,pwr[k],dir[k]);
     }
     this->pid[0].evalu(fabs(pos1[0]),fabs(pos1[1]),deltaT,pwr,dir[0]);
     //this->pid[0].evalu(fabs(delta_rpm[0]),fabs(delta_rpm[1]),deltaT,pwr,dir[0]);
      for(int k=0;k<2;k++)
      {
      this->setMotor(dir[k],pwr[k],pwm[k],in1[k],in2[k]);
      }
    }
   }
  }while(fabs(pos1[0])<fabs(target[0])&&fabs(pos1[1])<fabs(target[1]));
  dir[0]=0;
  dir[1]=0;
  for(int i=0;i<2;i++)
  {
     this->setMotor(dir[i],pwr[i],pwm[i],in1[i],in2[i]);
  }
};
static void readEncoder2(){
  int b = digitalRead(18);
  if(b > 0){
    posi1[0]++;
    m_posi1[0]++;
  }
  else{
    posi1[0]--;
    m_posi1[0]--;
  }
};
static void readEncoder3(){
  int b = digitalRead(21);
  if(b > 0){
    posi1[1]++;
    m_posi1[1]++;
  }
  else{
    posi1[1]--;
    m_posi1[1]--;
  }
};
