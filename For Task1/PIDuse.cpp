#include "PIDuse.h"
#include "m_dis.h"
PIDuse::PIDuse(int task,float &m_inte)
{
    m_inte=0;
    digitalWrite(this->STBY,HIGH);
    for(int k = 0; k < NMOTORS; k++){
    pinMode(this->enca[k],INPUT);
    pinMode(this->encb[k],INPUT);
    pinMode(this->pwm[k],OUTPUT);
    pinMode(this->in1[k],OUTPUT);
    pinMode(this->in2[k],OUTPUT);
    pid[k].setParams(5,0,5,255,m_inte);
    lastT=millis();
  }
  attachInterrupt(digitalPinToInterrupt(this->enca[0]),readEncoder0,RISING);
  attachInterrupt(digitalPinToInterrupt(this->enca[1]),readEncoder1,RISING);
   ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    for(int k = 0; k < NMOTORS; k++){
      posi[k]=0;
    }
  };
};
PIDuse::~PIDuse()
{
  detachInterrupt(digitalPinToInterrupt(this->enca[0]));
  detachInterrupt(digitalPinToInterrupt(this->enca[1]));
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    for(int k = 0; k < NMOTORS; k++){
      posi[k]=0;
    }
  };
}
void PIDuse::setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
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
float PIDuse::cm_to_signals(float m_cm)
{
    return m_cm/6.5*960;
}
float PIDuse::radians_to_degree(float m_radians)
{
  return m_radians/(2*this->pi)*360;
};
float PIDuse::degree_to_radians(float m_degree)
{
  return m_degree*(2*this->pi)/360;
};
float PIDuse::calangle(float Left_rpm,float Right_rpm)
{
  float v_r=rpm_m_to_velocity_cm_s(Right_rpm);
  float v_l=rpm_m_to_velocity_cm_s(Left_rpm);
  float w_s=(v_r-v_l)/this->m_carlength;
  float r_a=w_s/(2*this->pi)*360*this->system_T_s;
  return r_a;
};
float PIDuse::rpm_m_to_velocity_cm_s(float base_rpm)
{
  return base_rpm/60*this->wheel_diameter_cm;
};
float PIDuse::velocity_m_s_to_rpm_m(float velocity)
{
  return velocity*60/this->wheel_diameter;
};

int PIDuse::task(volatile float *current_angle,volatile float *current_x,volatile float *current_y,volatile float target_x,volatile float target_y,float pathl,float pathr,float* m_inte,float center_path_curve)
{
  m_dis dis;
  //long currT = micros();
  //float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  //this->prevT = currT;
  original_dis=dis.return_dis();
  float target[NMOTORS];
  float currentangle=0;
  int dir;
  //target[0]=-864;
  //target[1]=864;
  target[0]=cm_to_signals(pathl);
  target[1]=cm_to_signals(pathr);
  //Serial.print(target[0]);
  //Serial.print(target[1]);
  // Read the position in an atomic block to avoid a potential misread
  int pos[NMOTORS];
  do{
    system_T_ms=(millis()-lastT);
    system_T_s=system_T_ms/1000;
    if((millis()-lastT)==100||(millis()-lastT)>100)
    {
      current_dis=dis.return_dis();
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];
      }
      lastT=millis();
        for(int i = 0; i < NMOTORS; i++)
        { 
          this->current_rpm[i]=(float)(m_posi[i])/system_T_s/960*60;
          this->current_vel[i]=(m_posi[i])/system_T_s/960*wheel_diameter_mm;
          m_posi[i]=0;
          lastT=millis();
        }
      }
  float RVA_x_1=target_x-(*current_x);
  float RVA_y_1=target_y-(*current_y);
  float replacement=0;
  float this_period_angle=0;
  //计算当前两个轮子转速下不同速差导致的偏航角->当前偏航角
  if(this->current_rpm[0]<0&&this->current_rpm[1]<0)
  {
    this_period_angle=(calangle(fabs(this->current_rpm[0]),fabs(this->current_rpm[1])));
  }else{
  this_period_angle=((-1)*calangle(fabs(this->current_rpm[0]),fabs(this->current_rpm[1])));
  }
  //current_angle=4*(*current_angle);
  //计算行驶距离（行驶速度为两侧均速）
  float current_speed=(rpm_m_to_velocity_cm_s(this->current_rpm[0])+rpm_m_to_velocity_cm_s(this->current_rpm[1]))/2;
  float this_period_distance=current_speed*system_T_s;
  float w_S_limit=5;
  //通过pdf上的积分公式和偏航角计算当前x,y坐标()此处只处理x正负 后续需要添加
  float delta_x;
  if(this_period_angle>0)
  {
    delta_x=(this_period_distance*sin(degree_to_radians(this_period_angle)));
    *current_x=*current_x+delta_x;
  }else if(this_period_angle<0)
  {
    delta_x=(this_period_distance*sin(degree_to_radians(this_period_angle)));
    *current_x=*current_x+delta_x;
  }
  if(target_y>*current_y)
  {
    float delta_y=this_period_distance*cos(degree_to_radians(this_period_angle));
    *current_y=*current_y+delta_y;
  }else if(target_y<*current_y)
  {
    float delta_y=this_period_distance*cos(degree_to_radians(this_period_angle));
    *current_y=*current_y-delta_y;
  }
  //计算终点对于现在的相对位置
  *current_angle=*current_angle+this_period_angle;
  float RVA_x=target_x-(*current_x);
  float RVA_y=target_y-(*current_y);
  //小车中心（0°）相对终点的角度
  if(RVA_x!=0&&RVA_y!=0)
  {
  rva_car_center_angle_car_target=atan(RVA_x/RVA_y);
  rva_car_center_angle_car_target=radians_to_degree(rva_car_center_angle_car_target);
  }else{
    rva_car_center_angle_car_target=0;
  }
  //小车当前相对笛卡尔坐标系的角度（0°）相对终点的角度 即为小车理论要调整的偏航角W(degree)
  rva_car_dir_angle_car_target=(-1)*(*current_angle)+rva_car_center_angle_car_target;
  //设定不管多少度角都用时间1s来调整 代码间隔周期为System_s 既W_S(degree/s)为degree/(1m/System_s)
  float required_W_S=rva_car_dir_angle_car_target/(2/system_T_s);

  //通过需求的角速度去求理论上需要的速度差(m/s)
  float deltaspeed=required_W_S*m_carlength;
  m_deltarpm=fabs(velocity_m_s_to_rpm_m(deltaspeed));
    
    for(int k = 0; k < NMOTORS; k++){
      if(pwr[k]<30)
      {
        power_not_enough_flag=1;
      }
      this->pid[k].evaludis(*current_y,target_y,system_T_s,pwr[k],dir);
    }
    this->pid[0].evalu(*current_angle,0,system_T_s,pwr,dir,*m_inte);
      for(int i=0;i<2;i++)
      {
        this->setMotor(dir,pwr[i],pwm[i],in1[i],in2[i]);
      }
  }
  }while(fabs(*current_y)<fabs(target_y)||dis.return_dis()<2);
  dir=0;
  for(int i=0;i<2;i++)
  {
    this->setMotor(dir,0,pwm[i],in1[i],in2[i]);
  }
 return 1;
}
    


static void readEncoder0(){
  int b = digitalRead(18);
  if(b > 0){
    posi[0]++;
    m_posi[0]++;
  }
  else{
    posi[0]--;
    m_posi[0]--;
  }
};
static void readEncoder1(){
  int b = digitalRead(21);
  if(b > 0){
    posi[1]++;
    m_posi[1]++;
  }
  else{
    posi[1]--;
    m_posi[1]--;
  }
};
