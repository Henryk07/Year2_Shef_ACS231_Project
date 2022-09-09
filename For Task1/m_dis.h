class m_dis{
  public:
  const int echoPin=11;
  const int trigPin=9; 
  long duration; // variable for the duration of sound wave travel
  float distance; // variable for the distance measurement
  m_dis();
  float return_dis();
};
