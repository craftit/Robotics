

class ServoC
{
public:

  float m_position = 0;
  float m_targetPosition = 0;

  float m_velocity = 0;
  float m_velocityLimit = 100;
  float m_accelerationLimit = 10;


  float m_velocityRamp = 10;

  void Tick(float timeDelta) {
    float diff = m_targetPosition - m_position;
    float targetVel = diff * m_velocityRamp;

    // Limit velocity
    if(targetVel > m_velocityLimit)
      targetVel = m_velocity;
    if(targetVel < -m_velocityLimit)
      targetVel = -m_velocity;

    float accel = targetVel - m_velocity;

    // Limit acceleration
    if(accel > m_accelerationLimit)
      accel = m_accelerationLimit;
    if(accel < -m_accelerationLimit)
      accel = -m_accelerationLimit;

    m_velocity += accel * timeDelta;
    m_position += m_velocity * timeDelta;
  }
};
