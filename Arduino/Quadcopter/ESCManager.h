#ifndef ESC_MANAGER_H
#define ESC_MANAGER_H
#include "Config.h"
#include "ESC.h"

class ESCManager {
  public:
    void setInput(float pitch, float roll, float yaw, float thrust); //this allows us to work with any number of motors. For now, we just put them on a circle..
    void setEngines(Engine[] engines);

  private:
    Engine[] engines; //create this dynamicaly, really cool
};

ESC frontRight(MOTOR_FR_OUT_PIN, ESC_MIN, ESC_MAX);
ESC frontLeft(MOTOR_FL_OUT_PIN, ESC_MIN, ESC_MAX);
ESC rearRight(MOTOR_RR_OUT_PIN, ESC_MIN, ESC_MAX);
ESC rearLeft(MOTOR_RL_OUT_PIN, ESC_MIN, ESC_MAX);

#endif
