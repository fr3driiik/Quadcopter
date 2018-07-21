#ifndef ESCCONTROLLER_H
#define ESCCONTROLLER_H

struct Vector3 {
  float x, y, z;
}

struct Engine {
  Vector3 position;
  //Vector3 rotation; //too much
  ESC esc;
}

class ESCManager {
  public:
    void setOutputVector(Vector3 vector); //this allows us to work with any number of motors. For now, we just put them on a circle..
    void setEngines(Engine[] engines);

  private:
    Engine[] engines; //create this dynamicaly, really cool
}

ESC frontRight(MOTOR_FR_OUT_PIN, ESC_MIN, ESC_MAX);
ESC frontLeft(MOTOR_FL_OUT_PIN, ESC_MIN, ESC_MAX);
ESC rearRight(MOTOR_RR_OUT_PIN, ESC_MIN, ESC_MAX);
ESC rearLeft(MOTOR_RL_OUT_PIN, ESC_MIN, ESC_MAX);

#endif
