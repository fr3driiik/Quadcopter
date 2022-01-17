#include "IMU.h"
#include "Receiver.h"
#include "PIDs.h"
#include "Utils.h"
#include "ESCManager.h"

namespace Control {
  enum Mode {
    RATE_CONTROL,
    ATTITUDE_CONTROL,
    //GO_TO_POSITION,
    //RETURN_HOME,
  };
  Mode mode = RATE_CONTROL;

  void initialize() {
    setRatePidsIntegralLimits(-40, 40); //direct engine influence
    setStablePidsIntegralLimits(-202.5, 202.5); //max degrees per second to get right degree
    ESCManager::initialize();
  }

  void update(float deltaTime) {
    if(Receiver::throttleIn > 0.05){ //calc PIDS and run engines accordingly
      //calc stab pids
      computeStabPids(deltaTime);

      //yaw change desired? overwrite stab output and yaw target
      if(abs(Receiver::yawIn) > 0.05){
        yaw_stab_output = Utils::fromDecimalPercent(Receiver::yawIn, -YAW_MAX_DPS, YAW_MAX_DPS);
        yaw_target = yawDegrees;
      }

      //calc rate pids
      computeRatePids(deltaTime);

      ESCManager::setInput(pitch_output, roll_output, yaw_output, Receiver::throttleIn); //should actually feed values in % (pyr -100 - 100, throttle 0-100)
    } else { //too low throttle
      ESCManager::tooLowThrottle();
      #if DEBUG_OUTPUT
        Serial.print("Engines off");
      #endif

      //reset yaw
      yaw_target = yawDegrees;

      //reset pids
      resetPids();
    }
  }
}
