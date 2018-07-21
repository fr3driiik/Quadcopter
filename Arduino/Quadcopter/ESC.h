#ifndef ESC_H
#define ESC_H

class ESC {
  public:
    ESC(uint8_t escPin, unsigned int min, unsigned int max);
    void arm();
    void disarm();
    void setSpeed(float speed); // 0.00-1.00

  private:
    uint8_t escPin;
    unsigned int min, max;
}

#endif

