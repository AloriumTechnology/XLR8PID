#ifndef XLR8PID_H
#define XLR8PID_H

#ifdef ARDUINO_XLR8

#define MAX_PIDS 6
#define INVALID_PID 255

typedef struct {
  bool enable;
  int16_t Kp;
  int16_t Ki;
  int16_t Kd;
  int16_t Sp;
} PIDSettings_t;

typedef struct {
  PIDSettings_t settings;
} pid_t;

class XLR8PID {
  public:
    XLR8PID();
    void disable();
    void enable();
    void reset();
    void SetTunings(int16_t,int16_t,int16_t,int16_t);
    int16_t Compute();
    int16_t getPv();
    void setPv(int16_t);
    int16_t getKp();
    void setKp(int16_t);
    int16_t getKi();
    void setKi(int16_t);
    int16_t getKd();
    void setKd(int16_t);
    int16_t getSp();
    void setSp(int16_t);
  private:
    uint8_t pidIndex;
    void update();
};

#else
#error "XLRPID library requires Tools->Board->XLR8xxx selection. Install boards from https://github.com/AloriumTechnology/Arduino_Boards"
#endif

#endif
