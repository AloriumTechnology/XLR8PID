#include <Arduino.h>

#include "XLR8PID.h"

#define PIDCR_ADDR    _SFR_MEM8(0xEA)
#define PID_KD_H_ADDR _SFR_MEM8(0xEB)
#define PID_KD_L_ADDR _SFR_MEM8(0xEC)
#define PID_KT_H_ADDR _SFR_MEM8(0xED)
#define PID_KT_L_ADDR _SFR_MEM8(0xEE)
#define PID_KP_H_ADDR _SFR_MEM8(0xEF)
#define PID_KP_L_ADDR _SFR_MEM8(0xF0)
#define PID_SP_H_ADDR _SFR_MEM8(0xF1)
#define PID_SP_L_ADDR _SFR_MEM8(0xF2)
#define PID_PV_H_ADDR _SFR_MEM8(0xF3)
#define PID_PV_L_ADDR _SFR_MEM8(0xF4)
#define PID_OP_H_ADDR _SFR_MEM8(0xF5)
#define PID_OP_L_ADDR _SFR_MEM8(0xF6)

#define PEN  7
#define PDIS 6
#define PUP  5

static pid_t pids[MAX_PIDS];

uint8_t PIDCount = 0;

XLR8PID::XLR8PID() {
  if (PIDCount < MAX_PIDS) {
    this->pidIndex = PIDCount++;
    pids[this->pidIndex].settings.enable = 1;
    this->update();
  }
  else {
    this->pidIndex = INVALID_PID;
  }
}

void XLR8PID::disable() {
  pids[this->pidIndex].settings.enable = 0;
  this->update();
}

void XLR8PID::enable() {
  pids[this->pidIndex].settings.enable = 1;
  this->update();
}

void XLR8PID::reset() {
  PIDCR_ADDR = (pids[this->pidIndex].settings.enable << PEN) | (!pids[this->pidIndex].settings.enable << PDIS) 
    | (1 << PUP) | (0x0f & this->pidIndex);
}

void XLR8PID::SetTunings(int16_t Kp, int16_t Ki, int16_t Kd, int16_t Sp) {
  this->setKp(Kp);
  this->setKi(Ki);
  this->setKd(Kd);
  this->setSp(Sp);
}

int16_t XLR8PID::Compute() {
  PIDCR_ADDR = 0x0F & this->pidIndex;
  return ((uint8_t)(PID_OP_H_ADDR) << 8) | ((uint8_t)(PID_OP_L_ADDR));
}

int16_t XLR8PID::getPv() {
  PIDCR_ADDR = 0x0F & this->pidIndex;
  return ((uint8_t)(PID_PV_H_ADDR) << 8) | ((uint8_t)(PID_PV_L_ADDR));
}

void XLR8PID::setPv(int16_t Pv) {
  PIDCR_ADDR = 0x0F & this->pidIndex;
  PID_PV_H_ADDR = Pv >> 8;
  PID_PV_L_ADDR = Pv;
}

int16_t XLR8PID::getKp() {
  PIDCR_ADDR = 0x0F & this->pidIndex;
  return ((uint8_t)(PID_KP_H_ADDR) << 8) | ((uint8_t)(PID_KP_L_ADDR));
}

void XLR8PID::setKp(int16_t Kp) {
  pids[this->pidIndex].settings.Kp = Kp;
  PID_KP_H_ADDR = pids[this->pidIndex].settings.Kp >> 8;
  PID_KP_L_ADDR = pids[this->pidIndex].settings.Kp;
  this->update();
}

int16_t XLR8PID::getKi() {
  PIDCR_ADDR = 0x0F & this->pidIndex;
  return ((uint8_t)(PID_KT_H_ADDR) << 8) | ((uint8_t)(PID_KT_L_ADDR));
}

void XLR8PID::setKi(int16_t Ki) {
  pids[this->pidIndex].settings.Ki = Ki;
  PID_KT_H_ADDR = pids[this->pidIndex].settings.Ki >> 8;
  PID_KT_L_ADDR = pids[this->pidIndex].settings.Ki;
  this->update();
}

int16_t XLR8PID::getKd() {
  PIDCR_ADDR = 0x0F & this->pidIndex;
  return ((uint8_t)(PID_KD_H_ADDR) << 8) | ((uint8_t)(PID_KD_L_ADDR));
}

void XLR8PID::setKd(int16_t Kd) {
  pids[this->pidIndex].settings.Kd = Kd;
  PID_KD_H_ADDR = pids[this->pidIndex].settings.Kd >> 8;
  PID_KD_L_ADDR = pids[this->pidIndex].settings.Kd;
  this->update();
}

int16_t XLR8PID::getSp() {
  PIDCR_ADDR = 0x0F & this->pidIndex;
  return ((uint8_t)(PID_SP_H_ADDR) << 8) | ((uint8_t)(PID_SP_L_ADDR));
}

void XLR8PID::setSp(int16_t Sp) {
  pids[this->pidIndex].settings.Sp = Sp;
  PID_SP_H_ADDR = pids[this->pidIndex].settings.Sp >> 8;
  PID_SP_L_ADDR = pids[this->pidIndex].settings.Sp;
  this->update();
}

void XLR8PID::update() {
  PIDCR_ADDR = (pids[this->pidIndex].settings.enable << PEN) | (!pids[this->pidIndex].settings.enable << PDIS) 
    | (1 << PUP) | (0x0f & this->pidIndex);
}

