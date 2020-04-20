#include "XLR8PID.h"

#define NUM_PIDS 1

XLR8PID pids[NUM_PIDS];

void setup() {

  // Start the serial interface
  Serial.begin(115200);

  Serial.println("Testing Started, Enabling PIDs");
  Serial.println();

  // Enable all pids
  for (int idx = 0; idx < NUM_PIDS; idx++) {
    pids[idx].enable();
    pids[idx].SetTunings(1, 0, 0, 20);
    pids[idx].setPv(19);
  }

  for (long idx = 0; idx < NUM_PIDS; idx++) {
    Serial.print("Testing PID ");
    Serial.print(idx);
    Serial.println(":");
    Serial.print("Kp: ");
    Serial.print(pids[idx].getKp());
    Serial.print(" Ki: ");
    Serial.print(pids[idx].getKi());
    Serial.print(" Kd: ");
    Serial.print(pids[idx].getKd());
    Serial.print(" Sp: ");
    Serial.print(pids[idx].getSp());
    Serial.print(" Pv: ");
    Serial.print(pids[idx].getPv());
    Serial.print(" Output: ");
    Serial.print(pids[idx].Compute());
    Serial.println();
  }

}

void loop() {

}
