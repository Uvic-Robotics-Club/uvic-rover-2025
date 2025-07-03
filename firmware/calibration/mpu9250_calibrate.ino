/****************************************************************************************
 *  One-time calibration for MPU-9250  (UNo / bfs::Mpu9250 driver)
 *  - 3 s   still  → gyro + accel bias
 *  - 15 s  rotate → mag hard/soft-iron
 *  Saves all 12 floats + a magic tag to EEPROM
 ****************************************************************************************/
#include <Wire.h>
#include <EEPROM.h>
#include "mpu9250.h"

/* ------------ EEPROM layout --------------------------------------------------------- */
struct CalData {
  float gBias[3];     // rad/s
  float aBias[3];     // m/s²
  float mBias[3];     // µT
  float mScale[3];    // unit-less
  uint32_t tag;       // 0x9250CALIU
};
constexpr uint32_t CAL_TAG = 0x9250CA1U;

/* ------------ objects --------------------------------------------------------------- */
bfs::Mpu9250 imu;
CalData cal;

/* ------------ helpers --------------------------------------------------------------- */
void saveCal() { EEPROM.put(0, cal); }

void beep() { tone(8, 2000, 100); }   // piezo on pin 8 (optional)

/* ==================================================================================== */
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
  while (!imu.Begin()) { delay(100); }

  imu.ConfigDlpfBandwidth(bfs::Mpu9250::DLPF_BANDWIDTH_41HZ);
  imu.ConfigSrd(9);                               // 100 Hz

  Serial.println(F("=== CALIBRATION ==="));
  Serial.println(F("1)  Keep the board STILL on a level surface…"));
  delay(2000);
  Serial.println(F("   collecting accel & gyro bias (3 s)…"));

  /* -------- 3 s still : avg gyro + accel ------------------------------------------- */
  float gSum[3]={0}, aSum[3]={0};
  const uint16_t N = 300;                         // 3 s @ 100 Hz
  for (uint16_t i=0;i<N;i++) {
    while (!imu.Read()) {}
    gSum[0]+=imu.gyro_x_radps();
    gSum[1]+=imu.gyro_y_radps();
    gSum[2]+=imu.gyro_z_radps();

    /* accel bias = (reading − gravity) */
    aSum[0]+=imu.accel_x_mps2();
    aSum[1]+=imu.accel_y_mps2();
    float azRaw = imu.accel_z_mps2();
    float gSign = (azRaw > 0) ? +1.0f : -1.0f;  // detect if Z is up or down
    aSum[2] += (azRaw - (gSign * 9.80665f));
  }
  for (int i=0;i<3;i++){
    cal.gBias[i]=gSum[i]/N;
    cal.aBias[i]=aSum[i]/N;
  }
  Serial.println(F("✔ gyro/accel bias done"));
  beep();

  /* -------- 15 s mag dance --------------------------------------------------------- */
  Serial.println(F("\n2)  Slowly TUMBLE the board through all orientations"));
  Serial.println(F("    (15 s, figure-8 motion)…"));
  float mMin[3]={ 1e6, 1e6, 1e6};
  float mMax[3]={-1e6,-1e6,-1e6};
  uint32_t tStart=millis();
  while (millis()-tStart < 15000) {
    while (!imu.Read()) {}
    float mx=imu.mag_x_ut(), my=imu.mag_y_ut(), mz=imu.mag_z_ut();
    if(mx<mMin[0])mMin[0]=mx; if(mx>mMax[0])mMax[0]=mx;
    if(my<mMin[1])mMin[1]=my; if(my>mMax[1])mMax[1]=my;
    if(mz<mMin[2])mMin[2]=mz; if(mz>mMax[2])mMax[2]=mz;
  }
  for(int i=0;i<3;i++){
    cal.mBias[i]  = (mMax[i]+mMin[i])*0.5f;                 // hard-iron
    float halfRange = (mMax[i]-mMin[i])*0.5f;
    float avgRange = ((mMax[0]-mMin[0])+(mMax[1]-mMin[1])+(mMax[2]-mMin[2]))/6.0f;
    if (halfRange > 1e-6f) {                                // soft-iron scale, with zero guard
      cal.mScale[i] = avgRange / halfRange;
    } else {
      // no variation on this axis—leave scale at 1.0
      cal.mScale[i] = 1.0f;
      }
    }
  Serial.println(F("✔ magnetometer cal done"));
  beep();

  /* -------- store to EEPROM -------------------------------------------------------- */
  cal.tag = CAL_TAG;
  saveCal();
  Serial.println(F("\nSaved calibration to EEPROM."));
  Serial.print(F("gBias  (rad/s): ")); Serial.println(cal.gBias[0],4);
  Serial.print(F("aBias  (m/s²): ")); Serial.println(cal.aBias[2],4);
  Serial.print(F("mBias  (µT)  : ")); Serial.println(cal.mBias[2],2);
  Serial.print(F("mScale       : ")); Serial.println(cal.mScale[2],2);
  Serial.println(F("\nRe-flash the RUN sketch now!"));
  while(true){}            // stop
}

void loop(){}
