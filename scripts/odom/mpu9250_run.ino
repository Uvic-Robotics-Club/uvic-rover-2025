#include <Wire.h>
#include <EEPROM.h>
#define BOLDER_FLIGHT_ENABLE_PRINTF
#include "mpu9250.h"
#include <MadgwickAHRS.h>

/* ---------- EEPROM struct (same layout) -------------------------------------------- */
struct CalData {
  float gBias[3];
  float aBias[3];
  float mBias[3];
  float mScale[3];
  uint32_t tag;
};
constexpr uint32_t CAL_TAG = 0x9250CA1U;
CalData cal;

/* ---------- objects ---------------------------------------------------------------- */
bfs::Mpu9250 imu;
Madgwick     filter;

/* ---------- constants -------------------------------------------------------------- */
constexpr float GRAV = 9.80665f;

/* ================================================================================== */
void loadCalOrZero()
{
  EEPROM.get(0, cal);
  if (cal.tag != CAL_TAG) {
    //Serial.println(F("No valid EEPROM calibration data – using zeros"));
    memset(&cal, 0, sizeof(cal));
  } else {
    //Serial.println(F("Calibration data loaded from EEPROM"));
  }
}

/* ================================================================================== */
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  loadCalOrZero();

  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
  while (!imu.Begin()) { delay(100); }
  imu.ConfigDlpfBandwidth(bfs::Mpu9250::DLPF_BANDWIDTH_20HZ);
  imu.ConfigSrd(19);               // 50 Hz
  filter.begin(50.0f);
  filter.setBeta(0.25f);
}

/* ================================================================================== */
void loop() {
  if (!imu.Read()) return;

  float ax = imu.accel_x_mps2() - cal.aBias[0];
  float ay = imu.accel_y_mps2() - cal.aBias[1];
  float az = -(imu.accel_z_mps2() - cal.aBias[2]);

  float gx = imu.gyro_x_radps()  - cal.gBias[0];
  float gy = imu.gyro_y_radps()  - cal.gBias[1];
  float gz = imu.gyro_z_radps()  - cal.gBias[2];

  float mx = (imu.mag_x_ut() - cal.mBias[0]) * cal.mScale[0];
  float my = (imu.mag_y_ut() - cal.mBias[1]) * cal.mScale[1];
  float mz = (imu.mag_z_ut() - cal.mBias[2]) * cal.mScale[2];

  ax /= GRAV; ay /= GRAV; az /= GRAV;

  filter.update(gx,gy,gz, ax,ay,az, mx,my,mz);

  float qw,qx,qy,qz; filter.getQuaternion(qw,qx,qy,qz);

  /* 13-field stream (SI units) */
  Serial.print(ax*GRAV,4); Serial.print(' ');
  Serial.print(ay*GRAV,4); Serial.print(' ');
  Serial.print(az*GRAV,4); Serial.print(' ');

  Serial.print(mx,4); Serial.print(' ');
  Serial.print(my,4); Serial.print(' ');
  Serial.print(mz,4); Serial.print(' ');

  Serial.print(gx,4); Serial.print(' ');
  Serial.print(gy,4); Serial.print(' ');
  Serial.print(gz,4); Serial.print(' ');

  Serial.print(qw,4); Serial.print(' ');
  Serial.print(qx,4); Serial.print(' ');
  Serial.print(qy,4); Serial.print(' ');
  Serial.print(qz,4); Serial.println();
}
