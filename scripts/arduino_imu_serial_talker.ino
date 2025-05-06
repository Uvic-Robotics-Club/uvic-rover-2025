#include <Wire.h>
#include <MPU9250.h>

// Create an instance of the sensor
MPU9250 mpu;

// Global variables to store calculated calibration offsets
// (In this library, accelerometer is by default in [g], gyro in [deg/s].)
float accelBias[3] = {0.0f, 0.0f, 0.0f};
float gyroBias[3]  = {0.0f, 0.0f, 0.0f};

// Number of samples to average for manual calibration
const int numSamples = 500;

//----------------------------------------------------------
// Calibration routine: Computes offsets by averaging samples
//----------------------------------------------------------
void calibrateIMU() {
  float axSum = 0, aySum = 0, azSum = 0;
  float gxSum = 0, gySum = 0, gzSum = 0;

  // Collect data samples for calibration
  for (int i = 0; i < numSamples; i++) {
    // Update sensor readings
    // (mpu.update() returns true if new data is available)
    if (mpu.update()) {
      // Sum up accelerometer readings (in g)
      axSum += mpu.getAccX();
      aySum += mpu.getAccY();
      azSum += mpu.getAccZ();

      // Sum up gyroscope readings (in deg/s)
      gxSum += mpu.getGyroX();
      gySum += mpu.getGyroY();
      gzSum += mpu.getGyroZ();
    }
    delay(10); // ~100 Hz sampling
  }

  // Compute average offsets
  accelBias[0] = axSum / numSamples;
  accelBias[1] = aySum / numSamples;
  // For Z-axis, subtract ~1g so a stationary sensor reads near 0g
  accelBias[2] = (azSum / numSamples) - 1.0f;

  gyroBias[0] = gxSum / numSamples;
  gyroBias[1] = gySum / numSamples;
  gyroBias[2] = gzSum / numSamples;

  //=====================UNCOMMENT TO DEBUG=====================
  // Serial.println("Calibration complete!");
  // Serial.print("Accelerometer Biases: X="); Serial.print(accelBias[0], 4); Serial.print(" g, ");
  // Serial.print("Y="); Serial.print(accelBias[1], 4); Serial.print(" g, ");
  // Serial.print("Z="); Serial.print(accelBias[2], 4); Serial.println(" g");
  //
  // Serial.print("Gyroscope Biases: X="); Serial.print(gyroBias[0], 4); Serial.print(" deg/s, ");
  // Serial.print("Y="); Serial.print(gyroBias[1], 4); Serial.print(" deg/s, ");
  // Serial.print("Z="); Serial.print(gyroBias[2], 4); Serial.println(" deg/s");
  //===========================================================
}

void setup() {
  Serial.begin(115200);
  // Initialize I2C bus
  Wire.begin();

  // Configure the MPU9250; default address is 0x68 (if AD0 is low)
  mpu.setup(0x68);

  // Select the quaternion filter; available filters are NONE, MADGWICK, and MAHONY.
  // Here we choose the Madgwick filter (which is the default in many cases).
  //mpu.selectFilter(MPU9250::QuatFilterSel::MADGWICK);

  // Give sensor time to stabilize
  delay(250);

  // Calibrate the IMU (accelerometer and gyroscope)
  calibrateIMU();
}


void loop() {
  // Continuously update sensor data
  if (mpu.update()) {
    // Retrieve raw readings
    float ax = mpu.getAccX();   // in g
    float ay = mpu.getAccY();   // in g
    float az = mpu.getAccZ();   // in g
    float gx = mpu.getGyroX();  // in deg/s
    float gy = mpu.getGyroY();  // in deg/s
    float gz = mpu.getGyroZ();  // in deg/s
    float mx = mpu.getMagX();   // in mG (milligauss) or µT, depending on library defaults
    float my = mpu.getMagY();
    float mz = mpu.getMagZ();

    // Apply calibration offsets
    ax -= accelBias[0];
    ay -= accelBias[1];
    az -= accelBias[2];
    gx -= gyroBias[0];
    gy -= gyroBias[1];
    gz -= gyroBias[2];

    // Retrieve the quaternion from the built-in filter
    float qw = mpu.getQuaternionW();
    float qx = mpu.getQuaternionX();
    float qy = mpu.getQuaternionY();
    float qz = mpu.getQuaternionZ();

    //=====================UNCOMMENT TO DEBUG=====================
    // Serial.print("Accel: X="); Serial.print(ax, 3); Serial.print(" g, ");
    // Serial.print("Y="); Serial.print(ay, 3); Serial.print(" g, ");
    // Serial.print("Z="); Serial.print(az, 3); Serial.println(" g");
    //
    // Serial.print("Gyro: X="); Serial.print(gx, 3); Serial.print(" deg/s, ");
    // Serial.print("Y="); Serial.print(gy, 3); Serial.print(" deg/s, ");
    // Serial.print("Z="); Serial.print(gz, 3); Serial.println(" deg/s");
    //
    // Serial.print("Mag: X="); Serial.print(mx, 3); Serial.print(" mG, ");
    // Serial.print("Y="); Serial.print(my, 3); Serial.print(" mG, ");
    // Serial.print("Z="); Serial.print(mz, 3); Serial.println(" mG");
    // Serial.println("--------------------------------------------------");
    // delay(200);
    //============================================================

    // Print sensor values in one line (similar to your original code)
    Serial.print(ax, 3); Serial.print(" ");
    Serial.print(ay, 3); Serial.print(" ");
    Serial.print(az, 3); Serial.print(" ");
    Serial.print(mx, 3); Serial.print(" ");
    Serial.print(my, 3); Serial.print(" ");
    Serial.print(mz, 3); Serial.print(" ");
    Serial.print(gx, 3); Serial.print(" ");
    Serial.print(gy, 3); Serial.print(" ");
    Serial.print(gz, 3); Serial.print(" ");
    Serial.print(qw, 3); Serial.print(" ");
    Serial.print(qx, 3); Serial.print(" ");
    Serial.print(qy, 3); Serial.print(" ");
    Serial.print(qz, 3); Serial.println();

    // ~100 Hz loop from our 10ms delay in calibrateIMU (or adjust as needed)
  }

  delay(50);
}
