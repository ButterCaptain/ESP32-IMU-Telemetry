
#include <WiFi.h>
#include <Wire.h>
#include <math.h>
#include <WebServer.h>
#include <cstdint>
#include <esp_now.h>
#include <esp_wifi.h>

namespace MathConstants 
{
  constexpr float EPSILON {0.0001f};
}

namespace Mpu6050Constants {
  const uint8_t MPU_ADDR =            0x68U; 
  const uint8_t POWER_REG =           0x6BU;
  const uint8_t ACCEL_CONFIG_ADDR =   0x1CU;
  const uint8_t ACCEL_4G_SET =        0x08U; 
  const uint8_t GYRO_CONFIG_ADDR =    0x1BU; 
  const uint8_t GYRO_500DPS_SET =     0x08U;
  const uint8_t ACCEL_XOUT_H =        0x3BU;

  const uint8_t DATA_BYTES_TO_READ =  14U;
  const float ACCEL_SENS_4G =         8192.0f; 
  const float GYRO_SENS_500DPS =      65.5f;
  const uint16_t CALIBRATION_COUNT =  300U;
 
  const uint8_t CLEAR_BITS =          0x00U;
}
struct ImuData16 
{
    int16_t AcX;
    int16_t AcY;
    int16_t AcZ;
    int16_t GyX; 
    int16_t GyY; 
    int16_t GyZ; 
};
struct ImuData32 
{
    int32_t AcX;
    int32_t AcY;
    int32_t AcZ;
    int32_t GyX; 
    int32_t GyY; 
    int32_t GyZ; 
};
struct IMUData_t {
    float gx, gy, gz; // Gyroscope (rad/s)
    float ax, ay, az; // Accelerometer (g)
    float dt;         // Delta time
};

  // Initalize Struct for Raw Data
  ImuData16 currentReading = {0,0,0,0,0,0}; 
  // Struct for Sums, used for inital calibration
  ImuData32 cal_SumReading = {0,0,0,0,0,0};
  // Struct for offset data measurements
  ImuData32 offset = {0,0,0,0,0,0}; 
  // Struct for Calibrated and converted data
  IMUData_t pure_cal_data = {0,0,0,0,0,0,0};

  // Beta defines the balance between Gyro and Accelerometer.
  // 0.1 is a standard starting point. Higher = more trust in accelerometer.
  const float beta {0.2f}; 

  // Tell the compiler to use 1-byte alignment (no padding)
  #pragma pack(push, 1) 
  struct QuaternionPacket {
    float q1; 
    float q2; 
    float q3; 
    float q4; 
  };
  // Revert back to normal compiler padding rules 
  #pragma pack(pop)
  struct Angles {
    float rollDegrees;
    float pitchDegrees;
  }; 

  Angles Angle = {0.0,0.0}; 

  // Get current time
  static uint32_t prevTime = 0U;
 

  // Global variables to hold the current orientation quaternion
  // Initialized to the identity quaternion (no rotation)
  QuaternionPacket Quarternion = {1.0,0,0,0}; 

  uint32_t loopCount {0U};  

  float dt = 0.1; // Delta time (how fast each loop runs)

  // -- Lets set up the WiFi -- 
  // 00:70:07:E6:D3:74
  uint8_t broadcastAddress[] = {0x00, 0x70, 0x07, 0xE6, 0xD3, 0x74};


  float findMean(int32_t Value);
  void SetOffset(int32_t &stat, float offset);
  void calculateEulerAngles();
  void handleRoot(); 
  void madgwickUpdate(IMUData_t &data);
  bool isI2CHealthy(const uint8_t status, const char* sector);
  void endTransmissionSafely(const char* sector);

  void setup(){

    Wire.begin(); // Start I2C bus
    Wire.setTimeout(3); // Timeout after 3 ms and force a bus reset
    Serial.begin(115200); // Start serial monitor

    // Wake up the Senor through power management 1
    Wire.beginTransmission(Mpu6050Constants::MPU_ADDR); 
    Wire.write(Mpu6050Constants::POWER_REG); // Address of Power management 1 register
    Wire.write(Mpu6050Constants::CLEAR_BITS); // Send the register 00000000 to wake up the sensor
    endTransmissionSafely("POWER-CONFIG"); 

    // 3. Configure the Gyroscope Range
    Wire.beginTransmission(Mpu6050Constants::MPU_ADDR);
    Wire.write(Mpu6050Constants::GYRO_CONFIG_ADDR);    // Target the GYRO_CONFIG register
    Wire.write(Mpu6050Constants::GYRO_500DPS_SET);    // Write 00001000 to set the range to ± 500 degrees/second
    endTransmissionSafely("GYRO-CONFIG");

    // Set the Accelerometer to +/- 4g
    Wire.beginTransmission(Mpu6050Constants::MPU_ADDR); 
    Wire.write(Mpu6050Constants::ACCEL_CONFIG_ADDR); // Address of Accelerometer Config Register
    Wire.write(Mpu6050Constants::ACCEL_4G_SET); // Send the register 00001000 to set +/- 4g
    endTransmissionSafely("ACCEL-CONFIG"); 

    // Start Station mode
    WiFi.mode(WIFI_STA); 

    delay(100); 

    if(esp_now_init() != ESP_OK){
      // Fall back to another system or retry
      Serial.println("Error Initializing ESP-NOW"); 
      return; 
    }

    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

    // Register the peer (The Receiver)
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    peerInfo.ifidx = WIFI_IF_STA;

    esp_err_t addStatus = esp_now_add_peer(&peerInfo);
    if (addStatus == ESP_OK) {
        Serial.println("Pairing Success: Peer added to list.");
    } else {
        Serial.print("Pairing Failed: ");
        Serial.println(esp_err_to_name(addStatus));
    }

    prevTime = micros();
  }

  uint32_t lastSendTime = 0;
  const uint32_t sendInterval = 50; // Send data every 50ms


  void loop(){

    // Get current time
    static uint32_t prevTime = 0U; 
    const uint32_t curTime = micros(); // Returns it in micro seconds
    const int32_t loopDur = curTime - prevTime; // Calculate the duration of the previous loop.

    if(loopDur != 0)
    {
      pure_cal_data.dt = loopDur/ 1000000.0; // Converts microseconds into seconds & Update loop dt
    }

    prevTime = curTime; 

    // Lets read some Data
    // The Accelerometer Addresses
    // X - Registers 0x3B to 0x3C
    // Y - Registers 0x3D to 0x3E
    // Z - Registers 0x3F to 0x40
    // Point to the first register to read from, 0x3B
    Wire.beginTransmission(Mpu6050Constants::MPU_ADDR); 
    Wire.write(Mpu6050Constants::ACCEL_XOUT_H); 
    endTransmissionSafely("SET POINTER TO DATA"); 
    // Controller Request from peripherial device, Params: Address(device address), Quantity (in Bytes) , stop (optional) 

    Wire.requestFrom(Mpu6050Constants::MPU_ADDR,Mpu6050Constants::DATA_BYTES_TO_READ,true); 
    // This reads the first register and reads the next register after it. 
    // When storing the value into AcX we must shift the first 8 bits to the left of the value AcX, then combine them with the next 
    // Register with Or. 

    // Ex
    // 10101010 00000000
    // 00000000 10101010
    // -----------------
    // 10101010 10101010

    // The read() function auto increments to the next byte (register) in the order. 
    currentReading.AcX = Wire.read() << 8 | Wire.read();
    currentReading.AcY = Wire.read() << 8 | Wire.read(); 
    currentReading.AcZ = Wire.read() << 8 | Wire.read();

    // Skip Temp Values
    Wire.read();
    Wire.read();
    
    // Read Gyro Raw Values
    currentReading.GyX = Wire.read() << 8 | Wire.read();
    currentReading.GyY = Wire.read() << 8 | Wire.read();
    currentReading.GyZ = Wire.read() << 8 | Wire.read();



    // This Calibrates the values at the start
    if(loopCount <= Mpu6050Constants::CALIBRATION_COUNT)
    {
      loopCount += 1;

      cal_SumReading.AcX += currentReading.AcX;
      cal_SumReading.AcY += currentReading.AcY; 
      cal_SumReading.AcZ += currentReading.AcZ;

      cal_SumReading.GyX += currentReading.GyX;
      cal_SumReading.GyY += currentReading.GyY; 
      cal_SumReading.GyZ += currentReading.GyZ;

      delay(3); 

      if(loopCount >= Mpu6050Constants::CALIBRATION_COUNT)
      {
        SetOffset(offset.AcX, findMean(cal_SumReading.AcX));  
        SetOffset(offset.AcY, findMean(cal_SumReading.AcY));  
        offset.AcZ = Mpu6050Constants::ACCEL_SENS_4G - findMean(cal_SumReading.AcZ);
        SetOffset(offset.GyX, findMean(cal_SumReading.GyX)); 
        SetOffset(offset.GyY, findMean(cal_SumReading.GyY));
        SetOffset(offset.GyZ, findMean(cal_SumReading.GyZ)); 
        Serial.println("Calibration Complete!\n");
      }
    }


    // Accelerometer (Dividing by 8192.0 because we chose +/- 4g)
    pure_cal_data.ax = static_cast<float>(currentReading.AcX + offset.AcX) / Mpu6050Constants::ACCEL_SENS_4G;
    pure_cal_data.ay = static_cast<float>(currentReading.AcY + offset.AcY) / Mpu6050Constants::ACCEL_SENS_4G;
    pure_cal_data.az = static_cast<float>(currentReading.AcZ + offset.AcZ) / Mpu6050Constants::ACCEL_SENS_4G;

    // Gyroscope (dividing by 65.5 because we set the gyro at +/- 500 deg/s)
    // Returns Radians because the PI/180
    pure_cal_data.gx = (static_cast<float>(currentReading.GyX + offset.GyX) / Mpu6050Constants::GYRO_SENS_500DPS) * DEG_TO_RAD; 
    pure_cal_data.gy = (static_cast<float>(currentReading.GyY + offset.GyY) / Mpu6050Constants::GYRO_SENS_500DPS) * DEG_TO_RAD; 
    pure_cal_data.gz = (static_cast<float>(currentReading.GyZ + offset.GyZ) / Mpu6050Constants::GYRO_SENS_500DPS) * DEG_TO_RAD; 

    if(loopCount > Mpu6050Constants::CALIBRATION_COUNT)
    {
      // Update the orientation
      madgwickUpdate(pure_cal_data);
      
      // Send Data

      static uint32_t lastSend = 0;
      if (millis() - lastSend > 50) { // Slowed down to 50ms for debugging
          lastSend = millis();
          
          esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Quarternion, sizeof(Quarternion));
          
          if (result == ESP_OK) {
              //Serial.println("Sent with success");
          } else {
              //Serial.print("Error sending: ");
              //Serial.println(esp_err_to_name(result));
          }
      }
    }

  }

  float findMean(int32_t Value){
    float mean = static_cast<float>(Value)/ static_cast<float>(loopCount); 
    return mean; 
  }

  void SetOffset(int32_t &stat, float offset){
    stat -= offset; 
  }

  void madgwickUpdate(IMUData_t &data) {
      float recipNorm;
      float s1, s2, s3, s4;
      float qDot1, qDot2, qDot3, qDot4;
      float _2q1, _2q2, _2q3, _2q4, _4q1, _4q2, _4q3 ,_8q2, _8q3, q1q1, q2q2, q3q3, q4q4;

      // If accelerometer is returning zeros, skip the accel update and just do gyro
      if((abs(data.ax) < MathConstants::EPSILON) && 
          (abs(data.ay) < MathConstants::EPSILON) && 
          (abs(data.az) < MathConstants::EPSILON)) 
      {
          // Fallback to gyro-only code here if desired
          return; 
      }

      // Normalize accelerometer measurement
      // The math requires the measured gravity vector to have a length of 1
      recipNorm = 1.0f / sqrt(data.ax * data.ax + data.ay * data.ay + data.az * data.az);
      data.ax *= recipNorm;
      data.ay *= recipNorm;
      data.az *= recipNorm;

      // 3. Pre-calculate repeated terms to save microcontroller CPU cycles
      _2q1 = 2.0f * Quarternion.q1;
      _2q2 = 2.0f * Quarternion.q2;
      _2q3 = 2.0f * Quarternion.q3;
      _2q4 = 2.0f * Quarternion.q4;
      _4q1 = 4.0f * Quarternion.q1;
      _4q2 = 4.0f * Quarternion.q2;
      _4q3 = 4.0f * Quarternion.q3;
      _8q2 = 8.0f * Quarternion.q2;
      _8q3 = 8.0f * Quarternion.q3;
      q1q1 = Quarternion.q1 * Quarternion.q1;
      q2q2 = Quarternion.q2 * Quarternion.q2;
      q3q3 = Quarternion.q3 * Quarternion.q3;
      q4q4 = Quarternion.q4 * Quarternion.q4;

      // Gradient Descent Step: Calculate the gradient (nabla f)
      // This is the expanded algebra of (Jacobian Transpose * Error Function)
      s1 = _4q1 * q3q3 + _2q3 * data.ax + _4q1 * q2q2 - _2q2 * data.ay;
      s2 = _4q2 * q4q4 - _2q4 * data.ax + 4.0f * q1q1 * Quarternion.q2 - _2q1 * data.ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * data.az;
      s3 = 4.0f * q1q1 * Quarternion.q3 + _2q1 * data.ax + _4q3 * q4q4 - _2q4 * data.ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * data.az;
      s4 = 4.0f * q2q2 * Quarternion.q4 - _2q2 * data.ax + 4.0f * q3q3 * Quarternion.q4 - _2q3 * data.ay;

      // Normalize the gradient step
      recipNorm = 1.0f / sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
      s1 *= recipNorm;
      s2 *= recipNorm;
      s3 *= recipNorm;
      s4 *= recipNorm;

      // Calculate the Gyroscope Derivative (Equation 12 from your image)
      qDot1 = 0.5f * (-Quarternion.q2 * data.gx - Quarternion.q3 * data.gy - Quarternion.q4 * data.gz);
      qDot2 = 0.5f * ( Quarternion.q1 * data.gx + Quarternion.q3 * data.gz - Quarternion.q4 * data.gy);
      qDot3 = 0.5f * ( Quarternion.q1 * data.gy - Quarternion.q2 * data.gz + Quarternion.q4 * data.gx);
      qDot4 = 0.5f * ( Quarternion.q1 * data.gz + Quarternion.q2 * data.gy - Quarternion.q3 * data.gx);

      // The FUSION: Apply the accelerometer feedback to the gyroscope data
      qDot1 -= beta * s1;
      qDot2 -= beta * s2;
      qDot3 -= beta * s3;
      qDot4 -= beta * s4;

      // Integrate the fused derivative to find the new quaternion
      Quarternion.q1 += qDot1 * data.dt;
      Quarternion.q2 += qDot2 * data.dt;
      Quarternion.q3 += qDot3 * data.dt;
      Quarternion.q4 += qDot4 * data.dt;

      // Normalize the final quaternion so it remains a valid rotation
      recipNorm = 1.0f / sqrt(Quarternion.q1 * Quarternion.q1 + Quarternion.q2 * Quarternion.q2 + Quarternion.q3 * Quarternion.q3 + Quarternion.q4 * Quarternion.q4);
      Quarternion.q1 *= recipNorm;
      Quarternion.q2 *= recipNorm;
      Quarternion.q3 *= recipNorm;
      Quarternion.q4 *= recipNorm;
  }

  // This function was used for earlier Development
  void calculateEulerAngles() {
      // Calculate Roll
      float sinr_cosp = 2.0f * (Quarternion.q1 * Quarternion.q2 + Quarternion.q3 * Quarternion.q4);
      float cosr_cosp = 1.0f - 2.0f * (Quarternion.q2 * Quarternion.q2 + Quarternion.q3 * Quarternion.q3);
      float rollRadians = atan2(sinr_cosp, cosr_cosp);

      // Calculate Pitch
      float sinp = 2.0f * (Quarternion.q1 * Quarternion.q3 - Quarternion.q4 * Quarternion.q2);
      // Clamp sinp to exactly -1.0 or 1.0 to prevent NaN errors in asin()
      if (sinp > 1.0f) {
        sinp = 1.0f;
      }
      if (sinp < -1.0f){ 
        sinp = -1.0f;
      }
      float pitchRadians = asin(sinp);

      // Convert to Degrees for the Serial Monitor
      // Lastly we need to switch these based on how the hardware is configured
      Angle.rollDegrees = pitchRadians * (180.0f / PI);
      Angle.pitchDegrees = rollRadians * (180.0f / PI);
  }

  bool isI2CHealthy(const uint8_t status, const char* sector) {
    if (status == 0U) {
        return true; // All good!
    }

    // If we reach here, something went wrong.
    Serial.print("![I2C ERROR] Sector: ");
    Serial.print(sector);
    Serial.print(" | Code: ");
    
    switch (status) {
        case 1: Serial.println("1 (Data too long)"); break;
        case 2: Serial.println("2 (Address NACK)"); break;
        case 3: Serial.println("3 (Data NACK)");    break;
        case 4: Serial.println("4 (Other error)");  break;
        default: Serial.println("Unknown Error");   break;
    }
    return false;
}

void endTransmissionSafely(const char* sector){
  const uint8_t status = Wire.endTransmission(true); 
  if (!isI2CHealthy(status, sector)) {
      // Optional: Trigger a reset or skip this loop frame
      return; 
  }
}



