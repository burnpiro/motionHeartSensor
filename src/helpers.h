#include <Wire.h>

#define MPU9250_ADDRESS 0x68
#define MAG_ADDRESS 0x0C

#define GYRO_FULL_SCALE_250_DPS 0x00
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

#define ACC_FULL_SCALE_2_G 0x00
#define ACC_FULL_SCALE_4_G 0x08
#define ACC_FULL_SCALE_8_G 0x10
#define ACC_FULL_SCALE_16_G 0x18

uint8_t            socketNumber;
String             sensorData = "";
int16_t ax,ay,az,gx,gy,gz;

uint8_t aScale = ACC_FULL_SCALE_2_G, gScale = GYRO_FULL_SCALE_250_DPS; // set scale for gyro and acc (250DPS-200DPS and 2G-16G)
float aResolution, gResolution;

volatile int BPM;                   // int that holds raw Analog in 0. updated every 2mS
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // int that holds the time interval between beats! Must be seeded!
volatile boolean Pulse = false;     // "True" when User's live heartbeat is detected. "False" when not a "live beat".
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.

 // This function read Nbytes bytes from I2C device at address Address.
// Put read bytes starting at register Register in the Data array.
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
 // Set register address
 Wire.beginTransmission(Address);
 Wire.write(Register);
 Wire.endTransmission();

 // Read Nbytes
 Wire.requestFrom(Address, Nbytes);
 uint8_t index=0;
 while (Wire.available())
 Data[index++]=Wire.read();
}

// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

void setAccResolution() {
  switch(aScale) {
    case ACC_FULL_SCALE_2_G:
      aResolution = 2.0/32768.0;
      break;
    case ACC_FULL_SCALE_4_G:
      aResolution = 4.0/32768.0;
      break;
    case ACC_FULL_SCALE_8_G:
      aResolution = 8.0/32768.0;
      break;
    case ACC_FULL_SCALE_16_G:
      aResolution = 16.0/32768.0;
      break;
  }
}

void setGyroResolution() {
  switch(gScale) {
    case GYRO_FULL_SCALE_250_DPS:
      gResolution = 250.0/32768.0;
      break;
    case GYRO_FULL_SCALE_500_DPS:
      gResolution = 500.0/32768.0;
      break;
    case GYRO_FULL_SCALE_1000_DPS:
      gResolution = 1000.0/32768.0;
      break;
    case GYRO_FULL_SCALE_2000_DPS:
      gResolution = 2000.0/32768.0;
      break;
  }
}

float normalizeSensorValue(float sensorValue, float resolution) {
  return (float)sensorValue*resolution;
}

void setupSensor() {
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,gScale);
  setGyroResolution();
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,aScale);
  setAccResolution();
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);

  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
}

void getMotionSensorData() {

  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);

  // Create 16 bits values from 8 bits data

  // Accelerometer
  ax=-(Buf[0]<<8 | Buf[1]);
  ay=-(Buf[2]<<8 | Buf[3]);
  az=Buf[4]<<8 | Buf[5];

  // Gyroscope
  gx=-(Buf[8]<<8 | Buf[9]);
  gy=-(Buf[10]<<8 | Buf[11]);
  gz=Buf[12]<<8 | Buf[13];

  sensorData = String(String(normalizeSensorValue(ax, aResolution)) + ',' + String(normalizeSensorValue(ay, aResolution)) + ',' + String(normalizeSensorValue(az, aResolution)) + ',' + String(normalizeSensorValue(gx, gResolution)) + ',' + String(normalizeSensorValue(gy, gResolution)) + ',' + String(normalizeSensorValue(gz, gResolution)));
}

void getPulseSensorData() {
  Pulse = analogRead(A0);
}
