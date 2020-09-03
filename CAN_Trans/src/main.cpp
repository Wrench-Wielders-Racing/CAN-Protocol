#include <SPI.h>          //Library for using SPI Communication 
#include <mcp2515.h>      //Library for using CAN Communication
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

struct can_frame canMsg;
MCP2515 mcp2515(10);

void setup() 
{
  while (!Serial);
  Serial.begin(9600);
  
  SPI.begin();               //Begins SPI communication
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS,MCP_8MHZ); //Sets CAN at speed 500KBPS and Clock 8MHz
  mcp2515.setNormalMode();

  Serial.println("Initialize MPU6050");

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  // mpu.setThreshold(3);
  
  // Check settings
  checkSettings();
}

void checkSettings()
{
  Serial.println();
  
  Serial.print(" * Sleep Mode:            ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");
  
  Serial.print(" * Clock Source:          ");
  switch(mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }
  
  Serial.print(" * Accelerometer:         ");
  switch(mpu.getRange())
  {
    case MPU6050_RANGE_16G:            Serial.println("+/- 16 g"); break;
    case MPU6050_RANGE_8G:             Serial.println("+/- 8 g"); break;
    case MPU6050_RANGE_4G:             Serial.println("+/- 4 g"); break;
    case MPU6050_RANGE_2G:             Serial.println("+/- 2 g"); break;
  }  

  Serial.print(" * Accelerometer offsets: ");
  Serial.print(mpu.getAccelOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getAccelOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getAccelOffsetZ());
  
  Serial.print(" * Gyroscope:         ");
  switch(mpu.getScale())
  {
    case MPU6050_SCALE_2000DPS:        Serial.println("2000 dps"); break;
    case MPU6050_SCALE_1000DPS:        Serial.println("1000 dps"); break;
    case MPU6050_SCALE_500DPS:         Serial.println("500 dps"); break;
    case MPU6050_SCALE_250DPS:         Serial.println("250 dps"); break;
  } 
  
  Serial.print(" * Gyroscope offsets: ");
  Serial.print(mpu.getGyroOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getGyroOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getGyroOffsetZ());
  
  Serial.println();
}

void loop() 
{
  Vector rawAccel = mpu.readRawAccel();
  Vector normAccel = mpu.readNormalizeAccel();
  Vector rawGyro = mpu.readRawGyro();
  Vector normGyro = mpu.readNormalizeGyro();

  /*Serial.print(" ACCEL. Xraw = ");
  Serial.print(rawAccel.XAxis);
  Serial.print(" ACCEL. Yraw = ");
  Serial.print(rawAccel.YAxis);
  Serial.print(" ACCEL. Zraw = ");
  Serial.println(rawAccel.ZAxis);*/
  
  Serial.print(" ACCEL. Xnorm = ");
  Serial.print(normAccel.XAxis);
  Serial.print(" ACCEL. Ynorm = ");
  Serial.print(normAccel.YAxis);
  Serial.print(" ACCEL. Znorm = ");
  Serial.println(normAccel.ZAxis);
  
  /*Serial.print(" GYRO. Xraw = ");
  Serial.print(rawGyro.XAxis);
  Serial.print(" GYRO. Yraw = ");
  Serial.print(rawGyro.YAxis);
  Serial.print(" GYRO. Zraw = ");
  Serial.println(rawGyro.ZAxis);*/

  Serial.print(" GYRO. Xnorm = ");
  Serial.print(normGyro.XAxis);
  Serial.print(" GYRO. Ynorm = ");
  Serial.print(normGyro.YAxis);
  Serial.print(" GYRO. Znorm = ");
  Serial.println(normGyro.ZAxis);

  int Ax = normAccel.XAxis;
  int Ay = normAccel.YAxis;
  int Az = normAccel.ZAxis;
  int Gx = normGyro.XAxis;
  int Gy = normGyro.YAxis;
  int Gz = normGyro.ZAxis;

  canMsg.can_id  = 0x36;           //CAN id as 0x036 
  canMsg.can_dlc = 8;               //CAN data length as 8
  canMsg.data[0] = canMsg.can_id;
  canMsg.data[1] = Ax;
  canMsg.data[2] = Ay;
  canMsg.data[3] = Az;
  canMsg.data[4] = Gx;
  canMsg.data[5] = Gy;
  canMsg.data[6] = Gz;
  canMsg.data[7] = 0x00;
  mcp2515.sendMessage(&canMsg);     //Sends the CAN message

  delay(10);
}