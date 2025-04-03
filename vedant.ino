#include <SoftwareSerial.h>

const int flex1 = A0; // thumb sensor 5V to 2.8V
const int flex2 = A1; // index sensor 5V to 2.3V

int selectedMotor = 3;

//MPU6050 variables declaration
#include <Wire.h>

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
//end of MPU6050 declaration

SoftwareSerial BTSerial(0, 1);  // RX, TX

void setup() {
    //Serial.begin(9600);   // Serial Monitor
    BTSerial.begin(9600); // Bluetooth Module
    pinMode(flex1, INPUT);
    pinMode(flex2, INPUT);

    //MPU6050 initialization start
    Wire.begin();                      // Initialize comunication
    Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
    Wire.write(0x6B);                  // Talk to the register 6B
    Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
    Wire.endTransmission(true);        //end the transmission
  
  delay(20);
  //MPU6050 initialization end
}

void loop() {
        int flexValue1 = analogRead(flex1);
        int flexValue2 = analogRead(flex2);

        //thumb
        int angle1 = map(flexValue1, 573, 1023, 20, 180); //assuming min of 700 and max 900 analog value for sum of two flex sensors
        angle1 = constrain(angle1, 10, 170);

        //index
        int angle2 = map(flexValue2, 470, 1023, 0, 180); //assuming min of 700 and max 900 analog value for sum of two flex sensors
        angle2 = constrain(angle2, 10, 170);

        int thumbData = 7000 + angle1; // want 4 digits
       
        BTSerial.println(String(thumbData));  // Send command to HC-05;

        while (BTSerial.available()) {
          String response = BTSerial.readString();
        }
        delay(300);
        
        int indexData = 2000 + angle2; // want 4 digits

        BTSerial.println(String(indexData));  // Send command to HC-05

        while (BTSerial.available()) {
          String response = BTSerial.readString();
        }
        delay(300);  

        BTSerial.println(getMPU6050Bit());  // Send command to HC-05
 
        while (BTSerial.available()) {
          String response = BTSerial.readString();
        }
        delay(300);  
}

// Function to get the string of Bits for Motor 3 based on MPU6050 sensor data
String getMPU6050Bit (){
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX + 0.56; // GyroErrorX ~(-0.56)
  GyroY = GyroY - 2; // GyroErrorY ~(2)
  GyroZ = GyroZ + 0.79; // GyroErrorZ ~ (-0.8)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

  if (yaw < 0 || yaw > 180)
    yaw= 90;

int yaw_manipulated = 3000 + yaw;
    delay(150);
   
    return String(yaw_manipulated);
    //return String(yaw);
  }