#include <SoftwareSerial.h>
#include <Wire.h>

const int flex1 = A0; // thumb sensor
const int flex2 = A1; // index sensor
const int button1 = 2;

int selectedMotor = 7;

SoftwareSerial BTSerial(0, 1);  // RX, TX

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

void setup() {
    Serial.begin(9600);   // Serial Monitor
    BTSerial.begin(9600); // Bluetooth Module
    pinMode(flex1, INPUT);
    pinMode(flex2, INPUT);
    pinMode(button1, INPUT);
    // Serial.println("Bluetooth HC-05 Ready!");
}

void loop() {
    // if (digitalRead(button1) == HIGH) {
    //     selectedMotor = 1;
        // Serial.println("Motor 1 selected");
    // } else if (digitalRead(button2) == HIGH) {
    //     selectedMotor = 2;
    //     Serial.println("Motor 2 selected");
    // } else if (digitalRead(button3) == HIGH) {
    //     selectedMotor = 3;
    //     Serial.println("Motor 3 selected");
    // }
    
    if (digitalRead(button1) == HIGH) {
        int flexValue1 = analogRead(flex1);
        int flexValue2 = analogRead(flex2);
        int totalFlexValue = flexValue1 + flexValue2;

        // Serial.print("Total Flex Value: ");
        // Serial.println(totalFlexValue);
        
        // map(value, fromLow, fromHigh, toLow, toHigh);
        // need to tweak thresholds
        int angle = map(totalFlexValue, 700, 1800, 0, 180); //assuming min of 700 and max 900 analog value for sum of two flex sensors
        angle = constrain(angle, 0, 180);
        
        int dataToSend = (selectedMotor * 1000) + angle; // want 4 digits
       
       
        String command = String(dataToSend);
        BTSerial.println(command);
        // Serial.print("Sent: ");
        // Serial.println(command);
        
      while (BTSerial.available()) {
        String response = BTSerial.readString();
      }
    }

      if (Serial.available()) {
          String command = Serial.readStringUntil('\n');
          BTSerial.println(command);  // Send command to HC-05
      }
    
    delay(1000);
}