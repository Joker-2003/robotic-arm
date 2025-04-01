#include <SoftwareSerial.h>

SoftwareSerial BTSerial(0, 1);  // RX, TX
const int flexPin = A0; //connected to A0

void setup() {
    Serial.begin(9600);   // Serial Monitor
    BTSerial.begin(9600); // Bluetooth Module

    Serial.println("Bluetooth HC-05 Ready!");
}

void loop() {
    // Read flex sensor value
    // int flexValue = analogRead(flexPin);

   
    // if (flexValue > 300) {
    //     BTSerial.println("4120");  
    //     Serial.println("Sent over BT: 4120");
    // } else {
    //     BTSerial.println("4070");  
    //     Serial.println("Sent over BT: 4070");
    // }

    // // Print flex sensor value for debugging
    // Serial.print("Flex Sensor Value: ");
    // Serial.println(flexValue);

    // delay(1000); 

   
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        BTSerial.println(command);  // Send command to HC-05
        Serial.print("Sent: ");
        Serial.println(command);
    }

  
    while (BTSerial.available()) {
        String response = BTSerial.readString();
        Serial.print("Received: ");
        Serial.println(response);
    }
}