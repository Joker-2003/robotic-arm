//master:

#define ledPin 13

String receivedString = "";  

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  Serial.begin(38400); 
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    receivedString += c;
    delay(5);
  }

  if (receivedString.length() > 0) {
      Serial.write(receivedString); 
    receivedString = ""; 
  }
}