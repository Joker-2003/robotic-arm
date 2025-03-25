//slave:

String receivedString = "";  

void setup() {
  Serial.begin(38400); 
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    receivedString += c;
    delay(5); 
  }

  if (receivedString.length() > 0) {
      Serial.println(receivedString);
    receivedString = ""; 
  }
}