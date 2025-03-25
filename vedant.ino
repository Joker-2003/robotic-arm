//Constants:
const int ledPin = 3;  
const int flexPin = A0; 

//Variables:
int value; //save analog value
int kbit = 0;

void setup(){

  pinMode(ledPin, OUTPUT);  //Set pin 3 as 'output'
  Serial.begin(9600);       //Begin serial communication
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

}

void loop(){

  int grip = analogRead(flexPin);  // Read and save analog value from potentiometer
  if(grip>=950)
  kbit  = 2100;
  else
  kbit  = 2180;
  // ashley please send kbit here

  int wrist  = analogRead(A1);
  Serial.println(wrist);
  kbit = 1000 + (wrist/1023)180;
  // ashley please send kbit here

}