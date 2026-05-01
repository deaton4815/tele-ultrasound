const int fsrPin = A0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  int rawValue = analogRead(fsrPin);
  float voltage = rawValue * (5.0 / 1023.0);

  Serial.print("Voltage:");
  Serial.print(voltage, 3); 
  Serial.println("V"); 

  delay(50);
}
