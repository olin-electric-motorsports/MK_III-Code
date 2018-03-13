void setup() {
  // put your setup code here, to run once:
  pinMode(5, OUTPUT);
  analogWrite(5, 20);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(5, random(1, 255));
  delay(100);
}
