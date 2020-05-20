#define pin_POT A0   // analog pin of motion feedback potentiometer

int x;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);

  while(analogRead(pin_POT) > 512)
  {
    delay(50);
  }
}

void loop()
{
  if(analogRead(pin_POT) <= 512)
  {
    x = 1;
    Serial.println(x++);
    Serial.println(x++);
    Serial.println(x++);
    Serial.println(analogRead(pin_POT));
  }

  delay(1000);
}
