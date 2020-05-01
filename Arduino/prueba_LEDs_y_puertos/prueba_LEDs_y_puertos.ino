#define INVERT_LEDS 1

#define pin_LED_USR 7    // User LED
#define pin_LED_FAIL 8   // FAIL and calib: red LED
#define pin_LED_FREQ 9   // frequency: green LED
#define pin_LED_AMP 10   // amplitude: blue LED

void setup() {
  pinMode(pin_LED_USR, OUTPUT);
  pinMode(pin_LED_AMP, OUTPUT);
  pinMode(pin_LED_FREQ, OUTPUT);
  pinMode(pin_LED_FAIL, OUTPUT);

  Serial.begin(115200);
}

void loop() {
  Serial.println("LED_USR ON");
  LED_USR(1); LED_AMP(0); LED_FREQ(0); LED_FAIL(0);
  delay(2000);

  Serial.println("LED_AMP ON");
  LED_USR(0); LED_AMP(1); LED_FREQ(0); LED_FAIL(0);
  delay(2000);

  Serial.println("LED_FREQ ON");
  LED_USR(0); LED_AMP(0); LED_FREQ(1); LED_FAIL(0);
  delay(2000);

  Serial.println("LED_FAIL ON");
  LED_USR(0); LED_AMP(0); LED_FREQ(0); LED_FAIL(1);
  delay(2000);
}


void LED_FREQ(byte val)
{
  if(INVERT_LEDS)
    digitalWrite(pin_LED_FREQ, 1 - val);
  else
    digitalWrite(pin_LED_FREQ, val);
}

void LED_AMP(byte val)
{
  if(INVERT_LEDS)
    digitalWrite(pin_LED_AMP, 1 - val);
  else
    digitalWrite(pin_LED_AMP, val);
}

void LED_FAIL(byte val)
{
  if(INVERT_LEDS)
    digitalWrite(pin_LED_FAIL, 1 - val);
  else
    digitalWrite(pin_LED_FAIL, val);
}

void LED_USR(byte val)
{
  if(INVERT_LEDS)
    digitalWrite(pin_LED_USR, 1 - val);
  else
    digitalWrite(pin_LED_USR, val);
}
