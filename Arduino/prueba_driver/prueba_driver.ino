/* prueba_driver
   Código para probar el driver VNH5019A-E para el ventilador AmboVent-UVG
   Por: Luis Alberto Rivera y Miguel Zea
*/

#define profile_length 500
#define cycleTime 5        // milisec
#define smear_factor 1     // 0 to do all cycle in 2.5 seconds and wait for the rest
                           // 1 to "smear" the motion profile on the whole cycle time 

const int pin_INA = PD2;
const int pin_INB = PD4;
const int pin_PWM = PD3;
const int pin_FRQ = A3;

int index = 0, vel_actual = 0;
int motorPWM, A_freq;
int breath_cycle_time;
byte BPM, wanted_cycle_time = cycleTime;
unsigned long lastIndex;

byte vel[profile_length] = {129,129,130,130,131,131,132,133,133,134,135,135,136,136,137,138,138,138,139,140,
                            140,141,141,141,142,142,143,143,144,144,145,145,145,146,146,146,147,147,147,148,
                            148,148,149,149,149,150,150,150,150,151,151,151,151,151,152,152,152,152,152,152,
                            153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,
                            153,153,153,153,153,153,153,153,153,153,152,152,152,152,152,152,151,151,151,151,
                            151,150,150,150,150,149,149,149,148,148,148,147,147,147,146,146,146,145,145,145,
                            144,144,143,143,142,142,141,141,141,140,140,139,138,138,138,137,136,136,135,135,
                            134,133,133,132,131,131,130,130,129,129,128,128,128,128,128,128,128,128,128,128,
                            128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,
                            128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,
                            128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,
                            128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,
                            128,128,128,128,128,128,128,128,128,128,128,127,127,126,126,126,125,125,124,124,
                            123,123,122,122,121,121,120,120,120,119,118,118,118,117,117,116,116,115,115,115,
                            114,114,113,113,113,112,112,111,111,111,110,110,109,109,109,108,108,108,107,107,
                            107,107,106,106,106,105,105,105,105,105,104,104,104,104,104,104,103,103,103,103,
                            103,103,103,103,103,103,103,103,103,103,103,103,103,103,103,103,104,104,104,104,
                            104,105,105,105,105,105,106,106,106,107,107,107,108,108,108,109,109,109,110,110,
                            111,111,112,112,113,113,114,114,114,115,116,116,116,117,117,118,118,118,118,118,
                            119,119,119,119,119,119,119,120,120,120,120,120,120,120,121,121,121,121,121,121,
                            121,122,122,122,122,122,122,122,123,123,123,123,123,123,123,124,124,124,124,124,
                            124,124,124,124,125,125,125,125,125,125,125,125,125,126,126,126,126,126,126,126,
                            126,126,126,126,126,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,
                            127,127,127,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,
                            128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128};


void setup()
{
  // put your setup code here, to run once:
  pinMode(pin_INA, OUTPUT);
  pinMode(pin_INB, OUTPUT);
  pinMode(pin_PWM, OUTPUT);
}

void loop()
{
  if(millis() - lastIndex >= wanted_cycle_time)  // wait for the cycle time
  {
    lastIndex = millis();  // last start of cycle time

    vel_actual = vel[index] - 128;

    if(vel_actual < 0)
    {
      digitalWrite(pin_INA, HIGH);
      digitalWrite(pin_INB, LOW);
      vel_actual = -vel_actual;
    }
    else
    {
      digitalWrite(pin_INA, LOW);
      digitalWrite(pin_INB, HIGH);
    }

    motorPWM = (int)(vel_actual*255.0/25.0);  // 255, el 25 es el máximo valor absoluto del vector vel centrado.
    analogWrite(pin_PWM, motorPWM);

    index = (index + 1)%500;
  }

  A_freq = analogRead(pin_FRQ);
  BPM = (byte)(6 + (A_freq - 23)/55);  // BPM es tipo byte, sería mejor hacer type casting
  breath_cycle_time = (int)(60000/BPM);  // es tipo int,  sería mejor hacer type casting
  wanted_cycle_time = (byte)(cycleTime + int(float(breath_cycle_time - 500*cycleTime)*float(smear_factor)/500));
}
