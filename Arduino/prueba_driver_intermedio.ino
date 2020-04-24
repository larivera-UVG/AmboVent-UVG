/* prueba_driver
   Código para probar el driver VNH5019A-E para el ventilador AmboVent-UVG
   También se prueban algunas de las entradas analógicas y se despliegan
   resultados básicos.
   Según el valor de un potenciómetro, se ajusta el tiempo del ciclo del
   ventilador.
   Por: Luis Alberto Rivera y Miguel Zea
*/

// Rango del feedback: 410 - 826

#define BOARD 0   // 0 - Nano, 1 - Uno

#define profile_length 500
#define cycleTime 5      // milisec
#define smear_factor 0   // 0 to do all cycle in 2.5 seconds and wait for the rest
                         // 1 to "smear" the motion profile on the whole cycle time
#define DELTA_TELE_MONITOR (20*cycleTime)

#define perc_of_lower_volume_display 40

unsigned int max_arm_pos = 800, min_arm_pos = 400;

#if BOARD == 0 // Arduino Nano y Nano chino
const int pin_PWM =  3;
const int pin_INA = 12; // 12
const int pin_INB = 11; // 11
const int pin_POT = A0; // para el feedback del motor
const int pin_AMP = A1; // para el amplitude potentiometer control
const int pin_FRQ = A2; // para el amplitude frequency control
const int pin_PRE = A3; // para el pressure control
#endif

#if BOARD == 1  // Arduino Uno
const int pin_INA = PD2;
const int pin_PWM = PD3;
const int pin_INB = PD4;
const int pin_POT = A0; // para el feedback del motor
const int pin_AMP = A1; // para el amplitude potentiometer control
const int pin_FRQ = A2; // para el amplitude frequency control
const int pin_PRE = A3; // para el pressure control
#endif

int index = 0, vel_actual = 0;
int A_pot, prevA_pot, A_amplitude = 80, prev_A_amplitude, A_freq;
//int A_current; // TODAVÍA NO SÉ CÓMO VA ÉSTE...
int motorPWM;
int breath_cycle_time;
byte BPM, wanted_cycle_time = cycleTime, insp_pressure;
unsigned long lastIndex, lastTele;

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
  Serial.begin(115200);

  int pos_inicial = analogRead(pin_POT);
  if(pos_inicial > max_arm_pos)
    pos_inicial = max_arm_pos;
  if(pos_inicial < min_arm_pos)
    pos_inicial = min_arm_pos;

  // Para empezar en el índice adecuado, según la posición del motor
  index = map(pos_inicial, min_arm_pos, max_arm_pos, 0, profile_length);
}

void loop()
{
  read_IO();    // Read status of User Interface, act accordingly
  
  if(millis() - lastIndex >= wanted_cycle_time)  // wait for the cycle time
  {
    lastIndex = millis();  // last start of cycle time

    vel_actual = vel[index] - 128;

    if(vel_actual < 0)
    {
      digitalWrite(pin_INA, LOW);
      digitalWrite(pin_INB, HIGH);
      vel_actual = -vel_actual;
    }
    else
    {
      digitalWrite(pin_INA, HIGH);
      digitalWrite(pin_INB, LOW);
    }

    motorPWM = (int)(vel_actual*255.0/25.0);  // 255, el 25 es el máximo valor absoluto del vector vel centrado.
    analogWrite(pin_PWM, motorPWM);

    index = (index + 1)%profile_length;

    if(millis() - lastTele >= DELTA_TELE_MONITOR)  // esperar para mostrar telemetry
    {
      lastTele = millis();  // last time telemetry was displayed
      print_tele();
    }
  }
}

void read_IO()
{
  // Pot conectado al motor
  A_pot = analogRead(pin_POT);

  // Pot para Amplitud relacionada al volumen?
  A_amplitude = perc_of_lower_volume_display +
                int(float(analogRead(pin_AMP))*(100.0 - perc_of_lower_volume_display)/1023.0);
  if(A_amplitude > 100)
    A_amplitude = 100;
  if(A_amplitude < perc_of_lower_volume_display)
    A_amplitude = perc_of_lower_volume_display;


  // Pot relacionado con algo de presión (no el sensor)
  //if(pres_pot_available)
    insp_pressure = 10 + analogRead(pin_PRE)/12;  // NO CLARA LA ECUACIÓN

  if(insp_pressure < 30)
    insp_pressure = 30;

  if(insp_pressure > 70)
    insp_pressure = 70;

  // Pot relacionado con la frecuencia del ciclo
  A_freq = analogRead(pin_FRQ);
  BPM = (byte)(6 + (A_freq - 23)/55);  // BPM es tipo byte, es mejor hacer type casting
  breath_cycle_time = (int)(60000/BPM);  // es tipo int, es mejor hacer type casting
  wanted_cycle_time = (byte)(cycleTime + int(float(breath_cycle_time - profile_length*cycleTime)*float(smear_factor)/profile_length));
}

void print_tele()
{
  Serial.print("A_freq: ");             Serial.println(A_freq);
  Serial.print("Feedback: ");           Serial.println(A_pot);
  Serial.print("A_amplitude: ");        Serial.println(A_amplitude);
  Serial.print("insp_pressure: ");      Serial.println(insp_pressure);
  Serial.print("wanted_cycle_time: ");  Serial.println(wanted_cycle_time);
    
  Serial.println("");
  Serial.println("");
}