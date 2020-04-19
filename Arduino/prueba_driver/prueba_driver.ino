/* prueba_driver
   Código para probar el driver VNH5019A-E para el ventilador AmboVent-UVG
   También se prueban algunas de las entradas analógicas y se despliegan
   resultados básicos.
   Según el valor de un potenciómetro, se ajusta el tiempo del ciclo del
   ventilador.
   Por: Luis Alberto Rivera y Miguel Zea
*/

#define BOARD  0  // 0 - Nano, 1 - Uno
#define PERFIL 1  // 0 - el original, 1 - el más reciente

#if PERFIL == 0
  #define profile_length 500
  #define cycleTime 5      // milisec
  #define MAX_VEL_CENTRADO  25
#endif

#if PERFIL == 1
  #define profile_length 250
  #define cycleTime 10      // milisec
  #define MAX_VEL_CENTRADO  23
#endif


#define pressure_sensor_available 1 // 1 - you have installed an I2C pressure sensor
#define smear_factor 0   // 0 to do all cycle in 2.5 seconds and wait for the rest
                         // 1 to "smear" the motion profile on the whole cycle time
#define DELTA_TELE_MONITOR 23 // Para que no se despliegue tantas veces, y no siempre
                              // se desplieguen los mismos índices.

#define perc_of_lower_volume_display 40


////#include <EEPROM.h>
#include <Wire.h>    // Used for I2C
#include "Adafruit_MPRLS.h"
#include <LiquidCrystal_I2C.h>
////#include "ArduinoUniqueID.h"

// Rango del feedback
// 0 <= min_arm_pos < max_arm_pos <= 1023
unsigned int min_arm_pos = 200, max_arm_pos = 800;

#if pressure_sensor_available == 1
//MS5803 sparkfumPress(ADDRESS_HIGH);
Adafruit_MPRLS adafruitPress(-1, -1);  // valores por defecto
#endif

#if BOARD == 0 // Arduino Nano y Nano chino
  #define pin_PWM  3   // digital pin that sends the PWM to the motor
  #define pin_INA 12   // Para el driver
  #define pin_INB 11   // Para el driver
  #define pin_POT A0   // analog pin of motion feedback potentiometer
  #define pin_AMP A1   // analog pin of amplitude potentiometer control
  #define pin_FRQ A2   // analog pin of rate potentiometer control
  #define pin_PRE A3   // analog pin of pressure potentiometer control
#endif

#if BOARD == 1  // Arduino Uno
  #define pin_PWM PD3    // digital pin that sends the PWM to the motor
  #define pin_INA PD2    // Para el driver
  #define pin_INB PD4    // Para el driver
  #define pin_POT A0   // analog pin of motion feedback potentiometer
  #define pin_AMP A1   // analog pin of amplitude potentiometer control
  #define pin_FRQ A2   // analog pin of rate potentiometer control
  #define pin_PRE A3   // analog pin of pressure potentiometer control
#endif

int index = 0, vel_actual = 0;
int A_pot, prevA_pot, A_amplitude = 80, prev_A_amplitude, A_freq;
//int A_current; // TODAVÍA NO SÉ CÓMO VA ÉSTE...
int motorPWM;
int breath_cycle_time;
int pressure_abs, pressure_baseline, pressure;
byte BPM, wanted_cycle_time = cycleTime, insp_pressure;
unsigned long lastIndex, lastTele, last_read_pres;

#if PERFIL == 0
const PROGMEM byte vel[profile_length] = 
//byte vel[profile_length] = 
    {129,129,130,130,131,131,132,133,133,134,135,135,136,136,137,138,138,138,139,140,
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
#endif

#if PERFIL == 1
const PROGMEM byte vel[profile_length] =
//byte vel[profile_length] = 
    {129,132,134,136,137,139,140,141,142,143,143,144,144,145,146,146,146,147,147,147,
     148,148,148,148,149,149,149,149,149,149,150,150,150,150,150,150,150,150,150,150,
     150,150,150,150,150,149,149,149,149,149,149,148,148,148,148,147,147,147,146,146,
     146,145,144,144,143,143,142,141,140,139,137,136,134,132,129,128,128,128,128,128,
     128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,
     128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,
     128,128,128,128,128,127,125,123,121,120,119,117,116,115,114,113,112,111,111,110,
     109,109,108,108,107,107,106,106,106,106,105,105,105,105,105,105,105,105,105,105,
     105,105,105,105,105,105,106,106,106,107,107,107,108,108,109,109,110,110,111,111,
     112,113,113,114,115,116,117,118,118,119,119,120,120,120,121,121,121,122,122,122,
     123,123,123,124,124,124,124,125,125,125,125,125,126,126,126,126,126,127,127,127,
     127,127,127,127,128,128,128,128,128,128,128,128,128,128,128,128,129,129,129,129,
     129,129,129,129,129,128,128,128,128,128};
#endif

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

#if pressure_sensor_available == 1
//  sparkfumPress.reset();
//  sparkfumPress.begin();
//  pressure_baseline = sparkfumPress.getPressure(ADC_4096);
  adafruitPress.begin();
  pressure_baseline = adafruitPress.readPressure();
#endif

  // Para empezar en el índice adecuado, según la posición del motor
  index = map(pos_inicial, min_arm_pos, max_arm_pos, 0, profile_length);
}

void loop()
{
  read_IO();    // Read status of User Interface, act accordingly
  
  if(millis() - lastIndex >= wanted_cycle_time)  // wait for the cycle time
  {
    lastIndex = millis();  // last start of cycle time

    vel_actual = pgm_read_byte_near(vel + index) - 128;
//    vel_actual = vel[index] - 128;

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

    motorPWM = (int)(vel_actual*255.0/MAX_VEL_CENTRADO);
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
  BPM = (byte)(6 + (A_freq - 23)/55);  // BPM es tipo byte, sería mejor hacer type casting
  breath_cycle_time = (int)(60000/BPM);  // es tipo int,  sería mejor hacer type casting
  wanted_cycle_time = (byte)(cycleTime + 
      int(float(breath_cycle_time - profile_length*cycleTime)*float(smear_factor)/profile_length));


#if pressure_sensor_available == 1
  if(millis() - last_read_pres > 100)
  {
    last_read_pres = millis();

    //pressure_abs = int(sparkfumPress.getPressure(ADC_4096)-pressure_baseline);   // mbar
    pressure = int(adafruitPress.readPressure());
    pressure_abs = pressure - pressure_baseline;
    
    if(pressure_abs < 0)
      pressure_abs = 0;
  }
#endif
}

void print_tele()
{
  Serial.print("A_freq: ");             Serial.println(A_freq);
  Serial.print("Feedback: ");           Serial.println(A_pot);
  Serial.print("A_amplitude: ");        Serial.println(A_amplitude);
  Serial.print("insp_pressure: ");      Serial.println(insp_pressure);
  Serial.print("wanted_cycle_time: ");  Serial.println(wanted_cycle_time);
  Serial.print("Index: ");              Serial.println(index);
#if pressure_sensor_available == 1
  Serial.print("pressure baseline: ");  Serial.print(pressure_baseline);
  Serial.print(",   pressure: ");       Serial.print(pressure);
  Serial.print(",   pressure_abs: ");   Serial.println(pressure_abs);
#endif
  Serial.println("");
}
