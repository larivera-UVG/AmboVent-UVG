/* prueba_mov_manual
   Código para probar mover el motor siguiendo el movimiento
   de uno de los potenciómetros del pánel.
   Por: Luis Alberto Rivera
*/

#define BOARD  0  // 0 - Nano, 1 - Uno
#define MOTION_EN 1 // 0 - no se mueve el motor, 1 - sí se mueve

#define profile_length 250
#define cycleTime 10      // milisec
#define MAX_VEL_CENTRADO  23

#define pressure_sensor_available 0 // 1 - you have installed an I2C pressure sensor
#define DELTA_TELE_MONITOR 23 // Para que no se despliegue tantas veces, y no siempre
                              // se desplieguen los mismos índices.

#define KP 0.3      // motion control propportional gain 0.2
#define KI 4        // motion control integral gain 2
#define integral_limit 5    // limits the integral of error 

#define PWM_max 85
#define PWM_min (-PWM_max)
#define PWM_THR 20    // Umbral

#define invert_mot 0  // Cuidado con esto. Ver nota en el documento de Consideraciones

#include <EEPROM.h>

// Rango del feedback
// 0 <= min_arm_pos < max_arm_pos <= 1023
unsigned int min_arm_pos = 150, max_arm_pos = 600;  // 180, 900
unsigned int comp_pot_low = 0, comp_pot_high = 1023;  // 180, 900

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

int index = 0, manual_index = 0, vel_actual = 0;
int A_pot, prevA_pot, A_amplitude = 80, prev_A_amplitude, A_freq;
int motorPWM;
int breath_cycle_time;
int pos_from_pot;
int pressure_abs, pressure_baseline, pressure;
byte BPM, wanted_cycle_time = cycleTime, insp_pressure;
unsigned long lastIndex, lastTele, last_read_pres;
float wanted_manual_vel_PWM;
float wanted_pos, range, range_factor, integral, error, prev_error;

int direcc;

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
  adafruitPress.begin();
  pressure_baseline = adafruitPress.readPressure();
#endif

  // Para empezar en el índice adecuado, según la posición del motor
//  manual_index = map(pos_inicial, min_arm_pos, max_arm_pos, 0, profile_length-1);

  EEPROM.put(4, min_arm_pos);
  delay(200);
  EEPROM.put(8, max_arm_pos);
  delay(200);

}

void loop()
{
  read_IO();    // Read status of User Interface, act accordingly
  
  if(millis() - lastIndex >= wanted_cycle_time)  // wait for the cycle time
  {
    lastIndex = millis();  // last start of cycle time

    set_pos_vel_with_pot();

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
//  A_pot = 1023 - analogRead(pin_POT);
  A_pot = analogRead(pin_POT);

#if pressure_sensor_available == 1
  if(millis() - last_read_pres > 100)
  {
    last_read_pres = millis();

    pressure = int(adafruitPress.readPressure());
    pressure_abs = pressure - pressure_baseline;
    
    if(pressure_abs < 0)
      pressure_abs = 0;
  }
#endif
}

//---------------- CONTROL ----------------------------------------------
void set_pos_vel_with_pot()
{
  int pot_temp = analogRead(pin_AMP);
  pot_temp = constrain(pot_temp, comp_pot_low, comp_pot_high);

//  pos_from_pot = map(analogRead(pin_AMP), 0, 1023, min_arm_pos, max_arm_pos);
//  pos_from_pot = map(analogRead(pin_AMP), comp_pot_low, comp_pot_high, min_arm_pos, max_arm_pos);
  pos_from_pot = map(pot_temp, comp_pot_low, comp_pot_high, min_arm_pos, max_arm_pos);

  wanted_pos = float(pos_from_pot);

  prev_error = error;
  error = wanted_pos - 1*float(A_pot);

  integral = error*float(wanted_cycle_time)/1000;

  if(integral > integral_limit)
    integral = integral_limit;

  if(integral < -integral_limit)
    integral = -integral_limit;

  // PI correction 
  wanted_manual_vel_PWM = KP*error + KI*integral;

  // reduce speed for longer cycles
  wanted_manual_vel_PWM = wanted_manual_vel_PWM*float(cycleTime)/float(wanted_cycle_time);

  if(invert_mot)
    wanted_manual_vel_PWM = -wanted_manual_vel_PWM;

//  if(curr_sense)
//  {
//    if(A_current > max_allowed_current)
//      wanted_manual_vel_PWM = 0;
//  }

  if(wanted_manual_vel_PWM > PWM_max)
    wanted_manual_vel_PWM = PWM_max;  // limit PWM

  if(wanted_manual_vel_PWM < PWM_min)
    wanted_manual_vel_PWM = PWM_min;  // limit PWM

// Set PWM through the REV Robotics SPARK Motor driver. Values between 0 and 180 ---
//  motorPWM = PWM_mid + int(wanted_manual_vel_PWM);
//  motor.write(motorPWM);

// Set PWM through the VNH5019A-E driver. Values between 0 and 255 ---
  if(wanted_manual_vel_PWM < 0)
  {
    digitalWrite(pin_INA, LOW);
    digitalWrite(pin_INB, HIGH);
    wanted_manual_vel_PWM = -wanted_manual_vel_PWM;
    direcc = -1;
  }
  else
  {
    digitalWrite(pin_INA, HIGH);
    digitalWrite(pin_INB, LOW);
    direcc = 1;
  }

  motorPWM = (int)(wanted_manual_vel_PWM*255.0/PWM_max);  // set between 0 and 255

  if(motorPWM < PWM_THR)
    motorPWM = 0;
  
  analogWrite(pin_PWM, MOTION_EN*motorPWM);
}

void print_tele()
{
  Serial.print("Feedback: ");          Serial.print(A_pot);
  Serial.print(", pos_from_pot: ");    Serial.print(pos_from_pot);
  //Serial.print(", pos_from_pot: ");    Serial.print(analogRead(pin_AMP));
  
  Serial.print(", error: ");           Serial.print(error);
  Serial.print(", wanted_manual_vel_PWM: ");  Serial.print(wanted_manual_vel_PWM);
  Serial.print(", motorPWM: ");        Serial.print(motorPWM);
  Serial.print(", direccion: ");        Serial.println(direcc);

#if pressure_sensor_available == 1
  Serial.print("pressure baseline: ");  Serial.print(pressure_baseline);
  Serial.print(",   pressure: ");       Serial.print(pressure);
  Serial.print(",   pressure_abs: ");   Serial.println(pressure_abs);
#endif
  Serial.println("");
}
