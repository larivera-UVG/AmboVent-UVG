/* prueba_mov_manual
   Código para probar mover el motor siguiendo el movimiento
   de uno de los potenciómetros del pánel.
   Por: Luis Alberto Rivera
*/

#define DEBUG 0     // 1 para que forzar a |error| < ERROR_DEBUG
#define ERROR_DEBUG 20

// system configuration
#define full_configuration 1        // 1 is the default - full system.   0 is for partial system - potentiometer installed on pulley, no potentiometers, ...
#define pressure_sensor_available 0 // 1 - you have installed an I2C pressure sensor 
#define central_monitor_system 0    // 1 - send unique ID for 10 seconds upon startup, 0 - dont

// options for display and debug via serial com
#define telemetry 1           // 1 = send telemtry for debug
#define DELTA_TELE_MONITOR 23  // Delta time (in ms) for displaying telemetry and info to monitor

// UI
#define pot_alpha 0.85  // filter the pot values

// clinical
#define perc_of_lower_volume 50.0       // % of max press - defines lower volume  // 50.0
#define perc_of_lower_vol_display 33.0  // % of max press - defines lower volume to display when reaching the real lower volume // 33.0
#define wait_time_after_resistance 3    // seconds to wait before re-attempt to push air after max pressure was achieved 
#define max_pres_disconnected 10        // if the max pressure during breathing cycle does not reach this value - pipe is disconnected
#define insp_pressure_default 40        // defualt value - hold this pressure while breathing - the value is changed if INSP_Pressure potentiometer is inatalled 
#define safety_pres_above_insp 10       // defines safety pressure as the inspirium pressure + this one
#define safety_pressure 70              // quickly pullback arm when reaching this pressure in cm H2O
#define speed_multiplier_reverse 2      // factor of speeed for releasing the pressure (runs motion in reverse at X this speed
#define motion_time_default 35          // motion time in 100 mSec 35 = 3500 mSec
#define patient_triggered_breath_def 1  // 1 = trigger new breath in case of patient inhale during the PEEP plateu 
#define delta_pres_patient_inhale 5     // in cmH2O
#define alpha_pres 0.98                 // used to average the pressure during the PEEP plateu

#if (full_configuration == 1) // Direct arm conection 
  #define LCD_available 0
  #define pres_pot_available 1  // 1 if the system has 3 potentiometer and can control the inspirium pressure 
  #define pin_TST 4         // test mode - not in use
  #define pin_SW2 5         // breath - On / Off / cal
  #define pin_RST 6         // reset alarm - not in use
  #define pin_USR 7         // User LED
  #define pin_LED_AMP 8    // amplitude LED
  #define pin_LED_FREQ 9   // frequency LED
  #define pin_LED_Fail 10   // FAIL and calib blue LED
  #define pin_FD 13    // freq Down
  #define pin_FU 13    // freq Up
  #define pin_AD 13    // Amp Down
  #define pin_AU 13    // Amp Up
  #define curr_sense 0
  #define control_with_pot 1  // 1 = control with potentiometers  0 = with push buttons

  #define FF_MIN 0.1
  #define FF_MAX 5
  #define FF_DEF 0.6              // motion control feed forward. 0.6,  4.5
  #define DELTA_FF ((FF_MAX-FF_MIN)/100.0)

  #define KP_MIN 0.05
  #define KP_MAX 4
  #define KP_DEF 0.2              // motion control propportional gain 0.2, 1.2
  #define DELTA_KP ((KP_MAX-KP_MIN)/100.0)

  #define KI_MIN 0.1
  #define KI_MAX 7
  #define KI_DEF 2                // motion control integral gain 2, 7
  #define DELTA_KI ((KI_MAX-KI_MIN)/100.0)

  #define integral_limit 5    // limits the integral of error 
  #define f_reduction_up_val 0.85    // reduce feedforward by this factor when moving up 
#endif

// Other Arduino pins alocation

// Pins for Motor Driver
#define pin_PWM  3    // digital pin that sends the PWM to the motor
#define pin_INA 12    // Para el driver
#define pin_INB 11    // Para el driver

#define pin_POT A0   // analog pin of motion feedback potentiometer
#define pin_AMP A1   // analog pin of amplitude potentiometer control
#define pin_FRQ A2   // analog pin of rate potentiometer control
#define pin_PRE A3   // analog pin of pressure potentiometer control
//#define pin_CUR 6    // analog pin of current sense

// Talon SR or SPARK controller PWM settings ("angle" for Servo library)
#define PWM_mid 93  // mid value for PWM 0 motion - higher pushes up
#define PWM_max 85
#define PWM_min (-PWM_max)
#define max_allowed_current 100 // 100 <=> 10 Amps

// motion control parameters
#define cycleTime 10          // milisec
#define alpha 0.95            // filter for current apatation - higher = stronger low pass filter
#define profile_length 250    // motion control profile length
#define motion_control_allowed_error  30  // % of range

// motor and sensor definitions
#define invert_mot 1  // Cuidado con esto. Ver notas en
#define invert_pot 0  // el archivo de consideraciones.


#include <EEPROM.h>
//#include <Servo.h> 
#include <Wire.h>    // Used for I2C
//#include <SparkFun_MS5803_I2C.h>
#include "Adafruit_MPRLS.h"
#include <LiquidCrystal_I2C.h>
#include "ArduinoUniqueID.h"

//Servo motor;  //TODO: define a constant to select the driver

#if pressure_sensor_available == 1
//MS5803 sparkfumPress(ADDRESS_HIGH);
Adafruit_MPRLS adafruitPress(-1, -1);  // valores por defecto
#endif

#if LCD_available == 1
LiquidCrystal_I2C lcd(0x27, 16, 2); // Set the LCD address to 0x27 for a 16 chars and 2 line display
#endif

// Motion profile parameters 
// pos byte 0...255  units: promiles of full range
// vel int 0...255  ZERO is at 128 , units: pos change per 0.2 sec
// profile data:  press 125 points (50%) relase 125
const PROGMEM byte pos[profile_length] =
    {  0,  0,  1,  2,  4,  6,  8, 10, 13, 15, 18, 21, 25, 28, 31, 35, 38, 42, 46, 50,
      54, 57, 61, 66, 70, 74, 78, 82, 86, 91, 95, 99,104,108,112,117,121,125,130,134,
     138,143,147,151,156,160,164,169,173,177,181,185,189,194,198,201,205,209,213,217,
     220,224,227,230,234,237,240,242,245,247,249,251,253,254,255,255,255,255,255,255,
     255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
     255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
     255,255,255,255,255,255,255,254,253,252,250,248,246,244,241,238,235,232,229,225,
     222,218,214,210,206,202,198,193,189,184,180,175,171,166,162,157,152,148,143,138,
     134,129,124,120,115,111,106,102, 97, 93, 89, 84, 80, 76, 72, 68, 64, 61, 57, 54,
      50, 47, 44, 41, 38, 36, 33, 31, 29, 27, 25, 23, 22, 20, 19, 17, 16, 15, 13, 12,
      11, 10,  9,  8,  7,  6,  6,  5,  4,  3,  3,  2,  2,  1,  1,  1,  0,  0,  0,  1,
       1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,  2,  2,  1,  1,  1,  1,  1,  1,
       1,  1,  1,  0,  0,  0,  0,  0,  0,  0};

const PROGMEM byte vel[profile_length] =
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

byte FD, FU, AD, AU, prev_FD, prev_FU, prev_AD, prev_AU, SW2, prev_SW2, prev_TST,
     RST, LED_status, USR_status, blueOn, calibrated = 0, calibON, numBlinkFreq,
     SW2_pressed, TST_pressed, menu_state;
byte monitor_index = 0, BPM = 14, prev_BPM, in_wait, failure, send_beep,
     wanted_cycle_time, disconnected = 0, high_pressure_detected = 0,
     motion_failure = 0, sent_LCD, hold_breath, safety_pressure_detected;
byte counter_ON, counter_OFF, SW2temp, insp_pressure, prev_insp_pressure,
     safety_pressure_counter, no_fail_counter, TST, counter_TST_OFF, counter_TST_ON,
     TSTtemp, patient_triggered_breath, motion_time, progress,
     telemetry_option = 0, PID_pots_aligned;

int A_pot, prev_A_pot, A_current, Compression_perc = 80, prev_Compression_perc,
    A_rate, A_comp, A_pres;
int motorPWM, index = 0, prev_index, i, wait_cycles, cycle_number, cycles_lost,
    index_last_motion;
int pressure_abs, breath_cycle_time, max_pressure = 0, prev_max_pressure = 0,
    min_pressure = 100, prev_min_pressure = 0, index_to_hold_breath, pressure_baseline;
int comp_pot_low = 0, comp_pot_high = 1023, rate_pot_low = 0, rate_pot_high = 1023,
    pres_pot_low = 0, pres_pot_high = 1023;

unsigned int max_arm_pos, min_arm_pos;
unsigned long lastSent, lastIndex, lastUSRblink, last_TST_not_pressed, lastBlue,
              start_wait, last_sent_data, last_read_pres, start_disp_pres;

float pot_rate, pot_pres, pot_comp, avg_pres;
float wanted_pos, wanted_vel_PWM, range, range_factor, profile_planned_vel,
      planned_vel, integral, error, prev_error, f_reduction_up;

float FF = FF_DEF, KP = KP_DEF, KI = KI_DEF, FF_temp, KP_temp, KI_temp;

enum main_states:byte {STBY_STATE, BREATH_STATE, MENU_STATE};
enum main_states state;


void setup()
{
  pinMode(pin_PWM, OUTPUT);
  pinMode(pin_INA, OUTPUT);  // For driver VNH5019A-E
  pinMode(pin_INB, OUTPUT);  // For driver VNH5019A-E

  pinMode(pin_FD, INPUT_PULLUP);
  pinMode(pin_FU, INPUT_PULLUP);
  pinMode(pin_AD, INPUT_PULLUP);
  pinMode(pin_AU, INPUT_PULLUP);
  pinMode(pin_SW2, INPUT_PULLUP);
  pinMode(pin_TST, INPUT_PULLUP);
  pinMode(pin_LED_AMP, OUTPUT);
  pinMode(pin_LED_FREQ, OUTPUT);
  pinMode(pin_LED_Fail, OUTPUT);
  pinMode(pin_USR, OUTPUT);

//  motor.attach(pin_PWM);
  Serial.begin(115200);
  Wire.begin();

#if pressure_sensor_available == 1
//  sparkfumPress.reset();
//  sparkfumPress.begin();
//  pressure_baseline = sparkfumPress.getPressure(ADC_4096);
  adafruitPress.begin();
  pressure_baseline = adafruitPress.readPressure();
#endif

#if LCD_available == 1
  lcd.begin();      // initialize the LCD
  lcd.backlight();  // Turn on the backlight and print a message.
  lcd.setCursor(0, 0);  lcd.print("AmboVent-UVG   ");
  lcd.setCursor(0, 1);  lcd.print("1690.108       ");
#endif

#if central_monitor_system==1
  for (i = 0; i < 100; i++) 
  {
    UniqueIDdump(Serial);  
    delay(100); 
  }  // for IAI monitor run for 100 cycles
#endif

  state = STBY_STATE;
  EEPROM.get(4, min_arm_pos);     delay(20);
  EEPROM.get(8, max_arm_pos);     delay(20);
  EEPROM.get(12, comp_pot_low);   delay(20);
  EEPROM.get(16, comp_pot_high);  delay(20);
  EEPROM.get(20, rate_pot_low);   delay(20);
  EEPROM.get(24, rate_pot_high);  delay(20);
  EEPROM.get(28, pres_pot_low);   delay(20);
  EEPROM.get(32, pres_pot_high);  delay(20);

  if(min_arm_pos >= 0 && max_arm_pos < 1024 && min_arm_pos < max_arm_pos)
    calibrated = 1;
    
//  EEPROM.put(36, float(FF_DEF));                    delay(200);
//  EEPROM.put(36 + sizeof(float), float(KP_DEF));    delay(200);
//  EEPROM.put(36 + 2*sizeof(float), float(KI_DEF));  delay(200);

  EEPROM.get(36, FF);                    delay(20);
  EEPROM.get(36 + sizeof(float), KP);    delay(20);
  EEPROM.get(36 + 2*sizeof(float), KI);  delay(20);

  if(FF < FF_MIN || FF > FF_MAX)
    FF = FF_DEF;

  if(KP < KP_MIN || KP > KP_MAX)
    KP = KP_DEF;

  if(KI < KI_MIN || KI > KI_MAX)
    KI = KI_DEF;

  insp_pressure = insp_pressure_default;
  patient_triggered_breath = patient_triggered_breath_def;
  motion_time = motion_time_default;

#if LCD_available == 1
  lcd.backlight();  // Turn on the backlight and print a message.
#endif

#if DEBUG == 1   
  randomSeed(analogRead(0));
#endif
}

void loop()
{
  read_IO();    // Read status of User Interface

  switch(state)
  {
    case STBY_STATE:     // standby
      standby_func();

      if(SW2_pressed && calibrated == 1)  // start breathing motion
      { 
        state = BREATH_STATE;
        initialize_breath();

        PID_pots_aligned = 0;
      }

      break;

    case BREATH_STATE:     // run profile

// Lo siguiente iría dentro de la opción de calibrar PID, en el display_menu --
      if(PID_pots_aligned == 0)
      {
        telemetry_option = 2;
        // Primero colocar los pots en la posición correspondiente a las
        // constantes actuales, para que no haya cambios bruscos.
        align_PID_pots();
        PID_pots_aligned = 1;

        telemetry_option = 0;
      }
      else
      {
        set_PID_constants();
        run_profile_func();
      }

      if(SW2_pressed)
      {
        state = STBY_STATE;  // stop breathing motion

        // Mejor guardar las constantes del PID en la EEPROM
        EEPROM.put(36, FF);                    delay(200);
        EEPROM.put(36 + sizeof(float), KP);    delay(200);
        EEPROM.put(36 + 2*sizeof(float), KI);  delay(200);
      }
// ----------------------------------------------------------------------------
      break;
  }
  
  if(millis() - last_sent_data > DELTA_TELE_MONITOR)
  { 
    if(telemetry == 1)
      print_tele();

    last_sent_data = millis();
  }
}

// Controls / performs the sequence of steps for the motor to move, ultimately
// setting the corresponding PWM.
void run_profile_func()
{
  if(millis() - lastIndex >= wanted_cycle_time) // do when cycle time was reached
  {
    cycles_lost = (millis() - lastIndex)/wanted_cycle_time - 1;
    cycles_lost = constrain(cycles_lost,0,15);

    lastIndex = millis();  // last start of cycle time
    calculate_wanted_pos_vel();
    
    if(100*abs(error)/(max_arm_pos - min_arm_pos) >
                              motion_control_allowed_error && cycle_number > 1)
      motion_failure = 1;

    if(safety_pressure_detected)
      index -= speed_multiplier_reverse*(1+cycles_lost);  // run in reverse if high pressure was detected

    if(index < 0)
    {
      if(safety_pressure_detected == 1)
        safety_pressure_counter += 1;  // count the number of cases reaching safety pressure

      safety_pressure_detected = 0;
      wait_cycles = 100*wait_time_after_resistance;
      index = profile_length - 2;  // set index to the point of waiting 
    }  // stop the reverse when reching the cycle start point

    if(in_wait == 0)
      index += (1 + cycles_lost);  // advance index while not waiting at the end of cycle

    if(patient_triggered_breath == 1)  // detect drop in presure during the PEEP plateu and trigger breath based on this
    {
      if(in_wait == 1 || (index > profile_length/2 && (A_pot < min_arm_pos + range/18)))
      {
        if(avg_pres - pressure_abs > delta_pres_patient_inhale)
          start_new_cycle();  // start new breath cycle if patient tries to inhale durint the PEEP plateu

        avg_pres = avg_pres*alpha_pres + (1 - alpha_pres)*float(pressure_abs);  // calculate the filtered pressure
      }
      else
      { 
        avg_pres = pressure_abs;
      }  // initialize the filtered pressure
    }

    if(index >= (profile_length - 2))  // wait for the next cycle to begin in this point -> 2 points befoe the last cycle index
    {
      if(sent_LCD == 0)
      {
        sent_LCD = 1;
#if LCD_available == 1
        display_LCD();    // update the display at the end of cycle
#endif
      }

      if(millis() - start_wait < breath_cycle_time)
      {
        index = profile_length - 2;
        in_wait = 1;  // still need to wait ...
      }
      else
        start_new_cycle();  // time has come ... start from index = 0 
    }

    blink_user_led();
  }

//  calc_failure();
  set_motor_PWM(wanted_vel_PWM);
//  find_min_max_pressure();
}

//---------------- CONTROL ----------------------------------------------
void calculate_wanted_pos_vel()
{
  byte pos_from_profile, vel_from_profile;

  pos_from_profile = pgm_read_byte_near(pos + index);
  vel_from_profile = pgm_read_byte_near(vel + index + 1);

  range = range_factor*(max_arm_pos - min_arm_pos);  // range of movement in pot' readings
  wanted_pos = float(pos_from_profile)*range/255 + min_arm_pos;  // wanted pos in pot clicks
  profile_planned_vel = (float(vel_from_profile) - 128.01)*range/255;  // in clicks per 0.2 second

  planned_vel = profile_planned_vel;

  if(hold_breath == 1 && safety_pressure_detected == 0)
  {
    if(wanted_pos <= float(A_pot) || index == 0)
      hold_breath = 0;

    planned_vel = 0;
    integral = 0;
    wanted_pos = float(A_pot);  // hold current position
  }

  if(safety_pressure_detected)  // to do the revese in case high pressure detected
    planned_vel = -speed_multiplier_reverse*planned_vel;

  prev_error = error;
  error = wanted_pos - float(A_pot);
#if DEBUG == 1  
  error = -ERROR_DEBUG + random(2*ERROR_DEBUG);
#endif

  integral += error*float(wanted_cycle_time)/1000;

  if(integral > integral_limit)
    integral = integral_limit;

  if(integral < -integral_limit)
    integral = -integral_limit;

  if(index < 2 || prev_error*error < 0)
    integral = 0;  // zero the integral accumulator at the beginning of cycle and movement up

  if(planned_vel < 0)
    f_reduction_up = f_reduction_up_val;
  else
    f_reduction_up = 1;  // reduce f for the movement up
 
  // PID correction 
  wanted_vel_PWM = FF*planned_vel*f_reduction_up + KP*error + KI*integral;

  // reduce speed for longer cycles
  wanted_vel_PWM = wanted_vel_PWM*float(cycleTime)/float(wanted_cycle_time);
}


void standby_func()  // not running profile
{
  if(USR_status)
  {
    if(millis() - lastUSRblink > 10)
    {
      USR_status = 0;
      lastUSRblink = millis();
      LED_USR(0);
    }
  }
  else
  {
    if(millis() - lastUSRblink > 490)
    {
      USR_status = 1;
      lastUSRblink = millis();
      LED_USR(1);
    }
  }
  
  if(TST_pressed)
  {
    initialize_breath();
    progress = 1;
  }

  if(progress == 1)
  {
    run_profile_func();

    if(cycle_number > 0)
      progress = 0;
  }
  else
  {
    wanted_vel_PWM = 0;  // dont move
    set_motor_PWM(wanted_vel_PWM);
  }

  delay(1);
}

void initialize_breath()
{
  cycle_number = 0;
  start_wait = millis();
  integral = 0;
  reset_failures();
  index = 0;
  in_wait = 0;
  high_pressure_detected = 0;
}

void start_new_cycle()
{
  index = 0;
  cycle_number += 1;
  start_wait = millis();
  in_wait = 0;
  send_beep = 1;
  sent_LCD = 0;
  high_pressure_detected = 0;
}

int range_pot(int val, int low, int high)
{
  int new_val;
  new_val = int(long(val-low)*long(1023)/(high - low));
  new_val = constrain(new_val, 0, 1023);

  return(new_val);
}

void blink_user_led()
{
  if(high_pressure_detected || safety_pressure_detected)  // blink LED fast
  {
    if(USR_status)
    {
      if(millis() - lastUSRblink > 20)
      {
        USR_status = 0;
        lastUSRblink = millis();
        LED_USR(0);
      }
    }
    else
    {
      if(millis() - lastUSRblink > 80)
      {
        USR_status = 1;
        lastUSRblink = millis();
        LED_USR(1);
      }
    }
  }
  else  // not in failure - blink LED once per cycle 
  {
    if(index > 0.1*profile_length)
      LED_USR(0);
    else
      LED_USR(1);
  }
}

void reset_failures()
{
  motion_failure = 0;
  index_last_motion = index;
  failure = 0;
}

void set_motor_PWM(float wanted_vel_PWM)
{
  if(abs(A_pot - prev_A_pot) > 0 || abs(wanted_vel_PWM) < 15)
    index_last_motion = index;

//  if(calibON == 1)
//    wanted_vel_PWM = read_motion_for_calib();  // allows manual motion during calibration

  if(invert_mot)
    wanted_vel_PWM = -wanted_vel_PWM;

  if(curr_sense)
  {
    if(A_current > max_allowed_current)
      wanted_vel_PWM = 0;
  }

  if(motion_failure == 1 && calibON == 0)
    wanted_vel_PWM = 0;

  if(wanted_vel_PWM > 0)
    wanted_vel_PWM += 3;  // undo controller dead band

  if(wanted_vel_PWM < 0)
    wanted_vel_PWM -= 3;  // undo controller dead band

  if(wanted_vel_PWM > PWM_max)
    wanted_vel_PWM = PWM_max;  // limit PWM

  if(wanted_vel_PWM < PWM_min)
    wanted_vel_PWM = PWM_min;  // limit PWM

// Set PWM through the REV Robotics SPARK Motor driver. Values between 0 and 180 ---
//  motorPWM = PWM_mid + int(wanted_vel_PWM);
//  motor.write(motorPWM);

// Set PWM through the VNH5019A-E driver. Values between 0 and 255 ---
  if(wanted_vel_PWM < 0)
  {
    digitalWrite(pin_INA, LOW);
    digitalWrite(pin_INB, HIGH);
    wanted_vel_PWM = -wanted_vel_PWM;
  }
  else
  {
    digitalWrite(pin_INA, HIGH);
    digitalWrite(pin_INB, LOW);
  }

  motorPWM = (int)(wanted_vel_PWM*255.0/PWM_max);  // set between 0 and 255
  analogWrite(pin_PWM, motorPWM);
}

void store_prev_values()
{
  prev_FD = FD;
  prev_FU = FU;
  prev_AD = AD;
  prev_AU = AU;
  prev_SW2 = SW2;
  prev_TST = TST;
  prev_BPM = BPM;
  prev_A_pot = A_pot;
  prev_Compression_perc = Compression_perc;
}

// The program stays in this function until the potentiometers are placed so
// that the corresponding temporary PID values are close to the current PID
// values (within the margins). This is to prevent sudden changes of the
// values, given that the potentiometers could be in any position.
void align_PID_pots()
{
  while(abs(FF - FF_temp) > DELTA_FF ||
        abs(KP - KP_temp) > DELTA_KP ||
        abs(KI - KI_temp) > DELTA_KI)
  {
      FF_temp = (FF_MAX - FF_MIN)*1.0*(analogRead(pin_AMP) - 
                   comp_pot_low)/(1.0*(comp_pot_high - comp_pot_low)) + FF_MIN;
      FF_temp = constrain(FF_temp, FF_MIN, FF_MAX);

      KP_temp = (KP_MAX - KP_MIN)*1.0*(analogRead(pin_FRQ) - 
                   rate_pot_low)/(1.0*(rate_pot_high - rate_pot_low)) + KP_MIN;
      KP_temp = constrain(KP_temp, KP_MIN, KP_MAX);

      KI_temp = (KI_MAX - KI_MIN)*1.0*(analogRead(pin_PRE) - 
                   pres_pot_low)/(1.0*(pres_pot_high - pres_pot_low)) + KI_MIN;
      KI_temp = constrain(KI_temp, KI_MIN, KI_MAX);

    if(millis() - last_sent_data > DELTA_TELE_MONITOR)
    { 
      if(telemetry == 1)
        print_tele();

      last_sent_data = millis();
    }
  }
}

void set_PID_constants()
{
  FF = (FF_MAX - FF_MIN)*1.0*(analogRead(pin_AMP) - 
                   comp_pot_low)/(1.0*(comp_pot_high - comp_pot_low)) + FF_MIN;
  FF = constrain(FF, FF_MIN, FF_MAX);

  KP = (KP_MAX - KP_MIN)*1.0*(analogRead(pin_FRQ) - 
                   rate_pot_low)/(1.0*(rate_pot_high - rate_pot_low)) + KP_MIN;
  KP = constrain(KP, KP_MIN, KP_MAX);

  KI = (KI_MAX - KI_MIN)*1.0*(analogRead(pin_PRE) - 
                   pres_pot_low)/(1.0*(pres_pot_high - pres_pot_low)) + KI_MIN;
  KI = constrain(KI, KI_MIN, KI_MAX);
}


// Function that reads the status of User Interface (buttons, pots, etc.),
// and updates the corresponding global variables (parameters, alarms, etc).
void read_IO()
{
  store_prev_values();

#ifdef pin_RST
  RST = (1 - digitalRead(pin_RST));
#endif

  TSTtemp = (1 - digitalRead(pin_TST));
  SW2temp = (1 - digitalRead(pin_SW2));

  if(SW2temp == 1)
  {
    counter_ON += 1;

    if(counter_ON > 20)
    {
      SW2 = 1;
      counter_ON = 100;
    }
  }
  else
    counter_ON = 0;

  if(SW2temp == 0)
  {
    counter_OFF += 1;

    if(counter_OFF > 20)
    {
      SW2 = 0;
      counter_OFF = 100;
    }
  }
  else
    counter_OFF = 0;

  if(SW2 == 0 && prev_SW2 == 1)
    SW2_pressed = 1;
  else
    SW2_pressed = 0;

  if(TSTtemp == 1)
  {
    counter_TST_ON += 1;

    if(counter_TST_ON > 20)
    {
      TST = 1;
      counter_TST_ON = 100;
    }
  }
  else
    counter_TST_ON = 0;

  if(TSTtemp == 0)
  {
    counter_TST_OFF += 1;

    if(counter_TST_OFF > 20)
    {
      TST = 0;
      counter_TST_OFF = 100;
    }
  }
  else
    counter_TST_OFF = 0;

  if(TST == 0 && prev_TST == 1)
    TST_pressed = 1;
  else
    TST_pressed = 0;

//  A_pot = analogRead(pin_POT);

  if(invert_pot)
    A_pot = 1023 - analogRead(pin_POT);
  else
    A_pot = analogRead(pin_POT);

//  A_current = analogRead(pin_CUR)/8;  // in tenth Amps

  if(control_with_pot)
  {
    A_rate = analogRead(pin_FRQ);
    A_comp = analogRead(pin_AMP);
    A_pres = analogRead(pin_PRE);

    if(abs(pot_rate - A_rate) < 5)
      pot_rate = pot_alpha*pot_rate + (1 - pot_alpha)*A_rate;
    else
      pot_rate = A_rate;

    if(abs(pot_comp - A_comp) < 5)
      pot_comp = pot_alpha*pot_comp + (1 - pot_alpha)*A_comp;
    else
      pot_comp = A_comp;

    if(abs(pot_pres - A_pres) < 5)
      pot_pres = pot_alpha*pot_pres + (1 - pot_alpha)*A_pres;
    else
      pot_pres = A_pres;

    A_comp = range_pot(int(pot_comp), comp_pot_low, comp_pot_high);
    A_rate = range_pot(int(pot_rate), rate_pot_low, rate_pot_high);
    A_pres = range_pot(int(pot_pres), pres_pot_low, pres_pot_high);
 
    Compression_perc = perc_of_lower_vol_display + int(float(A_comp)*(100 - perc_of_lower_vol_display)/1023);
    Compression_perc = constrain(Compression_perc, perc_of_lower_vol_display, 100);

    BPM = 6 + (A_rate - 23)/55;       // 0 is 6 breaths per minute, 1023 is 24 BPM
    breath_cycle_time = 60000/BPM + 100;  // in milisec

    insp_pressure = 30 + A_pres/25;          // 0 is 30 mBar, 1023 is 70 mBar
    insp_pressure = constrain(insp_pressure, 30, 70);

    if(abs(insp_pressure - prev_insp_pressure) > 1)
    {
      prev_insp_pressure = insp_pressure;
      start_disp_pres = millis();

#if LCD_available == 1
      display_LCD();
#endif
    }
  }

  range_factor = perc_of_lower_volume +
                (Compression_perc - perc_of_lower_vol_display)*(100 - perc_of_lower_volume)/(100 - perc_of_lower_vol_display);
  range_factor = range_factor/100;

  if(range_factor > 1)
    range_factor = 1;

  if(range_factor < 0)
    range_factor = 0;

#if(pressure_sensor_available==1)
  if(millis() - last_read_pres > 100)
  {
    last_read_pres = millis();

    //pressure_abs = int(sparkfumPress.getPressure(ADC_4096)-pressure_baseline);   // mbar
    pressure_abs = int(adafruitPress.readPressure() - pressure_baseline);

    if(pressure_abs < 0)
      pressure_abs = 0;
  }
#endif

#if LCD_available == 1
  if(prev_BPM != BPM || prev_Compression_perc != Compression_perc)
    display_LCD();
#endif

  wanted_cycle_time = int(100)*int(motion_time)/profile_length; // between 10 and 20

  if(wanted_cycle_time > breath_cycle_time/profile_length)
    wanted_cycle_time = breath_cycle_time/profile_length;  // máximo es 40.4

  if(wanted_cycle_time < cycleTime)
    wanted_cycle_time = cycleTime;  // 10
}

void LED_USR(byte val)
{
  digitalWrite(pin_USR, val);
}

void print_tele()  // UNCOMMENT THE TELEMETRY NEEDED
{
  if(telemetry_option == 0)
  {
    Serial.print("Fail (disc, motion, hiPres): ");
    Serial.print(disconnected);
    Serial.print(", ");
    Serial.print(motion_failure);
    Serial.print(", ");
    Serial.print(high_pressure_detected);

    // Serial.print(" CL:");
    // Serial.print(cycles_lost);

    Serial.print("; (min_arm_pos, max_arm_pos):");
    Serial.print(min_arm_pos);
    Serial.print(", ");
    Serial.println(max_arm_pos);

    // Serial.print(" cur:");
    // Serial.print(A_current); 

    // Serial.print(" amp:");
    // Serial.print(A_amplitude); 

    // Serial.print(", freq:");
    // Serial.print(A_rate); 

    // Serial.print(" w cyc t:");
    // Serial.print(wanted_cycle_time);

    // Serial.print(" P(mBar):");
    // Serial.println(pressure_abs);

    Serial.print("RF: ");                Serial.print(range_factor); 
    Serial.print(", Index: ");           Serial.print(index);
    Serial.print(", Feedback: ");        Serial.print(A_pot);
    Serial.print(", wanted_pos: ");      Serial.print(wanted_pos);
    Serial.print(", error: ");           Serial.print(error);
    Serial.print(", integral: ");        Serial.println(integral);

    Serial.print("planned_vel: ");       Serial.print(planned_vel);
    Serial.print(", wanted_vel_PWM: ");  Serial.print(wanted_vel_PWM);
    Serial.print(", motorPWM: ");        Serial.print(motorPWM);
    Serial.print(", Calibrated: ");      Serial.println(calibrated);

    Serial.print("Current FF: ");        Serial.print(FF);
    Serial.print(", Current KP: ");      Serial.print(KP);
    Serial.print(", Current KI: ");      Serial.println(KI);
  }

  if(telemetry_option == 2)
  {
    Serial.print("Current FF: ");        Serial.print(FF);
    Serial.print(", FF_temp: ");         Serial.println(FF_temp);
    Serial.print("Current KP: ");        Serial.print(KP);
    Serial.print(", KP_temp: ");         Serial.println(KP_temp);
    Serial.print("Current KI: ");        Serial.print(KI);
    Serial.print(", KI_temp: ");         Serial.println(KI_temp);
  }

  Serial.println("");
}
