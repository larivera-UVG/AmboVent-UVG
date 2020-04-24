/*
 AmboVent-UVG
 Based on the original code for the AmboVent (April 12, 2020)

 CÓDIGO PARA PROBAR MÓDULOS/FUNCIONES QUE SE VAYAN AGREGANDO

***************** Key modifications *******************************************
 Pressure sensor: This version uses an Adafruit MPRLS pressure sensor (hPa),
 instead of the SparkFun MS5803 sensor (mbar).
 Conversion: 1 hPa (hectopascal) = 100 Pa (pascal) = 1 mbar

 Motor driver: This version uses a VNH5019A-E motor driver, instead of the
 REV Robotics SPARK Motor driver.
*******************************************************************************
*/

/*
 *  THIS CODE WAS WRITTEN FOR USE IN A HOME MADE VENTILATION DEVICE. 
 *  IT IS NOT TESTED FOR SAFETY AND NOT RECOMENDED FOR USE IN ANY COMERCIAL DEVICE.
 *  IT IS NOT APPROVED BY ANY REGULAOTRY AUTHORITY
 *  USE ONLY AT YOUR OWN RISK.
 */

/*  to start calibrations - first enter the maintenance setup menu by pressing TEST button for 3 seconds
 *  using the RATE potentiometer select the calibration required and press TEST to select
 *  follow instructions on screen
 *  for the Arm range calibration - use the Rate potentiometer to move the arm up/down
 */

#define DEBUG 0     // 1 para que forzar a |error| < ERROR_DEBUG
#define ERROR_DEBUG 20

// system configuration
#define full_configuration 1        // 1 is the default - full system.   0 is for partial system - potentiometer installed on pulley, no potentiometers, ...
#define pressure_sensor_available 1 // 1 - you have installed an I2C pressure sensor 
#define central_monitor_system 0    // 1 - send unique ID for 10 seconds upon startup, 0 - dont

// options for display and debug via serial com
#define send_to_monitor 1     // 1 = send data to monitor  0 = dont
#define telemetry 1           // 1 = send telemetry for debugging
#define DELTA_TELE_MONITOR 500  // Delta time (in ms) for displaying telemetry and info to monitor

// UI
#define deltaUD 5       // define the value change per each button press
#define pot_alpha 0.85  // filter the pot values

// clinical
// PROBAR DEJAR LOS SIGUIENTES DOS IGUALES, Y VARIARLOS AL MISMO TIEMPO.
#define perc_of_lower_volume 50.0       // % of max press - defines lower volume  // 50.0
#define perc_of_lower_vol_display 33.0  // % of max press - defines lower volume to display when reaching the real lower volume // 33.0

#define wait_time_after_resistance 3    // seconds to wait before re-attempt to push air after max pressure was achieved 
#define max_pres_disconnected 10        // if the max pressure during breathing cycle does not reach this value - pipe is disconnected
#define insp_pressure_default 40        // default value - hold this pressure while breathing - the value is changed if INSP_Pressure potentiometer is inatalled 
#define safety_pres_above_insp 10       // defines safety pressure as the inspirium pressure + this one
#define safety_pressure 70              // quickly pullback arm when reaching this pressure in cm H2O
#define speed_multiplier_reverse 2      // factor of speeed for releasing the pressure (runs motion in reverse at X this speed
#define motion_time_default 35          // motion time in 100 mSec 35 = 3500 mSec
#define patient_triggered_breath_def 1  // 1 = trigger new breath in case of patient inhale during the PEEP plateu 
#define delta_pres_patient_inhale 5     // in cmH2O
#define alpha_pres 0.98                 // used to average the pressure during the PEEP plateu


#if (full_configuration == 0)  // Arm connected with strip or wire
  #define LCD_available 0 
  #define pres_pot_available 0  // 1 if the system has 3 potentiometer and can control the inspirium pressure 
  #define pin_SW2 7   // breath - On / Off / cal
  #define pin_TST 2   // test mode - not in use
  #define pin_LED_AMP 11   // amplitude LED
  #define pin_LED_FREQ 9   // frequency LED
  #define pin_LED_Fail 10  // FAIL and calib blue LED
  #define pin_USR 12  // User LED
  #define pin_FD 4    // freq Down
  #define pin_FU 5    // freq Up
  #define pin_AD 8    // Amp Down
  #define pin_AU 6    // Amp Up
  #define curr_sense 1 
  #define control_with_pot 1    // 1 = control with potentiometers  0 = with push buttons
  #define FF 0.6      // motion control feed forward  
  #define KP 0.2      // motion control propportional gain 
  #define KI 2        // motion control integral gain 
  #define integral_limit 6  // limits the integral of error 
  #define f_reduction_up_val 0.65   // reduce feedforward by this factor when moving up

#endif

#if (full_configuration == 1) // Direct arm conection 
  #define LCD_available 1 
  #define pres_pot_available 1  // 1 if the system has 3 potentiometer and can control the inspirium pressure 
  #define pin_TST 4         // test mode - not in use
  #define pin_SW2 5         // breath - On / Off / cal
  #define pin_RST 6         // reset alarm
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

  #define FF_MIN 0.05
  #define FF_MAX 3
  #define FF_DEF 0.6              // motion control feed forward. 0.6,  4.5
  #define DELTA_FF ((FF_MAX-FF_MIN)/100.0)

  #define KP_MIN 0.05
  #define KP_MAX 1.5
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

// Pins for factory reset and configuration switches
#define pin_FRESET  2   // VERIFICAR ESTOS PINES
#define pin_CONFIG 13   // VERIFICAR ESTOS PINES

// Talon SR or SPARK controller PWM settings ("angle" for Servo library)
#define PWM_mid 93  // mid value for PWM 0 motion - higher pushes up
#define PWM_max 85
#define PWM_min (-PWM_max)
#define PWM_THR 20    // Umbral
#define max_allowed_current 100 // 100 <=> 10 Amps

// motion control parameters
#define cycleTime 10          // milisec
#define alpha 0.95            // filter for current apatation - higher = stronger low pass filter
#define profile_length 250    // motion control profile length
#define motion_control_allowed_error  40  // % of range

// motor and sensor definitions
#define invert_mot 0  // Cuidado con esto. Ver notas en
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
Adafruit_MPRLS adafruitPress(-1, -1);  // Default values
#endif

#if LCD_available == 1
// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);
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

byte FD, FU, AD, AU, prev_FD, prev_FU, prev_AD, prev_AU,
     LED_status, USR_status, blueOn, calibrated = 0, calibON, numBlinkFreq,
     menu_state, manual_mov_enabled = 0, PID_calib_enabled = 0;

byte monitor_index = 0, BPM = 14, prev_BPM, in_wait, failure, send_beep,
     wanted_cycle_time, disconnected = 0, high_pressure_detected = 0,
     motion_failure = 0, sent_LCD, hold_breath, safety_pressure_detected;

// For the detection of push buttons
byte SW2, prev_SW2, SW2temp, SW2_pressed, counter_SW2_ON, counter_SW2_OFF,
     TST, prev_TST, TSTtemp, TST_pressed, counter_TST_ON, counter_TST_OFF,
     RST, prev_RST, RSTtemp, RST_pressed, counter_RST_ON, counter_RST_OFF;

byte insp_pressure, prev_insp_pressure, safety_pressure_counter, no_fail_counter,
     patient_triggered_breath, motion_time, progress,
     telemetry_option = 0, adjusting_PID = 0; //, PID_pots_aligned;

int A_pot, prev_A_pot, A_current, Compression_perc = 80, prev_Compression_perc,
    A_rate, A_comp, A_pres;
int motorPWM, index = 0, prev_index, i, wait_cycles, cycle_number, cycles_lost,
    index_last_motion, pos_from_pot;
int pressure_abs, breath_cycle_time, max_pressure = 0, prev_max_pressure = 0,
    min_pressure = 100, prev_min_pressure = 0, index_to_hold_breath, pressure_baseline;
int comp_pot_low = 0, comp_pot_high = 1023, rate_pot_low = 0, rate_pot_high = 1023,
    pres_pot_low = 0, pres_pot_high = 1023;

unsigned int max_arm_pos, min_arm_pos;
unsigned long lastSent, lastIndex, lastUSRblink, last_TST_not_pressed, lastBlue,
              start_wait, last_sent_data, last_read_pres, start_disp_pres;

float pot_rate, pot_pres, pot_comp, avg_pres;
float wanted_pos, wanted_vel_PWM, range, range_factor, profile_planned_vel,
      planned_vel, integral, error, prev_error, f_reduction_up,
      wanted_manual_vel_PWM;

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
  pinMode(pin_TST, INPUT_PULLUP);  
  pinMode(pin_SW2, INPUT_PULLUP);
  pinMode(pin_RST, INPUT_PULLUP);

  pinMode(pin_CONFIG, INPUT_PULLUP);
  pinMode(pin_FRESET, INPUT_PULLUP);

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

  EEPROM.get(36, FF);                    delay(20);
  EEPROM.get(36 + sizeof(float), KP);    delay(20);
  EEPROM.get(36 + 2*sizeof(float), KI);  delay(20);

  if(FF < FF_MIN || FF > FF_MAX || isnan(FF) == 1)
    FF = FF_DEF;

  if(KP < KP_MIN || KP > KP_MAX || isnan(KP) == 1)
    KP = KP_DEF;

  if(KI < KI_MIN || KI > KI_MAX || isnan(KI) == 1)
    KI = KI_DEF;

// TEMPORAL, PARA FORZAR LAS POSICIONES EXTREMAS DEL BRAZO
//   min_arm_pos = 150;
//   max_arm_pos = 600;
// -------------------------------------------------------

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
  read_IO();    // Read status of User Interface, act accordingly

  switch(state)
  {
    case STBY_STATE:     // standby
      standby_func();

      if(SW2_pressed && calibrated == 1)  // start breathing motion
      { 
        state = BREATH_STATE;
        initialize_breath();
      }

      if(TST == 0)
        last_TST_not_pressed = millis();

      if(millis() - last_TST_not_pressed > 3000)
      { 
        LED_USR(1);

        while(TST == 1 || TST_pressed)
        {
          read_IO(); 
        }   // wait for button release

        progress = 0;
        state = MENU_STATE;
      }

      if(motion_failure == 1 || RST_pressed == 1)
        reset_failures();

      break;

    case BREATH_STATE:     // run profile
      run_profile_func();

      if(SW2_pressed)
        state = STBY_STATE;  // stop breathing motion

      if(motion_failure == 1 || RST_pressed == 1)
      {
        reset_failures();
        state = STBY_STATE;
      }

      break;

    case MENU_STATE:     // maintanance menu
      display_menu();

      break;
  }
  
  if(millis() - last_sent_data > DELTA_TELE_MONITOR)
  { 
    if(send_to_monitor == 1 && telemetry == 0)
      send_data_to_monitor();

    if(telemetry == 1)
      print_tele();

    last_sent_data = millis();
  }
}


void display_menu()
{
  // ¿No será mejor mapear de rate_pot_low, rate_pot_high a 0, 9?
//  menu_state = map(pot_rate, 0, 1023, 0, 9);
  menu_state = map(pot_rate, rate_pot_low, rate_pot_high, 0, 9);

  menu_state = constrain(menu_state, 0, 9);

  switch(menu_state)
  {
    case 1:     // calib pot
      display_text_2_lines("Calibrate Pots", "TEST to start");

      if(TST_pressed)
      {
        calibrate_pot_range();
        exit_menu();
      }

      break;

    case 2:     // calib pressure sensor
      display_text_2_lines("Calib pressure", "TEST to start");

      if(TST_pressed)
      {
#if pressure_sensor_available == 1
        //pressure_baseline = int(sparkfumPress.getPressure(ADC_4096));
        pressure_baseline = int(adafruitPress.readPressure());
#endif
        exit_menu();
      }

      break;

    case 3:     // move arm down once
      if(progress == 0)
      {
        display_text_2_lines("Press TEST to", "run one breath  ");

        if(TST_pressed) 
        {
          initialize_breath();
          progress = 1;
        }
      }

      if(progress == 1)
      {
        run_profile_func();

        if(cycle_number > 0)
          exit_menu();
      }

      break;
 
    case 4:     // calib arm range of movement
      display_text_2_lines("Calibrate Arm", "TEST to start");

      if(TST_pressed)
      {
        calibrate_arm_range();  // AGREGAR MOVIMIENTO MANUAL
        exit_menu();
      }

      break;

    case 5:     // set motion profile total time
      display_text_2_lines("Set Motion Time", "TEST to start ");

      if(TST_pressed)
      {
        read_IO();

        while(TST_pressed == 0)
        {
          read_IO();

          // ¿No será mejor mapear de rate_pot_low, rate_pot_high a 25, 50?
          motion_time = map(pot_rate, 0, 1023, 25, 50);
          motion_time = constrain(motion_time, 25, 50);

          if(millis() - lastUSRblink > 100)
          {
            lastUSRblink = millis();

#if LCD_available == 1
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Set Motion Time");
            lcd.setCursor(0, 1);
            lcd.print(int(100*motion_time));
            lcd.print(" mSec");
#endif
          }
        }

        delay(500);
        exit_menu();
      }

      break;

    case 6:     // toggle sync to patient
      if(patient_triggered_breath == 1)
        display_text_2_lines("Sync to patient", "ON  ");

      if(patient_triggered_breath == 0)
        display_text_2_lines("Sync to patient","OFF  ");

      if(TST_pressed)
      {
        patient_triggered_breath = 1 - patient_triggered_breath;
        delay(110); //  wait enough time that the display will be updated ..

        if(patient_triggered_breath == 1)
         display_text_2_lines("Sync to patient", "ON  ");

        if(patient_triggered_breath == 0)
          display_text_2_lines("Sync to patient", "OFF  ");

        delay(1000);
        exit_menu();
      }

      break;

    case 7:    // Manual movement of the arm, if enabled
      if(manual_mov_enabled)
      {
        display_text_2_lines("Manual Movement", "TEST to start");
        
        if(TST_pressed)
        {
          telemetry_option = 1;
          move_arm_with_pot();
          exit_menu();
        }
      }
      else
      {
        display_text_2_lines("Man Mov Disbld.", "Press TEST ");

        if(TST_pressed)
          exit_menu();
      }

      break;

    case 8:    // PID calibration, if enabled
       if(PID_calib_enabled)
       {
          display_text_2_lines("PID Calib ", "TEST to start");

          if(TST_pressed)
          {
            telemetry_option = 2;
// Primero colocar los pots en la posición correspondiente a las
// constantes actuales, para que no haya cambios bruscos.
            align_PID_pots();

            telemetry_option = 0;
            adjusting_PID = 1;

            read_IO();

            while(TST_pressed == 0)
            {
              set_PID_constants();
              run_profile_func();
              read_IO();
            }

            // Save control constants to the EEPROM
            EEPROM.put(36, FF);                    delay(200);
            EEPROM.put(36 + sizeof(float), KP);    delay(200);
            EEPROM.put(36 + 2*sizeof(float), KI);  delay(200);

            adjusting_PID = 0;
            exit_menu();
          }
        }
        else
        {
          display_text_2_lines("PID Cal Disbld.", "Press TEST ");

          if(TST_pressed)
            exit_menu();
        }

      break;


    default:
      display_text_2_lines("Exit Menu", "Press TEST ");

      if(TST_pressed)
        exit_menu();

      break;
  }
}

void exit_menu()
{
  read_IO();
  last_TST_not_pressed = millis();
  state = STBY_STATE;
  index = 0;
  calibON = 0;
  telemetry_option = 0;

#if LCD_available == 1
  display_LCD();
#endif

  progress = 0;
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

  calc_failure();
  set_motor_PWM(wanted_vel_PWM);
  find_min_max_pressure();
}

//---------------- CONTROL ----------------------------------------------
void calculate_wanted_pos_vel()
{
  byte pos_from_profile, vel_from_profile;

  pos_from_profile = pgm_read_byte_near(pos + index);
  vel_from_profile = pgm_read_byte_near(vel + index + 1);

  range = range_factor*(max_arm_pos - min_arm_pos);  // range of movement in pot' readings
  
// AQUÍ INCLUIR LO DEL VECTOR DE PESOS, PARA AJUSTAR MEJOR
// EL VOLUMEN DE COMPRESIÓN. DE REPENTE UNOS 20 PUNTOS.

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

// Maps back to 0 - 1023 interval
int range_pot(int val, int low, int high)
{
  int new_val;
  new_val = int(long(val-low)*long(1023)/(high - low));
  new_val = constrain(new_val, 0, 1023);

  return(new_val);
}


void find_min_max_pressure()
{
  if(max_pressure < pressure_abs)
    max_pressure = pressure_abs;  // find the max pressure in cycle

  if(min_pressure > pressure_abs)
    min_pressure = pressure_abs;  // find the min pressure in cycle

  if(index > profile_length - 10 && index < profile_length - 5)
  {
    prev_min_pressure = min_pressure;
    prev_max_pressure = max_pressure;
  }

  if(index >= profile_length - 5)
  { 
    max_pressure = 0;
    min_pressure = 999;
  }
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

void calc_failure()
{
  if(prev_max_pressure < max_pres_disconnected && cycle_number > 2)
    disconnected = 1;
  else
    disconnected = 0;  // tube was disconnected

  if(pressure_abs > insp_pressure && hold_breath == 0 && profile_planned_vel > 0)
  {
    high_pressure_detected = 1;
    hold_breath = 1;
    index_to_hold_breath = index;
  }  // high pressure detected

  if(pressure_abs > safety_pressure && profile_planned_vel > 0)
    safety_pressure_detected = 1;

  if(pressure_abs > insp_pressure + safety_pres_above_insp && profile_planned_vel > 0)
    safety_pressure_detected = 1;

  if(index == 0 && prev_index !=0 && failure == 0 && safety_pressure_detected == 0)
    no_fail_counter += 1;

  if(index == 0)
    failure = 0;

  if(disconnected)
    failure = 1;

  if(safety_pressure_detected && safety_pressure_counter >= 1)
  {
    failure = 2;
    safety_pressure_counter = 1;
  }

  if(motion_failure)
    failure = 3;  // 3

  if(disconnected == 1 || motion_failure == 1 || safety_pressure_detected == 1)
  {
    no_fail_counter = 0;
  }
  else
  {
    LED_FAIL(0);
  }

  if(no_fail_counter >= 3)
    safety_pressure_counter = 0;

  if(no_fail_counter >= 100)
    no_fail_counter = 100;

  prev_index = index;
}


void display_text_2_lines(char *message1, char *message2)
{
  if(millis() - lastUSRblink > 100)
  {
    lastUSRblink = millis();
#if LCD_available == 1
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(message1);
    lcd.setCursor(0, 1);
    lcd.print(message2);
#endif
  }
}

void display_text_calib(char *message)
{
#if LCD_available == 1
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(message);
  lcd.setCursor(0, 1);
  lcd.print("Then press Test");
#endif
}

void display_pot_during_calib()
{
  if(millis() - lastUSRblink > 100)
  {
#if LCD_available == 1
    lcd.setCursor(13, 0);
    lcd.print(A_pot);
    lcd.print(" ");
#endif
    lastUSRblink = millis();
  }
}

// Al calibrar las posiciones extremas del brazo (con el potenciómetro de Feedback),
// se asume que la "Upper position" corresponde al valor más bajo del potenciómetro,
// y la "Lower position", al valor más alto. Tiene que ser así.
void calibrate_arm_range()   // used for calibaration of motion range
{
  LED_USR(1);
  calibON = 1;
  progress = 0;

  display_text_calib("Set Upper");

  while(progress == 0)
    internal_arm_calib_step();  // step 1 - calibrate top position

  progress = 0;
  min_arm_pos = A_pot;

  display_text_calib("Set Lower");

  while(progress == 0)
    internal_arm_calib_step();  // step 2 - calibrate bottom position

  progress = 0;
  max_arm_pos = A_pot + 150;  // To compensate for Ambu pushing back.
  max_arm_pos = constrain(A_pot,0, 1023);

  display_text_calib("Move to Safe");

  while(progress == 0)
    internal_arm_calib_step();  // step 3 - manual control for positioning back in safe location 

  EEPROM.put(4, min_arm_pos);
  delay(200);
  EEPROM.put(8, max_arm_pos);
  delay(200);

  calibrated = 1;
}

void internal_arm_calib_step()
{
  read_IO();

  if(TST_pressed)
    progress = 1;

  if(SW2temp)  // Move a little in the positive direction
  {
    digitalWrite(pin_INA, HIGH);
    digitalWrite(pin_INB, LOW);

    motorPWM = 100;  // set between 0 and 255
    analogWrite(pin_PWM, motorPWM);
    delay(100);
    analogWrite(pin_PWM, 0);
  }

  if(RSTtemp)  // Move a little in the negative direction
  {
    digitalWrite(pin_INA, LOW);
    digitalWrite(pin_INB, HIGH);

    motorPWM = 60;  // set between 0 and 255
    analogWrite(pin_PWM, motorPWM);
    delay(100);
    analogWrite(pin_PWM, 0);
  }

  display_pot_during_calib();
  delay(3);
}

void calibrate_pot_range()   // used for calibaration of potentiometers
{ 
  LED_USR(1);
  calibON = 2;

  read_IO();
  display_text_calib("Pot to left pos");

  while(TST_pressed == 0)
    read_IO();    // step 1 - calibrate top position

  comp_pot_low = analogRead(pin_AMP);
  rate_pot_low = analogRead(pin_FRQ);
  pres_pot_low = analogRead(pin_PRE);

  read_IO();
  display_text_calib ("Pot to right pos");

  while(TST_pressed == 0)
    read_IO();    // step 2 - calibrate bottom position

  comp_pot_high = analogRead(pin_AMP);
  rate_pot_high = analogRead(pin_FRQ);
  pres_pot_high = analogRead(pin_PRE);

  EEPROM.put(12, comp_pot_low);   delay(100);
  EEPROM.put(16, comp_pot_high);  delay(100);
  EEPROM.put(20, rate_pot_low);   delay(100);
  EEPROM.put(24, rate_pot_high);  delay(100);
  EEPROM.put(28, pres_pot_low);   delay(100);
  EEPROM.put(32, pres_pot_high);  delay(100);
}

// This function allows to move the arm with the amplitude potentiometer
void move_arm_with_pot()
{
  read_IO();

//  display_text_2_lines("Manual Movement", "TEST to stop");
#if LCD_available == 1
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Manual Movement");
  lcd.setCursor(0, 1);
  lcd.print("TEST to stop");
#endif
  delay(100);

  while(TST_pressed == 0)
  {
    read_IO();    // Read status of User Interface
  
    if(millis() - lastIndex >= wanted_cycle_time)  // wait for the cycle time
    {
      lastIndex = millis();  // last start of cycle time

      set_pos_vel_with_pot();

      if(millis() - last_sent_data >= DELTA_TELE_MONITOR)  // esperar para mostrar telemetry
      {
        last_sent_data = millis();  // last time telemetry was displayed

        if(telemetry == 1)
          print_tele();

       // display_text_2_lines("Manual Movement", "TEST to stop");
          
      }
    }
  }
}

void set_pos_vel_with_pot()
{
  int pot_temp = analogRead(pin_AMP);
  pot_temp = constrain(pot_temp, comp_pot_low, comp_pot_high);

  pos_from_pot = map(pot_temp, comp_pot_low, comp_pot_high, min_arm_pos, max_arm_pos);

  wanted_pos = float(pos_from_pot);

  prev_error = error;
  error = wanted_pos - float(A_pot);

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

  if(curr_sense)
  {
    if(A_current > max_allowed_current)
      wanted_manual_vel_PWM = 0;
  }

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
  }
  else
  {
    digitalWrite(pin_INA, HIGH);
    digitalWrite(pin_INB, LOW);
  }

  motorPWM = (int)(wanted_manual_vel_PWM*255.0/PWM_max);  // set between 0 and 255

  if(motorPWM < PWM_THR)
    motorPWM = 0;
  
  analogWrite(pin_PWM, motorPWM);
}


#if LCD_available == 1
void display_LCD()  // here function that sends data to LCD
{
  if(calibON == 0 && state != MENU_STATE) 
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Dep:");
    lcd.print(byte(Compression_perc));
    lcd.print("%");
    lcd.print("  BPM:");
    lcd.print(byte(BPM));

    lcd.setCursor(0, 1);

    if(failure == 0)
    {
      if(millis() - start_disp_pres < 2000)
      {
        lcd.setCursor(0, 1);
        lcd.print("Insp. Press. :");
        lcd.print(byte(insp_pressure));
      }
      else
      {
        lcd.print("Pmin:");
        lcd.print(byte(prev_min_pressure));
        lcd.print("  Pmax:");
        lcd.print(byte(prev_max_pressure));
      }
    }

    if(failure == 1)
      lcd.print("Pipe Disconnect");

    if(failure == 2)
      lcd.print("High Pressure");

    if(failure == 3)
      lcd.print("Motion Fail");
  }
}
#endif

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

  if(calibON == 1)
    wanted_vel_PWM = read_motion_for_calib();  // allows manual motion during calibration

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

int read_motion_for_calib()
{
  int wanted_cal_PWM;

  if(control_with_pot)
  {
    if(pot_rate > 750)
      wanted_cal_PWM = (pot_rate - 750)/15;

    if(pot_rate < 250)
      wanted_cal_PWM = (pot_rate - 250)/15;

    if(pot_rate >= 250 && pot_rate <= 750)
      wanted_cal_PWM = 0;

    if(SW2 == 1)
      wanted_cal_PWM = -12;

    // if (RST==1) wanted_cal_PWM= 12;

//    Serial.println(wanted_cal_PWM);
  }
  else
  {
    wanted_cal_PWM = 0;

    if(FD == 1)
      wanted_cal_PWM = 8;

    if(FU == 1)
      wanted_cal_PWM = -8;

    if(AD == 1)
      wanted_cal_PWM = 16;

    if(AU == 1)
      wanted_cal_PWM = -16;
  }

  return(wanted_cal_PWM);
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
#if LCD_available == 1
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Set FF to: ");
  lcd.print((float)(FF));
  delay(100);
#endif

  while(abs(FF - FF_temp) > DELTA_FF)
  {
      FF_temp = (FF_MAX - FF_MIN)*1.0*(analogRead(pin_AMP) - 
                   comp_pot_low)/(1.0*(comp_pot_high - comp_pot_low)) + FF_MIN;
      FF_temp = constrain(FF_temp, FF_MIN, FF_MAX);

    if(millis() - last_sent_data > DELTA_TELE_MONITOR)
    { 
      if(telemetry == 1)
        print_tele();

      last_sent_data = millis();
    }

    if(millis() - lastUSRblink > 100)
    {
      lastUSRblink = millis();
#if LCD_available == 1
      lcd.setCursor(0, 1);
      lcd.print("FF = ");
      lcd.print((float)(FF_temp));
#endif
    }
  }

#if LCD_available == 1
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Set KP to: ");
  lcd.print((float)(KP));
  delay(100);
#endif

  while(abs(KP - KP_temp) > DELTA_KP)
  {
      KP_temp = (KP_MAX - KP_MIN)*1.0*(analogRead(pin_FRQ) - 
                   comp_pot_low)/(1.0*(comp_pot_high - comp_pot_low)) + KP_MIN;
      KP_temp = constrain(KP_temp, KP_MIN, KP_MAX);

    if(millis() - last_sent_data > DELTA_TELE_MONITOR)
    { 
      if(telemetry == 1)
        print_tele();

      last_sent_data = millis();
    }

    if(millis() - lastUSRblink > 100)
    {
      lastUSRblink = millis();
#if LCD_available == 1
      lcd.setCursor(0, 1);
      lcd.print("KP = ");
      lcd.print((float)(KP_temp));
#endif
    }
  }

#if LCD_available == 1
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Set KI to: ");
  lcd.print((float)(KI));
  delay(100);
#endif

  while(abs(KI - KI_temp) > DELTA_KI)
  {
      KI_temp = (KI_MAX - KI_MIN)*1.0*(analogRead(pin_PRE) - 
                   comp_pot_low)/(1.0*(comp_pot_high - comp_pot_low)) + KI_MIN;
      KI_temp = constrain(KI_temp, KI_MIN, KI_MAX);

    if(millis() - last_sent_data > DELTA_TELE_MONITOR)
    { 
      if(telemetry == 1)
        print_tele();

      last_sent_data = millis();
    }

    if(millis() - lastUSRblink > 100)
    {
      lastUSRblink = millis();
#if LCD_available == 1
      lcd.setCursor(0, 1);
      lcd.print("KI = ");
      lcd.print((float)(KI_temp));
#endif
    }
  }

#if LCD_available == 1
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Adjust FF KP KI");
  lcd.setCursor(0, 1);
  lcd.print("TEST when done");
  delay(1000);
#endif
}

// Function that reads the potentiometer values and changes
// the FF, KP and KI constants accordingly.
void set_PID_constants()
{
  FF = (FF_MAX - FF_MIN)*1.0*(analogRead(pin_AMP) - 
                   comp_pot_low)/(1.0*(comp_pot_high - comp_pot_low)) + FF_MIN;
  FF = constrain(FF, FF_MIN, FF_MAX);

  KP = (KP_MAX - KP_MIN)*1.0*(analogRead(pin_FRQ) - 
                   rate_pot_low)/(1.0*(rate_pot_high - rate_pot_low)) + KP_MIN;
  KP = constrain(KP, KP_MIN, KP_MAX);

  // Tiene que ser pin_PRE, pero no estaba funcionando. Para mientras, uso el pin_POT
  KI = (KI_MAX - KI_MIN)*1.0*(analogRead(pin_PRE) - 
                   pres_pot_low)/(1.0*(pres_pot_high - pres_pot_low)) + KI_MIN;
  KI = constrain(KI, KI_MIN, KI_MAX);

  if(millis() - lastUSRblink > 100)
  {
    lastUSRblink = millis();
#if LCD_available == 1
    lcd.setCursor(0, 0);
    lcd.print("FF     KP     KI");
    lcd.setCursor(0, 1);
    lcd.print((float)(FF));
    lcd.print("  ");
    lcd.print((float)(KP));
    lcd.print("  ");
    lcd.print((float)(KI));
#endif
  }
}

// Function that reads the status of User Interface (buttons, pots, etc.),
// and updates the corresponding global variables (parameters, alarms, etc).
void read_IO()
{
  store_prev_values();

//  manual_mov_enabled = digitalRead(pin_CONFIG);
//  PID_calib_enabled = digitalRead(pin_CONFIG);
  manual_mov_enabled = 1;
  PID_calib_enabled = 1;

  TSTtemp = (1 - digitalRead(pin_TST));
  SW2temp = (1 - digitalRead(pin_SW2));
  RSTtemp = (1 - digitalRead(pin_RST));

// First push button
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

// Second push button
  if(SW2temp == 1)
  {
    counter_SW2_ON += 1;

    if(counter_SW2_ON > 20)
    {
      SW2 = 1;
      counter_SW2_ON = 100;
    }
  }
  else
    counter_SW2_ON = 0;

  if(SW2temp == 0)
  {
    counter_SW2_OFF += 1;

    if(counter_SW2_OFF > 20)
    {
      SW2 = 0;
      counter_SW2_OFF = 100;
    }
  }
  else
    counter_SW2_OFF = 0;

  if(SW2 == 0 && prev_SW2 == 1)
    SW2_pressed = 1;
  else
    SW2_pressed = 0;

// Third push button
  if(RSTtemp == 1)
  {
    counter_RST_ON += 1;

    if(counter_RST_ON > 20)
    {
      RST = 1;
      counter_RST_ON = 100;
    }
  }
  else
    counter_RST_ON = 0;

  if(RSTtemp == 0)
  {
    counter_RST_OFF += 1;

    if(counter_RST_OFF > 20)
    {
      RST = 0;
      counter_RST_OFF = 100;
    }
  }
  else
    counter_RST_OFF = 0;

  if(RST == 0 && prev_RST == 1)
    RST_pressed = 1;
  else
    RST_pressed = 0;


  if(invert_pot)
    A_pot = 1023 - analogRead(pin_POT);
  else
    A_pot = analogRead(pin_POT);

//  A_current = analogRead(pin_CUR)/8;  // in tenth Amps

  if(control_with_pot)
  {
    // When adjusting PID values, don't use pot values to adjust BPM, 
    // Compression_perc and insp_pressure.
    if(adjusting_PID == 0)
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

      A_rate = range_pot(int(pot_rate), rate_pot_low, rate_pot_high);
      A_comp = range_pot(int(pot_comp), comp_pot_low, comp_pot_high);
      A_pres = range_pot(int(pot_pres), pres_pot_low, pres_pot_high);
    }
    else
    {
      A_rate = 800;  // BPM approx. 20
      A_comp = 615;  // Compression_perc approx. 90
      A_pres = 500;  // insp_pressure = 50
    }

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
  else
  {
    FD = (1 - digitalRead(pin_FD));
    FU = (1 - digitalRead(pin_FU));
    AD = (1 - digitalRead(pin_AD));
    AU = (1 - digitalRead(pin_AU));

    if(TST == 0)
    {
      if(FD == 0 && prev_FD == 1)
      {
        BPM -= 2;

        if(BPM < 6)
          BPM = 6;

        cycle_number = 0;
      }

      if(FU == 0 && prev_FU == 1)
      {
        BPM += 2;

        if(BPM > 24)
          BPM = 24;

        cycle_number = 0;
      }

      breath_cycle_time = 60000/BPM + 100;

      if(AD == 0 && prev_AD == 1)
      {
        Compression_perc -= deltaUD;

       if(Compression_perc < perc_of_lower_vol_display)
         Compression_perc = perc_of_lower_vol_display;
      }

      if(AU == 0 && prev_AU == 1)
      {
        Compression_perc += deltaUD;

        if(Compression_perc > 100)
          Compression_perc = 100;
      }
    }

    if(TST == 1)
    {
      if(FD == 0 && prev_FD == 1)
      {
        insp_pressure -=5;

        if(insp_pressure < 30)
          insp_pressure = 30;
      }

      if(FU == 0 && prev_FU == 1)
      {
        insp_pressure +=5;

        if(insp_pressure > 70)
          insp_pressure = 70;
      }

      if(AD == 0 && prev_AD == 1)
      {
        insp_pressure -=5;

        if(insp_pressure < 30)
          insp_pressure = 30;
      }

      if(AU == 0 && prev_AU == 1)
      {
        insp_pressure +=5;

        if(insp_pressure > 70)
          insp_pressure = 70;
      }
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

void send_data_to_monitor()
{
  if(monitor_index == 0)
    Serial.println("A");

  if(monitor_index == 1)
    Serial.println(byte(BPM));

  if(monitor_index == 2)
    Serial.println(byte(Compression_perc));

  if(monitor_index == 3)
    Serial.println(byte(pressure_abs));

  if(monitor_index == 4)
    Serial.println(byte(failure));

  if(monitor_index == 5)
  {
    if(send_beep)
    {
      Serial.println(byte(1));
      send_beep = 0;
    }
    else
      Serial.println(byte(0));
  }

  if(monitor_index == 6)
    Serial.println(byte(insp_pressure));

  monitor_index = (monitor_index + 1) % 7;
}

void LED_FREQ(byte val)
{
  digitalWrite(pin_LED_FREQ, val);
}

void LED_AMP(byte val)
{
  digitalWrite(pin_LED_AMP, val);
}

void LED_FAIL(byte val)
{
  digitalWrite(pin_LED_Fail, val);
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

    Serial.print("Curr. Press.: ");      Serial.print(adafruitPress.readPressure());
    Serial.print(", P. baseline: ");     Serial.print(pressure_baseline);
    Serial.print(", pressure_abs: ");    Serial.print(pressure_abs);
    Serial.println("");

    Serial.print("RF: ");                Serial.print(range_factor); 
    Serial.print(", Index: ");           Serial.print(index);
    Serial.print(", Feedback: ");        Serial.print(A_pot);
    Serial.print(", wanted_pos: ");      Serial.print(wanted_pos);
    Serial.print(", error: ");           Serial.print(error);
//    Serial.print(", integral: ");        Serial.print(integral);
    Serial.println("");

    Serial.print("planned_vel: ");       Serial.print(planned_vel);
    Serial.print(", wanted_vel_PWM: ");  Serial.print(wanted_vel_PWM);
    Serial.print(", motorPWM: ");        Serial.print(motorPWM);
    Serial.print(", Calibrated: ");      Serial.print(calibrated);
    Serial.println("");

    Serial.print("FF: ");               Serial.print(FF);
    Serial.print(", KP: ");             Serial.print(KP);
    Serial.print(", KI: ");             Serial.print(KI);
    Serial.println("");

/*
    Serial.print("TSTtemp: ");           Serial.print(TSTtemp);
    Serial.print(", TST_pressed: ");     Serial.print(TST_pressed);
    Serial.print(", SW2temp: ");         Serial.print(SW2temp);
    Serial.print(", SW2_pressed: ");     Serial.print(SW2_pressed);
    Serial.print(", RSTtemp: ");         Serial.print(RSTtemp);
    Serial.print(", RST_pressed: ");     Serial.println(RST_pressed);
*/
  }

  if(telemetry_option == 1)
  {
    Serial.print("Feedback: ");          Serial.print(A_pot);
    Serial.print(", pos_from_pot: ");    Serial.print(pos_from_pot);
    Serial.print(", error: ");           Serial.print(error);
    Serial.print(", wanted_manual_vel_PWM: ");  Serial.print(wanted_manual_vel_PWM);
    Serial.print(", motorPWM: ");        Serial.println(motorPWM);
  }

  Serial.println("");
}
