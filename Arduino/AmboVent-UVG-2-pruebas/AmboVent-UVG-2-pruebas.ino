/*
 AmboVent-UVG-2-pruebas
 Based on the original code for the AmboVent (April 12, 2020)
 Modified by Luis Alberto Rivera
 Universidad Del Valle de Guatemala

 CÓDIGO PARA PROBAR MÓDULOS/FUNCIONES QUE SE VAYAN AGREGANDO
 VERSIÓN PARA EL 2do PROTOTIPO DE LA PCB
***************** Key modifications *******************************************
 Pressure sensor: This version uses an Adafruit MPRLS pressure sensor (hPa),
 instead of the SparkFun MS5803 sensor (mbar).
 Conversion: 1 hPa (hectopascal) = 100 Pa (pascal) = 1 mbar

 Motor driver: This version uses a VNH5019A-E motor driver, instead of the
 REV Robotics SPARK Motor driver.
*******************************************************************************
*/

/*
 * THIS CODE WAS WRITTEN FOR USE IN A HOME MADE VENTILATION DEVICE.
 * IT IS NOT TESTED FOR SAFETY AND NOT RECOMENDED FOR USE IN ANY COMERCIAL DEVICE.
 * IT IS NOT APPROVED BY ANY REGULAOTRY AUTHORITY
 * USE ONLY AT YOUR OWN RISK.
 */

/* To start calibrations: first enter the maintenance setup menu by pressing TEST
 * button for 3 seconds. Using the RATE potentiometer, select the calibration
 * required and press TEST to select it. Follow instructions on screen.
 */

// --- System Configuration ---------------------------------------------------
#define LANGUAGE       1 // For LCD messages. 0 - English, 1 - Español.
#define INVERT_LEDS    1 // 1 if LEDs turn ON with 0 (common anode)
#define INVERT_BUZZER  0 // 1 if Buzzer turns ON with 0
#define COMP_PUSHBACK 50 // Compensates push-back from Ambu. Depends on Ambu used.

#define pressure_sensor_available 1 // 1 - I2C pressure sensor installed
#define TempSensor_available 0      // 1 - temperature sensor available
#define LCD_available 1             // 1 - LCD available
#define ALARM_ENABLED 1             // 1 - Alarm will sound, if conditions are met

// motor and sensor definitions
#define invert_mot 0  // Careful with these. Make sure connections
#define invert_pot 0  // are consistent.

// options for display, debug and logging data via serial com
#define telemetry 1   // 1 - send telemetry for debugging
#define LOGGER    1   // 1 - send log data. This disables the telemetry
#define DELTA_T_TELEMETRY 250  // Delta time (ms) for displaying telemetry
#define DELTA_LCD_REFRESH 150  // Delta time (ms) for refreshing the LCD

// --- clinical ---------------------------------------------------------------
// In the original code, the following two parameters are different (50 and 33)
#define perc_of_lower_volume      30  // % of max press - defines lower volume
#define perc_of_lower_vol_display 30  // % of max press - defines lower volume to display when reaching the real lower volume

#define wait_time_after_resistance 3    // seconds to wait before re-attempt to push air after max pressure was achieved 
#define max_pres_disconnected 10        // if the max pressure during breathing cycle does not reach this value - pipe is disconnected
#define insp_pressure_default 40        // default value - hold this pressure while breathing - the value is changed if INSP_Pressure potentiometer is inatalled 
#define safety_pres_above_insp 10       // defines safety pressure as the inspirium pressure + this one
#define safety_pressure 70              // quickly pullback arm when reaching this pressure in cm H2O
#define speed_multiplier_reverse 2      // factor of speed for releasing the pressure (runs motion in reverse at X this speed
#define motion_time_default 35          // motion time in 100 mSec 35 = 3500 mSec
#define patient_triggered_breath_def 1  // 1 = trigger new breath in case of patient inhale during the PEEP plateu 
#define delta_pres_patient_inhale 5     // in cmH2O
#define alpha_pres 0.98                 // used to average the pressure during the PEEP plateau

// ------------- Pin definitions ----------------------------------------------
#define pin_TST          4  // TEST button
#define pin_SW2          5  // START/STOP button
#define pin_RST          6  // RESET button
#define pin_LED_USR      7  // User LED
#define pin_BUZZER      A0  // BUZZER output

#define pin_TEMP_SENSOR  8  // Check this pin

#define pin_EVALV1       9  // Electro-valve 1
#define pin_EVALV2      10  // Electro-valve 2
#define pin_EVALV1_STAT  8  // Electro-valve 1 status
#define pin_EVALV2_STAT 13  // Electro-valve 2 status

// Pins for Motor Driver
#define pin_PWM    3  // digital pin that sends the PWM to the motor
#define pin_INA   11  // for INA of the driver
#define pin_INB   12  // for INB of the driver

#define pin_POT   A6  // Analog pin for motion feedback potentiometer (motor)
#define pin_POT2  A7  // Analog pin for motion feedback potentiometer (arm)
#define pin_AMP   A1  // Analog pin for amplitude potentiometer control
#define pin_FRQ   A2  // Analog pin for rate potentiometer control
#define pin_PRE   A3  // Analog pin for pressure potentiometer control

#define pin_CONFIG 2 // For factory reset and configuration switch

// --- Default parameter values -----------------------------------------------
#define MIN_ARM_POS_DEF 150  // Default minimum arm position
#define MAX_ARM_POS_DEF 550  // Default maximum arm position

// Controller parameters (min, max, default and delta for calibration)
#define FF_MIN 0.05
#define FF_MAX 3
#define FF_DEF 1            // motion control feed forward. 0.6, 4.5
#define DELTA_FF ((FF_MAX-FF_MIN)/100.0)

#define KP_MIN 0.05
#define KP_MAX 1.5
#define KP_DEF 0.2          // motion control propportional gain 0.2, 1.2
#define DELTA_KP ((KP_MAX-KP_MIN)/100.0)

#define KI_MIN 0.1
#define KI_MAX 7
#define KI_DEF 3            // motion control integral gain 2, 7
#define DELTA_KI ((KI_MAX-KI_MIN)/100.0)

#define integral_limit    10     // Limits the integral of error. Original: 5
#define f_reduction_up_val 0.85  // To reduce feed-forward when moving up. 0.85

// Adjustment vector min and max percentages
#define ADJ_V_MIN  10
#define ADJ_V_MAX 200

// UI
#define pot_alpha 0.85  // filter the pot values

// Talon SR or SPARK controller PWM settings ("angle" for Servo library)
#define PWM_max 85
#define PWM_min (-PWM_max)

// motion control parameters
#define cycleTime        8     // milisec  originally: 10
#define alpha            0.95  // filter. Higher = stronger low pass filter
#define profile_length 250     // motion control profile length
#define motion_control_allowed_error  80  // % of range 30, 40

#define N_adj 15  // Size of adjustment vector for wanted_pos

// --- Headers ----------------------------------------------------------------
#include <EEPROM.h>
#include <Wire.h>    // Used for I2C
#include "Adafruit_MPRLS.h"
#include <LiquidCrystal_I2C.h>
#include "ArduinoUniqueID.h"

// --- Initialize sensors and LCD, if available -------------------------------
#if pressure_sensor_available == 1
Adafruit_MPRLS adafruitPress(-1, -1);  // Default values
#endif

#if LCD_available == 1
// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);
#endif

#if TempSensor_available == 1
//OneWire ourWire(pin_TEMP_SENSOR);
//DallasTemperature DS18B20(&ourWire);
#endif

// --- Motion profile parameters ----------------------------------------------
// pos byte 0...255, units: promiles of full range
// vel byte 0...255, ZERO is at 128
// profile data: press 125 points (50%) relase 125
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
      11, 10,  9,  8,  7,  6,  6,  5,  4,  3,  3,  2,  2,  1,  1,  1,  0,  0,  0,  0,
       0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
       0,  0,  0,  0,  0,  0,  0,  0,  0,  0};

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
     127,127,127,127,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,
     128,128,128,128,128,128,128,128,128,128};

// --- Logical flags ----------------------------------------------------------
// Alarms: disconnected, motion_failure, high_pressure_detected,
//         safety_pressure_detected, patient_triggered_breath,
//         in_wait, hold_breath, calibON
byte Alarms;
#define disconnected              7
#define motion_failure            6
#define high_pressure_detected    5
#define safety_pressure_detected  4
#define patient_triggered_breath  3
#define in_wait                   2
#define hold_breath               1
#define send_beep                 0

// Status: USR_status, calibrated, manual_mov_enabled, CONFIG_enabled,
//         save_cancelled, progress, send_beep, sent_LCD
byte Status;
#define USR_status                7
#define calibrated                6
#define manual_mov_enabled        5
#define CONFIG_enabled            4
#define save_cancelled            3
#define progress                  2
#define sent_LCD                  1
#define calibON                   0

// Buttons1: SW2, prev_SW2, SW2temp, SW2_pressed
//           TST, prev_TST, TSTtemp, TST_pressed
// Buttons2: RST, prev_RST, RSTtemp, RST_pressed
byte Buttons1;
#define SW2                       7   // START/STOP button status bits
#define prev_SW2                  6
#define SW2temp                   5
#define SW2_pressed               4
#define TST                       3   // TEST button status bits
#define prev_TST                  2
#define TSTtemp                   1
#define TST_pressed               0

byte Buttons2;
#define RST                       7   // RESET button status bits
#define prev_RST                  6
#define RSTtemp                   5
#define RST_pressed               4

// --- Other global variables -------------------------------------------------
byte menu_state;
byte BPM = 14, prev_BPM, failure, wanted_cycle_time;

// For the detection of push buttons
byte counter_SW2_ON, counter_SW2_OFF, counter_TST_ON, counter_TST_OFF,
     counter_RST_ON, counter_RST_OFF;

byte insp_pressure, prev_insp_pressure, safety_pressure_counter, no_fail_counter,
     motion_time, Compression_perc = 80, prev_Compression_perc,
     telemetry_option = 0, adjusting_params = 0;

byte motorPWM, index = 0, prev_index, i, cycle_number, cycles_lost,
     index_last_motion;

int A_pot, prev_A_pot, A_rate, A_comp, A_pres;
int wait_cycles;

int pressure_abs, breath_cycle_time, max_pressure = 0, prev_max_pressure = 0,
    min_pressure = 100, prev_min_pressure = 0, index_to_hold_breath,
    pressure_baseline;
int comp_pot_low = 0, comp_pot_high = 1023, rate_pot_low = 0, rate_pot_high = 1023,
    pres_pot_low = 0, pres_pot_high = 1023;

int max_wanted_pos, min_wanted_pos, max_A_pot, min_A_pot;

unsigned int max_arm_pos, min_arm_pos;
unsigned long lastSent, lastIndex, lastUSRblink, last_TST_not_pressed, lastBlue,
              start_wait, last_sent_data, last_read_pres, start_disp_pres;

float pot_rate, pot_pres, pot_comp, avg_pres;
float wanted_pos, wanted_vel_PWM, range, range_factor, profile_planned_vel,
      planned_vel, integral, error, prev_error, f_reduction_up;

float FF = FF_DEF, KP = KP_DEF, KI = KI_DEF, FF_temp, KP_temp, KI_temp;

// For the adjustment vector (wanted_pos)
byte adj_v[N_adj] = {100, 100, 100, 100, 100, 100, 100, 100,
                     100, 100, 100, 100, 100, 100, 100};
byte adj_v_temp[N_adj], adj_ind;
float adj_val;

#if TempSensor_available == 1
//float temperature;
#endif

const PROGMEM byte Comp_perc_v[N_adj] = {30, 35, 40, 45, 50, 55, 60, 65,
                                         70, 75, 80, 85, 90, 95, 100};

enum main_states:byte {STBY_STATE, BREATH_STATE, MENU_STATE};
enum main_states state;

// Initializations and configurations
void setup()
{
  pinMode(pin_PWM, OUTPUT);
  pinMode(pin_INA, OUTPUT);  // For driver VNH5019A-E
  pinMode(pin_INB, OUTPUT);  // For driver VNH5019A-E

  pinMode(pin_TST, INPUT_PULLUP);
  pinMode(pin_SW2, INPUT_PULLUP);
  pinMode(pin_RST, INPUT_PULLUP);

  pinMode(pin_CONFIG, INPUT_PULLUP);

  pinMode(pin_LED_USR, OUTPUT);
  pinMode(pin_BUZZER, OUTPUT);

  pinMode(pin_EVALV1, OUTPUT);
  pinMode(pin_EVALV1_STAT, INPUT_PULLUP);
  pinMode(pin_EVALV2, OUTPUT);
  pinMode(pin_EVALV2_STAT, INPUT_PULLUP);

  Serial.begin(115200);
  Wire.begin();

#if pressure_sensor_available == 1
  adafruitPress.begin();
  pressure_baseline = adafruitPress.readPressure();
#endif

#if LCD_available == 1
  lcd.begin();      // initialize the LCD
  lcd.backlight();  // Turn on the back-light and print a message.
  lcd.setCursor(0, 0);  lcd.print("    AmboVent    ");
  lcd.setCursor(0, 1);  lcd.print("  UVG + HUMANA  ");
  delay(2000);
#endif

#if TempSensor_available == 1

#endif

  state = STBY_STATE;

// --- Get parameters from EEPROM ---------------------------------------------
  EEPROM.get(4, min_arm_pos);     delay(20);
  EEPROM.get(8, max_arm_pos);     delay(20);
  EEPROM.get(12, comp_pot_low);   delay(20);
  EEPROM.get(16, comp_pot_high);  delay(20);
  EEPROM.get(20, rate_pot_low);   delay(20);
  EEPROM.get(24, rate_pot_high);  delay(20);
  EEPROM.get(28, pres_pot_low);   delay(20);
  EEPROM.get(32, pres_pot_high);  delay(20);

  if(min_arm_pos >= 0 && max_arm_pos < 1024 && min_arm_pos < max_arm_pos)
    bitSet(Status, calibrated);
  else
    bitClear(Status, calibrated);

  EEPROM.get(36, FF);                    delay(20);
  EEPROM.get(36 + sizeof(float), KP);    delay(20);
  EEPROM.get(36 + 2*sizeof(float), KI);  delay(20);

  if(FF < FF_MIN || FF > FF_MAX || isnan(FF) == 1)
    FF = FF_DEF;

  if(KP < KP_MIN || KP > KP_MAX || isnan(KP) == 1)
    KP = KP_DEF;

  if(KI < KI_MIN || KI > KI_MAX || isnan(KI) == 1)
    KI = KI_DEF;

  for(i = 0; i < N_adj; i++)
  {
    EEPROM.get(36 + 3*sizeof(float) + i*sizeof(byte), adj_v[i]);
    delay(20);

    if(adj_v[i] <= 0 || adj_v[i] > 200 || isnan(adj_v[i]))
      adj_v[i] = 100;
  }

// ----------------------------------------------------------------------------

// Set initial values
  bitClear(Alarms, disconnected);
  bitClear(Alarms, motion_failure);
  bitClear(Alarms, high_pressure_detected);
  bitClear(Status, calibON);
  bitClear(Status, CONFIG_enabled);   // Will be updated at the first read_IO
  bitClear(Status, save_cancelled);

  insp_pressure = insp_pressure_default;
  bitWrite(Alarms, patient_triggered_breath, patient_triggered_breath_def);
  motion_time = motion_time_default;

#if LCD_available == 1
  lcd.backlight();  // Turn on the back-light and print a message.
#endif
}

// Main loop
void loop()
{
  read_IO();    // Read status of User Interface, act accordingly

  switch(state)
  {
    case STBY_STATE:     // Standby state
      standby_func();

      // Start breathing motion if system is calibrated and START button is pressed
      if(bitRead(Buttons1, SW2_pressed) && bitRead(Status, calibrated) == 1)
      {
        state = BREATH_STATE;
        initialize_breath();
      }

      // Reset TEST button pressed flag
      if(bitRead(Buttons1, TST) == 0)
        last_TST_not_pressed = millis();

      // To go to the main menu, press the TEST button for 3 seconds
      if(millis() - last_TST_not_pressed > 3000)
      {
        LED_USR(1);  // Turn on user LED to indicate that the button can be released

        while(bitRead(Buttons1, TST) == 1 || bitRead(Buttons1, TST_pressed))
        {
          read_IO();
        }   // wait for button release

        bitClear(Status, progress);
        state = MENU_STATE;
      }

      // Reset alarms if any alarm is ON and the RESET button is pressed
      if((bitRead(Alarms,motion_failure) == 1 || bitRead(Alarms,disconnected) == 1 ||
          bitRead(Alarms, high_pressure_detected) == 1) &&
          bitRead(Buttons2, RST_pressed) == 1)
      {
        reset_failures();
#if LCD_available == 1
        display_LCD();
#endif
      }

      break;

    case BREATH_STATE:     // Breathing state: run profile
      run_profile_func();

      // Stop breathing motion if STOP button is pressed
      if(bitRead(Buttons1, SW2_pressed))
        state = STBY_STATE;

      // Reset alarms if any alarm is ON and the RESET button is pressed
      if((bitRead(Alarms,motion_failure) == 1 || bitRead(Alarms,disconnected) == 1 ||
          bitRead(Alarms, high_pressure_detected) == 1) &&
          bitRead(Buttons2, RST_pressed) == 1)
      {
        reset_failures();
#if LCD_available == 1
        display_LCD();
#endif
        state = STBY_STATE;  // go back to Standby state
      }

      break;

    case MENU_STATE:     // maintenance menu
      display_menu();

      break;
  }

  // Print telemetry if enough time has passed and LOGGER option is not active
  if(millis() - last_sent_data > DELTA_T_TELEMETRY)
  { 
    if(telemetry == 1 && LOGGER == 0)
      print_tele();

    last_sent_data = millis();
  }
}

// Display menu function. Checks the value of the 2nd potentiometer, and
// displays the corresponding option
void display_menu()
{
  menu_state = map(pot_rate, 0, 1023, 0, 11);  // map the pot value and restrict
  menu_state = constrain(menu_state, 0, 11);   // the value to the # of options

  switch(menu_state)
  {
    case 1:   // move arm down once by pressing TEST
      if(bitRead(Status, progress) == 0)
      {
#if LANGUAGE == 0
        display_text_2_lines("Press TEST to", "run one breath");
#else
        display_text_2_lines("Pres. TEST para", "correr 1 ciclo");
#endif

        if(bitRead(Buttons1, TST_pressed)) 
        {
          initialize_breath();
          bitSet(Status, progress);
        }
      }

      if(bitRead(Status, progress) == 1)
      {
        run_profile_func();

        if(cycle_number > 0)  // If 1 cycle is done, exit
          exit_menu();
      }

      break;


    case 2:   // Calibrate potentiometers
#if LANGUAGE == 0
      display_text_2_lines("Calibrate Pots", "TEST to start");
#else
      display_text_2_lines("Calibrar Pots", "TEST p/ iniciar");
#endif

      if(bitRead(Buttons1, TST_pressed))
      {
        calibrate_pot_range();
      // Confirmation message displayed in previous function
        exit_menu();  // Exit when done
      }

      break;


    case 3:   // Calibrate pressure sensor, if available
#if pressure_sensor_available == 1
  #if LANGUAGE == 0
      display_text_2_lines("Calib pressure", "TEST to start");
  #else
      display_text_2_lines("Calib presiOn", "TEST p/ iniciar");
  #endif

      if(bitRead(Buttons1, TST_pressed))
      {
        pressure_baseline = int(adafruitPress.readPressure());

      // Confirmation message
  #if LCD_available == 1
        lcd.clear();
        lcd.setCursor(0, 0);
    #if LANGUAGE == 0
        lcd.print("Press. calibr.");
    #else
        lcd.print("Calibr. presiOn");
    #endif
        lcd.setCursor(0, 1);
        lcd.print("Base: ");
        lcd.print(pressure_baseline);
        delay(2000);
  #endif
        exit_menu();
      }
#else
  #if LANGUAGE == 0
      display_text_2_lines("Pressure sensor", "unavailable");
  #else
      display_text_2_lines("Sensor d presiOn", "no disponible");
  #endif
#endif
      break;


    case 4:   // Calibrate arm range of movement
#if LANGUAGE == 0
      display_text_2_lines("Calibrate Arm", "TEST to start");
#else
      display_text_2_lines("Calibrar Brazo", "TEST p/ iniciar");
#endif

      if(bitRead(Buttons1, TST_pressed))
      {
        bitSet(Status, calibON);
        calibrate_arm_range();
      // Confirmation message displayed in previous function
        reset_failures();
        exit_menu();
      }

      break;


    case 5:   // Set motion profile total time
#if LANGUAGE == 0
      display_text_2_lines("Set Motion Time", "TEST to start");
#else
      display_text_2_lines("Fijar T/ciclo", "TEST p/ iniciar");
#endif

      if(bitRead(Buttons1, TST_pressed))
      {
        bitSet(Status, calibON);
        read_IO();

        while(bitRead(Buttons1, TST_pressed) == 0) // Press TEST to finish
        {
          read_IO();

          motion_time = map(pot_rate, 0, 1023, 20, 50);  // map pot position
          motion_time = constrain(motion_time, 20, 50);  // to 2 - 5 sec

          // Refresh LCD only after DELTA_LCD_REFRESH ms
          if(millis() - lastUSRblink > DELTA_LCD_REFRESH)
          {
            lastUSRblink = millis();

#if LCD_available == 1
            lcd.clear();
            lcd.setCursor(0, 0);
  #if LANGUAGE == 0
            lcd.print("Set Motion Time");
  #else
            lcd.print("Fijar T/ciclo");
  #endif
            lcd.setCursor(0, 1);
            lcd.print(int(100*motion_time));
            lcd.print(" mSec");
#endif
          }
        }

      // Confirmation message
#if LCD_available == 1
        lcd.clear();
        lcd.setCursor(0, 0);
  #if LANGUAGE == 0
        lcd.print("Motion Time Set");
  #else
        lcd.print("T/ciclo fijado");
  #endif
        lcd.setCursor(0, 1);
        lcd.print(int(100*motion_time));
        lcd.print(" mSec");
        delay(2000);
#endif
        reset_failures();  // Reset possible failures and
        exit_menu();       // exit menu.
      }

      break;


    case 6:   // Toggle sync to patient
      if(bitRead(Alarms, patient_triggered_breath) == 1)
      {
#if LANGUAGE == 0
        display_text_2_lines("Sync to patient", "ON  ");
#else
        display_text_2_lines("Sinc c/paciente", "ACTIVO");
#endif
      }

      if(bitRead(Alarms, patient_triggered_breath) == 0)
      {
#if LANGUAGE == 0
        display_text_2_lines("Sync to patient","OFF  ");
#else
        display_text_2_lines("Sinc c/paciente", "INACTIVO");
#endif
      }

      if(bitRead(Buttons1, TST_pressed))
      {
        //patient_triggered_breath = 1 - patient_triggered_breath;
        bitWrite(Alarms, patient_triggered_breath,
                                1 - bitRead(Alarms, patient_triggered_breath));
        delay(DELTA_LCD_REFRESH); // wait enough time for display to be updated

        if(bitRead(Alarms, patient_triggered_breath) == 1)
        {
#if LANGUAGE == 0
         display_text_2_lines("Sync to patient", "ON  ");
#else
         display_text_2_lines("Sinc c/paciente", "ACTIVO");
#endif
        }

        if(bitRead(Alarms, patient_triggered_breath) == 0)
        {
#if LANGUAGE == 0
          display_text_2_lines("Sync to patient", "OFF  ");
#else
          display_text_2_lines("Sinc c/paciente", "INACTIVO");
#endif
        }

        delay(1000);
        exit_menu();
      }

      break;


    case 7:   // Controller calibration, if enabled (config switch ON)
      if(bitRead(Status, CONFIG_enabled))
      {
#if LANGUAGE == 0
        display_text_2_lines("Ctrl. Calib.", "TEST to start");
#else
        display_text_2_lines("Calib de control", "TEST p/ iniciar");
#endif

        if(bitRead(Buttons1, TST_pressed))
        {
          telemetry_option = 1;  // Changes what is displayed in telemetry func
          bitSet(Status, calibON);

// First, the pots need to be moved so to match the current controller constants.
// This is done so that the motion does not change abruptly.
          align_Controller_pots();

          telemetry_option = 0;  // Change back to the default telemetry display
          adjusting_params = 1;  // Indicates controller adjustment is being done
          bitClear(Status, save_cancelled);  // Make sure cancel flag is not set

          read_IO();

          // Save the values, in case the process is cancelled
          FF_temp = FF;
          KP_temp = KP;
          KI_temp = KI;

          while(bitRead(Buttons1, TST_pressed) == 0)  // Press TEST to finish
          {
            set_Controller_constants();  // This function is to set the constants
            run_profile_func();
            read_IO();

            if(bitRead(Buttons2, RST_pressed) == 1)  // Press RESET to cancel
            {
              bitSet(Status, save_cancelled);
              break;
            }
          }

          // Make sure the motor stops after controller calibration
          wanted_vel_PWM = 0;  // Set motor vel. to zero
          set_motor_PWM(wanted_vel_PWM);

          // If not cancelled, save new constants to EEPROM
          if(bitRead(Status, save_cancelled) == 0)
          {
            delay(DELTA_LCD_REFRESH);
#if LANGUAGE == 0
            display_text_2_lines("Ctrl. Calibr.", "Completed");
#else
            display_text_2_lines("Calib de control", "Completada");
#endif

            // Save control constants to the EEPROM
            EEPROM.put(36, FF);                    delay(200);
            EEPROM.put(36 + sizeof(float), KP);    delay(200);
            EEPROM.put(36 + 2*sizeof(float), KI);  delay(2000);
          }
          else  // if the process was cancelled
          {
            delay(DELTA_LCD_REFRESH);
#if LANGUAGE == 0
            display_text_2_lines("Ctrl. Calibr.", "Cancelled");
#else
            display_text_2_lines("Calib de control", "Cancelada");
#endif

            // Put the former values back
            FF = FF_temp;
            KP = KP_temp;
            KI = KI_temp;

            delay(1000);
          }

          adjusting_params = 0;  // To indicate adjustment is not being done
          reset_failures();  // Reset possible failures
          exit_menu();       // and exit menu.
        }
      }
      else  // Controller calibration disabled (config switch OFF)
      {
#if LANGUAGE == 0
        display_text_2_lines("Ctrl Cal Disbld", "Press TEST");
#else
        display_text_2_lines("Cal Ctrl Deshab", "Presione TEST");
#endif

        if(bitRead(Buttons1, TST_pressed))  // TEST to exit
        {
          delay(100);
          exit_menu();
        }
      }

      break;


    case 8:   // Volume adj. vector calibration, if enabled (config switch ON)
      if(bitRead(Status, CONFIG_enabled))
      {
#if LANGUAGE == 0
        display_text_2_lines("Adj_v Calib.", "TEST to start");
#else
        display_text_2_lines("Calib. Adj_v", "TEST p/ iniciar");
#endif

        if(bitRead(Buttons1, TST_pressed))
        {
          telemetry_option = 2;  // Changes what is displayed in telemetry func
          adjusting_params = 2;  // Indicates vol. adjustment is being done
          bitSet(Status, calibON);  // Calibration flag
          bitClear(Status, save_cancelled);  // Make sure cancel flag is not set
          delay(100);  // give some time for TST_pressed to change back to 0

          read_IO();

          adj_v_module();  // before coming back, the motor will be stopped.
          // Confirmation message displayed in previous function

          telemetry_option = 0;  // Change back to the default telemetry display
          adjusting_params = 0;  // To indicate adjustment is not being done

          reset_failures();  // Reset possible failure
          exit_menu();       // and exit menu.
        }
      }
      else  // Volume adj. vector calibration disabled (config switch OFF)
      {
#if LANGUAGE == 0
        display_text_2_lines("Adj_v Cal Disbld", "Press TEST");
#else
        display_text_2_lines("Cal Adj_v Deshab", "Presione TEST");
#endif

        if(bitRead(Buttons1, TST_pressed))  // TEST to exit
        {
          delay(100);
          exit_menu();
        }
      }

      break;


    case 9:   // FACTORY RESET, if enabled (config switch ON)
      if(bitRead(Status, CONFIG_enabled))
      {
#if LANGUAGE == 0
        display_text_2_lines("FACTORY RESET", "TEST to reset");
#else
        display_text_2_lines("RESET de FABRICA", "Presione TEST");
#endif

        if(bitRead(Buttons1, TST_pressed))
        {
          factory_reset();
          // Confirmation message displayed in previous function
          reset_failures();  // Reset possible failure
          exit_menu();       // and exit menu.
        }
        else if(bitRead(Buttons2, RST_pressed))  // RESET button to cancel
        {
          delay(DELTA_LCD_REFRESH);
#if LANGUAGE == 0
          display_text_2_lines("FACTORY RESET", "Cancelled");
#else
        display_text_2_lines("RESET de FABRICA", "Cancelado");
#endif
          delay(1000);
          exit_menu();
        }
      }
      else  // Factory Reset disabled (config switch OFF)
      {
#if LANGUAGE == 0
        display_text_2_lines("RESET Disbld", "Press TEST");
#else
        display_text_2_lines("RESET Deshab", "Presione TEST");
#endif

        if(bitRead(Buttons1, TST_pressed))
        {
          delay(100);
          exit_menu();
        }
      }

      break;


    default:   // menu_state == 0, Exit Menu message
#if LANGUAGE == 0
      display_text_2_lines("Exit Menu", "Press TEST");
#else
      display_text_2_lines("Salir del MenU", "Presione TEST");
#endif

      if(bitRead(Buttons1, TST_pressed))
        exit_menu();

      break;
  }
}

// Clear and reset a few things before leaving the Menu State.
void exit_menu()
{
  read_IO();
  last_TST_not_pressed = millis();
  state = STBY_STATE;
  index = 0;
  bitClear(Status, calibON);
  telemetry_option = 0;

#if LCD_available == 1
  display_LCD();
#endif

  bitClear(Status, progress);
}

// Controls / performs the sequence of steps for the motor to move, ultimately
// setting the corresponding PWM.
void run_profile_func()
{
  // wanted_cycle_time: time between index changes, in ms
  if(millis() - lastIndex >= wanted_cycle_time) // do when cycle time is reached
  {
    cycles_lost = (millis() - lastIndex)/wanted_cycle_time - 1;
    cycles_lost = constrain(cycles_lost, 0, 15);  // restrict the # of cycles lost

    lastIndex = millis();  // last start of cycle time
    calculate_wanted_pos_vel();

    // check condition for motion failure (percentual error)
    if(100*abs(error)/(max_arm_pos - min_arm_pos) >
                              motion_control_allowed_error && cycle_number > 1)
      bitSet(Alarms, motion_failure);

    // run in reverse if high pressure was detected
    if(bitRead(Alarms, safety_pressure_detected))
      index -= speed_multiplier_reverse*(1+cycles_lost);

    // stop the reverse when reaching the cycle start point
    if(index < 0)
    {
      if(bitRead(Alarms, safety_pressure_detected) == 1)  // count the number
        safety_pressure_counter += 1;      // of cases reaching safety pressure

      bitClear(Alarms, safety_pressure_detected);
      wait_cycles = 100*wait_time_after_resistance;
      index = profile_length - 2;  // set index to the point of waiting 
    }

    // advance index while not waiting at the end of cycle
    if(bitRead(Alarms, in_wait) == 0)
      index += (1 + cycles_lost);

    // detect drop in pressure during PEEP plateau, trigger breath based on this
    if(bitRead(Alarms, patient_triggered_breath) == 1)
    {
      if(bitRead(Alarms, in_wait) == 1 ||
                (index > profile_length/2 && (A_pot < min_arm_pos + range/18)))
      {
        // start new breath cycle if patient tries to inhale during PEEP plateau
        if(avg_pres - pressure_abs > delta_pres_patient_inhale)
          start_new_cycle();

        // calculate the filtered pressure
        avg_pres = avg_pres*alpha_pres + (1 - alpha_pres)*float(pressure_abs);
      }
      else  // initialize the filtered pressure
      { 
        avg_pres = pressure_abs;
      }
    }

    // wait for the next cycle to begin in this point -> 2 points before the
    // last cycle index
    if(index >= (profile_length - 2))
    {
      if(bitRead(Status, sent_LCD) == 0)
      {
        bitSet(Status, sent_LCD);
#if LCD_available == 1
        display_LCD();    // update the display at the end of cycle
#endif
      }

      // breath_cycle_time: time to complete an entire breathing cycle
      if(millis() - start_wait < breath_cycle_time)
      {
        index = profile_length - 2;
        bitSet(Alarms, in_wait);  // still need to wait ...
      }
      else
        start_new_cycle();  // time has come ... start from index = 0 
    }

    blink_user_led();
  }

  calc_failure();  // See if there were any failures
  set_motor_PWM(wanted_vel_PWM);  // Set the motor PWM value
  find_min_max_pressure();  // min and max pressure values in the cycle
}

//---------------- CONTROL ----------------------------------------------------
// Function to determine the desired position and velocity. The controller
// correction is done here.
void calculate_wanted_pos_vel()
{
  byte pos_from_profile, vel_from_profile;

  // Get position and velocity from profiles, according to the current index
  pos_from_profile = pgm_read_byte_near(pos + index);
  vel_from_profile = pgm_read_byte_near(vel + index + 1);

  // range of movement in pot' readings
  range = range_factor*(max_arm_pos - min_arm_pos);

  // wanted pos in pot clicks
  wanted_pos = adj_val*float(pos_from_profile)*range/255 + min_arm_pos;
  wanted_pos = constrain(wanted_pos, 0.0, 1023.0);

  // vel in clicks per unit of time (not necessarily seconds)
  profile_planned_vel = (float(vel_from_profile) - 128.01)*range/255;

  planned_vel = profile_planned_vel;

  // Condition to hold current position
  if(bitRead(Alarms, hold_breath) == 1 &&
                                bitRead(Alarms, safety_pressure_detected) == 0)
  {
    if(wanted_pos <= float(A_pot) || index == 0)
      bitClear(Alarms, hold_breath);

    planned_vel = 0;
    integral = 0;
    wanted_pos = float(A_pot);  // hold current position
  }

  // To do the reverse in case high pressure detected
  if(bitRead(Alarms, safety_pressure_detected))
    planned_vel = -speed_multiplier_reverse*planned_vel;

  // Position error
  prev_error = error;
  error = wanted_pos - float(A_pot);

  // Integral term. Make sure it's bounded
  integral += error*float(wanted_cycle_time)/1000;

  if(integral > integral_limit)
    integral = integral_limit;

  if(integral < -integral_limit)
    integral = -integral_limit;

  // zero the integral accumulator at the beginning of cycle and movement up
  if(index < 2 || prev_error*error < 0)
    integral = 0;

  if(planned_vel < 0)
    f_reduction_up = f_reduction_up_val;
  else
    f_reduction_up = 1;  // reduce f for the movement up
 
  // Controller correction
  wanted_vel_PWM = FF*planned_vel*f_reduction_up + KP*error + KI*integral;

  // reduce speed for longer cycles
  wanted_vel_PWM = wanted_vel_PWM*float(cycleTime)/float(wanted_cycle_time);

  // To help prevent the arm from going beyond the min wanted position
  if(index > int(0.6*profile_length) && A_pot < (min_arm_pos + int(0.02*range)))
    wanted_vel_PWM = 0;

  // Terms to be sent to the logger
#if LOGGER == 1
  if(int(wanted_pos) < min_wanted_pos)
    min_wanted_pos = int(wanted_pos);

  if(int(wanted_pos) > max_wanted_pos)
    max_wanted_pos = int(wanted_pos);

  if(A_pot < min_A_pot)
    min_A_pot = A_pot;

  if(A_pot > max_A_pot)
    max_A_pot = A_pot;
#endif
}

// Function called when not running the profile.
// LED blinking
void standby_func()
{
  if(bitRead(Status, USR_status))
  {
    if(millis() - lastUSRblink > 10)
    {
      bitClear(Status, USR_status);
      lastUSRblink = millis();
      LED_USR(0);
    }
  }
  else
  {
    if(millis() - lastUSRblink > 490)
    {
      bitSet(Status, USR_status);
      lastUSRblink = millis();
      LED_USR(1);
    }
  }

  // If TEST button is pressed (but not held), move the arm for one cycle
/*  if(bitRead(Buttons1, TST_pressed))
  {
    initialize_breath();
    bitSet(Status, progress);
  }

  if(bitRead(Status, progress) == 1)
  {
    run_profile_func();

    if(cycle_number > 0)
      bitClear(Status, progress);
  }
  else
  {  */
    wanted_vel_PWM = 0;  // don't move
    set_motor_PWM(wanted_vel_PWM);
//  }

  delay(1);
}

// Function to reset variables and alarms when changing to the breathing state
// and when doing the single cycle through the main menu.
void initialize_breath()
{
  cycle_number = 0;
  start_wait = millis();
  integral = 0;
  reset_failures();
  index = 0;
  bitClear(Alarms, in_wait);
  bitClear(Alarms, high_pressure_detected);
}

// Function called every time a new cycle starts. 
void start_new_cycle()
{
  index = 0;  // beginning of the cycle

  if(cycle_number < 255)  // not really important once it goes beyond 2
    cycle_number += 1;

  // reset a few alarms
  start_wait = millis();
  bitClear(Alarms, in_wait);
  bitSet(Alarms, send_beep);  // not yet used
  bitClear(Status, sent_LCD);
  bitClear(Alarms, high_pressure_detected);

// If logging is enabled, and the system is in breathing state, send the data.
#if LOGGER == 1
  if(state == BREATH_STATE)
  {
    Serial.println(min_wanted_pos);
    Serial.println(max_wanted_pos);
    Serial.println(min_A_pot);
    Serial.println(max_A_pot);
    Serial.println(prev_min_pressure);
    Serial.println(prev_max_pressure);
    Serial.println(int(BPM));

    // Reset values for next cycle (reseting pressure vals is done somewhere else)
    min_wanted_pos = 1023;
    max_wanted_pos = 0;
    min_A_pot = 1023;
    max_A_pot = 0;
  }
#endif
}

// Maps back to 0 - 1023 interval
int range_pot(int val, int low, int high)
{
  int new_val;
  new_val = int(long(val-low)*long(1023)/(high - low));
  new_val = constrain(new_val, 0, 1023);

  return(new_val);
}

// Function to find the pressure extrema per cycle
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

  // reset the extrema at the end of the cycle
  if(index >= profile_length - 5)
  { 
    max_pressure = 0;
    min_pressure = 999;
  }
}

// Function to manage the user LED
void blink_user_led()
{
  // blink LED fast if alarm conditions are detected
  if(bitRead(Alarms, high_pressure_detected) ||
                                     bitRead(Alarms, safety_pressure_detected))
  {
    if(bitRead(Status, USR_status))
    {
      if(millis() - lastUSRblink > 20)
      {
        bitClear(Status, USR_status);
        lastUSRblink = millis();
        LED_USR(0);
      }
    }
    else
    {
      if(millis() - lastUSRblink > 80)
      {
        bitSet(Status, USR_status);
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

// Function to determine failures in the system
void calc_failure()
{
  if(prev_max_pressure < max_pres_disconnected && cycle_number > 2)
    bitSet(Alarms, disconnected);
  else
    bitClear(Alarms, disconnected);  // tube was disconnected

  // high pressure detected
  if(pressure_abs > insp_pressure && bitRead(Alarms, hold_breath) == 0 &&
                                                       profile_planned_vel > 0)
  {
    bitSet(Alarms, high_pressure_detected);
    bitSet(Alarms, hold_breath);
    index_to_hold_breath = index;
  }

  // pressure above safety levels
  if(pressure_abs > safety_pressure && profile_planned_vel > 0)
    bitSet(Alarms, safety_pressure_detected);

  if(pressure_abs > insp_pressure + safety_pres_above_insp && profile_planned_vel > 0)
    bitSet(Alarms, safety_pressure_detected);

  // No problems
  if(index == 0 && prev_index != 0 && failure == 0 &&
                                bitRead(Alarms, safety_pressure_detected) == 0)
    no_fail_counter += 1;

  if(index == 0)
    failure = 0;

  // Air/oxygen circuit disconnected
  if(bitRead(Alarms, disconnected))
    failure = 1;

  // Safety pressure detected at least once
  if(bitRead(Alarms, safety_pressure_detected) && safety_pressure_counter >= 1)
  {
    failure = 2;
    safety_pressure_counter = 1;
  }

  // Motion failure (motion error above the threshold
  if(bitRead(Alarms, motion_failure))
    failure = 3;

  // reset no fail counter if there was any failure
  if(bitRead(Alarms, disconnected) == 1 || bitRead(Alarms, motion_failure) == 1 ||
                                bitRead(Alarms, safety_pressure_detected) == 1)
  {
    no_fail_counter = 0;
  }

  if(no_fail_counter >= 3)
    safety_pressure_counter = 0;

  if(no_fail_counter >= 100)  // not really important beyond 3
    no_fail_counter = 100;

  prev_index = index;
}

// Function to display two lines in the LCD
void display_text_2_lines(char *message1, char *message2)
{
  if(millis() - lastUSRblink > DELTA_LCD_REFRESH)
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

// Function for displaying specific messages when calibrating the system
void display_text_calib(char *message)
{
#if LCD_available == 1
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(message);
  lcd.setCursor(0, 1);
  #if LANGUAGE == 0
  lcd.print("Then press TEST");
  #else
  lcd.print("Luego pres. TEST");
  #endif
#endif
}

// Specific display when calibrating the arm
void display_pot_during_calib()
{
  if(millis() - lastUSRblink > DELTA_LCD_REFRESH)
  {
#if LCD_available == 1
    lcd.setCursor(13, 0);
    lcd.print(A_pot);
    lcd.print(" ");
#endif
    lastUSRblink = millis();
  }
}

// Function for calibration of motion range.
// IMPORTANT: it is assumed that the "Upper position" of the arm corresponds to
//            the lowest feedback potentiometer value, and the "Lower position"
//            of the arm corresponds to the highest pot value.
void calibrate_arm_range()   // used for calibration of motion range
{
  LED_USR(1);
  bitSet(Status, calibON);
  bitClear(Status, progress);

#if LANGUAGE == 0
  display_text_calib("Set Upper");
#else
  display_text_calib("Ajus. Sup");
#endif

  while(bitRead(Status, progress) == 0)
    internal_arm_calib_step();  // step 1 - calibrate top position

  bitClear(Status, progress);
  min_arm_pos = A_pot;

#if LANGUAGE == 0
  display_text_calib("Set Lower");
#else
  display_text_calib("Ajus. Inf");
#endif

  while(bitRead(Status, progress) == 0)
    internal_arm_calib_step();  // step 2 - calibrate bottom position

  bitClear(Status, progress);
  max_arm_pos = A_pot + COMP_PUSHBACK;  // To compensate for Ambu pushing back.
  max_arm_pos = constrain(A_pot,0, 1023);

#if LANGUAGE == 0
  display_text_calib("Move to Safe");
#else
  display_text_calib("Pos de seg.");
#endif

  while(bitRead(Status, progress) == 0)
    internal_arm_calib_step();  // step 3 - manual control for positioning back
                                //          in safe location 

#if LCD_available == 1
  lcd.clear();
  lcd.setCursor(0, 0);
  #if LANGUAGE == 0
  lcd.print("Arm Calibrated");
  lcd.setCursor(0, 1);
  lcd.print("U: ");
  lcd.print(min_arm_pos);
  lcd.print(", L: ");
  #else
  lcd.print("Brazo Calibrado");
  lcd.setCursor(0, 1);
  lcd.print("S: ");
  lcd.print(min_arm_pos);
  lcd.print(", I: ");
  #endif
  lcd.print(max_arm_pos);
#endif

  // Save the min and max positions to the EEPROM
  EEPROM.put(4, min_arm_pos);
  delay(200);
  EEPROM.put(8, max_arm_pos);
  delay(2000);

  bitSet(Status, calibrated);
}

// Function that allows moving the arm down or up with the 2nd and 3rd buttons,
// respectively. The progress flag is set if the TEST button is pressed, which
// is the condition to advancing in the calibrate_arm_range function.
void internal_arm_calib_step()
{
  read_IO();

  if(bitRead(Buttons1, TST_pressed))
    bitSet(Status, progress);  // with this, the calibrate_arm_range can move on

  if(bitRead(Buttons1, SW2temp))  // Move a little in the positive direction
  {
    digitalWrite(pin_INA, HIGH);
    digitalWrite(pin_INB, LOW);

    motorPWM = 100;  // determines how aggressive is the motion downwards (0, 255] // 100
    analogWrite(pin_PWM, motorPWM);
    delay(50); // 100
    analogWrite(pin_PWM, 0);
  }

  if(bitRead(Buttons2, RSTtemp))  // Move a little in the negative direction
  {
    digitalWrite(pin_INA, LOW);
    digitalWrite(pin_INB, HIGH);

    motorPWM = 50;  // determines how aggressive is the motion upwards (0, 255]
    analogWrite(pin_PWM, motorPWM);
    delay(50); // 100
    analogWrite(pin_PWM, 0);
  }

  display_pot_during_calib();
  delay(3);
}

// Function used for calibration of potentiometers
void calibrate_pot_range()
{ 
  LED_USR(1);

  read_IO();
#if LANGUAGE == 0
  display_text_calib("Pots to left pos");
#else
  display_text_calib("Pots a pos izq");
#endif

  // Step 1 - calibrate top position. When ready, press the TEST button.
  while(bitRead(Buttons1, TST_pressed) == 0)
    read_IO();

  comp_pot_low = analogRead(pin_AMP);
  rate_pot_low = analogRead(pin_FRQ);
  pres_pot_low = analogRead(pin_PRE);

  read_IO();
#if LANGUAGE == 0
  display_text_calib("Pots to right pos");
#else
  display_text_calib("Pots a pos der");
#endif

  // Step 2 - calibrate bottom position. When ready, press the TEST button.
  while(bitRead(Buttons1, TST_pressed) == 0)
    read_IO();

  comp_pot_high = analogRead(pin_AMP);
  rate_pot_high = analogRead(pin_FRQ);
  pres_pot_high = analogRead(pin_PRE);

  delay(DELTA_LCD_REFRESH);
#if LANGUAGE == 0
  display_text_2_lines("Potentiometers", "Calibrated");
#else
  display_text_2_lines("PotenciOmetros", "Calibrados");
#endif

  // Save the pots' min and max values to the EEPROM.
  EEPROM.put(12, comp_pot_low);   delay(200);
  EEPROM.put(16, comp_pot_high);  delay(200);
  EEPROM.put(20, rate_pot_low);   delay(200);
  EEPROM.put(24, rate_pot_high);  delay(200);
  EEPROM.put(28, pres_pot_low);   delay(200);
  EEPROM.put(32, pres_pot_high);  delay(200);
}

// Used to send data to the LCD when not in calibration nor in menu state.
#if LCD_available == 1
void display_LCD()
{
  if(bitRead(Status, calibON) == 0 && state != MENU_STATE) 
  {
    lcd.clear();
    lcd.setCursor(0, 0);
  #if LANGUAGE == 0
    lcd.print("Dep:");
    lcd.print(byte(Compression_perc));
    lcd.print("%");
    lcd.print("  BPM:");
  #else
    lcd.print("Comp:");
    lcd.print(byte(Compression_perc));
    lcd.print("%");
    lcd.print(" RPM:");
  #endif
    lcd.print(byte(BPM));

    lcd.setCursor(0, 1);

    // If no failure has been detected, display the pressure values.
    if(failure == 0)
    {
      if(millis() - start_disp_pres < 2000)
      {
        //lcd.setCursor(0, 1);
  #if LANGUAGE == 0
        lcd.print("Insp. Press.:");
  #else
        lcd.print("Pres. de Insp:");
  #endif
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

    // If there are failures, display corresponding message.
    if(failure == 1)
    {
  #if LANGUAGE == 0
      lcd.print("Pipe Disconnect");
  #else
      lcd.print("Tubo Desconect.");
  #endif
    }

    if(failure == 2)
    {
  #if LANGUAGE == 0
      lcd.print("High Pressure");
  #else
      lcd.print("PresiOn Alta");
  #endif
    }

    if(failure == 3)
    {
  #if LANGUAGE == 0
      lcd.print("Motion Fail");
  #else
      lcd.print("Falla de Movim.");
  #endif
    }
  }
}
#endif

// Function used to reset failure flags.
void reset_failures()
{
  bitClear(Alarms, motion_failure);
  index_last_motion = index;
  failure = 0;
}

// Function to set the motor PWM.
// Input: the wanted velocity calculated in the calculate_wanted_pos_vel
//        function and in other parts of the program.
void set_motor_PWM(float wanted_vel_PWM)
{
  if(abs(A_pot - prev_A_pot) > 0 || abs(wanted_vel_PWM) < 15)
    index_last_motion = index;

  if(invert_mot)
    wanted_vel_PWM = -wanted_vel_PWM;

  // Don't move if there was a motion failure while not in calibration.
  if(bitRead(Alarms, motion_failure) == 1 && bitRead(Status, calibON) == 0)
    wanted_vel_PWM = 0;

  if(wanted_vel_PWM > 0)
    wanted_vel_PWM += 3;  // undo controller dead band

  if(wanted_vel_PWM < 0)
    wanted_vel_PWM -= 3;  // undo controller dead band

  if(wanted_vel_PWM > PWM_max)
    wanted_vel_PWM = PWM_max;  // limit PWM

  if(wanted_vel_PWM < PWM_min)
    wanted_vel_PWM = PWM_min;  // limit PWM

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

  // Map value between 0 and 255 and send the PWM.
  motorPWM = (byte)(wanted_vel_PWM*255.0/PWM_max);
  analogWrite(pin_PWM, motorPWM);
}

// Save some variables and flags to their "previous" counterparts. This is
// needed for certain conditions checked in different parts of the program.
void store_prev_values()
{
  bitWrite(Buttons1, prev_SW2, bitRead(Buttons1, SW2));
  bitWrite(Buttons1, prev_TST, bitRead(Buttons1, TST));
  bitWrite(Buttons2, prev_RST, bitRead(Buttons2, RST));

  prev_BPM = BPM;
  prev_A_pot = A_pot;
  prev_Compression_perc = Compression_perc;
}

// The program stays in this function until the potentiometers are placed so
// that the corresponding temporary controller values are close to the current
// values (within the margins). This is to prevent sudden changes of the
// values, given that the potentiometers could be in any position.
void align_Controller_pots()
{
#if LCD_available == 1
  lcd.clear();
  lcd.setCursor(0, 0);
  #if LANGUAGE == 0
  lcd.print("Set FF to: ");
  #else
  lcd.print("Ajus FF a: ");
  #endif
  lcd.print((float)(FF));
  delay(100);
#endif

  // Wait until the 1st pot is in a position matching the current FF parameter.
  while(abs(FF - FF_temp) > DELTA_FF)
  {
      FF_temp = (FF_MAX - FF_MIN)*1.0*(analogRead(pin_AMP) - 
                   comp_pot_low)/(1.0*(comp_pot_high - comp_pot_low)) + FF_MIN;
      FF_temp = constrain(FF_temp, FF_MIN, FF_MAX);

    if(millis() - last_sent_data > DELTA_T_TELEMETRY)
    { 
      if(telemetry == 1 && LOGGER == 0)
        print_tele();

      last_sent_data = millis();
    }

    if(millis() - lastUSRblink > DELTA_LCD_REFRESH)
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
  #if LANGUAGE == 0
  lcd.print("Set KP to: ");
  #else
  lcd.print("Ajus KP a: ");
  #endif
  lcd.print((float)(KP));
  delay(100);
#endif

  // Wait until the 2nd pot is in a position matching the current KP parameter.
  while(abs(KP - KP_temp) > DELTA_KP)
  {
      KP_temp = (KP_MAX - KP_MIN)*1.0*(analogRead(pin_FRQ) - 
                   comp_pot_low)/(1.0*(comp_pot_high - comp_pot_low)) + KP_MIN;
      KP_temp = constrain(KP_temp, KP_MIN, KP_MAX);

    if(millis() - last_sent_data > DELTA_T_TELEMETRY)
    { 
      if(telemetry == 1 && LOGGER == 0)
        print_tele();

      last_sent_data = millis();
    }

    if(millis() - lastUSRblink > DELTA_LCD_REFRESH)
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
  #if LANGUAGE == 0
  lcd.print("Set KI to: ");
  #else
  lcd.print("Ajus KI a: ");
  #endif
  lcd.print((float)(KI));
  delay(100);
#endif

  // Wait until the 3rd pot is in a position matching the current KI parameter.
  while(abs(KI - KI_temp) > DELTA_KI)
  {
      KI_temp = (KI_MAX - KI_MIN)*1.0*(analogRead(pin_PRE) - 
                   comp_pot_low)/(1.0*(comp_pot_high - comp_pot_low)) + KI_MIN;
      KI_temp = constrain(KI_temp, KI_MIN, KI_MAX);

    if(millis() - last_sent_data > DELTA_T_TELEMETRY)
    {
      if(telemetry == 1 && LOGGER == 0)
        print_tele();

      last_sent_data = millis();
    }

    if(millis() - lastUSRblink > DELTA_LCD_REFRESH)
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
  #if LANGUAGE == 0
  lcd.print("Adjust FF KP KI");
  lcd.setCursor(0, 1);
  lcd.print("TEST when done");
  #else
  lcd.print("Ajuste FF KP KI");
  lcd.setCursor(0, 1);
  lcd.print("TEST p/ terminar");
  #endif
  delay(1000);
#endif
}

// Function that reads the potentiometer values and changes the FF, KP and KI
// constants accordingly.
void set_Controller_constants()
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

  if(millis() - lastUSRblink > DELTA_LCD_REFRESH)
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

// Function called when selecting the volume vector adjustment option in the
// main menu. It goes through all the volume percentage values, waiting for the
// user to adjust the corresponding weights.
void adj_v_module()
{
  // Save the values, in case the process is cancelled
  for(i = 0; i < N_adj; i++)
    adj_v_temp[i] = adj_v[i];

  // Loops though all N_adj values
  for(i = 0; i < N_adj; i++)
  {
    Compression_perc = (byte)(pgm_read_byte_near(Comp_perc_v + i));

    read_IO();

    // Wait for the TEST button to be pressed
    while(bitRead(Buttons1, TST_pressed) == 0)
    {
      set_adj_v_values();  // in this function we set the weights
      run_profile_func();  // the arm needs to be moving

      read_IO();

      // If the RESET button is pressed, the whole calibration is cancelled.
      if(bitRead(Buttons2, RST_pressed) == 1)
      {
        bitSet(Status, save_cancelled);
        break;
      }

      // don't refresh the telemetry too often
      if(millis() - last_sent_data > DELTA_T_TELEMETRY)
      {
        if(telemetry == 1 && LOGGER == 0)
          print_tele();

        last_sent_data = millis();
      }
    }

    // exit the loop if the process was cancelled
    if(bitRead(Status, save_cancelled) == 1)
      break;

    delay(100);  // give some time for TST_pressed to change back to 0
  }

  // Make sure the motor stops after setting the adj_v values
  wanted_vel_PWM = 0;  // don't move
  set_motor_PWM(wanted_vel_PWM);

  if(bitRead(Status, save_cancelled) == 0)
  {
    delay(DELTA_LCD_REFRESH);
#if LANGUAGE == 0
    display_text_2_lines("Calib. Adj_v", "Completed");
#else
    display_text_2_lines("Calib de Adj_v", "Completada");
#endif

    // Save adjustment vector values to the EEPROM.
    for(i = 0; i < N_adj; i++)
    {
      EEPROM.put(36 + 3*sizeof(float) + i*sizeof(byte), adj_v[i]);
      delay(200);
    }
  }
  else  // if the process was cancelled
  {
    delay(DELTA_LCD_REFRESH);
#if LANGUAGE == 0
    display_text_2_lines("Adj_v Calib.", "Cancelled");
#else
    display_text_2_lines("Calib de Adj_v", "Cancelada");
#endif

    // Put the former values back.
    for(i = 0; i < N_adj; i++)
      adj_v[i] = adj_v_temp[i];

    delay(1000);
  }
}

// Function that reads the AMP potentiometer value and changes
// the current adj_v accordingly.
// Variables i and Compression_perc are being changed in adj_v_module().
void set_adj_v_values()
{
  // map the pot value to the corresponding range
  adj_v[i] = map(analogRead(pin_AMP), comp_pot_low, comp_pot_high,
                                                         ADJ_V_MIN, ADJ_V_MAX);
  adj_v[i] = constrain(adj_v[i], ADJ_V_MIN, ADJ_V_MAX);

  if(millis() - lastUSRblink > DELTA_LCD_REFRESH)
  {
    lastUSRblink = millis();
#if LCD_available == 1
    lcd.clear();
    lcd.setCursor(0, 0);
  #if LANGUAGE == 0
    lcd.print("Comp per: ");
  #else
    lcd.print("% Comp: ");
  #endif
    lcd.print(Compression_perc);
    lcd.setCursor(0, 1);
    lcd.print("adj_v %: ");
    lcd.print(adj_v[i]);
#endif
  }
}

// Function to reset all the parameters to the default ones. This function is
// only available if the Config switch is in the ON position.
void factory_reset()
{
  byte res_confirm;

#if LCD_available == 1
  lcd.clear();
  lcd.setCursor(0, 0);
  #if LANGUAGE == 0
  lcd.print("To Confirm: TEST");
  lcd.setCursor(0, 1);
  lcd.print("To Cancel: RST");
  #else
  lcd.print("Confirmar: TEST");
  lcd.setCursor(0, 1);
  lcd.print("Cancelar: RST");
  #endif
#endif

  delay(500);  // Give time for buttons to be released

  while(1)  // wait until confirmation or cancellation
  {
    read_IO();

    // Confirm with the TEST button, cancel with the RESET button.
    if(bitRead(Buttons1, TST_pressed))
    {
      res_confirm = 1;
      break;
    }
    else if(bitRead(Buttons2, RST_pressed))
    {
      res_confirm = 0;
      break;
    }
  }

#if LCD_available == 1
    lcd.clear();
    lcd.setCursor(0, 0);
  #if LANGUAGE == 0
    lcd.print("FACTORY RESET");
  #else
    lcd.print("RESET de FABRICA");
  #endif
#endif

  if(res_confirm == 1)
  {
#if LCD_available == 1
    lcd.setCursor(0, 1);
  #if LANGUAGE == 0
    lcd.print("Confirmed...");
  #else
    lcd.print("Confirmado...");
  #endif
#endif

    // If the factory reset was confirmed, set the parameters to their defaults.
    min_arm_pos = MIN_ARM_POS_DEF;
    max_arm_pos = MAX_ARM_POS_DEF;
    comp_pot_low = 0;
    comp_pot_high = 1023;
    rate_pot_low = 0;
    rate_pot_high = 1023;
    pres_pot_low = 0;
    pres_pot_high = 1023;
    FF = FF_DEF;
    KP = KP_DEF;
    KI = KI_DEF;

    // Save the reset values to the EEPROM
    EEPROM.put(4, min_arm_pos);            delay(200);
    EEPROM.put(8, max_arm_pos);            delay(200);
    EEPROM.put(12, comp_pot_low);          delay(200);
    EEPROM.put(16, comp_pot_high);         delay(200);
    EEPROM.put(20, rate_pot_low);          delay(200);
    EEPROM.put(24, rate_pot_high);         delay(200);
    EEPROM.put(28, pres_pot_low);          delay(200);
    EEPROM.put(32, pres_pot_high);         delay(200);
    EEPROM.put(36, FF);                    delay(200);
    EEPROM.put(36 + sizeof(float), KP);    delay(200);
    EEPROM.put(36 + 2*sizeof(float), KI);  delay(200);

    for(i = 0; i < N_adj; i++)
    {
      adj_v[i] = 100;
      EEPROM.put(36 + 3*sizeof(float) + i*sizeof(byte), adj_v[i]);
      delay(200);
    }
  }
  else
  {
#if LCD_available == 1
    lcd.setCursor(0, 1);
  #if LANGUAGE == 0
    lcd.print("Cancelled...");
  #else
    lcd.print("Cancelado...");
  #endif
#endif
    delay(1000);
  }
}

// Function that reads the status of the User Interface (buttons, pots, etc.),
// and updates the corresponding global variables (parameters, alarms, etc).
void read_IO()
{
  store_prev_values();

// Check Configuration switch to see if Config (calibration) is enabled.
  bitWrite(Status, CONFIG_enabled, digitalRead(pin_CONFIG));

  bitWrite(Buttons1, SW2temp, 1 - digitalRead(pin_SW2));
  bitWrite(Buttons1, TSTtemp, 1 - digitalRead(pin_TST));
  bitWrite(Buttons2, RSTtemp, 1 - digitalRead(pin_RST));

// Detection of the first push button
  if(bitRead(Buttons1, TSTtemp) == 1)
  {
    counter_TST_ON += 1;

    if(counter_TST_ON > 20)
    {
      bitSet(Buttons1, TST);
      counter_TST_ON = 100;  // value not really important (as long as > 20)
    }
  }
  else
    counter_TST_ON = 0;

  if(bitRead(Buttons1, TSTtemp) == 0)
  {
    counter_TST_OFF += 1;

    if(counter_TST_OFF > 20)
    {
      bitClear(Buttons1, TST);
      counter_TST_OFF = 100;  // value not really important (as long as > 20)
    }
  }
  else
    counter_TST_OFF = 0;

  if(bitRead(Buttons1, TST) == 0 && bitRead(Buttons1, prev_TST) == 1)
    bitSet(Buttons1, TST_pressed);
  else
    bitClear(Buttons1, TST_pressed);

// Detection of the second push button
  if(bitRead(Buttons1, SW2temp) == 1)
  {
    counter_SW2_ON += 1;

    if(counter_SW2_ON > 20)
    {
      bitSet(Buttons1, SW2);
      counter_SW2_ON = 100;  // value not really important (as long as > 20)
    }
  }
  else
    counter_SW2_ON = 0;

  if(bitRead(Buttons1, SW2temp) == 0)
  {
    counter_SW2_OFF += 1;

    if(counter_SW2_OFF > 20)
    {
      bitClear(Buttons1, SW2);
      counter_SW2_OFF = 100;  // value not really important (as long as > 20)
    }
  }
  else
    counter_SW2_OFF = 0;

  if(bitRead(Buttons1, SW2) == 0 && bitRead(Buttons1, prev_SW2) == 1)
    bitSet(Buttons1, SW2_pressed);
  else
    bitClear(Buttons1, SW2_pressed);

// Detection of the third push button
  if(bitRead(Buttons2, RSTtemp) == 1)
  {
    counter_RST_ON += 1;

    if(counter_RST_ON > 20)
    {
      bitSet(Buttons2, RST);
      counter_RST_ON = 100;  // value not really important (as long as > 20)
    }
  }
  else
    counter_RST_ON = 0;

  if(bitRead(Buttons2, RSTtemp) == 0)
  {
    counter_RST_OFF += 1;

    if(counter_RST_OFF > 20)
    {
      bitClear(Buttons2, RST);
      counter_RST_OFF = 100;  // value not really important (as long as > 20)
    }
  }
  else
    counter_RST_OFF = 0;

  if(bitRead(Buttons2, RST) == 0 && bitRead(Buttons2, prev_RST) == 1)
    bitSet(Buttons2, RST_pressed);
  else
    bitClear(Buttons2, RST_pressed);

  if(invert_pot)
    A_pot = 1023 - analogRead(pin_POT);
  else
    A_pot = analogRead(pin_POT);

  // When adjusting Controller values, or adj_v values, don't use pot values
  // to adjust BPM, Compression_perc and insp_pressure.
  if(adjusting_params != 1)  // NO controller adjustment being done
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
  else  // Adjusting Controller parameters
  {
    A_rate = 600;  // BPM approx. 20 (800 if BPM range is 6-24, 600 if 6-30)
    A_comp = 615;  // Compression_perc approx. 90
    A_pres = 500;  // insp_pressure = 50
  }

  if(adjusting_params != 2)  // Adj_v NOT being calibrated
  {
    Compression_perc = perc_of_lower_vol_display +
                    byte(float(A_comp)*(100 - perc_of_lower_vol_display)/1023);
    Compression_perc = constrain(Compression_perc, perc_of_lower_vol_display, 100);
  }

//  BPM = 6 + (A_rate - 23)/55;       // 0 is 6 breaths p/min, 1023 is 24 BPM
  BPM = round(6.0 + 24.0*A_rate/1023);  // 0 is 6 breaths p/min, 1023 is 30 BPM
  breath_cycle_time = 60000/BPM + 100;  // in millisec. Why the additional 100?

  insp_pressure = 30 + A_pres/25;       // 0 is 30 mBar, 1023 is 70 mBar
  insp_pressure = constrain(insp_pressure, 30, 70);

  // check if pressure change is at least 1
  if(abs(insp_pressure - prev_insp_pressure) > 1)
  {
    prev_insp_pressure = insp_pressure;
    start_disp_pres = millis();

#if LCD_available == 1
    display_LCD();
#endif
  }

  // This factor affects the arm movement.
  range_factor = 1.0*perc_of_lower_volume +
                1.0*(Compression_perc - perc_of_lower_vol_display)*
                (100 - perc_of_lower_volume)/(100 - perc_of_lower_vol_display);
  range_factor = range_factor/100.0;

  // limit the factor, in case it went beyond the 0-100% range
  if(range_factor > 1.0)
    range_factor = 1.0;

  if(range_factor < 0)
    range_factor = 0;

// Adjust the wanted_pos vector, according to the adj_v values, which
// are calibrated to match the actual volumes corresponding to the
// compression_perc.
  adj_ind = map(Compression_perc, byte(perc_of_lower_volume), 100, 0, N_adj-1);
  adj_ind = constrain(adj_ind, 0, N_adj-1);

// If the Compression_perc matches one of the predefined (calibrated percentages),
// use the corresponding adj_v. If not, interpolate.
  if(Compression_perc == pgm_read_byte_near(Comp_perc_v + adj_ind))
    adj_val = adj_v[adj_ind]/100.0;
  else if(Compression_perc > pgm_read_byte_near(Comp_perc_v + adj_ind))
  {
    adj_val = (1.0*(Compression_perc - pgm_read_byte_near(Comp_perc_v + adj_ind))*
              (adj_v[adj_ind + 1] - adj_v[adj_ind])/
              (pgm_read_byte_near(Comp_perc_v + adj_ind + 1) - 
               pgm_read_byte_near(Comp_perc_v + adj_ind)) + 1.0*adj_v[adj_ind])/100.0;
  }
  else
  {
    adj_val = (1.0*(Compression_perc - pgm_read_byte_near(Comp_perc_v + adj_ind - 1))*
              (adj_v[adj_ind] - adj_v[adj_ind - 1])/
              (pgm_read_byte_near(Comp_perc_v + adj_ind) - 
               pgm_read_byte_near(Comp_perc_v + adj_ind - 1)) + 1.0*adj_v[adj_ind - 1])/100.0;
  }

#if pressure_sensor_available == 1
  // Don't read the pressure sensor too often
  if(millis() - last_read_pres > 100)
  {
    last_read_pres = millis();

    pressure_abs = int(adafruitPress.readPressure() - pressure_baseline);

    if(pressure_abs < 0)
      pressure_abs = 0;
  }
#endif

#if LCD_available == 1
  if(prev_BPM != BPM || prev_Compression_perc != Compression_perc)
    display_LCD();
#endif

  // Set the time between each step of the motion profile
  wanted_cycle_time = int(100)*int(motion_time)/profile_length;

  if(wanted_cycle_time > breath_cycle_time/profile_length)
    wanted_cycle_time = breath_cycle_time/profile_length;

  if(wanted_cycle_time < cycleTime)
    wanted_cycle_time = cycleTime;  // min cycleTime was #defined above

// Cond. for buzzing: Not in logging mode, pressure failure, not in calibration
#if LOGGER == 0 && ALARM_ENABLED == 1
  if(failure == 1 && bitRead(Status, calibON) == 0)
    Buzzer(1);
  else
    Buzzer(0);
#endif

#if TempSensor_available == 1
  // If a temperature sensor is added to the system.
#endif
}

// To turn ON or OFF the user LED
void LED_USR(byte val)
{
  if(INVERT_LEDS)
    digitalWrite(pin_LED_USR, 1 - val);
  else
    digitalWrite(pin_LED_USR, val);
}

#if LOGGER == 0 && ALARM_ENABLED == 1
// To turn ON or OFF the buzzer
void Buzzer(byte val)
{
  if(INVERT_BUZZER)
    digitalWrite(pin_BUZZER, 1 - val);
  else
    digitalWrite(pin_BUZZER, val);
}
#endif

// Function to print the telemetry information. Comment/uncomment as desired.
void print_tele()
{
  if(telemetry_option == 0)
  {
    Serial.print("State: "); Serial.print(state);
    Serial.print(", Fail (disc, motion, hiPres): ");
    Serial.print(bitRead(Alarms, disconnected));
    Serial.print(", ");
    Serial.print(bitRead(Alarms, motion_failure));
    Serial.print(", ");
    Serial.print(bitRead(Alarms, high_pressure_detected));

    // Serial.print(" CL:");
    // Serial.print(cycles_lost);

    Serial.print("; (min_arm_pos, max_arm_pos):");
    Serial.print(min_arm_pos);
    Serial.print(", ");
    Serial.println(max_arm_pos);

    // Serial.print("Vol pot: ");           Serial.print(A_comp);
    // Serial.print(", BPM pot:");          Serial.print(A_rate);
    // Serial.print(", Pressure pot:");     Serial.print(A_pres);
    // Serial.print(", w cyc t:");          Serial.print(wanted_cycle_time);
    // Serial.println("");

#if pressure_sensor_available == 1
    Serial.print("Curr. Press.: ");      Serial.print(adafruitPress.readPressure());
    Serial.print(", P. baseline: ");     Serial.print(pressure_baseline);
    Serial.print(", pressure_abs: ");    Serial.print(pressure_abs);
#endif

#if TempSensor_available == 1
//  Serial.print(", Temp.: ");  Serial.print(temperature);  Serial.println(" °C");
#endif

    Serial.println("");

    Serial.print("RF: ");                Serial.print(range_factor); 
    Serial.print(", Index: ");           Serial.print(index);
    Serial.print(", Feedback: ");        Serial.print(A_pot);
    Serial.print(", wanted_pos: ");      Serial.print(wanted_pos);
    Serial.print(", error: ");           Serial.print(error);
    Serial.print(", % error: ");         Serial.print(100*abs(error)/(max_arm_pos - min_arm_pos));
//    Serial.print(", integral: ");        Serial.print(integral);
    Serial.println("");

    Serial.print("planned_vel: ");       Serial.print(planned_vel);
    Serial.print(", wanted_vel_PWM: ");  Serial.print(wanted_vel_PWM);
    Serial.print(", motorPWM: ");        Serial.print(motorPWM);
    Serial.print(", Calibrated: ");      Serial.print(bitRead(Status, calibrated));
    Serial.println("");

    Serial.print("FF: ");                Serial.print(FF);
    Serial.print(", KP: ");              Serial.print(KP);
    Serial.print(", KI: ");              Serial.print(KI);
    Serial.println("");

    Serial.print("Compression_perc: ");  Serial.print(Compression_perc);
    Serial.print(", adj_ind: ");         Serial.print(adj_ind);
    Serial.print(", adj_val: ");         Serial.print(adj_val);
    Serial.println("");
  }

// Controller Calibration
  if(telemetry_option == 1)
  {
    Serial.print("Current FF: ");        Serial.print(FF);
    Serial.print(", FF_temp: ");         Serial.println(FF_temp);
    Serial.print("Current KP: ");        Serial.print(KP);
    Serial.print(", KP_temp: ");         Serial.println(KP_temp);
    Serial.print("Current KI: ");        Serial.print(KI);
    Serial.print(", KI_temp: ");         Serial.println(KI_temp);
  }

// Adjustment_vector Calibration
  if(telemetry_option == 2)
  {
    Serial.print("i: ");                   Serial.print(i);
    Serial.print(", Compression_perc: ");  Serial.print(Compression_perc);
    Serial.print(", adj_v[i] %: ");        Serial.println(adj_v[i]);
    Serial.print("BPM: ");                 Serial.print(BPM);
    Serial.print(", insp_pressure: ");     Serial.print(insp_pressure);
  }

  Serial.println("");
}
