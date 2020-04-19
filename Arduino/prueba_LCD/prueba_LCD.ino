/*
 AmboVent-UVG
 prueba_LCD.ino
 Para verificar el funcionamiento del LCD y las secuencias de calibración
 Based on the original code for the AmboVent (April 12, 2020)
*/

/*  to start calibrations - first enter the maintenance setup menu by pressing TEST button for 3 seconds
 *  using the RATE potentiometer select the calibration required and press TEST to select
 *  follow instructions on screen
 *  for the Arm range calibration - use the Rate potentiometer to move the arm up/down
 */

// system configuration
#define full_configuration 1        // 1 is the default - full system.   0 is for partial system - potentiometer installed on pulley, no potentiometers, ...
#define pressure_sensor_available 1 // 1 - you have installed an I2C pressure sensor 
#define central_monitor_system 0    // 1 - send unique ID for 10 seconds upon startup, 0 - dont

// options for display and debug via serial com
#define send_to_monitor 0     // 1 = send data to monitor  0 = dont
#define telemetry 1           // 1 = send telemtry for debug
#define DELTA_TELE_MONITOR 23 // Para que no se despliegue tantas veces, y no siempre
                              // se desplieguen los mismos índices.

// UI
#define deltaUD 5       // define the value change per each button press
#define pot_alpha 0.85  // filter the pot values

// clinical
#define perc_of_lower_volume 50.0       // % of max press - defines lower volume
#define perc_of_lower_vol_display 33.0  // % of max press - defines lower volume to display when reaching the real lower volume
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
  #define LCD_available 1 
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
  #define FF 4.5              // motion control feed forward  
  #define KP 1.2              // motion control propportional gain 
  #define KI 7                // motion control integral gain 
  #define integral_limit 5    // limits the integral of error 
  #define f_reduction_up_val 0.85    // reduce feedforward by this factor when moving up 
#endif

// Other Arduino pins alocation

// Pins for Motor Driver
#define pin_PWM  3   // digital pin that sends the PWM to the motor
#define pin_INA 12   // Para el driver
#define pin_INB 11   // Para el driver

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
#define invert_mot 0
#define invert_pot 0


#include <EEPROM.h>
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


byte FD, FU, AD, AU, prev_FD, prev_FU, prev_AD, prev_AU, SW2, prev_SW2, prev_TST,
     RST, LED_status, USR_status, blueOn, calibrated = 0, calibON, numBlinkFreq,
     SW2_pressed, TST_pressed, menu_state;
byte monitor_index = 0, BPM = 14, prev_BPM, in_wait, failure, send_beep,
     wanted_cycle_time, disconnected = 0, high_pressure_detected = 0,
     motion_failure = 0, sent_LCD, hold_breath, safety_pressure_detected = 0;
byte counter_ON, counter_OFF, SW2temp, insp_pressure, prev_insp_pressure,
     safety_pressure_counter, no_fail_counter, TST, counter_TST_OFF, counter_TST_ON,
     TSTtemp, patient_triggered_breath, motion_time, progress;

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
      planned_vel, integral, error,prev_error, f_reduction_up ;

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
  lcd.setCursor(0, 0);  lcd.print("AmvoVent-UVG   ");
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

  if(min_arm_pos >= 0 && min_arm_pos < 1024 && max_arm_pos >= 0 && max_arm_pos < 1024)
    calibrated = 1;

  insp_pressure = insp_pressure_default;
  patient_triggered_breath = patient_triggered_breath_def;
  motion_time = motion_time_default;

#if LCD_available == 1
  lcd.backlight();  // Turn on the backlight and print a message.
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
        //initialize_breath();
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

      break;

    case BREATH_STATE:     // run profile
      //run_profile_func();

      if(SW2_pressed)
        state = STBY_STATE;  // stop breathing motion

      break;

    case MENU_STATE:     // maintanance menu
      display_menu();
      break;
  }
  
  if(millis() - last_sent_data > DELTA_TELE_MONITOR)
  { 
//    if(send_to_monitor == 1 && telemetry == 0)
//      send_data_to_monitor();

    if(telemetry == 1)
      print_tele();

    last_sent_data = millis();
  }
}


void display_menu()
{ 
  menu_state = map(pot_rate, 0, 1023, 0, 8);
  menu_state = constrain(menu_state, 0, 8);

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
//        run_profile_func();

        if(cycle_number > 0)
          exit_menu();
      }

      break;
 
    case 4:     // calib arm range of movement
      display_text_2_lines("Calibrate Arm", "TEST to start");

      if(TST_pressed)
      {
        calibrate_arm_range();
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

#if LCD_available == 1
  display_LCD();
#endif

  progress = 0;
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
//    initialize_breath();
    progress = 1;
  }

  if(progress == 1)
  {
//    run_profile_func();

    if(cycle_number > 0)
      progress = 0;
  }
  else
  {
    wanted_vel_PWM = 0;  // dont move
//    set_motor_PWM(wanted_vel_PWM);
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
  max_arm_pos = A_pot;

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

//  set_motor_PWM(0);

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


#if LCD_available == 1
void display_LCD()  // here function that sends data to LCD
{
  if(calibON == 0 && state != MENU_STATE) 
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("BPM:");
    lcd.print(byte(BPM));
    lcd.print("  Dep:");
    lcd.print(byte(Compression_perc));
    lcd.print("%");
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

  A_pot = analogRead(pin_POT);

  if(invert_pot)
    A_pot = 1023 - A_pot;

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

  wanted_cycle_time = int(100)*int(motion_time)/profile_length;

  if(wanted_cycle_time > breath_cycle_time/profile_length)
    wanted_cycle_time = breath_cycle_time/profile_length;

  if(wanted_cycle_time < cycleTime)
    wanted_cycle_time = cycleTime;
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
  Serial.print("State: ");              Serial.println(state);
  Serial.print("menu_state: ");         Serial.println(menu_state);
  
  Serial.print("A_freq: ");             Serial.println(A_rate);
  Serial.print("Feedback: ");           Serial.println(A_pot);
  Serial.print("A_amplitude: ");        Serial.println(A_comp);
  Serial.print("wanted_cycle_time: ");  Serial.println(wanted_cycle_time);
  Serial.print("min_arm_pos: ");        Serial.print(min_arm_pos);
  Serial.print(", max_arm_pos: ");      Serial.println(max_arm_pos);
  Serial.print("Calibrated: ");         Serial.println(calibrated);
#if(pressure_sensor_available==1)
  Serial.print("pressure baseline: ");  Serial.print(pressure_baseline);
  Serial.print(",   pressure_abs: ");   Serial.println(pressure_abs);
#endif
  Serial.println("");
}
