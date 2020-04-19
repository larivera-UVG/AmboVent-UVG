/*
 AmboVent-UVG
 prueba_Control.ino
 Para verificar la función calculate_wanted_pos_vel()
 Based on the original code for the AmboVent (April 12, 2020)
*/

#define DEBUG 1     // 1 para que forzar a |error| < ERROR_DEBUG
#define ERROR_DEBUG 25

#define cycleTime 10        // milisec
#define FF 0.6              // motion control feed forward. 0.6,  4.5
#define KP 0.2              // motion control propportional gain 0.2, 1.2
#define KI 2                // motion control integral gain 2, 7
#define integral_limit 5    // limits the integral of error 
#define f_reduction_up_val 0.85  // reduce feedforward by this factor when moving up
#define motion_control_allowed_error  30  // % of range
#define profile_length 250    // motion control profile length

#define PWM_mid 93  // mid value for PWM 0 motion - higher pushes up
#define PWM_max 85
#define PWM_min (-PWM_max)

#define DELTA_TELE_MONITOR 23 // Para que no se despliegue tantas veces, y no siempre
                              // se desplieguen los mismos índices.
#define pin_POT A0   // analog pin of motion feedback potentiometer

#define invert_mot 0

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

// --------- PARÁMETROS PARA PROBAR -------------------------------------------
// 0 <= min_arm_pos < max_arm_pos <= 1023
unsigned int min_arm_pos = 8, max_arm_pos = 1023;

float range_factor = 1.0;     // entre 0.0 y 1.0

byte wanted_cycle_time = 10;  // entre 10 y 20
// ----------------------------------------------------------------------------

byte motion_failure = 0;
float wanted_pos, wanted_vel_PWM, range, profile_planned_vel,
      planned_vel, integral, error, prev_error, f_reduction_up;
int index = 0, A_pot, motorPWM;
unsigned long lastTele = 0, lastIndex = 0;


void setup()
{
  Serial.begin(115200);
#if DEBUG == 1   
  randomSeed(analogRead(0));
#endif
}

void loop()
{
  A_pot = analogRead(pin_POT);

  if(millis() - lastIndex >= wanted_cycle_time)  // wait for the cycle time
  {
    lastIndex = millis();  // last start of cycle time
    calculate_wanted_pos_vel();

    if(100*abs(error)/(max_arm_pos - min_arm_pos) > motion_control_allowed_error)
      motion_failure = 1;
    else
      motion_failure = 0;

    if(millis() - lastTele >= DELTA_TELE_MONITOR)  // esperar para mostrar telemetry
    {
      lastTele = millis();  // last time telemetry was displayed
      print_tele();
    }

    index = (index + 1)%profile_length;
  }

  set_motor_PWM(wanted_vel_PWM);
}

void calculate_wanted_pos_vel()
{
  byte pos_from_profile, vel_from_profile;

  pos_from_profile = pgm_read_byte_near(pos + index);
  vel_from_profile = pgm_read_byte_near(vel + index + 1);

  range = range_factor*(max_arm_pos - min_arm_pos);  // range of movement in pot' readings
  wanted_pos = float(pos_from_profile)*range/255 + min_arm_pos;  // wanted pos in pot clicks
  profile_planned_vel = (float(vel_from_profile) - 128.01)*range/255;  // in clicks per 0.2 second

  planned_vel = profile_planned_vel;

/*
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
*/

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

void set_motor_PWM(float wanted_vel_PWM)
{
// Se quitaron muchas cosas de la función en AmboVent-UVG
  if(invert_mot)
    wanted_vel_PWM = -wanted_vel_PWM;

  if(wanted_vel_PWM > 0)
    wanted_vel_PWM += 3;  // undo controller dead band

  if(wanted_vel_PWM < 0)
    wanted_vel_PWM -= 3;  // undo controller dead band

  if(wanted_vel_PWM > PWM_max)
    wanted_vel_PWM = PWM_max;  // limit PWM

  if(wanted_vel_PWM < PWM_min)
    wanted_vel_PWM = PWM_min;  // limit PWM

  motorPWM = (int)(wanted_vel_PWM*255.0/PWM_max);  // set between 0 and 255

}


void print_tele()
{
  Serial.print("Index: ");             Serial.print(index);
  Serial.print(", Feedback: ");        Serial.print(A_pot);
  Serial.print(", wanted_pos: ");      Serial.print(wanted_pos);
  Serial.print(", error: ");           Serial.print(error);
  Serial.print(", integral: ");        Serial.println(integral);
  
  Serial.print("planned_vel: ");       Serial.print(planned_vel);
  Serial.print(", wanted_vel_PWM: ");  Serial.print(wanted_vel_PWM);
  Serial.print(", motorPWM: ");        Serial.print(motorPWM);
  Serial.print(", motion_failure: ");  Serial.println(motion_failure);

  Serial.println("");
}
