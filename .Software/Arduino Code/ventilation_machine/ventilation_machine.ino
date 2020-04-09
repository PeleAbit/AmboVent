/*  
 *  THIS CODE WAS WRITTEN FOR TESTING AND RUNNING A HOME MADE VENTILATION DEVICE. 
 *  IT IS NOT TESTED FOR SAFETY AND NOT APPROVED FOR USE IN ANY CLINICAL , MEDICAL OR COMERCIAL DEVICE.
 *  IT IS NOT APPROVED BY ANY REGULAOTRY AUTHORITY
 *  USE ONLY AT YOUR OWN RISK 
 */

/*  to start calibrations - first enter the maintenance setup menu by pressing TEST button for 3 seconds
 *  using the RATE potentiometer select the calibration required and press TEST to select
 *  follow instructions on screen
 *  for the Arm range calibration - use the Rate potentiometer to move the arm 
 */


/* 
------INCLUDES-------
*/ 

#include <EEPROM.h>
#include <Servo.h> 
#include <Wire.h>
#include <SparkFun_MS5803_I2C.h> 
#include <LiquidCrystal_I2C.h>
#include "ArduinoUniqueID.h"

#define TRUE 1
#define FALSE 0
/* 
------CONFIGURATION DEFINITIONS-------
*/ 
// system configuration 
#define IS_FULL_CONFIGURATION TRUE               // TRUE is the default - full system.   FALSE is for partial system - potentiometer installed on pulley, no potentiometers, ...
#define IS_PRESSURE_SENSOR_AVAILABLE TRUE        // TRUE - you have installed an I2C pressure sensor 
#define IS_CENTAL_MONITOR_SYS FALSE           // TRUE - send unique ID for 10 seconds upon startup, FALSE - dont

// options for display and debug via serial com
#define IS_SEND_TO_MONITOR TRUE    // TRUE = send data to monitor  FALSE = dont
#define IS_TELEMETRY_ENABLED FALSE          // TRUE = send telemtry for debug  ... see end of code for optional IS_TELEMETRY_ENABLED data to send (uncomment selected lines)


/* 
------HARDWARE DEFINITIONS-------
*/
#define BAUD_RATE 115200

// UI
#define BUTTON_STEP 5       // define the value chnage per each button press for the non-potentiometer version only
#define POT_EMA_ALPHA 0.85  // filter the pot values
#define NUM_CENTRAL_MONITOR_ANNOUNCMENTS 100
#define CENTRAL_MONITOR_ANNOUNCMENTS_DELAY_MSEC 100
#define POT_RATE_MAX 1023
#define POT_RATE_MIN 0
#define NUM_DISPLAY_STATES 6



// clinical 
#define MIN_VOLUME_PERCENT 50.0      // % of max press - defines lower volume
#define MIN_VOLUME_PERCENT_DISPLAY 33.0   // % of max press - defines lower volume to display when reaching the real lower volume
#define REATTEMPT_DELAY 3  // seconds to wait before re-attempt to push air after max pressure was achieved 
#define DISCONNECTED_MAX_PRES_TH 10      // if the max pressure during breathing cycle does not reach this value - pipe is disconnected
#define DEFAULT_INSP_PRESSUE 40      // defualt value - hold this pressure while breathing - the value is changed if INSP_Pressure potentiometer is inatalled 
#define SAFTEY_PRES_UPPER_MARGIN 10     // defines safety pressure as the inspirium pressure + this one
#define SAFTEY_PRES 70            // quicly pullnack arm when reaching this pressure in cm H2O
#define REVERSE_SPEED_FACTOR 2    // factor of speeed for releasing the pressure (runs motion in reverse at X this speed
#define DEFAULT_SMEAR_FACTOR 0                // % --> 0 to do all cycle in 2.5 seconds and wait for the rest 100 to "smear" the motion profile on the whole cycle time 
#define DEFAULT_IS_PATIENT_TRIGGERED_BREATH TRUE     // TRUE = trigger new breath in case of patient inhale during the PEEP plateu 
#define PATIENT_INHALE_PRES_STEP 5 // in cmH2O
#define PRES_EMA_ALPHA 0.98                  // used to average the pressure during the PEEP plateu
#define MIN_MIN_ARM_POS 0 //minimal value for g_stMcuMngr.m_MinArmPos
#define MAX_MIN_ARM_POS 1024 //maximal value for g_stMcuMngr.m_MinArmPos
#define MIN_MAX_ARM_POS 0 //minimal value for g_stMcuMngr.m_MaxArmPos
#define MAX_MAX_ARM_POS 1024 //maximal value for g_stMcuMngr.m_MaxArmPos


#if (IS_FULL_CONFIGURATION==FALSE)  // no pot for UI, feedback pot on pulley
  #define IS_LCD_AVAILABLE FALSE 
  #define IS_PRES_POT_AVAILABLE FALSE        // 1 if the system has 3 potentiometer and can control the inspirium pressure 
  #define BREATH_SWITCH_PIN 7   // breath - On / Off / cal
  #define TEST_PIN 2   // test mode - not in use
  #define LED_INTENSITY_PIN 11   // amplitude LED
  #define LED_FREQUENCY_PIN 9   // frequency LED
  #define FAILED_LED_PIN 10  // FAIL and calib blue LED
  #define USER_LED_PIN 12  // User LED
  #define FREQ_DOWN_PIN 4    // freq Down
  #define FREQ_UP_PIN 5    // freq Up
  #define INTENSITY_UP_PIN 8    // Amp Down
  #define INTENSITY_DECREASE_PIN 6    // Amp Up
  #define IS_CURRENT_SENSE TRUE  // TRUE- there is a curent sensor
  #define IS_POT_CONTROLED FALSE   // TRUE = control with potentiometers  FALSE = with push buttons
  #define FEED_FORAWRD_COEFF 0.6       // motion control feed forward  
  #define KP 0.2      // motion control propportional gain 
  #define KI 2        // motion control integral gain 
  #define MAX_INTEGRAL_ERR 6  // limits the integral of error 
  #define FEED_FORAWRD_UP_REDUCTION 0.65   // reduce feedforward by this factor when moving up 
#endif

#if (IS_FULL_CONFIGURATION==TRUE) // feedback pot on arm, potentiometers for UI 
  #define IS_LCD_AVAILABLE TRUE 
  #define IS_PRES_POT_AVAILABLE TRUE        // TRUE if the system has 3 potentiometer and can control the inspirium pressure 
  #define BREATH_SWITCH_PIN 4         // breath - On / Off / cal
  #define TEST_PIN 2         // test mode - not in use
  #define RESET_ALARM_PIN 5         // reset alarm - not in use
  #define LED_INTENSITY_PIN 13    // amplitude LED
  #define LED_FREQUENCY_PIN 13   // frequency LED
  #define FAILED_LED_PIN 10   // FAIL and calib blue LED
  #define USER_LED_PIN 9         // User LED
  #define FREQ_DOWN_PIN 13         // freq Down - not used when you have potentiometers
  #define FREQ_UP_PIN 13         // freq Up - not used when you have potentiometers
  #define INTENSITY_UP_PIN 13         // Amp Down - not used when you have potentiometers
  #define INTENSITY_DECREASE_PIN 13         // Amp Up - not used when you have potentiometers
  #define IS_CURRENT_SENSE FALSE      // FALSE no current sensor
  #define IS_POT_CONTROLED TRUE    // TRUE = control with potentiometers  FALSE = with push buttons
  #define FEED_FORAWRD_COEFF 5               // motion control feed forward  
  #define KP 2              // motion control propportional gain 
  #define KI 5              // motion control integral gain 
  #define MAX_INTEGRAL_ERR 5  // limits the integral of error 
  #define FEED_FORAWRD_UP_REDUCTION 0.85    // reduce feedforward by this factor when moving up 
#endif

// other Arduino pins alocation
#define MOTOR_PWM_PIN 3   // digital pin that sends the PWM to the motor
#define POT_PIN 0   // analog pin of motion feedback potentiometer
#define CURRENT_SENSE_PIN 1   // analog pin of current sense
#define POT_INTENSITY_PIN 2   // analog pin of amplitude potentiometer control
#define POT_FREQUENCY_PIN 3   // analog pin of rate potentiometer control
#define POT_PRES_PIN 6   // analog pin of pressure potentiometer control

// Talon SR or SPARK controller PWM settings ("angle" for Servo library) 
#define PWM_MID 93  // was 93 -   mid value for PWM 0 motion - higher pushes up
#define PWM_MAX 85
#define PWM_MIN -85
#define PWM_MAX_CURRENT_DEC_100MAmp 100 // 10 Amps

// motion control parameters
#define CYCLE_TIME_MSEC 10          // milisec
#define MOTION_EMA_ALPHA 0.95            // filter for current apatation - higher = stronger low pass filter
#define PROFILE_LEN 250    // motion control profile length 
#define MOTION_CONTROL_ERR_MARGIN_PERCENT  30  // % of range

// motor and sensor definitions
#define IS_INVERT_MOT TRUE
#define IS_INVERT_POT FALSE


#define LCD_ADDRESS 0x27
#define LCD_NUM_CHARS 16
#define LCD_NUM_LINES 2

#define MIN_ARM_POS_EEPROM_ADD 0x04
#define MAX_ARM_POS_EEPROM_ADD 0x08
#define COMP_POT_LOW_EEPROM_ADD 0x0C
#define COMP_POT_HIGH_EEPROM_ADD 0x10
#define RATE_POT_LOW_EEPROM_ADD 0x14
#define RATE_POT_HIGH_EEPROM_ADD 0x18
#define RATE_PRES_LOW_EEPROM_ADD 0x1C
#define RATE_PRES_HIGH_EEPROM_ADD 0x20
#define EEPROM_DELAY_MSEC 20


/* 
------ENUMS-------
*/
enum eSysState {
  eState_Boot,
  eState_Standby,
  eState_Operating,
  eState_Config,
  eState_Error,
};
enum eMenuState {
  eMenuState_CalibPot,
  eMenuState_CalibPres,
  eMenuState_CalibArm,
  eMenuState_ToggleSync,
  eMenuState_SetSmear,
  eMenuState_ExitMenu,
};

/* 
------STRUCTS-------
*/
struct stMcuMngr {
  eSysState m_CurrState;
  eSysState m_NextState;
  unsigned int m_MaxArmPos;
  unsigned int m_MinArmPos;
  int m_CompPotLow;
  int m_CompPotHigh;
  int m_PresPotLow;
  int m_PresPotHigh;
  int m_RatePotLow;
  int m_RatePotHigh;
  bool m_isCalibrated; 
  byte m_InspPressure;
  bool m_isPatientTriggeredBreath;
  byte m_SmearFactor;
};

/* 
------MACROS-------
*/

#define MAP_CONSTRAIN(value, fromLow, fromHigh, toLow, toHigh)  constrain(map(value, fromLow, fromHigh, toLow, toHigh), toLow, toHigh)
/* 
------GLOBAL VARIABLES-------
*/
Servo g_ServoMotor;
LiquidCrystal_I2C g_Lcd(LCD_ADDRESS, LCD_NUM_CHARS, LCD_NUM_LINES); // Set the LCD address to 0x27 for a 16 chars and 2 line display
#if (IS_PRESSURE_SENSOR_AVAILABLE==TRUE) 
  MS5803 g_PresSense(ADDRESS_HIGH); 
#endif

// Motion profile parameters 
// pos byte 0...255  units: promiles of full range
// vel int 0...255  ZERO is at 128 , units: pos change per 0.2 sec
// profile data:  press 125 points (50%) relase 125 


byte g_PosArray[PROFILE_LEN]={0,0,1,2,4,6,8,10,13,15,18,21,25,28,31,35,38,42,46,50,54,57,61,66,70,74,78,82,86,91,95,99,104,108,112,117,121,125,130,134,138,143,147,151,156,160,164,169,173,177,181,185,189,194,198,201,205,209,213,217,220,224,227,230,234,237,240,242,245,247,249,251,253,254,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,254,253,252,250,248,246,244,241,238,235,232,229,225,222,218,214,210,206,202,198,193,189,184,180,175,171,166,162,157,152,148,143,138,134,129,124,120,115,111,106,102,97,93,89,84,80,76,72,68,64,61,57,54,50,47,44,41,38,36,33,31,29,27,25,23,22,20,19,17,16,15,13,12,11,10,9,8,7,6,6,5,4,3,3,2,2,1,1,1,0,0,0,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0};
byte g_VelArray[PROFILE_LEN]={129,132,134,136,137,139,140,141,142,143,143,144,144,145,146,146,146,147,147,147,148,148,148,148,149,149,149,149,149,149,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,149,149,149,149,149,149,148,148,148,148,147,147,147,146,146,146,145,144,144,143,143,142,141,140,139,137,136,134,132,129,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,127,125,123,121,120,119,117,116,115,114,113,112,111,111,110,109,109,108,108,107,107,106,106,106,106,105,105,105,105,105,105,105,105,105,105,105,105,105,105,105,105,106,106,106,107,107,107,108,108,109,109,110,110,111,111,112,113,113,114,115,116,117,118,118,119,119,120,120,120,121,121,121,122,122,122,123,123,123,124,124,124,124,125,125,125,125,125,126,126,126,126,126,127,127,127,127,127,127,127,128,128,128,128,128,128,128,128,128,128,128,128,129,129,129,129,129,129,129,129,129,128,128,128,128,128};

stMcuMngr g_stMcuMngr;

byte FD,FU,AD,AU,FDFB,FUFB,ADFB,AUFB,SW2,SW2FB,TSTFB,RST,LED_status,USR_status,blueOn, calibON, numBlinkFreq, SW2_pressed,TST_pressed,menu_state;
int A_pot,prevA_pot, A_current, Compression_perc=80, prev_Compression_perc, A_rate, A_comp, A_pres;
int motorPWM,index=0, prev_index,i, wait_cycles,cycle_number, cycles_lost,index_last_motion;

float wanted_pos, wanted_vel_PWM, range, range_factor, profile_planned_vel, planned_vel, integral, error, f_reduction_up ;
unsigned long lastSent,lastIndex, lastUSRblink,last_TST_not_pressed,lastBlue,start_wait, last_sent_data, last_read_pres,start_disp_pres;
byte monitor_index=0, BPM=14,prev_BPM, in_wait, failure, send_beep, wanted_cycle_time, disconnected=0,high_pressure_detected=0, motion_failure=0, sent_LCD, hold_breath, SAFTEY_PRES_detected;
byte counter_ON,counter_OFF,SW2temp,insp_pressure,prev_insp_pressure, SAFTEY_PRES_counter, no_fail_counter,TST, counter_TST_OFF,counter_TST_ON,TSTtemp;
float pot_rate, pot_pres, pot_comp,avg_pres;
int pressure_abs,breath_cycle_time, max_pressure=0 , prev_max_pressure=0, min_pressure=100, prev_min_pressure=0, index_to_hold_breath,pressure_baseline;



/* 
------INIT FUNCTIONS-------
*/
void Init_GPIO()
{
  pinMode (MOTOR_PWM_PIN,OUTPUT);
  pinMode (FREQ_DOWN_PIN,INPUT_PULLUP);
  pinMode (FREQ_UP_PIN,INPUT_PULLUP);
  pinMode (INTENSITY_UP_PIN,INPUT_PULLUP);
  pinMode (INTENSITY_DECREASE_PIN,INPUT_PULLUP);
  pinMode (BREATH_SWITCH_PIN,INPUT_PULLUP);
  pinMode (TEST_PIN,INPUT_PULLUP);
  pinMode (LED_INTENSITY_PIN,OUTPUT);
  pinMode (LED_FREQUENCY_PIN,OUTPUT);
  pinMode (FAILED_LED_PIN,OUTPUT);
  pinMode (USER_LED_PIN,OUTPUT);
}

void Init_ServoMotor()
{
  g_ServoMotor.attach(MOTOR_PWM_PIN); 
}

void Init_Lcd()
{

 #if(IS_LCD_AVAILABLE == TRUE)
 {
    g_Lcd.begin();  // initialize the LCD
    g_Lcd.backlight();  // Turn on the blacklight and print a message.
    g_Lcd.setCursor(0, 0);      g_Lcd.print("AmvoVent       ");
    g_Lcd.setCursor(0, 1);      g_Lcd.print("1690.108       ");
    g_Lcd.backlight();  // Turn on the blacklight and print a message.
  }
  #endif
}

void Init_PresSense()
{
#if (IS_PRESSURE_SENSOR_AVAILABLE==1) 
  {
    g_PresSense.reset();
    g_PresSense.begin();
    pressure_baseline = int(g_PresSense.getPressure(ADC_4096));
  }
#endif
}

void Init_Comm()
{
  Serial.begin (BAUD_RATE);
  Wire.begin();
}

void Announce_ID()
{
  for (i = 0; i < NUM_CENTRAL_MONITOR_ANNOUNCMENTS; i++)
  {
    UniqueIDdump(Serial);
    delay(CENTRAL_MONITOR_ANNOUNCMENTS_DELAY_MSEC); 
  }  // for IAI monitor run for 100 cycles
}

void Init_Periph()
{
  Init_ServoMotor();
  Init_PresSense();
  Init_Lcd();
}

void Init_McuMngr()
{
  g_stMcuMngr.m_CurrState = eState_Boot;
  g_stMcuMngr.m_NextState = eState_Boot;
  g_stMcuMngr.m_MaxArmPos = 0;
  g_stMcuMngr.m_MinArmPos = 0;
  g_stMcuMngr.m_CompPotLow = 0;
  g_stMcuMngr.m_CompPotHigh = 0;
  g_stMcuMngr.m_PresPotLow = 0;
  g_stMcuMngr.m_PresPotHigh = 0;
  g_stMcuMngr.m_RatePotLow = 0;
  g_stMcuMngr.m_RatePotHigh = 0;
  g_stMcuMngr.m_InspPressure = DEFAULT_INSP_PRESSUE;
  g_stMcuMngr.m_isPatientTriggeredBreath = DEFAULT_IS_PATIENT_TRIGGERED_BREATH;
  g_stMcuMngr.m_SmearFactor = DEFAULT_SMEAR_FACTOR;
  g_stMcuMngr.m_isCalibrated = FALSE;   

}

void LoadFromEEPROM()
{
  EEPROM.get(MIN_ARM_POS_EEPROM_ADD, g_stMcuMngr.m_MinArmPos);     delay(EEPROM_DELAY_MSEC);
  EEPROM.get(MAX_ARM_POS_EEPROM_ADD, g_stMcuMngr.m_MaxArmPos);     delay(EEPROM_DELAY_MSEC);
  EEPROM.get(COMP_POT_LOW_EEPROM_ADD, g_stMcuMngr.m_CompPotLow);   delay(EEPROM_DELAY_MSEC);
  EEPROM.get(COMP_POT_HIGH_EEPROM_ADD, g_stMcuMngr.m_CompPotHigh);  delay(EEPROM_DELAY_MSEC);
  EEPROM.get(RATE_POT_LOW_EEPROM_ADD, g_stMcuMngr.m_RatePotLow);   delay(EEPROM_DELAY_MSEC);
  EEPROM.get(RATE_POT_HIGH_EEPROM_ADD, g_stMcuMngr.m_RatePotHigh);  delay(EEPROM_DELAY_MSEC);
  EEPROM.get(RATE_PRES_LOW_EEPROM_ADD, g_stMcuMngr.m_PresPotLow);   delay(EEPROM_DELAY_MSEC);
  EEPROM.get(RATE_PRES_HIGH_EEPROM_ADD, g_stMcuMngr.m_PresPotHigh);  delay(EEPROM_DELAY_MSEC);
  
  if (g_stMcuMngr.m_MinArmPos>=MIN_MIN_ARM_POS &&
  g_stMcuMngr.m_MinArmPos<MAX_MIN_ARM_POS &&
  g_stMcuMngr.m_MaxArmPos>=MIN_MAX_ARM_POS &&
  g_stMcuMngr.m_MaxArmPos<MAX_MAX_ARM_POS &&
  g_stMcuMngr.m_MaxArmPos > g_stMcuMngr.m_MinArmPos)
  {
  g_stMcuMngr.m_isCalibrated = TRUE;
  }  
}

void Init_App()
{
  Init_McuMngr();
  if (IS_CENTAL_MONITOR_SYS == TRUE)
  {
  Announce_ID();
  }
  LoadFromEEPROM();
}

/* 
------MENU AND DISPLAY-------
*/

eMenuState GetMenuState()
{
  menu_state=MAP_CONSTRAIN(pot_rate,POT_RATE_MIN,POT_RATE_MAX,0,NUM_DISPLAY_STATES);
  return (eMenuState)menu_state;
}


void display_menu()
{ 
  switch (GetMenuState()){
    case eMenuState_CalibPot:     // calib pot
  {
      display_text_2_lines("Calibrate Pots","TEST to start");
      if (TST_pressed)   
    { 
    calibrate_pot_range();
    exit_menu();
    }
  }
    break;

    case eMenuState_CalibPres:     // calib pressure sensor
  {
    display_text_2_lines("Calib pressure","TEST to start");
    if (TST_pressed)
    {
      pressure_baseline = int(g_PresSense.getPressure(ADC_4096));
      delay(100);
      exit_menu();
    }
  }
  break;
   
    case eMenuState_CalibArm:     // calib arm range of movement
  {
    display_text_2_lines("Calibrate Arm","TEST to start");
      if (TST_pressed)  
    {  
      calibrate_arm_range();
      exit_menu(); 
    }
  }

    case eMenuState_ToggleSync:     // toggle sync to patient
  {
    if (g_stMcuMngr.m_isPatientTriggeredBreath == TRUE)
    {   
      display_text_2_lines("Sync to patient","ON  ");
    }
    else
    {
      display_text_2_lines("Sync to patient","OFF  ");
    }
       if (TST_pressed)
     {
        g_stMcuMngr.m_isPatientTriggeredBreath = !g_stMcuMngr.m_isPatientTriggeredBreath;
    delay(110);
        if (g_stMcuMngr.m_isPatientTriggeredBreath == TRUE)
    {
      display_text_2_lines("Sync to patient","ON  ");
    }
    else
    {
      display_text_2_lines("Sync to patient","OFF  ");
    }
        delay (1500);
        exit_menu();
    }
  }
  break;

    case eMenuState_SetSmear:     // set motion profile smear factor
  {
    display_text_2_lines("Set Smear Factor","TEST to start ");
    if (TST_pressed)  {
      read_IO();
      while (TST_pressed==0)
      { 
        read_IO();
        g_stMcuMngr.m_SmearFactor=MAP_CONSTRAIN(pot_rate,0,1023,0,100);
        if (millis()-lastUSRblink> 100)
        {
          lastUSRblink=millis();
          g_Lcd.clear(); 
          g_Lcd.setCursor(0, 0); g_Lcd.print("Smear motion fac");  
          g_Lcd.setCursor(0, 1); g_Lcd.print(g_stMcuMngr.m_SmearFactor);
        }
      }
      delay (500);
      exit_menu();
    }
  }
  break;

    default:
  {
    display_text_2_lines("Exit Menu","Press TEST ");
    {
    if (TST_pressed)
    {
      exit_menu();
    }
    }
      
  }

    break;
  }
}

void exit_menu()
{
  SW2FB=0;
  last_TST_not_pressed= millis();
  g_stMcuMngr.m_NextState = eState_Standby;
  index=0;
  calibON = 0; 
  display_LCD();
}
void run_profile_func()
{
  if (millis()-lastIndex >= wanted_cycle_time)                        // do when cycle time was reached
     {
      cycles_lost = (millis()-lastIndex)/wanted_cycle_time-1;  
      cycles_lost = constrain (cycles_lost,0,15);
      lastIndex=millis();  // last start of cycle time

      range = range_factor*(g_stMcuMngr.m_MaxArmPos - g_stMcuMngr.m_MinArmPos);                 // range of movement in pot' readings
      wanted_pos = float(g_PosArray[index])*range/255 + g_stMcuMngr.m_MinArmPos;           // wanted pos in pot clicks
      profile_planned_vel = (float(g_VelArray[index+1]) - 128.01)*range/255;   // in clicks per 0.2 second
      if (hold_breath==1 && SAFTEY_PRES_detected==0) 
        {   if (wanted_pos <= float (A_pot) || index ==0) hold_breath=0;
            planned_vel=0; 
            integral = 0;       
            wanted_pos = float (A_pot);                                 // hold current position
        }
      else 
        { 
          planned_vel= profile_planned_vel;
        }
      if (SAFTEY_PRES_detected) planned_vel=-REVERSE_SPEED_FACTOR*planned_vel;          // to do the revese in case high pressure detected

      error = wanted_pos-float(A_pot);    
      if (100*abs(error)/(g_stMcuMngr.m_MaxArmPos - g_stMcuMngr.m_MinArmPos)>MOTION_CONTROL_ERR_MARGIN_PERCENT && cycle_number>1) motion_failure=1;
         
      integral += error*float(wanted_cycle_time)/1000;
      if (integral> MAX_INTEGRAL_ERR) integral= MAX_INTEGRAL_ERR;
      if (integral<-MAX_INTEGRAL_ERR) integral=-MAX_INTEGRAL_ERR;
      if (index<2 || index ==140 ) integral=0;   // zero the integral accumulator at the beginning of cycle and movement up

      if (planned_vel<0) f_reduction_up = FEED_FORAWRD_UP_REDUCTION; else f_reduction_up=1;  // reduce f for the movement up
  
      wanted_vel_PWM = FEED_FORAWRD_COEFF*planned_vel*f_reduction_up + KP*error+KI*integral;           // PID correction 
      wanted_vel_PWM = wanted_vel_PWM*float(CYCLE_TIME_MSEC)/float(wanted_cycle_time);      // reduce speed for longer cycles 
 
      if (SAFTEY_PRES_detected) {index-=REVERSE_SPEED_FACTOR*(1+cycles_lost);  }  // run in reverse if high pressure was detected
      if (index<0) { if(SAFTEY_PRES_detected==1) SAFTEY_PRES_counter+=1;          // count the number of cases reaching safety pressure
                     SAFTEY_PRES_detected=0; 
                     wait_cycles=200*REATTEMPT_DELAY; 
                     index=PROFILE_LEN-2;                                               // set index to the point of waiting 
                    }    // stop the reverse when reching the cycle start point

      if (in_wait==0) index +=(1+cycles_lost);   //  advance index while not waiting at the end of cycle 
      if (g_stMcuMngr.m_isPatientTriggeredBreath == TRUE)           // detect drop in presure during the PEEP plateu and trigger breath based on this
      { 
        if (in_wait==1 || (index>PROFILE_LEN/2 && (A_pot<g_stMcuMngr.m_MinArmPos+range/18))) 
        {
          if (avg_pres - pressure_abs > PATIENT_INHALE_PRES_STEP) start_new_cycle();    // start new breath cycle if patient tries to inhale durint the PEEP plateu
          avg_pres = avg_pres * PRES_EMA_ALPHA  + (1-PRES_EMA_ALPHA) * float (pressure_abs);         // calculate the filtered pressure
        }
        else { avg_pres=pressure_abs; }                                                      // initialize the filtered pressure
      }
      
      if (index >= (PROFILE_LEN-2))                  // wait for the next cycle to begin in this point -> 2 points befoe the last cycle index
        {
        if (sent_LCD==0) {sent_LCD=1; display_LCD();}   // update the display at the end of cycle
        if (millis()-start_wait < breath_cycle_time) { index = PROFILE_LEN-2; in_wait=1;   }    // still need to wait ...
        else {  start_new_cycle(); }                    // time has come ... start from index = 0 
        }
      blink_user_led ();
    }
    calc_failure();
    set_motor_PWM (wanted_vel_PWM);
    find_min_max_pressure();
}

void standby()  // not running profile
{ 
    if (USR_status) 
      {if (millis()-lastUSRblink>10) {USR_status=0; lastUSRblink=millis(); LED_USR(0);}}
      else {if (millis()-lastUSRblink>490) {USR_status=1; lastUSRblink=millis(); LED_USR(1);}}
    wanted_vel_PWM=0;    // dont move
    set_motor_PWM (wanted_vel_PWM);
    delay (1);
}

void initialize_breath()
{
    cycle_number=0;
    start_wait=millis(); 
    integral=0; 
    reset_failures();
    index=0; 
    in_wait=0; 
    high_pressure_detected=0;
}

void start_new_cycle()
{
 index =0; 
 cycle_number+=1; 
 start_wait=millis(); 
 in_wait=0; 
 send_beep=1; 
 sent_LCD=0;
 high_pressure_detected=0;
}

int range_pot (int val,int low, int high)
 {
  int new_val;
  new_val = int( long (val-low)* long(1023)/(high-low));
  new_val = constrain (new_val,0,1023);
  return (new_val);
 }
 
void find_min_max_pressure()
{
  if (max_pressure<pressure_abs) max_pressure=pressure_abs;           // find the max pressure in cycle 
  if (min_pressure>pressure_abs) min_pressure=pressure_abs;           // find the min pressure in cycle 
  if (index > PROFILE_LEN-10 && index < PROFILE_LEN-5) { prev_min_pressure = min_pressure; prev_max_pressure = max_pressure; }   
  if (index >=  PROFILE_LEN-5) { max_pressure=0;  min_pressure=999; }  
}

void blink_user_led()
{
  if (high_pressure_detected || SAFTEY_PRES_detected)   // blink LED fast
      { if (USR_status) 
        { if (millis()-lastUSRblink>20) {USR_status=0; lastUSRblink=millis(); LED_USR(0);}}
        else {if (millis()-lastUSRblink>80) {USR_status=1; lastUSRblink=millis(); LED_USR(1);}} }
  else     //  not in failure - blink LED once per cycle 
  { if (index>0.1*PROFILE_LEN) LED_USR(0); else LED_USR(1); }  
}

void calc_failure()
{
  if (prev_max_pressure < DISCONNECTED_MAX_PRES_TH && cycle_number>2) disconnected=1; else disconnected=0; // tube was disconnected
  if (pressure_abs>insp_pressure && hold_breath==0 && profile_planned_vel>0) { high_pressure_detected=1; hold_breath=1; index_to_hold_breath=index; }   // high pressure detected 
  if (pressure_abs>SAFTEY_PRES && profile_planned_vel>0) SAFTEY_PRES_detected=1;
  if (pressure_abs>insp_pressure+SAFTEY_PRES_UPPER_MARGIN && profile_planned_vel>0) SAFTEY_PRES_detected=1;
  if (index==0 && prev_index!=0 && failure==0 && SAFTEY_PRES_detected==0) no_fail_counter+=1;
  if (index==0)       failure =0;
  if (disconnected)   failure =1;   
  if (SAFTEY_PRES_detected && SAFTEY_PRES_counter>=1) { failure=2; SAFTEY_PRES_counter=1; } 
  if (motion_failure) failure =3;
  if (disconnected==1 || motion_failure==1 || SAFTEY_PRES_detected==1) {blinkBlue (1); no_fail_counter=0;}  else {LED_FAIL(0); }
  if (no_fail_counter>=3) SAFTEY_PRES_counter=0;
  if (no_fail_counter>=100) no_fail_counter=100;
  prev_index= index;
}  

void display_text_2_lines (char *message1, char *message2)
{ 
  if (millis()-lastUSRblink> 100)
  {
  lastUSRblink=millis();
  g_Lcd.clear(); 
  g_Lcd.setCursor(0, 0); g_Lcd.print(message1);  
  g_Lcd.setCursor(0, 1); g_Lcd.print(message2);
  }
}

void display_text_calib (char *message)
{
  g_Lcd.clear(); 
  g_Lcd.setCursor(0, 0); g_Lcd.print(message);  
  g_Lcd.setCursor(0, 1); g_Lcd.print("Then press Test");
}

void display_pot_during_calib()
{
      if (millis()-lastUSRblink>100) {g_Lcd.setCursor(12, 0); g_Lcd.print(A_pot); g_Lcd.print("  "); lastUSRblink=millis();}
}

void calibrate_arm_range()   // used for calibaration of motion range
{ 
  byte progress;
  LED_USR(1);   calibON = 1; 
  progress=0;  delay(30);

  display_text_calib ("Set Upper");
  while (progress==0)  // step 1 - calibrate top position
  {
    read_IO (); delay(3);
    if (TST_pressed) progress=1;
    set_motor_PWM (0);
    display_pot_during_calib();
  }
  delay(30);  progress=0;  LED_USR(0);
  read_IO (); g_stMcuMngr.m_MinArmPos=A_pot;

  display_text_calib ("Set Lower");
  while (progress==0)  // step 2 - calibrate bottom position
  {
    read_IO (); delay(3);
    if (TST_pressed) progress=1;
    set_motor_PWM (0);
    display_pot_during_calib();
  }
  delay(30);   progress=0;   LED_USR(1);
  read_IO ();   g_stMcuMngr.m_MaxArmPos=A_pot; 

  display_text_calib ("Move to Safe");
  while (progress==0)   // step 3 - manual control for positioning back in safe location 
  {
    read_IO (); delay(3);
    if (TST_pressed) progress=1;
    set_motor_PWM (0);
    display_pot_during_calib();
  }

  EEPROM.put(4, g_stMcuMngr.m_MinArmPos);  delay(200);
  EEPROM.put(8, g_stMcuMngr.m_MaxArmPos);  delay(200);
  g_stMcuMngr.m_isCalibrated=1;
}

void calibrate_pot_range()   // used for calibaration of potentiometers
{ 
  byte progress;
  LED_USR(1);   calibON = 2; 
  progress=0; TSTFB=0; delay(30);
  display_text_calib ("Pot to left pos");
  while (progress==0)  // step 1 - calibrate top position
  {
    read_IO (); delay(5);
    if (TST_pressed) progress=1;
  }
  TSTFB=0;  delay(30);  progress=0;  LED_USR(0);
  g_stMcuMngr.m_CompPotLow=analogRead (POT_INTENSITY_PIN);  g_stMcuMngr.m_RatePotLow=analogRead (POT_FREQUENCY_PIN);  g_stMcuMngr.m_PresPotLow=analogRead (POT_PRES_PIN);
    
  display_text_calib ("Pot to right pos");
  while (progress==0)  // step 2 - calibrate bottom position
  {
    read_IO (); delay(5);
    if (TST_pressed) progress=1;
  }
  TSTFB=0;  delay(30);   progress=0;   LED_USR(1);
  g_stMcuMngr.m_CompPotHigh=analogRead (POT_INTENSITY_PIN);  g_stMcuMngr.m_RatePotHigh=analogRead (POT_FREQUENCY_PIN);  g_stMcuMngr.m_PresPotHigh=analogRead (POT_PRES_PIN);

  EEPROM.put(12, g_stMcuMngr.m_CompPotLow);   delay(100);
  EEPROM.put(16, g_stMcuMngr.m_CompPotHigh);  delay(100);
  EEPROM.put(20, g_stMcuMngr.m_RatePotLow);   delay(100);
  EEPROM.put(24, g_stMcuMngr.m_RatePotHigh);  delay(100);
  EEPROM.put(28, g_stMcuMngr.m_PresPotLow);   delay(100);
  EEPROM.put(32, g_stMcuMngr.m_PresPotHigh);  delay(100);
}

void display_LCD()   // here function that sends data to LCD
{ if (IS_LCD_AVAILABLE) 
  {
  if (calibON==0 && g_stMcuMngr.m_CurrState!= eState_Operating) 
    {
    g_Lcd.clear();
    g_Lcd.setCursor(0, 0);   g_Lcd.print("BPM:");   g_Lcd.print(byte(BPM));  
    g_Lcd.print("  Dep:"); g_Lcd.print(byte(Compression_perc));  g_Lcd.print("%");
    g_Lcd.setCursor(0, 1);  
    if (failure ==0)
    {
      if (millis()- start_disp_pres<2000) { g_Lcd.setCursor(0, 1); g_Lcd.print("Insp. Press. :");  g_Lcd.print(byte(insp_pressure));}
      else {g_Lcd.print("Pmin:"); g_Lcd.print(byte(prev_min_pressure)); g_Lcd.print("  Pmax:"); g_Lcd.print(byte(prev_max_pressure));}
    }
    if (failure ==1) g_Lcd.print("Pipe Disconnect");
    if (failure ==2) g_Lcd.print("High Pressure");
    if (failure ==3) g_Lcd.print("Motion Fail");
    }   
  }
}

void reset_failures()
{
  motion_failure=0;
  index_last_motion=index;
  failure=0;
}

void set_motor_PWM (float wanted_vel_PWM)
{
  if (abs(A_pot-prevA_pot)>0 || abs(wanted_vel_PWM)<15) index_last_motion=index;
  //if (index-index_last_motion>10 || A_pot==0 || A_pot==1023) motion_failure=1;
  
  if (calibON==1 ) wanted_vel_PWM=read_motion_for_calib();  // allows manual motion during calibration
  if (IS_INVERT_MOT) wanted_vel_PWM=-wanted_vel_PWM;
  if (IS_CURRENT_SENSE) {if (A_current>PWM_MAX_CURRENT_DEC_100MAmp)   wanted_vel_PWM=0;}
  if (motion_failure==1 && calibON==0) wanted_vel_PWM=0;
  if (wanted_vel_PWM > 0) wanted_vel_PWM+=3;   // undo controller dead band
  if (wanted_vel_PWM < 0) wanted_vel_PWM-=3;   // undo controller dead band
  if (wanted_vel_PWM > PWM_MAX) wanted_vel_PWM= PWM_MAX;  // limit PWM
  if (wanted_vel_PWM < PWM_MIN) wanted_vel_PWM= PWM_MIN;  // limit PWM
  motorPWM = PWM_MID + int( wanted_vel_PWM );  
  g_ServoMotor.write(motorPWM);
}

int read_motion_for_calib()
{ int wanted_cal_PWM;
  if (IS_POT_CONTROLED)
    { 
      if (pot_rate>750) wanted_cal_PWM=(pot_rate-750)/15;
      if (pot_rate<250) wanted_cal_PWM=(pot_rate-250)/15;
      if (pot_rate>=250 && pot_rate<=750) wanted_cal_PWM=0;
      if (SW2==1) wanted_cal_PWM= -12;
      // if (RST==1) wanted_cal_PWM= 12;
    }
    else
    { wanted_cal_PWM=0;
      if (FD==1) wanted_cal_PWM = 8;  
      if (FU==1) wanted_cal_PWM = -8;   
      if (AD==1) wanted_cal_PWM = 16;
      if (AU==1) wanted_cal_PWM = -16;
    }
    return (wanted_cal_PWM);
}


void read_IO ()
{ FDFB=FD; FUFB=FU; ADFB=AD;  AUFB=AU;   SW2FB=SW2; TSTFB=TST;
  prev_Compression_perc=Compression_perc;
  prev_BPM=BPM;
  prevA_pot=A_pot;

  RST = (1-digitalRead  (RESET_ALARM_PIN));
  TSTtemp = (1-digitalRead (TEST_PIN));
  SW2temp = (1-digitalRead (BREATH_SWITCH_PIN));
  if (SW2temp==1) {counter_ON+=1;       if (counter_ON>20)      {SW2=1; counter_ON=100;     }} else counter_ON=0;
  if (SW2temp==0) {counter_OFF+=1;      if (counter_OFF>20)     {SW2=0; counter_OFF=100;    }} else counter_OFF=0;
  if (TSTtemp==1) {counter_TST_ON+=1;   if (counter_TST_ON>20)  {TST=1; counter_TST_ON=100; }} else counter_TST_ON=0;
  if (TSTtemp==0) {counter_TST_OFF+=1;  if (counter_TST_OFF>20) {TST=0; counter_TST_OFF=100;}} else counter_TST_OFF=0;
  if (SW2==0 && SW2FB==1) SW2_pressed=1; else SW2_pressed=0;
  if (TST==0 && TSTFB==1) TST_pressed=1; else TST_pressed=0;  
  A_pot= analogRead   (POT_PIN);   
  if (IS_INVERT_POT) A_pot=1023-A_pot;
  A_current= analogRead (CURRENT_SENSE_PIN)/8;  // in tenth Amps
  if(IS_POT_CONTROLED)
  { 
    A_rate = analogRead (POT_FREQUENCY_PIN);
    A_comp = analogRead (POT_INTENSITY_PIN);
    A_pres = analogRead (POT_PRES_PIN);
    if (abs(pot_rate-A_rate)<5) pot_rate = POT_EMA_ALPHA*pot_rate + (1-POT_EMA_ALPHA)*A_rate; else pot_rate=A_rate;
    if (abs(pot_comp-A_comp)<5) pot_comp = POT_EMA_ALPHA*pot_comp + (1-POT_EMA_ALPHA)*A_comp; else pot_comp=A_comp;
    if (abs(pot_pres-A_pres)<5) pot_pres = POT_EMA_ALPHA*pot_pres + (1-POT_EMA_ALPHA)*A_pres; else pot_pres=A_pres;
    A_comp = range_pot(int(pot_comp),g_stMcuMngr.m_CompPotLow,g_stMcuMngr.m_CompPotHigh);
    A_rate = range_pot(int(pot_rate),g_stMcuMngr.m_RatePotLow,g_stMcuMngr.m_RatePotHigh);
    A_pres = range_pot(int(pot_pres),g_stMcuMngr.m_PresPotLow,g_stMcuMngr.m_PresPotHigh);
 
    Compression_perc= MIN_VOLUME_PERCENT_DISPLAY + int(float(A_comp)*(100-MIN_VOLUME_PERCENT_DISPLAY)/1023);
    Compression_perc= constrain(Compression_perc,MIN_VOLUME_PERCENT_DISPLAY,100);

    BPM = 6+(A_rate-23)/55;               // 0 is 6 breaths per minute, 1023 is 24 BPM
    breath_cycle_time = 60000/BPM+100;    // in milisec

    insp_pressure= 30+A_pres/25;          // 0 is 30 mBar, 1023 is 70 mBar
    insp_pressure= constrain(insp_pressure,30,70);
    if (abs(insp_pressure-prev_insp_pressure)>1) { prev_insp_pressure=insp_pressure; start_disp_pres=millis(); display_LCD(); }
  }
  else
  {   FD = (1-digitalRead  (FREQ_DOWN_PIN)); 
      FU = (1-digitalRead  (FREQ_UP_PIN));
      AD = (1-digitalRead  (INTENSITY_UP_PIN));
      AU = (1-digitalRead  (INTENSITY_DECREASE_PIN));
    if (TST==0) {
        if (FD==0 && FDFB==1) {BPM-=2; if(BPM<6) BPM=6; cycle_number=0;} 
        if (FU==0 && FUFB==1) {BPM+=2; if(BPM>24) BPM=24; cycle_number=0;}
        breath_cycle_time = 60000/BPM+100;
        if (AD==0 && ADFB==1) {Compression_perc-=BUTTON_STEP; if (Compression_perc<MIN_VOLUME_PERCENT_DISPLAY) Compression_perc=MIN_VOLUME_PERCENT_DISPLAY; }
        if (AU==0 && AUFB==1) {Compression_perc+=BUTTON_STEP; if (Compression_perc>100) Compression_perc=100;}
        }
     if (TST==1) {
        if (FD==0 && FDFB==1) {insp_pressure-=5; if(insp_pressure<30) insp_pressure=30; } 
        if (FU==0 && FUFB==1) {insp_pressure+=5; if(insp_pressure>70) insp_pressure=70;}
        if (AD==0 && ADFB==1) {insp_pressure-=5; if(insp_pressure<30) insp_pressure=30; } 
        if (AU==0 && AUFB==1) {insp_pressure+=5; if(insp_pressure>70) insp_pressure=70;}
        }
  }
  range_factor = MIN_VOLUME_PERCENT+(Compression_perc-MIN_VOLUME_PERCENT_DISPLAY)*(100-MIN_VOLUME_PERCENT)/(100-MIN_VOLUME_PERCENT_DISPLAY);
  range_factor = range_factor/100;
  if (range_factor>1) range_factor=1;  if (range_factor<0) range_factor=0; 

#if (IS_PRESSURE_SENSOR_AVAILABLE==1)  
   {if (millis()-last_read_pres>100) 
     {
      last_read_pres = millis();  
      pressure_abs = int( g_PresSense.getPressure(ADC_4096)-pressure_baseline);   // mbar
      if (pressure_abs<0) pressure_abs=0;
     }
   }
#endif

  if (prev_BPM != BPM || prev_Compression_perc!=Compression_perc)  display_LCD();
  wanted_cycle_time= CYCLE_TIME_MSEC + int (float(breath_cycle_time-PROFILE_LEN*CYCLE_TIME_MSEC)*float(g_stMcuMngr.m_SmearFactor)/100/PROFILE_LEN);
}

void send_data_to_monitor()
{ 
  if (monitor_index==0) Serial.println("A"); 
  if (monitor_index==1) Serial.println(byte(BPM)); 
  if (monitor_index==2) Serial.println(byte(Compression_perc)); 
  if (monitor_index==3) Serial.println(byte(pressure_abs)); 
  if (monitor_index==4) Serial.println(byte(failure)); 
  if (monitor_index==5) {if (send_beep) {Serial.println(byte(1)); send_beep=0;} else Serial.println(byte(0)); }
  if (monitor_index==6) Serial.println(byte(insp_pressure)); 
  monitor_index+=1; if (monitor_index==7) monitor_index=0;
}

void blinkBlue (int numb)
{
  if (blueOn==1) {if (millis()-lastBlue>20) {lastBlue=millis(); blueOn=0; LED_FAIL(0); }}   
  if (blueOn==0) {if (millis()-lastBlue>100*numb) {lastBlue=millis(); blueOn=1; LED_FAIL(1); }}   
}

void LED_FREQ(byte val)  {digitalWrite ( LED_FREQUENCY_PIN, val);}
void LED_AMP (byte val)  {digitalWrite ( LED_INTENSITY_PIN, val); }
void LED_FAIL (byte val)  {digitalWrite ( FAILED_LED_PIN, val);     }
void LED_USR (byte val)  {digitalWrite ( USER_LED_PIN, val);     }

void print_tele ()  // UNCOMMENT THE TELEMETRY NEEDED
{
//  Serial.print(" Fail (disc,motion,hiPres):"); Serial.print(disconnected); Serial.print(","); Serial.print(motion_failure); Serial.print(","); Serial.print(high_pressure_detected);
//  Serial.print(" CL:");  Serial.print(cycles_lost);  
//  Serial.print(" min,max:");  Serial.print(g_stMcuMngr.m_MinArmPos); Serial.print(","); Serial.print(g_stMcuMngr.m_MaxArmPos);  
//  Serial.print(" WPWM :");  Serial.print(motorPWM); 
//  Serial.print(" integral:");  Serial.print(int(integral));  
//  Serial.print(" Wa:");  Serial.print(int(wanted_pos));  
//  Serial.print(" Ac:");  Serial.print(A_pot); 
//  Serial.print(" cur:");  Serial.print(A_current); 
//  Serial.print(" amp:");  Serial.print(Compression_perc); 
//  Serial.print(" freq:");  Serial.print(A_rate); 
//  Serial.print(" w cyc t:"); Serial.print(wanted_cycle_time);
//  Serial.print(" P :"); Serial.print(pressure_abs);
//  Serial.print(" AvgP :"); Serial.print(int(avg_pres));
//  Serial.print(" RF:");  Serial.print(range_factor); 
    Serial.println("");
}


void CommMngr_Run()
{
  if (millis()-last_sent_data>20)
  { 
    if (IS_SEND_TO_MONITOR == TRUE &&
  IS_TELEMETRY_ENABLED == FALSE)
  {  
    send_data_to_monitor(); 
  }
     if (IS_TELEMETRY_ENABLED == TRUE)
  { 
    print_tele();
  }
    last_sent_data=millis();
  }
}

void Run_StandbyState()
{
  g_stMcuMngr.m_CurrState = eState_Standby;
  g_stMcuMngr.m_NextState = eState_Standby;
  
  while(g_stMcuMngr.m_CurrState == g_stMcuMngr.m_NextState)
  {
    CommMngr_Run();
    if (USR_status) 
        {
      if (millis()-lastUSRblink>10)
        {
          USR_status=0;
          lastUSRblink=millis();
          LED_USR(0);
        }
      }
        else 
    {
      if (millis()-lastUSRblink>490) 
      {
      USR_status=1;
      lastUSRblink=millis();
      LED_USR(1);
      }
    }
    wanted_vel_PWM=0;    // dont move
    set_motor_PWM (wanted_vel_PWM);
    delay (1);
    
    if (SW2_pressed)   // start breathing motion   
    {
      g_stMcuMngr.m_NextState = eState_Operating;
    }
    if (TST==0)
    {   
      last_TST_not_pressed=millis();
    }
    if (millis()-last_TST_not_pressed>3000)
    {
      LED_USR(1);
      while (TST==1 || TST_pressed)
      { 
        read_IO(); // wait for button release
      }   
            g_stMcuMngr.m_NextState = eState_Config;
    }
  }
}


void Run_OperatingState()
{
  initialize_breath();
  g_stMcuMngr.m_CurrState = eState_Operating;
  g_stMcuMngr.m_NextState = eState_Operating;
  
  while(g_stMcuMngr.m_CurrState == g_stMcuMngr.m_NextState)
  {
    CommMngr_Run();
    run_profile_func ();
    if (SW2_pressed)
    {
      g_stMcuMngr.m_NextState = eState_Standby;  // stop breathing motion
    }
  }
}

void Run_ConfigState()
{
  CommMngr_Run();
  g_stMcuMngr.m_CurrState = eState_Config;
  g_stMcuMngr.m_NextState = eState_Config;
  
  while(g_stMcuMngr.m_CurrState == g_stMcuMngr.m_NextState)
  {
    display_menu();
  }
}

void Run_BootState()
{
  CommMngr_Run();
  g_stMcuMngr.m_CurrState = eState_Boot;
  g_stMcuMngr.m_NextState = eState_Boot;
  
  while(g_stMcuMngr.m_CurrState == g_stMcuMngr.m_NextState)
  {
    g_stMcuMngr.m_NextState = eState_Standby;
  }
}

void Run_ErrorState()
{
  CommMngr_Run();
  g_stMcuMngr.m_CurrState = eState_Error;
  g_stMcuMngr.m_NextState = eState_Error;
  
  while(g_stMcuMngr.m_CurrState == g_stMcuMngr.m_NextState)
  {

  }
}


void setup() {
  Init_GPIO();
  Init_Comm();
  Init_Periph();
  Init_App();
}

void loop() 
{
  read_IO ();
  switch (g_stMcuMngr.m_NextState){
    case eState_Boot:
  {
    Run_BootState();
    }
    break;
    case eState_Standby:
  {
    Run_StandbyState();
    }
    break;
    case eState_Operating:
  {
    Run_OperatingState();
    }
    break;
    case eState_Config:
  {
    Run_ConfigState();
    }
    break;
    case eState_Error:
  {
    Run_ErrorState();
    }
    break;    
  }
}
