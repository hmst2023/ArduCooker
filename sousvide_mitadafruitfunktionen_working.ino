//Include libraries
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include<LiquidCrystal.h>
#include <EEPROM.h>


#define button 9
#define ONE_WIRE_BUS 7
#define relais 8
#define poti 0
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);  // sets the interfacing pins

// ************************************************
// PID Variables and constants
// ************************************************
 
//Define Variables we'll be connecting to
double Setpoint;
double Input;
double Output;
 
volatile long onTime = 0;
 
// pid tuning parameters
double Kp;
double Ki;
double Kd;
 
// EEPROM addresses for persisted data
const int SpAddress = 0;
const int KpAddress = 8;
const int KiAddress = 16;
const int KdAddress = 24;
 
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
 
// 10 second Time Proportional Output window
int WindowSize = 10000; 
unsigned long windowStartTime;

// ************************************************
// Auto Tune Variables and constants
// ************************************************
byte ATuneModeRemember=2;
 
double aTuneStep=500;
double aTuneNoise=1;
unsigned int aTuneLookBack=20;
 
boolean tuning = false;
 
PID_ATune aTune(&Input, &Output);
 



void setup()
{
  Serial.begin(9600);
  Serial.println("Arduino Sous Vide Starting");
  pinMode(relais, OUTPUT);
  digitalWrite(relais, HIGH);
  pinMode(button, INPUT_PULLUP);
  lcd.begin(16, 2);  // initializes the 16x2 LCD
  sensors.begin();
  
  delay(3000); 
  myPID.SetMode(AUTOMATIC);

  LoadParameters();
  myPID.SetTunings(Kp,Ki,Kd);
  myPID.SetSampleTime(1000);
  myPID.SetOutputLimits(0, WindowSize);
  windowStartTime = millis();
  Serial.print("Kp:");
  Serial.println(Kp);
  Serial.print("Ki:");
  Serial.println(Ki);
  Serial.print("Kd:");
  Serial.println(Kd);
  delay(1000);
}




void loop(void)
{ 
  sensors.requestTemperatures();
  if (sensors.isConversionAvailable(0))
  {
    Input = sensors.getTempCByIndex(0);
  }
  
    
  int sollTemperatur = map(analogRead(poti), 0, 1023, 0, 99);
  Setpoint=sollTemperatur;
  
  if (digitalRead(button) == LOW) {
     StartAutoTune();
    }
  if (tuning) // run the auto-tuner
  {
     if (aTune.Runtime()) // returns 'true' when done
     {
        FinishAutoTune();
     }
  }
  else // Execute control algorithm
  {
     myPID.Compute();
     onTime = Output; 
     Serial.println(Output);
  }
  
  char *result = malloc(2);
  sprintf(result,"%02d",sollTemperatur);
   
   long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if(now - windowStartTime>WindowSize) { //time to shift the Relay Window
     windowStartTime += WindowSize;
  }
  if((onTime > 100) && (onTime > (now - windowStartTime))){
     digitalWrite(relais,LOW);
  }else{
     digitalWrite(relais,HIGH);
  }


  if (tuning==false){
    lcd.setCursor(0,0);           //sets the cursor at row 0 column 0
    lcd.print("SOUS VIDE COOKER"); // prints 16x2 LCD MODULE
    } else {
    lcd.setCursor(0,0);           //sets the cursor at row 0 column 0
    lcd.print("TUNING COOKER"); // prints 16x2 LCD MODULE
    }
      
  lcd.setCursor(0,1);           //sets the cursor at row 1 column 2
      // prints HELLO WORLD
  lcd.print(Input);
  lcd.setCursor(5,1); 
  lcd.print("/");
  lcd.setCursor(6,1); 
  lcd.print(result);
  lcd.setCursor(8,1); 
  lcd.print(" GRAD");
}





void StartAutoTune()
{
   // REmember the mode we were in
   ATuneModeRemember = myPID.GetMode();
 
   // set up the auto-tune parameters
   aTune.SetNoiseBand(aTuneNoise);
   aTune.SetOutputStep(aTuneStep);
   aTune.SetLookbackSec((int)aTuneLookBack);
   tuning = true;
}
 
// ************************************************
// Return to normal control
// ************************************************
void FinishAutoTune()
{
   tuning = false;
 
   // Extract the auto-tune calculated parameters
   Kp = aTune.GetKp();
   Ki = aTune.GetKi();
   Kd = aTune.GetKd();
 
   // Re-tune the PID and revert to normal control mode
   myPID.SetTunings(Kp,Ki,Kd);
   myPID.SetMode(ATuneModeRemember);
   
   // Persist any changed parameters to EEPROM
   SaveParameters();
}
void SaveParameters()
{
   if (Setpoint != EEPROM_readDouble(SpAddress))
   {
      EEPROM_writeDouble(SpAddress, Setpoint);
   }
   if (Kp != EEPROM_readDouble(KpAddress))
   {
      EEPROM_writeDouble(KpAddress, Kp);
   }
   if (Ki != EEPROM_readDouble(KiAddress))
   {
      EEPROM_writeDouble(KiAddress, Ki);
   }
   if (Kd != EEPROM_readDouble(KdAddress))
   {
      EEPROM_writeDouble(KdAddress, Kd);
   }
}
 
// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
    // Load from EEPROM
   Setpoint = EEPROM_readDouble(SpAddress);
   Kp = EEPROM_readDouble(KpAddress);
   Ki = EEPROM_readDouble(KiAddress);
   Kd = EEPROM_readDouble(KdAddress);
   
   // Use defaults if EEPROM values are invalid
   if (isnan(Setpoint))
   {
     Setpoint = 60;
   }
   if (isnan(Kp))
   {
     Kp = 850;
   }
   if (isnan(Ki))
   {
     Ki = 0.5;
   }
   if (isnan(Kd))
   {
     Kd = 0.1;
   }  
     
}
 
 
// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
}
 
// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      *p++ = EEPROM.read(address++);
   }
   return value;
}

