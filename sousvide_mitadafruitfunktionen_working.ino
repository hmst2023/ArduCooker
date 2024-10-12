//Include libraries
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include<LiquidCrystal.h>
#include <EEPROM.h>
#include <RCSwitch.h>
//LIPO
#define USBIn 3
#define piezo 2
#define batteryPin 1
//LIPO
#define button 13
#define ONE_WIRE_BUS 11
#define funk 12
#define poti 0
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
LiquidCrystal lcd(9, 10, 8, 7, 6, 5);  // sets the interfacing pins
RCSwitch sender = RCSwitch();
//new
DeviceAddress tempDeviceAddress;
//LIPO

    int batteryLoad = 860;    //initialization of sensor variable, equivalent to EMA Y
    long lastBatteryCheck = 0;
    long lastBeep = 0;
    bool alarm = false;
    bool piezoSound = false;
    byte batterieLinksVoll[8] = 
{
  B00111,
  B01000,
  B01010,
  B10010,
  B10010,
  B01010,
  B01000,
  B00111,
};

byte batterieLinksLeer[8] = 
{
  B00111,
  B01000,
  B01000,
  B10000,
  B10000,
  B01000,
  B01000,
  B00111,
};
byte batterieRechtsVoll[8] = 
{
  B11110,
  B00001,
  B10101,
  B10101,
  B10101,
  B10101,
  B00001,
  B11110,
};

byte batterieRechtsHalb[8] = 
{
  B11110,
  B00001,
  B10001,
  B10001,
  B10001,
  B10001,
  B00001,
  B11110,
};
byte batterieRechtsLeer[8] = 
{
  B11110,
  B00001,
  B00001,
  B00001,
  B00001,
  B00001,
  B00001,
  B11110,
};
byte grad[8] = 
{
  B00110,
  B01001,
  B01001,
  B00110,
  B00000,
  B00000,
  B00000,
  B00000,
};
//LIPO


int channel = 3;
bool switchState;
bool debounce = false;
int  resolution = 12;
unsigned long lastTempRequest = 0;
unsigned long lastDebounce = 0;
unsigned long lastTempSend = 0;
int  delayInMillis = 0;
float temperature = 0.0;
int  idle = 0;
int menue = 0;
float EMA_a = 0.1;      //initialization of EMA alpha
float EMA_S;          //initialization of EMA S
////
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
byte ATuneModeRemember = 2;

double aTuneStep = 500;
double aTuneNoise = 1;
unsigned int aTuneLookBack = 20;

boolean tuning = false;

PID_ATune aTune(&Input, &Output);




void setup()
{
  //LIPO
  pinMode(piezo, OUTPUT);
  //LIPO
  EMA_S = analogRead(poti);
  Serial.begin(115200);
  Serial.println("Arduino Sous Vide Starting");
  sender.enableTransmit(funk);
  sender.setProtocol(1);
  sender.setPulseLength(418);
  pinMode(button, INPUT_PULLUP);
  lcd.begin(16, 2);  // initializes the 16x2 LCD
  //LIPO
  lcd.createChar(0,batterieLinksVoll);
  lcd.createChar(1,batterieLinksLeer);
  lcd.createChar(2,batterieRechtsVoll);
  lcd.createChar(3,batterieRechtsHalb);
  lcd.createChar(4,batterieRechtsLeer);
  lcd.createChar(5,grad);
  //LIPO
  sensors.begin();
  sensors.getAddress(tempDeviceAddress, 0);
  sensors.setResolution(tempDeviceAddress, resolution);

  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  delayInMillis = 750 / (1 << (12 - resolution));
  delay(3000);
  lastTempRequest = millis();
  ///

  myPID.SetMode(AUTOMATIC);

  LoadParameters();
  myPID.SetTunings(Kp, Ki, Kd);
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




void loop(void) {
  long moment=millis();
  if (debounce) {
    if (moment - lastDebounce > 1000) {
      debounce = false;
    }
  }
   if (lastBatteryCheck+10000<moment){
    batteryLoad = analogRead(batteryPin);
    lastBatteryCheck=moment;
    }
    if ((batteryLoad<680) && (digitalRead(USBIn)==LOW)) alarm=true;
    else alarm=false;
    

    if (alarm) {
      if (lastBeep+1000<moment){
         
        if (piezoSound) piezoSound=false;
        else piezoSound = true;
        lastBeep=moment;
        }
        
      } else piezoSound = false;

    if (piezoSound) {
      tone(piezo,2000);      
      } else { 
      noTone(piezo);
      }
    
  tempCheck();

  switch (menue)
  {
    case 0:
      heating();
      break;
    case 1:
      conf();
      break;
    case 2:
      zieltemperaturEinstellen();
      break;
    case 3:
      autoTuning();
      break;
    case 4:
      off();
      break;
    case 5:
      pid_conf();
      break;
    case 6:
      Kp_conf();
      break;
    case 7:
      Ki_conf();
      break;
    case 8:
      Kd_conf();
      break;
    case 9:
      frequenzEinstellen();
      break;
  }

  funksteckdoseSchalten();
}

void frequenzEinstellen(){
  lcd.setCursor(0, 0);         //sets the cursor at row 0 column 0
  lcd.print("Channel: ");
  int sensorValue = analogRead(poti);
  sensorValue = sensorValue/341+1;
  lcd.setCursor(0, 1);          //sets the cursor at row 1 column 2
  lcd.print(sensorValue, 1);
  lcd.print("            ");
  if (!debounce) {
    if (digitalRead(button) == LOW) {
      channel = sensorValue;
      menue = 0;
      debounce = true;
      lastDebounce = millis();
    }
  }
}


void tempCheck() {
  if (millis() - lastTempRequest >= delayInMillis) {

    temperature = sensors.getTempCByIndex(0);
    sensors.requestTemperatures();
    lastTempRequest = millis();
  }
  if (millis() - lastTempSend >= 7500) {
    Serial.println(temperature);
    lastTempSend = millis();
  }
  Input = temperature;
}

void conf() {
  if (analogRead(poti) > 920) {
    lcd.setCursor(0, 0);         //sets the cursor at row 0 column 0
    lcd.print("< PID|Off       ");
    lcd.setCursor(0, 1);         //sets the cursor at row 0 column 0
    lcd.print("      ---       ");
    }  else if (analogRead(poti) > 715) {
    lcd.setCursor(0, 0);         //sets the cursor at row 0 column 0
     lcd.print("< PID|Off       ");
    lcd.setCursor(0, 1);         //sets the cursor at row 0 column 0
    lcd.print("  ---            ");
  } else if (analogRead(poti) > 510) {
    lcd.setCursor(0, 0);         //sets the cursor at row 0 column 0
    lcd.print("Cook|Temp|Freq >");
    lcd.setCursor(0, 1);         //sets the cursor at row 0 column 0
    lcd.print("          ----  ");
  } else if (analogRead(poti) > 205) {
    lcd.setCursor(0, 0);
    lcd.print("Cook|Temp|Freq >");
    lcd.setCursor(0, 1);         //sets the cursor at row 0 column 0
    lcd.print("     ----       ");
  } else {
    lcd.setCursor(0, 0);
    lcd.print("Cook|Temp|Freq >");
    lcd.setCursor(0, 1);         //sets the cursor at row 0 column 0
    lcd.print("----            ");
  }
  if (!debounce) {
    if (digitalRead(button) == LOW) {
      if (analogRead(poti) > 920) {
        menue = 4;
      } else if (analogRead(poti) > 715) {

        menue = 5;


      } else if (analogRead(poti) > 510) {

        menue = 9;


      } else if (analogRead(poti) > 205) {
        menue = 2;
      } else {
        menue = 0;
      }
      debounce = true;
      lastDebounce = millis();
    }
  }
}
void Kp_conf() {
  lcd.setCursor(0, 0);         //sets the cursor at row 0 column 0
  lcd.print("KP Wert: ");
  int sensorValue = analogRead(poti);
  EMA_S = (EMA_a * sensorValue) + ((1 - EMA_a) * EMA_S);
  int Temp = EMA_S * 3;
  lcd.setCursor(0, 1);          //sets the cursor at row 1 column 2
  lcd.print(Temp, 1);
  lcd.print("           ");
  if (!debounce) {
    if (digitalRead(button) == LOW) {
      Kp = Temp;
      menue = 7;
      debounce = true;
      lastDebounce = millis();
    }
  }
}

void Ki_conf() {
  lcd.setCursor(0, 0);         //sets the cursor at row 0 column 0
  lcd.print("KI Wert: ");
  int sensorValue = analogRead(poti);
  EMA_S = (EMA_a * sensorValue) + ((1 - EMA_a) * EMA_S);
  float Temp = EMA_S * 10 / 1024;
  lcd.setCursor(0, 1);          //sets the cursor at row 1 column 2
  lcd.print(Temp, 1);
  lcd.print("           ");
  if (!debounce) {
    if (digitalRead(button) == LOW) {
      Ki = Temp;
      menue = 8;
      debounce = true;
      lastDebounce = millis();
    }
  }
}

void Kd_conf() {
  lcd.setCursor(0, 0);         //sets the cursor at row 0 column 0
  lcd.print("KD Wert: ");
  int sensorValue = analogRead(poti);
  EMA_S = (EMA_a * sensorValue) + ((1 - EMA_a) * EMA_S);
  float Temp = EMA_S * 10 / 1024;
  lcd.setCursor(0, 1);          //sets the cursor at row 1 column 2
  lcd.print(Temp, 1);
  lcd.print("           ");
  if (!debounce) {
    if (digitalRead(button) == LOW) {
      Kd = Temp;
      menue = 0;
      debounce = true;
      lastDebounce = millis();
      myPID.SetTunings(Kp, Ki, Kd);
    }
  }
}



void pid_conf() {
  if (analogRead(poti) > 920) {
    lcd.setCursor(0, 0);         //sets the cursor at row 0 column 0
    lcd.print("< Store         ");
    lcd.setCursor(0, 1);         //sets the cursor at row 0 column 0
    lcd.print("  -----         ");
  } else if (analogRead(poti) > 715) {
    lcd.setCursor(0, 0);         //sets the cursor at row 0 column 0
    lcd.print("< Show|Autotune>");
    lcd.setCursor(0, 1);         //sets the cursor at row 0 column 0
    lcd.print("       -------- ");
  } else if (analogRead(poti) > 510) {
    lcd.setCursor(0, 0);         //sets the cursor at row 0 column 0
    lcd.print("< Show|Autotune>");
    lcd.setCursor(0, 1);         //sets the cursor at row 0 column 0
    lcd.print("  ----          ");
  } else if (analogRead(poti) > 205) {
    lcd.setCursor(0, 0);
    lcd.print("Back|Default| >");
    lcd.setCursor(0, 1);         //sets the cursor at row 0 column 0
    lcd.print("     -------   ");
  } else {
    lcd.setCursor(0, 0);
    lcd.print("Back|Default| >");
    lcd.setCursor(0, 1);         //sets the cursor at row 0 column 0
    lcd.print("----           ");
  }
  if (!debounce) {
    if (digitalRead(button) == LOW) {
      if (analogRead(poti) > 920) {
        SaveParameters();
        lcd.setCursor(0, 0);
        lcd.print("Storing Values  ");
        lcd.setCursor(0, 1);         //sets the cursor at row 0 column 0
        lcd.print("                ");
        delay(1000);
      } else if (analogRead(poti) > 715) {
        if (abs(Input - Setpoint) < 0.5) {
          menue = 3;
          StartAutoTune();
        } else {
          lcd.setCursor(0, 1);
          lcd.print("Zieltemp zu weit");
          delay(3000);
        }
      }
      else if (analogRead(poti) > 510) {
        long now = millis();
        lcd.setCursor(0, 0);
        lcd.print("KP  |  KI  |  KD");
        lcd.setCursor(0, 1);         //sets the cursor at row 0 column 0
        lcd.print(Kp, 1);
        lcd.print("|");
        lcd.print(Ki, 1);
        lcd.print("|");
        lcd.print(Kd, 1);
        delay(500);
        do {
          if (digitalRead(button) == LOW) {
            menue = 6;
            break;
          }
        } while (now + 10000 > millis());

      } else if (analogRead(poti) > 205) {
        menue = 0;
        Setpoint = 55;
        Kp = 2500;
        Ki = 0;
        Kd = 1;
        myPID.SetTunings(Kp, Ki, Kd);
        lcd.setCursor(0, 0);
        lcd.print("Loading Defaults");
        lcd.setCursor(0, 1);         //sets the cursor at row 0 column 0
        lcd.print("                ");
        delay(1000);
      } else {
        menue = 1;
      }
      debounce = true;
      lastDebounce = millis();
    }
  }
}




void zieltemperaturEinstellen() {
  lcd.setCursor(0, 0);         //sets the cursor at row 0 column 0
  lcd.print("Zieltemperatur: ");
  int sensorValue = analogRead(poti);
  EMA_S = (EMA_a * sensorValue) + ((1 - EMA_a) * EMA_S);
  float sollTemperatur = 20.0 + EMA_S * 80 / 1024;
  lcd.setCursor(0, 1);          //sets the cursor at row 1 column 2
  lcd.print(sollTemperatur, 1);
  lcd.print(" Grad       ");
  if (!debounce) {
    if (digitalRead(button) == LOW) {
      Setpoint = sollTemperatur;
      menue = 0;
      debounce = true;
      lastDebounce = millis();
    }
  }
}



void off() {
  if (!debounce) {
    if (digitalRead(button) == LOW) {
      menue = 1;
      debounce = true;
      lastDebounce = millis();
    }
  }

  lcd.setCursor(0, 0);
  lcd.print("SOUS VIDE COOKER");
  lcd.setCursor(0, 1);
  lcd.print("Temp:");
  lcd.print(Input, 1);
  lcd.print("  OFF  ");
  onTime = 0;
}
void heating() {
  if (!debounce) {
    if (digitalRead(button) == LOW) {
      menue = 1;
      debounce = true;
      lastDebounce = millis();
    }
  }

  lcd.setCursor(0, 0);
  lcd.print("   Arducooker   ");
  lcd.setCursor(0, 1);
  lcd.print(Input, 1);
  lcd.setCursor(4, 1);
  lcd.print("/");
  lcd.setCursor(5, 1);
  lcd.print(Setpoint, 1);
  lcd.setCursor(9, 1);
  lcd.write(byte(5));
  if (batteryLoad>820){
      lcd.write(byte(0)); //batterie ganz voll
      lcd.write(byte(2));
    } else if (batteryLoad>780){ 
      lcd.write(byte(0)); //batterie gut
      lcd.write(byte(3)); //batterie gut
    } else if (batteryLoad>710){
      lcd.write(byte(0)); //batterie wenig
      lcd.write(byte(4)); //batterie wenig  
    } else {
      lcd.write(byte(1)); //batterie laden
      lcd.write(byte(4)); //batterie laden
    };
    int percent = (batteryLoad - 614)*99 /246;
    lcd.print(percent);
    lcd.print("%");
  myPID.Compute();
  onTime = Output;
}

void autoTuning() {

  if (aTune.Runtime()) {
    FinishAutoTune();
  }
  onTime = Output;
  lcd.setCursor(0, 0);          //sets the cursor at row 0 column 0
  lcd.print("     TUNING     "); // prints 16x2 LCD MODULE
  lcd.setCursor(0, 1);
  lcd.print("     COOKER     ");
}







void StartAutoTune()
{
  // REmember the mode we were in
  ATuneModeRemember = myPID.GetMode();

  // set up the auto-tune parameters
  aTune.SetNoiseBand(aTuneNoise);
  aTune.SetOutputStep(aTuneStep);
  aTune.SetLookbackSec((int)aTuneLookBack);
}

// ************************************************
// Return to normal control
// ************************************************
void FinishAutoTune()
{

  // Extract the auto-tune calculated parameters
  Kp = aTune.GetKp();
  Ki = aTune.GetKi();
  Kd = aTune.GetKd();

  // Re-tune the PID and revert to normal control mode
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetMode(ATuneModeRemember);

  // Persist any changed parameters to EEPROM
  SaveParameters();
  menue = 0;
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
    Setpoint = 55;
  }
  if (isnan(Kp))
  {
    Kp = 2500;
  }
  if (isnan(Ki))
  {
    Ki = 0;
  }
  if (isnan(Kd))
  {
    Kd = 1;
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
void funksteckdoseSchalten() {
  if (!alarm){
    long now = millis();
    if (now - windowStartTime > WindowSize) { //time to shift the Relay Window
      windowStartTime += WindowSize;
    }
    if ((onTime > 600) && (onTime > (now - windowStartTime))) {
      if (switchState == 0) {
        switchState = 1;
        if (channel == 1){
          sender.sendTriState("0FFF0FFFFFFF");
        }
        if (channel == 2){
          sender.sendTriState("0FFFF0FFFFFF");
        }
        if (channel == 3){
          sender.sendTriState("0FFFFF0FFFFF");
        }
        if (channel == 4){
          sender.sendTriState("0FFFFFF0FFFF");
        }
      }
    } else {
      if (switchState == 1) {
        switchState = 0;
        if (channel == 1){
          sender.sendTriState("0FFF0FFFFFF0");
        }
        if (channel == 2){
          sender.sendTriState("0FFFF0FF0000");
        }
        if (channel == 3){
          sender.sendTriState("0FFFFF0FFFF0");
        }
        if (channel == 4){
          sender.sendTriState("0FFFFFF00000");
        }
      }
    }

  } else {
    switchState = 0;
         if (channel == 1){
          sender.sendTriState("0FFF0FFFFFF0");
         }
         if (channel == 2){
          sender.sendTriState("0FFFF0FF0000");
         }
         if (channel == 3){
          sender.sendTriState("0FFFFF0FFFF0");
         }
         if (channel == 4){
          sender.sendTriState("0FFFFFF00000");
         }
    }
}
