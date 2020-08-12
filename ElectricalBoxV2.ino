#include <EEPROM.h>
#include <Wire.h>
#include "Adafruit_Si7021.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <LiquidCrystal_I2C.h>
#include <avr/wdt.h>

//Change I/O pins
const byte up = 2;         
const byte sel = 3;       
const byte down = 4;        
const byte mist_butt = 5;
const byte HPA_Solenoid = 6;
const byte buzzer = 7;
Adafruit_Si7021 temp_and_RH_Sensor = Adafruit_Si7021();
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

//Objects
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Button press state
bool current_up = LOW;
bool last_up = LOW;
bool current_sel = LOW;
bool last_sel = LOW;
bool last_down = LOW;
bool current_down = LOW;
bool current_mistButtonState = LOW;
bool last_mistButtonState = LOW;

byte page_counter = 1;

// Case 4: mister controlls
byte subpage_counter = 0;
unsigned long countDownTimer = 0;

byte mister_On_Actual_Time_Minutes;
byte mister_On_Actual_Time_Seconds;

byte mister_Off_Actual_Time_Minutes;
byte mister_Off_Actual_Time_Seconds;

byte mister_On_Display_Time_Minutes;
byte mister_On_Display_Time_Seconds;

byte mister_Off_Display_Time_Minutes;
byte mister_Off_Display_Time_Seconds;

unsigned long TotalOnTimeInMillis;

unsigned long TotalOffTimeInMillis;

unsigned long time_Since_Last_Mist_Cycle = 0;

// Case 2,3,4: display temperature and humidity
unsigned long lastRun2 = 0;

// Custom back char
byte back[8] = {
  0b00100,
  0b01000,
  0b11111,
  0b01001,
  0b00101,
  0b00001,
  0b00001,
  0b11111
};

// Custom arrow char
byte arrow[8] = {
  0b01000,
  0b00100,
  0b00010,
  0b11111,
  0b00010,
  0b00100,
  0b01000,
  0b00000
};

// Custom save char
byte save[8] = {
  0b00100,
  0b00100,
  0b10101,
  0b01110,
  0b00100,
  0b10001,
  0b11111,
  0b11111
};

void setup() {
  Serial.begin(9600);
  // Declare pin modes
  pinMode(up, INPUT);
  pinMode(sel, INPUT);
  pinMode(down, INPUT);
  pinMode(mist_butt, INPUT);
  pinMode(HPA_Solenoid, OUTPUT);
  pinMode(buzzer, OUTPUT);

  digitalWrite(HPA_Solenoid, HIGH);
  
  Wire.begin(); 
  
  lcd.begin();
  lcd.backlight();
  lcd.createChar(1, back);
  lcd.createChar(2, arrow);
  lcd.createChar(3, save);

  //initialise temperarure & relative humididty sensor
  if (! temp_and_RH_Sensor.begin()) {
    lcd.setCursor(5, 0);
    lcd.print("T&H Err");
    delay(200000);
  }

  configureLuxSensor();

  //Retrive time values from non-volatile memory
  mister_On_Actual_Time_Minutes = EEPROM.read(0);
  mister_On_Actual_Time_Seconds = EEPROM.read(1);

  mister_Off_Actual_Time_Minutes = EEPROM.read(2);
  mister_Off_Actual_Time_Seconds = EEPROM.read(3);

  mister_Off_Display_Time_Minutes = mister_Off_Actual_Time_Minutes;
  mister_Off_Display_Time_Seconds = mister_Off_Actual_Time_Seconds;
  mister_On_Display_Time_Minutes = mister_On_Actual_Time_Minutes;
  mister_On_Display_Time_Seconds = mister_On_Actual_Time_Seconds;
  
  watchdogSetup();
}

boolean debounce(boolean last, int pin)
{
  boolean current = digitalRead(pin);
  if (last != current)
  {
    // sound buzzer
    if(current == HIGH)
    {
      for (int i = 0; i <10; i++)
      {
        digitalWrite(buzzer, HIGH);
        delay(1);// Delay 1ms
        digitalWrite(buzzer, LOW);
        delay(1);// delay ms
      }
    }
    else delay(20);
    current = digitalRead(pin);
  }
  return current;
}

void loop() {
  current_up = debounce(last_up, up);      
  current_sel = debounce(last_sel, sel);     
  current_down = debounce(last_down, down);  
  current_mistButtonState = debounce(last_mistButtonState, mist_butt);

  //if not editing sub page we can change the page counter
  if (subpage_counter == 0) {
    //Page Up
    if (last_up == LOW && current_up == HIGH) { 
      //Clear lcd if page is changed to print new one
      lcd.clear();
      if (page_counter < 4) {
        //Page up
        page_counter++;
      }
      else {
        page_counter = 1;
      }
    }
    last_up = current_up;

    if (last_down == LOW && current_down == HIGH) {
      lcd.clear();
      if (page_counter >1) {
        //Page down
        page_counter--;
      }
      else {
        page_counter = 4;
      }
    }
    last_down = current_down; //Save down button last state
  }

  switch (page_counter) {
  //home page - Display time and date
  case 1:
    //edit time for misters
    lcd.setCursor(0, 0);
    lcd.print("Mister ");
    lcd.setCursor(7, 0);
    lcd.print("ON");
    lcd.setCursor(2, 1);
    lcd.write(byte(3));  
    lcd.setCursor(5, 1);
    lcd.write(byte(1));   
    lcd.setCursor(7, 1);
    lcd.print("OFF");
      
    //count down timer
    //When solenoid is on:
    if (digitalRead(HPA_Solenoid) == LOW && subpage_counter == 0) {
      if (millis() - countDownTimer > 1000) {
        if (mister_On_Display_Time_Seconds == 0 && mister_On_Display_Time_Minutes > 0) {
          mister_On_Display_Time_Seconds = 59;
          mister_On_Display_Time_Minutes--;
        }
        else if (mister_On_Display_Time_Seconds > 0) {
          mister_On_Display_Time_Seconds--;
        }
        countDownTimer = millis();
      }
    }
    
    //when solenoid is off:
    else if ((digitalRead(HPA_Solenoid) == HIGH && subpage_counter == 0)) {
      if (millis() - countDownTimer > 1000) {
        if (mister_Off_Display_Time_Seconds == 0 && mister_Off_Display_Time_Minutes > 0) {
          mister_Off_Display_Time_Seconds = 59;
          mister_Off_Display_Time_Minutes--;
        }
        else if (mister_Off_Display_Time_Seconds > 0) {
          mister_Off_Display_Time_Seconds--;
        }
        countDownTimer = millis();
      }
    }

    //Print to LCD
    if (subpage_counter == 0) {
      lcd.setCursor(11, 0);
      if (mister_On_Display_Time_Minutes<10) {
        lcd.print("0");
      }
      lcd.print(mister_On_Display_Time_Minutes);

      lcd.setCursor(14, 0);
      if (mister_On_Display_Time_Seconds<10) {
        lcd.print("0");
      }
      lcd.print(mister_On_Display_Time_Seconds);

      lcd.setCursor(11, 1);
      if (mister_Off_Display_Time_Minutes<10) {
        lcd.print("0");
      }
      lcd.print(mister_Off_Display_Time_Minutes);

      lcd.setCursor(14, 1);
      if (mister_Off_Display_Time_Seconds<10) {       
        lcd.print("0");
      }
      lcd.print(mister_Off_Display_Time_Seconds);
    }
    else if (subpage_counter != 0)
    {
      lcd.setCursor(11, 0);  
      if (mister_On_Actual_Time_Minutes<10) {      
        lcd.print("0");
      }
      lcd.print(mister_On_Actual_Time_Minutes);

      lcd.setCursor(14, 0);
      if (mister_On_Actual_Time_Seconds<10) {     
        lcd.print("0");
      }
      lcd.print(mister_On_Actual_Time_Seconds);

      lcd.setCursor(11, 1);
      if (mister_Off_Actual_Time_Minutes<10) {      
        lcd.print("0");
      }
      lcd.print(mister_Off_Actual_Time_Minutes);

      lcd.setCursor(14, 1);
      if (mister_Off_Actual_Time_Seconds<10) {    
        lcd.print("0");
      }
      lcd.print(mister_Off_Actual_Time_Seconds);
    }
    
    if (last_sel == LOW && current_sel == HIGH) {
      if (subpage_counter < 6) {            
        subpage_counter++;               
      }
      else {                                      
        subpage_counter = 1;
      }
    }

    last_sel = current_sel;                    

    if (subpage_counter == 1) {
      lcd.setCursor(4, 1);      
      lcd.print(" ");
      lcd.setCursor(10, 0);      
      lcd.write(byte(2));

      if (last_up == LOW && current_up == HIGH) {  //Up 
        if (mister_On_Actual_Time_Minutes < 59) {
          mister_On_Actual_Time_Minutes++;
        }
        else {
          mister_On_Actual_Time_Minutes = 0;
        }
      }

      last_up = current_up;

      if (last_down == LOW && current_down == HIGH) {//Down
        if (mister_On_Actual_Time_Minutes >0) {
          mister_On_Actual_Time_Minutes--;
        }
        else {
          mister_On_Actual_Time_Minutes = 59;
        }
      }

      last_down = current_down;
    }

    if (subpage_counter == 2) {
      lcd.setCursor(10, 0);
      lcd.print(" ");
      lcd.setCursor(13, 0);
      lcd.write(byte(2));

      if (last_up == LOW && current_up == HIGH) {  //Up 
        if (mister_On_Actual_Time_Seconds < 59) {
          mister_On_Actual_Time_Seconds++;
        }
        else {
          mister_On_Actual_Time_Seconds = 0;
        }
      }

      last_up = current_up;

      if (last_down == LOW && current_down == HIGH) {//Down
        if (mister_On_Actual_Time_Seconds >0) {
          mister_On_Actual_Time_Seconds--;
        }
        else {
          mister_On_Actual_Time_Seconds = 59;
        }
      }

      last_down = current_down;
    }

    if (subpage_counter == 3) {
      lcd.setCursor(13, 0);
      lcd.print(" ");
      lcd.setCursor(10, 1);
      lcd.write(byte(2));

      if (last_up == LOW && current_up == HIGH) {  //Up 
        if (mister_Off_Actual_Time_Minutes < 59) {
          mister_Off_Actual_Time_Minutes++;
        }
        else {
          mister_Off_Actual_Time_Minutes = 0;
        }
      }
      last_up = current_up;

      if (last_down == LOW && current_down == HIGH) {
        if (mister_Off_Actual_Time_Minutes >0) {
          mister_Off_Actual_Time_Minutes--;
        }
        else {
          mister_Off_Actual_Time_Minutes = 59;
        }
      }
      last_down = current_down;
    }

    if (subpage_counter == 4) {
      lcd.setCursor(10, 1);
      lcd.print(" ");
      lcd.setCursor(13, 1);
      lcd.write(byte(2));

      if (last_up == LOW && current_up == HIGH) {
        if (mister_Off_Actual_Time_Seconds < 59) {
          mister_Off_Actual_Time_Seconds++;
        }
        else {
          mister_Off_Actual_Time_Seconds = 0;
        }
      }
      last_up = current_up;

      if (last_down == LOW && current_down == HIGH) {
        if (mister_Off_Actual_Time_Seconds >0) {
          mister_Off_Actual_Time_Seconds--;
        }
        else {
          mister_Off_Actual_Time_Seconds = 59;
        }
      }
      last_down = current_down;
    }

    if (subpage_counter == 5) {
      lcd.setCursor(13, 1);
      lcd.print(" ");
      lcd.setCursor(1, 1);
      lcd.write(byte(2));
      //Move item + or -
      if (last_up == LOW && current_up == HIGH) {
        lcd.setCursor(0, 1);     
        lcd.print(" ");
        subpage_counter = 0;    

        EEPROM.write(0, mister_On_Actual_Time_Minutes);
        EEPROM.write(1, mister_On_Actual_Time_Seconds);

        EEPROM.write(2, mister_Off_Actual_Time_Minutes);
        EEPROM.write(3, mister_Off_Actual_Time_Seconds);

        lcd.clear();
        lcd.setCursor(5, 1);
        lcd.print("SAVED!");
        delay(1000);
        lcd.clear();

        //Save and start with misters off
        time_Since_Last_Mist_Cycle = millis();
        countDownTimer = millis();
        digitalWrite(HPA_Solenoid, HIGH);
        mister_On_Display_Time_Minutes = mister_On_Actual_Time_Minutes;
        mister_On_Display_Time_Seconds = mister_On_Actual_Time_Seconds;
        mister_Off_Display_Time_Minutes = mister_Off_Actual_Time_Minutes;
        mister_Off_Display_Time_Seconds = mister_Off_Actual_Time_Seconds;

        last_sel = current_sel;
      }

      last_up = current_up;

      if (last_down == LOW && current_down == HIGH) {
        lcd.setCursor(1, 1); 
        lcd.print(" ");
        subpage_counter = 6;    
      }

      last_down = current_down;
    }

    if (subpage_counter == 6) {
      lcd.setCursor(1, 1);
      lcd.print(" ");
      lcd.setCursor(4, 1);
      lcd.write(byte(2));

      if (last_up == LOW && current_up == HIGH) {
        lcd.setCursor(4, 1);
        lcd.print(" ");
        subpage_counter = 0;

        //Restart with misters off
        time_Since_Last_Mist_Cycle = millis();
        countDownTimer = millis();
        digitalWrite(HPA_Solenoid, HIGH);
        mister_On_Actual_Time_Minutes = EEPROM.read(0);
        mister_On_Actual_Time_Seconds = EEPROM.read(1);
        mister_Off_Actual_Time_Minutes = EEPROM.read(2);
        mister_Off_Actual_Time_Seconds = EEPROM.read(3);

        mister_On_Display_Time_Minutes = mister_On_Actual_Time_Minutes;
        mister_On_Display_Time_Seconds = mister_On_Actual_Time_Seconds;
        mister_Off_Display_Time_Minutes = mister_Off_Actual_Time_Minutes;
        mister_Off_Display_Time_Seconds = mister_Off_Actual_Time_Seconds;
      }

      last_up = current_up;

      if (last_down == LOW && current_down == HIGH) {
        lcd.setCursor(4, 1);
        lcd.print(" ");
        subpage_counter = 1;
      }

      last_down = current_down;
    }
    break;

  // Display temperature and humidity
  case 2:
    if (millis() - lastRun2 > 2000) {
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(temp_and_RH_Sensor.readTemperature());
      lcd.print((char)223);
      lcd.print("C");
      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");
      lcd.print(temp_and_RH_Sensor.readHumidity());
      lcd.print("%");
      lastRun2 = millis();
    }
    break;
    
  // DisplayLumosity
  case 3:
    if (millis() - lastRun2 > 500) {
      sensors_event_t event;
      tsl.getEvent(&event);
      lcd.setCursor(0, 0);
      lcd.print("Lumosity Value:");
      lcd.setCursor(4, 1);
  
      if (event.light)
      {
        lcd.print(event.light, 2);
        lcd.setCursor(9, 1);
        lcd.print("LUX");
      }
      else
      {
        /* If event.light = 0 lux the sensor is probably saturated
           and no reliable data could be generated! */
        lcd.print("overload");
      }
      lastRun2 = millis();
    }
    break;
    
  // Display line pressure
  case 4:
    if (millis() - lastRun2 > 100) {
      //0.5v == 102 (analog val) @ 0psi
      //4.5v == 921 (analog val) @ 174.045 psi
      // y = mx + c == y = ((174.045-0)/(921-102))x-(174.045*102)/(921-102)
            
      lcd.setCursor(0,0);
      lcd.print("Pressure Reading");
      lcd.setCursor(4 ,1);
      
      long pressure = (174.045/819.0)*(analogRead(A1)-102.0);

      //print plank space if the pressure is less than 100\
      this is because the LCD will keep its last value so if the pressure goes above and then below 100 the 3rd significant figure will remain
      if (pressure < 100){
        lcd.print(" ");
      }
      lcd.print(pressure);

      lcd.setCursor(9,1);
      lcd.print("PSI");
    }
    
    break;
  }

  TotalOnTimeInMillis = (mister_On_Actual_Time_Minutes * 60000 + mister_On_Actual_Time_Seconds * 1000);
  TotalOffTimeInMillis = (mister_Off_Actual_Time_Minutes * 60000 + mister_Off_Actual_Time_Seconds * 1000);

  //if solenoid is off and 
  if ((digitalRead(HPA_Solenoid) == HIGH) && (millis() - time_Since_Last_Mist_Cycle) > TotalOffTimeInMillis && TotalOffTimeInMillis != 0) {
    digitalWrite(HPA_Solenoid, LOW);
    time_Since_Last_Mist_Cycle = millis();
    countDownTimer = millis();
    mister_Off_Display_Time_Minutes = mister_Off_Actual_Time_Minutes;
    mister_Off_Display_Time_Seconds = mister_Off_Actual_Time_Seconds;
  }

  if ((digitalRead(HPA_Solenoid) == LOW) && (millis() - time_Since_Last_Mist_Cycle) > TotalOnTimeInMillis && TotalOnTimeInMillis != 0) {
    digitalWrite(HPA_Solenoid, HIGH);
    time_Since_Last_Mist_Cycle = millis();
    countDownTimer = millis();
    mister_On_Display_Time_Minutes = mister_On_Actual_Time_Minutes;
    mister_On_Display_Time_Seconds = mister_On_Actual_Time_Seconds;
  }

  //Mist button
  if (current_mistButtonState == HIGH) {
    digitalWrite(HPA_Solenoid, LOW);
    last_mistButtonState = current_mistButtonState;
  }
  else if (last_mistButtonState == HIGH && current_mistButtonState == LOW) {
    digitalWrite(HPA_Solenoid, HIGH);
    
    //reinitialise all counters for misting
    time_Since_Last_Mist_Cycle = millis();
    countDownTimer = millis();
    mister_On_Display_Time_Minutes = mister_On_Actual_Time_Minutes;
    mister_On_Display_Time_Seconds = mister_On_Actual_Time_Seconds;
    mister_Off_Display_Time_Minutes = mister_Off_Actual_Time_Minutes;
    mister_Off_Display_Time_Seconds = mister_Off_Actual_Time_Seconds;
    last_mistButtonState = current_mistButtonState;
  }
  wdt_reset();
}

void configureLuxSensor(void)
{
  //initialise lumosity sensor
  if(!tsl.begin())
  {
    lcd.setCursor(5, 0);
    lcd.print("LUX Err");
    delay(200000);
  }
  
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */
}

void watchdogSetup(void) {
  cli(); // disable all interrupts 
  wdt_reset(); // reset the WDT timer 
  /*   WDTCSR conﬁgurations:
  WDIE  = 1: Interrupt Enable
  WDE   = 1 :Reset Enable
  WDP3  WDP2  WDP1  WDP0  Time-Out(ms)
  0     0     0     0     16
  0     0     0     1     32
  0     0     1     0     64
  0     0     1     1     125
  0     1     0     0     250
  0     1     0     1     500
  0     1     1     0     1000
  0     1     1     1     2000
  1     0     0     0     4000
  1     0     0     1     8000
  */
  // Enter Watchdog Conﬁguration mode: 
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  // Set Watchdog settings: 
  WDTCSR = (1 << WDIE) | (1 << WDE) | (0 << WDP3) | (1 << WDP2) | (1 << WDP1) | (0 << WDP0);
  sei();
}
ISR(WDT_vect) // Watchdog timer interrupt. 
{
  //store and event in EEPROM
  // Include your code here - be careful not to use functions they may cause the interrupt to hang and 
  // prevent a reset.
}
