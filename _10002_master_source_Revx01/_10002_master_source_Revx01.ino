 /* Johnathan Branch - PHYS 499B Project; Title: Wireless Biometric System on PCB
 * 
 * PURPOSE: Source Code to program the PCBA associated w/PCB # 10002
 *
 * The PCB:
 *  - Uses ATMEGA328P as the MCU, same as the Arduino Uno
 *  - Uart debug on PD0(RXD) and PD1(TXD)
 *  - SPI setup on PB[2-5]; SS(PB2), MOSI(PB3), MISO(PB4), SCK(PB5)
 *  - MCU reset button attached to PC6
 *  - I2C setup on PC4(SDA) and PC5(SCL)
 *  - 2 circuit interrupts; 30102_INT on PC3 and NRF51_IRQ on PD5
 *  - 1 oush button interrupt for user control attached to PD2
 *  - LCD Control pinout: PCO(RS), PC1(RW), PC2(E)
 *  - LCD Data path pinout: PB0(DB5), PB1(DB4), PD6(DB7), PD7(DB6)
 *  - LCD Backlight control: PD3(B:MSB), PD4(A:LSB)
 *  - Decoder outputs/ RGB inputs using input from the ATMega328;  Default = GREEN(11),  User_Data = BLUE(10),  Transmit_Data = RED(01)
 *  
 *  
 *  
 * created 
 * 2019 NOV
 * by Johnathan Branch
 * 
 * 
 * Revision x01 
 *  
 *
 * Derivative work from: 
 * SPARKFUN - Example5_HeartRate.ino
 * MAXIM - RD117_Arduino.ino
 * J. Branch - RGB_LGB_test.ino
 * J. Branch - LCD_Mulitpurpose_single_pushbutton_demo.ino
 * J. Branch - MAX117Test.ino
 * J. Branch - PHYS499_Arduino_PinMapping_test
 * * 
 */

// *** Libraries and Headers *** // 
#include <LiquidCrystal.h>
#include <Wire.h>
#include "MAX30105_Modif.h"
#include "heartRate.h"
#include "spo2_algorithm_Modified.h" 


// *** Constructors *** // 
LiquidCrystal lcd(A0, A2, 9, 8, 7, 6);         // constructor for the 16x2 LCD(RS,E,DB4,DB5,DB6,DB7) 
MAX30105 spo2Sensor;                           // constructor for the MAX30102 pulse oximeter sensor 


// ** Definitions and Global Variable Declarations ** // 
// *** Arduino Bit Mask *** // 
const byte B = 3;                              // Arduino bitmask for the PORT pin connected to decoder(MSB)
const byte A = 4;                              // Arduino bitmask for the PORT pin connected to decoder(LSB)
const byte button_pin = 2;                     // Arduino bitmaks for the PORT pin connected to the push button

const byte DB4 = 9;                            // Arduino bitmask for LSB of parallel data going to the LCD
const byte DB5 = 8;                            // Arduino bitmask for parallel data into the LCD
const byte DB6 = 7;                            // Arduino bitmask for parallel data into the LCD
const byte DB7 = 6;                            // Arduino bitmask for MSB of parallel data going into the LCD
int32_t spo2 = 0;                              // Intermediate SPO2 value
int spo2_Avg = 0;                              // Average spo2 value
int beatAvg = 0;                               // Average heart rate value
float beatsPerMinute = 0;                      // Intermediate heart rate value    
byte meas_counter = 0;                         // Byte counter used in main() to split timing between measurements 
bool loop_flag = false;                        // Bool used to tell when a function or loop is first called 
String rgbColor;                               // Used to control the color of the RGB backlight on the 16x2 LCD
byte usr_message;                              // Used to control the message sent to the user 
volatile byte mode = 0;                        // Used to control the progression of the switch case for the codes progression
volatile byte modeSize = 7;                    // Number of distinct modes used in the application not including the default case
char lcd_controller;                           // Char variable to control to color of the RGB
byte customChar[8] = {                         // Custom heart shape character to display on 16x2 LCD 
  B00000,
  B00000,
  B00000,
  B01010,
  B10101,
  B10101,
  B01010,
  B00100
}; 


// ** Function Prototypes ** //
void LCD_BacklightControl(char);              // void function to control the backlight of RGB display(may change to have param passed by value from Main)
void takeHRMeas();                              // Subroutine to get the heart rate values and perform averaging calculations
void takeSPO2Meas();                            // Subroutine to get the SPO2 values and perform averaging calculations
//void uartDebug();                             // void function to enable uart for debug purposes will be kept commented out during normal operation(NOT IMPLEMENTED)
// ** Bluetooth functions should go HERE ****************************************************************************************************** //


// ******************************************************************************************************************************************** // 
// ** Setup Routine ** //
void setup() {
                                                // Pin mode setup 
  pinMode(B, OUTPUT);
  pinMode(A, OUTPUT);
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  lcd.begin(16, 2);
  pinMode(button_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button_pin), isr, FALLING);


                                                // Serial setup
  Serial.begin(115200);
  //Serial.println("Initializing...");

                          // Initialize sensor
  if (!spo2Sensor.begin(Wire, I2C_SPEED_FAST))  //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }
  Serial.println(F("Place index finger on the sensor with steady pressure."));

  spo2Sensor.setup();                           // Configure sensor with default settings
  spo2Sensor.setPulseAmplitudeRed(0x0A);        // Turn Red LED to low to indicate sensor is running
}

// ******************************************************************************************************************************************** //  

// ** Main Loop ** //
void loop() 
{
  switch(mode)
  {
  case 1:                             // Start up case statement                 
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("Welcome to WBHM!"));
      lcd.setCursor(0,1);
      lcd.print(F("Version 1"));
      delay(5000);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("Push S1 to reset"));
      lcd.setCursor(0,1);
      lcd.print(F("at any time"));
      delay(5000);
      mode++;
      break;
    
  case 2:                            // User prompt to perpare for measurment case statement      
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("Place finger on"));
      lcd.setCursor(0,1);
      lcd.print(F("sensor now"));
      delay(3000);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("Push S1 to"));
      lcd.setCursor(0,1);
      lcd.print(F("take measurement"));
      delay(5000);
      break;

  case 3:                             // Heart rate measurement case statement
      LCD_BacklightControl('u');      // sets the appropriate backligh color on the RGB for data modes
      while(loop_flag == false) 
      {
       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print(F("Apply finger"));
       lcd.setCursor(0,1);
       lcd.print(F("with pressure"));
       delay(5000);
       lcd.clear();
       loop_flag = true;
      }
      takeHRMeas();
      if(meas_counter > 50)
      {
       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print(F("Latching BPM val"));
       lcd.setCursor(0,1);
       lcd.print(F("BPM: "));
       lcd.print(beatAvg);
       delay(3000);
       loop_flag = false;
       mode++;
      }
      break;

  case 4:                             // SPO2 measurement case statement 
      while(loop_flag == false)
      {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(F("Apply finger"));
        lcd.setCursor(0,1);
        lcd.print(F("lightly"));
        delay(3000);
        lcd.clear();
        loop_flag = true;
      }
      takeSPO2Meas();
      if(meas_counter > 60)
      {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(F("Latching SPO2 val"));
        lcd.setCursor(0,1);
        lcd.print(F("SPO2: "));
        lcd.print(spo2_Avg);
        delay(3000);
        meas_counter = 0;
        loop_flag = false;
        mode++;
      }
      break;    

  case 5:
        LCD_BacklightControl('b');     // test for red capability
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(F("SPO2:"));
        lcd.setCursor(5,0);
        lcd.print(spo2_Avg);
        lcd.setCursor(9,0);
        lcd.print(F("BPM:"));
        lcd.setCursor(13,0);
        lcd.print(beatAvg);
        lcd.setCursor(0,1);
        lcd.print(F("Restart? ->S2"));
        break;

  case 6:
        meas_counter = 0;
        beatAvg = 0;
        spo2_Avg = 0;
        mode = 3;
        
        break;         
    
  default:                            // Default state of the system 
      LCD_BacklightControl('d');      // sets the appropriate backlight color on the RGB for default modes
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F(" Press the user"));
      lcd.setCursor(0,1);
      lcd.print(F(" button to start"));
      delay(500);
 
  }
 
}                                      // end of Main function

void takeHRMeas()
{
                                      // Sensor Variables for Heart Rate Detection
    static const byte RATE_SIZE = 10; // Increase this for more averaging. 4 is good.(inceased to 10 in this application)
    static byte rates[RATE_SIZE];     // Array of heart rates
    static byte rate_Idx = 0;         // Index of the heart rate array
    static long lastBeat = 0;         // Time at which the last beat occurred
    static long irValue = 0;          // IR return value 
    static long delta = 0;            // Return value of the current heart rate time minus previous reading
    static float compare_beat = 0;    // Compare value for the lock-in algorithm to average heart rate values
    static byte count = 0;            // Count variable to indicate when to check lock-in criteria
    static byte compare_set = 0;      // Flag for checking when the compare value is within lock-in range(in this application it's +/-20)
    static int prevBeatAvg = -1;      // Catches the beatAvg value before exiting the subroutine
    static byte icon_pos = 7;         // Used to move the LCD icon everytime a value is pushed into the averaging array
    
    irValue = spo2Sensor.getIR();     // IR return value


  if (checkForBeat(irValue) == true)
  {
                                       // We sensed a beat!
    delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

                                       // Lock-In Averaging Algorithm
    if(beatsPerMinute < 150 && beatsPerMinute > 30)
    {
     
      if(count == 0) 
        compare_beat = beatsPerMinute;  // Get a new compare value every time count returns to zero
      
      count++;
      count %= 4;                       // Wrap around count at 4
      
      if( (compare_beat - beatsPerMinute)<= 20 && (compare_beat - beatsPerMinute)>= -20)
        compare_set = 1;                // Set HIGH is compare value is within +/- 20 window of the current BPM measurement
      
      else
        compare_set = 0;    
                                        // Serial Debug for the compare flag, counter, and value
        /*
      Serial.println(compare_set);
      Serial.print(F(" "));
      Serial.println(count);
      Serial.print(F(" "));
      Serial.println(compare_beat);
      Serial.print(F(" "));
        */
      if(count == 3 && compare_set == 1)        // Lock-In conditonal expression
        {
                                                // Serial Debug for the lock-in conditional expression
          //Serial.println(F(", Locked-In "));
          //delay(50);
          
          
          rate_Idx++;
          rates[rate_Idx] = (byte)compare_beat; // Store this reading in the array
          rate_Idx %= RATE_SIZE;                // Wrap variable

                                                // Take average of readings
          beatAvg = 0;
          for (byte x = 0 ; x < RATE_SIZE ; x++)
            beatAvg += rates[x];
          beatAvg /= RATE_SIZE;
          beatAvg += 8;                        // Calibration boost

          icon_pos = (icon_pos+1)%16;          // Wrap variable for the position of the icon 
          if(icon_pos == 0) 
            icon_pos = 7;                      // Push back to 7, in the case of modulus overflow
        }
                                               // Check to see if the beat value is changing or has settled
          if( ((beatAvg - prevBeatAvg)<= 5 && (beatAvg - prevBeatAvg)>= -5) && beatAvg != 0)
            meas_counter++;                    // Counter increment in the if case
          else
            meas_counter = 1;
  
            prevBeatAvg = beatAvg;              // Catch of the previous beat value

                                                // Serial telemetry setup
         /*   
         char beatsPerMinute_text[40];
         char text[80];
                          
         dtostrf(beatsPerMinute, 6, 2, beatsPerMinute_text);
         snprintf(text, 80, "%d,%s", beatAvg, beatsPerMinute_text);
         Serial.println(text);  
          */
    }                                           // End of Lock-In Algorithm
  }  

  
                                                // If the IR return value is low, then no finger is present
  if (irValue < 50000)
  {
   // Serial.print(F(", No finger ?"));
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("No finger ?"));
    delay(500);
    beatAvg = 0;
  }
                                                // Serial Debug Output to see the IR value, BPM, and averaged BPM
  /*
  Serial.print(F("IR="));
  Serial.print(irValue);
  Serial.print(F(", BPM="));
  Serial.print(beatsPerMinute);
  Serial.print(F(", Avg BPM="));
  Serial.print(beatAvg);
  */

                                               // LCD messages to user for the averaged BPM reading and heart display icon
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("BPM"));
  lcd.setCursor(1,1);
  lcd.print(beatAvg);
  lcd.createChar(0, customChar);
  lcd.setCursor(icon_pos,0);
  lcd.write(byte(0));
                                                
}                                               // End of HR subroutine


void takeSPO2Meas()
{
                                               // Sensor variables for the SPO2 measurement
  static const byte RATE_SIZE = 5;             // Increase this for more averaging. 4 is good.(inceased to 10 in this application)
  uint16_t irBuffer[35];                       // Infrared LED sensor data
  uint16_t redBuffer[35];                      // Red LED sensor data
  static long irValue = 0;                     // IR return value
  static const int32_t bufferLength = 35;      // Buffer length of 35 stores 0.75 seconds of samples running at 25spsc
  static uint32_t spo2_arr[10];                // Array to store the intermediate spo2 values in
  static byte spo2_idx = 0;                    // Index variable to for the spo2 array
  static byte icon_pos = 7;                    // Used to move the LCD icon everytime a value is pushed into the averaging array
  
  
  int8_t validSPO2;                            // Indicator to show if the SPO2 calculation is valid
  int32_t heartRate;                           // Heart rate value
  int8_t validHeartRate;                       // Indicator to show if the heart rate calculation is valid
 
                                               // Read the first 35 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (spo2Sensor.available() == false)    // Do we have new data?
      spo2Sensor.check();                      // Check the sensor for new data

    redBuffer[i] = spo2Sensor.getRed();
    irBuffer[i] = spo2Sensor.getIR();

    irValue = spo2Sensor.getIR();              // IR return value

    if (irValue < 25000)                       // If the IR return value is really low, then no finger is present
     {
      // Serial.print(F(", No finger ?"));
       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print(F("No finger ?"));
       delay(500);
       spo2_Avg = 0;
  }
  
   spo2Sensor.nextSample();                    // We're finished with this sample so move to next sample
  }
                                               // If the IR return value is low, then no finger is present

    //calculate heart rate and SpO2 after first 35 samples (first 750 milliseconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  if(validSPO2 == 1)                           // Check if the SPO2 value returned is valid or not
    {
      spo2_arr[spo2_idx] = spo2;               // Store this reading in the aray 
      spo2_idx++;
      spo2_idx %= RATE_SIZE;                   // Wrap variable  
 
                                               // Take average of readings
      spo2_Avg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++)
            spo2_Avg += spo2_arr[x];
           spo2_Avg = spo2_Avg/RATE_SIZE;
       meas_counter++;

          icon_pos--;                          // Variable for the position of the icon 
          if(icon_pos == 0) 
            icon_pos = 7;                      // Push back to 7, in the case of below 0 
    }
                                               // Serial debug for the SPO2 value
/*
      Serial.print(spo2_Avg);
      Serial.print(" , ");  
      Serial.print(spo2);
      Serial.println();
  */

                                               // LCD messages to user for the averaged SPO2 reading and heart display icon
  lcd.clear();
  lcd.setCursor(icon_pos, 1);
 // lcd.createChar(0, customChar);
  lcd.write(byte(0));
  lcd.setCursor(12,0);
  lcd.print(F("SPO2"));
  lcd.setCursor(13,1);
  lcd.print(spo2_Avg);

}                                              // End of SPO2 subroutine


void LCD_BacklightControl(char color)
{
  if(color == 'd')                             // Default background coloring(green) 
    {
      digitalWrite(B, HIGH);
      digitalWrite(A, HIGH);
    }
   else if(color == 'u')                       // User data background coloring(blue)
   {
      digitalWrite(B, HIGH);
      digitalWrite(A, LOW);
   }
   else if(color == 'b')                       // Bluetooth mode background coloring(red)
   {
      digitalWrite(B, LOW);
      digitalWrite(A, HIGH); 
   }

}

 
                                               // ISR to increment through the program modes via attachment to external push button on PD2
void isr() 
{
  volatile static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 75UL)  // ignores interupts for 75 milliseconds
  {
       mode = (mode+1)%modeSize;                    // Use for debug until code works, then use to control text to user
  }
  last_interrupt_time = interrupt_time;

}
