/**************************************************************************************/
/*    G. Batigne
/*      23/11/2018
/**************************************************************************************/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

#define _DEBUG_ 0 // Debug flag, mainly to manage the use of the Serial port.

Adafruit_7segment digitDisplay = Adafruit_7segment(); // Manager of the 7-segment display.

// Pins connected to measurement points:
const int V_MOTOR_L = A0;
const int U_SHUNT_L = A1;
const int V_MOTOR_R = A2;
const int U_SHUNT_R = A3;
const int RESET = 2; // The initialisation of the computation has to be reset manually.
const int D_AFF = A4; // Data pin for the display signal.
const int C_AFF = A5; // Clock pin for the display signal.

boolean isRunning = false; // Boolean used to know if the start of the race has been given.
boolean isArrived = false; // Boolean used to know if the robot is stopped. In that case,
                           // it should correspond to the final arrival. 
boolean mightStopped = false; // Boolean used to figure out if the robot is stopped.
                              // It is considered has stopped if the power stays low for
                              // a given duration (see minimumStop variable).
boolean isMOTORLSaturated = false; // Boolean used to know if that channel has reached saturation.
boolean isSHUNTLSaturated = false;
boolean isMOTORRSaturated = false;
boolean isSHUNTRSaturated = false;

const float ADCChannels = 1024.; // Total number of ADC channels.
const float ADCSaturation = ADCChannels-1; // Maximum ADC value. If a measurement gives that value there are chances that the saturation is reached.
const float ADCMaxV = 5.; // Maximum value that the ADC can measure.
const float ADCtoV = ADCMaxV/ADCChannels; // Conversion factor between ADC values and Voltage.
const float GAIN_L = 20.; // Values of the amplification gain used for current measurement.
const float GAIN_R = 20.;
const float RShunt = 0.1; // Value of the shunt resistances (in Ohm).
const float convertFactor_L = ADCtoV*ADCtoV/GAIN_L/RShunt; // Conversion factor between raw values and Power.
const float convertFactor_R = ADCtoV*ADCtoV/GAIN_R/RShunt;
const float microsToSeconds = 0.000001;
const int motorThreshold = 100; // Limit under which the motor is considered as stopped. 100 => ~ 0.5V.
const float powerThreshold = motorThreshold*(GAIN_L*convertFactor_L+GAIN_R*convertFactor_R); // Power under which the two motors might be considered as stopped.

unsigned long timeStart = 0; // Starting time of the race.
unsigned long timePrevious = 0; // Time of the previous measurement. Used to estimate the integral of power, that is energy.
unsigned long timePresent = 0; // Time of the last measurement (when running).
unsigned long lastDisplay = 0; // Time when the display was refreshed at.
unsigned long displayDuration = 500000; // Minimum time during which the display is frozen during the race (in us).
unsigned long timeDisplay = 1000; // Minimum duration to display a value after the race (in ms).
unsigned long lastStop = 0; // Last time when power was below the threshold (powerThreshold).
unsigned long minimumStop = 50000; // Duration from which the robot is consisdered as stopped (in us).

float timeRace = 0.; // Duration of the race.
float Energy = 0.; // Energy consumption.
float Power = 0.; // Instantaneous power.

void setup() {
  // put your setup code here, to run once:
  pinMode(V_MOTOR_L,INPUT); // The measurement pins are obviously inputs.
  pinMode(U_SHUNT_L,INPUT);
  pinMode(V_MOTOR_R,INPUT);
  pinMode(U_SHUNT_R,INPUT);
  pinMode(RESET,INPUT_PULLUP); // The reset pin is obviously an input. It is set to Pull-up in order to avoid
                               // the use of an extra resistance. In that case, the default state of that pin
                               // is HIGH. The push button should be connected to the ground then. An impulse 
                               // should set the pin state to LOW and then trigger a reset.
  pinMode(D_AFF,OUTPUT); // Data pin for the display signal.
  pinMode(C_AFF,OUTPUT); // Clock pin for the display signal.
#if _DEBUG_
  Serial.begin(9600);
#endif
  digitDisplay.begin(0x70); // Set-up of the address of the display.
  initRace(); // Initialisation of the measurement (energy and booleans).
}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(RESET)==LOW) initRace(); // Initialisation when a reset command is received.
  if (isRunning) { // If running, compute the power (and energy) and display it.
    updatePower();
    if (timePresent-lastDisplay > displayDuration) {
      lastDisplay = timePresent;
      displayPower();
    }
  } else {
    if (isArrived) { // When arrived, display the race time and the energy consumption.
      displayTime();
      displayEnergy();
      displayName();
      }
    else { // This instructions are done after a reset and before the beginning of the race.
      while (analogRead(V_MOTOR_L)<motorThreshold && analogRead(V_MOTOR_R)<motorThreshold) { // Waiting for the start.
        delay(1);
        }
      isRunning = true;
      timeStart = micros();
#if _DEBUG_ 
      Serial.println("Start: " +String(float(timeStart)*microsToSeconds));
#endif
    }
  }
}

void updatePower() { // Compute the power and energy from the measurements. 
                     // Detect when the robot is stopped (and supposed to be arrived).
  float vl = analogRead(V_MOTOR_L); // Get value of the Left Motor.
  float sl = analogRead(U_SHUNT_L); // Get value of the Left Shunt.
  float vr = analogRead(V_MOTOR_R); // Get value of the Right Motor.
  float sr = analogRead(U_SHUNT_R); // Get value of the Right Shunt.

  // Check if saturation has been reached on each channel. If it has been reached once, the boolean stays at True (keep in memory). 
  if (vl==ADCSaturation && !isMOTORLSaturated) isMOTORLSaturated = true;
  if (sl==ADCSaturation && !isSHUNTLSaturated) isSHUNTLSaturated = true;
  if (vr==ADCSaturation && !isMOTORRSaturated) isMOTORRSaturated = true;
  if (sr==ADCSaturation && !isSHUNTRSaturated) isSHUNTRSaturated = true;
  
  timePresent = micros();
#if _DEBUG_
  Serial.println(String(vl) + " " + String(sl) + " / " + String(vr) + " " + String(sr));  
#endif
  Power = (vl-sl/GAIN_L)*sl*convertFactor_L + (vr-sr/GAIN_R)*sr*convertFactor_R;
  Energy += Power*float(timePresent-timePrevious)*microsToSeconds; // Integration of the power to compute the energy.
  timePrevious = timePresent;
  
  if (Power<powerThreshold) { // The two motors are stopped. The robot might get stuck which means that it has finished the race.
    if (mightStopped) { // If the power stays below its threshold,
      if (timePresent-lastStop > minimumStop) { // and if the motors are stopped for long enough,
        isRunning = false; // the robot is definitely stopped
        isArrived = true;  // and the race is finished.
        timeRace = float(lastStop-timeStart)*microsToSeconds;
      }
    } else { // First time when the power is 0. No stopping yet since it can be due to some electronic noise.
      mightStopped = true;
      lastStop = timePresent; // Time when the motors have been seen as stopped for the first time.
    }
  }
  else {
    mightStopped = false;
  }    
}

void displayPower() { // The display gives the value in mW (xxxx) or the saturation code eventually.
#if _DEBUG_
  Serial.println("Power: " + String(Power));
#endif
  boolean drawDots = false;
  if (isMOTORLSaturated || isSHUNTLSaturated || isMOTORRSaturated || isSHUNTRSaturated) { 
    // If saturation has been reached, "1" is displayed and "-" if not.
    // The position of this character depends on the channel that triggered a saturation.
    if (isMOTORLSaturated) {
      digitDisplay.writeDigitNum(0,1,drawDots); // Display "1" at position 0
    } else {
      digitDisplay.writeDigitRaw(0,0x40); // Display "-" at position 0
    }
    if (isSHUNTLSaturated) {
      digitDisplay.writeDigitNum(1,1,drawDots);
    } else {
      digitDisplay.writeDigitRaw(1,0x40);
    }
    digitDisplay.drawColon(drawDots);
    if (isMOTORRSaturated) {
      digitDisplay.writeDigitNum(3,1,drawDots);
    } else {
      digitDisplay.writeDigitRaw(3,0x40);
    }
    if (isSHUNTRSaturated) {
      digitDisplay.writeDigitNum(4,1,drawDots);
    } else {
      digitDisplay.writeDigitRaw(4,0x40);
    }
  } else { // If no saturation has been reached so far.
    digitDisplay.writeDigitNum(0,int(Power)%10,drawDots);
    digitDisplay.writeDigitNum(1,int(Power*10.)%10,drawDots);
    digitDisplay.drawColon(drawDots);
    digitDisplay.writeDigitNum(3,int(Power*100.)%10,drawDots);
    digitDisplay.writeDigitNum(4,int(Power*1000.)%10,drawDots);
  }
  digitDisplay.writeDisplay();
}

void displayTime() { // The display gives the value in seconds and hundredth of seconds (s:cs)
#if _DEBUG_
  Serial.println("Time: " + String(timeRace));
#endif
  boolean drawDots = true;
  digitDisplay.writeDigitNum(0,int(timeRace/10),drawDots);
  digitDisplay.writeDigitNum(1,int(timeRace)%10,drawDots);
  digitDisplay.drawColon(drawDots);
  digitDisplay.writeDigitNum(3,int(timeRace*10.)%10,drawDots);
  digitDisplay.writeDigitNum(4,int(timeRace*100.)%10,drawDots);
  digitDisplay.writeDisplay();
  delay(timeDisplay);
}

void displayEnergy() { // The display gives the value in J (xxx.y).
#if _DEBUG_
  Serial.println("Energy: " + String(Energy));
#endif
  boolean drawDots = false;
  digitDisplay.writeDigitNum(0,int(Energy/100.),drawDots);
  digitDisplay.writeDigitNum(1,int(Energy/10.)%10,drawDots);
  digitDisplay.writeDigitRaw(2,0x10); // Draw the upper right dot to indicate the decimals.
  digitDisplay.writeDigitNum(3,int(Energy)%10,drawDots);
  digitDisplay.writeDigitNum(4,int(Energy*10.)%10,drawDots);
  digitDisplay.writeDisplay();
  delay(timeDisplay);
}
  
void initRace() { // Initialisation of the energy and the booleans.
                  // Called before starting the race (reset).
  Energy = 0.;
  isRunning = false;
  isArrived = false;
  mightStopped = false;
  isMOTORLSaturated = false;
  isSHUNTLSaturated = false;
  isMOTORRSaturated = false;
  isSHUNTRSaturated = false;
  displayName();
}

void displayName() { // Display the team reference.
  digitDisplay.writeDigitRaw(0,0x73); // P
//  digitDisplay.writeDigitRaw(1,0x77); // R
  digitDisplay.writeDigitRaw(1,0x50); // r
  digitDisplay.writeDigitRaw(2,0x00); // no dots
//  digitDisplay.writeDigitRaw(3,0x3F); // O
  digitDisplay.writeDigitRaw(3,0x5C); // o
  digitDisplay.writeDigitRaw(4,0x71); // F
  digitDisplay.writeDisplay();
  delay(timeDisplay);
}
  // Bit-LED correspondance:
  //         0      The middle LED is bit n°6
  //         __
  //      5 |  | 1    Example: display of 5
  //        |__|        bit position:   6 5 4 | 3 2 1 0
  //      4 |  | 2      value (BIN):    1 1 0 | 1 1 0 1 => 0b1101101
  //        |__|        value (HEX):      6   |  13=D   => 0x6D
  //          3         value (DEC):    109


