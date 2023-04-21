#include "arduinoFFT.h"
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>
#define steps 200

#define SAMPLES 128 //max amount of samples for arduino uno
#define SAMPLING_FREQUENCY 2048 //must be 2*highest expected frequency, A is 440Hz and after 630Hz seems like it's about to snap

#define MAXFREQ 630.00
#define MINFREQ 220.00
#define printTolerance 10
#define tolerance 6.0

#define NTUNEALL 19
#define NTUNE 4

#define analogPin A0//read the analog input -> frequency
#define digitalPin 5//read the digital input -> activation
#define redLedPin 3//red led pin 
#define greenLedPin 4//green led pin

#define btnPin 2//switch pin

const float tunesFreq[NTUNEALL] = {220.00, 233.08, 246.94, 261.63, 277.18, 293.66, 311.13, 329.63, 349.23, 369.99, 392.00, 415.30, 446.00, 466.16, 493.88, 523.25, 554.37, 587.33, 622.25};
const char* tunesAll[NTUNEALL] = {"A3", "A3#", "B3", "C4", "C4#", "D4", "D4#", "E4", "F4", "F4#", "G4", "G4#", "A4", "A4#", "B4", "C5", "C5#", "D5", "D5#"};

arduinoFFT FFT = arduinoFFT();

//G C E A 
const int step[NTUNE] = {12, 10, 6, 8};
const int dir[NTUNE] = {13, 11, 7, 9};

unsigned int samplingPeriod;
unsigned long microSeconds;

const char* sel = "Selected: ";
const char* tunes[NTUNE] = {"G4", "C4", "E4","A4"};
const float freqs[NTUNE] = {392.00, 261.63, 329.63, 446.00};

int currBtnState;
int prevBtnState;
int currTune = 0;
int direction = 1;

bool tuning = false;

//LCD_I2C lcd(0x27);
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup(){
  Serial.begin(115200);
  lcd.init();
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print(sel);
  lcd.setCursor(strlen(sel), 0);
  lcd.print(tunes[currTune]);
  
  pinMode(analogPin, INPUT);
  pinMode(digitalPin, INPUT);

  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);

  pinMode(btnPin, INPUT_PULLUP);
  currBtnState = digitalRead(btnPin);
  prevBtnState = currBtnState;

  samplingPeriod = round(1000000*(1.0/SAMPLING_FREQUENCY));
  digitalWrite(redLedPin, LOW);
  digitalWrite(greenLedPin, LOW);

  for(int i = 0; i < NTUNE; i++){
    pinMode(step[i], OUTPUT);
    pinMode(dir[i], OUTPUT);
  }
}

void blink(int led){
  digitalWrite(led, HIGH);
  delay(1000);
  digitalWrite(led, LOW);
  delay(200);
}

void switchNote(){
  currBtnState = digitalRead(btnPin);
  if(currBtnState != prevBtnState){
    if(currBtnState){
      currTune += 1;
      currTune %= NTUNE;
      lcd.setCursor(strlen(sel), 0);
      lcd.print(tunes[currTune]);
      for(int i = 0; i < 3; i++){
        lcd.setCursor(i,1); //deleting the # in case it exists
        lcd.print(" ");
      }
    }
    prevBtnState = currBtnState;
    delay(200);
  }
}

bool running = false;

void loop (){

  switchNote();
  //redeclared everytime for memory
  double vReal[SAMPLES]; //array of real samples
  double vImg[SAMPLES]; //array of img samples
  int val = digitalRead(digitalPin);
  if(val == HIGH){
    for(int i = 0; i < SAMPLES; i++){
      microSeconds = micros();
      
      vReal[i] = analogRead(analogPin);
      vImg[i] = 0;

      while(micros() < (microSeconds + samplingPeriod)){
        //do nothing
      }
    }
    //process to FFT the samples
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HANN, FFT_FORWARD);
    FFT.Compute(vReal, vImg, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImg, SAMPLES);

    //find the "main" frequency
    double freq = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);

    //working on the frequency
    //frequency out of bounds
    if( freq < MINFREQ || freq > MAXFREQ )
    return;

    //select played frequency to print
    int i = 0;
    while(i<NTUNEALL && (freq < tunesFreq[i]-printTolerance || freq > tunesFreq[i]+printTolerance))
      i += 1;
    //print on lcd
    lcd.setCursor(2,1); //deleting the # in case it exists
    lcd.print(" ");
    lcd.setCursor(0,1); //moving the cursor back to the beginning and printing
    lcd.print(tunesAll[i]);
    Serial.println(freq);
    //if played tune is the same as the selected one
    if((freqs[currTune]-tolerance < freq) && (freqs[currTune]+tolerance > freq) && i != NTUNEALL)
      blink(greenLedPin);
    else{
      blink(redLedPin);
      tuning = true;
      

      if(freqs[currTune]-tolerance > freq){ //i'm low
        digitalWrite(dir[currTune], 1);
      }
      if(freqs[currTune]+tolerance < freq){//i'm high
        digitalWrite(dir[currTune], 0);
      } 
      Serial.println("need to tune");

      // Spin the stepper motor 1 revolution slowly:
      for(int i = 0; i < steps; i++) //for(int i = 0; i < steps*multiplier dato dalla distanza; i++)
      {
        // These four lines result in 1 step:
        digitalWrite(step[currTune], HIGH);
        delayMicroseconds(2000);
        digitalWrite(step[currTune], LOW);
        delayMicroseconds(2000);
      }
    }
  }

}
