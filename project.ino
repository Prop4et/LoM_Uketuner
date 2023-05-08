#include "arduinoFFT.h"
#include <LiquidCrystal_I2C.h>
//#include <AccelStepper.h>

#define SAMPLES 128 //max amount of samples for arduino uno
#define SAMPLING_FREQUENCY 2048 //must be 2*highest expected frequency, A is 440Hz and after 630Hz seems like it's about to snap
#define SCREENLEN 17
#define FULLSTEPS 400
#define MAXFREQ 500.00
#define MINFREQ 225.00

#define PRINTTOLERANCE 10

#define NTUNEALL 14
#define NTUNE 4

#define analogPin A0//read the analog input -> frequency
#define digitalPin 6//read the digital input -> activation
#define redLedPin 5//red led pin 
#define greenLedPin 4//green led pin

#define btnPin 3 //switch pin
#define rotatePin 2//pin to rotate the motor
#define potenPin A1 //potentiometer reading pin
 //complete circle at half step

#define stepPin 12;
#define dirPin 11;
int delayRotation = 0;
//AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);

//save variables for the notes (cannot loop unfortunately) from B3 to A4#

const float printTolerances[NTUNEALL] = {2, 2, 3.3, 5, 4, 4, 6, 6, 5.5, 8, 6, 8.5, 7, 6};
const float tunesFreq[NTUNEALL] = {233.08, 246.94, 261.63, 277.18, 293.66, 311.13, 329.63, 349.23, 369.99, 392.00, 415.30, 440.00, 466.16, 493.88};
const char* tunesAll[NTUNEALL] = {"A3", "B3", "C4", "C4#", "D4", "D4#", "E4", "F4", "F4#", "G4", "G4#", "A4", "A4#", "B4"};


unsigned int samplingPeriod;
unsigned long microSeconds;

//G C E A


char dest[30];
const char* tunes[NTUNE] = {"G4", "C4", "E4", "A4"};
const float freqs[NTUNE] = {392.00, 261.63, 329.63, 440.00};
const float tolerances[NTUNE] = {8, 5.3, 6, 10.0};
const float ranges[NTUNE] = {1.5, 0.8, 0.6, 1.2};

int currBtnState;
int prevBtnState;
int rotateBtnState;
int prevRotateBtnState;
int currTune = -1;

//bool tuning = false;
bool buttonPressedFirst = false; //0 never pressed, 1 pressed 1 time, 2 do nothing
bool freqOOB = false;
const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;

unsigned long scrollTime = 0;
//LCD_I2C lcd(0x27);
LiquidCrystal_I2C lcd(0x27, 16, 2);
char beginSentence[] = "Press the button to start ";
int sentenceCounter = 0;
void setup() {
  //Setup LCD
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(beginSentence);

  //setup mic pin
  pinMode(analogPin, INPUT);
  pinMode(digitalPin, INPUT);

  //setup led pins
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  digitalWrite(redLedPin, LOW);
  digitalWrite(greenLedPin, LOW);
  
  //setup button pin
  pinMode(btnPin, INPUT_PULLUP);
  currBtnState = digitalRead(btnPin);
  prevBtnState = currBtnState;

  pinMode(rotatePin, INPUT_PULLUP);
  rotateBtnState = digitalRead(rotatePin);
  prevRotateBtnState = rotatePin;
  
  
  //setup fastfft parameters
  samplingPeriod = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
  
  //setup motor parameters
  scrollTime = millis();
}


int scrollFun2(char* sentence, int len){
  lcd.setCursor(0,0);
  lcd.print("                       ");
  delayMicroseconds(2000);
  lcd.setCursor(0,0);
  int i = sentenceCounter;
  int j = 0;
  while(j < SCREENLEN){
    lcd.print(sentence[i]);
    i += 1;
    i = i % len;
    lcd.setCursor(j, 0);
    j++;
  }
  sentenceCounter += 1;
  sentenceCounter %= len;
}


void blink(int led) {
  digitalWrite(led, HIGH);
  unsigned long curr = millis();
  while(millis() - curr < 2000){
    if(millis() - scrollTime>= 500){
      scrollFun2(dest, strlen(dest));
      scrollTime = millis();
    }
  }
  digitalWrite(led, LOW);
}
void spin(int dir, int stepDiff, float freqDiff){
  
  int stepCounter = 0;
  digitalWrite(dirPin, dir);
  if(stepDiff >= 2){
    int appSteps = FULLSTEPS*(int(stepDiff/2));
    
    while(stepCounter < appSteps){
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(1000);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(1000);
      //cannot put print in here cause it's heavy
      stepCounter += 1;
    }
  }
  else{
    int steps;
    if(freqDiff > 17)
      steps = 200;
    else
      steps = random(10,15)*freqDiff;
    
    
    while(stepCounter < steps){
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(1000);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(1000);
      //cannot put print in here cause it's heavy
      stepCounter += 1;
    }
  }
  stepCounter = 0;
}

int speedUp() {

  int customDelay = analogRead(potenPin); // Reads the potentiometer
  int newCustom = map(customDelay, 0, 1023, 500, 2000); // Convrests the read values of the potentiometer from 0 to 1023 into desireded delay values (300 to 4000)
  return newCustom;  

}

void rotateMotor(){
  rotateBtnState = digitalRead(rotatePin);
  if (rotateBtnState != prevRotateBtnState) {
    digitalWrite(dirPin, 1);
    while(!rotateBtnState){
      int appRotation = speedUp();
      if(abs(delayRotation - appRotation) > 150){
        delayRotation = appRotation;
      }
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(delayRotation);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(delayRotation);
      rotateBtnState = digitalRead(rotatePin); 
    }
    prevRotateBtnState = rotateBtnState;    
  }
}

void switchNote() {
  currBtnState = digitalRead(btnPin);
  if (currBtnState != prevBtnState) {
    if (!currBtnState) {
      buttonPressedFirst = true;
      lcd.clear();
      sentenceCounter = 0;
      currTune += 1;
      currTune %= NTUNE;
      strncpy(dest, "Selected: ", 12);
      strncat(dest, tunes[currTune], strlen(tunes[currTune]));
      strcat(dest, "(");
      int fint = (int)freqs[currTune];
      char app[3] = {" "};
      sprintf(app, "%d", fint);
      strncat(dest, app, 3);
      strcat(dest, ".00)  ");
      lcd.print(dest);
      for (int i = 0; i < 3; i++) {
        lcd.setCursor(i, 1); //deleting the # in case it exists
        lcd.print(" ");
      }
    }
    prevBtnState = currBtnState;
    delayMicroseconds(200);
  }
}



void loop () {
  switchNote();
  rotateMotor();
  if(!buttonPressedFirst){
    if(millis() - scrollTime>= 500){
      scrollFun2(beginSentence, strlen(beginSentence));
      scrollTime = millis();
    }
    return;
  }
  
  if(millis() - scrollTime>= 500){
    scrollFun2(dest, strlen(dest));
    scrollTime = millis();
  }
  //redeclared everytime for memory
  double vReal[SAMPLES]; //array of real samples
  double vImg[SAMPLES]; //array of img samples
  int val = digitalRead(digitalPin);

  if (val == HIGH) {
    for (int i = 0; i < SAMPLES; i++) {
      microSeconds = micros();

      vReal[i] = analogRead(analogPin);
      vImg[i] = 0;

      while (micros() < (microSeconds + samplingPeriod)) {
        //do nothing
      }
    }

    freqOOB = false;
    //process to FFT the samples
    arduinoFFT FFT = arduinoFFT(vReal, vImg, SAMPLES, SAMPLING_FREQUENCY);
    FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(FFT_FORWARD);
    FFT.ComplexToMagnitude();

    //find the "main" frequency
    double freq = FFT.MajorPeak();
    Serial.println(freq);
    //working on the frequency
    //frequency out of bounds
    if ( freq < MINFREQ || freq > MAXFREQ ){
      if(!freqOOB){
        lcd.setCursor(0, 1); //moving the cursor back to the beginning and printing
        lcd.print("Frequency OOB");
        freqOOB = true;
      }
    }
    
    //select played frequency to print
    int i = 0;
    if(!freqOOB){
      while (i < NTUNEALL-1 && !(freq >= tunesFreq[i] + printTolerances[i] - PRINTTOLERANCE && freq < tunesFreq[i+1]))
        i += 1;
      freq - tunesFreq[i] < tunesFreq[i+1] - freq ? i : i+=1;
      freqOOB = false;
      //print on lcd
      lcd.setCursor(0, 1); //deleting the # in case it exists
      lcd.print("              ");
      delayMicroseconds(2000);
      lcd.setCursor(0, 1); //moving the cursor back to the beginning and printing
      
      lcd.print(tunesAll[i]);
      //if played tune is the same as the selected one
    }
    if (!freqOOB && (freqs[currTune] + tolerances[currTune] - ranges[currTune] <= freq) && (freqs[currTune] + tolerances[currTune] + ranges[currTune] >= freq) && i != NTUNEALL){
      blink(greenLedPin);  
    }
    else if(!freqOOB) {
      float freqDiff = abs(freq - (freqs[currTune]+tolerances[currTune]));
      int dir = 0; 
      blink(redLedPin);
      if(currTune == 1 && freq > 360){
         spin(1, 4, 0);
         return;
      }
      //tuning = true;
      int stepDiff = 0;
      int findex = i;
      if (freqs[currTune]+ tolerances[currTune] < freq) { //selected is less than played
        dir = 0;
        while(findex >= 0 && tunesFreq[findex] > freqs[currTune]){
          
          stepDiff+=1;
          findex-=1;
        }
      }
      if (freqs[currTune] + tolerances[currTune]/1.5 > freq) { //selected is higher than played 

        dir = 1;

        while(findex < NTUNEALL && tunesFreq[findex] < freqs[currTune]){
          
          stepDiff+=1;
          findex+=1;
          
        }
      }
      spin(dir, stepDiff, freqDiff);
      
      
    }
  }

}
