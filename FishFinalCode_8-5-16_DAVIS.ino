//***************Weak electric fish JAR  project **************************************
//This code generate sinus signal of arbitrary frequency at A0 pin and records Week electric fish signal
//at pin A1. 
//Procedure:
//Generate signal for PERIOD_OF_MEASUREMENTS_IN_SEC seconds
//Stops generating signal
//Meausre signal on A1 pin for one second
//calculate mean of measured signal
//calculate frequency of measured signal by counting number of times signal crosses mean value

int timeCounter = 0;

#include "Arduino.h"
#include "wiring_private.h"

#define PERIOD_OF_MEASUREMENTS_IN_SEC 5 //change this to set length of interval for stimulation (in seconds)

//code can be in one of the two modes MODE_STIMULATION or MODE_MEASUREMENT
//when we want to change mode we set value of "mode" variable to one of the 
//two constant values so that rest of the code knows that we are in particular mode 
#define MODE_STIMULATION 1
#define MODE_MEASUREMENT 2
int mode = MODE_STIMULATION;

//our sampling frequency for recording is 10kHz
#define SAMPLING_FREQUENCY_FOR_MEASUREMENTS 10000


volatile int sIndex; //Tracks sinewave points in array
int samplesPerBlock = 100;//nummber of points in sine wave
int *wavSamples; //array to store sinewave points

// EDITED EDITED EDITED EDITED EDITED
uint32_t sampleRate = 20000; //sample rate of the sine wave - (changed for testing)
double follow = 2; //in Hertz, set for difference between fish frequency and stimulus on button push

//when this counter hits zero we are measuring 
//frequency and than we reset counter again
//Basicaly we measure PERIOD_OF_MEASUREMENTS_IN_SEC time with this
//we countdown this on every TC5_Handler call during stimulation
long int timerCounterForMeasurements = 0;

//this variable will hold initial value of timerCounterForMeasurements counter
//so that we can set to timerCounterForMeasurements every time we start stimulation period
long int initialValueOfTimerCounter = 0;

//debug output pin
//not related to functionality 
#define PIN 10

//EDITED EDITED EDITED EDITED EDITED
//LCD screen display    
#include <SoftwareSerial.h>
SoftwareSerial mySerial(3,2); // pin 2 = TX, pin 3 = RX (unused)

//EDITED EDITED EDITED EDITED EDITED
// Setting button pins for chasing frequencies 
const int lowerFrequencyPin = 3;     // the number of the pushbutton pin 4
const int higherFrequencyPin =  4;   // the number of the LED pin        7
const int testPin1 = 5;
const int testPin2 = 6;

int state = 1;

//EDITED EDITED EDITED EDITED EDITED
int buttonState1 = 0;         // variable for reading the pushbutton status EDITED
int buttonState2 = 0;
int buttonState3 = 0;
int buttonState4 = 0;


//-------------variables used for measurement mode below

//keeps track of number of samples we measured by now. It is incremented every time we c another sample
//when "sampleCount" hits SAMPLING_FREQUENCY_FOR_MEASUREMENTS we measured for 1 second
long sampleCount = 0; 

//counts number of rising crossings
int flipCount = 0;

//holds value of previous sample
//so that we can check if we crossed the mean value
int prevSample = 0;

//current estimated frequency
float currentFrequency = 0.0;

//If we estimate frequency just based on the number of crossings we
//will have significant error since we are counting just complete periods of 
//sinus signals. In addition to count of complete periods we add number of samples
//in incomplete periods at the begining of recording and at the end of recording.
//In that way we can acchieve sub 1Hz precision.
//These tree variables are used for that
int numberOfSamplesBeforeFirstCross;
int numberOfSamplesAfterLastCross;
bool firstCrossHappened;

//pin used for recording of signal
uint32_t anaPin = A1;

//variable that holds average/mean value
//initialy it is at 2000 as a rough estimate of half of the 
//interval of 12bit ADC (~ (2^12)/2)
float average = 2000;


//-------------------------------- Bunch of helper functions - ignore ----------------------
// This is an C/C++ code to insert repetitive code sections in-line pre-compilation
// Wait for synchronization of registers between the clock domains
// ADC
static __inline__ void ADCsync() __attribute__((always_inline, unused));
static void   ADCsync() {
  while (ADC->STATUS.bit.SYNCBUSY == 1); 
}
static void syncGCLK() {
  while (GCLK->STATUS.bit.SYNCBUSY == 1); //Just wait till the ADC is free
}

uint32_t valueRead;

#ifdef _VARIANT_ARDUINO_ZERO_
volatile uint32_t *setPin = &PORT->Group[g_APinDescription[PIN].ulPort].OUTSET.reg;
volatile uint32_t *clrPin = &PORT->Group[g_APinDescription[PIN].ulPort].OUTCLR.reg;
const uint32_t  PinMASK = (1ul << g_APinDescription[PIN].ulPin);
#endif

//-------------------------------- End of Bunch of helper functions ----------------------




//------------------------------------------ SETUP ------------------------------------------
//setup is executed only once after powering up Zero
void setup() {

  //EDITED EDITED EDITED EDITED EDITED
  pinMode(higherFrequencyPin, INPUT); //Set buttons as input
  pinMode(lowerFrequencyPin, INPUT);
  pinMode(testPin1, INPUT);
  pinMode(testPin2, INPUT);
  
  Serial.begin(115200); 
  delay(500); // wait for display to boot up
  mySerial.begin(9600); // set up serial port for 9600 baud
  mySerial.write("                "); //Clear the display
  mySerial.write("                ");
  
  //Write stimulus and fish frequencies to the LCD display
  mySerial.write("S: "); 
  mySerial.print((double)sampleRate/100);
//  mySerial.write("    ");
  mySerial.write(" T: ");
  mySerial.print(timeCounter);
  
  mySerial.write("  F: ");
  mySerial.print(currentFrequency);
  
  //set the debug pin as output
  pinMode(PIN, OUTPUT); 
  digitalWrite(PIN,LOW);
  
  analogWriteResolution(10); //set the Arduino DAC for 10 bits of resolution (max)
  getSinParameters(); //get sinewave parameters from user on serial monitor

  /*Allocate the buffer where the samples are stored*/
  wavSamples = (int *) malloc(samplesPerBlock * sizeof(int));
  genSin(samplesPerBlock); //function generates one period of sine wave

  //init PERIOD_OF_MEASUREMENTS_IN_SEC counter
  //we countdown this in TC5_Handler during stimulation
  timerCounterForMeasurements = initialValueOfTimerCounter;

  //setup measurements
  selAnalog(anaPin);//setup analog input
  fastADCsetup();//setup ADC to very high speed

  //init variables for cont of leftower samples to zero
  //prepare for first meausrement of frequency
  numberOfSamplesBeforeFirstCross = 0;
  numberOfSamplesAfterLastCross = 0;
  firstCrossHappened = false;
  
}




//------------------------------------------ MAIN LOOP ------------------------------------------
void loop() {

//EDIT FOR NEW BUTTON FUNCTIONALITY
buttonState1 = digitalRead(higherFrequencyPin); //Read pins on button push
buttonState2 = digitalRead(lowerFrequencyPin);
buttonState3 = digitalRead(testPin1);
buttonState4 = digitalRead(testPin2);
     
     if (buttonState1 == HIGH){ //Button 1 for reducing stimulus frequency to a non-consequential value (can change to 0)
  // sampleRate = (currentFrequency*100) + (follow*100); //Change stimulus to fish frequency + follow
     sampleRate = 20000;
     initialValueOfTimerCounter = sampleRate * PERIOD_OF_MEASUREMENTS_IN_SEC;

     delay(300);
     mySerial.write("                   "); // move cursor to beginning of first line
     mySerial.write(254);
     mySerial.write(128);
     mySerial.write("S: ");
     mySerial.print((double) sampleRate/100);
     mySerial.write(" T: ");
     mySerial.print(timeCounter);
     
     
     mySerial.write("  F: ");
     mySerial.print(currentFrequency);
     
     Serial.println(sampleRate/100);
     }
     
     if (buttonState2 == HIGH){ //Button 2 for chasing with a stimulus frequency below the fish
     sampleRate = currentFrequency*100 - (follow*100); //Set stimulus to fish frequency - follow
     initialValueOfTimerCounter = sampleRate * PERIOD_OF_MEASUREMENTS_IN_SEC;
     delay(300); 
     mySerial.write("                   ");  // move cursor to beginning of first line
     mySerial.write(254);
     mySerial.write(128);
     mySerial.write("S: ");
     mySerial.print((double) sampleRate/100);
     mySerial.write(" T: ");
     mySerial.print(timeCounter);


     mySerial.write("  F: ");
     mySerial.print(currentFrequency);
     
     Serial.println(sampleRate/100);
     }  

if (buttonState3 == HIGH){ //Button 3 in testing 
     sampleRate = currentFrequency*100 - (follow*200); //Set stimulus to fish frequency - follow
     initialValueOfTimerCounter = sampleRate * PERIOD_OF_MEASUREMENTS_IN_SEC;
     delay(300); 
     mySerial.write("                   ");  // move cursor to beginning of first line
     mySerial.write(254);
     mySerial.write(128);
     mySerial.write("S: ");
     mySerial.print((double) sampleRate/100);
     mySerial.write(" T: ");
     mySerial.print(timeCounter);


     mySerial.write("  F: ");
     mySerial.print(currentFrequency);
     
     Serial.println(sampleRate/100);
     }


if (buttonState4 == HIGH){ //Button 4 in testing
     sampleRate = currentFrequency*100 + (follow*200); //Set stimulus to fish frequency - follow
     initialValueOfTimerCounter = sampleRate * PERIOD_OF_MEASUREMENTS_IN_SEC;
     delay(300); 
     mySerial.write("                   ");  // move cursor to beginning of first line
     mySerial.write(254);
     mySerial.write(128);
     mySerial.write("S: ");
     mySerial.print((double) sampleRate/100);
     mySerial.write(" T: ");
     mySerial.print(timeCounter);


     mySerial.write("  F: ");
     mySerial.print(currentFrequency);
     
     Serial.println(sampleRate/100);
     }






  if(mode == MODE_STIMULATION)
  {
      sIndex = 0;   //Set to zero to start from beginning of waveform
      tcConfigure(); //setup the timer counter based off of the user entered sample rate
      //loop until all the sine wave points have been played
      while (sIndex<samplesPerBlock)
      { 
          //we have to check every line if we are in stimulation mode (if(mode == MODE_STIMULATION)
          //since mode can be changed in interrupt TC5_Handler at any time and
          //we don't want to continue executing code for stimulation while in meaurement mode
          if(mode == MODE_STIMULATION)
          {
            tcStartCounter(); //start timer, once timer is done interrupt will occur and DAC value will be updated
          }
      }
      if(mode == MODE_STIMULATION)
      {
          //disable and reset timer counter
          tcDisable();
      }
      if(mode == MODE_STIMULATION)
      {
          tcReset();
      }
  }
  else   //mode measurements
  {
      //only this else is executing during measurement mode
      //we don't need to do anything here. All logic for meaurement 
      //is in TC5_Handler that is periodicaly executed
      
      
  }
}

//------------------------------------------ CALCULATE SIN WAVE ------------------------------------------
//This function generates a sine wave and stores it in the wavSamples array
//The input argument is the number of points the sine wave is made up of
void genSin(int sCount) {
 const float pi2 = 6.28; //2 x pi
 float in; 
 
 for(int i=0; i<sCount;i++) { //loop to build sine wave based on sample count
  in = pi2*(1/(float)sCount)*(float)i; //calculate value in radians for sin()
  wavSamples[i] = ((int)(sin(in)*511.5 + 511.5)); //Calculate sine wave value and offset based on DAC resolution 511.5 = 1023/2
 }


   //calculate how many samples we need to count
  //to be able to wait for PERIOD_OF_MEASUREMENTS_IN_SEC
  //it depends on sample rate of stimulation that's why we calculate it here.
  //I know that every time we change the sample rate we will have to recalculate sinus 
  //so this is the right place
  initialValueOfTimerCounter = sampleRate * PERIOD_OF_MEASUREMENTS_IN_SEC;
}


//------------------------------------------ GET PARAMETERS FROM USER ------------------------------------------
//This function handles getting and setting the sine wave parameters from 
//the serial monitor. It is important to use the Serial.end() function
//to ensure it doesn't mess up the Timer counter interrupts later

//EDITED EDITED EDITED EDITED EDITED
void getSinParameters() {
  Serial.begin(115200);
  //Serial.println("Enter desired frequency (range 100-1000)");

  //sample rate is frequency multiplied with number of samples per period
  //Note that we fixed number of samples per period to 100
  //sampleRate = readParameter()*100;
  sampleRate = 20000;

  if (sampleRate < 100 || sampleRate > 100000) sampleRate = 100000;
 

  Serial.println((double)sampleRate / 100);
  Serial.println("Generating sine wave...");
  //Serial.end();
}

//waits for serial data and reads it in. This function reads in the parameters
// that are entered into the serial terminal
int readParameter() {
 while(!Serial.available());
 return Serial.parseInt(); //get int that was entered on Serial monitor
}

//------------------------------------------ TIMER / COUNTER CODE ------------------------------------------

// Configures the TC timer to call TC5_Handler at certain frequency.
//i.e. sampleRate in stimulation mode and SAMPLING_FREQUENCY_FOR_MEASUREMENTS in measurement mode
//We must call this timer configuration function every time we change the mode. It will check the mode 
// and set timer frequency correctly
 void tcConfigure()
{
     // Enable GCLK for TCC2 and TC5 (timer counter input clock)
     GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
     while (GCLK->STATUS.bit.SYNCBUSY);
    
      tcReset(); //reset TC5
    
      // Set Timer counter Mode to 16 bits
     TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
     // Set TC5 mode as match frequency
     TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
     //set prescaler and enable TC5
     TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;
    
    
     //!!!Here we set up timer to different period for stimulation mode (sampleRate) and for measurement mode (SAMPLING_FREQUENCY_FOR_MEASUREMENTS)
     if(mode == MODE_STIMULATION)
     {
       //set TC5 timer counter based off of the system clock and the user defined sample rate or waveform
        TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate - 1);
     }
     else
     {
       //set TC5 timer counter based off of the system clock and SAMPLING_FREQUENCY_FOR_MEASUREMENTS
        TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / SAMPLING_FREQUENCY_FOR_MEASUREMENTS - 1);
     }
     while (tcIsSyncing());
     
     // Configure interrupt request
     NVIC_DisableIRQ(TC5_IRQn);
     NVIC_ClearPendingIRQ(TC5_IRQn);
     NVIC_SetPriority(TC5_IRQn, 0);
     NVIC_EnableIRQ(TC5_IRQn);
    
      // Enable the TC5 interrupt request
     TC5->COUNT16.INTENSET.bit.MC0 = 1;
     while (tcIsSyncing()); //wait until TC5 is done syncing 
} 

//Every time we change something regarding timer we need for it to 
//sinc - adopt new values 
//Function that is used to check if TC5 is done syncing
//returns true when it is done syncing
bool tcIsSyncing()
{
    return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

//This function enables TC5 timer and waits for it to be ready
void tcStartCounter()
{
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
    while (tcIsSyncing()); //wait until snyc'd
}

//Reset TC5 timer
void tcReset()
{
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing());
  while (TC5->COUNT16.CTRLA.bit.SWRST);
}

//disable TC5 timer
void tcDisable()
{
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tcIsSyncing());
}




//Timer handler  - Main job is done here!!
//This is called 
// - every 100 microsecond (10 000Hz) during measurement mode
// - every 1/sampleRate seconds during stimulation mode (for ex. if our stimulation frequency is 1000Hz 
// and we have  100 samples per period than sampleRate is 100*1000 = 100 000Hz so this is called
// 1/sampleRate = 1/100000 = every 10microseconds
//
// - During stimulation mode every time timer call this function we "draw" another dot/sample of the stimulation sinus
//   in other words we send another sample to DAC with analogWrite function. Also we decrement 10 sec timer timerCounterForMeasurements
//   and if we hit zero we change mode to measurement
// - During measurement mode every time timer call this function we measure another sample, update average and check if we crossed average.
//   Also we increment sampleCount. If sampleCount == SAMPLING_FREQUENCY_FOR_MEASUREMENTS that means that we measured for 1 second and that we need 
//   to calculate frequency, display frequency to user and to change mode back to stimulation.
void TC5_Handler (void)
{
  *setPin = PinMASK;//flip the pin 10 for debug purpose
  
  if(mode == MODE_STIMULATION)//if we are in stimulation mode
  {
        analogWrite(A0, wavSamples[sIndex]);//send another sinus sample to DAC
        sIndex++;//update index of sample for next time
        timerCounterForMeasurements--;//decrement 10 seconds timer
        if(timerCounterForMeasurements==0)//if we hit zero (we stimulated for 10 seconds) switch to measurement mode
        {
            mode = MODE_MEASUREMENT;//switch to measurement mode 
            timerCounterForMeasurements = initialValueOfTimerCounter;//reset 10 second timer for next period of stimulation when we return from measurements

             //configure timer for measurement mode (to 10 000 Hz)
             tcDisable();
             tcReset();
             tcConfigure(); //this function will check "mode" and setup the timer to 10 000Hz
             tcStartCounter(); 
        }
  }
  else  // mode measurement
  {
    
       
        //------------------------- Read one sample ----------------------------------
        ADC->INTFLAG.bit.RESRDY = 1;              // Data ready flag cleared
        while (ADC->STATUS.bit.SYNCBUSY == 1);
        ADC->SWTRIG.bit.START = 1;                // Start ADC conversion
        while ( ADC->INTFLAG.bit.RESRDY == 0 );   // Wait till conversion done
        while (ADC->STATUS.bit.SYNCBUSY == 1);
        valueRead = ADC->RESULT.reg;     // read the result
        while (ADC->STATUS.bit.SYNCBUSY == 1);
        ADC->SWTRIG.reg = 0x01;  
       //--------------------------- end of Read one sample -----------------------------

        average = average*0.99999+(float)valueRead*0.00001;//update slow moving average


        if(prevSample<average && valueRead>=average)//if signal is crosing the average
        {
          flipCount++;//increment number of crosses 
          numberOfSamplesAfterLastCross = 0;
          firstCrossHappened = true;
        }
        
        if(!firstCrossHappened)//count samples before first cross to use for more precise calculation of freq
        {
            numberOfSamplesBeforeFirstCross++;
        }
  
        numberOfSamplesAfterLastCross++;//count after last cross to use for more precise calculation of freq
  
        prevSample = valueRead;//update prevSample for next time
        
        sampleCount++;//increment number of samples during measurement mode
//if (sampleCount == SAMPLING_FREQUENCY_FOR_MEASUREMENTS * recordTime)//if we are measuring for 1 second
        if (sampleCount == SAMPLING_FREQUENCY_FOR_MEASUREMENTS)
        {
          //calculate freq here
          currentFrequency = (flipCount-1)+((float)(numberOfSamplesBeforeFirstCross + numberOfSamplesAfterLastCross))/((float)10000/(float)flipCount);

          //reset all variables that we use during measurement of freq for next time 
          flipCount = 0;
          sampleCount = 0;
          numberOfSamplesBeforeFirstCross = 0;
          numberOfSamplesAfterLastCross = 0;
          firstCrossHappened = false;
          

          //we have result here. Display result to user
//!!!!!!!!!!      or save to SD card or display on LCD Davis here !!!!!!!!!!!!!!!!!!!!!!!!
          
         // Serial.print(average);
          //  Serial.print(" - ");
          Serial.println(currentFrequency);

          //you can also update frequency of stimmulation here  like:
          //sampleRate = newFrequency*100;
          
//EDITED EDITED EDITED EDITED EDITED          
          //print to LCD
          mySerial.write("                   "); // move cursor to beginning of first line
          mySerial.write(254);
          mySerial.write(128);
          mySerial.write("S: ");
          mySerial.print((double)  sampleRate/100);
//          mySerial.print("    ");
          mySerial.write(" T: ");
          mySerial.print(timeCounter);
          
          mySerial.write("  F: ");
          mySerial.print(currentFrequency);
          
          //go back to stimmulation
          timeCounter = timeCounter + PERIOD_OF_MEASUREMENTS_IN_SEC;
          mode = MODE_STIMULATION;
        }  
            
  }
  *clrPin = PinMASK;//reset debug pin
  TC5->COUNT16.INTFLAG.bit.MC0 = 1;
}

//--------------------------------- ADC code -------------------------------------



//this will just set our recording pin to A1
uint32_t selAnalog(uint32_t ulPin){      // Selects the analog input channel in INPCTRL
  ADCsync();
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[ulPin].ulADCChannelNumber; // Selection for the positive ADC input
}

//This will setup ADC to work really fast
uint32_t fastADCsetup() {
  //Input control register
  ADCsync();
  ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;      // Gain select as 1X
  //Set ADC reference source
  ADCsync();
  ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_AREFA_Val;//ADC_REFCTRL_REFSEL_INTVCC0_Val; //  2.2297 V Supply VDDANA
  // Set sample length and averaging
  ADCsync();
  ADC->AVGCTRL.reg = 0x00 ;       //Single conversion no averaging
  ADCsync();
  ADC->SAMPCTRL.reg = 0x00;       //Minimal sample length is 1/2 CLK_ADC cycle
  //Control B register
  ADCsync();
  ADC->CTRLB.reg =  0x400     ; // Prescale 64, 12 bit resolution, singel conversion
  // Enable ADC in control B register
  ADCsync();
  ADC->CTRLA.bit.ENABLE = 0x01;

 }
