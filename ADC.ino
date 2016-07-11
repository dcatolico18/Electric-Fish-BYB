long sampleCount = 0;
int flipCount = 0;
float currentFrequency = 0.0;
int prevSample = 0;    
int currentSample = 0;
int numberOfSamplesBeforeFirstCross;
int numberOfSamplesAfterLastCross;
bool firstCrossHappened;
#include "Arduino.h"
#include "wiring_private.h"

#define PIN 10

uint32_t anaPin = A1;

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
float average = 0;
uint32_t valueRead;

#ifdef _VARIANT_ARDUINO_ZERO_
volatile uint32_t *setPin = &PORT->Group[g_APinDescription[PIN].ulPort].OUTSET.reg;
volatile uint32_t *clrPin = &PORT->Group[g_APinDescription[PIN].ulPort].OUTCLR.reg;
const uint32_t  PinMASK = (1ul << g_APinDescription[PIN].ulPin);
#endif


int flipflop = 1;
void setup() {
  // put your setup code here, to run once:
  
Serial.begin(115200);
  tcConfigure();
  pinMode(10, OUTPUT); 
  digitalWrite(10,HIGH);
  // Fast ADC setup
  selAnalog(anaPin);
  //fastADCsetup2();
  //fastADCsetup2();
  fastADCsetup();
  numberOfSamplesBeforeFirstCross = 0;
  numberOfSamplesAfterLastCross = 0;
  firstCrossHappened = false;
  tcStartCounter();
   
}

void loop() {
  // put your main code here, to run repeatedly:

Serial.print(average);
  Serial.print(" - ");
  Serial.println(currentFrequency);
  
}

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
 //set TC5 timer counter based off of the system clock and the user defined sample rate or waveform
 TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / 10000 - 1);
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

//Function that is used to check if TC5 is done syncing
//returns true when it is done syncing
bool tcIsSyncing()
{
  return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

//This function enables TC5 and waits for it to be ready
void tcStartCounter()
{
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  while (tcIsSyncing()); //wait until snyc'd
}

//Reset TC5 
void tcReset()
{
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing());
  while (TC5->COUNT16.CTRLA.bit.SWRST);
}

//disable TC5
void tcDisable()
{
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tcIsSyncing());
}

void TC5_Handler (void)
{
 
        *setPin = PinMASK;
        //------------------------- Read one sample ----------------------------------
        ADC->INTFLAG.bit.RESRDY = 1;              // Data ready flag cleared
        while (ADC->STATUS.bit.SYNCBUSY == 1);
        ADC->SWTRIG.bit.START = 1;                // Start ADC conversion
        while ( ADC->INTFLAG.bit.RESRDY == 0 );   // Wait till conversion done
        while (ADC->STATUS.bit.SYNCBUSY == 1);
        valueRead = ADC->RESULT.reg;     // read the result
        
        while (ADC->STATUS.bit.SYNCBUSY == 1);
        ADC->SWTRIG.reg = 0x01;  
                
       //DEBUG test and serial print to test
       //--------------------------- end of Read one sample -----------------------------
     // average = (average-(average>>7))+(valueRead>>7);
     average = average*0.99999+(float)valueRead*0.00001;
       // tempaverage = 98*average + 2* valueRead;//update average
       // average average>>4
       //average = tempaverage/100;

if(prevSample<average && valueRead>=average)
{
  flipCount++;
  numberOfSamplesAfterLastCross = 0;
  firstCrossHappened = true;
  
}
  if(!firstCrossHappened)
  {
      numberOfSamplesBeforeFirstCross++;
  }
  
  numberOfSamplesAfterLastCross++;
  
prevSample = valueRead;
  sampleCount++;
if (sampleCount == 10000)
  {
  //calculate freq here
  
    currentFrequency = (flipCount-1)+((float)(numberOfSamplesBeforeFirstCross + numberOfSamplesAfterLastCross))/((float)10000/(float)flipCount);
    flipCount = 0;
    sampleCount = 0;
    numberOfSamplesBeforeFirstCross = 0;
    numberOfSamplesAfterLastCross = 0;
    firstCrossHappened = false;
  }  
       *clrPin = PinMASK;       
  TC5->COUNT16.INTFLAG.bit.MC0 = 1;
  
}



uint32_t selAnalog(uint32_t ulPin){      // Selects the analog input channel in INPCTRL
  ADCsync();
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[ulPin].ulADCChannelNumber; // Selection for the positive ADC input
}


//###################################################################################
// ADC set-up  here
//
//###################################################################################

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
