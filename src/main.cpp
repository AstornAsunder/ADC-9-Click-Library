#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>

#include <array>

#include "MCP3564.hpp"
#include "Adafruit_MAX31865.h" // For cold junction

#define SD 1
#define SERIAL 1

uint8_t pinCS = 10;
uint8_t pinSCK = 13;
uint8_t pinMOSI = 11;
uint8_t pinMISO = 12;
uint8_t pinMCLK = 2; // use analogWrite to control the pin
uint8_t pinINT = 4;

MCP3564 click {pinCS, pinSCK, pinMOSI, pinMISO, pinMCLK, pinINT, &SPI}; // don't know what to do with the MCLK pin yet

uint32_t priorAnalogResolution = 8;



volatile int32_t TC1_rawADCData = 0;
volatile int32_t TC2_rawADCData = 0;
volatile int32_t TC3_rawADCData = 0;
volatile int32_t TC4_rawADCData = 0;
volatile bool dataReadyFlag = true;

int32_t TC1_filteredADCData = 0;
int32_t TC2_filteredADCData = 0;
int32_t TC3_filteredADCData = 0;
int32_t TC4_filteredADCData = 0;

std::array<volatile int32_t*, 4> TC_rawADCDataArr = {&TC1_rawADCData, &TC2_rawADCData, &TC3_rawADCData, &TC4_rawADCData};
std::array<int32_t* , 4> TC_filteredADCDataArr = {&TC1_filteredADCData, &TC2_filteredADCData, &TC3_filteredADCData, &TC4_filteredADCData};
uint32_t i = 0; // only increment in the filterData function, so give it a wide delay between each conversion
                // else something not good will happen.

int32_t coldJunctionTemp = 0;
float TC1_trueTemp = 0.0f;
float TC2_trueTemp = 0.0f;
float TC3_trueTemp = 0.0f;
float TC4_trueTemp = 0.0f;
std::array<float*, 4> TC_trueTempArr = {&TC1_trueTemp, &TC2_trueTemp, &TC3_trueTemp, &TC4_trueTemp};

elapsedMillis mainTimer; 

void dataReadyHandler() // should give a wide delay to process data
{
  *(TC_rawADCDataArr.at(i)) = click.readADCRawData24();
  dataReadyFlag = true;
}

int32_t filterData(volatile int32_t inputUnfilteredData)
{
    //cli();
    uint32_t unfilteredData = 0;
    uint32_t theSign = 0;
    int32_t filteredData = 0;

    unfilteredData = inputUnfilteredData;

    unfilteredData = unfilteredData & ~(0xFF000000); // discard the most MSbyte, in case it's a garbage byte
                                                     // or a status byte that I don't care for now.


    theSign = unfilteredData & 0x800000;
    bitClear(unfilteredData, 23);
    theSign = theSign << 8;

    filteredData = unfilteredData | theSign;

    i = i + 1;
    if( i == 4) i = 0; 
    //sei();
    return filteredData;
}

float convertToTrueTemp() // relies on cold junction
{
  float trueTemp = 0.0f;

  return trueTemp;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  SPI.begin();

  delay(10);

  if(click.begin())
  {
    // check again attachinterrupt
    attachInterrupt(digitalPinToInterrupt(pinINT), dataReadyHandler, FALLING); // when data is ready in SCAN mode

    delay(10);
    //click.fastCommand(FASTCMD_FULLRESET); // reset the entire registers on startup. Don't wanna previous settings from previous runs

    // Use a standby so that the click reads as commanded and store data in the ADCDATA reg, which can the be read.
    click.setADCMode(ADC_MODE::ADC_Conversion);
    click.setCONV_MODE(CONV_MODE::Continuous);

    // supply the clock source via a pwm on a teensy
    click.setClock(CLK_SEL::ExtDigitalClock_default);
    priorAnalogResolution = analogWriteResolution(8);
    analogWriteFrequency(pinMCLK, 4'915'200); // ideal frequency, according to the datasheet
    analogWrite(pinMCLK, 256);
    //priorAnalogResolution = analogWriteResolution()

    //click.setEN_GAINCAL(EN_GAINCAL::Enabled);
    //click.setEN_OFFCAL(EN_OFFCAL::Enabled);

    // Turn on SCAN mode, disable MUX mode.
    click.setDiff_CHA(true);
    click.setDiff_CHB(true);
    click.setDiff_CHC(true);
    click.setDiff_CHD(true);
    

    // Set delay timer. Change these up to set how fast data are read.
    // give a wide delay so data can be processed in between conversions.
    // during test, decrease delay to find a sweet delay time. 
    click.setDLY(DLY::DMCLKMul32);
    click.setTIMER(20);


    // Send all values to all of the coresponding registers
    click.transfer(CONFIG0_reg, click.getCONFIG0_current());
    click.transfer(CONFIG1_reg, click.getCONFIG1_current());
    click.transfer(CONFIG2_reg, click.getCONFIG2_current());
    click.transfer(CONFIG3_reg, click.getCONFIG3_current());
    click.transfer(IRQ_reg, click.getIRQ_current());
    click.transfer(MUX_reg, click.getMUX_current());
    click.transfer(SCAN_reg, click.getSCAN_current());
  }
  else 
  {
    Serial.println("ADC 9 click failed to initialize");
    while(1) delay(500);
  }
}

void loop() {
  cli();
  if (dataReadyFlag) 
  {
    *(TC_filteredADCDataArr.at(i)) = filterData(*(TC_rawADCDataArr.at(i)));
    dataReadyFlag = false;
  }
  sei();

  if (mainTimer >= 20)
  {
    Serial.println("/////////////////////////////");
    Serial.println("/////////////////////////////");
    Serial.println("/////////////////////////////");

    Serial.println(*TC_filteredADCDataArr.at(0));
    Serial.println(*TC_filteredADCDataArr.at(1));
    Serial.println(*TC_filteredADCDataArr.at(2));
    Serial.println(*TC_filteredADCDataArr.at(3));
  }


  // there is still more TODO!!!!.

  
  
}