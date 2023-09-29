#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>

#include <array>
#include "MCP3564.hpp"
#include "Adafruit_MAX31865.h" // For cold junction

#include <bitset>

#define SD 1
#define SERIAL 1

#define RTD_MODE 0
#define CLICK_MODE 1

// Clicks
//                            THE CLICK USES 3.3V!!!!
// With a 24-bit resolution and a 5V reference, the step is -> 5/(2^24) * 1000 = 0.000298 mV = 0.298 microVolt
// So for 41.234mV or 41234 microvolts, the ADC should give ~137584 

// With a 24-bit resolution and a 3.3V reference, the step is -> 3.3/(2^24) * 1000 = 0.000197 mV = 0.197 microVolt
// So for 41.234mV or 41234 microvolts, the ADC should give ~209309

// --> DO microVolts so that I don't have to worry about decimals in milliVolts which is useful for cold junction compensation
// when each degree increase in temperature is equal to 0.041 milliVolts or 41 microVolts for type K thermocouple.

const uint8_t pinCS = 10;
const uint8_t pinSCK = 13;
const uint8_t pinMOSI = 11;
const uint8_t pinMISO = 12;
const uint8_t pinMCLK = 2; // use analogWrite to control the pin
const uint8_t pinINT = 4;

MCP3564 click0 {pinCS, pinSCK, pinMOSI, pinMISO, pinMCLK, pinINT, DEVICE_ADDR1, &SPI};

//std::array<MCP3564, 4> clicks {click0, click1, click2};

uint32_t priorAnalogResolution = 8;

volatile uint32_t TC1_rawADCData = 0;
volatile uint32_t TC2_rawADCData = 0;
volatile uint32_t TC3_rawADCData = 0;
volatile uint32_t TC4_rawADCData = 0;
volatile bool dataReadyFlag0 = false;

int32_t TC1_filteredADCData = 0;
int32_t TC2_filteredADCData = 0;
int32_t TC3_filteredADCData = 0;
int32_t TC4_filteredADCData = 0;

std::array<volatile uint32_t*, 4> TC_rawADCDataArr = {&TC1_rawADCData, &TC2_rawADCData, &TC3_rawADCData, &TC4_rawADCData};
std::array<int32_t* , 4> TC_filteredADCDataArr = {&TC1_filteredADCData, &TC2_filteredADCData, &TC3_filteredADCData, &TC4_filteredADCData};
uint32_t i = 0; // only increment in the filterData function, so give it a wide delay between each conversion
                // else something not good will happen.

float TC1_trueTemp = 0.0f;
float TC2_trueTemp = 0.0f;
float TC3_trueTemp = 0.0f;
float TC4_trueTemp = 0.0f;
std::array<float*, 4> TC_trueTempArr = {&TC1_trueTemp, &TC2_trueTemp, &TC3_trueTemp, &TC4_trueTemp};

elapsedMillis mainTimer; 

// RTD cold junction

Adafruit_MAX31865 RTD_Sensor {0, &SPI1};
float coldJunctionTemp;
uint16_t rtd;
#define RREF 430.0
#define RRNOMINAL 100.0

// LOOK UP TABLES TO DO COLD JUNCTION COMPENSATION

void dataReadyHandler0() // should give a wide delay to process data (ie. for the filterData function to play catch up)
{
  *(TC_rawADCDataArr.at(i)) = click0.readADCRawData32();
  dataReadyFlag0 = true;
}

int32_t filterData(volatile uint32_t inputUnfilteredData)
{
    
    uint32_t unfilteredData = 0;
    int32_t filteredData = 0;
    
    cli();
    switch (click0.getCONFIG3_current().DATA_FORMAT)
    {
      case 0:
        unfilteredData = inputUnfilteredData; // 24 bits
        break;
      case 1:
        unfilteredData = inputUnfilteredData >> 8; // Shift to the right since if ADC data format is option 1. (page 42)
      case 2:
        break;
      case 3:
        break;
    }
    sei();
    
    //unfilteredData = inputUnfilteredData >> 8;  // Shift to the right since if ADC data format is option 1. (page 42)
    if (unfilteredData & 0x800000) // -> detects if it's a negative number. If this is true, the data is negative, which is represented by the 2's complement system
    {
      filteredData = unfilteredData | (0xFF000000);
    }
    else
    {
      filteredData = unfilteredData;
    }

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
  SPI1.begin();

#if RTD_MODE
  RTD_Sensor.begin(MAX31865_2WIRE);
#endif

  delay(10);

  if(click0.begin())
  {
    //attachInterrupt(digitalPinToInterrupt(click0.getPinINT()), dataReadyHandler0, CHANGE);  // when data is ready in SCAN mode
                                                                                  // if interrupt pin doesn't work, try the other 2 mechanisms
    //SPI.usingInterrupt(IRQ_NUMBER_t::IRQ_GPIO6789);
    delay(10);
    //click0.fastCommand(FASTCMD_FULLRESET); // reset the entire registers on startup. Don't want previous settings from the previous runs

    // Use a standby so that the click0 reads as commanded and store data in the ADCDATA reg, which can the be read.
    click0.setADCMode(ADC_MODE::ADC_Conversion);
    click0.setCONV_MODE(CONV_MODE::Continuous);
    click0.setDATA_FORMAT(DATA_FORMAT::FORMAT_24bit_default);


    click0.setEN_FASTCMD(EN_FASTCMD::Disabled); // disable fast commands for security reasons.
    
    //click0.setClock(CLK_SEL::IntClock_NoClockOutput);

    // supply the clock source via a pwm on a teensy
    click0.setClock(CLK_SEL::ExtDigitalClock_default);
    priorAnalogResolution = analogWriteResolution(8);
    analogWriteFrequency(pinMCLK, 4'915'200); // ideal frequency according to the datasheet
    analogWrite(pinMCLK, 256); // full duty cycle
    
    //click0.setEN_GAINCAL(EN_GAINCAL::Enabled);
    //click0.setEN_OFFCAL(EN_OFFCAL::Enabled);

    // Turn on SCAN mode, disable MUX mode.
    click0.setDiff_CHA(true);
    click0.setDiff_CHB(true);
    click0.setDiff_CHC(true);
    click0.setDiff_CHD(true);
    

    // Set delay timer. Change these up to set how fast data are read.
    // give a wide delay so data can be processed in between conversions.
    // during test, decrease delay to find a sweet delay time. 
    
    // Set delay between conversions
    //click0.setDLY(DLY::DMCLKMul512); //-> from calculations with DMCLK set to 4.9152MHz, this should set each conversion to be 0.1 ms apart.
    click0.setDLY(DLY::DMCLKMul64);

    // Set delay between SCAN cycles
    //click0.setTIMER(49152); // -> from calculations with DMCLK set to 4.9152MHz, this should each SCAN cycle to be 10 ms apart.
    //click0.setTIMER(300);
    /*
    // Send all values to all of the coresponding registers
    click0.writeRegister8(CONFIG1_reg, click0.getCONFIG1_current().raw);
    delay(5);
    click0.writeRegister8(CONFIG2_reg, click0.getCONFIG2_current().raw);
    delay(5);
    click0.writeRegister8(IRQ_reg, click0.getIRQ_current().raw);
    delay(5);
    click0.writeRegister8(MUX_reg, click0.getMUX_current().raw);
    delay(5);
    click0.writeRegister24(SCAN_reg, click0.getSCAN_current().raw);
    delay(5);

    click0.writeRegister8(CONFIG3_reg, click0.getCONFIG3_current().raw);
    delay(5);
    click0.writeRegister8(CONFIG0_reg, click0.getCONFIG0_current().raw); // Begins reading right away when ADCMODE gets set to continuous.
    delay(5);
    click0.setLOCK(); // Prevent accidental writing or error readings from the registers due to EMF and such.
    */
  }
  else 
  {
    Serial.println("ADC 9 click0 failed to initialize");
    while(1) delay(500);
  }
}

void loop() {
#ifdef CLICK_MODE
  cli();
  if (dataReadyFlag0) 
  {
    *(TC_filteredADCDataArr.at(i)) = filterData(*(TC_rawADCDataArr.at(i)));
    sei();
    Serial.println("/////////////////////////////");
    Serial.println("/////////////////////////////");
    Serial.println("/////////////////////////////");

    Serial.println(*TC_filteredADCDataArr.at(0));
    Serial.println(*TC_filteredADCDataArr.at(1));
    Serial.println(*TC_filteredADCDataArr.at(2));
    Serial.println(*TC_filteredADCDataArr.at(3));

  #ifdef RTD_MODE
    rtd = RTD_Sensor.readRTD();
    coldJunctionTemp = RTD_Sensor.temperature(RRNOMINAL, RREF);
    float ratio = rtd;
    ratio = ratio / 32768;
    Serial.print("Ratio = "); Serial.println(ratio,8);
    Serial.print("Resistance = "); Serial.println(RREF*ratio, 8);
    Serial.print("Cold Junction Temperature = "); Serial.println(coldJunctionTemp, 8);
  #endif

    dataReadyFlag0 = false;
  }
  sei();
#endif
  // Debug: manually read registers

  delay(10);
  Serial.println("PRINT");
  click0.updateCONFIG2_current();
  //click0.readADCRawData32();

#if RTD_MODE
  rtd = RTD_Sensor.readRTD();
  coldJunctionTemp = RTD_Sensor.temperature(RRNOMINAL, RREF);
  float ratio = rtd;
  ratio = ratio / 32768;
  Serial.print("Ratio = "); Serial.println(ratio,8);
  Serial.print("Resistance = "); Serial.println(RREF*ratio, 8);
  Serial.print("Cold Junction Temperature = "); Serial.println(coldJunctionTemp, 8);
#endif
/*
  SPI.beginTransaction(SPISettings{1000000, MSBFIRST, SPI_MODE0});
  digitalWrite(pinCS, LOW);

  digitalWrite(pinCS, HIGH);
  SPI.endTransaction();
*/

  //std::string stringBits = theBits.to_string();
  //Serial.println(stringBits.c_str());
  //Serial.println(theNumber);

  // 



/*
  if (mainTimer >= 100) // 20, 100
  {
    Serial.println("/////////////////////////////");
    Serial.println("/////////////////////////////");
    Serial.println("/////////////////////////////");

    Serial.println(*TC_filteredADCDataArr.at(0));
    Serial.println(*TC_filteredADCDataArr.at(1));
    Serial.println(*TC_filteredADCDataArr.at(2));
    Serial.println(*TC_filteredADCDataArr.at(3));
    mainTimer = 0;
    Serial.print("INT PIN is: ");
    Serial.println(digitalRead(pinINT));
  }
*/

  //while (1); // letting the loop run once for debugging (with a logic analyzer for example)

  // there is still more TODO!!!!.
}