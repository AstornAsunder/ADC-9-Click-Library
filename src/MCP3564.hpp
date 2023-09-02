/*
    Author: Lam Khuu
    Date started: 08/31/2023
    Last Updated: 

    SPI Driver for the MCU on a teensy 4.1 to talk to ADC 9 click
*/


// Are there any major differences when it comes to writing for 3.6 or 4.1?

#ifndef MCP3564_HPP
#define MCP3564_HPP

#include <Arduino.h>
#include <SPI.h>

// Mux settings
#define MCP_OFFSET (0x88)  
#define MCP_VCM    (0xF8) 
#define MCP_AVDD   (0x98) 
#define MCP_TEMP   (0xDE) 
#define MCP_DIFFD  (0x67) 
#define MCP_DIFFC  (0x45) 
#define MCP_DIFFB  (0x23) 
#define MCP_DIFFA  (0x01) 
#define MCP_CH7    (0x78) 
#define MCP_CH6    (0x68) 
#define MCP_CH5    (0x58) 
#define MCP_CH4    (0x48) 
#define MCP_CH3    (0x38) 
#define MCP_CH2    (0x28) 
#define MCP_CH1    (0x18) 
#define MCP_CH0    (0x08)  

#define MCP3564_DEVICE_TYPE    (0x000F)  //!< MCP3564 device ID

// Register map
#define ADCDATA_reg     (0x00)
#define CONFIG0_reg     (0x01)
#define CONFIG1_reg     (0x02)
#define CONFIG2_reg     (0x03)
#define CONFIG3_reg     (0x04)
#define IRQ_reg         (0x05)
#define MUX_reg         (0x06)
#define SCAN_reg        (0x07)
#define TIMER_reg       (0x08)
#define OFFSETCAL_reg   (0x09)
#define GAINCAL_reg     (0x0A)
#define LOCK_reg        (0x0D)
#define CRCCFG_reg      (0x0F)
//#define RESERVED1   (0x0B)
//#define RESERVED2   (0x0C)
//#define RESERVED3   (0x0E)


// SPI settings
#define MAX_SPI_SPEED 2'000'000
// use MSBFIRST and SPI_MODE

////////////////////////////////////////////////////////////////////////////////
//////////////////// REGISTER OPTIONS //////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/////////////////////////// CONFIG0 register options ///////////////////////////
enum class ADC_MODE // [1:0], ADC Operating Mode Selection
{
    ADC_Shutdown_default = 0,
    ADC_Shutdown = 1,
    ADC_Standby = 2,
    ADC_Conversion = 3 // <-------------- want this setting
};

enum class CS_SEL // [3:2], Current Source/Sink Selection Bits for Sensor Bias (source on Vin+/sink on Vin-)
{
    CS_SEL_0MuA_default = 0,
    CS_SEL_0p9MuA = 1,
    CS_SEL_3p7MuA = 2,
    CS_SEL_15MuA = 3,
};

enum class CLK_SEL // [5:4], Clock Selection
{
    ExtDigitalClock_default = 0,
    ExtDigitalClock = 1,
    IntClock_NoClockOutput = 2,
    IntClock_AMCLKPresent = 3
};

enum class CONFIG0  // [7:6], Full Shutdown Mode Enabled
                    // these bits are writen but have no effect except that they force FULL SHUTDOWN mode
                    // when they are set to "00" and when all other CONFIG0 bits are set to '0'
{
    FULL_SHUTDOWN = 0
};

/////////////////////////// CONFIG1 register ///////////////////////////////////

// [1:0] reserved

enum class OSR // [6:3], Oversampling 
{
    OSR_32 = 0,
    OSR_64 = 1,
    OSR_128 = 2,
    OSR_256_default = 3, // default
    OSR_512 = 4, 
    OSR_1024 = 5,
    OSR_2048 = 6,
    OSR_4096 = 7,
    OSR_8192 = 8,
    OSR_16384 = 9,
    OSR_20480 = 10,
    OSR_24576 = 11,
    OSR_40960 = 12,
    OSR_49152 = 13,
    OSR_81920 = 14,
    OSR_98304 = 15 
};

enum class PRE // [7:6], Prescaler Value Selection for AMCLK
{
    MCLK_Div1_default = 0, // default
    MCLK_Div2 = 1,
    MCLK_Div4 = 2,
    MCLK_Div8 = 3
};

/////////////////////////// CONFIG2 register ///////////////////////////////////

// [1:0] reserved, should always be set to '11' or 3

enum class AZ_MUX // [2], Auto-Zeroing MUX setting
{
    AZ_Disable_default = 0, 
    AZ_Enable = 1
};

enum class GAIN // [5:3], ADC Gain Selection
{
    GAIN_x1div3 = 0,
    GAIN_x1_default = 1,
    GAIN_x2 = 2,
    GAIN_x4 = 3,
    GAIN_x8 = 4,
    GAIN_x16 = 5,
    GAIN_x32 = 6, // (x16 analog, x2 digital)
    GAIN_x64 = 7 // (x16 analog, x4 digital) 
};

enum class BOOST // [7:6], ADC Bias Current Selection
{
    ADC_BIAS_x0p5 = 0,
    ADC_BIAS_x0p66 = 1,
    ADC_BIAS_x1 = 2,
    ADC_BIAS_x2 = 3
};

/////////////////////////// CONFIG3 register ///////////////////////////////////

enum class EN_GAINCAL // [0], Enable Digital Gain Calibration
{
    Disabled_default = 0,
    Enabled = 1
};

enum class EN_OFFCAL // [1], Enable Digital Offset Calibration
{
    Disabled_default = 0,
    Enabled = 1
};

enum class EN_CRCCOM // [2], CRC Checksum Selection on Read COmmunications
{
    CRC_OnCom_Disabled_default = 0,
    CRC_OnCom_Enabled = 1
};

enum class CRC_FORMAT // [3], CRC Checksum Format Selection on Read Communications
{
    CRC_16bit_default = 0,
    CRC_32bit = 1
};

enum class DATA_FORMAT // [5:4], ADC Output Data Format Selection
{
    FORMAT_24bit_default = 0,
    FORMAT_32bit_24BitLeftJustified = 1,
    FORMAT_32bit_25BitrightJustified = 2,
    FORMAT_32bit_25BitRightJustified_CHID = 3
};

enum class CONV_MODE // [7:6], Conversion Mode Selection
{
    Oneshot_Shutdown_default1 = 0, // options 1 and 0 are the same
    Oneshot_Shutdown_default2 = 1, // options 1 and 0 are the same
    Oneshot_Standby = 2,   //
    Continuous = 3 // <------------ NEED THIS SETTING
};

/////////////////////////// IRQ register ///////////////////////////////////

enum class EN_STP // [0], Enable conversion Start Interrupt Output
{
    Disabled = 0,
    Enabled_default = 1
};

enum class EN_FASTCMD // [1], Enable Fast Commands in the COMMAND Byte
{
    Disabled = 0,
    Enabled_default = 1
};

enum class IRQ_MODE // [3:2] 
{
    IRQ_HighZ = 0,
    IRQ_LogicHigh = 1,
    MDAT_HighZ = 2,
    MDAT_LogicHigh = 3
};
// use the 3 status bits to check for last read


/////////////////////////// MUX Register ///////////////////////////////////
enum class MUX_VIN_neg // [0:3]
{
    CH0 = 0,
    CH1_default = 1,
    CH2 = 2,
    CH3 = 3,
    CH4 = 4,
    CH5 = 5,
    CH6 = 6,
    CH7 = 7,
    A_GND = 8,
    A_VDD = 9,
    // RESERVED = 10,
    REF_IN_POS = 11,
    REF_IN_NEG = 12,
    Temp_Diode_P = 13,
    Temp_Diode_M = 14,
    Internal_VCM = 15
};

enum class MUX_VIN_POS // [7:4]
{
    CH0 = 0,
    CH1_default = 1,
    CH2 = 2,
    CH3 = 3,
    CH4 = 4,
    CH5 = 5,
    CH6 = 6,
    CH7 = 7,
    A_GND = 8,
    A_VDD = 9,
    // RESERVED = 10,
    REF_IN_POS = 11,
    REF_IN_NEG = 12,
    Temp_Diode_P = 13,
    Temp_Diode_M = 14,
    Internal_VCM = 15
};

/////////////////////////// SCAN Register ///////////////////////////////////

// SCAN BITS. Default set channels here
bool SinEn_CH0 = 0;
bool SinEn_CH1 = 0;
bool SinEn_CH2 = 0;
bool SinEn_CH3 = 0;
bool SinEn_CH4 = 0;
bool SinEn_CH5 = 0;
bool SinEn_CH6 = 0;
bool SinEn_CH7 = 0;
bool Diff_CHA = 1;   //CH0-CH1
bool Diff_CHB = 1;   //CH2-CH3
bool Diff_CHA = 1;  //CH4-CH5
bool Diff_CHB = 1;  //CH6-CH7
bool TEMP = 1;
bool A_VDD = 1;
bool VCM = 1;
bool OFFSET = 1;

enum class DLY // [23:21], multiplied delay time between each conversion during a scan cycle
{
    DMCLKMul0_default = 0,
    DMCLKMul8 = 1,
    DMCLKMul16 = 2,
    DMCLKMul32 = 3,
    DMCLKMul64 = 4,
    DMCLKMul128 = 5,
    DMCLKMul256 = 6,
    DMCLKMul512 = 7
};

/////////////////////////// TIMER Register ///////////////////////////////////
// DELAY timer between two consecutive scan cycles when CONV_MODE[1:0]

// The register uses the entire 24 bits to set the timer.


/////////////////////////// OFFSETCAL Register ////////////////////////////////
// Offset Error Calibration Code (two's complement, MSb first coding)

// The register uses the entire 24 bits.


/////////////////////////// GAINCAL Register ////////////////////////////////
// Gain Error Digital Calibration Code (unsigned, MSb first coding)
// Default value is 800'000, which provides a gain of 1x.

// The register uses the entire 24 bits.



/////////////////////////// LOCK REGISTER ////////////////////////////////
// Write access password entry code.
// the access code is 0x5a, or 0b10100101. Passing in anyvalue other than this will lock write access to
// the entire register map

class MCP3564
{
private:
    int32_t _adcRawData = 0; // 24 or 32 bits depending on DATA_FORMAT[1:0] or modulator output stream (4-bit wide)

    uint8_t _pinCS = 0;
    uint8_t _pinMOSI = 0;
    uint8_t _pinMISO = 0;
    uint8_t _pinSCK = 0;
    uint8_t _pinMCK = 0;
    uint8_t _pinINT = 0;
    SPIClass* _SPI = {nullptr};

    SPISettings _defaultSPISettings = SPISettings{MAX_SPI_SPEED, MSBFIRST, SPI_MODE0};
    // for custom settings, do it in main

public:
    MCP3564(uint8_t CS, uint8_t MOSI, uint8_t MISO, uint8_t SCK, uint8_t MCK, uint8_t INT, SPIClass* mainSPI= &SPI);

    bool begin();

    int32_t getADCRawData() const;


    // make a setting function for each setting bit (or sets of setting bits) of the registers 
    // that will set the configurations, instead of manually turning on/off bits in main
    // also might need to make these private
    
    // refer to the register map for the corresponding options.

    // use bit writes to set bits?

    // CONFIG0
    void setADCMode(const uint8_t& option);



    int32_t readRegister4(uint8_t* addr); // MDAT output mode for ADCDATA
    int32_t readRegister8(uint8_t* addr);
    int32_t readRegister16(uint8_t* addr); // only CRCCFG has 16 bits
    int32_t readRegister24(uint8_t* addr);
    int32_t readRegister32(uint8_t* addr); // only for 32bit ADCDATA. probably will not use the 32bit setting



};

#endif