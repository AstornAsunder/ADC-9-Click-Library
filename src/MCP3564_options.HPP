#ifndef MCP3564_OPTIONS_HPP
#define MCP3564_OPTIONS_HPP

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
    FULL_SHUTDOWN = 0,
    CONFIG0_default1 = 1,
    CONFIG0_default2 = 2,
    CONFIG0_default3 = 3
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

enum class MUX_VIN_pos // [7:4]
{
    CH0_default = 0,
    CH1 = 1,
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



#endif