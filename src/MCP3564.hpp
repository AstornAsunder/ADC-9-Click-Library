/*
    Author: Lam Khuu
    Date started: 08/31/2023
    Last Updated: 

    SPI Driver for the MCU on a teensy 4.1 to talk to ADC 9 click.

    For CSULB Beach Launch Team.
*/


// Are there any major differences when it comes to writing for 3.6 or 4.1?

#ifndef MCP3564_HPP
#define MCP3564_HPP

#include <Arduino.h>
#include <SPI.h>
#include "MCP3564_options.HPP"
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

#define MCP3564_DEVICE_TYPE     (0x000F)  // MCP3564 device ID

#define MCP3564_DEVICE_ADDR     (0b01)  // AAC-1852-VP9, not sure abt the SPI address yet
                                        // If it's not 1, then try 2,3, or 4

// USE _SPI.Transfer(FASTCMD_...)
#define FASTCMD_DONTCARE        ((MCP3564_DEVICE_ADDR << 6) | 0) // for selecting the device on the bus if multiple devices are used on the same SPI bus
#define FASTCMD_CONVERSION      ((MCP3564_DEVICE_ADDR << 6) | 0b101000)
#define FASTCMD_STANDBY         ((MCP3564_DEVICE_ADDR << 6) | 0b101100)
#define FASTCMD_SHUTDOWN        ((MCP3564_DEVICE_ADDR << 6) | 0b110000)
#define FASTCMD_FULLSHUTDOWN    ((MCP3564_DEVICE_ADDR << 6) | 0b110100)
#define FASTCMD_FULLRESET       ((MCP3564_DEVICE_ADDR << 6) | 0b111000) // reset entire register map to default value according to the datasheet

// for the 3 commands below, insert register address via | 0b0000'00, where the first 4 MS bits are the register address
#define FASTCMD_STATIC_READ     ((MCP3564_DEVICE_ADDR << 6) | 0b01)
#define FASTCMD_INCR_WRITE      ((MCP3564_DEVICE_ADDR << 6) | 0b10)
#define FASTCMD_INCR_READ       ((MCP3564_DEVICE_ADDR << 6) | 0b11)


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

struct status
{
    bool DR = 1;
    bool CRCCFG = 1;
    bool POR = 1;
};

class MCP3564
{
private:
    int32_t _adcRawData = 0; // 23 bit + sign or 31 bits + sign depending on DATA_FORMAT[1:0] or modulator output stream (4-bit wide)

    uint8_t _pinCS = 0;
    uint8_t _pinMOSI = 0;
    uint8_t _pinMISO = 0;
    uint8_t _pinSCK = 0;
    uint8_t _pinMCK = 0;
    uint8_t _pinINT = 0;
    SPIClass* _SPI = {nullptr};

    uint8_t _status; // might need to display this as bytes to see what SPI sends back

    // Default registers' values
    const uint8_t CONFIG0_default = 0b1100'0000;
    const uint8_t CONFIG1_default = 0b0000'1100;
    const uint8_t CONFIG2_default = 0b1000'1011;
    const uint8_t CONFIG3_default = 0b0000'0000;
    const uint8_t IRQ_default     = 0b0000'0011;
    const uint8_t MUX_default     = 0b0000'0001;
    const uint32_t SCAN_default   = 0;
    const uint32_t TIMER_default  = 0;              // unsigned
    const int32_t OFFSETCAL_default = 0;            // signed
    const uint32_t GAINCAL_default = 0x800000;      // unsigned
    const uint8_t LOCK_default = 0b1010'0101; // or 0xA5, for enabling spi write to the registers
    
    // Current registers' values

    uint8_t CONFIG0_current = CONFIG0_default;
    uint8_t CONFIG1_current = CONFIG1_default;
    uint8_t CONFIG2_current = CONFIG2_default;
    uint8_t CONFIG3_current = CONFIG3_default;
    uint8_t IRQ_current     = IRQ_default;
    uint8_t MUX_current     = MUX_default;
    uint32_t SCAN_current   = SCAN_default;
    uint32_t TIMER_current  = TIMER_default;
    int32_t OFFSETCAL_current = OFFSETCAL_default;
    uint32_t GAINCAL_current = GAINCAL_default;
    uint8_t LOCK_current = LOCK_default; // current must match default for register write access
    
    status IRQ_status;

    SPISettings _defaultSPISettings = SPISettings{MAX_SPI_SPEED, MSBFIRST, SPI_MODE0};
    // for custom settings, do it in main

    // SCAN BITS. Default set channels here
    // Might need to rework this
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
    bool Diff_CHC = 1;  //CH4-CH5
    bool Diff_CHD = 1;  //CH6-CH7
    bool TEMP = 1;
    bool A_VDD = 1;
    bool VCM = 1;
    bool OFFSET = 1;

    void _transfer(const uint8_t& addr, const uint8_t& data);
    void _transfer(const uint8_t& addr, const uint32_t& data);
public:
    MCP3564(uint8_t CS, uint8_t MOSI, uint8_t MISO, uint8_t SCK, uint8_t MCK, uint8_t INT, SPIClass* mainSPI= &SPI);

    // make a setting function for each setting bit (or sets of setting bits) of the registers 
    // that will set the configurations, instead of manually turning on/off bits in main
    // also might need to make these private
    
    // refer to the register map for the corresponding options.

    bool begin();

    void updateIRQ_status();    // read the 3 bits in the IRQ registers and update 
                                // read via bit masking and shifting the wanted bit to the first position
    int32_t getADCRawData() const;



    int32_t readRegister4(uint8_t* addr); // MDAT output mode for ADCDATA
    int32_t readRegister8(uint8_t* addr);
    int32_t readRegister16(uint8_t* addr); // only CRCCFG has 16 bits
    int32_t readRegister24(uint8_t* addr);
    int32_t readRegister32(uint8_t* addr); // only for 32bit ADCDATA. probably will not use the 32bit setting

// SET BITS BY USING ONLY THE SETTING OPTIONS BELOW!!!!
/////////////////////////// CONFIG0 register options ///////////////////////////
    void setADCMode(const ADC_MODE& option);
    void setSourceSink(const CS_SEL& option);
    void setClock(const CLK_SEL& option);
    void setCONFIG0(const CONFIG0& option); // FULL SHUTDOWN

/////////////////////////// CONFIG1 register ///////////////////////////////////
    void setOSR(const OSR& option);
    void setPRE(const PRE& option);

/////////////////////////// CONFIG2 register ///////////////////////////////////
    void setAZ_MUX(const AZ_MUX& option);
    void setGAIN(const GAIN& option);
    void setBOOST(const BOOST& option);

/////////////////////////// CONFIG3 register ///////////////////////////////////
    void setEN_GAINCAL(const EN_GAINCAL& option);
    void setEN_OFFCAL(const EN_OFFCAL& option);
    void setEN_CRCCOM(const EN_CRCCOM& option);
    void setCRC_FORMAT(const CRC_FORMAT& option);
    void setDATA_FORMAT(const DATA_FORMAT& option);
    void setCONV_MODE(const CONV_MODE& option);

/////////////////////////// IRQ register //////////////////////////////////////
    void setEN_STP(const EN_STP& option);
    void setEN_FASTCMD(const EN_FASTCMD& option);
    void setIRQ_MODE(const IRQ_MODE& option);

/////////////////////////// MUX register //////////////////////////////////////
    void setMUX_VIN_neg(const MUX_VIN_neg& option);
    void setMUX_VIN_pos(const MUX_VIN_pos& option);

/////////////////////////// SCAN register //////////////////////////////////////
    // Use the bitWrite function to set the channels
    void setSE_CH0(const bool& option);
    void setSE_CH1(const bool& option);
    void setSE_CH2(const bool& option);
    void setSE_CH3(const bool& option);
    void setSE_CH4(const bool& option);
    void setSE_CH5(const bool& option);
    void setSE_CH6(const bool& option);
    void setSE_CH7(const bool& option);
    void setDiff_CHA(const bool& option);
    void setDiff_CHB(const bool& option);
    void setDiff_CHC(const bool& option);
    void setDiff_CHD(const bool& option);
    void setTEMP(const bool& option);
    void setA_VDD(const bool& option);
    void setVCM(const bool& option);
    void setOFFSET(const bool& option);

    void setDLY(const DLY& option);

/////////////////////////// TIMER register //////////////////////////////////////
    void setTIMER(const uint32_t& time);

/////////////////////////// OFFSETCAL register //////////////////////////////////
    void setOFFSETCAL(const int32_t& value);

/////////////////////////// GAINCAL register ///////////////////////////////////
    void setGAINCAL(const uint32_t& value);

/////////////////////////// LOCK register //////////////////////////////////////
    void setLOCK(); // 0xA5 = full access to register write
    void setUNLOCK();



};

#endif