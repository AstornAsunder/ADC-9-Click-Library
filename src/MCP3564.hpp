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
#include <array>
#include "MCP3564_options.HPP"

//////////////////////////////////
// #defines for command byte
#define MCP3564_DEVICE_TYPE     (0x000F)  // MCP3564 device ID

#define DEVICE_ADDR0     (0b00)
#define DEVICE_ADDR1     (0b01) // AAC-1852-VP9 (font too small, might not be corrent, need a magnifying glass), not sure abt the SPI address yet
                                        // If it's not 1, then try 0, 2, or 3
#define DEVICE_ADDR2     (0b10)
#define DEVICE_ADDR3     (0b11)


// USE _SPI.Transfer(FASTCMD_...)
// When using a fast command, a status byte will also be embedded as the MSbyte along the SDO line with the data.
#define FASTCMD_DONTCARE        (0) // for selecting the device on the bus if multiple devices are used on the same SPI bus
#define FASTCMD_CONVERSION      (0b1010)
#define FASTCMD_STANDBY         (0b1011)
#define FASTCMD_SHUTDOWN        (0b1100)
#define FASTCMD_FULLSHUTDOWN    (0b1101)
#define FASTCMD_FULLRESET       (0b1110) // reset entire register map to default value according to the datasheet


// for the 3 commands below, insert register address via | 0b0000'00, where the first 4 MS bits are the register address
#define STATIC_READ (0b01) // read a single register. (page 66)
#define INCREMENTAL_WRITE (0b10)
#define INCREMENTAL_WRITE (0b11)


//////////////////////////////////

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
#define MAX_SPI_SPEED 20'000'000
// use MSBFIRST and SPI_MODE

// union singletons
union CONFIG0_union 
{
    struct 
    {
        uint8_t ADC_MODE : 2;
        uint8_t CS_SEL: 2;
        uint8_t CLK_SEL: 2;
        uint8_t CONFIG0: 2;
    };
    uint8_t raw = 0;
};

union CONFIG1_union 
{
    struct
    {
        uint8_t : 2; // RESERVED
        uint8_t OSR : 4;
        uint8_t PRE : 2;
    };
    uint8_t raw = 0;
};

union CONFIG2_union
{
    struct
    {
        uint8_t : 2;  // RESERVED
        bool AZ_MUX : 1;
        uint8_t GAIN : 3;
        uint8_t BOOST: 2;
    };
    uint8_t raw = 0;
};

union CONFIG3_union
{
    struct
    {
        bool EN_GAINCAL : 1;
        bool EN_OFFCAL : 1;
        bool EN_CRCCOM : 1;
        bool CRC_FORMAT : 1;
        uint8_t DATA_FORMAT : 2;
        uint8_t CONV_MODE : 2;
    };
    uint8_t raw = 0;
};

union IRQ_union
{
    struct
    {
        bool EN_STP : 1;
        bool EN_FASTCMD: 1;
        uint8_t IRQ_MODE: 2;
        bool POR_STATUS: 1;
        bool CRCCFG_STATUS: 1;
        bool DR_STATUS: 1;
        bool : 1; // NOT USED
    };
    uint8_t raw = 0;
};

union MUX_union
{
    struct
    {
        uint8_t MUX_VIN_neg : 4;
        uint8_t MUX_VIN_pos : 4;
    };
    uint8_t raw = 0;
};

union SCAN_union
{
    struct
    {
        bool CH0 : 1;
        bool CH1 : 1;
        bool CH2 : 1;
        bool CH3 : 1;
        bool CH4 : 1;
        bool CH5 : 1;
        bool CH6 : 1;
        bool CH7 : 1;
        bool Diff_CHA : 1;
        bool Diff_CHB : 1;
        bool Diff_CHC : 1;
        bool Diff_CHD : 1;
        bool TEMP : 1;
        bool AVdd : 1;
        bool VCM : 1;
        bool OFFSET: 1;
        uint8_t : 4; // empty bits
        bool : 1; // RESERVED
        uint8_t DLY : 3;
        uint8_t : 8; // NOT USED
    };
    uint32_t raw = 0;
};

union Command_byte
{
    struct
    {
        uint8_t CMD_type : 2;
        uint8_t fastCommand_or_RegAddr: 4;
        uint8_t devAddr: 2;
    };
    uint8_t raw = 0;
};

union Status_byte { // After sending a command byte to the click, the click will send back a status byte simultaneously
                    // Use this to catch that byte
    struct
    {
        bool POR;    // During continue conversion mode, send random command byte to poll for this bit, then
        bool CRCCFG;
        bool DR;
        bool HI_state;
        uint8_t dev_addr : 2;
        uint8_t : 2; // 2 emty bits
    };
    uint8_t raw = 0;
};



class MCP3564
{
private:
    volatile int32_t _adcRawData = 0; // 23 bit + sign or 31 bits + sign depending on DATA_FORMAT[1:0] or modulator output stream (4-bit wide)

    uint8_t _pinCS = 0;
    uint8_t _pinMOSI = 0;
    uint8_t _pinMISO = 0;
    uint8_t _pinSCK = 0;
    uint8_t _pinMCLK = 0;
    uint8_t _pinINT = 0;
    SPIClass* _SPI = {nullptr};

    //uint8_t _status; // might need to display this as bytes to see what SPI sends back

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
    // Might need to use union for better access to the bits.
    CONFIG0_union _CONFIG0_current;
    CONFIG1_union _CONFIG1_current;
    CONFIG2_union _CONFIG2_current;
    CONFIG3_union _CONFIG3_current;
    IRQ_union _IRQ_current;
    MUX_union _MUX_current;
    SCAN_union _SCAN_current;
    uint32_t _TIMER_current;
    int32_t _OFFSETCAL_current;
    uint32_t _GAINCAL_current;
    uint8_t _LOCK_current; // current must match default for register write access

    Status_byte _status;

    SPISettings _defaultSPISettings = SPISettings{MAX_SPI_SPEED, MSBFIRST, SPI_MODE0};

public:
    MCP3564(uint8_t CS, uint8_t SCK, uint8_t MOSI, uint8_t MISO, uint8_t MCLK, uint8_t INT, SPIClass* mainSPI= &SPI);

    // make a setting function for each setting bit (or sets of setting bits) of the registers 
    // that will set the configurations, instead of manually turning on/off bits in main
    // also might need to make these private
    
    // refer to the register map for the corresponding options.
    
    // transfer buffer for the MOSI line.
    uint8_t TXBuffer8[1] = {0};
    uint8_t TXBuffer16[2] = {0,0};
    uint8_t TXBuffer24[3] = {0,0,0};
    uint8_t TXBuffer32[4] = {0,0,0,0};

    // return buffer for the MISO line.
    uint8_t RXBuffer8[1] = {0};
    uint8_t RXBuffer16[2] = {0,0};
    uint8_t RXBuffer24[3] = {0,0,0};
    uint8_t RXBuffer32[4] = {0,0,0,0};

    bool begin();

    volatile int32_t readADCRawData32();
    volatile int32_t getADCRawData() const;

    bool isLocked(); // use readRegister 8 to read the LOCK register if the value is 0xA5

    


    void disableSCAN();

    int32_t readRegister4(uint8_t* addr); // MDAT output mode for ADCDATA
    int32_t readRegister8(uint8_t* addr);
    int32_t readRegister16(uint8_t* addr); // only CRCCFG has 16 bits
    int32_t readRegister24(uint8_t* addr);
    int32_t readRegister32(uint8_t* addr); // only for 32bit ADCDATA. probably will not use the 32bit setting

    

    void transfer(const uint8_t& addr, const uint8_t& data);
    void transfer(const uint8_t& addr, const uint32_t& data);
    void fastCommand(const uint8_t& cmd);
    void fastCommand(const uint8_t& cmd, const uint8_t& addr); // for the 3 fast commands used to read a register.
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
    void setAVdd(const bool& option);
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

    Status_byte getStatus() {return _status;}

    
    void updateAllRegValues(); // read all of the registers and update the CONFIG0_current, CONFIG1_current, CONFIG2_current, etc.
    void updateCONFIG0_current();
    void updateCONFIG1_current();
    void updateCONFIG2_current();
    void updateCONFIG3_current();
    void updateIRQ_current();
    void updateMUX_current();
    void updateSCAN_current();

    // TODO: FOR THESE GETTERS, READ FROM REGISTERS FIRST AND ASSIGNING THE VALUES TO THE CURRENTS BEFORE RETURNING THEM.
    CONFIG0_union getCONFIG0_current() {updateCONFIG0_current(); return _CONFIG0_current;}
    CONFIG1_union getCONFIG1_current() {updateCONFIG1_current(); return _CONFIG1_current;}
    CONFIG2_union getCONFIG2_current() {updateCONFIG2_current(); return _CONFIG2_current;}
    CONFIG3_union getCONFIG3_current() {updateCONFIG3_current(); return _CONFIG3_current;}
    IRQ_union getIRQ_current() {updateIRQ_current(); return _IRQ_current;}
    MUX_union getMUX_current() {updateMUX_current(); return _MUX_current;}
    SCAN_union getSCAN_current() {updateMUX_current(); return _SCAN_current;}
    uint32_t getTIMER_current() {return _TIMER_current;}
    int32_t getOFFSETCAL_current() {return _OFFSETCAL_current;}
    uint32_t getGAINCAL_current() {return _GAINCAL_current;}
    // for LOCK, read the register directly to see what the current code is.
};

#endif