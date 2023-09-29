#include "MCP3564.hpp"

MCP3564::MCP3564(uint8_t CS, uint8_t SCK, uint8_t MOSI, uint8_t MISO, uint8_t MCLK, uint8_t INT, uint8_t devAddr, SPIClass* mainSPI)
    : _pinCS{CS}, _pinSCK{SCK}, _pinMOSI{MOSI}, _pinMISO{MISO}, _pinMCLK{MCLK}, _pinINT{INT}, _devAddr{devAddr}, _SPI{mainSPI}
{
    // SETTING DEFAULT VALUES
    _CONFIG0_current.raw = CONFIG0_default;
    _CONFIG1_current.raw = CONFIG1_default;
    _CONFIG2_current.raw = CONFIG2_default;
    _CONFIG3_current.raw = CONFIG3_default;
    _IRQ_current.raw = IRQ_default;
    _MUX_current.raw = MUX_default;
    _SCAN_current.raw = SCAN_default;
    _TIMER_current  = TIMER_default;
    _OFFSETCAL_current = OFFSETCAL_default;
    _GAINCAL_current = GAINCAL_default;
    _LOCK_current = LOCK_default; // current must match default for register write access
}

bool MCP3564::begin()
{
    pinMode(_pinCS, OUTPUT);
    digitalWrite(_pinCS, HIGH); // disable CS 
/*
    pinMode(_pinSCK, OUTPUT);
    //digitalWrite(_pinSCK, LOW); // may not need this line

    pinMode(_pinMOSI, OUTPUT);
    pinMode(_pinMISO, INPUT);
*/  
    pinMode(_pinMCLK, OUTPUT);  // For CS_SEL = 0, meaning ext clock is used.
                                // Disable this if used internal clock instead.
    pinMode(_pinINT, INPUT_PULLUP);
    delay(10);
    return true;
}

void MCP3564::_transferAndReceive(const void* commandBuffer, void* statusBuffer, const void* dataBuffer, void* registerBuffer, uint8_t count)
{
    _SPI->beginTransaction(_defaultSPISettings);
    digitalWrite(_pinCS, LOW);

    _SPI->transfer(commandBuffer, statusBuffer, 1);
    _SPI->transfer(dataBuffer, registerBuffer, count);
    
    digitalWrite(_pinCS, HIGH);
    _SPI->endTransaction();
}

//int32_t readReg

volatile uint32_t MCP3564::readADCRawData32()
{
    //return readRegister24(ADCDATA_reg);

    Command_byte command;

    command.devAddr = _devAddr;// not so sure about the devAddress. Switch between 0,1,2,3 to see which one it is.
    command.fastCommand_or_RegAddr = ADCDATA_reg;
    command.CMD_type = STATIC_READ; 


    uint8_t rawCommandByte[1] = {command.raw};
    uint8_t rawStatusByte[1] = {0};
    uint8_t getThemBytes[4] = {0,0,0,0}; // ASSUMING DATA FORMAT IS 32 BIT 24 BIT LEFTJUSTIFIED!!!!!!
    uint8_t themBytes[4] = {0,0,0,0};

    _SPI->beginTransaction(_defaultSPISettings);
    digitalWrite(_pinCS, LOW);

#ifdef TEENSY4_X
    _SPI->transfer(rawCommandByte, rawStatusByte, 1);   // MAKE THIS A COMMAND INSTEAD, ALSO USE THE BUFFER VERSION. Also use the getter to get the status byte immedietaly after
                                                        // this readADCRawData32() function in main
    _status.raw = rawStatusByte[0];

    //_SPI->transfer(getThemBytes, themBytes, 4);
    //_adcRawData = (themBytes[0] << 24) | (themBytes[1] << 16) | (themBytes[2] << 8)  | themBytes[3] ;

    // data ready interrupt is cleared (Pulled 1 or HIGH) in 2 events:
    // 1. First falling edge of SCK during an *ADC Output register read*, (so it's pulled 1 when read like above)
    // meaning it's pulled HIGH right away when read begins();
    // 2. 16DMCLK clock periods before current conversion ends.

    // OR
/*
    union 
    {
        byte bytes[4];
        int32_t data = 0; 
    };

    _status.raw = _SPI->transfer(ADCDATA_reg); // MAKE THIS A COMMAND INSTEAD
    bytes[0] = _SPI->transfer(0);   // byte 1
    bytes[1] = _SPI->transfer(0);   // byte 2
    bytes[2] = _SPI->transfer(0);   // byte 3
    bytes[3] = _SPI->transfer(0);   // byte 4
    _adcRawData = data;
*/
#endif

#ifdef TEENSY3_x
    Serial.println("Teensy3_x");
#endif
    

    digitalWrite(_pinCS, HIGH);
    _SPI->endTransaction();
    return _adcRawData;

    // need to check again. might not done;
}

volatile uint32_t MCP3564::getADCRawData() const
{
    return _adcRawData;
}

void MCP3564::disableSCAN()
{
    _SCAN_current.raw = 0;
    writeRegister24(SCAN_reg, _SCAN_current.raw);
}

// for CONFIG0, CONFIG1, CONFIG2, CONFIG3, IRQ, MUX, LOCK
void MCP3564::writeRegister8(const uint8_t& addr, const uint8_t& data)
{
    Command_byte command;
    command.devAddr = _devAddr;// not so sure about the devAddress. Switch between 0,1,2,3 to see which one it is.
    command.fastCommand_or_RegAddr = addr;
    command.CMD_type = INCREMENTAL_WRITE; 
    

    uint8_t rawCommandByte[1] = {command.raw};
    uint8_t rawStatusByte[1] = {0};
    uint8_t dataByte[1] = {data};

    _SPI->beginTransaction(_defaultSPISettings);
    digitalWrite(_pinCS, LOW);

    _SPI->transfer(rawCommandByte, rawStatusByte, 1);   // Use the getter to get the status byte immedietaly after this readADCRawData32() function in main
    _status.raw = rawStatusByte[0];
    _SPI->transfer(dataByte, 1);

    digitalWrite(_pinCS, HIGH);
    _SPI->endTransaction();
}

// for SCAN, TIMER, OFFSETCAL, GAINCAL
void MCP3564::writeRegister24(const uint8_t& addr, const uint32_t& data)
{
    Command_byte command;
    command.devAddr = _devAddr;// not so sure about the devAddress. Switch between 0, 1, 2, 3 to see which one it is.
    command.fastCommand_or_RegAddr = addr;
    command.CMD_type = INCREMENTAL_WRITE; 

    uint8_t rawCommandByte[1] = {command.raw};
    uint8_t rawStatusByte[1] = {0};
    uint8_t dataBytes[4] = {0,0,0,0};
    dataBytes[0] = (data & 0x00FF0000) >> 16;
    dataBytes[1] = (data & 0x0000FF00) >> 8;
    dataBytes[2] = (data & 0x000000FF);

    _SPI->beginTransaction(_defaultSPISettings);
    digitalWrite(_pinCS, LOW);

    _SPI->transfer(rawCommandByte, rawStatusByte, 1);   // Use the getter to get the status byte immediately after this readADCRawData32() function in main
    _status.raw = rawStatusByte[0];
    _SPI->transfer(dataBytes, 4);

    digitalWrite(_pinCS, HIGH);
    _SPI->endTransaction();
}

void MCP3564::fastCommand(const uint8_t& cmd)
{
    _SPI->beginTransaction(_defaultSPISettings);
    digitalWrite(_pinCS, LOW);
    // try all 4 addresses, then determine which one is which
    _SPI->transfer(cmd - 64); // correspond to addr of 0
    _status.raw = _SPI->transfer(cmd); // 1
    _SPI->transfer(cmd + 64); // 2
    _SPI->transfer(cmd + 128); // 3

    digitalWrite(_pinCS, HIGH);
    _SPI->endTransaction();
}
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////// CONFIG0 register settings ///////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void MCP3564::setADCMode(const ADC_MODE& option) // [1:0]
{
    _CONFIG0_current.ADC_MODE = static_cast<uint8_t>(option);
}

void MCP3564::setSourceSink(const CS_SEL& option) // [3:2]
{
    _CONFIG0_current.CS_SEL = static_cast<uint8_t>(option);
}
void MCP3564::setClock(const CLK_SEL& option) // [5:4]
{
    _CONFIG0_current.CLK_SEL = static_cast<uint8_t>(option);
}
void MCP3564::setCONFIG0(const CONFIG0& option) // [7:6]
{
    _CONFIG0_current.CONFIG0 = static_cast<uint8_t>(option);
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////// CONFIG1 register settings ///////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void MCP3564::setOSR(const OSR& option) // [5:2]
{
    _CONFIG1_current.OSR = static_cast<uint8_t>(option);
}

void MCP3564::setPRE(const PRE& option) // [7:6]
{
    _CONFIG1_current.PRE = static_cast<uint8_t>(option);
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////// CONFIG2 register settings ///////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void MCP3564::setAZ_MUX(const AZ_MUX& option) // [2]
{
    _CONFIG2_current.AZ_MUX = static_cast<bool>(option);
}

void MCP3564::setGAIN(const GAIN& option) //[5:3]
{
    _CONFIG2_current.GAIN = static_cast<uint8_t>(option);
}

void MCP3564::setBOOST(const BOOST& option) // [7:6]
{
    _CONFIG2_current.BOOST = static_cast<uint8_t>(option);
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////// CONFIG3 register settings ///////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void MCP3564::setEN_GAINCAL(const EN_GAINCAL& option) //[0]
{
    _CONFIG3_current.EN_GAINCAL = static_cast<bool>(option);
}

void MCP3564::setEN_OFFCAL(const EN_OFFCAL& option) //[1]
{
    _CONFIG3_current.EN_OFFCAL = static_cast<bool>(option);
}

void MCP3564::setEN_CRCCOM(const EN_CRCCOM& option) //[2]
{
    _CONFIG3_current.EN_CRCCOM = static_cast<bool>(option);
}

void MCP3564::setCRC_FORMAT(const CRC_FORMAT& option) //[3]
{
    _CONFIG3_current.CRC_FORMAT = static_cast<bool>(option);
}

void MCP3564::setDATA_FORMAT(const DATA_FORMAT& option) // [5:4]
{
    _CONFIG3_current.DATA_FORMAT = static_cast<uint8_t>(option);
}

void MCP3564::setCONV_MODE(const CONV_MODE& option) //[7-6]
{
    _CONFIG3_current.CONV_MODE = static_cast<uint8_t>(option);
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////// IRQ register settings ///////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void MCP3564::setEN_STP(const EN_STP& option) //[0]
{
    _IRQ_current.EN_STP = static_cast<bool>(option);
}

void MCP3564::setEN_FASTCMD(const EN_FASTCMD& option) //[1]
{
    _IRQ_current.EN_FASTCMD = static_cast<bool>(option);
}

void MCP3564::setIRQ_MODE(const IRQ_MODE& option) //[3:2]
{
    _IRQ_current.IRQ_MODE = static_cast<uint8_t>(option);
}
////////////////////////////////////////////////////////////////////////////
/////////////////////////// MUX register ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void MCP3564::setMUX_VIN_neg(const MUX_VIN_neg& option) //[3:0]
{
    _MUX_current.MUX_VIN_neg = static_cast<uint8_t>(option);
}

void MCP3564::setMUX_VIN_pos(const MUX_VIN_pos& option) //[7:4]
{
    _MUX_current.MUX_VIN_pos = static_cast<uint8_t>(option);
}
////////////////////////////////////////////////////////////////////////////
/////////////////////////// SCAN register //////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void MCP3564::setSE_CH0(const bool& option)
{
    _SCAN_current.CH0 = static_cast<bool>(option);
}

void MCP3564::setSE_CH1(const bool& option)
{
    _SCAN_current.CH1 = static_cast<bool>(option);
}

void MCP3564::setSE_CH2(const bool& option)
{
    _SCAN_current.CH2 = static_cast<bool>(option);
}
void MCP3564::setSE_CH3(const bool& option)
{
    _SCAN_current.CH3 = static_cast<bool>(option);
}

void MCP3564::setSE_CH4(const bool& option)
{
    _SCAN_current.CH4 = static_cast<bool>(option);
}

void MCP3564::setSE_CH5(const bool& option)
{
    _SCAN_current.CH5 = static_cast<bool>(option);
}

void MCP3564::setSE_CH6(const bool& option)
{
    _SCAN_current.CH6 = static_cast<bool>(option);
}

void MCP3564::setSE_CH7(const bool& option)
{
    _SCAN_current.CH7 = static_cast<bool>(option);
}

void MCP3564::setDiff_CHA(const bool& option)
{
    _SCAN_current.Diff_CHA = static_cast<bool>(option);
}

void MCP3564::setDiff_CHB(const bool& option)
{
    _SCAN_current.Diff_CHB = static_cast<bool>(option);
}

void MCP3564::setDiff_CHC(const bool& option)
{
    _SCAN_current.Diff_CHC = static_cast<bool>(option);
}

void MCP3564::setDiff_CHD(const bool& option)
{
    _SCAN_current.Diff_CHD = static_cast<bool>(option);
}

void MCP3564::setTEMP(const bool& option)
{
    _SCAN_current.TEMP = static_cast<bool>(option);
}

void MCP3564::setAVdd(const bool& option)
{
    _SCAN_current.AVdd = static_cast<bool>(option);
}

void MCP3564::setVCM(const bool& option)
{
    _SCAN_current.VCM = static_cast<bool>(option);
}

void MCP3564::setOFFSET(const bool& option)
{
    _SCAN_current.OFFSET = static_cast<bool>(option);
}

void MCP3564::setDLY(const DLY& option) //[23:21]
{
    _SCAN_current.DLY = static_cast<uint8_t>(option);
}
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////// TIMER register //////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void MCP3564::setTIMER(const uint32_t& time)
{
    writeRegister24(TIMER_reg, time);
}
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////// OFFSETCAL register //////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void MCP3564::setOFFSETCAL(const int32_t& value)
{
    //transfer(OFFSETCAL_reg, static_cast<uint32_t>(value));
    union 
    {
        uint32_t unsignedValue;
        int32_t signedValue;
    };
    signedValue = value;

    //SPI.transfer the unsignedValue
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////// GAINCAL register ////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void MCP3564::setGAINCAL(const uint32_t& value)
{
    writeRegister24(GAINCAL_reg, value);
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////// LOCK register //////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void MCP3564::setLOCK()
{
    writeRegister8(LOCK_reg, static_cast<uint8_t>(69));
}

void MCP3564::setUNLOCK()
{
    writeRegister8(LOCK_reg, static_cast<uint8_t>(0xA5));
}

void MCP3564::updateAllRegValues()
{
    updateCONFIG0_current();
    updateCONFIG1_current();
    updateCONFIG2_current();
    updateCONFIG3_current();
    updateIRQ_current();
    updateMUX_current();
    updateSCAN_current();
    updateTIMER_current();
    updateOFFSETCAL_current();
    updateGAINCAL_current();
}

void MCP3564::updateCONFIG0_current()
{
    Command_byte command;
    command.devAddr = _devAddr;// not so sure about the devAddress. Switch between 0,1,2,3 to see which one it is.
    command.fastCommand_or_RegAddr = CONFIG0_reg;
    command.CMD_type = STATIC_READ; 

    uint8_t rawCommandByte[1] = {command.raw};
    uint8_t rawStatusByte[1] = {0};
    uint8_t discardByte[1] = {0};
    uint8_t registerByte[1] = {0};

    _transferAndReceive(rawCommandByte, rawStatusByte, discardByte, registerByte, 1);
/*
    _SPI->beginTransaction(_defaultSPISettings);
    digitalWrite(_pinCS, LOW);

    _SPI->transfer(rawCommandByte, rawStatusByte, 1);
    _SPI->transfer(discardByte, registerByte, 1);
    
    digitalWrite(_pinCS, HIGH);
    _SPI->endTransaction();
*/
    _status.raw = rawStatusByte[0];
    _CONFIG0_current.raw = registerByte[0];
}

void MCP3564::updateCONFIG1_current()
{
    Command_byte command;
    command.devAddr = _devAddr;// not so sure about the devAddress. Switch between 0,1,2,3 to see which one it is.
    command.fastCommand_or_RegAddr = CONFIG1_reg; 
    command.CMD_type = STATIC_READ; 

    uint8_t rawCommandByte[1] = {command.raw};
    uint8_t rawStatusByte[1] = {0};
    uint8_t discardByte[1] = {0};
    uint8_t registerByte[1] = {0};
    _transferAndReceive(rawCommandByte, rawStatusByte, discardByte, registerByte, 1);
    _status.raw = rawStatusByte[0];
    _CONFIG1_current.raw = registerByte[0];
}
void MCP3564::updateCONFIG2_current()
{
    Command_byte command;
    command.devAddr = _devAddr;// not so sure about the devAddress. Switch between 0,1,2,3 to see which one it is.
    Serial.println(_devAddr);
    Serial.println(command.devAddr);
    command.fastCommand_or_RegAddr = CONFIG2_reg;
    command.CMD_type = STATIC_READ; 

    uint8_t rawCommandByte[1] = {command.raw};
    uint8_t rawStatusByte[1] = {0};
    uint8_t discardByte[1] = {0};
    uint8_t registerByte[1] = {0};

    _transferAndReceive(rawCommandByte, rawStatusByte, discardByte, registerByte, 1);
    _status.raw = rawStatusByte[0];
    _CONFIG2_current.raw = registerByte[0];
}

void MCP3564::updateCONFIG3_current()
{
    Command_byte command;
    command.devAddr = _devAddr;// not so sure about the devAddress. Switch between 0,1,2,3 to see which one it is.
    command.fastCommand_or_RegAddr = CONFIG3_reg;
    command.CMD_type = STATIC_READ; 

    uint8_t rawCommandByte[1] = {command.raw};
    uint8_t rawStatusByte[1] = {0};
    uint8_t discardByte[1] = {0};
    uint8_t registerByte[1] = {0};

    _transferAndReceive(rawCommandByte, rawStatusByte, discardByte, registerByte, 1);
    _status.raw = rawStatusByte[0];
    _CONFIG3_current.raw = registerByte[0];
}
void MCP3564::updateIRQ_current()
{
    Command_byte command;
    command.devAddr = _devAddr;// not so sure about the devAddress. Switch between 0,1,2,3 to see which one it is.
    command.fastCommand_or_RegAddr = IRQ_reg;
    command.CMD_type = STATIC_READ; 

    uint8_t rawCommandByte[1] = {command.raw};
    uint8_t rawStatusByte[1] = {0};
    uint8_t discardByte[1] = {0};
    uint8_t registerByte[1] = {0};

    _transferAndReceive(rawCommandByte, rawStatusByte, discardByte, registerByte, 1);
    _status.raw = rawStatusByte[0];
    _IRQ_current.raw = registerByte[0];
}

void MCP3564::updateMUX_current()
{
    Command_byte command;
    command.devAddr = _devAddr;// not so sure about the devAddress. Switch between 0,1,2,3 to see which one it is.
    command.fastCommand_or_RegAddr = MUX_reg;
    command.CMD_type = STATIC_READ; 

    uint8_t rawCommandByte[1] = {command.raw};
    uint8_t rawStatusByte[1] = {0};
    uint8_t discardByte[1] = {0};
    uint8_t registerByte[1] = {0};

    _transferAndReceive(rawCommandByte, rawStatusByte, discardByte, registerByte, 1);
    _status.raw = rawStatusByte[0];
    _MUX_current.raw = registerByte[0];
}
void MCP3564::updateSCAN_current()
{
    Command_byte command;
    command.devAddr = _devAddr;// not so sure about the devAddress. Switch between 0,1,2,3 to see which one it is.
    command.fastCommand_or_RegAddr = SCAN_reg;
    command.CMD_type = STATIC_READ; 

    uint8_t rawCommandByte[1] = {command.raw};
    uint8_t rawStatusByte[1] = {0};
    uint8_t discardBytes[3] = {0,0,0};
    uint8_t registerBytes[3] = {0,0,0};

    _transferAndReceive(rawCommandByte, rawStatusByte, discardBytes, registerBytes, 3);
    _status.raw = rawStatusByte[0];
    _SCAN_current.raw = (registerBytes[0] << 16) | (registerBytes[1] << 8) | registerBytes[2];
}

void MCP3564::updateTIMER_current()
{
    Command_byte command;
    command.devAddr = _devAddr;// not so sure about the devAddress. Switch between 0,1,2,3 to see which one it is.
    command.fastCommand_or_RegAddr = TIMER_reg;
    command.CMD_type = STATIC_READ; 

    uint8_t rawCommandByte[1] = {command.raw};
    uint8_t rawStatusByte[1] = {0};
    uint8_t discardBytes[3] = {0,0,0};
    uint8_t registerBytes[3] = {0,0,0};

    _transferAndReceive(rawCommandByte, rawStatusByte, discardBytes, registerBytes, 3);
    _status.raw = rawStatusByte[0];
    _TIMER_current = (registerBytes[0] << 16) + (registerBytes[1] << 8) + registerBytes[2];
}

void MCP3564::updateOFFSETCAL_current()
{
    Command_byte command;
    command.devAddr = _devAddr;// not so sure about the devAddress. Switch between 0,1,2,3 to see which one it is.
    command.fastCommand_or_RegAddr = OFFSETCAL_reg;
    command.CMD_type = STATIC_READ; 

    uint8_t rawCommandByte[1] = {command.raw};
    uint8_t rawStatusByte[1] = {0};
    uint8_t discardBytes[3] = {0,0,0};
    uint8_t registerBytes[3] = {0,0,0};

    _transferAndReceive(rawCommandByte, rawStatusByte, discardBytes, registerBytes, 3);
    _status.raw = rawStatusByte[0];
    _OFFSETCAL_current = (registerBytes[0] << 16) | (registerBytes[1] << 8) | registerBytes[2];
}

void MCP3564::updateGAINCAL_current()
{
    Command_byte command;
    command.devAddr = _devAddr;// not so sure about the devAddress. Switch between 0,1,2,3 to see which one it is.
    command.fastCommand_or_RegAddr = GAINCAL_reg;
    command.CMD_type = STATIC_READ; 

    uint8_t rawCommandByte[1] = {command.raw};
    uint8_t rawStatusByte[1] = {0};
    uint8_t discardBytes[3] = {0,0,0};
    uint8_t registerBytes[3] = {0,0,0};

    _transferAndReceive(rawCommandByte, rawStatusByte, discardBytes, registerBytes, 3);
    _status.raw = rawStatusByte[0];
    _GAINCAL_current = (registerBytes[0] << 16) | (registerBytes[1] << 8) | registerBytes[2];
}
//MCP3564::