#include "MCP3564.hpp"

MCP3564::MCP3564(uint8_t CS, uint8_t MOSI, uint8_t MISO, uint8_t SCK, uint8_t MCK, uint8_t INT, SPIClass* mainSPI)
    : _pinCS{CS}, _pinMOSI{MOSI}, _pinMISO{MISO}, _pinSCK{SCK}, _pinMCK{MCK}, _pinINT {INT}, _SPI{mainSPI}
    {};

bool MCP3564::begin()
{
    pinMode(_pinCS, OUTPUT);
    digitalWrite(_pinCS, HIGH); // disable CS 
    // set more

    delay(10);

    // Do a bunch of settings here


    delay(10);
}

//int32_t readReg

int32_t MCP3564::getADCRawData() const
{
    return _adcRawData;
}

// for CONFIG0, CONFIG1, CONFIG2, CONFIG3, IRQ, MUX, LOCK
void MCP3564::_transfer(const uint8_t& addr, const uint8_t& data)
{
    _SPI->beginTransaction(_defaultSPISettings);
    digitalWrite(_pinCS, LOW);
    _status = _SPI->transfer(addr); // begin talking to the register
    _SPI->transfer(data);
    digitalWrite(_pinCS, HIGH);
    _SPI->endTransaction();
}
// for SCAN, TIMER, OFFSETCAL, GAINCAL
void MCP3564::_transfer(const uint8_t& addr, const uint32_t& data)
{
    _SPI->beginTransaction(_defaultSPISettings);
    digitalWrite(_pinCS, LOW);
    _status = _SPI->transfer(addr); // begin talking to the register
    _SPI->transfer32(data);
    digitalWrite(_pinCS, HIGH);
    _SPI->endTransaction();
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////// CONFIG0 register settings ///////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void MCP3564::setADCMode(const ADC_MODE& option) // [0:1]
{
    switch (option)
    {
        case ADC_MODE::ADC_Shutdown_default:
        case ADC_MODE::ADC_Shutdown:
            bitClear(CONFIG0_current, 0);
            bitClear(CONFIG0_current, 1);
            break;
        case ADC_MODE::ADC_Standby:
            bitClear(CONFIG0_current, 0);
            bitSet(CONFIG0_current, 1);
            break;
        case ADC_MODE::ADC_Conversion: // WANT THIS SETTING
            bitSet(CONFIG0_current, 0);
            bitSet(CONFIG0_current, 1);
            break;
    }
}


void MCP3564::setSourceSink(const CS_SEL& option) // [3:2]
{
    switch (option)
    {
        case CS_SEL::CS_SEL_0MuA_default:
            bitClear(CONFIG0_current, 2);
            bitClear(CONFIG0_current, 3);
            break;
        case CS_SEL::CS_SEL_0p9MuA:
            bitSet(CONFIG0_current, 2);
            bitClear(CONFIG0_current, 3);
            break;
        case CS_SEL::CS_SEL_3p7MuA:
            bitClear(CONFIG0_current, 2);
            bitSet(CONFIG0_current, 3);
            break;
        case CS_SEL::CS_SEL_15MuA: 
            bitSet(CONFIG0_current, 2);
            bitSet(CONFIG0_current, 3);
            break;
    }
}
void MCP3564::setClock(const CLK_SEL& option) // [5:4]
{
    switch (option)
    {
        case CLK_SEL::ExtDigitalClock_default:
        case CLK_SEL::ExtDigitalClock:
            bitClear(CONFIG0_current, 4);
            bitClear(CONFIG0_current, 5);
            break;
        case CLK_SEL::IntClock_NoClockOutput:
            bitClear(CONFIG0_current, 4);
            bitSet(CONFIG0_current, 5);
            break;
        case CLK_SEL::IntClock_AMCLKPresent:
            bitSet(CONFIG0_current, 4);
            bitSet(CONFIG0_current, 5);  
            break;
    }
}
void MCP3564::setCONFIG0(const CONFIG0& option) // [7:6]
{
    switch (option)
    {
        case CONFIG0::FULL_SHUTDOWN:
            CONFIG0_current = 0;
            break;
        case CONFIG0::CONFIG0_default1:
        case CONFIG0::CONFIG0_default2:
        case CONFIG0::CONFIG0_default3:
            break;
    }
/*
    _SPI->beginTransaction(_defaultSPISettings);
    digitalWrite(_pinCS, LOW);
    _SPI->transfer(CONFIG0_reg); // begin talking to the register
    _SPI->transfer(CONFIG0_current);
    digitalWrite(_pinCS, HIGH);
    _SPI->endTransaction();
*/
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////// CONFIG1 register settings ///////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void MCP3564::setOSR(const OSR& option) // [5:2]
{
    switch (option)
    {
        case OSR::OSR_32:
            bitClear(CONFIG1_current, 2);
            bitClear(CONFIG1_current, 3);
            bitClear(CONFIG1_current, 4);
            bitClear(CONFIG1_current, 5);
            break;
        case OSR::OSR_64:
            bitSet(CONFIG1_current, 2);
            bitClear(CONFIG1_current, 3);
            bitClear(CONFIG1_current, 4);
            bitClear(CONFIG1_current, 5);
            break;
        case OSR::OSR_128:
            bitClear(CONFIG1_current, 2);
            bitSet(CONFIG1_current, 3);
            bitClear(CONFIG1_current, 4);
            bitClear(CONFIG1_current, 5);
            break;
        case OSR::OSR_256_default:
            bitSet(CONFIG1_current, 2);
            bitSet(CONFIG1_current, 3);
            bitClear(CONFIG1_current, 4);
            bitClear(CONFIG1_current, 5);
            break;
        case OSR::OSR_512:
            bitClear(CONFIG1_current, 2);
            bitClear(CONFIG1_current, 3);
            bitSet(CONFIG1_current, 4);
            bitClear(CONFIG1_current, 5);
            break;
        case OSR::OSR_1024:
            bitSet(CONFIG1_current, 2);
            bitClear(CONFIG1_current, 3);
            bitSet(CONFIG1_current, 4);
            bitClear(CONFIG1_current, 5);
            break;
        case OSR::OSR_2048:
            bitClear(CONFIG1_current, 2);
            bitSet(CONFIG1_current, 3);
            bitSet(CONFIG1_current, 4);
            bitClear(CONFIG1_current, 5);
            break;
        case OSR::OSR_4096:
            bitSet(CONFIG1_current, 2);
            bitSet(CONFIG1_current, 3);
            bitSet(CONFIG1_current, 4);
            bitClear(CONFIG1_current, 5);
            break;
        case OSR::OSR_8192:
            bitClear(CONFIG1_current, 2);
            bitClear(CONFIG1_current, 3);
            bitClear(CONFIG1_current, 4);
            bitSet(CONFIG1_current, 5);
            break;
        case OSR::OSR_16384:
            bitSet(CONFIG1_current, 2);
            bitClear(CONFIG1_current, 3);
            bitClear(CONFIG1_current, 4);
            bitSet(CONFIG1_current, 5);
            break;
        case OSR::OSR_20480:
            bitClear(CONFIG1_current, 2);
            bitSet(CONFIG1_current, 3);
            bitClear(CONFIG1_current, 4);
            bitSet(CONFIG1_current, 5);
            break;
        case OSR::OSR_24576:
            bitSet(CONFIG1_current, 2);
            bitSet(CONFIG1_current, 3);
            bitClear(CONFIG1_current, 4);
            bitSet(CONFIG1_current, 5);
            break;
        case OSR::OSR_40960:
            bitClear(CONFIG1_current, 2);
            bitClear(CONFIG1_current, 3);
            bitSet(CONFIG1_current, 4);
            bitSet(CONFIG1_current, 5);
            break;
        case OSR::OSR_49152:
            bitSet(CONFIG1_current, 2);
            bitClear(CONFIG1_current, 3);
            bitSet(CONFIG1_current, 4);
            bitSet(CONFIG1_current, 5);
            break;
        case OSR::OSR_81920:
            bitClear(CONFIG1_current, 2);
            bitSet(CONFIG1_current, 3);
            bitSet(CONFIG1_current, 4);
            bitSet(CONFIG1_current, 5);
            break;
        case OSR::OSR_98304:
            bitSet(CONFIG1_current, 2);
            bitSet(CONFIG1_current, 3);
            bitSet(CONFIG1_current, 4);
            bitSet(CONFIG1_current, 5);
            break;
    }
}

void MCP3564::setPRE(const PRE& option) // [7:6]
{
    switch (option)
    {
        case PRE::MCLK_Div1_default:
            bitClear(CONFIG1_current, 6);
            bitClear(CONFIG1_current, 7);
            break;
        case PRE::MCLK_Div2:
            bitSet(CONFIG1_current, 6);
            bitClear(CONFIG1_current, 7);
            break;
        case PRE::MCLK_Div4:
            bitClear(CONFIG1_current, 6);
            bitSet(CONFIG1_current, 7);
            break;
        case PRE::MCLK_Div8:
            bitSet(CONFIG1_current, 6);
            bitSet(CONFIG1_current, 7);
            break;
    }
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////// CONFIG2 register settings ///////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void MCP3564::setAZ_MUX(const AZ_MUX& option) // [2]
{
    switch (option)
    {
        case AZ_MUX::AZ_Disable_default:
            bitClear(CONFIG2_current, 2);
            break;
        case AZ_MUX::AZ_Enable:
            bitSet(CONFIG2_current, 2);
            break;
    }
}

void MCP3564::setGAIN(const GAIN& option) //[5:3]
{
    switch (option)
    {
        case GAIN::GAIN_x1div3:
            bitClear(CONFIG2_current, 3);
            bitClear(CONFIG2_current, 4);
            bitClear(CONFIG2_current, 5);
            break;
        case GAIN::GAIN_x1_default:
            bitSet(CONFIG2_current, 3);
            bitClear(CONFIG2_current, 4);
            bitClear(CONFIG2_current, 5);
            break;
        case GAIN::GAIN_x2:
            bitClear(CONFIG2_current, 3);
            bitSet(CONFIG2_current, 4);
            bitClear(CONFIG2_current, 5);
            break;
        case GAIN::GAIN_x4:
            bitSet(CONFIG2_current, 3);
            bitSet(CONFIG2_current, 4);
            bitClear(CONFIG2_current, 5);
            break;
        case GAIN::GAIN_x8:
            bitClear(CONFIG2_current, 3);
            bitClear(CONFIG2_current, 4);
            bitSet(CONFIG2_current, 5);
            break;
        case GAIN::GAIN_x16:
            bitSet(CONFIG2_current, 3);
            bitClear(CONFIG2_current, 4);
            bitSet(CONFIG2_current, 5);
            break;
        case GAIN::GAIN_x32:
            bitClear(CONFIG2_current, 3);
            bitSet(CONFIG2_current, 4);
            bitSet(CONFIG2_current, 5);
            break;
        case GAIN::GAIN_x64:
            bitSet(CONFIG2_current, 3);
            bitSet(CONFIG2_current, 4);
            bitSet(CONFIG2_current, 5);
            break;
    }
}

void MCP3564::setBOOST(const BOOST& option) // [7:6]
{
    switch (option)
    {
        case BOOST::ADC_BIAS_x0p5:
            bitClear(CONFIG2_current, 6);
            bitClear(CONFIG2_current, 7);
            break;
        case BOOST::ADC_BIAS_x0p66:
            bitSet(CONFIG2_current, 6);
            bitClear(CONFIG2_current, 7);
            break;
        case BOOST::ADC_BIAS_x1:
            bitClear(CONFIG2_current, 6);
            bitSet(CONFIG2_current, 7);
            break;
        case BOOST::ADC_BIAS_x2:
            bitSet(CONFIG2_current, 6);
            bitSet(CONFIG2_current, 7);
            break;
    }
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////// CONFIG3 register settings ///////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void MCP3564::setEN_GAINCAL(const EN_GAINCAL& option) //[0]
{
    
    switch (option)
    {
        case EN_GAINCAL::Disabled_default:
            bitClear(CONFIG3_current, 0);
            break;
        case EN_GAINCAL::Enabled:
            bitSet(CONFIG3_current, 0);
            break;
    }
}

void MCP3564::setEN_OFFCAL(const EN_OFFCAL& option) //[1]
{
    switch (option)
    {
        case EN_OFFCAL::Disabled_default:
            bitClear(CONFIG3_current, 1);
            break;
        case EN_OFFCAL::Enabled:
            bitSet(CONFIG3_current, 1);
            break;
    }
}

void MCP3564::setEN_CRCCOM(const EN_CRCCOM& option) //[2]
{
    switch (option)
    {
        case EN_CRCCOM::CRC_OnCom_Disabled_default:
            bitClear(CONFIG3_current, 2);
            break;
        case EN_CRCCOM::CRC_OnCom_Enabled:
            bitSet(CONFIG3_current, 2);
            break;
    }
}

void MCP3564::setCRC_FORMAT(const CRC_FORMAT& option) //[3]
{
    switch (option)
    {
        case CRC_FORMAT::CRC_16bit_default:
            bitClear(CONFIG3_current, 3);
            break;
        case CRC_FORMAT::CRC_32bit:
            bitSet(CONFIG3_current, 3);
            break;
    }
}

void MCP3564::setDATA_FORMAT(const DATA_FORMAT& option) // [5:4]
{
    switch (option)
    {
        case DATA_FORMAT::FORMAT_24bit_default:
            bitClear(CONFIG3_current, 4);
            bitClear(CONFIG3_current, 5);
            break;
        case DATA_FORMAT::FORMAT_32bit_24BitLeftJustified:
            bitSet(CONFIG3_current, 4);
            bitClear(CONFIG3_current, 5);
            break;
        case DATA_FORMAT::FORMAT_32bit_25BitRightJustified:
            bitClear(CONFIG3_current, 4);
            bitSet(CONFIG3_current, 5);
            break;
        case DATA_FORMAT::FORMAT_32bit_25BitRightJustified_CHID:
            bitSet(CONFIG3_current, 4);
            bitSet(CONFIG3_current, 5);
            break;
    }
}

void MCP3564::setCONV_MODE(const CONV_MODE& option) //[7-6]
{
    switch (option)
    {
        case CONV_MODE::Oneshot_Shutdown_default1:
        case CONV_MODE::Oneshot_Shutdown_default2:
            bitClear(CONFIG3_current, 6);
            bitClear(CONFIG3_current, 7);
            break;
        case CONV_MODE::Oneshot_Standby:
            bitClear(CONFIG3_current, 6);
            bitSet(CONFIG3_current, 7);
            break;
        case CONV_MODE::Continuous:
            bitSet(CONFIG3_current, 6);
            bitSet(CONFIG3_current, 7);
            break;
    }
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////// IRQ register settings ///////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void MCP3564::setEN_STP(const EN_STP& option) //[0]
{

    switch (option)
    {
        case EN_STP::Disabled:
            bitClear(IRQ_current, 0);
            break;
        case EN_STP::Enabled_default:
            bitSet(IRQ_current, 0);
            break; 
    }
}

void MCP3564::setEN_FASTCMD(const EN_FASTCMD& option) //[1]
{
    switch (option)
    {
        case EN_FASTCMD::Disabled:
            bitClear(IRQ_current, 1);
            break;
        case EN_FASTCMD::Enabled_default:
            bitSet(IRQ_current, 1);
            break; 
    }
}

void MCP3564::setIRQ_MODE(const IRQ_MODE& option) //[3:2]
{
    switch (option)
    {
        case IRQ_MODE::IRQ_HighZ:
            bitClear(IRQ_current, 2);
            bitClear(IRQ_current, 3);
            break;
        case IRQ_MODE::IRQ_LogicHigh:
            bitSet(IRQ_current, 2);
            bitClear(IRQ_current, 3);
            break;
        case IRQ_MODE::MDAT_HighZ:
            bitClear(IRQ_current, 2);
            bitSet(IRQ_current, 3);
            break;
        case IRQ_MODE::MDAT_LogicHigh:
            bitSet(IRQ_current, 2);
            bitSet(IRQ_current, 3);
            break;
    }
}
////////////////////////////////////////////////////////////////////////////
/////////////////////////// MUX register ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void MCP3564::setMUX_VIN_neg(const MUX_VIN_neg& option) //[3:0]
{
    switch (option)
    {
        case MUX_VIN_neg::CH0:
            bitClear(MUX_current, 0);
            bitClear(MUX_current, 1);
            bitClear(MUX_current, 2);
            bitClear(MUX_current, 3);
            break;
        case MUX_VIN_neg::CH1_default:
            bitSet(MUX_current, 0);
            bitClear(MUX_current, 1);
            bitClear(MUX_current, 2);
            bitClear(MUX_current, 3);
            break;
        case MUX_VIN_neg::CH2:
            bitClear(MUX_current, 0);
            bitSet(MUX_current, 1);
            bitClear(MUX_current, 2);
            bitClear(MUX_current, 3);
            break;
        case MUX_VIN_neg::CH3:
            bitSet(MUX_current, 0);
            bitSet(MUX_current, 1);
            bitClear(MUX_current, 2);
            bitClear(MUX_current, 3);
            break;
        case MUX_VIN_neg::CH4:
            bitClear(MUX_current, 0);
            bitClear(MUX_current, 1);
            bitSet(MUX_current, 2);
            bitClear(MUX_current, 3);
            break;
        case MUX_VIN_neg::CH5:
            bitSet(MUX_current, 0);
            bitClear(MUX_current, 1);
            bitSet(MUX_current, 2);
            bitClear(MUX_current, 3);
            break;
        case MUX_VIN_neg::CH6:
            bitClear(MUX_current, 0);
            bitSet(MUX_current, 1);
            bitSet(MUX_current, 2);
            bitClear(MUX_current, 3);
            break;
        case MUX_VIN_neg::CH7:
            bitSet(MUX_current, 0);
            bitSet(MUX_current, 1);
            bitSet(MUX_current, 2);
            bitClear(MUX_current, 3);
            break;
        case MUX_VIN_neg::A_GND:
            bitClear(MUX_current, 0);
            bitClear(MUX_current, 1);
            bitClear(MUX_current, 2);
            bitSet(MUX_current, 3);
            break;
        case MUX_VIN_neg::A_VDD:
            bitSet(MUX_current, 0);
            bitClear(MUX_current, 1);
            bitClear(MUX_current, 2);
            bitSet(MUX_current, 3);
            break;
        case MUX_VIN_neg::REF_IN_POS:
            bitSet(MUX_current, 0);
            bitSet(MUX_current, 1);
            bitClear(MUX_current, 2);
            bitSet(MUX_current, 3);
            break;
        case MUX_VIN_neg::REF_IN_NEG:
            bitClear(MUX_current, 0);
            bitClear(MUX_current, 1);
            bitSet(MUX_current, 2);
            bitSet(MUX_current, 3);
            break;
        case MUX_VIN_neg::Temp_Diode_P:
            bitSet(MUX_current, 0);
            bitClear(MUX_current, 1);
            bitSet(MUX_current, 2);
            bitSet(MUX_current, 3);
            break;
        case MUX_VIN_neg::Temp_Diode_M:
            bitClear(MUX_current, 0);
            bitSet(MUX_current, 1);
            bitSet(MUX_current, 2);
            bitSet(MUX_current, 3);
            break;
        case MUX_VIN_neg::Internal_VCM:
            bitSet(MUX_current, 0);
            bitSet(MUX_current, 1);
            bitSet(MUX_current, 2);
            bitSet(MUX_current, 3);
            break;
    }
}

void MCP3564::setMUX_VIN_pos(const MUX_VIN_pos& option) //[7:4]
{
    switch (option)
    {
        case MUX_VIN_pos::CH0_default:
            bitClear(MUX_current, 4);
            bitClear(MUX_current, 5);
            bitClear(MUX_current, 6);
            bitClear(MUX_current, 7);
        case MUX_VIN_pos::CH1:
            bitSet(MUX_current, 4);
            bitClear(MUX_current, 5);
            bitClear(MUX_current, 6);
            bitClear(MUX_current, 7);
            break;
        case MUX_VIN_pos::CH2:
            bitClear(MUX_current, 4);
            bitSet(MUX_current, 5);
            bitClear(MUX_current, 6);
            bitClear(MUX_current, 7);
            break;
        case MUX_VIN_pos::CH3:
            bitSet(MUX_current, 4);
            bitSet(MUX_current, 5);
            bitClear(MUX_current, 6);
            bitClear(MUX_current, 7);
            break;
        case MUX_VIN_pos::CH4:
            bitClear(MUX_current, 4);
            bitClear(MUX_current, 5);
            bitSet(MUX_current, 6);
            bitClear(MUX_current, 7);
        case MUX_VIN_pos::CH5:
            bitSet(MUX_current, 4);
            bitClear(MUX_current, 5);
            bitSet(MUX_current, 6);
            bitClear(MUX_current, 7);
            break;
        case MUX_VIN_pos::CH6:
            bitClear(MUX_current, 4);
            bitSet(MUX_current, 5);
            bitSet(MUX_current, 6);
            bitClear(MUX_current, 7);
            break;
        case MUX_VIN_pos::CH7:
            bitSet(MUX_current, 4);
            bitSet(MUX_current, 5);
            bitSet(MUX_current, 6);
            bitClear(MUX_current, 7);
            break;
        case MUX_VIN_pos::A_GND:
            bitClear(MUX_current, 4);
            bitClear(MUX_current, 5);
            bitClear(MUX_current, 6);
            bitSet(MUX_current, 7);
            break;
        case MUX_VIN_pos::A_VDD:
            bitSet(MUX_current, 4);
            bitClear(MUX_current, 5);
            bitClear(MUX_current, 6);
            bitSet(MUX_current, 7);
            break;
        case MUX_VIN_pos::REF_IN_POS:
            bitSet(MUX_current, 4);
            bitSet(MUX_current, 5);
            bitClear(MUX_current, 6);
            bitSet(MUX_current, 7);
            break;
        case MUX_VIN_pos::REF_IN_NEG:
            bitClear(MUX_current, 4);
            bitClear(MUX_current, 5);
            bitSet(MUX_current, 6);
            bitSet(MUX_current, 7);
            break;
        case MUX_VIN_pos::Temp_Diode_P:
            bitSet(MUX_current, 4);
            bitClear(MUX_current, 5);
            bitSet(MUX_current, 6);
            bitSet(MUX_current, 7);
            break;
        case MUX_VIN_pos::Temp_Diode_M: 
            bitClear(MUX_current, 4);
            bitSet(MUX_current, 5);
            bitSet(MUX_current, 6);
            bitSet(MUX_current, 7);
            break;
        case MUX_VIN_pos::Internal_VCM:
            bitSet(MUX_current, 4);
            bitSet(MUX_current, 5);
            bitSet(MUX_current, 6);
            bitSet(MUX_current, 7);
            break;
    }
}
////////////////////////////////////////////////////////////////////////////
/////////////////////////// SCAN register //////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void MCP3564::setSE_CH0(const bool& option)
{
    bitWrite(SCAN_current, 0, option);
}

void MCP3564::setSE_CH1(const bool& option)
{
    bitWrite(SCAN_current, 1, option);
}

void MCP3564::setSE_CH2(const bool& option)
{
    bitWrite(SCAN_current, 2, option);
}
void MCP3564::setSE_CH3(const bool& option)
{
    bitWrite(SCAN_current, 3, option);
}

void MCP3564::setSE_CH4(const bool& option)
{
    bitWrite(SCAN_current, 4, option);
}

void MCP3564::setSE_CH5(const bool& option)
{
    bitWrite(SCAN_current, 5, option);
}

void MCP3564::setSE_CH6(const bool& option)
{
    bitWrite(SCAN_current, 6, option);
}

void MCP3564::setSE_CH7(const bool& option)
{
    bitWrite(SCAN_current, 7, option);
}

void MCP3564::setDiff_CHA(const bool& option)
{
    bitWrite(SCAN_current, 8, option);
}

void MCP3564::setDiff_CHB(const bool& option)
{
    bitWrite(SCAN_current, 9, option);
}

void MCP3564::setDiff_CHC(const bool& option)
{
    bitWrite(SCAN_current, 10, option);
}

void MCP3564::setDiff_CHD(const bool& option)
{
    bitWrite(SCAN_current, 11, option);
}

void MCP3564::setTEMP(const bool& option)
{
    bitWrite(SCAN_current, 12, option);
}

void MCP3564::setA_VDD(const bool& option)
{
    bitWrite(SCAN_current, 13, option);
}

void MCP3564::setVCM(const bool& option)
{
    bitWrite(SCAN_current, 14, option);
}

void MCP3564::setOFFSET(const bool& option)
{
    bitWrite(SCAN_current, 15, option);
}

void MCP3564::setDLY(const DLY& option) //[23:21]
{
    switch (option)
    {
        case DLY::DMCLKMul0_default:
            bitClear(SCAN_current, 21);
            bitClear(SCAN_current, 22);
            bitClear(SCAN_current, 23);
            break;
        case DLY::DMCLKMul8:
            bitSet(SCAN_current, 21);
            bitClear(SCAN_current, 22);
            bitClear(SCAN_current, 23);
            break;
        case DLY::DMCLKMul16:
            bitClear(SCAN_current, 21);
            bitSet(SCAN_current, 22);
            bitClear(SCAN_current, 23);
            break;
        case DLY::DMCLKMul32:
            bitSet(SCAN_current, 21);
            bitSet(SCAN_current, 22);
            bitClear(SCAN_current, 23);
            break;
        case DLY::DMCLKMul64:
            bitClear(SCAN_current, 21);
            bitClear(SCAN_current, 22);
            bitSet(SCAN_current, 23);
            break;
        case DLY::DMCLKMul128:
            bitSet(SCAN_current, 21);
            bitClear(SCAN_current, 22);
            bitSet(SCAN_current, 23);
            break;
        case DLY::DMCLKMul256:
            bitClear(SCAN_current, 21);
            bitSet(SCAN_current, 22);
            bitSet(SCAN_current, 23);
            break;
        case DLY::DMCLKMul512:
            bitSet(SCAN_current, 21);
            bitSet(SCAN_current, 22);
            bitSet(SCAN_current, 23);
            break;
    }
}
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////// TIMER register //////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void MCP3564::setTIMER(const uint32_t& time)
{
    _transfer(TIMER_reg, time);
}
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////// OFFSETCAL register //////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void MCP3564::setOFFSETCAL(const int32_t& value)
{
    _transfer(OFFSETCAL_reg, static_cast<uint32_t>(value));
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////// GAINCAL register ////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void MCP3564::setGAINCAL(const uint32_t& value)
{
    _transfer(GAINCAL_reg, value);
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////// LOCK register //////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void MCP3564::setLOCK()
{
    _transfer(LOCK_reg, static_cast<uint8_t>(69));
}

void MCP3564::setUNLOCK()
{
    _transfer(LOCK_reg, static_cast<uint8_t>(0xA5));
}


//MCP3564::