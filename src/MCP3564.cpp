#include "MCP3564.hpp"

MCP3564::MCP3564(uint8_t CS, uint8_t MOSI, uint8_t MISO, uint8_t SCK, uint8_t MCK, uint8_t INT, SPIClass* mainSPI)
    : _pinCS{CS}, _pinMOSI{MOSI}, _pinMISO{MISO}, _pinSCK{SCK}, _pinMCK{MCK}, _pinINT {INT}, _SPI{mainSPI}
    {};

bool MCP3564::begin()
{
    pinMode(_pinCS, OUTPUT);
    digitalWrite(_pinCS, HIGH); // disable CS 
    // set more
}

//int32_t readReg

int32_t MCP3564::getADCRawData() const
{
    return _adcRawData;
}

