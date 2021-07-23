/******************************************************************//**
* @file lib_MAX31856.cpp
*
* @author Devin Alexander
*
* @version 1.0
*
* Started: SEPTEMBER 14th 2017
*
* Updated: Jully 2021 By Yannic Simon
*
* @brief Source file for MAX3185 class
*
***********************************************************************
*
* @copyright 
* Copyright (C) 2015 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
**********************************************************************/
#include "lib_MAX31856.h"

#define LOG(args...)    printf(args)

//*****************************************************************************
MAX31856::MAX31856(SPI& _spi, PinName _ncs, uint8_t _type, uint8_t _fltr, uint8_t _samples, uint8_t _conversion_mode) : spi(_spi), ncs(_ncs), samples(_samples)
{
    spi.format(8,3); //configure the correct SPI mode to beable to program the registers intially correctly
    init_MAX31856 &= setThermocoupleType(_type);
    init_MAX31856 &= setEmiFilterFreq(_fltr);
    init_MAX31856 &= setNumSamplesAvg(_samples);
    init_MAX31856 &= setConversionMode(_conversion_mode);
    lastReadTime = time(NULL);
    wait_us(1000000);
}


//*****************************************************************************
float MAX31856::readTC()
{
    //Check and see if the MAX31856 is set to conversion mode ALWAYS ON
    if (conversion_mode==0) {   //means that the conversion mode is normally off
        init_MAX31856 &= setOneShotMode(CR0_1_SHOT_MODE_ONE_CONVERSION); // turn on the one shot mode for singular conversion
        thermocouple_conversion_count=0; //reset the conversion count back to zero to make sure minimum conversion time reflects one shot mode requirements
    }
    if(!init_MAX31856) return NAN;
    //calculate minimum wait time for conversions
    calculateDelayTime();
    //initialize other info for the read functionality
    uint32_t buf_read[3] = {0}, buf_write[3] = {ADDRESS_LTCBH_READ, ADDRESS_LTCBM_READ, ADDRESS_LTCBL_READ};
    if(checkFaultsThermocoupleConnection()) //no faults with connection are present so continue on with normal read of temperature
    {
        uint32_t tm = time(NULL)*1000000;
        uint32_t duration = tm - lastReadTime;
        lastReadTime = tm;
        if (duration > conversion_time)
        {
            for(int i=0; i<3; i++) buf_read[i] = registerReadByte(buf_write[i]);
            //Convert the registers contents into the correct value
            int32_t temp  = ((buf_read[0] & 0xFF) << 0x18) + ((buf_read[1] & 0xFF) << 0x10) + ((buf_read[2] & 0xFF) << 0x08);   // LTCBH + LTCBM + LTCBL
            return prev_TC = (temp >> 0x0D) * 0.0078125;
        }
    }
    thermocouple_conversion_count++; //iterate the conversion count to speed up time in between future converions in always on mode
    checkFaultsThermocoupleThresholds();  //print any faults to the terminal
    return prev_TC;
}


//*****************************************************************************
float MAX31856::readCJ()
{
    if(!init_MAX31856) return NAN;
    uint32_t buf_read[2] = {0}, buf_write[2] = {ADDRESS_CJTH_READ, ADDRESS_CJTL_READ};
    for(int i=0; i<2; i++) buf_read[i] = registerReadByte(buf_write[i]);
    int16_t temp  = ((buf_read[0] & 0xFF) << 8) + (buf_read[1] & 0xFF); // CJTH + CJTL
    return temp/256.0;
}

//*****************************************************************************
uint8_t MAX31856::checkFaultsThermocoupleThresholds()
{  
    uint8_t fault_byte=registerReadByte(ADDRESS_SR_READ); //Read contents of fault status register
    uint8_t temp[2], return_int;
    for(int i=0; i<2; i++)
        temp[i]=fault_byte;
    
    //Check if any of the faults for thermocouple connection are triggered
    if      ((fault_byte&0x4C)==0) //means no fault is detected for thermocouple thresholds
        return_int=0;
    else {
        if ((fault_byte&0x40)==0) {   //check if normal operation of thermocouple is true
            if      (temp[0]&0x08) {
                LOG("FAULT! Thermocouple temp is higher than the threshold that is set!\r\n");
                return_int=1;
            }
            else if (temp[1]&0x04) {
                LOG("FAULT! Thermocouple temp is lower than the threshold that is set!\r\n");
                return_int=2;
            }
        }
        else {                      //Thermocouples is operating outside of normal range
            LOG("FAULT! Thermocouple temperature is out of range for specific type of thermocouple!\r\n");
            if      (temp[0]&0x08) {
                LOG("FAULT! Thermocouple temp is higher than the threshold that is set!\r\n");
                return_int=4;
            }
            else if (temp[1]&0x04) {
                LOG("FAULT! Thermocouple temp is lower than the threshold that is set!\r\n");
                return_int=5;
            }
            else                    //no other faults are flagged besides unnatural operation
                return_int=3; 
        }
    }
    return return_int;
}

//*****************************************************************************
uint8_t MAX31856::checkFaultsColdJunctionThresholds()
{  
    uint8_t fault_byte=registerReadByte(ADDRESS_SR_READ); //Read contents of fault status register
    uint8_t temp[2], return_int;
    for(int i=0; i<2; i++)
        temp[i]=fault_byte;
    
    //Check if any of the faults for thermocouple connection are triggered
    if ((fault_byte&0xB0)==0)  //means no fault is detected for cold junction thresholds
        return_int=0;
    else {
        if ((fault_byte&0x80)==0) {   //check if normal operation of cold junction is true
            if      (temp[0]&0x20) {
                LOG("FAULT! Cold Junction temp is higher than the threshold that is set!\r\n");
                return_int=1;
            }
            else if (temp[1]&0x10) {
                LOG("FAULT! Cold Junction temp is lower than the threshold that is set!\r\n");
                return_int=2;
            }
        }
        else {                      //Cold Junction is operating outside of normal range
            LOG("FAULT! Cold Junction temperature is out of range for specific type of thermocouple!\r\n");
            if      (temp[0]&0x20) {
                LOG("FAULT! Cold Junction temp is higher than the threshold that is set!\r\n");
                return_int=4;
            }
            else if (temp[1]&0x10) {
                LOG("FAULT! Cold Junction temp is lower than the threshold that is set!\r\n");
                return_int=5;
            }
            else                    //no other faults are flagged besides unnatural operation
                return_int=3;
        }
    }
    return return_int;
}

//*****************************************************************************
bool MAX31856::checkFaultsThermocoupleConnection()
{
    return !registerReadByte(ADDRESS_SR_READ);  //Read contents of fault status register
}


//Register:CR0    Bits: 7
//*****************************************************************************
bool MAX31856::setConversionMode(uint8_t val) 
{
    switch(val)
    {
        case CR0_CONV_MODE_NORMALLY_OFF: case CR0_CONV_MODE_NORMALLY_ON:
            conversion_mode = (val == CR0_CONV_MODE_NORMALLY_ON)?1:0;
            return registerReadWriteByte(ADDRESS_CR0_READ, ADDRESS_CR0_WRITE, CR0_CLEAR_BITS_7, val);
        break;
        default:
            //LOG("Incorrect parameter selected for Control Register 0 (CR0) bit 7. Default value not changed.\r\nPlease see MAX31856.h for list of valid parameters. \r\n"); 
            return false;
        break;
    }
}


//Register:CR0    Bits: 6
//*****************************************************************************
bool MAX31856::setOneShotMode(uint8_t val) 
{
    switch(val)
    {
        case CR0_1_SHOT_MODE_NO_CONVERSION: case CR0_1_SHOT_MODE_ONE_CONVERSION:
            return registerReadWriteByte(ADDRESS_CR0_READ, ADDRESS_CR0_WRITE, CR0_CLEAR_BITS_6, val);
        break;
        default:
            //LOG("Incorrect parameter selected for Control Register 0 (CR0) bit 6. Default value not changed.\r\nPlease see MAX31856.h for list of valid parameters. \r\n");
            return false;
        break;
    }
}


//Register:CR0    Bits: 5:4
//*****************************************************************************
bool MAX31856::setOpenCircuitFaultDetection(uint8_t val) 
{
    switch(val)
    {
        case CR0_OC_DETECT_DISABLED: case CR0_OC_DETECT_ENABLED_R_LESS_5k: case CR0_OC_DETECT_ENABLED_TC_LESS_2ms: case CR0_OC_DETECT_ENABLED_TC_MORE_2ms:
            return registerReadWriteByte(ADDRESS_CR0_READ, ADDRESS_CR0_WRITE, CR0_CLEAR_BITS_5_4, val);
        break;
        default:
            //LOG("Incorrect parameter selected for Control Register 0 (CR0) bits 5:4. Default value not changed.\r\nPlease see MAX31856.h for list of valid parameters. \r\n"); 
            return false;
        break;
    }
}


//Register:CR0    Bits: 3
//*****************************************************************************
bool MAX31856::setColdJunctionDisable(uint8_t val) 
{
    switch(val)
    {
        case CR0_COLD_JUNC_ENABLE: case CR0_COLD_JUNC_DISABLE:
            cold_junction_enabled = (val==CR0_COLD_JUNC_ENABLE)?1:0;
            return registerReadWriteByte(ADDRESS_CR0_READ, ADDRESS_CR0_WRITE, CR0_CLEAR_BITS_3, val);
        break;
        default:
            //LOG("Incorrect parameter selected for Control Register 0 (CR0) bit 3. Default value not changed.\r\nPlease see MAX31856.h for list of valid parameters. \r\n"); 
            return false;
        break;
    }
}


//Register:CR0    Bits: 2
//*****************************************************************************
bool MAX31856::setFaultMode(uint8_t val) 
{
    switch(val)
    {
        case CR0_FAULT_MODE_COMPARATOR: case CR0_FAULT_MODE_INTERUPT:
            return registerReadWriteByte(ADDRESS_CR0_READ, ADDRESS_CR0_WRITE, CR0_CLEAR_BITS_2, val);
        break;
        default:
            //LOG("Incorrect parameter selected for Control Register 0 (CR0) bit 2. Default value not changed.\r\nPlease see MAX31856.h for list of valid parameters. \r\n"); 
            return false;
        break;
    }
}


//Register:CR0    Bits: 1
//*****************************************************************************
bool MAX31856::setFaultStatusClear(uint8_t val) 
{
    switch(val)
    {
        case CR0_FAULTCLR_DEFAULT_VAL: case CR0_FAULTCLR_RETURN_FAULTS_TO_ZERO:
            return registerReadWriteByte(ADDRESS_CR0_READ, ADDRESS_CR0_WRITE, CR0_CLEAR_BITS_1, val);
        break;
        default:
            //LOG("Incorrect parameter selected for Control Register 0 (CR0) bit 1. Default value not changed.\r\nPlease see MAX31856.h for list of valid parameters. \r\n"); 
            return false;
        break;
    }
}


//Register:CR0    Bits: 0
//*****************************************************************************
bool MAX31856::setEmiFilterFreq(uint8_t val) 
{
    switch(val)
    {
        case CR0_FILTER_OUT_60Hz: case CR0_FILTER_OUT_50Hz:
            filter_mode = val;
            return registerReadWriteByte(ADDRESS_CR0_READ, ADDRESS_CR0_WRITE, CR0_CLEAR_BITS_0, val);
        break;
        default:
            //LOG("Incorrect parameter selected for Control Register 0 (CR0) bit 0. Default value not changed.\r\nPlease see MAX31856.h for list of valid parameters. \r\n"); 
            return false;
        break;
    }
}


//Register:CR1    Bits: 6:4
//*****************************************************************************
bool MAX31856::setNumSamplesAvg(uint8_t val) 
{
    switch(val)
    {
        case CR1_AVG_TC_SAMPLES_1: case CR1_AVG_TC_SAMPLES_2: case CR1_AVG_TC_SAMPLES_4: case CR1_AVG_TC_SAMPLES_8: case CR1_AVG_TC_SAMPLES_16:
            samples = 1 << (val >> 4);
            return registerReadWriteByte(ADDRESS_CR1_READ, ADDRESS_CR1_WRITE, CR1_CLEAR_BITS_6_4, val);
        break;
        default:
            //LOG("Incorrect parameter selected for Control Register 1 (CR1) bits 6:4. Default value not changed.\r\nPlease see MAX31856.h for list of valid parameters. \r\n"); 
            return false;
        break;
    }
}


//Register:CR1    Bits: 3:0
//*****************************************************************************
bool MAX31856::setThermocoupleType(uint8_t val) 
{
    switch(val)
    {
        case CR1_TC_TYPE_B: case CR1_TC_TYPE_E: case CR1_TC_TYPE_J: case CR1_TC_TYPE_K: case CR1_TC_TYPE_N: case CR1_TC_TYPE_R: case CR1_TC_TYPE_S: case CR1_TC_TYPE_T: case CR1_TC_TYPE_VOLT_MODE_GAIN_8: case CR1_TC_TYPE_VOLT_MODE_GAIN_32:
            voltage_mode = ((val == CR1_TC_TYPE_VOLT_MODE_GAIN_8) || (val == CR1_TC_TYPE_VOLT_MODE_GAIN_32));
            return registerReadWriteByte(ADDRESS_CR1_READ, ADDRESS_CR1_WRITE, CR1_CLEAR_BITS_3_0, val);
        break;
        default:
            //LOG("Incorrect parameter selected for Control Register 1 (CR1) bits 3:0. Default value not changed.\r\nPlease see MAX31856.h for list of valid parameters. \r\n"); 
            return false;
        break;
    }
}


//Register:MASK    Bits: 5:0
//*****************************************************************************
bool MAX31856::setFaultMasks(uint8_t val, bool enable) 
{
    if(enable) val = 0;
    switch(val)
    {
        case MASK_CJ_FAULT_THRESHOLD_HIGH:
            return registerReadWriteByte(ADDRESS_MASK_READ, ADDRESS_MASK_WRITE, MASK_CLEAR_BITS_5, val);
        break;
        case MASK_CJ_FAULT_THRESHOLD_LOW:
            return registerReadWriteByte(ADDRESS_MASK_READ, ADDRESS_MASK_WRITE, MASK_CLEAR_BITS_4, val);
        break;
        case MASK_TC_FAULT_THRESHOLD_HIGH:
            return registerReadWriteByte(ADDRESS_MASK_READ, ADDRESS_MASK_WRITE, MASK_CLEAR_BITS_3, val);
        break;
        case MASK_TC_FAULT_THRESHOLD_LOW:
            return registerReadWriteByte(ADDRESS_MASK_READ, ADDRESS_MASK_WRITE, MASK_CLEAR_BITS_2, val);
        break;
        case MASK_OVER_UNDER_VOLT_FAULT:
            return registerReadWriteByte(ADDRESS_MASK_READ, ADDRESS_MASK_WRITE, MASK_CLEAR_BITS_1, val);
        break;
        case MASK_OPEN_CIRCUIT_FAULT:
            return registerReadWriteByte(ADDRESS_MASK_READ, ADDRESS_MASK_WRITE, MASK_CLEAR_BITS_0, val);
        break;
        default:
            //LOG("Incorrect parameter selected for Mask Register bits 5:0. Default value not changed.\r\nPlease see MAX31856.h for list of valid parameters. \r\n"); 
            return false;
        break;
    }
}


//Register:MASK    Bits: 5:0
//******************************************************************************
bool MAX31856::setFaultThresholds(uint8_t val, float temperature) 
{
    switch(val)
    {
        case MASK_CJ_FAULT_THRESHOLD_HIGH:
            return registerWriteByte(ADDRESS_CJHF_WRITE, temperature);
        break;
        case MASK_CJ_FAULT_THRESHOLD_LOW:
            return registerWriteByte(ADDRESS_CJLF_WRITE, temperature);
        break;
        case MASK_TC_FAULT_THRESHOLD_HIGH:{
            int8_t temperature_byte[2];
            int16_t temperature_multi_byte =temperature*4.0;
            temperature_byte[0]=((uint8_t)((temperature_multi_byte)&(0xFF00) >> 8));
            temperature_byte[1]=((uint8_t)((temperature_multi_byte)&(0x00FF)));
            return registerWriteByte(ADDRESS_LTHFTH_WRITE, temperature_byte[0]) && registerWriteByte(ADDRESS_LTHFTL_WRITE, temperature_byte[1]);}
        break;
        case MASK_TC_FAULT_THRESHOLD_LOW:{
            int8_t temperature_byte[2];
            int16_t temperature_multi_byte =temperature*4.0;
            temperature_byte[0]=((uint8_t)((temperature_multi_byte)&(0xFF00) >> 8));
            temperature_byte[1]=((uint8_t)((temperature_multi_byte)&(0x00FF)));
            return registerWriteByte(ADDRESS_LTLFTH_WRITE, temperature_byte[0]) && registerWriteByte(ADDRESS_LTLFTL_WRITE, temperature_byte[1]);}
        break;
        default:
            return false;
            //LOG("Please select correct threshold register to program with the correct value!\r\n");
        break;
    }
}

//******************************************************************************
bool MAX31856::coldJunctionOffset(float temperature)
{
    if (temperature > 7.9375 || temperature < -8.0)
    {
        //LOG("Input value to offest the cold junction point is non valid. enter in value in range -8 to +7.9375\r\n");
        return false;
    }
    int8_t temp_val=temperature*16.0f; //normalize the value to get rid of decimal and shorten it to size of register
    return registerWriteByte(ADDRESS_CJTO_WRITE, temp_val); //write the byte to cold junction offset register
}


//The following functions are for internal library use only
//******************************************************************************
void MAX31856::spiEnable() 
{
    ncs=0; //Set CS low to start transmission (interrupts conversion)
    return;
}


//******************************************************************************
void MAX31856::spiDisable() 
{
    ncs=1; //Set CS high to stop transmission (restarts conversion)
    return;
}


//******************************************************************************
bool MAX31856::registerReadWriteByte(uint8_t read_address, uint8_t write_address, int clear_bits, uint8_t val) 
{   
    //Read the current contents of a register
    uint8_t buf_read = registerReadByte(read_address);
    
    //Modify contents pulled from the register 
    buf_read &= clear_bits; //Clear the contents of bits of parameter you are trying to clear for later or equal operation
    buf_read |= val;        //Bitwise OR the input parameter with cleaned buf_read[1] to create new byte
    val = buf_read;
    
    //Write the updated byte to the register 
    registerWriteByte(write_address, val);

    //Read the current contents of a register
    buf_read = registerReadByte(read_address);

    return buf_read == val;
}


//******************************************************************************
bool MAX31856::registerWriteByte(uint8_t write_address, uint8_t val) 
{   
    //Write the updated byte to the register 
    spiEnable();
    spi.write(write_address);
    spi.write(val);
    spiDisable();
    return true;
}

//******************************************************************************
uint8_t MAX31856::registerReadByte(uint8_t read_address) 
{
    spiEnable();
    spi.write(read_address);
    uint8_t buf_read = spi.write(0);
    spiDisable();
    return buf_read;
}

//******************************************************************************
void MAX31856::calculateDelayTime() {
    uint32_t temp_int;
    
    if (conversion_mode==0 || thermocouple_conversion_count==0) {
        if (filter_mode==0)  //60Hz
            temp_int=82+(samples-1)*33.33f;
        else                 //50Hz
            temp_int=98+(samples-1)*40.00f;
    }
    else  { 
        if (filter_mode==0)  //60Hz
            temp_int=82+(samples-1)*16.67f;
        else                //50Hz
            temp_int=98+(samples-1)*20.00f;
    }
    
    if (cold_junction_enabled==0) //cold junction is disabled enabling 25 millisecond faster conversion times
        temp_int=temp_int-25;
    conversion_time=1000*temp_int; //set private member conversion time to calculated minimum wait time in microseconds
    return;
}

//*****************************************************************************
MAX31856::~MAX31856(void) 
{
  //empty block
}