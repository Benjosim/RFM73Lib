/*
 * Library for the RFM73 Low Power High Performance 2.4 GHz GFSK Transceiver Chip
 * Datasheet: https://cdn.soselectronic.com/productdata/d2/4d/4a7068e5/rfm73-s.pdf
 */

#include "RFM73Lib.h"
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#include <util/delay.h>
#include <inttypes.h>
#else
#include "WProgram.h"
#endif

void (*RFM73Lib::user_onReceive)(void);


///////////////////////////////////////////
//Arrays for Register initialization
///////////////////////////////////////////

//Addresses for RXDataPipe 0,1 and TX
const uint8_t RXAddrP0[] = { (0x20|0x0A), 0x34,0x43,0x10,0x10,0x01};
const uint8_t RXAddrP1[] = {(0x20 | 0x0B), 0x35, 0x43, 0x10, 0x10, 0x02};
const uint8_t TXAddr[]  = {(0x20 | 0x10), 0x34, 0x43, 0x10, 0x10, 0x01};

//RegisterBank0 initialization; Writes MSB first
const uint8_t initRegBank0[][2] = {
        // address data
        { (0x20|0x00), 0x0F },
        { (0x20|0x01), 0x3F },
        { (0x20|0x02), 0x3F },
        { (0x20|0x03), 0x03 },
        { (0x20|0x04), 0x08 },
        { (0x20|0x05), 0x17 },
        { (0x20|0x06), 0x0F },
        { (0x20|0x07), 0x07 },
        { (0x20|0x08), 0x00 },
        { (0x20|0x09), 0x00 },
        { (0x20|0x0C), 0xc3 },
        { (0x20|0x0D), 0xc4 },
        { (0x20|0x0E), 0xc5 },
        { (0x20|0x0F), 0xc6 },
        { (0x20|0x11), 0x20 },
        { (0x20|0x12), 0x20 },
        { (0x20|0x13), 0x20 },
        { (0x20|0x14), 0x20 },
        { (0x20|0x15), 0x20 },
        { (0x20|0x16), 0x20 },
        { (0x20|0x17), 0x20 },
        { (0x20|0x1C), 0x3F },
        { (0x20|0x1D), 0x07 }
};

//RegisterBank1 initialization; Writes LSB first
const uint8_t initRegBank1[][5] = {
        // address data
        { (0x20|0x00), 0x40, 0x4B, 0x01, 0xE2 },
        { (0x20|0x01), 0xC0, 0x4B, 0x00, 0x00 },
        { (0x20|0x02), 0xD0, 0xFC, 0x8C, 0x02 },
        { (0x20|0x03), 0x99, 0x00, 0x39, 0x41 },
        { (0x20|0x04), 0xD9, 0x96, 0x82, 0x1B },
        { (0x20|0x05), 0x3C, 0x02, 0x7F, 0xA6 },
        { (0x20|0x06), 0x00, 0x00, 0x00, 0x00 },
        { (0x20|0x07), 0x00, 0x00, 0x00, 0x00 },
        { (0x20|0x08), 0x00, 0x00, 0x00, 0x00 },
        { (0x20|0x09), 0x00, 0x00, 0x00, 0x00 },
        { (0x20|0x0a), 0x00, 0x00, 0x00, 0x00 },
        { (0x20|0x0b), 0x00, 0x00, 0x00, 0x00 },
        { (0x20|0x0C), 0x00, 0x12, 0x73, 0x00 },
        { (0x20|0x0D), 0x46, 0xb4, 0x80, 0x00 }
};

//RegisterBank1 initialization; Here MSB first; See Datasheet
const uint8_t initRegBank1Ramp[] = {(0x20|0x0E), 0x41,0x20,0x08,0x04,0x81,0x20,0xCF,0xF7,0xFE,0xFF,0xFF};

//Commands used for initialization
const uint8_t switchRegBank[] = {0x50, 0x53 }; // switch Register Bank
const uint8_t flushRXFIFO[] = { 0xe2, 0x00 }; // flush RX FIFO
const uint8_t flushTXFIFO[] = { 0xe1, 0x00 }; // flush TX FIFO
const uint8_t activateAdvFeatures[] = { 0x50, 0x73 }; // Activation command
const uint8_t setCarrierMode1[]={ (0x20|0x04), 0xd9 | 0x06, 0x9e, 0x86, 0x0b };
const uint8_t setCarrierMode0[]={(0x20 | 0x04), 0xd9 & ~0x06, 0x9e, 0x86, 0x0b};

///////////////////////////////////////////
//SPI initialization
///////////////////////////////////////////

/*
 * Initializes the Arduino 2560MEGA SPI Interface
*/
void RFM73Lib::initSPI(uint8_t cs, uint8_t clk_div) {
    // setup SPI Port directions
    setPinMode(SCK, OUTPUT);
    setPinMode(MOSI, OUTPUT);
    setPinMode(MISO, INPUT);
    setPinMode(cs, OUTPUT);
    // setup SPI port states
    setPinState(MOSI, LOW);
    setPinState(SCK, LOW);
    setPinState(cs, HIGH);
    // init SPI
    SPCR = (1<<SPE)|(1<<MSTR);
    // det clock divider
    spiSetClockDivider(clk_div);
    // remember cs-pin
    CS = cs;
}


///////////////////////////////////////////
//Hardware initialization
///////////////////////////////////////////

/*
 * Initializes Pins on the Arduino 2560MEGA used for controlling the Chip
*/
void RFM73Lib::initPins(uint8_t ce, uint8_t irq){
    // setup RFM Port directions
    CE = ce;
    setPinMode(ce, OUTPUT);
    setPinState(ce, LOW);
}

///////////////////////////////////////////
// Chip initialization
///////////////////////////////////////////

/*
 * Initializes all Registers in the Chip, by writing default Parameters into all Registers;
 * See initRegBank1 and initRegBank0
*/
void RFM73Lib::initRegisters() {
    // init bank 0 registers
    selectBank(0);
    // !! The last two regs in the bank0Init list will be handled later
    for (int i = 0; i < 20; i++)
        writeReg(pgm_read_byte(&initRegBank0[i][0]), pgm_read_byte(&initRegBank0[i][1]));
    // init address registers in bank 0
    writeRegBuffer((uint8_t *) RXAddrP0, sizeof(RXAddrP0));
    writeRegBuffer((uint8_t *) RXAddrP1, sizeof(RXAddrP1));
    writeRegBuffer((uint8_t *) TXAddr, sizeof(TXAddr));
    // activate Feature register
    if(!readReg(REGFeature))
        writeRegBuffer((uint8_t *) activateAdvFeatures, sizeof(activateAdvFeatures));
    // now set Registers 1D and 1C
    writeReg(pgm_read_byte(&initRegBank0[22][0]), pgm_read_byte(&initRegBank0[22][1]));
    writeReg(pgm_read_byte(&initRegBank0[21][0]), pgm_read_byte(&initRegBank0[21][1]));
    // init bank 1 registers
    selectBank(1);
    for (int i=0; i < 14; i++)
        writeRegBuffer((uint8_t *) initRegBank1[i], sizeof(initRegBank1[i]));
    // set ramp curve
    writeRegBuffer((uint8_t *) initRegBank1Ramp, sizeof(initRegBank1Ramp));
    // do we have to toggle some bits here like in the example code?
    writeRegBuffer((uint8_t *) setCarrierMode1, sizeof(setCarrierMode1));
    writeRegBuffer((uint8_t *) setCarrierMode0, sizeof(setCarrierMode0));
    delayMs(EndDelayms);
    selectBank(0);
    setModeRX();
}

/*
 *Combines initRegisters, initPins and initSPI
*/
void RFM73Lib::initChip(uint8_t ce, uint8_t irq, uint8_t cs, uint8_t clk_div) {
    initPins(ce, irq);
    initSPI(cs, clk_div);
    delayMs(InitDelayms);
    initRegisters();
}

///////////////////////////////////////////
//Control and Config Functions
///////////////////////////////////////////

/*
 * Selects given Register Bank of the RFM73 for Reading and Writing commands
*/
void RFM73Lib::selectBank(uint8_t bank) {
    uint8_t tmp = readReg(0x07) & 0x80;
    if(bank) {
        if(!tmp)
            writeRegBuffer((uint8_t *) switchRegBank, sizeof(switchRegBank));
    }
    else {
        if(tmp)
            writeRegBuffer((uint8_t *) switchRegBank, sizeof(switchRegBank));
    }
}

/*
 * Sets the RFM73 in:
 * mode>0 ==> PrimaryRX
 * mode=0 ==> PrimaryTX
 * Combines functions setModeRX and setModeTX
*/
void RFM73Lib::setMode(uint8_t mode) {
    if (mode)
        setModeRX();
    else
        setModeTX();
}

/*
 * Sets the RFM73 to Primary RX Mode
*/
void RFM73Lib::setModeRX(void) {
    uint8_t val;
    writeRegBuffer((uint8_t *) flushRXFIFO, sizeof(flushRXFIFO)); // Flush RX FIFO
    val = readReg(REGStatus); // Read Status
    writeReg(CMDWriteReg | REGStatus, val); // Reset IRQ bits
    setPinState(CE, LOW); // RFM chip disable
    // set PRIM_RX bit to 1
    val= readReg(REGConfig);
    val |= BitPrimaryRX;
    writeReg(CMDWriteReg | REGConfig, val);
    setPinState(CE, HIGH); // RFM chip enable
}

/*
 * Sets the RFM73 to Primary TX Mode
*/
void RFM73Lib::setModeTX(void) {
    uint8_t val;
    writeRegBuffer((uint8_t *) flushTXFIFO, sizeof(flushTXFIFO)); // Flush TX FIFO
    setPinState(CE, LOW); // RFM chip disable
    // set PRIM_RX bit to 0
    val= readReg(REGConfig);
    val &= ~BitPrimaryRX;
    writeReg(CMDWriteReg | REGConfig, val);
    setPinState(CE, HIGH); // RFM chip enable
}

/*
 * Configures given RXPipe
 * See Datasheet for all options
 * Returns 1 if operation was successful
*/
uint8_t RFM73Lib::configRxPipe(uint8_t pipe_nr, uint8_t * adr, uint8_t plLen, uint8_t en_aa) {

    uint8_t tmp;
    uint8_t nr = pipe_nr -1;

    if(plLen > 32 || nr > 5 || en_aa > 1)
        return 0;

    // write address
    if(nr<2)	// full length for rx pipe 0 an 1
        writeRegBufferWithCMD(CMDWriteReg | (REGRXAddressP0 + nr), adr, sizeof(adr));
    else // only LSB for pipes 2..5
        writeReg(CMDWriteReg | (REGRXAddressP0 + nr), adr[0]); //ODO:check this

    // static
    if (plLen) {
        // set payload len
        writeReg(CMDWriteReg | (REGRXPayloadLengthP0 + nr), plLen);
        // set EnableAutoAck bit
        tmp = readReg(REGEnableAutoAck);
        if (en_aa)
            tmp |= 1 << nr;
        else
            tmp &= ~(1 << nr);
        writeReg(CMDWriteReg | REGEnableAutoAck, tmp);
        // clear DPL bit
        tmp = readReg(REGDynamicPayload);
        tmp &= ~(1 << nr);
        writeReg(CMDWriteReg | REGDynamicPayload, tmp);
        // set Enable pipe bit
        enableRxPipe(nr);
    }
        // dynamic
    else {
        // set payload len to default
        writeReg(CMDWriteReg | (REGRXPayloadLengthP0 + nr), 0x20);
        // set EnableAutoAck bit
        tmp = readReg(REGEnableAutoAck);
        tmp |= 1 << nr;
        writeReg(CMDWriteReg | REGEnableAutoAck, tmp);
        // set DPL bit
        tmp = readReg(REGDynamicPayload);
        tmp |= 1 << nr;
        writeReg(CMDWriteReg | REGDynamicPayload, tmp);
        // set Enable pipe bit
        enableRxPipe(nr);
    }
    return 1;
}

/*
 * Enables given RXPipe
 * If Multiple Pipes are enabled, communication with each Pipe is possible by setting the TX address
 * to the same address as the one of the RXPipe
*/
void RFM73Lib::enableRxPipe(uint8_t pipe_nr) {
    uint8_t nr = pipe_nr - 1;
    if (nr > 5) return;
    uint8_t tmp;
    // set Enable pipe bit
    tmp = readReg(REGEnableRXAddresses);
    tmp |= 1 << nr;
    writeReg(CMDWriteReg | REGEnableRXAddresses, tmp);
}

/*
 * Disables given RXPipe
*/
void RFM73Lib::disableRxPipe(uint8_t pipe_nr) {
    uint8_t nr = pipe_nr - 1;
    if (nr > 5) return;
    uint8_t tmp;
    // set Enable pipe bit
    tmp = readReg(REGEnableRXAddresses);
    tmp &= ~(1 << nr);
    writeReg(CMDWriteReg | REGEnableRXAddresses, tmp);

}

/*
 * Configures TXPipe
 * Use Array of unsigned char to write adr!
 * adr see Datasheet for all options
 * pltype refers to the type of paload to be sent (dynamic or static length)
 * !!!In this function the address of DataPipe0 is also set to the given address for ease of use!!!
*/
void RFM73Lib::configTxPipe(uint8_t * adr, uint8_t pltype) {
    // write TX address
    writeRegBufferWithCMD(CMDWriteReg | REGTXAddress, adr, sizeof(adr));
    // write RX0 address
    writeRegBufferWithCMD(CMDWriteReg | REGRXAddressP0, adr, sizeof(adr));
    // set static or dynamic payload
    uint8_t tmp;
    tmp = readReg(REGDynamicPayload);
    if(pltype == DynamicPayloadLengthTX) // dynamic
        tmp |= 1;
    else
        tmp &= ~(1 << 0);
    writeReg(CMDWriteReg | REGDynamicPayload, tmp);
}

/*
 * Configures the gain of the received Signal
 * gain either 1 or 0
*/
void RFM73Lib::configLNAGain(uint8_t gain) {
    if (gain > 1) return;
    uint8_t tmp = readReg(REGRFSetUp);
    tmp &= 0xFE;
    tmp |= gain;
    writeReg(CMDWriteReg | REGRFSetUp, tmp);
}

/*
 * Configures the power of the sent Signal
 * pwr between 1...3
 * See datasheet for values
*/
void RFM73Lib::configRFPower(uint8_t pwr) {
    if (pwr > 3) return;
    uint8_t tmp = readReg(REGRFSetUp);
    tmp &= 0xF9;
    tmp |= pwr << 1;
    writeReg(CMDWriteReg | REGRFSetUp, tmp);
}

/*
 * Configures the length of used Addresses
 * width between 3 and 5
*/
void RFM73Lib::configAddressWidth(uint8_t width) {
    if (width < 3 || width > 5) return;
    uint8_t tmp = readReg(REGSetUpAddressWidth);
    tmp &= ( 0xF0 | (width - 2 ));
    writeReg(CMDWriteReg | REGSetUpAddressWidth, tmp);
}

/*
 * Powers up the Chip
 * 1 = Power up
 * 0 = Power down
*/
void RFM73Lib::setPower(uint8_t pwr) {
    if (pwr > 1) return;
//  setPinState(CE, LOW); // RFM chip disable
    uint8_t tmp = readReg(REGConfig);
    tmp &= (0xFD | (pwr << 1));
    writeReg(CMDWriteReg | REGConfig, tmp);
//  setPinState(CE, HIGH); // RFM chip enable
}

/*
 * Clears all flags in the Status register
*/
void RFM73Lib::clearAll() {
    uint8_t tmp = readReg(REGStatus) | 0x70;
    writeReg(CMDWriteReg | REGStatus, tmp);
}

/*
 * Clears the Data received Flag in the Status register
*/
void RFM73Lib::clearRXDataReceived() {
    uint8_t tmp = readReg(REGStatus) | 0x40 & 0xCF;
    writeReg(CMDWriteReg | REGStatus, tmp);
}

/*
 * Clears the Data sent Flag in the Status register
*/
void RFM73Lib::clearTXDataSent() {
    uint8_t tmp = readReg(REGStatus) | 0x20 & 0xAF;
    writeReg(CMDWriteReg | REGStatus, tmp);
}

/*
 * Clears the TX Timeout Flag in the Status register
*/
void RFM73Lib::clearTimeout() {
    uint8_t tmp = readReg(REGStatus) | 0x10 & 0x9F;
    writeReg(CMDWriteReg | REGStatus, tmp);
}

/*
 * Deletes all current Data in the TX FIFO
*/
void RFM73Lib::flushTxFIFO() {
    writeRegBuffer((uint8_t *) flushTXFIFO, sizeof(flushTXFIFO)); // Flush TX FIFO
}

/*
 * Deletes all current Data in the RX FIFO
*/
void RFM73Lib::flushRxFIFO() {
    writeRegBuffer((uint8_t *) flushRXFIFO, sizeof(flushRXFIFO)); // Flush RX FIFO
}

///////////////////////////////////////////
//Getter Functions
///////////////////////////////////////////

/*
 * Gets the current Mode of the Chip
 * Returns:
 * 0 ==> Primary TX
 * 0> ==> Primary RX
*/
uint8_t RFM73Lib::getMode(void) {
    return readReg(REGConfig) & BitPrimaryRX;
}

/*
 * Sets the current Channel in which Data is transmitted
 * cnum between:
 * 0000000 ... 1111111
*/
void RFM73Lib::setChannel(uint8_t cnum){writeReg(CMDWriteReg | REGRFChannel, cnum);}

/*
 * Returns the current Channel in which Data is transmitted
 * returns between:
 * 0000000 ... 1111111
*/
uint8_t RFM73Lib::getChannel(void) {return readReg(REGRFChannel);}

/*
 * Returns the current Status of the Data received Flag
 * returns:
 * 1 if new data in any DataPipe else 0
*/
uint8_t RFM73Lib::getRXDataReceived() {
    uint8_t status = readReg(REGStatus);
    if(status & IRQRXDataReceivedStatus) {
        return ((status & 0x0E) >> 1) + 1;
    }
}

/*
 * Returns the current Status of the Data sent Flag
 * returns:
 * 1 if data was sent else 0
*/
uint8_t RFM73Lib::getTXDataSent() {return readReg(REGStatus) & IRQTXDataSentStatus;}

/*
 * Returns the current Status of the TX Timeout Flag
 * returns:
 * 1 if Transmission timed out(ARD elapsed) else 0
*/
uint8_t RFM73Lib::getTXTimeout() {return readReg(REGStatus) & IRQTXMaxRetryStatus;}

/*
 * Returns the current Status of the TX FIFO Flag
 * returns:
 * 1 if TX FIFO is full else 0
*/
uint8_t RFM73Lib::getTXFIFOFull() {return readReg(REGFIFOStatus) & FIFOTXFullStatus;}

/*
 * Returns the current Status of the RX FIFO Flag
 * returns:
 * 1 if TX FIFO is empty else 0
*/
uint8_t RFM73Lib::getTXFIFOEmpty() {return readReg(REGFIFOStatus) & FIFOTXEmptyStatus;}

/*
 * Returns the current Status of the RX FIFO Flag
 * returns:
 * 1 if RX FIFO is Full else 0
*/
uint8_t RFM73Lib::getRXFIFOFull() {return readReg(REGFIFOStatus) & FIFORXFullStatus;}

/*
 * Returns the current Status of the RX FIFO Flag
 * returns:
 * 1 if RX FIFO is empty else 0
*/
uint8_t RFM73Lib::getRXFIFOEmpty() {return readReg(REGFIFOStatus) & FIFORXEmptyStatus;}

///////////////////////////////////////////
//Chip Communication
///////////////////////////////////////////

//Register Control

/*
 * Reads the contents of the given register
 * Returns:
 * Contents of given Register
*/
uint8_t RFM73Lib::readReg(uint8_t reg) {
    uint8_t res;
    setPinState(CS, LOW);
    delayMs(CSDelayms);
    transmitSPI(reg);
    res=transmitSPI(0);
    setPinState(CS, HIGH);
    delayMs(CSDelayms);
    return res;
}

/*
 * Reads the contents of the given register
 * Modifies the given array
 * used for reading sent data
*/
void RFM73Lib::readReg(uint8_t cmd, uint8_t * buf, uint8_t len) {
    uint8_t status, byte_ctr;
    setPinState(CS, LOW);
    delayMs(CSDelayms);
    status = transmitSPI(cmd); // Select register to write, and read status UINT8
    for(byte_ctr = 0; byte_ctr < len; byte_ctr++)
        buf[byte_ctr] = transmitSPI(0); // Perform SPI_RW to read UINT8 from rfm73
    setPinState(CS, HIGH);
    delayMs(CSDelayms);
}

/*
 * Writes given value into the given register
 * Only works with a single byte
*/
uint8_t RFM73Lib::writeReg(uint8_t cmd, uint8_t val) {
    setPinState(CS, LOW);
    delayMs(CSDelayms);
    transmitSPI(cmd);
    transmitSPI(val);
    setPinState(CS, HIGH);
    delayMs(CSDelayms);
}

/*
 * Writes given value into the given register
 * cmdbuf: An array of bytes
 * length: Length of the given array
*/
uint8_t RFM73Lib::writeRegBuffer(uint8_t * cmdbuf, uint8_t len) {
    setPinState(CS, LOW);
    delayMs(CSDelayms);
    while(len--) {
        transmitSPI(pgm_read_byte(cmdbuf++));
    }
    setPinState(CS, HIGH);
    delayMs(CSDelayms);
}

/*
 * Writes given value into the given register
 * buf: An array of bytes
 * length: Length of the given array
*/
uint8_t RFM73Lib::writeRegBufferWithCMD(uint8_t cmd, uint8_t * buf, uint8_t len) {
    setPinState(CS, LOW);
    delayMs(CSDelayms);
    transmitSPI(cmd);
    while(len--) {
        transmitSPI(*(buf++));
    }
    setPinState(CS, HIGH);
    delayMs(CSDelayms);
}

//Send Commands

/*
 * Sends given payload of given length without acknowledgement expectation
*/
uint8_t RFM73Lib::sendPayload(uint8_t * payload, uint8_t len) {sendPayload(payload, len, NoAck);}

/*
 * Sends given payload of given length with user defined acknowledgement expectation
*/
uint8_t RFM73Lib::sendPayload(uint8_t * payload, uint8_t len, uint8_t toAck) {
    // check TX_FIFO
    uint8_t status;
    status = readReg(REGFIFOStatus);
    // send payload
    setPinState(CS, LOW);
    delayMs(CSDelayms);
    if(toAck == -1)
        transmitSPI(CMDWriteTXPayloadAck);
    else if (toAck == 0)
        transmitSPI(CMDWriteTXPayloadNoAck);
    else
        transmitSPI(CMDWriteTXPayload);
    while(len--) {
        transmitSPI(*(payload++));
    }
    setPinState(CS, HIGH);
    delayMs(CSDelayms);

    return 1;
}

/*
 * Reads payload with length defined by PayloadWidth
 * Returns received length of received Payload
*/
uint8_t RFM73Lib::receivePayload(uint8_t *payload) {
    uint8_t len;
    uint8_t status;
    status = readReg(REGStatus);
    if (status & IRQRXDataReceivedStatus) {
        uint8_t fifo_sta;
        len = readReg(CMDRXPayloadWidth);
        readReg(CMDReadRXPayload, payload, len);
        fifo_sta = readReg(REGFIFOStatus);
        if (fifo_sta & FIFORXEmptyStatus) {
            status|= 0x40 & 0xCF;
            writeReg(CMDWriteReg | REGStatus, status);
        }
        return len;
    }
    else
        return 0;
}

//Foundational Functions

/*
 * Transmits given value via Ardunio SPI interface
 * Returns last value in SPI Buffer
*/
uint8_t RFM73Lib::transmitSPI(uint8_t val) {
    SPDR = val;
    while (!(SPSR & _BV(SPIF)));
    return SPDR;
}

/*
 * Sets the given Pin to in given State HIGH or LOW
*/
void RFM73Lib::setPinState(uint8_t pin, uint8_t state) {digitalWrite(pin, state);}

/*
 * Sets the given Pin to in given State INPUT or OUTPUT
*/
void RFM73Lib::setPinMode(uint8_t pin, uint8_t mode) {pinMode(pin, mode);}

/*
 * Delays given amount of ms (milliseconds)
*/
void RFM73Lib::delayMs(uint8_t ms) {if (ms)delay(ms);}

/*
 * Sets SPI clock divider
*/
void RFM73Lib::spiSetClockDivider(uint8_t rate) {
    SPCR = (SPCR & ~SPI_CLOCK_MASK) | (rate & SPI_CLOCK_MASK);
    SPSR = (SPSR & ~SPI_2XCLOCK_MASK) | ((rate >> 2) & SPI_2XCLOCK_MASK);
}

/*
 * Standard parameters for Arduino Pins and RFM73 Chip
*/
void RFM73Lib::initChip(void){
    initChip(8, 2,SS,0);
    configAddressWidth(3);
    configRFPower(3);
    configLNAGain(1);
    setMode(PrimaryRX);
}

/*
 * Checks if new data has arrived, if true calls a the user defined Function
 * defined in the onReceive function
*/
void RFM73Lib::checkNewData() {
    byte len = receivePayload(receiveBuffer);
    if(len<32) receiveBuffer[len] = '\0'; //NullTerminates the String for use
    //new packet if len > 0
    if (len!=0) {
        packetLength = (uint8_t)len;
        if(!user_onReceive){
        }
        else{
            user_onReceive();
        }
    }
    return;
}

/*
 * Executes given function when new data arrives
*/
void RFM73Lib::onReceive( void (*function)(void) ){user_onReceive = function;}

/*
 * Gets the Receive Buffer
 * Returns: Receive Buffer (32byte Array)
*/
uint8_t* RFM73Lib::getReceiveBuffer(void){return receiveBuffer;}

/*
 * Gets one byte from the Receive Buffer
 * Returns: Receive Buffer (1byte)
*/
uint8_t RFM73Lib::getReceiveByte(uint8_t byte){return receiveBuffer[byte];}

/*
 * Gets the length of the last transmitted Packet
 * Returns: length of last packet 1..32 (byte)
*/
uint8_t RFM73Lib::getPacketLength(void){return packetLength;}

/*
 * Sends given Payload with given legnth
 * Recommended instead of sendPayload
*/
void RFM73Lib::send(uint8_t* s, uint8_t len) {
    //switch to tx mode
    clearTXDataSent();
    setMode(PrimaryTX);
    if (getTXTimeout()) {
        flushTxFIFO();
        clearTimeout();
    }
    sendPayload(s, len);
    //wait for packet to be sent
    _delay_us(DelayAfterSendus);
    //switch to rx mode
    clearRXDataReceived();
    setMode(PrimaryRX);
}

RFM73Lib RFM;
