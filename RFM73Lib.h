//
// Created by Benjosim on 21.02.2024.
//

#ifndef CPEWETTERSTATION_RFM73LIB_H
#define CPEWETTERSTATION_RFM73LIB_H
#endif

#include <stdio.h>
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <avr/pgmspace.h>
#include "pins_arduino.h"

///////////////////////////////////////////
//Mask for Register writing
///////////////////////////////////////////
#define WithAck     0x01 //Mask for sendPayload command with acknowledge expectation
#define NoAck       0x00 //Mask for sendPayload command without acknowledge expectation
#define PrimaryTX     0x00 //Mask for setMode function to set transmitter
#define PrimaryRX     0x01 //Mask for setMode function to set receiver
#define EnableAutoAck 0x01 //Mask for configRxPipe function to enable pipe auto acknowledge
#define NoAutoAck    0x00 //Mask for configRxPipe function to disable pipe auto acknowledge
#define DynamicPayloadLengthTX       0x01 //Mask for configTxPipe to enable dynamic payload for PTX
#define StaticPayloadLengthTX       0x00 //Mask for configTxPipe to disable dynamic payload for PTX
#define CRCLength0         0x00 //Mask for configCRC to disable CRC
#define CRCLength1         0x01 //Mask for configCRC to set 1 byte CRC
#define CRCLength2         0x02 //Mask for configCRC to set 2 byte CRC
#define PowerOff      0x00 //Mask for setPower to off
#define PowerOn       0x01 //Mask for setPower to on

///////////////////////////////////////////
//RFM73 Constants
///////////////////////////////////////////
#define InitDelayms 3000 //Chip Start up time
#define EndDelayms 100 //Register Boot up time
#define CSDelayms 0 //Delay for SS
#define ReceiveBufferSize 32 // RFM73 receivebuffer size
#define DelayAfterSendus 255 // Delay after sending; Max time for timeout

///////////////////////////////////////////
//Commands and Registers
///////////////////////////////////////////
//Commands
//See Datasheet for more Info
#define CMDReadReg 0x00 //Read command to register
#define CMDWriteReg 0x20 //Write command to register
#define CMDReadRXPayload 0x61 //Read RX payload command
#define CMDWriteTXPayload 0xA0 //Write TX payload command
#define CMDFlushTX 0xE1 //Flush TX register command
#define CMDFlushRX 0xE2 //Flush RX register command
#define CMDReuseTXPayload 0xE3 //Reuse TX payload register command
#define CMDWriteTXPayloadNoAck 0xb0 //TX payload NOACK command
#define CMDWriteTXPayloadAck 0xa8 //Write ack command
#define CMDActivate 0x50 //Feature activation command
#define CMDRXPayloadWidth 0x60 //Received payload width command
#define CMDNoOperation 0xFF //No Operation used to read Status register
//Registers
//See Datasheet for more Info
#define REGConfig 0x00 //Config register address
#define REGEnableAutoAck 0x01 //Enable Auto Acknowledgment Register Address
#define REGEnableRXAddresses 0x02 //Enabled RX addresses Register Address
#define REGSetUpAddressWidth 0x03 //Setup address width Register Address
#define REGSetUpRetransmit 0x04 //Setup Auto. Retransmit Register Address
#define REGRFChannel 0x05 //RF channel Register Address
#define REGRFSetUp 0x06 //RF setup Register Address
#define REGStatus 0x07 //Status Register Address
#define REGTransmitObserve 0x08 //Observe TX Register Address
#define REGCarrierDetect 0x09 //Carrier Detect Register Address
#define REGRXAddressP0 0x0A //RX address pipe0 Register Address
#define REGRXAddressP1 0x0B //RX address pipe1 Register Address
#define REGRXAddressP2 0x0C //RX address pipe2 Register Address
#define REGRXAddressP3 0x0D //RX address pipe3 Register Address
#define REGRXAddressP4 0x0E //RX address pipe4 Register Address
#define REGRXAddressP5 0x0F //RX address pipe5 Register Address
#define REGTXAddress 0x10 //TX address Register Address
#define REGRXPayloadLengthP0 0x11 //RX payload width, pipe0 Register Address
#define REGRXPayloadLengthP1 0x12 //RX payload width, pipe1 Register Address
#define REGRXPayloadLengthP2 0x13 //RX payload width, pipe2 Register Address
#define REGRXPayloadLengthP3 0x14 //RX payload width, pipe3 Register Address
#define REGRXPayloadLengthP4 0x15 //RX payload width, pipe4 Register Address
#define REGRXPayloadLengthP5 0x16 //RX payload width, pipe5 Register Address
#define REGFIFOStatus 0x17 //FIFO Status Register Address
#define REGDynamicPayload 0x1c //Enable dynamic payload length Register Address
#define REGFeature 0x1d //Feature Register Address

///////////////////////////////////////////
//Masks for Status bits
///////////////////////////////////////////
//Interrupts
#define IRQRXDataReceivedStatus 0x40
#define IRQTXDataSentStatus 0x20
#define IRQTXMaxRetryStatus 0x10
//FIFO
#define FIFOReusePayloadStatus 0x40
#define FIFOTXFullStatus 0x20
#define FIFOTXEmptyStatus 0x10
#define FIFORXFullStatus 0x02
#define FIFORXEmptyStatus 0x01
//Chip
#define BitPrimaryRX 0x01
#define BitPower 0x02
#define WrongChipID 0x01
#define FIFOFull 0x02

///////////////////////////////////////////
//SPI Clockdividers
///////////////////////////////////////////
#define RFM77_SPI_CLOCK_DIV4 0x00
#define RFM77_SPI_CLOCK_DIV16 0x01
#define RFM77_SPI_CLOCK_DIV64 0x02
#define RFM77_SPI_CLOCK_DIV128 0x03
#define RFM77_SPI_CLOCK_DIV2 0x04
#define RFM77_SPI_CLOCK_DIV8 0x05
#define RFM77_SPI_CLOCK_DIV32 0x06
#define RFM77_SPI_CLOCK_DIV64 0x07
#define SPI_CLOCK_MASK 0x03 // SPR1 = bit 1, SPR0 = bit 0 on SPCR
#define SPI_2XCLOCK_MASK 0x01 // SPI2X = bit 0 on SPSR
#define RFM77_DEFAULT_SPI_CLOCK_DIV RFM77_SPI_CLOCK_DIV2

class RFM73Lib {
private:
    uint8_t CS;
    uint8_t CE;
    uint8_t receiveBuffer[ReceiveBufferSize];
    uint8_t packetLength;
    void setPinState(uint8_t pin, uint8_t state);
    void setPinMode(uint8_t pin, uint8_t mode);
    void delayMs(uint8_t ms);
    void initSPI(uint8_t cs, uint8_t clk_div);
    void initPins(uint8_t ce, uint8_t irq);
    void initRegisters(void);
    static inline uint8_t transmitSPI(uint8_t val);
    uint8_t writeRegBuffer(uint8_t * cmdbuf, uint8_t len);
    uint8_t writeRegBufferWithCMD(uint8_t cmd, uint8_t * buf, uint8_t len);
    void setModeRX(void);
    void setModeTX(void);

    static void (*user_onReceive)(void);
public:
    void send(uint8_t*, uint8_t len);
    void checkNewData(void);
    void onReceive(void (*function)(void) );
    uint8_t* getReceiveBuffer(void);
    uint8_t getReceiveByte(uint8_t byte);
    uint8_t getPacketLength(void);
    void initChip(void);
    void initChip(uint8_t ce, uint8_t irq, uint8_t cs, uint8_t clk_div);
    void setMode(uint8_t mode); //0 = PrimaryTX, 1 = PrimaryRX
    void setChannel(uint8_t cnum);
    uint8_t getChannel(void);
    uint8_t sendPayload(uint8_t * payload, uint8_t len); // No Acknowledge expected
    uint8_t sendPayload(uint8_t * payload, uint8_t len, uint8_t toAck); //0 = noAck, 1 = AckRequest
    uint8_t receivePayload(uint8_t *payload);
    void flushTxFIFO();
    void flushRxFIFO();
    static inline void spiSetClockDivider(uint8_t rate);
    uint8_t getMode(void); // 0 = PrimaryTX, 1 = PrimaryRX
    void setPower(uint8_t pwr); // PowerOn | PowerOff = 1|0
    uint8_t getRXDataReceived(); // >0 = Received on Pipe 1 to 6
    uint8_t getTXDataSent(); // 1 = sent NoAck package or Ack package received
    uint8_t getTXTimeout(); // 1 = Acknowledge Timeout
    uint8_t getTXFIFOFull(); // 1 = full 0 =  Space available
    uint8_t getTXFIFOEmpty(); // 1 = empty 0 = Space available
    uint8_t getRXFIFOFull(); // 1 = full 0 = Space available
    uint8_t getRXFIFOEmpty(); // 1 = empty 0 = Space available
    void clearAll(); //Clears all flags
    void clearRXDataReceived(); //Clears Data Receive Flag
    void clearTXDataSent(); //Clears Data Sent Flag
    void clearTimeout(); //Clears Timeout Flag
    uint8_t configRxPipe(uint8_t nr, uint8_t * adr, uint8_t plLen, uint8_t ena_aa); //nr = 1 to 6 plLen = 0 => DPL, plLen > 0 => SPL(plLen). ena_aa = EnableAutoAck | NO_AA
    void enableRxPipe(uint8_t nr); //nr = 1 to 6
    void disableRxPipe(uint8_t nr); //nr = 1 to 6
    void configTxPipe(uint8_t * adr, uint8_t pltype); //plType = StaticPayloadLengthTX || DynamicPayloadLengthTX = 0|1
    void configRFPower(uint8_t pwr); //0 = -10dBm, 1 = -5dBm, 2 = 0dBM, 3 = +5dBm
    void configLNAGain(uint8_t gain); //0 = low , 1 = high
    void configAddressWidth(uint8_t width); //width = 3 to 5
    uint8_t readReg(uint8_t cmd); //Read Register function !!!Hardware Access!!!
    void readReg(uint8_t reg, uint8_t * buf, uint8_t len); //Read Register function !!!Hardware Access!!!
    uint8_t writeReg(uint8_t cmd, uint8_t val); //Write Register function !!!Hardware Access!!!
    void selectBank(uint8_t bank); //Switches RegisterBank !!!Always set to desired RegBank before a Write Command!!!
};
extern RFM73Lib RFM;

//TODO
//Setter and getter functions for missing utilities
//Safety for existing functions
//