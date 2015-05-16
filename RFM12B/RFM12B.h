/*
 RFM12B Library. Based on work done by JeeLabs.org ported to mBed by SK Pang.
 http://jeelabs.net/projects/cafe/wiki/RF12
 Jan 2012 skpang.co.uk

 RFM12B Library (Moteino Comunication Protocol). Based on work done by Felix Rusu ported to mBed by Hugo Rodrigues
 http://lowpowerlab.com/blog/2012/12/28/rfm12b-arduino-library/
 May 2013 Hugo Rodrigues

 http://opensource.org/licenses/mit-license.php

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 */

#ifndef _RFM12B_H
#define _RFM12B_H

#include "mbed.h"

/// RF12 Maximum message size in bytes.
#define RF12_MAXDATA    128
/// Max transmit/receive buffer: 4 header + data + 2 crc bytes
#define RF_MAX          (RF12_MAXDATA + 6)

#define RF12_433MHZ     1
#define RF12_868MHZ     2
#define RF12_915MHZ     3

#define RF12_HDR_IDMASK      0x7F
#define RF12_HDR_ACKCTLMASK  0x80
#define RF12_DESTID   (rf12_hdr1 & RF12_HDR_IDMASK)
#define RF12_SOURCEID (rf12_hdr2 & RF12_HDR_IDMASK)

// shorthands to simplify sending out the proper ACK when requested
#define RF12_WANTS_ACK ((rf12_hdr2 & RF12_HDR_ACKCTLMASK) && !(rf12_hdr1 & RF12_HDR_ACKCTLMASK))

/// Shorthand for RF12 group byte in rf12_buf.
#define rf12_grp        rf12_buf[0]
/// pointer to 1st header byte in rf12_buf (CTL + DESTINATIONID)
#define rf12_hdr1        rf12_buf[1]
/// pointer to 2nd header byte in rf12_buf (ACK + SOURCEID)
#define rf12_hdr2        rf12_buf[2]

/// Shorthand for RF12 length byte in rf12_buf.
#define rf12_len        rf12_buf[3]
/// Shorthand for first RF12 data byte in rf12_buf.
#define rf12_data       (rf12_buf + 4)

// RF12 command codes
#define RF_RECEIVER_ON  0x82DD
#define RF_XMITTER_ON   0x823D
#define RF_IDLE_MODE    0x820D
#define RF_SLEEP_MODE   0x8205
#define RF_WAKEUP_MODE  0x8207
#define RF_TXREG_WRITE  0xB800
#define RF_RX_FIFO_READ 0xB000
#define RF_WAKEUP_TIMER 0xE000

//RF12 status bits
#define RF_LBD_BIT      0x0400
#define RF_RSSI_BIT     0x0100

//#define DEBUG

// transceiver states, these determine what to do with each interrupt
enum {
    TXCRC1, TXCRC2, TXTAIL, TXDONE, TXIDLE, TXRECV, TXPRE1, TXPRE2, TXPRE3, TXSYN1, TXSYN2,
};

class RFM12B
{

public:
    /* Constructor */
    RFM12B(PinName SDI, PinName SDO, PinName SCK, PinName NCS, PinName NIRQ, PinName NIRQ_LED);

    /* Initialises the RFM12B module */
    void Initialize(uint8_t nodeid, uint8_t freqBand, uint8_t groupid=0xAA, uint8_t txPower=0, uint8_t airKbps=0x08);

    void ReceiveStart(void);
    bool ReceiveComplete(void);
    bool CanSend();

    void SendStart(uint8_t toNodeId, bool requestACK = false, bool sendACK = false);
    void SendStart(uint8_t toNodeId, const void* sendBuf, uint8_t sendLen, bool requestACK = false,
                   bool sendACK = false);
    void SendACK(const void* sendBuf = "", uint8_t sendLen = 0);
    void Send(uint8_t toNodeId, const void* sendBuf, uint8_t sendLen, bool requestACK = false);

    void Sleep(int n);
    void Sleep();
    void Wakeup();
    bool LowBattery();

    volatile uint8_t * GetData();
    uint8_t GetDataLen(void);                   // how many bytes were received
    uint8_t GetSender(void);

    bool ACKRequested();
    bool ACKReceived(uint8_t fromNodeID = 0);

    void Encryption(bool encrypt);              // does en-/decryption
    void SetEncryptionKey(const uint8_t* key);  // set encryption key

    bool CRC_Pass(void);

protected:
    /* SPI module */
    SPI spi;

    /* Other digital pins */
    DigitalOut NCS;
    InterruptIn NIRQ;
    DigitalIn NIRQ_in;
    DigitalOut NIRQ_LED;

    volatile uint8_t nodeID;                    // address of this node
    volatile uint8_t networkID;                 // network group

    volatile uint8_t* Data;
    volatile uint8_t* DataLen;

    void InterruptHandler();                    // interrupt routine for data reception

    int writeCmd(int cmd);                      // write a command to the RF module
    uint16_t crc16_update(uint16_t crc, uint8_t data);
    uint16_t xfer(uint16_t cmd);
    uint8_t byte(uint8_t out);

private:
    volatile uint8_t rf12_buf[RF_MAX];          // recv/xmit buf, including hdr & crc bytes

    volatile uint8_t rxfill;                    // number of data bytes in rf12_buf
    volatile int8_t rxstate;                    // current transceiver state
    volatile uint16_t rf12_crc;                 // running crc value
    uint32_t seqNum;                            // encrypted send sequence number
    uint32_t cryptKey[4];                       // encryption key to use
    long rf12_seq;                              // seq number of encrypted packet (or -1)

    bool useEncryption;
};

#endif /* _RFM12B_H */
