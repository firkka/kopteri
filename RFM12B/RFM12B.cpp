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

#include "RFM12B.h"

RFM12B::RFM12B(PinName _SDI, PinName _SDO, PinName _SCK, PinName _NCS, PinName _NIRQ, PinName _NIRQ_LED) :
    spi(_SDI, _SDO, _SCK), NCS(_NCS), NIRQ(_NIRQ), NIRQ_in(_NIRQ), NIRQ_LED(_NIRQ_LED)
{

    useEncryption = false;

    /* SPI frequency, 8 bit word length, polarity and phase */
    spi.format(8, 0);
    spi.frequency(2000000);

    /* Set ~CS high */
    NCS = 1;

    /* Setup interrupt to happen on falling edge of NIRQ */
    NIRQ.fall(this, &RFM12B::InterruptHandler);
}

int RFM12B::writeCmd(int cmd)
{
    NCS = 0;
    int recv = spi.write(cmd >> 8);
    recv = spi.write(cmd);
    NCS = 1;
    return recv;
}

uint16_t RFM12B::crc16_update(uint16_t crc, uint8_t data)
{
    int i;

    crc ^= data;
    for (i = 0; i < 8; ++i) {
        if (crc & 1)
            crc = (crc >> 1) ^ 0xA001;
        else
            crc = (crc >> 1);
    }

    return crc;
}

uint8_t RFM12B::byte(uint8_t out)
{
    return spi.write(out);
}

uint16_t RFM12B::xfer(uint16_t cmd)
{
    NCS = 0;
    uint16_t reply = byte(cmd >> 8) << 8;
    reply |= byte(cmd);
    NCS = 1;
    return reply;
}

// Call this once with params:
// - node ID (0-31)
// - frequency band (RF12_433MHZ, RF12_868MHZ, RF12_915MHZ)
// - networkid [optional - default = 170] (0-255 for RF12B, only 212 allowed for RF12)
// - txPower [optional - default = 0 (max)] (7 is min value)
// - airKbps [optional - default = 38.31Kbps]
void RFM12B::Initialize(uint8_t nodeid, uint8_t freqBand, uint8_t groupid, uint8_t txPower, uint8_t airKbps)
{

    nodeID = nodeid;
    networkID = groupid;
    rf12_grp= groupid;

    writeCmd(0x0000);                   // initial SPI transfer added to avoid power-up problem
    writeCmd(RF_SLEEP_MODE);            // DC (disable clk pin), enable lbd

    // wait until RFM12B is out of power-up reset, this takes several *seconds*
    writeCmd(RF_TXREG_WRITE);           // in case we're still in OOK mode

    while (NIRQ == 0)
        writeCmd(0x0000);

    writeCmd(0x80C7 | (freqBand << 4)); // EL (ena TX), EF (ena RX FIFO), 12.0pF
    writeCmd(0xA640);                   // Frequency is exactly 434/868/915MHz (whatever freqBand is)
    writeCmd(0xC600 + airKbps);         //Air transmission baud rate: 0x08= ~38.31Kbps
    writeCmd(0x94A2);                   // VDI,FAST,134kHz,0dBm,-91dBm
    writeCmd(0xC2AC);                   // AL,!ml,DIG,DQD4
    if (networkID != 0) {
        writeCmd(0xCA83);               // FIFO8,2-SYNC,!ff,DR
        writeCmd(0xCE00 | networkID);   // SYNC=2DXX
    } else {
        writeCmd(0xCA8B);               // FIFO8,1-SYNC,!ff,DR
        writeCmd(0xCE2D);               // SYNC=2D
    }

    writeCmd(0xC483);                   // @PWR,NO RSTRIC,!st,!fi,OE,EN
    writeCmd(0x9850 | (txPower > 7 ? 7 : txPower)); // !mp,90kHz,MAX OUT
    writeCmd(0xCC77);                   // OB1, OB0, LPX, ddy, DDIT, BW0
    writeCmd(0xE000);                   // NOT USE
    writeCmd(0xC800);                   // NOT USE
    writeCmd(0xC049);                   // 1.66MHz,3.1V

    rxstate = TXIDLE;
}

void RFM12B::InterruptHandler()
{

    NIRQ_LED = 1;

    // a transfer of 2x 16 bits @ 2 MHz over SPI takes 2x 8 us inside this ISR
    writeCmd(0x0000);

    if (rxstate == TXRECV) {
        uint8_t in = xfer(RF_RX_FIFO_READ);

        if (rxfill == 0 && networkID != 0)
            rf12_buf[rxfill++] = networkID;

        rf12_buf[rxfill++] = in;
        rf12_crc = crc16_update(rf12_crc, in);

        if (rxfill >= rf12_len+ 6 || rxfill >= RF_MAX)
            xfer(RF_IDLE_MODE);
    } else {
        uint8_t out;

        if (rxstate < 0) {
            uint8_t pos = 4 + rf12_len + rxstate++;
            out = rf12_buf[pos];
            rf12_crc = crc16_update(rf12_crc, out);
        } else {
            switch (rxstate++) {
                case TXSYN1:
                    out = 0x2D;
                    break;
                case TXSYN2:
                    out = rf12_grp;
                    rxstate = - (3 + rf12_len);
                    break;
                case TXCRC1:
                    out = rf12_crc;
                    break;
                case TXCRC2:
                    out = rf12_crc >> 8;
                    break;
                case TXDONE:
                    xfer(RF_IDLE_MODE); // fall through
                    out = 0xAA;
                    break;
                default:
                    out = 0xAA;
            }
        }
        xfer(RF_TXREG_WRITE + out);
    }
    NIRQ_LED = 0;
}

void RFM12B::ReceiveStart(void)
{
    rxfill = rf12_len= 0;
    rf12_crc = ~0;

    if (networkID != 0)
        rf12_crc = crc16_update(~0, networkID);

    rxstate = TXRECV;
    xfer(RF_RECEIVER_ON);
}

bool RFM12B::ReceiveComplete(void)
{
    if (rxstate == TXRECV && (rxfill >= rf12_len+ 6 || rxfill >= RF_MAX)) {
        rxstate = TXIDLE;

        if (rf12_len > RF12_MAXDATA) {
            rf12_crc = 1; // force bad crc if packet length is invalid
        }
        if (RF12_DESTID == 0 || RF12_DESTID == nodeID) {

            if (rf12_crc == 0 && useEncryption)
                Encryption(false);
            else
                rf12_seq = -1;

#ifdef DEBUG
            printf("\nReceived message from [%d]; crc:%x,  len: %d, message: ", RF12_SOURCEID, rf12_crc, rf12_len);
            for (int i=0; i<rf12_len; i++) {
                printf("%c", rf12_data[i]);
            }
            printf("\n");
#endif

            return true; // it's a broadcast packet or it's addressed to this node
        }
    }
    if (rxstate == TXIDLE)
        ReceiveStart();

    return false;
}

bool RFM12B::CanSend()
{
    // no need to test with interrupts disabled: state TXRECV is only reached
    // outside of ISR and we don't care if rxfill jumps from 0 to 1 here
    if (rxstate == TXRECV && rxfill == 0 && (byte(0x00) & (RF_RSSI_BIT >> 8)) == 0) {
        xfer(RF_IDLE_MODE); // stop receiver
        rxstate = TXIDLE;
        return true;
    }
    return false;
}

void RFM12B::SendStart(uint8_t toNodeID, bool requestACK, bool sendACK)
{

    rf12_hdr1= toNodeID | (sendACK ? RF12_HDR_ACKCTLMASK : 0);
    rf12_hdr2= nodeID | (requestACK ? RF12_HDR_ACKCTLMASK : 0);

#ifdef DEBUG
    printf("SendStart to Node [%d], from Node [%d] \n", toNodeID, nodeID);
#endif

    if (useEncryption)
        Encryption(true);

    rf12_crc = ~0;
    rf12_crc = crc16_update(rf12_crc, rf12_grp);
    rxstate = TXPRE1;

    xfer(RF_XMITTER_ON); // bytes will be fed via interrupts
}

void RFM12B::SendStart(uint8_t toNodeID, const void* sendBuf, uint8_t sendLen, bool requestACK, bool sendACK)
{

    rf12_len = sendLen;
    memcpy((void*) rf12_data, sendBuf, sendLen);

#ifdef DEBUG
    printf("\nSending message from [%d]; crc:%x,  len: %d, message: ", nodeID, rf12_crc, rf12_len);
    for (int i=0; i<rf12_len; i++) {
        printf("%c", rf12_data[i]);
    }
#endif


    SendStart(toNodeID, requestACK, sendACK);
}

/// Should be called immediately after reception in case sender wants ACK
void RFM12B::SendACK(const void* sendBuf, uint8_t sendLen)
{
    while (!CanSend())
        ReceiveComplete();
    SendStart(RF12_SOURCEID, sendBuf, (sendLen > 0) ? sendLen: strlen((const char*)sendBuf), false, true);
}

void RFM12B::Send(uint8_t toNodeID, const void* sendBuf, uint8_t sendLen, bool requestACK)
{
    while (!CanSend())
        ReceiveComplete();
    SendStart(toNodeID, sendBuf, (sendLen > 0) ? sendLen: strlen((const char*)sendBuf) , requestACK, false);
}

uint8_t RFM12B::GetSender(void)
{
    return RF12_SOURCEID;
}

volatile uint8_t * RFM12B::GetData(void)
{

    return (uint8_t*) rf12_data;
}

uint8_t RFM12B::GetDataLen(void)
{
    return rf12_len;
}

bool RFM12B::ACKRequested()
{
    return RF12_WANTS_ACK;
}

/// Should be polled immediately after sending a packet with ACK request
bool RFM12B::ACKReceived(uint8_t fromNodeID)
{
    if (ReceiveComplete())
        return CRC_Pass() && RF12_DESTID == nodeID && (RF12_SOURCEID == fromNodeID || fromNodeID == 0) && (rf12_hdr1&RF12_HDR_ACKCTLMASK) &&
               !(rf12_hdr2 & RF12_HDR_ACKCTLMASK);
    return false;
}

bool RFM12B::CRC_Pass(void)
{
    return (rf12_crc == 0);
}

// XXTEA by David Wheeler, adapted from http://en.wikipedia.org/wiki/XXTEA
#define DELTA 0x9E3779B9
#define MX (((z>>5^y<<2) + (y>>3^z<<4)) ^ ((sum^y) + (cryptKey[(uint8_t)((p&3)^e)] ^ z)))
void RFM12B::Encryption(bool encrypt)
{

    uint32_t y, z, sum, *v = (uint32_t*) rf12_data;
    uint8_t p, e, rounds = 6;

    if (encrypt) {
        // pad with 1..4-byte sequence number
        *(uint32_t*) (rf12_data + rf12_len) = ++seqNum;
        uint8_t pad = 3 - (rf12_len & 3);
        rf12_len += pad;
        rf12_data[rf12_len] &= 0x3F;
        rf12_data[rf12_len] |= pad << 6;
        ++rf12_len;
        // actual encoding
        char n = rf12_len / 4;
        if (n > 1) {
            sum = 0;
            z = v[n-1];
            do {
                sum += DELTA;
                e = (sum >> 2) & 3;
                for (p=0; p<n-1; p++)
                    y = v[p+1], z = v[p] += MX;
                y = v[0];
                z = v[n-1] += MX;
            } while (--rounds);
        }
    } else if (rf12_crc == 0) {
        // actual decoding
        char n = rf12_len / 4;
        if (n > 1) {
            sum = rounds*DELTA;
            y = v[0];
            do {
                e = (sum >> 2) & 3;
                for (p=n-1; p>0; p--)
                    z = v[p-1], y = v[p] -= MX;
                z = v[n-1];
                y = v[0] -= MX;
            } while ((sum -= DELTA) != 0);
        }
        // strip sequence number from the end again
        if (n > 0) {
            uint8_t pad = rf12_data[--rf12_len] >> 6;
            rf12_seq = rf12_data[rf12_len] & 0x3F;
            while (pad-- > 0)
                rf12_seq = (rf12_seq << 8) | rf12_data[--rf12_len];
        }
    }

}

void RFM12B::SetEncryptionKey(const uint8_t* key)
{
    if (key != 0) {
        for (uint8_t i = 0; i < sizeof cryptKey; ++i)
            ((uint8_t*) cryptKey)[i] = key[i];

        useEncryption = true;
    } else
        useEncryption = false;
}

void RFM12B::Sleep(int n)
{
    if (n < 0)
        xfer(RF_IDLE_MODE);
    else {
        xfer(RF_WAKEUP_TIMER | 0x0500 | n);
        xfer(RF_SLEEP_MODE);
        if (n > 0)
            xfer(RF_WAKEUP_MODE);
    }
    rxstate = TXIDLE;
}
void RFM12B::Sleep()
{
    Sleep(0);
}
void RFM12B::Wakeup()
{
    Sleep(-1);
}

bool RFM12B::LowBattery()
{
    return (xfer(0x0000) & RF_LBD_BIT) != 0;
}