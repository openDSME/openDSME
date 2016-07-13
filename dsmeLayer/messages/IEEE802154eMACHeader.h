/*
 * openDSME
 *
 * Implementation of the Deterministic & Synchronous Multi-channel Extension (DSME)
 * introduced in the IEEE 802.15.4e-2012 standard
 *
 * Authors: Florian Meier <florian.meier@tuhh.de>
 *          Maximilian Koestler <maximilian.koestler@tuhh.de>
 *          Sandrina Backhauss <sandrina.backhauss@tuhh.de>
 *
 * Based on
 *          DSME Implementation for the INET Framework
 *          Tobias Luebkert <tobias.luebkert@tuhh.de>
 *
 * Copyright (c) 2015, Institute of Telematics, Hamburg University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef IEEE802154EMACHEADER_H
#define IEEE802154EMACHEADER_H

class IEEE802154eMACHeader;

#include "../../mac_services/DSME_Common.h"

#include "../../mac_services/dataStructures/DSMEMessageElement.h"
#include "../../mac_services/dataStructures/IEEE802154MacAddress.h"

namespace dsme {

class IEEE802154eMACHeader : public DSMEMessageElement {
public:
    /**
     * See IEEE 802.15.4e-2012 5.2.1.1.1, Table 2
     */
    enum FrameType {
        BEACON = 0x0, DATA = 0x1, ACKNOWLEDGEMENT = 0x2, COMMAND = 0x3, LLDN = 0x4, MULTIPURPOSE = 0x5
    };

    /**
     * See IEEE 802.15.4e-2012 5.2.1.1.7, Table 3a
     */
    enum FrameVersion {
        IEEE802154_2003 = 0x0, IEEE802154_2006 = 0x1, IEEE802154_2012 = 0x2, RESERVED = 0x3
    };

    /**
     * See IEEE 802.15.4e-2012 5.2.1.1, Figure 36
     */
    struct FrameControl {
        FrameType frameType :3;
        uint8_t securityEnabled :1;
        uint8_t framePending :1;
        uint8_t ackRequest :1;
        uint8_t panIDCompression :1;
        uint8_t reserved :1;
        uint8_t seqNumSuppression :1;
        uint8_t ieListPresent :1;
        AddrMode dstAddrMode :2;
        FrameVersion frameVersion :2;
        AddrMode srcAddrMode :2;
    };

    IEEE802154eMACHeader() {
        // TODO find useful default values or make constructor private and DSMEMessage friend
        frameControl.frameType = DATA;
        frameControl.securityEnabled = 0;
        frameControl.framePending = 0;
        frameControl.ackRequest = 0; // TODO
        frameControl.panIDCompression = 0; // TODO
        frameControl.reserved = 0;
        frameControl.seqNumSuppression = 0; // TODO
        frameControl.ieListPresent = 0; // TODO
        frameControl.dstAddrMode = SHORT_ADDRESS; // TODO
        frameControl.frameVersion = IEEE802154_2012;
        frameControl.srcAddrMode = SHORT_ADDRESS; // TODO

        seqNum = 17; // TODO

        dstPAN = 0; // TODO
        srcPAN = 0; // TODO
    }

    void setSrcAddrMode(const AddrMode& srcAddrMode) {
        this->frameControl.srcAddrMode = srcAddrMode;
    }

    AddrMode getSrcAddrMode() {
        return AddrMode(this->frameControl.srcAddrMode);
    }

    void setSrcPANId(uint16_t srcPANId) {
        this->srcPAN = srcPANId;
    }

    uint16_t getSrcPANId() {
        return this->srcPAN;
    }

    void setSrcAddr(const IEEE802154MacAddress& addr) {
        srcAddr = addr;
    }

    const IEEE802154MacAddress& getSrcAddr() const {
        return srcAddr;
    }

    IEEE802154MacAddress& getSrcAddr() {
        return srcAddr;
    }

    void setDstAddrMode(const AddrMode& dstAddrMode) {
        this->frameControl.dstAddrMode = dstAddrMode;
    }

    AddrMode getDstAddrMode() {
        return AddrMode(this->frameControl.dstAddrMode);
    }

    void setDstPANId(uint16_t dstPANId) {
        this->dstPAN = dstPANId;
    }

    uint16_t getDstPANId() {
        return this->dstPAN;
    }

    void setDstAddr(const IEEE802154MacAddress& addr) {
        dstAddr = addr;
    }

    const IEEE802154MacAddress& getDestAddr() const {
        return dstAddr;
    }

    IEEE802154MacAddress& getDestAddr() {
        return dstAddr;
    }

    FrameControl& getFrameControl() {
        return frameControl;
    }

    void setAckRequest(bool ar) {
        frameControl.ackRequest = ar;
    }

    bool isAckRequested() const {
        return frameControl.ackRequest;
    }

    void setFrameType(FrameType type) {
        frameControl.frameType = type;

        if (frameControl.frameType == ACKNOWLEDGEMENT) {
            frameControl.ackRequest = 0;
            frameControl.dstAddrMode = NO_ADDRESS;
            frameControl.srcAddrMode = NO_ADDRESS;
        }
    }

    FrameType getFrameType() const {
        return frameControl.frameType;
    }

    void setSequenceNumber(uint8_t seq) {
        seqNum = seq;
    }

    uint8_t getSequenceNumber() const {
        return seqNum;
    }

    bool isEnhancedBeacon() const {
        return frameControl.frameType == BEACON && frameControl.frameVersion == IEEE802154_2012;
    }

    void setSecurityEnabled(bool enabled) {
        this->frameControl.securityEnabled = enabled;
    }

    bool isSecurityEnabled() {
        return this->frameControl.securityEnabled;
    }

    void setPanIDCompression(bool compression) {
        this->frameControl.panIDCompression = compression;
    }

    void setIEListPresent(bool present) {
        this->frameControl.ieListPresent = present;
    }

    void setSeqNumSuppression(bool suppression) {
        this->frameControl.seqNumSuppression = suppression;
    }

private:
    FrameControl frameControl;

    uint8_t seqNum;

    uint16_t dstPAN;

    IEEE802154MacAddress dstAddr;

    uint16_t srcPAN;

    IEEE802154MacAddress srcAddr;

public:
    /* HELPER METHODS FOR HEADER FORMAT -----------------------------------> */

    inline bool hasSourceAddress() const {
        return this->frameControl.srcAddrMode != NO_ADDRESS;
    }

    inline bool hasDestinationAddress() const {
        return this->frameControl.dstAddrMode != NO_ADDRESS;
    }

    inline bool isVersion2012() const {
        return this->frameControl.frameVersion == IEEE802154_2012;
    }

    inline bool hasPanIdCompression() const {
        return this->frameControl.panIDCompression;
    }

    inline bool hasSequenceNumberSuppression() const {
        return this->frameControl.seqNumSuppression;
    }

    /**
     * See IEEE 802.15.4e-2012 5.2.1.1.5a
     */
    inline bool hasSequenceNumber() const {
        return !(isVersion2012() && hasSequenceNumberSuppression());
    }

    /**
     * See IEEE 802.15.4e-2012 5.2.1.1.6, Table 3
     */
    inline uint8_t sourceAddressLength() const {
        switch (this->frameControl.srcAddrMode) {
            default:
            case NO_ADDRESS:
                return 0;
            case SIMPLE_ADDRESS:
                return 1;
            case SHORT_ADDRESS:
                return 2;
            case EXTENDED_ADDRESS:
                return 8;
        }
    }

    /**
     * See IEEE 802.15.4e-2012 5.2.1.1.5, Table 2a
     */
    inline uint8_t sourcePanIdLength() const {
        if (hasSourceAddress() && !hasPanIdCompression() && !(hasDestinationAddress() && isVersion2012())) {
            return 2;
        } else {
            return 0;
        }
    }

    /**
     * See IEEE 802.15.4e-2012 5.2.1.1.6, Table 3
     */
    inline uint8_t destinationAddressLength() const {
        switch (this->frameControl.dstAddrMode) {
            default:
            case NO_ADDRESS:
                return 0;
            case SIMPLE_ADDRESS:
                return 1;
            case SHORT_ADDRESS:
                return 2;
            case EXTENDED_ADDRESS:
                return 8;
        }
    }

    /**
     * See IEEE 802.15.4e-2012 5.2.1.1.5, Table 2a
     */
    inline uint8_t destinationPanIdLength() const {
        if ((hasDestinationAddress() && (!hasPanIdCompression() || (hasSourceAddress() && !isVersion2012())))
                || (!hasSourceAddress() && !hasDestinationAddress() && hasPanIdCompression())) {
            return 2;
        } else {
            return 0;
        }
    }

    /* <----------------------------------- HELPER METHODS FOR HEADER FORMAT */


    virtual uint8_t getSerializationLength() const {
        uint8_t size = 0;

        size += 2; // frame control

        if (this->hasSequenceNumber()) {
            size += 1; // sequence number
        }

        if (this->frameControl.frameType != ACKNOWLEDGEMENT) {
            size += this->destinationPanIdLength(); // destination PAN ID
            size += this->destinationAddressLength(); // destination address

            size += this->sourcePanIdLength(); // source PAN ID
            size += this->sourceAddressLength(); // source address

            size += 0; // security
            size += 0; // IEs
        }

        return size;
    }

    virtual void serialize(Serializer& serializer);

    friend uint8_t* operator<<(uint8_t* &buffer, const IEEE802154eMACHeader& header);
    friend const uint8_t* operator>>(const uint8_t* &buffer, IEEE802154eMACHeader& header);
};

Serializer& operator<<(Serializer& serializer, IEEE802154eMACHeader::FrameControl& fc);

/* NEW FAST SERIALISATION **************************************************************/

inline uint8_t* operator<<(uint8_t* &buffer, const IEEE802154eMACHeader::FrameControl& fc) {
    uint8_t fcs_0, fcs_1;

    fcs_0  = fc.frameType         << 0;
    fcs_0 |= fc.securityEnabled   << 3;
    fcs_0 |= fc.framePending      << 4;
    fcs_0 |= fc.ackRequest        << 5;
    fcs_0 |= fc.panIDCompression  << 6;
    fcs_0 |= fc.reserved          << 7;
    *(buffer++) = fcs_0;

    fcs_1  = fc.seqNumSuppression << 0;
    fcs_1 |= fc.ieListPresent     << 1;
    fcs_1 |= fc.dstAddrMode       << 2;
    fcs_1 |= fc.frameVersion      << 4;
    fcs_1 |= fc.srcAddrMode       << 6;
    *(buffer++) = fcs_1;

    return buffer;
}

inline const uint8_t* operator>>(const uint8_t* &buffer, IEEE802154eMACHeader::FrameControl& fc) {
    uint8_t fcs_0, fcs_1;

    fcs_0 = *(buffer++);
    fc.frameType         = IEEE802154eMACHeader::FrameType((fcs_0 >> 0) & 0x7);
    fc.securityEnabled   = (fcs_0 >> 3) & 0x1;
    fc.framePending      = (fcs_0 >> 4) & 0x1;
    fc.ackRequest        = (fcs_0 >> 5) & 0x1;
    fc.panIDCompression  = (fcs_0 >> 6) & 0x1;
    fc.reserved          = (fcs_0 >> 7) & 0x1;

    fcs_1 = *(buffer++);
    fc.seqNumSuppression = (fcs_1 >> 0) & 0x1;
    fc.ieListPresent     = (fcs_1 >> 1) & 0x1;
    fc.dstAddrMode       = AddrMode((fcs_1 >> 2) & 0x3);
    fc.frameVersion      = IEEE802154eMACHeader::FrameVersion((fcs_1 >> 4) & 0x3);
    fc.srcAddrMode       = AddrMode((fcs_1 >> 6) & 0x3);
    return buffer;
}

inline uint8_t* operator<<(uint8_t* &buffer, const IEEE802154eMACHeader& header) {
    /* serialize frame control */
    buffer << header.frameControl;

    /* serialize sequence number */
    if (header.hasSequenceNumber()) {
        *(buffer++) = header.seqNum;
    }

    /* serialize destination address */
    if (header.destinationPanIdLength() != 0) {
        *(buffer++) = header.dstPAN & 0xFF;
        *(buffer++) = header.dstPAN >> 8;
    }

    if (header.destinationAddressLength() == 2) {
        uint16_t shortDstAddr = header.dstAddr.getShortAddress();
        *(buffer++) = shortDstAddr & 0xFF;
        *(buffer++) = shortDstAddr >> 8;
    } else if (header.destinationAddressLength() == 8) {
        buffer << header.dstAddr;
    }

    /* serialize source address */
    if (header.sourcePanIdLength() != 0) {
        *(buffer++) = header.srcPAN & 0xFF;
        *(buffer++) = header.srcPAN >> 8;
    }

    if (header.sourceAddressLength() == 2) {
        uint16_t shortSrcAddr = header.srcAddr.getShortAddress();
        *(buffer++) = shortSrcAddr & 0xFF;
        *(buffer++) = shortSrcAddr >> 8;
    } else if (header.sourceAddressLength() == 8) {
        buffer << header.srcAddr;
    }

    return buffer;
}

inline const uint8_t* operator>>(const uint8_t* &buffer, IEEE802154eMACHeader& header) {
    /* deserialize frame control */
    buffer >> header.frameControl;

    /* deserialize sequence number */
    if (header.hasSequenceNumber()) {
        header.seqNum = *(buffer++);
    }

    /* deserialize destination address */
    if (header.destinationPanIdLength() != 0) {
        header.dstPAN = *(buffer) | (*(buffer + 1) << 8);
        buffer += 2;
    }
    if (header.destinationAddressLength() == 2) {
        uint16_t shortDstAddr = *(buffer) | (*(buffer + 1) << 8);
        buffer += 2;
        header.dstAddr.setShortAddress(shortDstAddr);
    } else if (header.destinationAddressLength() == 8) {
        buffer >> header.dstAddr;
    }

    /* deserialize source address */
    if (header.sourcePanIdLength() != 0) {
        header.srcPAN = *(buffer) | (*(buffer + 1) << 8);
        buffer += 2;
    }
    if (header.sourceAddressLength() == 2) {
        uint16_t shortSrcAddr = *(buffer) | (*(buffer + 1) << 8);
        buffer += 2;
        header.srcAddr.setShortAddress(shortSrcAddr);
    } else if (header.sourceAddressLength() == 8) {
        buffer >> header.srcAddr;
    }

    return buffer;
}

}
#endif