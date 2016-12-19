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
        IEEE802154_2003 = 0x0, IEEE802154_2006 = 0x1, IEEE802154_2015 = 0x2, RESERVED = 0x3
    };

    enum PANID {
        BROADCAST_PAN = 0xFFFF
    };

    /**
     * See IEEE 802.15.4e-2012 5.2.1.1, Figure 36
     */
    struct FrameControl {
        FrameType frameType : 3;
        uint8_t securityEnabled : 1;
        uint8_t framePending : 1;
        uint8_t ackRequest : 1;
        uint8_t panIDCompression : 1;
        uint8_t reserved : 1;
        uint8_t seqNumSuppression : 1;
        uint8_t ieListPresent : 1;
        AddrMode dstAddrMode : 2;
        FrameVersion frameVersion : 2;
        AddrMode srcAddrMode : 2;
    };

    IEEE802154eMACHeader() {
        reset();
    }

    void reset() {
        // TODO find useful default values or make constructor private and DSMEMessage friend
        frameControl.frameType = DATA;
        frameControl.securityEnabled = 0;
        frameControl.framePending = 0;
        frameControl.ackRequest = 0; // TODO
        frameControl.reserved = 0;
        frameControl.seqNumSuppression = 0; // TODO
        frameControl.ieListPresent = 0; // TODO
        frameControl.dstAddrMode = SHORT_ADDRESS; // TODO
        frameControl.frameVersion = IEEE802154_2015;
        frameControl.srcAddrMode = SHORT_ADDRESS; // TODO

        seqNum = 17; // TODO

        frameControl.panIDCompression = 0; // will be set during serialization
        panIDCompressionOverridden = false;
        dstPAN = BROADCAST_PAN;
        srcPAN = BROADCAST_PAN;

        hasSrcPAN = false;
        srcPAN = false;
        finalized = false;
    }

    void setSrcAddrMode(const AddrMode& srcAddrMode) {
        finalized = false;
        this->frameControl.srcAddrMode = srcAddrMode;
    }

    AddrMode getSrcAddrMode() const {
        return AddrMode(this->frameControl.srcAddrMode);
    }

    void setSrcPANId(uint16_t srcPANId) {
        finalized = false;
        this->srcPAN = srcPANId;
    }

    uint16_t getSrcPANId() const {
        return this->srcPAN;
    }

    void setSrcAddr(const IEEE802154MacAddress& addr) {
        finalized = false;
        srcAddr = addr;
    }

    const IEEE802154MacAddress& getSrcAddr() const {
        return srcAddr;
    }

    IEEE802154MacAddress& getSrcAddr() {
        return srcAddr;
    }

    void setDstAddrMode(const AddrMode& dstAddrMode) {
        finalized = false;
        this->frameControl.dstAddrMode = dstAddrMode;
    }

    AddrMode getDstAddrMode() const {
        return AddrMode(this->frameControl.dstAddrMode);
    }

    void setDstPANId(uint16_t dstPANId) {
        finalized = false;
        this->dstPAN = dstPANId;
    }

    uint16_t getDstPANId() const {
        return this->dstPAN;
    }

    void setDstAddr(const IEEE802154MacAddress& addr) {
        finalized = false;
        dstAddr = addr;
    }

    const IEEE802154MacAddress& getDestAddr() const {
        return dstAddr;
    }

    IEEE802154MacAddress& getDestAddr() {
        return dstAddr;
    }

    const FrameControl& getFrameControl() const {
        return frameControl;
    }

    void setAckRequest(bool ar) {
        finalized = false;
        frameControl.ackRequest = ar;
    }

    bool isAckRequested() const {
        return frameControl.ackRequest;
    }

    void setFrameType(FrameType type) {
        finalized = false;
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
        finalized = false;
        seqNum = seq;
    }

    uint8_t getSequenceNumber() const {
        return seqNum;
    }

    bool isEnhancedBeacon() const {
        return frameControl.frameType == BEACON && frameControl.frameVersion == IEEE802154_2015;
    }

    void setSecurityEnabled(bool enabled) {
        finalized = false;
        this->frameControl.securityEnabled = enabled;
    }

    bool isSecurityEnabled() const {
        return this->frameControl.securityEnabled;
    }

    /* Usually the PAN ID Compression bit is set automatically.
     * If you override it and it does not match the automatic setting,
     * the serialization will run in an ASSERT.
     */
    void overridePanIDCompression(bool compression) {
        finalized = false;
        this->panIDCompressionOverridden = true;
        this->frameControl.panIDCompression = compression;
    }

    void setIEListPresent(bool present) {
        finalized = false;
        this->frameControl.ieListPresent = present;
    }

    void setSeqNumSuppression(bool suppression) {
        finalized = false;
        this->frameControl.seqNumSuppression = suppression;
    }

    void setFrameControl(uint8_t lowByte, uint8_t highByte) {
        frameControl.frameType         = IEEE802154eMACHeader::FrameType((lowByte >> 0) & 0x7);
        frameControl.securityEnabled   = (lowByte >> 3) & 0x1;
        frameControl.framePending      = (lowByte >> 4) & 0x1;
        frameControl.ackRequest        = (lowByte >> 5) & 0x1;
        frameControl.panIDCompression  = (lowByte >> 6) & 0x1;
        frameControl.reserved          = (lowByte >> 7) & 0x1;

        frameControl.seqNumSuppression = (highByte >> 0) & 0x1;
        frameControl.ieListPresent     = (highByte >> 1) & 0x1;
        frameControl.dstAddrMode       = AddrMode((highByte >> 2) & 0x3);
        frameControl.frameVersion      = IEEE802154eMACHeader::FrameVersion((highByte >> 4) & 0x3);
        frameControl.srcAddrMode       = AddrMode((highByte >> 6) & 0x3);

        // The source PAN ID is OMITTED iff
        // - the source address is missing or
        // - the the PAN ID compression bit is set or
        if(getSrcAddrMode() == NO_ADDRESS || frameControl.panIDCompression) {
            hasSrcPAN = false;
        }
        else if(!isVersion2015()) {
            // ... for version 0 and 1 never
            hasSrcPAN = true;
        }
        else {
            // ... for version 2 both are extended addresses
            hasSrcPAN = !(getSrcAddrMode() == EXTENDED_ADDRESS && getDstAddrMode() == EXTENDED_ADDRESS);
        }

        // The destination PAN ID is available iff...
        if(!isVersion2015()) {
            // ... for version 0 and 1 the destination address exists
            hasDstPAN = (getDstAddrMode() != NO_ADDRESS);
        }
        else {
            // ... for version 2, either
            // - the destination address exists and the compression bit is unset or
            // - both addresses exist and either of the addresses is a short address or
            // - no address exists and the compression bit is set
            bool dst = getDstAddrMode() != NO_ADDRESS;
            bool src = getSrcAddrMode() != NO_ADDRESS;
            bool shortAddr = getSrcAddrMode() == SHORT_ADDRESS || getDstAddrMode() == SHORT_ADDRESS;
            bool compress = frameControl.panIDCompression;
            hasDstPAN = (dst && !compress) || (dst && src && shortAddr) || (!dst && !src && compress);
        }

        finalized = true;
    }

    uint8_t getFrameControlLowByte() {
        uint8_t fcf_0;
        fcf_0  = frameControl.frameType         << 0;
        fcf_0 |= frameControl.securityEnabled   << 3;
        fcf_0 |= frameControl.framePending      << 4;
        fcf_0 |= frameControl.ackRequest        << 5;
        fcf_0 |= frameControl.panIDCompression  << 6;
        fcf_0 |= frameControl.reserved          << 7;
        return fcf_0;
    }

    uint8_t getFrameControlHighByte() {
        uint8_t fcf_1;
        fcf_1  = frameControl.seqNumSuppression << 0;
        fcf_1 |= frameControl.ieListPresent     << 1;
        fcf_1 |= frameControl.dstAddrMode       << 2;
        fcf_1 |= frameControl.frameVersion      << 4;
        fcf_1 |= frameControl.srcAddrMode       << 6;
        return fcf_1;
    }

private:
    FrameControl frameControl;

    uint8_t seqNum;

    bool hasDstPAN;
    uint16_t dstPAN;

    IEEE802154MacAddress dstAddr;

    bool hasSrcPAN;
    uint16_t srcPAN;

    bool panIDCompressionOverridden;

    IEEE802154MacAddress srcAddr;

    bool finalized;

    void finalize();

public:
    /* HELPER METHODS FOR HEADER FORMAT -----------------------------------> */

    inline bool hasSourceAddress() const {
        return this->frameControl.srcAddrMode != NO_ADDRESS;
    }

    inline bool hasDestinationAddress() const {
        return this->frameControl.dstAddrMode != NO_ADDRESS;
    }

    inline bool isVersion2015() const {
        return this->frameControl.frameVersion == IEEE802154_2015;
    }

    inline bool hasPanIdCompression() const {
        return this->frameControl.panIDCompression;
    }

    inline bool hasSequenceNumberSuppression() const {
        return this->frameControl.seqNumSuppression;
    }

    inline uint8_t hasDestinationPANId() const {
        return hasDstPAN;
    }

    inline uint8_t hasSourcePANId() const {
        return hasSrcPAN;
    }

    /**
     * See IEEE 802.15.4e-2012 5.2.1.1.5a
     */
    inline bool hasSequenceNumber() const {
        return !(isVersion2015() && hasSequenceNumberSuppression());
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

    /* <----------------------------------- HELPER METHODS FOR HEADER FORMAT */


    virtual uint8_t getSerializationLength() {
        if(!finalized) {
            finalize();
        }

        uint8_t size = 0;

        size += 2; // frame control

        if (this->hasSequenceNumber()) {
            size += 1; // sequence number
        }

        if (this->frameControl.frameType != ACKNOWLEDGEMENT) {
            if(hasDstPAN) {
                size += 2;
            }
            size += this->destinationAddressLength(); // destination address

            if(hasSrcPAN) {
                size += 2;
            }
            size += this->sourceAddressLength(); // source address

            size += 0; // security
            size += 0; // IEs
        }

        return size;
    }

    virtual void serialize(Serializer& serializer);

    bool deserializeFrom(const uint8_t*& buffer, uint8_t payloadLength);
    void serializeTo(uint8_t*& buffer);
};

}
#endif
