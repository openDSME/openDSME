/*
 * openDSME
 *
 * Implementation of the Deterministic & Synchronous Multi-channel Extension (DSME)
 * described in the IEEE 802.15.4-2015 standard
 *
 * Authors: Florian Kauer <florian.kauer@tuhh.de>
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

#include "./IEEE802154eMACHeader.h"

#include "../../../dsme_platform.h"
#include "../../helper/Integers.h"
#include "../../mac_services/DSME_Common.h"
#include "../../mac_services/dataStructures/IEEE802154MacAddress.h"
#include "../../mac_services/dataStructures/Serializer.h"

namespace dsme {

void IEEE802154eMACHeader::serialize(Serializer& serializer) {
    // TODO deprecated
    if(serializer.getType() == serialization_type_t::SERIALIZATION) {
        uint8_t*& data = serializer.getDataRef();
        this->serializeTo(data);
    } else {
        /*
         * This function is too slow due to too much indirection for deserialization for use on hardware, so ACK-timings would be missed.
         * Use the member deserializeFrom(...) instead.
         */
        const uint8_t* data = serializer.getDataRef();
        bool success = this->deserializeFrom(data, 127);
        serializer.getDataRef() += getSerializationLength();
        DSME_ASSERT(success);
    }
    return;
}

void IEEE802154eMACHeader::finalize() {
    // Check if we are able to use PAN ID Compression assuming a missing...
    // - ... Destination PAN ID is equal to 0xFFFF (Sect. 6.7.2 case c) )
    // - ... Source PAN ID is equal to the destination PAN (Sect. 6.7.2 5th paragraph from end)
    bool compressionIfEqual = false;
    bool panIDCompression;

    if(!isVersion2015()) {
        // Frame version field value is 0b00 or 0b01
        if(getDstAddrMode() != NO_ADDRESS && getSrcAddrMode() != NO_ADDRESS) {
            compressionIfEqual = true;
        } else {
            panIDCompression = true;

            if(getDstAddrMode() != NO_ADDRESS) {
                hasDstPAN = true;
            } else { // Source address has to be present
                hasSrcPAN = true;
            }
        }
    } else {
        // Frame version field value is 0b10
        if((getSrcAddrMode() == SHORT_ADDRESS && getDstAddrMode() != NO_ADDRESS) || (getDstAddrMode() == SHORT_ADDRESS && getSrcAddrMode() != NO_ADDRESS)) {
            // Footnote
            compressionIfEqual = true;
        } else if(getDstAddrMode() == NO_ADDRESS && getSrcAddrMode() != NO_ADDRESS) {
            // Row 5 of Table 7-2
            panIDCompression = 0;
            hasDstPAN = false;
            hasSrcPAN = true;
        } else {
            hasSrcPAN = false;
            hasDstPAN = (dstPAN != IEEE802154eMACHeader::BROADCAST_PAN);

            if(getSrcAddrMode() == NO_ADDRESS && getDstAddrMode() == NO_ADDRESS) {
                // First two rows of Table 7-2
                panIDCompression = hasDstPAN;
            } else {
                panIDCompression = !hasDstPAN;
            }
        }
    }

    if(compressionIfEqual) {
        hasDstPAN = true;
        if(srcPAN == dstPAN) {
            // Probably a mistake in the standard - the footnote says zero,
            // while the text about the old versions says one
            panIDCompression = 1;
            hasSrcPAN = false;
        } else {
            panIDCompression = 0;
            hasSrcPAN = true;
        }
    }

    if(panIDCompressionOverridden) {
        DSME_ASSERT(frameControl.panIDCompression == panIDCompression);
    } else {
        frameControl.panIDCompression = panIDCompression;
    }

    finalized = true;
}

void IEEE802154eMACHeader::serializeTo(uint8_t*& buffer) {
    if(!finalized) {
        finalize();
    }

    /* serialize frame control */
    *(buffer++) = getFrameControlLowByte();
    *(buffer++) = getFrameControlHighByte();

    // DSME_LOG_INFO("TX " << destinationAddressLength() << " " << sourceAddressLength() << " " << hasDstPAN << " " << hasSrcPAN << " " <<
    // frameControl.panIDCompression);

    /* serialize sequence number */
    if(hasSequenceNumber()) {
        *(buffer++) = seqNum;
    }

    /* serialize destination address */
    if(hasDstPAN) {
        *(buffer++) = dstPAN & 0xFF;
        *(buffer++) = dstPAN >> 8;
    }

    if(destinationAddressLength() == 2) {
        uint16_t shortDstAddr = dstAddr.getShortAddress();
        *(buffer++) = shortDstAddr & 0xFF;
        *(buffer++) = shortDstAddr >> 8;
    } else if(destinationAddressLength() == 8) {
        buffer << dstAddr;
    }

    /* serialize source address */
    if(hasSrcPAN) {
        *(buffer++) = srcPAN & 0xFF;
        *(buffer++) = srcPAN >> 8;
    }

    if(sourceAddressLength() == 2) {
        uint16_t shortSrcAddr = srcAddr.getShortAddress();
        *(buffer++) = shortSrcAddr & 0xFF;
        *(buffer++) = shortSrcAddr >> 8;
    } else if(sourceAddressLength() == 8) {
        buffer << srcAddr;
    }
}

bool IEEE802154eMACHeader::deserializeFrom(const uint8_t*& buffer, uint8_t payloadLength) {
    if(payloadLength < 2) {
        return false;
    }

    /* deserialize frame control */
    uint8_t fcLow = *(buffer++);
    uint8_t fcHigh = *(buffer++);
    this->setFrameControl(fcLow, fcHigh);

    // DSME_LOG_INFO("RX " << this->destinationAddressLength() << " " << this->sourceAddressLength() << " " << this->hasDestinationPANId() << " " <<
    // this->hasSourcePANId() << " " << this->frameControl.panIDCompression);

    if(payloadLength < getSerializationLength()) {
        return false;
    }

    /* deserialize sequence number */
    if(hasSequenceNumber()) {
        this->seqNum = *(buffer++);
    }

    /* deserialize destination address */
    if(this->hasDestinationPANId()) {
        this->dstPAN = *(buffer) | (*(buffer + 1) << 8);
        buffer += 2;
    } else {
        // A missing Destination PAN ID indicates 0xFFFF (Sect. 6.7.2 case c) )
        this->dstPAN = BROADCAST_PAN;
    }
    if(destinationAddressLength() == 2) {
        uint16_t shortDstAddr = *(buffer) | (*(buffer + 1) << 8);
        buffer += 2;
        this->dstAddr.setShortAddress(shortDstAddr);
    } else if(destinationAddressLength() == 8) {
        buffer >> this->dstAddr;
    } else {
        this->dstAddr = IEEE802154MacAddress::UNSPECIFIED;
    }

    /* deserialize source address */
    if(this->hasSourcePANId()) {
        this->srcPAN = *(buffer) | (*(buffer + 1) << 8);
        buffer += 2;
    } else {
        // A missing Source PAN ID is to be set to the destination PAN ID (Sect. 6.7.2 5th paragraph from end)
        this->srcPAN = this->dstPAN;
    }
    if(sourceAddressLength() == 2) {
        uint16_t shortSrcAddr = *(buffer) | (*(buffer + 1) << 8);
        buffer += 2;
        this->srcAddr.setShortAddress(shortSrcAddr);
    } else if(sourceAddressLength() == 8) {
        buffer >> this->srcAddr;
    } else {
        this->srcAddr = IEEE802154MacAddress::UNSPECIFIED;
    }

    return true;
}

} /* namespace dsme */
