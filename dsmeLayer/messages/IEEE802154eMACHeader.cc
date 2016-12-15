/*
 * openDSME
 *
 * Implementation of the Deterministic & Synchronous Multi-channel Extension (DSME)
 * described in the IEEE 802.15.4-2015 standard
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

#include "IEEE802154eMACHeader.h"
#include "../../../dsme_platform.h"

namespace dsme {

void IEEE802154eMACHeader::serialize(Serializer& serializer) {
    // TODO deprecated
    if(serializer.getType() == serialization_type_t::SERIALIZATION) {
        uint8_t*& data = serializer.getDataRef();
        data << *this;
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

Serializer& operator<<(Serializer& serializer, IEEE802154eMACHeader::FrameControl& fc) {
    uint16_t fcf;

    if(serializer.getType() == SERIALIZATION) {
        fcf = fc.frameType           << 0;
        fcf |= fc.securityEnabled    << 3;
        fcf |= fc.framePending       << 4;
        fcf |= fc.ackRequest         << 5;
        fcf |= fc.panIDCompression   << 6;
        fcf |= fc.reserved           << 7;
        fcf |= fc.seqNumSuppression  << 8;
        fcf |= fc.ieListPresent      << 9;
        fcf |= fc.dstAddrMode        << 10;
        fcf |= fc.frameVersion       << 12;
        fcf |= fc.srcAddrMode        << 14;

        serializer << fcf;
    } else {
        serializer << fcf;

        fc.frameType         = IEEE802154eMACHeader::FrameType((fcf >> 0) & 0x7);
        fc.securityEnabled   = (fcf >> 3) & 0x1;
        fc.framePending      = (fcf >> 4) & 0x1;
        fc.ackRequest        = (fcf >> 5) & 0x1;
        fc.panIDCompression  = (fcf >> 6) & 0x1;
        fc.reserved          = (fcf >> 7) & 0x1;
        fc.seqNumSuppression = (fcf >> 8) & 0x1;
        fc.ieListPresent     = (fcf >> 9) & 0x1;
        fc.dstAddrMode       = AddrMode((fcf >> 10) & 0x3);
        fc.frameVersion      = IEEE802154eMACHeader::FrameVersion((fcf >> 12) & 0x3);
        fc.srcAddrMode       = AddrMode((fcf >> 14) & 0x3);
    }

    return serializer;
}

uint8_t* operator<<(uint8_t*& buffer, const IEEE802154eMACHeader& header) {
    // Check if we are able to use PAN ID Compression assuming a missing...
    // - ... Destination PAN ID is equal to 0xFFFF (Sect. 6.7.2 case c) )
    // - ... Source PAN ID is equal to the destination PAN (Sect. 6.7.2 5th paragraph from end) 
    bool serializeSrcPAN = false;
    bool serializeDstPAN = false;
    bool compressionIfEqual = false;
    bool panIDCompression;

    if(!header.isVersion2015()) {
        // Frame version field value is 0b00 or 0b01
        if(header.getDstAddrMode() != NO_ADDRESS && header.getSrcAddrMode() != NO_ADDRESS) {
            compressionIfEqual = true;
        }
        else {
            panIDCompression = true;

            if(header.getDstAddrMode() != NO_ADDRESS) {
                serializeDstPAN = true;
            }
            else { // Source address has to be present
                serializeSrcPAN = true;
            }
        }
    }
    else {
        // Frame version field value is 0b10
        if( (header.getSrcAddrMode() == SHORT_ADDRESS && header.getDstAddrMode() != NO_ADDRESS)
          ||(header.getDstAddrMode() == SHORT_ADDRESS && header.getSrcAddrMode() != NO_ADDRESS) ) {
            // Footnote
            compressionIfEqual = true;
        }
        else if(header.getDstAddrMode() == NO_ADDRESS && header.getSrcAddrMode() != NO_ADDRESS) {
            // Row 5 of Table 7-2
            panIDCompression = 0;
            serializeDstPAN = false;
            serializeSrcPAN = true;
        }
        else {
            serializeSrcPAN = false;
            serializeDstPAN = (header.dstPAN != IEEE802154eMACHeader::BROADCAST_PAN);

            if(header.getSrcAddrMode() == NO_ADDRESS && header.getDstAddrMode() == NO_ADDRESS) {
                // First two rows of Table 7-2
                panIDCompression = serializeDstPAN;
            }
            else {
                panIDCompression = !serializeDstPAN;
            }
        }
    }

    if(compressionIfEqual) {
        serializeDstPAN = true;
        if(header.srcPAN == header.dstPAN) {
            // Probably a mistake in the standard - the footnote says zero,
            // while the text about the old versions says one
            panIDCompression = 1;
            serializeSrcPAN = false;
        }
        else {
            panIDCompression = 0;
            serializeSrcPAN = true;
        }
    }

    if(header.panIDCompressionOverridden) {
        DSME_ASSERT(header.frameControl.panIDCompression == panIDCompression);
    }

    /* serialize frame control */
    auto frameControl = header.frameControl;
    frameControl.panIDCompression = panIDCompression?1:0;
    buffer << frameControl;

    //LOG_INFO("TX " << header.destinationAddressLength() << " " << header.sourceAddressLength() << " " << serializeDstPAN << " " << serializeSrcPAN << " " << frameControl.panIDCompression);


    /* serialize sequence number */
    if (header.hasSequenceNumber()) {
        *(buffer++) = header.seqNum;
    }

    /* serialize destination address */
    if (serializeDstPAN) {
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
    if (serializeSrcPAN) {
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

bool IEEE802154eMACHeader::deserializeFrom(const uint8_t*& buffer, uint8_t payloadLength) {
    if(payloadLength < 2) {
        return false;
    }

    /* deserialize frame control */
    buffer >> this->frameControl;

    //LOG_INFO("RX " << this->destinationAddressLength() << " " << this->sourceAddressLength() << " " << this->hasDestinationPANId() << " " << this->hasSourcePANId() << " " << this->frameControl.panIDCompression);

    if(payloadLength < getSerializationLength()) {
        return false;
    }

    /* deserialize sequence number */
    if (hasSequenceNumber()) {
        this->seqNum = *(buffer++);
    }

    /* deserialize destination address */
    if (this->hasDestinationPANId()) {
        this->dstPAN = *(buffer) | (*(buffer + 1) << 8);
        buffer += 2;
    }
    else {
        // A missing Destination PAN ID indicates 0xFFFF (Sect. 6.7.2 case c) )
        this->dstPAN = BROADCAST_PAN;
    }
    if (destinationAddressLength() == 2) {
        uint16_t shortDstAddr = *(buffer) | (*(buffer + 1) << 8);
        buffer += 2;
        this->dstAddr.setShortAddress(shortDstAddr);
    } else if (destinationAddressLength() == 8) {
        buffer >> this->dstAddr;
    } else {
        this->dstAddr = IEEE802154MacAddress::UNSPECIFIED;
    }

    /* deserialize source address */
    if (this->hasSourcePANId()) {
        this->srcPAN = *(buffer) | (*(buffer + 1) << 8);
        buffer += 2;
    }
    else {
        // A missing Source PAN ID is to be set to the destination PAN ID (Sect. 6.7.2 5th paragraph from end) 
        this->srcPAN = this->dstPAN;
    }
    if (sourceAddressLength() == 2) {
        uint16_t shortSrcAddr = *(buffer) | (*(buffer + 1) << 8);
        buffer += 2;
        this->srcAddr.setShortAddress(shortSrcAddr);
    } else if (sourceAddressLength() == 8) {
        buffer >> this->srcAddr;
    } else {
        this->srcAddr = IEEE802154MacAddress::UNSPECIFIED;
    }

    return true;
}

}
