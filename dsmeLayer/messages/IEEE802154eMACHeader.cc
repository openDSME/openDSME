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
#include "dsme_platform.h"

namespace dsme {

void IEEE802154eMACHeader::serialize(Serializer& serializer) {
    /*
     * This function is too slow due to too much indirection, so ACK-timings would be missed.
     * Use the member deserializeFrom(...) instead.
     */
    DSME_ASSERT(false);

    // TODO deprecated
    if(serializer.getType() == serialization_type_t::SERIALIZATION) {
        uint8_t*& data = serializer.getDataRef();
        data << *this;
    }
    else {
        const uint8_t* data = serializer.getDataRef();
        // data >> *this; /* removed due to deprecation of the operator Ü/
        serializer.getDataRef() += this->getSerializationLength();
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
    }
    else {
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

}
