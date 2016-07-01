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

namespace dsme {

void IEEE802154eMACHeader::serialize(Serializer& serializer) {
    // TODO deprecated
    if(serializer.getType() == serialization_type_t::SERIALIZATION) {
        uint8_t*& data = serializer.getDataRef();
        data << *this;
    }
    else {
        const uint8_t* data = serializer.getDataRef();
        data >> *this;
        serializer.getDataRef() += this->getSerializationLength();
    }
    return;
}

Serializer& operator<<(Serializer& serializer, IEEE802154eMACHeader::FrameControl& fc) {
    uint16_t fcs;

    if(serializer.getType() == SERIALIZATION) {
        fcs = fc.frameType           << 0;
        fcs |= fc.securityEnabled    << 3;
        fcs |= fc.framePending       << 4;
        fcs |= fc.ackRequest         << 5;
        fcs |= fc.panIDCompression   << 6;
        fcs |= fc.reserved           << 7;
        fcs |= fc.seqNumSuppression  << 8;
        fcs |= fc.ieListPresent      << 9;
        fcs |= fc.dstAddrMode        << 10;
        fcs |= fc.frameVersion       << 12;
        fcs |= fc.srcAddrMode        << 14;

        serializer << fcs;
    }
    else {
        serializer << fcs;

        fc.frameType         = IEEE802154eMACHeader::FrameType((fcs >> 0) & 0x7);
        fc.securityEnabled   = (fcs >> 3) & 0x1;
        fc.framePending      = (fcs >> 4) & 0x1;
        fc.ackRequest        = (fcs >> 5) & 0x1;
        fc.panIDCompression  = (fcs >> 6) & 0x1;
        fc.reserved          = (fcs >> 7) & 0x1;
        fc.seqNumSuppression = (fcs >> 8) & 0x1;
        fc.ieListPresent     = (fcs >> 9) & 0x1;
        fc.dstAddrMode       = AddrMode((fcs >> 10) & 0x3);
        fc.frameVersion      = IEEE802154eMACHeader::FrameVersion((fcs >> 12) & 0x3);
        fc.srcAddrMode       = AddrMode((fcs >> 14) & 0x3);
    }

    return serializer;
}

}
