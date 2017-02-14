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

#ifndef DSMESABSPECIFICATION_H_
#define DSMESABSPECIFICATION_H_

#include "../../../dsme_settings.h"
#include "./DSMEBitVector.h"

namespace dsme {

class DSMESABSpecification {
public:
    typedef BitVector<MAX_GTSLOTS * MAX_CHANNELS * MAX_SAB_UNITS> SABSubBlock;

    DSMESABSpecification() : subBlockIndex(0) {
    }

    explicit DSMESABSpecification(uint8_t subBlockLengthBytes) : subBlockIndex(0) {
        subBlock.initialize(subBlockLengthBytes * 8);
    }

    explicit DSMESABSpecification(SABSubBlock& bitVector) : subBlockIndex(0), subBlock(bitVector) {
    }

    void setSubBlockLengthBytes(uint8_t bytes) {
        subBlock.setLength(bytes * 8);
    }

    uint8_t getSubBlockLengthBytes() const {
        return subBlock.length() / 8;
    }

    const SABSubBlock& getSubBlock() const {
        return subBlock;
    }

    SABSubBlock& getSubBlock() {
        return subBlock;
    }

    uint16_t getSubBlockIndex() const {
        return subBlockIndex;
    }

    void setSubBlockIndex(uint16_t subBlockIndex) {
        this->subBlockIndex = subBlockIndex;
    }

    // TODO do we really want this?
    DSMESABSpecification& operator=(const DSMESABSpecification& other) {
        this->subBlockIndex = other.subBlockIndex;
        this->subBlock = other.subBlock;
        return (*this);
    }

    bool operator==(const DSMESABSpecification& other) const {
        return (subBlockIndex == other.subBlockIndex) && (subBlock == other.subBlock);
    }

private:
    /*subBlockIndex in units not in bits! (page 115 top, fig 59r in IEEE 802.15.4e-2012)*/
    uint16_t subBlockIndex;
    SABSubBlock subBlock;

public:
    virtual uint8_t getSerializationLength() const {
        uint8_t size = 0;

        size += 1; // sub-block length
        size += 2; // sub-block index
        size += subBlock.getSerializationLength();

        return size;
    }

    friend Serializer& operator<<(Serializer& serializer, DSMESABSpecification& b);
};

inline Serializer& operator<<(Serializer& serializer, DSMESABSpecification& b) {
    if(serializer.getType() == SERIALIZATION) {
        uint8_t subBlockLength = BITVECTOR_BYTE_LENGTH(b.subBlock.length());
        serializer << subBlockLength;
    } else {
        uint8_t subBlockLength = 0; /* unused value */
        serializer << subBlockLength;
        b.subBlock.setLength(subBlockLength * 8);
    }

    serializer << b.subBlockIndex;

    serializer << b.subBlock;

    return serializer;
}

} /* namespace dsme */

#endif /* DSMESABSPECIFICATION_H_ */
