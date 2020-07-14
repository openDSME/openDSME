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

/* INCLDUDES *****************************************************************/

#include "./DSMEBitVector.h"

#include "./Serializer.h"

#include "../../../dsme_platform.h"

namespace dsme {

/* CONSTRUCTORS & DESTRUCTOR *************************************************/

BitVectorBase::BitVectorBase(uint8_t* byte_array) : bitSize(0), byte_array(byte_array), endSetIterator(this, 0, true), endUnsetIterator(this, 0, false) {
}

void BitVectorBase::initialize(bit_vector_size_t bitSize, bool initial_fill) {
    this->bitSize = bitSize;
    this->endSetIterator = BitVectorIterator(this, bitSize, true);
    this->endUnsetIterator = BitVectorIterator(this, bitSize, false);
    this->fill(initial_fill);
}

BitVectorBase::BitVectorBase(uint8_t* byte_array, const BitVectorBase& other)
    : bitSize(other.bitSize), byte_array(byte_array), endSetIterator(this, other.bitSize, true), endUnsetIterator(this, other.bitSize, false) {
    this->copyFrom(other);
}

/* PUBLIC METHODS ************************************************************/

BitVectorBase::iterator BitVectorBase::beginSetBits() {
    bit_vector_size_t currentpos = 0;
    for(currentpos = 0; currentpos < this->bitSize && !this->get(currentpos); currentpos++) {
    }
    return iterator(this, currentpos, true);
}

const BitVectorBase::iterator BitVectorBase::endSetBits() const {
    return endSetIterator;
}

BitVectorBase::iterator BitVectorBase::beginUnsetBits() {
    bit_vector_size_t currentpos = 0;
    for(currentpos = 0; currentpos < this->bitSize && this->get(currentpos); currentpos++) {
    }
    return iterator(this, currentpos, false);
}

const BitVectorBase::iterator BitVectorBase::endUnsetBits() const {
    return endUnsetIterator;
}

void BitVectorBase::fill(bool value) {
    for(uint16_t i = 0; i < BITVECTOR_BYTE_LENGTH(this->bitSize); i++) {
        this->byte_array[i] = value * 0xFF;
    }
}

void BitVectorBase::set(bit_vector_size_t position, bool value) {
    if(position >= this->bitSize) {
        /* '-> ERROR */
        ASSERT(false);
        return;
    }

    if(value) {
        this->byte_array[position >> 3] |= (1 << (position & 0x07));
    } else {
        this->byte_array[position >> 3] &= ~(1 << (position & 0x07));
    }
}

bool BitVectorBase::get(bit_vector_size_t position) const {
    if(position >= this->bitSize) {
        /* '-> ERROR */
        ASSERT(false);
        return false;
    }

    return (this->byte_array[position >> 3] & (1 << (position & 0x07)));
}

bit_vector_size_t BitVectorBase::length() const {
    return this->bitSize;
}

void BitVectorBase::copyFrom(const BitVectorBase& other, bit_vector_size_t theirOffset) {
    if(theirOffset + this->bitSize > other.bitSize) {
        /* '-> ERROR */
        ASSERT(false);
        return;
    }

    if((theirOffset % 8) == 0) {
        bit_vector_size_t fullBytes = BITVECTOR_BYTE_LENGTH(this->bitSize + 1) - 1;
        bit_vector_size_t offsetByte = (theirOffset / 8);

        for(bit_vector_size_t i = 0; i < fullBytes; i++) {
            this->byte_array[i] = other.byte_array[offsetByte + i];
        }

        for(bit_vector_size_t i = (fullBytes * 8); i < this->bitSize; i++) {
            this->set(i, other.get((offsetByte * 8) + i));
        }
    } else {
        for(bit_vector_size_t i = 0; i < this->bitSize; i++) {
            this->set(i, other.get(theirOffset + i));
        }
    }
    return;
}

void BitVectorBase::setOperationJoin(const BitVectorBase& other, bit_vector_size_t myOffset) {
    if(myOffset + other.bitSize > this->bitSize && myOffset != 0) {
        /* '-> ERROR */
        ASSERT(false);
        return;
    }

    if((myOffset % 8) == 0) {
        bit_vector_size_t fullBytes = BITVECTOR_BYTE_LENGTH(other.bitSize + 1) - 1;
        bit_vector_size_t offsetByte = (myOffset / 8);

        for(bit_vector_size_t i = 0; i < fullBytes; i++) {
            this->byte_array[offsetByte + i] |= other.byte_array[i];
        }

        for(bit_vector_size_t i = (fullBytes * 8); i < other.bitSize; i++) {
            this->set((offsetByte * 8) + i, this->get((offsetByte * 8) + i) || other.get(i));
        }
    } else {
        for(bit_vector_size_t i = 0; i < other.bitSize; i++) {
            this->set(myOffset + i, this->get(myOffset + i) || other.get(i));
        }
    }
    return;
}

void BitVectorBase::setOperationComplement(const BitVectorBase& other, bit_vector_size_t myOffset) {
    if(myOffset + other.bitSize > this->bitSize) {
        /* '-> ERROR */
        ASSERT(false);
        return;
    }

    if((myOffset % 8) == 0) {
        bit_vector_size_t fullBytes = BITVECTOR_BYTE_LENGTH(other.bitSize + 1) - 1;
        bit_vector_size_t offsetByte = (myOffset / 8);

        for(bit_vector_size_t i = 0; i < fullBytes; i++) {
            this->byte_array[offsetByte + i] &= ~other.byte_array[i];
        }

        for(bit_vector_size_t i = (fullBytes * 8); i < other.bitSize; i++) {
            this->set((offsetByte * 8) + i, this->get((offsetByte * 8) + i) && !other.get(i));
        }
    } else {
        for(bit_vector_size_t i = 0; i < other.bitSize; i++) {
            this->set(myOffset + i, this->get(myOffset + i) && !other.get(i));
        }
    }
    return;
}

bool BitVectorBase::isZero() const {
    bit_vector_size_t fullBytes = BITVECTOR_BYTE_LENGTH(this->bitSize + 1) - 1;

    for(bit_vector_size_t i = 0; i < fullBytes; i++) {
        if(byte_array[i] != 0) {
            return false;
        }
    }

    for(bit_vector_size_t i = (fullBytes * 8); i < this->bitSize; i++) {
        if(this->get(i)) {
            return false;
        }
    }

    return true;
}

bit_vector_size_t BitVectorBase::count(bool value) const {
    bit_vector_size_t count = 0;
    bit_vector_size_t fullBytes = BITVECTOR_BYTE_LENGTH(this->bitSize + 1) - 1;

    for(bit_vector_size_t i = 0; i < fullBytes; i++) {
        uint8_t x = this->byte_array[i];
        if(!value) {
            x = ~x;
        }
        while(x > 0) {
            count++;
            x &= x - 1; // Wegner (1960)
        }
    }

    for(bit_vector_size_t i = (fullBytes * 8); i < this->bitSize; i++) {
        count += this->get(i) ^ (!value);
    }

    return count;
}

bool BitVectorBase::operator==(const BitVectorBase& other) const {
    if(this->bitSize != other.bitSize) {
        return false;
    }

    bit_vector_size_t fullBytes = BITVECTOR_BYTE_LENGTH(this->bitSize + 1) - 1;

    for(bit_vector_size_t i = 0; i < fullBytes; i++) {
        if(this->byte_array[i] != other.byte_array[i]) {
            return false;
        }
    }

    for(bit_vector_size_t i = (fullBytes * 8); i < this->bitSize; i++) {
        if(this->get(i) != other.get(i)) {
            return false;
        }
    }

    return true;
}

bool BitVectorBase::operator!=(const BitVectorBase& other) const {
    return !(*this == other);
}

uint8_t BitVectorBase::getSerializationLength() const {
    return BITVECTOR_BYTE_LENGTH(bitSize);
}

Serializer& operator<<(Serializer& serializer, const BitVectorBase& bv) {
    for(bit_vector_size_t i = 0; i < BITVECTOR_BYTE_LENGTH(bv.bitSize); i++) {
        serializer << bv.byte_array[i];
    }

    return serializer;
}

} /* namespace dsme */
