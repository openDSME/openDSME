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

#ifndef DSMEBITVECTOR_H_
#define DSMEBITVECTOR_H_

/* INCLDUDES *****************************************************************/

#include <stdint.h>

#include "BitVectorIterator.h"
#include "Serializer.h"

/* DEFINES & MACROS **********************************************************/

#define BITVECTOR_BYTE_LENGTH(len) (((len - 1) / 8) + 1)

/* CLASSES *******************************************************************/

namespace dsme {

class BitVectorBase {
    friend class BitVectorIterator;

public:
    typedef BitVectorIterator iterator;

    explicit BitVectorBase(uint8_t* byte_array);

    void initialize(bit_vector_size_t bitSize, bool initial_fill = false);

    iterator beginSetBits();
    const iterator endSetBits() const;
    iterator beginUnsetBits();
    const iterator endUnsetBits() const;

    void fill(bool value);

    void set(bit_vector_size_t position, bool value);

    bool get(bit_vector_size_t position) const;

    bit_vector_size_t length() const;

    void setLength(bit_vector_size_t length);

    bit_vector_size_t count(bool value) const;

    void copyFrom(const BitVectorBase& other, bit_vector_size_t theirOffset = 0);

    void setOperationJoin(const BitVectorBase& other, bit_vector_size_t myOffset = 0);

    void setOperationComplement(const BitVectorBase& other, bit_vector_size_t myOffset = 0);

    bool isZero() const;

    bool operator==(const BitVectorBase& other) const;
    bool operator!=(const BitVectorBase& other) const;

    virtual uint8_t getSerializationLength() const;

protected:
    bit_vector_size_t bitSize;
    uint8_t* const byte_array;

    iterator endSetIterator;
    iterator endUnsetIterator;

    BitVectorBase(uint8_t* byte_array, const BitVectorBase& other);

    friend Serializer& operator<<(Serializer& serializer, const BitVectorBase& bv);
};

template <bit_vector_size_t MAX_SIZE>
class BitVector : public BitVectorBase {
public:
    BitVector() : BitVectorBase(array) {
    }

    BitVector(const BitVector& other) : BitVectorBase(array, other) {
    }

    BitVector& operator=(const BitVector& other) {
        this->setLength(other.length(), false);
        this->copyFrom(other, 0);
        return *this;
    }

    bool setLength(bit_vector_size_t bitSize, bool initial_fill = false) {
        if(bitSize > MAX_SIZE) {
            // WARNING this is safety and security relevant,
            // because setLength might be called with incoming message content
            return false;
        }

        initialize(bitSize, initial_fill);

        return true;
    }

    bool operator==(const BitVector& other) const {
        return BitVectorBase::operator==(other);
    }

private:
    uint8_t array[BITVECTOR_BYTE_LENGTH(MAX_SIZE)];
};

} /* namespace dsme */

#endif /* DSMEBITVECTOR_H_ */
