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

#ifndef DSMEGACKBITMAPFRAGMENT_H_
#define DSMEGACKBITMAPFRAGMENT_H_

/* INCLDUDES *****************************************************************/

#include "../../helper/Integers.h"
#include "./IEEE802154MacAddress.h"
#include "./RBTree.h"

class Serializer;
class DSMEGACKBitmap;


/* CLASSES *******************************************************************/

namespace dsme {

class DSMEGACKBitmapFragment {
    friend class DSMEGACKBitmap;
    friend auto operator<<(Serializer& serializer, DSMEGACKBitmap& gack) -> Serializer&;

public:
    DSMEGACKBitmapFragment(uint8_t sequenceNumber, uint8_t bits=0) : next(nullptr), sequenceNumber(sequenceNumber), bits(bits) {}
    ~DSMEGACKBitmapFragment() {}

    /** Sets the bit at the given position in the fragment to 1.
     *
     * @param pos The position of the bit
     */
    auto setBit(uint8_t pos, bool set) -> void;

    /** Retrieves the bit at the given position in the fragment.
     *
     * @param pos The position of the bit
     */
    auto getBit(uint8_t pos) const -> bool;

    /** Returns the index of the first set bit.
     *
     * @return The index of the first set bit
     */
    auto getFirstSetBitIndex() const -> uint8_t;

    /** Returns the index of the last set bit.
     *
     * @return The index of the last set bit
     */
    auto getLastSetBitIndex() const -> uint8_t;

    /** Returns the number of set bits in the fragment.
     *
     * @return The number of set bits
     */
    auto getNumSetBits() const -> uint8_t;

    /** Returns the last sequence number of the fragment
     *
     * @return The last sequence number of the fragment
     */
    auto getLastSequenceNumber() const -> uint8_t;

    auto setNext(DSMEGACKBitmapFragment* n) -> void {
        next = n;
    }

    auto getNext() const -> DSMEGACKBitmapFragment* {
        return next;
    }

    auto getBits() const -> uint8_t {
        return bits;
    }

    auto getSequenceNumber() const -> uint8_t {
        return sequenceNumber;
    }

protected:
    DSMEGACKBitmapFragment *next;
    uint8_t sequenceNumber;
    uint8_t bits;
};

} /* namespace dsme */

#endif /* DSMEGACKBITMAP_H_ */
