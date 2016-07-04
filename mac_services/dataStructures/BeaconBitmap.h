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

#ifndef BEACONBITMAP_H
#define BEACONBITMAP_H

/* INCLUDES ******************************************************************/

#include <stdint.h>

#include "../../../dsme_settings.h"
#include "DSMEBitVector.h"

namespace dsme {

class BeaconBitmap_Iterator;

/* CLASSES *******************************************************************/

class BeaconBitmap {
public:
    /*
     * Creates a new BeaconBitmap
     */
    BeaconBitmap();

    void setSDIndex(uint16_t SDIndex);

    uint16_t getSDIndex() const;

    bool setSDBitmapLengthBytes(uint16_t SDBitmapLengthBytes, bool value);

    uint16_t getSDBitmapLengthBytes() const;

    /*
     * Fills the whole bitmap with one value
     * @param value written to every bit of the bitmap
     */
    void fill(bool value);

    /*
     * Sets a specific bit of the bitmap
     * @param position bit to set
     * @param value bit gets set to this
     */
    void set(uint16_t position, bool value);

    /*
     * Get the value of a specific bit
     * @param position bit to read
     * @return value of the bit
     */
    bool get(uint16_t position) const;

    /*
     * Copy all bits from another bitmap
     * @param bitmap other bitmap to copy bits from
     */
    void copyBitsFrom(const BeaconBitmap& bitmap);

    /*
     * Perform a bitwise OR with another bitmap, overwrite own bits
     * @param bitmap other bitmap to OR with
     */
    void orWith(const BeaconBitmap& bitmap);

    /**
     * Get index of next allocated slot after start
     * @param start first index to check
     * @return -1 if no slot found, valid index else
     */
    int32_t getNextAllocated(uint16_t start) const;

    /**
     * Get index of next free slot in bitmap.
     * @return -1 if no slot found, valid index else
     */
    int32_t getNextFreeSlot() const;

    /**
     * Get index of random free slot in bitmap.
     * @return -1 if no slot found, valid index else
     */
    int32_t getRandomFreeSlot(uint16_t randomNumber) const;

    /**
     * Count allocated slots
     */
    uint16_t getAllocatedCount() const;

    uint8_t getSerializationLength() const;
    friend Serializer& operator<<(Serializer& serializer, BeaconBitmap& b);

private:
    uint16_t sdIndex;                           // current beacon allocation in beacon interval
    BitVector<MAX_TOTAL_SUPERFRAMES> sdBitmap;  // bitmap // TODO total superframes or just superframes?
};

Serializer& operator<<(Serializer& serializer, BeaconBitmap& b);

}

#endif /* BEACONBITMAP_H_ */
