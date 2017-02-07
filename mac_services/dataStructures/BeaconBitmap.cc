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

/* INCLUDES ******************************************************************/

#include "BeaconBitmap.h"

namespace dsme {

/* Function DEFINITIONS ******************************************************/

BeaconBitmap::BeaconBitmap() : sdIndex(0) {
}

void BeaconBitmap::setSDIndex(uint16_t SDIndex) {
    this->sdIndex = SDIndex;
}

uint16_t BeaconBitmap::getSDIndex() const {
    return sdIndex;
}

bool BeaconBitmap::setSDBitmapLengthBytes(uint16_t SDBitmapLengthBytes, bool value) {
    return sdBitmap.setLength(SDBitmapLengthBytes * 8, value);
}

uint16_t BeaconBitmap::getSDBitmapLengthBytes() const {
    return sdBitmap.length() / 8;
}

void BeaconBitmap::fill(bool value) {
    sdBitmap.fill(value);
}

void BeaconBitmap::set(uint16_t position, bool value) {
    sdBitmap.set(position, value);
}

bool BeaconBitmap::get(uint16_t position) const {
    return sdBitmap.get(position);
}

void BeaconBitmap::copyBitsFrom(const BeaconBitmap& bitmap) {
    sdBitmap.copyFrom(bitmap.sdBitmap);
}

void BeaconBitmap::orWith(const BeaconBitmap& bitmap) {
    sdBitmap.setOperationJoin(bitmap.sdBitmap);
}

int32_t BeaconBitmap::getNextAllocated(uint16_t start) const {
    for(uint16_t i = start; i < sdBitmap.length(); i++) {
        if(sdBitmap.get(i)) {
            return i;
        }
    }
    for(uint16_t i = 0; i < start; i++) {
        if(sdBitmap.get(i)) {
            return i;
        }
    }

    return -1;
}

int32_t BeaconBitmap::getNextFreeSlot() const {
    for(uint16_t i = 0; i < sdBitmap.length(); i++) {
        if(!sdBitmap.get(i)) {
            return i;
        }
    }

    return -1;
}

int32_t BeaconBitmap::getRandomFreeSlot(uint16_t randomNumber) const {
    uint16_t start = randomNumber % sdBitmap.length();
    for(uint16_t i = start; i < sdBitmap.length(); i++) {
        if(!sdBitmap.get(i)) {
            return i;
        }
    }
    for(uint16_t i = 0; i < start; i++) {
        if(!sdBitmap.get(i)) {
            return i;
        }
    }

    return -1;
}

uint16_t BeaconBitmap::getAllocatedCount() const {
    uint16_t c = 0;
    for(unsigned int i = 0; i < sdBitmap.length(); i++) {
        if(sdBitmap.get(i)) {
            c++;
        }
    }
    return c;
}

uint8_t BeaconBitmap::getSerializationLength() const {
    uint8_t size = 0;
    size += 2;                                        // SD Index
    size += 2;                                        // SD Bitmap Length
    size += BITVECTOR_BYTE_LENGTH(sdBitmap.length()); // SD Bitmap
    return size;
}

Serializer& operator<<(Serializer& serializer, BeaconBitmap& b) {
    serializer << b.sdIndex;

    if(serializer.getType() == SERIALIZATION) {
        uint16_t SDBitmapLength = BITVECTOR_BYTE_LENGTH(b.sdBitmap.length());
        serializer << SDBitmapLength;
    } else {
        uint16_t SDBitmapLength = 0; /* unused value */
        serializer << SDBitmapLength;
        b.sdBitmap.setLength(SDBitmapLength * 8);
    }

    serializer << b.sdBitmap;

    return serializer;
}
}
