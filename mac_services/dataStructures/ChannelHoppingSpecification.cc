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

#include "./ChannelHoppingSpecification.h"

#include "./Serializer.h"

namespace dsme {

/* Function DEFINITIONS ******************************************************/

ChannelHoppingSpecification::ChannelHoppingSpecification() : hoppingSequenceId(0), panCoordinatorBSN(0), channelOffset(0) {
}

uint8_t ChannelHoppingSpecification::getChannelOffsetBitmapLength() const {
    return channelOffsetBitmap.length();
}

void ChannelHoppingSpecification::setChannelOffsetBitmapLength(uint8_t channelOffsetBitmapLength) {
    channelOffsetBitmap.setLength(channelOffsetBitmapLength);
}

uint16_t ChannelHoppingSpecification::getChannelOffset() const {
    return channelOffset;
}

void ChannelHoppingSpecification::setChannelOffset(uint16_t channelOffset) {
    this->channelOffset = channelOffset;
}

BitVector<MAX_CHANNELS>& ChannelHoppingSpecification::getChannelOffsetBitmap() {
    return channelOffsetBitmap;
}

void ChannelHoppingSpecification::setChannelOffsetBitmap(const BitVector<MAX_CHANNELS>& channelOffsetBitmap) {
    this->channelOffsetBitmap = channelOffsetBitmap;
}

uint8_t ChannelHoppingSpecification::getHoppingSequenceId() const {
    return hoppingSequenceId;
}

void ChannelHoppingSpecification::setHoppingSequenceId(
        uint8_t hoppingSequenceId) {
    this->hoppingSequenceId = hoppingSequenceId;
}

uint8_t ChannelHoppingSpecification::getPanCoordinatorBsn() const {
    return panCoordinatorBSN;
}

void ChannelHoppingSpecification::setPanCoordinatorBsn(uint8_t panCoordinatorBsn) {
    panCoordinatorBSN = panCoordinatorBsn;
}

uint8_t ChannelHoppingSpecification::getSerializationLength() const {
    uint8_t size = 0;
    size += 1;                                        // Hopping Sequence ID
    size += 1;                                        // PAN Coordinator BSN
    size += 2;                                        // Channel Offset
    size += 1;                                        // Channel Offset Bitmap Length
    size += BITVECTOR_BYTE_LENGTH(channelOffsetBitmap.length()); // Channel Offset Bitmap
    return size;
}

Serializer& operator<<(Serializer& serializer, ChannelHoppingSpecification& b) {
    serializer << b.hoppingSequenceId;
    serializer << b.panCoordinatorBSN;
    serializer << b.channelOffset;

    if(serializer.getType() == SERIALIZATION) {
        uint8_t channelOffsetBitmapLength = BITVECTOR_BYTE_LENGTH(b.channelOffsetBitmap.length());
        serializer << channelOffsetBitmapLength;
    } else {
        uint8_t channelOffsetBitmapLength = 0;
        serializer << channelOffsetBitmapLength;
        b.channelOffsetBitmap.setLength(channelOffsetBitmapLength * 8);
    }

    serializer << b.channelOffsetBitmap;

    return serializer;
}

} /* namespace dsme */
