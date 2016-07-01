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

#include "GTS.h"

namespace dsme {
const GTS GTS::UNDEFINED = GTS(0xffff, 0xffff, 0xff);

GTS::GTS(uint16_t superframeID, uint16_t slotID, uint8_t channel) {
    this->superframeID = superframeID;
    this->slotID = slotID;
    this->channel = channel;
}
GTS::GTS(const GTS &other) {
    this->superframeID = other.superframeID;
    this->slotID = other.slotID;
    this->channel = other.channel;

}
GTS& GTS::operator=(const GTS &other) {
    this->superframeID = other.superframeID;
    this->slotID = other.slotID;
    this->channel = other.channel;
    return (*this);
}

// TODO: Types -> Related to TODO in GTSManager.h
abs_slot_idx_t GTS::absoluteIndex(uint8_t numGTSlots, uint8_t numChannels) {
    abs_slot_idx_t idx = this->superframeID;
    idx *= numGTSlots;
    idx += this->slotID;
    idx *= numChannels;
    idx += this->channel;
    return idx;
}

GTS GTS::GTSfromAbsoluteIndex(abs_slot_idx_t idx, uint8_t numGTSlots, uint8_t numChannels, uint8_t numSuperframesPerMultiSuperframe) {
    uint8_t channel = idx % numChannels;
    idx /= numChannels;
    uint16_t slotID = idx % numGTSlots;
    idx /= numGTSlots;
    uint16_t superframeID = idx % numSuperframesPerMultiSuperframe;
    idx /= numSuperframesPerMultiSuperframe;
    return GTS(superframeID, slotID, channel);
}

} /* dsme */

