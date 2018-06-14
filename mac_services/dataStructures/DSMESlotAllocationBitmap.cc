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

#include "./DSMESlotAllocationBitmap.h"

#include "./DSMEBitVector.h"
#include "./DSMESABSpecification.h"
#include "./GTS.h"

namespace dsme {

DSMESlotAllocationBitmap::DSMESlotAllocationBitmap() : numSuperframesPerMultiSuperframe(0), numGTSlotsFirstSuperframe(0), numGTSlotsLatterSuperframes(0), numChannels(0) {
}

void DSMESlotAllocationBitmap::initialize(uint16_t numSuperframesPerMultiSuperframe, uint8_t numGTSlotsFirstSuperframe, uint8_t numGTSlotsLatterSuperframes, uint8_t numChannels) {
    this->numSuperframesPerMultiSuperframe = numSuperframesPerMultiSuperframe;
    this->numGTSlotsFirstSuperframe = numGTSlotsFirstSuperframe;
    this->numGTSlotsLatterSuperframes = numGTSlotsLatterSuperframes;
    this->numChannels = numChannels;
    occupied.initialize((numGTSlotsFirstSuperframe + (numSuperframesPerMultiSuperframe-1)*numGTSlotsLatterSuperframes) * numChannels);
    return;
}

void DSMESlotAllocationBitmap::clear() {
    occupied.fill(false);
}

uint16_t DSMESlotAllocationBitmap::getSubblockOffset(uint8_t subBlockIndex) const {
    if(subBlockIndex == 0) {
        return 0;
    }
    else {
        return (numGTSlotsFirstSuperframe + (subBlockIndex-1)*numGTSlotsLatterSuperframes) * numChannels;
    }
}

void DSMESlotAllocationBitmap::addOccupiedSlots(const DSMESABSpecification& subBlock) {
    occupied.setOperationJoin(subBlock.getSubBlock(), getSubblockOffset(subBlock.getSubBlockIndex()));
    return;
}

void DSMESlotAllocationBitmap::removeOccupiedSlots(const DSMESABSpecification& subBlock) {
    occupied.setOperationComplement(subBlock.getSubBlock(), getSubblockOffset(subBlock.getSubBlockIndex()));
    return;
}

void DSMESlotAllocationBitmap::getOccupiedSubBlock(DSMESABSpecification& subBlock, uint16_t subBlockIndex) const {
    subBlock.getSubBlock().copyFrom(occupied, getSubblockOffset(subBlockIndex));
    return;
}

void DSMESlotAllocationBitmap::getOccupiedChannels(BitVector<MAX_CHANNELS>& channelVector, uint16_t subBlockIndex, uint16_t subBlockOffset) const {
    channelVector.copyFrom(occupied, getSubblockOffset(subBlockIndex) + subBlockOffset * numChannels);
}

bool DSMESlotAllocationBitmap::isOccupied(abs_slot_idx_t idx) {
    return occupied.get(idx);
}

} /* namespace dsme */
