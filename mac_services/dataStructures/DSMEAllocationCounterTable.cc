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

#include "DSMEAllocationCounterTable.h"

#include "../../../dsme_platform.h"

using namespace dsme;

DSMEAllocationCounterTable::DSMEAllocationCounterTable() {
}

void DSMEAllocationCounterTable::initialize(uint16_t numSuperFramesPerMultiSuperframe, uint8_t numGTSlots, uint8_t numChannels) {
    this->numSuperFramesPerMultiSuperframe = numSuperFramesPerMultiSuperframe;
    this->numGTSlots = numGTSlots;
    this->numChannels = numChannels;
    bitmap.initialize(numSuperFramesPerMultiSuperframe * numGTSlots, false);
}

DSMEAllocationCounterTable::iterator DSMEAllocationCounterTable::begin() {
    return act.begin();
}

DSMEAllocationCounterTable::iterator DSMEAllocationCounterTable::end() {
    return act.end();
}

DSMEAllocationCounterTable::iterator DSMEAllocationCounterTable::find(uint16_t superframeID, uint8_t gtSlotID) {
    ACTPosition pos;
    pos.superframeID = superframeID;
    pos.gtSlotID = gtSlotID;
    return act.find(pos);
}


void DSMEAllocationCounterTable::printChange(const char* type, uint16_t superframeID, uint8_t gtSlotID, uint8_t channel, bool direction, uint16_t address) {
    LOG_INFO_PREFIX;LOG_INFO_PURE(DECOUT << type << " " << palId_id());
    if (direction == TX) {
        LOG_INFO_PURE(">");
    } else {
        LOG_INFO_PURE("<");
    }LOG_INFO_PURE(address << " " << (uint16_t)(gtSlotID+9) << "," << superframeID << "," << (uint16_t)channel << cometos::endl);
}

bool DSMEAllocationCounterTable::add(uint16_t superframeID, uint8_t gtSlotID, uint8_t channel, Direction direction, uint16_t address, ACTState state) {
    printChange("alloc", superframeID, gtSlotID, channel, direction, address);

    if(isAllocated(superframeID, gtSlotID)) {
        DSME_ASSERT(false);
    }
    DSME_ASSERT(!isAllocated(superframeID, gtSlotID));

    bitmap.set(superframeID * numGTSlots + gtSlotID, true);

    if (direction == TX) {
        RBTree<uint16_t, uint16_t>::iterator numSlotIt = numAllocatedTxSlots.find(address);
        if (numSlotIt == numAllocatedTxSlots.end()) {
            LOG_INFO("Inserting 0x" << HEXOUT << address << DECOUT<< " into numAllocatedTxSlots.");
            numAllocatedTxSlots.insert(1, address);
        } else {
            (*numSlotIt)++;
            LOG_INFO("Incrementing slot count for " << address << DECOUT << " (now at " << *numSlotIt << ".");
        }
    } else {
        LOG_INFO("Slot marked for reception from " << address << ".");
    }

    ACTPosition pos;
    pos.superframeID = superframeID;
    pos.gtSlotID = gtSlotID;
    return act.insert(ACTElement(superframeID, gtSlotID, channel, direction, address, state), pos);
}

void DSMEAllocationCounterTable::remove(DSMEAllocationCounterTable::iterator it) {
    DSME_ASSERT(it != act.end());

    uint16_t superframeID = it->getSuperframeID();
    uint8_t gtSlotID = it->getGTSlotID();

    printChange("dealloc", it->superframeID, it->slotID, it->channel, it->direction, it->address);

    DSME_ASSERT(isAllocated(superframeID, gtSlotID));

    if (it->direction == TX) {
        RBTree<uint16_t, uint16_t>::iterator numSlotIt = numAllocatedTxSlots.find(it->address);
        DSME_ASSERT(numSlotIt != numAllocatedTxSlots.end());
        (*numSlotIt)--;
        LOG_INFO("Decrementing slot count for " << it->address << DECOUT << " (now at " << *numSlotIt << ".");
        if ((*numSlotIt) == 0) {
            numAllocatedTxSlots.remove(numSlotIt);
        }
    }

    act.remove(it);

    bitmap.set(superframeID * numGTSlots + gtSlotID, false);
}

bool DSMEAllocationCounterTable::isAllocated(uint16_t superframeID, uint8_t gtSlotID) const {
    return bitmap.get(superframeID * numGTSlots + gtSlotID);
}

uint16_t DSMEAllocationCounterTable::getNumAllocatedTxGTS(uint16_t address) {
    RBTree<uint16_t, uint16_t>::iterator numSlotIt = numAllocatedTxSlots.find(address);
    if (numSlotIt == numAllocatedTxSlots.end()) {
        return 0;
    } else {
        return *numSlotIt;
    }
}

void DSMEAllocationCounterTable::setACTStateIfExists(DSMESABSpecification &subBlock, ACTState state) {
    Direction ignoredDirection = TX;
    setACTState(subBlock, state, ignoredDirection, 0);
}

void DSMEAllocationCounterTable::setACTState(DSMESABSpecification &subBlock, ACTState state, Direction direction, uint16_t deviceAddress) {
    // Supporting more than one slot allocation induces many open issues and is probably not needed most of the time.
    DSME_ASSERT(subBlock.getSubBlock().count(true) == 1);

    for (DSMESABSpecification::SABSubBlock::iterator it = subBlock.getSubBlock().beginSetBits();
            it != subBlock.getSubBlock().endSetBits();
            it++) {
        GTS gts = GTS::GTSfromAbsoluteIndex((*it) + subBlock.getSubBlockIndex() * numGTSlots * numChannels, numGTSlots,
                numChannels,
                numSuperFramesPerMultiSuperframe);
        DSMEAllocationCounterTable::iterator actit = find(gts.superframeID, gts.slotID);
        LOG_DEBUG("search slot " << (uint16_t)gts.slotID << " " << (uint16_t)gts.superframeID);

        if(actit != end() && state == REMOVED) {
            DSME_ASSERT(actit->getChannel() == gts.channel);
            remove(actit);
            continue;
        }

        if(actit == end()) {
            // does not yet exist
            if(deviceAddress != 0) {
                // -> create, otherwise "setACTStateIfExists" was called
                LOG_DEBUG("add slot " << (uint16_t)gts.slotID << " " << (uint16_t)gts.superframeID);
                bool added = add(gts.superframeID, gts.slotID, gts.channel, direction, deviceAddress, state);
                DSME_ASSERT(added);
            }
        }
        else {
            LOG_DEBUG("set slot " << (uint16_t)actit->getGTSlotID() << " " << (uint16_t)actit->getSuperframeID() << " " << (uint16_t)actit->getChannel());
            DSME_ASSERT(actit->getChannel() == gts.channel);
            actit->setState(state);
        }
    }
}

