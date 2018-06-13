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

#ifndef DSMEALLOCATIONCOUNTERTABLE_H_
#define DSMEALLOCATIONCOUNTERTABLE_H_

#include "../../../dsme_settings.h"
#include "./ACTElement.h"
#include "./DSMEBitVector.h"
#include "./DSMESABSpecification.h"
#include "./RBTree.h"
#include "../../interfaces/IDSMEPlatform.h"

namespace dsme {

struct ACTPosition {
    uint16_t superframeID;
    uint8_t gtSlotID;

    bool operator>(const ACTPosition& other) const {
        if(this->superframeID > other.superframeID) {
            return true;
        }
        if(this->superframeID < other.superframeID) {
            return false;
        }
        return this->gtSlotID > other.gtSlotID;
    }

    bool operator<(const ACTPosition& other) const {
        if(this->superframeID < other.superframeID) {
            return true;
        }
        if(this->superframeID > other.superframeID) {
            return false;
        }
        return this->gtSlotID < other.gtSlotID;
    }

    bool operator==(const ACTPosition& other) const {
        return ((superframeID == other.superframeID) && (gtSlotID == other.gtSlotID));
    }
};

class DSMELayer;

// own allocated slots
class DSMEAllocationCounterTable {
public:
    typedef RBTree<ACTElement, ACTPosition>::iterator iterator;
    typedef bool (*condition_t)(ACTElement);

    DSMEAllocationCounterTable();

    void initialize(uint16_t numSuperFramesPerMultiSuperframe, uint8_t numGTSlots, uint8_t numChannels, DSMELayer* dsme);

    iterator begin();

    iterator end();

    void clear();

    iterator find(uint16_t superframeID, uint8_t gtSlotID);

    void printChange(const char* type, uint16_t superframeID, uint8_t gtSlotID, uint8_t channel, bool direction, uint16_t address);

    bool add(uint16_t superframeID, uint8_t gtSlotID, uint8_t channel, Direction direction, uint16_t address, ACTState state);

    void remove(iterator it);

    bool isAllocated(uint16_t superframeID, uint8_t gtSlotID) const;

    uint16_t getNumAllocatedGTS(uint16_t address, Direction direction);

    void setACTState(DSMESABSpecification& subBlock, ACTState state, Direction direction, uint16_t deviceAddress, uint16_t channelOffset, bool useChannelOffset,
                     bool checkAddress = false);
    void setACTState(DSMESABSpecification& subBlock, ACTState state, Direction direction, uint16_t deviceAddress, uint16_t channelOffset, bool useChannelOffset,
                     condition_t condition, bool checkAddress = false);
    void setACTStateIfExists(DSMESABSpecification& subBlock, ACTState state, uint16_t channelOffset);

private:
    DSMEAllocationCounterTable(const DSMEAllocationCounterTable& other) = delete;

    uint16_t numSuperFramesPerMultiSuperframe;
    uint8_t numGTSlots;
    uint8_t numChannels;

    BitVector<MAX_SUPERFRAMES_PER_MULTI_SUPERFRAME * MAX_GTSLOTS> bitmap;
    RBTree<ACTElement, ACTPosition> act;

    // TODO integrate this nicely into the NeighborQueue
    RBTree<uint16_t, uint16_t> numAllocatedSlots[2]; // 0 == TX, 1 == RX

    DSMELayer* dsme;
};

} /* namespace dsme */

#endif /* DSMEALLOCATIONCOUNTERTABLE_H_ */
