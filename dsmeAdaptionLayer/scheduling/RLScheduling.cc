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

#include "./RLScheduling.h"
#include <iostream>
#include "../../../DSMEPlatform.h"
#include "../../dsmeLayer/DSMELayer.h"


namespace dsme {

GTSSchedulingDecision RLScheduling::getNextSchedulingAction(uint16_t address) {
    uint16_t numAllocatedSlots = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(address, Direction::TX);
    int16_t target = getSlotTarget(address);

    if(target > numAllocatedSlots) {
        // Observe initial state
        uint8_t numInputs = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(0) + this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(1) * (this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe()-1);
        float initialState[numInputs] = {0};
        observeState(initialState, numInputs);
  
        // Decide for an action  
        DSMEPlatform& platform = *dynamic_cast<DSMEPlatform*>(&(this->dsmeAdaptionLayer.getDSME().getPlatform()));
        uint8_t eps = platform.par("eps").intValue();
        bool learning = platform.par("learning").boolValue();
        uint8_t actions = platform.par("actions").intValue();
        uint8_t p = this->dsmeAdaptionLayer.getRandom() % 100;
        if(p > eps*100 || !learning) {
            quicknet::Vector<float> input{numInputs, initialState};
            quicknet::Vector<float> &output = this->network.feedForward(input);
            uint8_t actionID = quicknet::idmax(output);
            
            uint8_t slotID = 0;
            uint8_t superframeID = 0;            
            fromActionID(actionID, slotID, superframeID);
            return GTSSchedulingDecision{address, ManagementType::ALLOCATION, Direction::TX, 1, superframeID, slotID};
        } else  {
            uint8_t numSuperFramesPerMultiSuperframe = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe();
            uint8_t randomSuperframeID = this->dsmeAdaptionLayer.getRandom() % numSuperFramesPerMultiSuperframe;
            uint8_t numGTSlots = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(randomSuperframeID);
            uint8_t randomSlotID = this->dsmeAdaptionLayer.getRandom() % numGTSlots;
            uint8_t actionID = toActionID(randomSlotID, randomSuperframeID);            

            return GTSSchedulingDecision{address, ManagementType::ALLOCATION, Direction::TX, 1, randomSuperframeID, randomSlotID};
        }   
    
    // not handled in RL yet 
    } else if(target < numAllocatedSlots && numAllocatedSlots > 1) {
        /* TODO: slot and superframe ID are currently ignored for DEALLOCATION */
        return GTSSchedulingDecision{address, ManagementType::DEALLOCATION, Direction::TX, 1, 0, 0};
    } else {
        return NO_SCHEDULING_ACTION;
    }   
}

void RLScheduling::observeState(float *state, uint8_t numStates) const {
    for(uint8_t i=0; i<numStates; i++) state[i] = 0;
    DSMEAllocationCounterTable& macDSMEACT = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT;
    for(auto const& value: macDSMEACT) {
        uint8_t actionID = toActionID(value.getGTSlotID(), value.getSuperframeID()); 
        if(value.getDirection() == Direction::TX) {
            state[actionID] = 1;        
        } 
        if(value.getDirection() == Direction::RX) {
            state[actionID] = -1;        
        } 
    }
}

uint8_t RLScheduling::toActionID(const uint8_t slotID, const uint8_t superframeID) const {
    uint8_t actionID = slotID; 
    if(superframeID > 0) actionID += this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(0);
    if(superframeID > 1) actionID += (this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(1)-1);
        
    return actionID; 
}

void RLScheduling::fromActionID(const uint8_t actionID, uint8_t &slotID, uint8_t &superframeID) const {
    slotID = 0;
    superframeID = 0;
    uint8_t action = actionID;

    for(uint8_t i=0; i< this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe(); i++) {
        if(action > this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(i)) {
            action -= this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(i);
            superframeID++;
        } else {
            slotID = actionID;
            return;
        }
    }
}


} /* namespace dsme */
