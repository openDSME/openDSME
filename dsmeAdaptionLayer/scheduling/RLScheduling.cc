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
    if(!initialized) {
        if(dsmeAdaptionLayer.getMAC_PIB().macShortAddress > 7) {
            cursor = 1;
        } else {
            cursor = 0;
        }
        initialized = true;
    }

    // Observe initial state
    uint8_t numInputs = (this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(0) + this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(1) * (this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe()-1)) * 2;
    float initialState[numInputs] = {0};
    observeState(initialState, numInputs);
    logState(initialState, numInputs);

    // Get next action  
    quicknet::Vector<float> input{numInputs, initialState};
    quicknet::Vector<float> &output = this->network.feedForward(input);
    uint8_t actionID = quicknet::idmax(output);

    std::cout << "{" << "\"id\" : " << dsmeAdaptionLayer.getMAC_PIB().macShortAddress << ", \"action\" : " << (int)actionID << "}" << std::endl; 

    switch(actionID) {
        case 0: 
            return deallocateSlot(address); 
        case 1:     
            return allocateSlot(address); 
        case 2: // cursor left 
            cursor = (cursor + numInputs / 2 - 2) % numInputs / 2;
            return NO_SCHEDULING_ACTION;
        case 3: // cursor right 
            cursor = (cursor + 2) % numInputs / 2; 
            return NO_SCHEDULING_ACTION;
    } 
}

GTSSchedulingDecision RLScheduling::allocateSlot(uint16_t address) const {
    uint16_t numAllocatedSlots = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(address, Direction::TX);
    if(numAllocatedSlots == 0) {
        uint8_t numSuperFramesPerMultiSuperframe = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe();
        uint8_t randomSuperframeID = this->dsmeAdaptionLayer.getRandom() % numSuperFramesPerMultiSuperframe;

        uint8_t numGTSlots = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(randomSuperframeID);
        uint8_t randomSlotID = this->dsmeAdaptionLayer.getRandom() % numGTSlots;

        return GTSSchedulingDecision{address, ManagementType::ALLOCATION, Direction::TX, 1, randomSuperframeID, randomSlotID};
    } else {
        uint8_t slotID = 0;
        uint8_t superframeID = 0;            
        fromActionID(cursor, slotID, superframeID);
           
        std::cout << "{" << "\"id\" : " << dsmeAdaptionLayer.getMAC_PIB().macShortAddress << ", \"action\" : alloc" << ", \"slot\" : " << (int)slotID << ", \"superframe\" : " << (int)superframeID << "}" << std::endl; 

 
        return GTSSchedulingDecision{address, ManagementType::ALLOCATION, Direction::TX, 1, superframeID, slotID};
    }
}

GTSSchedulingDecision RLScheduling::deallocateSlot(uint16_t address) const {
    uint16_t numAllocatedSlots = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(address, Direction::TX);
    if(numAllocatedSlots < 2) {
        return NO_SCHEDULING_ACTION; 
    }

    uint8_t slotID = 0;
    uint8_t superframeID = 0;            
    fromActionID(cursor, slotID, superframeID);
        
    std::cout << "{" << "\"id\" : " << dsmeAdaptionLayer.getMAC_PIB().macShortAddress << ", \"action\" : dealloc" << ", \"slot\" : " << (int)slotID << ", \"superframe\" : " << (int)superframeID << "}" << std::endl; 

    return GTSSchedulingDecision{IEEE802154MacAddress::NO_SHORT_ADDRESS, ManagementType::DEALLOCATION, Direction::TX, 1, superframeID, slotID};
}

void RLScheduling::observeState(float *state, uint8_t numStates) const {
    for(uint8_t i=0; i<numStates; i++) state[i] = 0;
    DSMEAllocationCounterTable& macDSMEACT = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT;
    DSMESlotAllocationBitmap& macDSMESAB = this->dsmeAdaptionLayer.getMAC_PIB().macDSMESAB;
    uint16_t numChannels = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumChannels(); 
    
    for(auto const& value: macDSMEACT) {
        uint8_t actionID = toActionID(value.getGTSlotID(), value.getSuperframeID()); 
        if(value.getDirection() == Direction::TX) {
            state[actionID] = 1;        
        } 
        if(value.getDirection() == Direction::RX) {
            state[actionID] = -1;        
        } 
    }
    
    state[numStates/2 + cursor] = 1;
    /*for(uint8_t i=0; i<numStates/2; i++) {
        uint8_t superframeID, slotID; 
        fromActionID(i, slotID, superframeID);       
        BitVector<MAX_CHANNELS> occupied;
        occupied.setLength(numChannels);
        macDSMESAB.getOccupiedChannels(occupied, superframeID, slotID);
        for(uint8_t channel=0; channel<numChannels; channel++) {
            if(occupied.get(channel)) {
                state[numStates/2 + i] = 1;
                break;
            }
        }
    }*/ 
}

void RLScheduling::logState(float *state, uint8_t numStates) const {
    std::cout << "{" << "\"id\" : " << this->dsmeAdaptionLayer.getMAC_PIB().macShortAddress << ", \"slots\" : ["; 
    for(int i=0; i<numStates; i++) {
        std::cout << state[i] << " ";
        if(i == numStates/2-1) {
            std::cout << "|";
        }
    }  
    std::cout << "]}" << std::endl;
}

uint8_t RLScheduling::toActionID(const uint8_t slotID, const uint8_t superframeID) const {
    uint8_t actionID = slotID; 
    if(superframeID > 0) actionID += this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(0);
    if(superframeID > 1) actionID += this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(1) * (superframeID - 1);
        
    return actionID; 
}

void RLScheduling::fromActionID(const uint8_t actionID, uint8_t &slotID, uint8_t &superframeID) const {
    slotID = 0;
    superframeID = 0;
    uint8_t action = actionID;
    for(uint8_t i=0; i<this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe(); i++) {
        if(action >= this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(i)) {
            action -= this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(i);
            superframeID++;
        } else {
            slotID = action;
            return;
        }
    }
}


} /* namespace dsme */
