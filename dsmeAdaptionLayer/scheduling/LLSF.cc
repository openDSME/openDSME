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

#include "./LLSF.h"

#include "../../../dsme_platform.h"
#include "../../dsmeLayer/DSMELayer.h"
#include "../../mac_services/dataStructures/IEEE802154MacAddress.h"
#include "../../mac_services/pib/MAC_PIB.h"
#include "../../mac_services/pib/dsme_mac_constants.h"
#include "../DSMEAdaptionLayer.h"

#include <iostream>


namespace dsme {


void LLSF::multisuperframeEvent() {
    for(GTSSchedulingData &data : this->txLinks) {
        data.slotTarget = data.messagesInLastMultisuperframe; 
        
        // reset counter 
        data.messagesInLastMultisuperframe = 0;
        data.messagesOutLastMultisuperframe = 0;
    }
}

GTSSchedulingDecision LLSF::getNextSchedulingAction(uint16_t address) {
    uint16_t scheduledCells = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(address, Direction::TX);
    int16_t requiredCells = getSlotTarget(address);

    std::cout << (int) scheduledCells << "/" << (int) requiredCells <<  std::endl;
 
    if(scheduledCells < requiredCells) {
        // allocate slot 
        uint8_t superframeID = 0; 
        uint8_t slotID = 0; 

        if(true) {
            // allocate slot randomly 
            uint8_t numSuperFramesPerMultiSuperframe = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe();
            superframeID = this->dsmeAdaptionLayer.getRandom() % numSuperFramesPerMultiSuperframe;
            
            uint8_t numGTSlots = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(superframeID);
            slotID = this->dsmeAdaptionLayer.getRandom() % numGTSlots;
        } else {
            // allocate according to rules (1)-(3)
            std::vector<uint8_t> allocatedSlots;
            std::vector<uint8_t> slots;
            std::vector<uint8_t> superframes;        

            DSMEAllocationCounterTable& macDSMEACT = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT;  
            for(DSMEAllocationCounterTable::iterator it = macDSMEACT.begin(); it != macDSMEACT.end(); it++) {
                if(it->getDirection() == Direction::RX) {
                    uint8_t absSlot = aNumSuperframeSlots*it->getSuperframeID() + (aNumSuperframeSlots-dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(it->getSuperframeID())) + it->getGTSlotID();
                    for(int i=0; i<allocatedSlots.size(); i++) {
                        if(absSlot < allocatedSlots[i]) {
                            allocatedSlots.insert(allocatedSlots.begin() + i, absSlot);
                            slots.insert(slots.begin() + i, it->getGTSlotID());
                            superframes.insert(superframes.begin() + i, it->getSuperframeID());
                            break;
                        }
                    }
                    if(allocatedSlots.size() == 0) { 
                        allocatedSlots.push_back(absSlot); 
                        slots.push_back(it->getGTSlotID());
                        superframes.push_back(it->getSuperframeID()); 
                    }
                }
            }

            std::cout << "-------------------------------" << std::endl;
            for(int i=0; i<allocatedSlots.size(); i++) {
                std::cout << (int)allocatedSlots[i] << " " << (int)slots[i] << " " << (int)superframes[i] << std::endl;
            } 
            std::cout << "end" << std::endl;

            // calcualte gaps 
            int8_t maxGap = -1;
            for(uint8_t i=0; i<allocatedSlots.size(); i++) {
                int8_t gap = abs(allocatedSlots[i] - allocatedSlots[(allocatedSlots.size()+i-1)%allocatedSlots.size()]); 
                if(gap > maxGap) {
                    maxGap = gap;
                    superframeID = superframes[i];
                    slotID = slots[i];
                    std::cout << "new: " << (int)slotID << " " << (int)superframeID <<  " " << gap << std::endl; 
                }
            }
        }

        return GTSSchedulingDecision{address, ManagementType::ALLOCATION, Direction::TX, 1, superframeID, slotID};
    } else if(requiredCells < scheduledCells - sf0Thresh) {
        // deallocate slot
        uint8_t superframeID = 0; 
        uint8_t slotID = 0; 

        if(!this->isLeaf()) {
             // allocate according to rules (1)-(3)
            std::vector<uint8_t> allocatedSlots;
            std::vector<uint8_t> slots;
            std::vector<uint8_t> superframes;        

            DSMEAllocationCounterTable& macDSMEACT = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT;  
            for(DSMEAllocationCounterTable::iterator it = macDSMEACT.begin(); it != macDSMEACT.end(); it++) {
                if(it->getDirection() == Direction::TX) {
                    uint8_t absSlot = aNumSuperframeSlots*it->getSuperframeID() + (aNumSuperframeSlots-dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(it->getSuperframeID())) + it->getGTSlotID();
                    for(int i=0; i<allocatedSlots.size(); i++) {
                        if(absSlot < allocatedSlots[i]) {
                            allocatedSlots.insert(allocatedSlots.begin() + i, absSlot);
                            slots.insert(slots.begin() + i, it->getGTSlotID());
                            superframes.insert(superframes.begin() + i, it->getSuperframeID());
                            break;
                        }
                    }
                    if(allocatedSlots.size() == 0) { 
                        allocatedSlots.push_back(absSlot); 
                        slots.push_back(it->getGTSlotID());
                        superframes.push_back(it->getSuperframeID()); 
                    }
                }
            }

            // calcualte gaps 
            int8_t maxGap = -1;
            for(uint8_t i=0; i<allocatedSlots.size(); i++) {
                int8_t gap = abs(allocatedSlots[i] - allocatedSlots[(allocatedSlots.size()+i-1)%allocatedSlots.size()]); 
                if(gap > maxGap) {
                    maxGap = gap;
                    superframeID = superframes[i];
                    slotID = slots[i];
                    std::cout << "new: " << (int)slotID << " " << (int)superframeID <<  " " << gap << std::endl; 
                }
            }
        }

        std::cout << "DEALLOC" << std::endl;
        return GTSSchedulingDecision{address, ManagementType::DEALLOCATION, Direction::TX, 1, superframeID, slotID};
    } else {
        std::cout << "Nope" << std::endl;
        return NO_SCHEDULING_ACTION; 
    }
}

bool LLSF::isLeaf() {
    DSMEAllocationCounterTable& macDSMEACT = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT;  
    for(DSMEAllocationCounterTable::iterator it = macDSMEACT.begin(); it != macDSMEACT.end(); it++) {
        if(it->getDirection() == Direction::RX) {
            return false;
        } 
    }  
    return true;
} 

} /* namespace dsme */
