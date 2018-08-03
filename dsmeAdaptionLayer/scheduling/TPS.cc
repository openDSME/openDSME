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

#include "./TPS.h"

#include <cmath>
#include <deque>
#include "../../../dsme_platform.h"
#include "../../dsmeLayer/DSMELayer.h"
#include "../../mac_services/dataStructures/IEEE802154MacAddress.h"
#include "../../mac_services/pib/MAC_PIB.h"
#include "../DSMEAdaptionLayer.h"

static bool header = false;

namespace dsme {

TPSTxData::TPSTxData() : avgIn(0) {
}

void TPS::setAlpha(float alpha) {
    this->alpha = alpha;
}

void TPS::setMinFreshness(uint16_t minFreshness) {
    this->minFreshness = minFreshness;
}

void TPS::setUseHysteresis(bool useHysteresis) {
    this->useHysteresis = useHysteresis;
}

void TPS::multisuperframeEvent() {
    if(!header) {
        LOG_DEBUG("control"
                  << ","
                  << "from"
                  << ","
                  << "to"
                  << ","
                  << "in"
                  << ","
                  << "out"
                  << ","
                  << "avgIn"
                  << ","
                  << "slots"
                  << ","
                  << "slotTarget"
                  << ","
                  << "freshness");

        header = true;
    }

    for(TPSTxData& data : this->txLinks) {
        DSME_ASSERT(alpha > 0);
        DSME_ASSERT(minFreshness > 0);
        data.avgIn = data.messagesInLastMultisuperframe * alpha + data.avgIn * (1 - alpha);

        uint8_t slots = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(data.address, Direction::TX);
        float error = data.avgIn - slots;

        int8_t change = 0;
        if(useHysteresis) {
            if(error > 0) {
                change = ceil(error);
            } else if(error < -2) {
                change = ceil(error) + 1;
            }
        }
        else {
            change = ceil(error);
        }

        data.slotTarget = slots + change;

        if(data.multisuperframesSinceLastPacket > minFreshness) {
            data.slotTarget = 0;
        }

        if(data.messagesInLastMultisuperframe == 0) {
            if(data.multisuperframesSinceLastPacket < 0xFFFE) {
                data.multisuperframesSinceLastPacket++;
            }
        }
        else {
            data.multisuperframesSinceLastPacket = 0;
        }

        LOG_DEBUG("control"
                  << ",0x" << HEXOUT << this->dsmeAdaptionLayer.getDSME().getMAC_PIB().macShortAddress
                  << ",0x" << data.address << "," << DECOUT << data.messagesInLastMultisuperframe
                  << "," << data.messagesOutLastMultisuperframe << "," << FLOAT_OUTPUT(data.avgIn) << "," << (uint16_t)slots << "," << data.slotTarget
                  << "," << data.multisuperframesSinceLastPacket);

        data.messagesInLastMultisuperframe = 0;
        data.messagesOutLastMultisuperframe = 0;
    }
}



/*GTSSchedulingDecision TPS::getNextSchedulingAction(uint16_t address) {
    uint16_t numAllocatedSlots = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(address, Direction::TX);
    int16_t target = getSlotTarget(address);

    if(target > numAllocatedSlots) {
        uint8_t numInputs = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(0) + this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(1) * (this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe()-1);
        int8_t schedule[numInputs];
        observeState(schedule, numInputs);
        
        uint8_t slot = schedule_tx(schedule, numInputs, this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(1));
        if(lastAction == slot) {
            slot = this->dsmeAdaptionLayer.getDSME().getPlatform().getRandom() % numInputs;
            std::cout << "Switching to new action" << std::endl;
            //slot = schedule_tx(schedule, numInputs, this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(1));
        }
        lastAction = slot;
        logState(schedule, numInputs);        

        uint8_t superframeID;
        uint8_t slotID; 
        fromActionID(slot, slotID, superframeID);  

        return GTSSchedulingDecision{address, ManagementType::ALLOCATION, Direction::TX, 1, superframeID, slotID};
    } else if(target < numAllocatedSlots && numAllocatedSlots > 1) {
        return GTSSchedulingDecision{address, ManagementType::DEALLOCATION, Direction::TX, 1, 0, 0};
    } else {
        return NO_SCHEDULING_ACTION;
    }
}*/

uint8_t TPS::schedule_tx(int8_t *schedule, uint8_t slots, uint8_t superframe_length) const {
    float delay[slots] = {255};
    for(uint8_t tx_slot=0; tx_slot<slots; tx_slot++) {
	    if(schedule[tx_slot] != 0) {
		    delay[tx_slot] = 255;
		    continue;
	    }
	
	    uint8_t local_delay[slots] = {255};
	    for(uint8_t gen_slot=0; gen_slot<slots; gen_slot++) {
		
	        std::deque<uint8_t> queue;
	        uint8_t max_delay = 0;
	        for(uint8_t eval_slot=0; eval_slot<2*slots; eval_slot++) {
		        uint8_t actual_slot = eval_slot % slots;
                if(actual_slot == gen_slot) {
                    queue.push_back(0);
                }
                if(schedule[actual_slot] == -1) {
		            queue.push_back(0);
		        }	
		        if((schedule[actual_slot] == 1 || actual_slot == tx_slot) && !queue.empty()) {
		            max_delay = max_delay > queue.front() ? max_delay : queue.front();
		            queue.pop_front();
		        }
		        for(uint8_t elem=0; elem<queue.size(); elem++) {
		            if((actual_slot+1)%superframe_length == 0) {
   			            queue[elem] = queue[elem] + 10;
		            } else {
			            queue[elem] = queue[elem] + 1;
		            }
		        }
	        }
	        std::cout << (int)max_delay << " "; 
	        local_delay[gen_slot] = max_delay;
	    }
	    std::cout << std::endl;

	    /* Calculate expected delay */
	    uint32_t exp_delay = 0;
	    for(uint8_t slot=0; slot<slots; slot++) {
	        exp_delay += local_delay[slot];
	    }
	    delay[tx_slot] = exp_delay / (float)slots;
    }	   
   
    for(uint8_t i=0; i<slots;i++) {
    	std::cout << " " << delay[i];
    }
    std::cout << std::endl;

    uint8_t min_exp_delay_slot = 0;
    for(uint8_t slot=0; slot<slots; slot++) {
	    min_exp_delay_slot = delay[min_exp_delay_slot] <= delay[slot] ? min_exp_delay_slot : slot;
    }
    std::cout << "slot: " << (int)min_exp_delay_slot << std::endl; 
    return min_exp_delay_slot;
}

void TPS::observeState(int8_t *state, uint8_t numStates) const {
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

uint8_t TPS::toActionID(const uint8_t slotID, const uint8_t superframeID) const {
    uint8_t actionID = slotID; 
    if(superframeID > 0) actionID += this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(0);
    if(superframeID > 1) actionID += this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(1) * (superframeID - 1);
        
    return actionID; 
}

void TPS::fromActionID(const uint8_t actionID, uint8_t &slotID, uint8_t &superframeID) const {
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

void TPS::logState(int8_t *state, uint8_t numStates) const {
    std::cout << "{" << "\"id\" : " << this->dsmeAdaptionLayer.getMAC_PIB().macShortAddress << ", \"slots\" : ["; 
    for(int i=0; i<numStates; i++) {
        std::cout << (int)state[i] << " ";
    }  
    std::cout << "]}" << std::endl;
}


} /* namespace dsme */
