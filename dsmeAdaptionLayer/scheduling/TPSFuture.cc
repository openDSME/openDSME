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

#include "./TPSFuture.h"

#include <cmath>
#include "../../../dsme_platform.h"
#include "../../dsmeLayer/DSMELayer.h"
#include "../../mac_services/dataStructures/IEEE802154MacAddress.h"
#include "../../mac_services/pib/MAC_PIB.h"
#include "../DSMEAdaptionLayer.h"

static bool header = false;

namespace dsme {

TPSFTxData::TPSFTxData() : avgIn(0), multisuperframesSinceLastPacket(0) {
}

void TPSFuture::setAlpha(float alpha) {
    this->alpha = alpha;
}

void TPSFuture::setMinFreshness(uint16_t minFreshness) {
    this->minFreshness = minFreshness;
}

void TPSFuture::setUseHysteresis(bool useHysteresis) {
    this->useHysteresis = useHysteresis;
}

void TPSFuture::setFutureLength(uint16_t multisuperframes) {
    this->futureLength = multisuperframes; 
}

void TPSFuture::setAllocateLeastSlots(bool allocateLeastSlots) {
    this->allocateLeastSlots = allocateLeastSlots;
}

void TPSFuture::multisuperframeEvent() {
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

    for(TPSFTxData& data : this->txLinks) {
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
        } else {
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
        } else {
            data.multisuperframesSinceLastPacket = 0;
        }

        LOG_DEBUG("control"
                  << ",0x" << HEXOUT << this->dsmeAdaptionLayer.getDSME().getMAC_PIB().macShortAddress << ",0x" << data.address << "," << DECOUT
                  << data.messagesInLastMultisuperframe << "," << data.messagesOutLastMultisuperframe << "," << FLOAT_OUTPUT(data.avgIn) << ","
                  << (uint16_t)slots << "," << data.slotTarget << "," << data.multisuperframesSinceLastPacket);

        data.messagesInLastMultisuperframe = 0;
        data.messagesOutLastMultisuperframe = 0;
    }
}

GTSSchedulingDecision TPSFuture::getNextSchedulingAction(uint16_t address) {
    uint16_t numAllocatedSlots = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(address, Direction::TX);
 
    int16_t target = getSlotTarget(address);
    if(target > numAllocatedSlots) {
        uint8_t randomSuperframeID; 
        uint8_t numSuperFramesPerMultiSuperframe = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe();
        if(this->allocateLeastSlots) {
            uint8_t leftSlots = 0; 
            for(uint8_t i=0; i<this->futureLength; i++) {
                uint8_t checkID = (this->dsmeAdaptionLayer.getDSME().getCurrentSuperframe() + i) % numSuperFramesPerMultiSuperframe; 
                //TODO
            }
        } else {
            uint8_t randomSuperframeOffset = this->dsmeAdaptionLayer.getRandom() % this->futureLength;  
            randomSuperframeID = (this->dsmeAdaptionLayer.getDSME().getCurrentSuperframe() + randomSuperframeOffset) % numSuperFramesPerMultiSuperframe; 
        }
        
        uint8_t numGTSlots = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(randomSuperframeID);                                                          
        uint8_t randomSlotID = this->dsmeAdaptionLayer.getRandom() % numGTSlots;
        return GTSSchedulingDecision{address, ManagementType::ALLOCATION, Direction::TX, 1, randomSuperframeID, randomSlotID};
    } else if(target < numAllocatedSlots && numAllocatedSlots > 1) {
        /* TODO: slot and superframe ID are currently ignored for DEALLOCATION */
        return GTSSchedulingDecision{address, ManagementType::DEALLOCATION, Direction::TX, 1, 0, 0};
    } else {
        return NO_SCHEDULING_ACTION;
    }
}

} /* namespace dsme */
