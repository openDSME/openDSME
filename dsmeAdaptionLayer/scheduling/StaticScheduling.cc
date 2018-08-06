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

#include "./StaticScheduling.h"

#include "../../../dsme_platform.h"
#include "../../dsmeLayer/DSMELayer.h"
#include "../../mac_services/dataStructures/IEEE802154MacAddress.h"
#include "../../mac_services/pib/MAC_PIB.h"
#include "../DSMEAdaptionLayer.h"


void StaticScheduling::multisuperframeEvent() {
}


GTSSchedulingDecision StaticScheduling::getNextSchedulingAction(uint16_t address) {
    return NO_SCHEDULING_ACTION; 
}

void addStaticSlot(uint16_t absSlotID, uint16_t address, DIRECTION direction) {
    uint8_t slotID;
    uint8_t superframeID; 
    uint8_t channelID; 
    fromAbsSlotID(absSlotID, slotID, superframeID, channelID); 
    
    assert(dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.add(superframeID, slotID, channelID, direction, address, ACTState::VALID));
}

void StaticScheduling::fromAbsSlotID(const uint8_t absSlotID, uint8_t &slotID, uint8_t &superframeID, uint8_t &channelID) const {
    slotID = 0;
    superframeID = 0;
    channelID = 0;

    uint8_t absSlot = absSlotID;
    for(uint8_t i=0; i<this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe(); i++) {
        if(absSlot >= this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(i) * this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumChannels()) {
            absSlot -= this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(i) * this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumChannels();
            superframeID++;
        } else {
            for(uint8_t slot=0; slot<this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(i); slot++) {
                if(absSlot >= this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumChannels()) {
                    absSlot -= this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumChannels();
                } else {
                    slotID = slot; 
                    channelID = absSlot;
                    return; 
                }
            }
        }
    }
}

} /* namespace dsme */
