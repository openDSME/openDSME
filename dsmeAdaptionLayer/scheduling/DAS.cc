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

#include "./DAS.h"

#include <cmath>
#include "../../../dsme_platform.h"
#include "../../dsmeLayer/DSMELayer.h"
#include "../../mac_services/dataStructures/IEEE802154MacAddress.h"
#include "../../mac_services/pib/MAC_PIB.h"
#include "../DSMEAdaptionLayer.h"

static bool header = false;

namespace dsme {

DASTxData::TPSTxData() : avgIn{0}, pastQValues{0}, packetsInLastMultisuperframe{0} {
}

void DAS::setAlpha(float alpha) {
    this->alpha = alpha;
}


void DAS::setUseMultiplePacketsPerGTS(bool useMultiplePackets) {
    this->useMultiplePacketsPerGTS = useMultiplePackets;
}

void DAS::multisuperframeEvent() {


    for(DASTxData& data : this->txLinks) {
        DSME_ASSERT(alpha > 0);
        DSME_ASSERT(minFreshness > 0);
        data.avgIn = data.messagesInLastMultisuperframe * alpha + data.avgIn * (1 - alpha);

        uint8_t slots = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(data.address, Direction::TX);

        //LengthFrameInSymbols = Preamble +SFD + PHR + PSDU (PHYPayload)
        //Preamble = 8 symbols;
        //SFD = 2 symbols;
        //PHR = 2 symbols;
        //PSDU = MHR + MACPayload + MFR;
        uint8_t packets_per_slot = 1;
        if(useMultiplePacketsPerGTS) {
            packets_per_slot = (this->dsmeAdaptionLayer.getMAC_PIB().helper.getSymbolsPerSlot() - PRE_EVENT_SHIFT) / ((6 + 127)*2 + this->dsmeAdaptionLayer.getMAC_PIB().helper.getAckWaitDuration() + const_redefines::macLIFSPeriod);
                /* '-> calculate number of packets per slot with assumption of maximum packet size and maximum acknowledgement wait duration -> THIS CAN BE DONE MUCH BETTER */
        }

        LOG_DEBUG("Packets per slot: " << (int)packets_per_slot);
        // TODO
        data.slotTarget = slots;     // + change;
        LOG_DEBUG("DAS target: " << data.slotTarget);

        data.messagesInLastMultisuperframe = 0;
        data.messagesOutLastMultisuperframe = 0;
    }

}

GTSSchedulingDecision DAS::getNextSchedulingAction(uint16_t address) {
       uint16_t numAllocatedSlots = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(address, Direction::TX);

       int16_t target = getSlotTarget(address);

       if(target > numAllocatedSlots) {
           uint8_t numSuperFramesPerMultiSuperframe = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe();
           uint8_t numGTSlots = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(randomSuperframeID);
           DSMESlotAllocationBitmap& macDSMESAB = this->dsmeAdaptionLayer.getMAC_PIB().macDSMESAB;
           uint8_t numChannels = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumChannels();
           DSMEAllocationCounterTable& macDSMEACT = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT;
           //  auswahl superframe und GTS


           uint8_t currentSF = this->dsmeAdaptionLayer.getDSME().getCurrentSuperframe();
           uint8_t targetSF = (currentSF + 1) % numSuperFramesPerMultiSuperframe;

           for(uint8_t superframe; superframe < numSuperFramesPerMultiSuperframe; superframe++) {
               for(uint8_t timeslot = 8; timeslot < (numGTSlots-1);timeslot++){
                   if(!macDSMEACT.isAllocated((targetSF + superframe)%numSuperFramesPerMultiSuperframe,timeslot)) {
                       if(!macDSMEACT.isAllocated((targetSF + superframe)%numSuperFramesPerMultiSuperframe,timeslot+1)) {
                           return GTSSchedulingDecision{address, ManagementType::DEALLOCATION, Direction::TX, 1, (targetSF + superframe)%numSuperFramesPerMultiSuperframe, timeslot};
                       }
                   }
               }
           }
              // neuer loot von i = 7 bis timeslot 8, sonst i-- und try again
              // Matrix mit abständen bilden, niedrigsten abstand auswählen, break bei abstand 1


           return GTSSchedulingDecision{address, ManagementType::ALLOCATION, Direction::TX, 1, randomSuperframeID, randomSlotID};
       } else if(target < numAllocatedSlots && numAllocatedSlots > 1) {
           /* TODO: slot and superframe ID are currently ignored for DEALLOCATION */
           return GTSSchedulingDecision{address, ManagementType::DEALLOCATION, Direction::TX, 1, 0, 0};
       } else {
           return NO_SCHEDULING_ACTION;
       }
   }
GTSSchedulingDecision DAS::getNextSchedulingActionRx(uint16_t address,uint8_t prefSF) {
              uint8_t numSuperFramesPerMultiSuperframe = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe();
              uint8_t numGTSlots = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(randomSuperframeID);
              DSMESlotAllocationBitmap& macDSMESAB = this->dsmeAdaptionLayer.getMAC_PIB().macDSMESAB;
              uint8_t numChannels = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumChannels();
              DSMEAllocationCounterTable& macDSMEACT = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT;
              //  auswahl superframe und GTS

              for(uint8_t timeslot = 8; timeslot < (numGTSlots-1);timeslot++){
                 if(!macDSMEACT.isAllocated((prefSF),timeslot)) {
                    if(!macDSMEACT.isAllocated((prefSF),timeslot+1)) {
                         return GTSSchedulingDecision{address, ManagementType::DEALLOCATION, Direction::TX, 1, prefSF, timeslot};
                         }
                    }
              }

                 // neuer loot von i = 7 bis timeslot 8, sonst i-- und try again
                 // Matrix mit abständen bilden, niedrigsten abstand auswählen, break bei abstand 1

              return GTSSchedulingDecision{address, ManagementType::ALLOCATION, Direction::TX, 1, randomSuperframeID, randomSlotID};
}

} /* namespace dsme */
