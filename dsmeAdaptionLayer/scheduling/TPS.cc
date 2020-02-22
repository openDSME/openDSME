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
#include "../../../dsme_platform.h"
#include "../../dsmeLayer/DSMELayer.h"
#include "../../mac_services/dataStructures/IEEE802154MacAddress.h"
#include "../../mac_services/pib/MAC_PIB.h"
#include "../DSMEAdaptionLayer.h"

static bool header = false;

namespace dsme {

TPSTxData::TPSTxData() : avgIn(0), multisuperframesSinceLastPacket(0) {
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

void TPS::setUseMultiplePacketsPerGTS(bool useMultiplePackets) {
    this->useMultiplePacketsPerGTS = useMultiplePackets;
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
        float error = (data.avgIn / packets_per_slot) - slots;
        LOG_DEBUG("TPS error: " << error);
        LOG_DEBUG("TPS slots: " << (int)slots);

        int8_t change = 0;
        if(useHysteresis) {
            if(error > 0) {
                change = ceil(error);
            } else if(error < -2) {
                //IAMG. PROof of concept capOncapOff. queue management implementation in case the incomming traffic decreases drastically
                uint8_t queueLevel = GTSSchedulingImpl::registerIncomingMessage(data.address);
                //if the level of the queue is equal or greater than the number of allocated slots. then change = 0
                if(queueLevel >= slots){
                    LOG_DEBUG("IAMG. Inside TPS queueLevel >= slots");
                    change = 0; //keep slots in order to alliviate the queue
                }else if(queueLevel < slots){
                    LOG_DEBUG("IAMG. Inside TPS queueLevel < slots");
                    change= ceil(queueLevel-slots) +1; // deallocate slots that are not needed to supply queue demand and keep 1 slot.
                }
                //change = ceil(error) + 1; //original code
            }
        } else {
            change = ceil(error);
        }

        data.slotTarget = slots + change;
        LOG_DEBUG("TPS target: " << data.slotTarget);

        if(data.messagesInLastMultisuperframe == 0) {
            if(data.multisuperframesSinceLastPacket < 0xFFFE) {
                data.multisuperframesSinceLastPacket++;
            }
        } else {
            data.multisuperframesSinceLastPacket = 0;
        }

        if(data.multisuperframesSinceLastPacket > minFreshness) {
            data.slotTarget = 0;
        }

        LOG_DEBUG("control"
                  << ",0x" << HEXOUT << this->dsmeAdaptionLayer.getDSME().getMAC_PIB().macShortAddress << ",0x" << data.address << "," << DECOUT
                  << data.messagesInLastMultisuperframe << "," << data.messagesOutLastMultisuperframe << "," << FLOAT_OUTPUT(data.avgIn) << ","
                  << (uint16_t)slots << "," << data.slotTarget << "," << data.multisuperframesSinceLastPacket);

        data.messagesInLastMultisuperframe = 0;
        data.messagesOutLastMultisuperframe = 0;
    }
}

uint8_t TPS::registerIncomingMessage(uint16_t address){
    uint8_t queueLevel = GTSSchedulingImpl::registerIncomingMessage(address);
    this->dsmeAdaptionLayer.getDSME().getPlatform().signalGTSQueueLevel(true); // signal one incoming packet to the queue
    return queueLevel;
}

void TPS::registerOutgoingMessage(uint16_t address, bool success, int32_t serviceTime, uint8_t queueAtCreation){
    GTSSchedulingImpl::registerOutgoingMessage(address,success,serviceTime,queueAtCreation);
    this->dsmeAdaptionLayer.getDSME().getPlatform().signalGTSQueueLevel(false); // signal one incoming packet to the queue
}


} /* namespace dsme */
