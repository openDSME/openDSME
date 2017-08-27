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

#include "../../../dsme_platform.h"
#include "../DSMEAdaptionLayer.h"
#include "../../mac_services/dataStructures/IEEE802154MacAddress.h"
#include "../../mac_services/pib/MAC_PIB.h"
#include "../../dsmeLayer/DSMELayer.h"
#include <cmath>

static bool header = false;

namespace dsme {

TPSTxData::TPSTxData() : avgIn(0), totalInSystem(0), maServiceTimePerQueueLength(0), lastMusu(0) {
}

void TPS::registerOutgoingMessage(uint16_t address, bool success, int32_t serviceTime, uint8_t queueAtCreation) {
    queueLevel--;
    iterator it = this->txLinks.find(address);
    if(it != this->txLinks.end()) {
        it->messagesOutLastMultisuperframe++;

        if(success) {
            float a = 0.5; // TODO -> adapt to frequency
            float serviceTimePerQueueLength = (serviceTime/(float)queueAtCreation);
            it->maServiceTimePerQueueLength = it->maServiceTimePerQueueLength*a + (1-a)*serviceTimePerQueueLength;
        }
    }

    return;
}

void TPS::multisuperframeEvent() {
    if(!header) {
        LOG_DEBUG("control"
             << "," << "from"
             << "," << "to"
             << "," << "in"
             << "," << "out"
             << "," << "avgIn"
             << "," << "totalInSystem"
             << "," << "reqCap"
             << "," << "slots"
             << "," << "stotTarget");
             
        header = true;
    }

    int sumslots = 0;
    for(TPSRxData& data : this->rxLinks) {
        uint8_t slots = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(data.address,Direction::RX);
        float a = 0.1; // TODO no float
        int unused = slots-data.messagesRxLastMultisuperframe;
        if(unused < 0) {
            unused = 0;
        }
        data.unused = unused*a + data.unused*(1-a);

        sumslots++;

        if(slots > 0 || data.messagesRxLastMultisuperframe > 0) {
            LOG_ERROR("RXSTATS 0x" << HEXOUT << data.address << " \t" << DECOUT << data.messagesRxLastMultisuperframe << " \t" << slots << " \t" << unused << " \t" << data.unused);
        }

        data.messagesRxLastMultisuperframe = 0;
    }
    LOG_ERROR("RXSTATS ------------ " << sumslots);

    for(TPSTxData& data : this->txLinks) {
        float a = 0.5; // TODO no float
        data.avgIn = data.messagesInLastMultisuperframe*a + data.avgIn*(1-a);
        data.totalInSystem += data.messagesInLastMultisuperframe - data.messagesOutLastMultisuperframe;

        auto reqCap = slotsForLoad(data.avgIn);

        // TODO avoid this calculation
        uint32_t now = dsmeAdaptionLayer.getDSME().getPlatform().getSymbolCounter();
        uint32_t musuDuration = now-data.lastMusu;
        data.lastMusu = now;

        uint8_t slots = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(data.address,Direction::TX);
        float predCap = std::min((float)slots,musuDuration/data.maServiceTimePerQueueLength);

        //a = 0.01;
        a = 0.1;
        float error = slotsForLoad(data.messagesInLastMultisuperframe)-slots;
        data.maError = error*a + data.maError*(1-a);
        error = data.maError;

        //float error = reqCap - predCap;
        /*
        int8_t change;
        if(-1.5 <= error && error <= -1.0) {
            change = 0;
        }
        else {
            change = std::ceil(error);
        }
        */

        int8_t change = 0;
        if(error > 0) {
            //change = 1;
            change = ceil(error);
        }
        else if(error < -2) {
            change = -1;
        }

        data.slotTarget = slots + change;

        LOG_DEBUG("control"
             << "," << this->dsmeAdaptionLayer.getDSME().getMAC_PIB().macShortAddress
             << "," << data.address
             << "," << data.messagesInLastMultisuperframe
             << "," << data.messagesOutLastMultisuperframe
             << "," << data.avgIn
             << "," << data.totalInSystem
             << "," << reqCap
             << "," << slots
             << "," << data.slotTarget
             );

        data.messagesInLastMultisuperframe = 0;
        data.messagesOutLastMultisuperframe = 0;
    }
}

float TPS::slotsForLoad(float packetsPerMUSU) {
    return packetsPerMUSU;
}

} /* namespace dsme */
