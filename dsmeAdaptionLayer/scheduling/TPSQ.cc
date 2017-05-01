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

#include "./TPSQ.h"

#include "../../../dsme_platform.h"
#include "../../mac_services/dataStructures/IEEE802154MacAddress.h"
#include "../../dsmeLayer/DSMELayer.h"
#include <cmath>

constexpr uint16_t SCALING = 10;
constexpr uint16_t bounds[] = {8,17,26,35,44,53,62,71,80,89,98,106,115,124,133,142,151,160,169,177,186,193,202,211,220,229,238,247};
static bool header = false;

namespace dsme {

TPSQData::TPSQData() : avgIn(0), totalInSystem(0), maServiceTimePerQueueLength(0), lastMusu(0) {
}

void TPSQ::registerOutgoingMessage(uint16_t address, bool success, int32_t serviceTime, uint8_t queueAtCreation) {
    queueLevel--;
    iterator it = this->links.find(address);
    if(it != this->links.end()) {
        it->messagesOutLastMultisuperframe++;

        if(success) {
            float a = 0.5; // TODO -> adapt to frequency
            float serviceTimePerQueueLength = (serviceTime/(float)queueAtCreation);
            it->maServiceTimePerQueueLength = it->maServiceTimePerQueueLength*a + (1-a)*serviceTimePerQueueLength;
        }
    }

    return;
}

void TPSQ::multisuperframeEvent() {
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

    for(TPSQData& data : this->links) {
        float a = 0.5; // TODO no float
        data.avgIn = data.messagesInLastMultisuperframe*a + data.avgIn*(1-a);
        data.totalInSystem += data.messagesInLastMultisuperframe - data.messagesOutLastMultisuperframe;

        uint8_t reqCap;
        for(reqCap = 1; reqCap <= sizeof(bounds); reqCap++) {
            if(bounds[reqCap-1] >= data.avgIn*SCALING) {
                break;
            }
        }

        // TODO avoid this calculation
        uint32_t now = dsmeAdaptionLayer.getDSME().getPlatform().getSymbolCounter();
        uint32_t musuDuration = now-data.lastMusu;
        data.lastMusu = now;

        uint8_t slots = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedTxGTS(data.address);
        float predCap = std::min((float)slots,musuDuration/data.maServiceTimePerQueueLength);

        float error = reqCap - predCap;
        int8_t change;
        if(-1.5 <= error && error <= -1.0) {
            change = 0;
        }
        else {
            change = std::ceil(error);
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

} /* namespace dsme */
