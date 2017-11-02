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

TPSTxData::TPSTxData() : avgIn(0) {
}

void TPS::setAlpha(float alpha) {
    this->alpha = alpha;
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
                  << "stotTarget");

        header = true;
    }

    for(TPSTxData& data : this->txLinks) {
        DSME_ASSERT(alpha > 0);
        data.avgIn = data.messagesInLastMultisuperframe * alpha + data.avgIn * (1 - alpha);

        uint8_t slots = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(data.address, Direction::TX);
        float error = data.avgIn - slots;

        int8_t change = 0;
        if(error > 0) {
            change = ceil(error);
        } else if(error < -2) {
            change = ceil(error) + 1;
        }

        data.slotTarget = slots + change;

        LOG_DEBUG("control"
                  << "," << this->dsmeAdaptionLayer.getDSME().getMAC_PIB().macShortAddress << "," << data.address << "," << data.messagesInLastMultisuperframe
                  << "," << data.messagesOutLastMultisuperframe << "," << data.avgIn << "," << slots << "," << data.slotTarget);

        data.messagesInLastMultisuperframe = 0;
        data.messagesOutLastMultisuperframe = 0;
    }
}

} /* namespace dsme */
