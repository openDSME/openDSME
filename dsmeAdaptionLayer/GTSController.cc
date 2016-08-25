/*
 * openDSME
 *
 * Implementation of the Deterministic & Synchronous Multi-channel Extension (DSME)
 * described in the IEEE 802.15.4-2015 standard
 *
 * Authors: Florian Meier <florian.meier@tuhh.de>
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

#include "GTSController.h"

#include "../../dsme_platform.h"
#include "../mac_services/pib/MAC_PIB.h"
#include "DSMEAdaptionLayer.h"

constexpr int16_t K_P_POS = 0;
constexpr int16_t K_I_POS = 28;

constexpr int16_t K_P_NEG = 74;
constexpr int16_t K_I_NEG = 16;

constexpr uint16_t SCALING = 128;

namespace dsme {

GTSControllerData::GTSControllerData() : messagesInLastMultisuperframe(0), messagesOutLastMultisuperframe(0), error_sum(0), control(1) {

}

GTSController::GTSController(DSMEAdaptionLayer &dsmeAdaptionLayer) : dsmeAdaptionLayer(dsmeAdaptionLayer){

}

void GTSController::registerIncomingMessage(uint16_t address) {
    LOG_DEBUG("Controller-Incoming");

    iterator it = this->links.find(address);
    if(it == this->links.end()) {
        GTSControllerData data;
        data.address = address;
        data.messagesInLastMultisuperframe++;
        this->links.insert(data, address);
    } else {
        it->messagesInLastMultisuperframe++;
    }
    return;
}

void GTSController::registerOutgoingMessage(uint16_t address) {
    iterator it = this->links.find(address);
    DSME_ASSERT(it != this->links.end());

    it->messagesOutLastMultisuperframe++;
    return;
}

void GTSController::multisuperframeStartEvent() {
    for(GTSControllerData &d : this->links) {

        uint16_t w = d.messagesInLastMultisuperframe;
        uint16_t y = d.messagesOutLastMultisuperframe;

        int16_t e = w - y;
        int16_t &i = d.error_sum;
        int16_t &u = d.control;

        i += e;

        if(e > 0) {
            u = (K_P_POS * e + K_I_POS * i) / SCALING;
        } else {
            u = (K_P_NEG * e + K_I_NEG * i) / SCALING;
        }

        LOG_DEBUG("Controller-Data->" << d.address << " w: " << w << "; y: " << y << "; e: " << e << "; i: " << i << "; u: " << u);

        d.messagesInLastMultisuperframe = 0;
        d.messagesOutLastMultisuperframe = 0;
    }
}

int16_t GTSController::getControl(uint16_t address) {
    iterator it = this->links.find(address);
    DSME_ASSERT(it != this->links.end());

    return it->control;
}

static uint16_t abs(int16_t v) {
    if(v > 0) {
        return v;
    } else {
        return -v;
    }
}

uint16_t GTSController::getPriorityLink() {
    uint16_t address = IEEE802154MacAddress::NO_SHORT_ADDRESS;
    int16_t control = 0;
    for(GTSControllerData &d : this->links) {
        if(abs(control) < abs(d.control)) {
            control = d.control;
            address = d.address;
        }
    }
    return address;
}

} /* dsme */
