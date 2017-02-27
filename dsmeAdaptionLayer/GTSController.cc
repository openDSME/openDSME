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

#include "./GTSController.h"

#include "../../dsme_platform.h"
#include "../mac_services/dataStructures/IEEE802154MacAddress.h"

namespace dsme {

GTSController::GTSController(DSMEAdaptionLayer& dsmeAdaptionLayer) : dsmeAdaptionLayer(dsmeAdaptionLayer) {
}

void GTSController::reset() {
    while(this->links.size() > 0) {
        auto it = this->links.begin();
        this->links.remove(it);
    }
}

void GTSController::registerIncomingMessage(uint16_t address) {
    LOG_DEBUG("Controller-Incoming");

    iterator it = this->links.find(address);
    if(it == this->links.end()) {
        GTSControllerData data;
        data.address = address;
        data.messagesIn[data.history_position]++;
        this->links.insert(data, address);
    } else {
        it->messagesIn[it->history_position]++;
    }
    return;
}

void GTSController::registerOutgoingMessage(uint16_t address) {
    iterator it = this->links.find(address);
    if(it != this->links.end()) {
        it->messagesOut[it->history_position]++;
    }

    return;
}

void GTSController::superframeEvent() {
    global_superframe++;

    for(GTSControllerData& data : this->links) {
        data.queue_size += data.messagesIn[data.history_position] - data.messagesOut[data.history_position];
        uint16_t slots = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedTxGTS(data.address);

        data.control = slots + data.queue_size - 1;

        data.history_position++;
        if(data.history_position >= CONTROL_HISTORY_LENGTH) {
            data.history_position = 0;
        }
        data.messagesIn[data.history_position] = 0;
        data.messagesOut[data.history_position] = 0;
    }
}

int16_t GTSController::getControl(uint16_t address) {
    iterator it = this->links.find(address);
    DSME_ASSERT(it != this->links.end());

    return it->control;
}

void GTSController::indicateChange(uint16_t address, int16_t change) {
    iterator it = this->links.find(address);
    DSME_ASSERT(it != this->links.end());

    it->control -= change;
    return;
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
    for(const GTSControllerData& d : this->links) {
        if(abs(control) < abs(d.control)) {
            control = d.control;
            address = d.address;
        }
    }
    return address;
}

} /* namespace dsme */
