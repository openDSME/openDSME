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
#include "../dsmeLayer/DSMELayer.h"
#include "../mac_services/dataStructures/IEEE802154MacAddress.h"

#include <iostream>

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

void GTSController::multisuperframeEvent() {
    LOG_DEBUG("Controller-Calculating");

    global_multisuperframe++;

    DSMEPlatform& platform = *dynamic_cast<DSMEPlatform*>(&(this->dsmeAdaptionLayer.getDSME().getPlatform()));

    bool flipEnabled = platform.par("flipTime").longValue() != -1;

    if(flipEnabled && global_multisuperframe >= platform.par("flipTime").longValue() + platform.par("evaluationDuration").longValue()) {
        platform.endSimulation();
    }

    uint8_t flipLink{0};

    if(flipEnabled) {
        if(this->links.size() > 0) {
            uint8_t active_size = 0;
            for(GTSControllerData& data : this->links) {
                if(this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedTxGTS(data.address) > 0) {
                    active_size++;
                }
            }
            if(active_size > 0) {
                uint8_t link_choice = platform.par("flipLink").longValue() % active_size;

                for(GTSControllerData& data : this->links) {
                    if(this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedTxGTS(data.address) > 0) {
                        if(link_choice == 0) {
                            break;
                        }
                        flipLink++;
                        link_choice--;
                    }
                }
            }
        }
    }

    uint8_t i = 0;
    for(GTSControllerData& data : this->links) {
        data.queueSize[data.history_position] += data.messagesIn[data.history_position] - data.messagesOut[data.history_position];
        uint16_t slots = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedTxGTS(data.address);
        uint16_t myAddress = this->dsmeAdaptionLayer.getMAC_PIB().macShortAddress;

        if(flipEnabled && i == flipLink && myAddress == platform.par("flipNode").longValue()) {
            std::cout << "trace: ";
            std::cout << simTime() << ",";
            std::cout << myAddress << ",";
            std::cout << data.address << ",";
            std::cout << data.control << ",";
            std::cout << slots << ",";
            std::cout << std::endl;
        }

        if(flipEnabled && i == flipLink && myAddress == platform.par("flipNode").longValue() && global_multisuperframe >= platform.par("flipTime").longValue()) {
            if(global_multisuperframe == platform.par("flipTime").longValue()) {
                std::cout << "state: ";
                std::cout << myAddress << ",";
                std::cout << data.address << ",";
                std::cout << slots << ",";
                std::cout << platform.par("flipOption").longValue() << ",";
                std::cout << platform.par("evaluationDuration").longValue() << ",";

                uint8_t k = data.history_position;
                for(uint8_t j = 0; j < CONTROL_HISTORY_LENGTH; j++) {
                    std::cout << data.queueSize[k] << ",";
                    // std::cout << data.messagesIn[k] << ",";
                    // std::cout << data.messagesOut[k] << ",";

                    k++;
                    if(k >= CONTROL_HISTORY_LENGTH) {
                        k = 0;
                    }
                }
                std::cout << std::endl;

                data.control = platform.par("flipOption").longValue();
            } else {
                /* do not change control value any more */
            }
        } else {
            float control_input_array[CONTROL_HISTORY_LENGTH + 1];
            control_input_array[0] = slots;
            uint8_t k = data.history_position;
            for(uint8_t j = 0; j < CONTROL_HISTORY_LENGTH; j++) {
                control_input_array[j + 1] = data.queueSize[k];
                k++;
                if(k >= CONTROL_HISTORY_LENGTH) {
                    k = 0;
                }
            }
            quicknet::vector_t input{CONTROL_HISTORY_LENGTH + 1, control_input_array};

            /* input: slots | l0 | l1 | l2 | l3 | l4 | l5 | l6 | l7 */
            quicknet::vector_t& output = this->network.feedForward(input);
            /* output: 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 */ // TODO

            data.control = slots + quicknet::idmax(output) - 2; // TODO

            std::cout << "network: ";
            std::cout << myAddress << ",";
            std::cout << data.control << ",";
            std::cout << std::endl;
        }

        uint16_t currentQueueSize = data.queueSize[data.history_position];
        ++data.history_position;
        if(data.history_position >= CONTROL_HISTORY_LENGTH) {
            data.history_position = 0;
        }
        data.queueSize[data.history_position] = currentQueueSize;
        data.messagesIn[data.history_position] = 0;
        data.messagesOut[data.history_position] = 0;

        i++;
    }
}

int16_t GTSController::getControl(uint16_t address) {
    iterator it = this->links.find(address);
    DSME_ASSERT(it != this->links.end());

    return it->control;
}

//void GTSController::indicateChange(uint16_t address, int16_t change) {
//    iterator it = this->links.find(address);
//    DSME_ASSERT(it != this->links.end());
//
//    it->control -= change;
//    return;
//}

static uint16_t abs(int16_t v) {
    if(v > 0) {
        return v;
    } else {
        return -v;
    }
}

uint16_t GTSController::getPriorityLink() {
    uint16_t address = IEEE802154MacAddress::NO_SHORT_ADDRESS;
    int16_t difference = 0;
    for(const GTSControllerData& d : this->links) {
        uint16_t slots = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedTxGTS(d.address);

        if(abs(difference) < abs(d.control - slots)) {
            difference = d.control - slots;
            address = d.address;
        }
    }
    return address;
}

} /* namespace dsme */
