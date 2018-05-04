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

#include "./MLController.h"
#include <iostream>
#include "../../../DSMEPlatform.h"
#include "../../dsmeLayer/DSMELayer.h"
#include "./TPS.h"

namespace dsme {

constexpr int16_t K_P_POS = 0;
constexpr int16_t K_I_POS = 30;
constexpr int16_t K_D_POS = 26;

constexpr int16_t K_P_NEG = 50;
constexpr int16_t K_I_NEG = 30;
constexpr int16_t K_D_NEG = 38;

constexpr uint16_t SCALING = 128;


MLControllerData::MLControllerData() : error_sum(0), last_error(0), queueLevel(0), transmissionRate(0), avgIn(0), multisuperframesSinceLastPacket(0) {
}

void MLController::multisuperframeEvent() {
    DSMEPlatform& platform = *dynamic_cast<DSMEPlatform*>(&(this->dsmeAdaptionLayer.getDSME().getPlatform()));

    for(MLControllerData& data : this->txLinks) {
        data.transmissionRate = data.messagesInLastMultisuperframe + (data.queueLevel - (data.messagesInLastMultisuperframe - data.messagesOutLastMultisuperframe));
        data.queueLevel += data.messagesInLastMultisuperframe - data.messagesOutLastMultisuperframe;
    }

    if(platform.par("learning").boolValue()) {
       doTPS(0.1, 28);
        //doPID();
    } else {
	static uint16_t maxTr= 0;
	static uint16_t maxRr= 0;
        for(MLControllerData& data : this->txLinks) {
            maxTr = data.transmissionRate > maxTr ? data.transmissionRate : maxTr;
            maxRr = data.messagesInLastMultisuperframe > maxRr ? data.transmissionRate : maxRr;
		
	    float inputArray[5];
            inputArray[0] = data.transmissionRate / maxTr;
	    inputArray[1] = data.messagesInLastMultisuperframe / maxRr;
            inputArray[2] = data.queueLevel / 22;
            inputArray[3] = dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(data.address, Direction::TX) / 14; 
            inputArray[4] = dsmeAdaptionLayer.getMAC_PIB().macShortAddress / 19;
       
            quicknet::vector_t input{5, inputArray};
            quicknet::vector_t& output = this->network.feedForward(input);
            data.slotTarget = quicknet::idmax(output);
        }
    }
}

void MLController::finish() {
    for(MLControllerData& data : this->txLinks) {
        /* log the training values to file */
        std:: cout << "{";
        std::cout << "\"time\" : " << omnetpp::simTime() << ", ";
        std::cout << "\"from\" : " <<  dsmeAdaptionLayer.getMAC_PIB().macShortAddress << ", ";
        std::cout << "\"to\" : " <<  data.address << ", ";
        std::cout << "\"rr\" : " << data.messagesInLastMultisuperframe << ", ";
        std::cout << "\"tr\" : " <<  data.transmissionRate << ", ";
        std::cout << "\"q\" : " << data.queueLevel << ", ";
        std::cout << "\"s_o\" : " << dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(data.address, Direction::TX) << ", "; 
        std::cout << "\"s_i\" : " << dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(data.address, Direction::RX) << ", ";
        std::cout << "\"slot_target\" : " <<  data.slotTarget<< "} " << std::endl;
    }
}

void MLController::doPID() {
    for(MLControllerData& data : this->txLinks) {
        uint16_t w = data.messagesInLastMultisuperframe;
        uint16_t y = data.messagesOutLastMultisuperframe;

        int16_t e = w - y;
        int16_t d = e - data.last_error;
        int16_t& i = data.error_sum;
        int16_t u;

        i += e;

        if(e > 0) {
            u = (K_P_POS * e + K_I_POS * i + K_D_POS * d) / SCALING;
        } else {
            u = (K_P_NEG * e + K_I_NEG * i + K_D_NEG * d) / SCALING;
        }

        uint16_t slots = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(data.address, Direction::TX);
        data.slotTarget = slots + u;

        if(data.slotTarget < 1) {
            data.slotTarget = 1;
        }

        LOG_DEBUG_PREFIX;
        LOG_DEBUG_PURE("Scheduling-Data->" << data.address);
        LOG_DEBUG_PURE("; w: " << (const char*)(" ") << w);
        LOG_DEBUG_PURE("; y: " << (const char*)(" ") << y);
        LOG_DEBUG_PURE("; e: " << (const char*)(e < 0 ? "" : " ") << e);
        LOG_DEBUG_PURE("; i: " << (const char*)(i < 0 ? "" : " ") << i);
        LOG_DEBUG_PURE("; d: " << (const char*)(d < 0 ? "" : " ") << d);
        LOG_DEBUG_PURE("; u: " << (const char*)(u < 0 ? "" : " ") << u);
        LOG_DEBUG_PURE(LOG_ENDL);

        data.last_error = e;
        data.messagesInLastMultisuperframe = 0;
        data.messagesOutLastMultisuperframe = 0;
    }
}

void MLController::doTPS(float alpha, uint16_t minFreshness) {
    for(MLControllerData& data : this->txLinks) {
        DSME_ASSERT(alpha > 0);
        DSME_ASSERT(minFreshness > 0);
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

        if(data.multisuperframesSinceLastPacket > minFreshness) {
            data.slotTarget = 0;
        }

        if(data.messagesInLastMultisuperframe == 0) {
            if(data.multisuperframesSinceLastPacket < 0xFFFE) {
                data.multisuperframesSinceLastPacket++;
            }
        }
        else {
            data.multisuperframesSinceLastPacket = 0;
        }

        LOG_DEBUG("control"
                  << ",0x" << HEXOUT << this->dsmeAdaptionLayer.getDSME().getMAC_PIB().macShortAddress
                  << ",0x" << data.address << "," << DECOUT << data.messagesInLastMultisuperframe
                  << "," << data.messagesOutLastMultisuperframe << "," << FLOAT_OUTPUT(data.avgIn) << "," << (uint16_t)slots << "," << data.slotTarget
                  << "," << data.multisuperframesSinceLastPacket);

        data.messagesInLastMultisuperframe = 0;
        data.messagesOutLastMultisuperframe = 0;
    }
}


//void MLController::multisuperframeEvent() {
//    global_multisuperframe++;
//
//    DSMEPlatform& platform = *dynamic_cast<DSMEPlatform*>(&(this->dsmeAdaptionLayer.getDSME().getPlatform()));
//
//    if(flipEnabled && global_multisuperframe >= platform.par("flipTime").longValue() + platform.par("evaluationDuration").longValue()) {
//        platform.endSimulation();
//    }
//
//    uint8_t flipLink{0};
//
//    if(flipEnabled) {
//        if(this->txLinks.size() > 0) {
//            uint8_t active_size = 0;
//            for(MLControllerData& data : this->txLinks) {
//                if(this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(data.address, Direction::TX) > 0) {
//                    active_size++;
//                }
//            }
//            if(active_size > 0) {
//                uint8_t link_choice = platform.par("flipLink").longValue() % active_size;
//
//                for(MLControllerData& data : this->txLinks) {
//                    if(this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(data.address, Direction::TX) > 0) {
//                        if(link_choice == 0) {
//                            break;
//                        }
//                        flipLink++;
//                        link_choice--;
//                    }
//                }
//            }
//        }
//    }
//
//    uint8_t i = 0;
//    for(MLControllerData& data : this->txLinks) {
//        data.queueingHistory[data.historyPosition] += data.messagesInLastMultisuperframe - data.messagesOutLastMultisuperframe;
//        uint16_t slots = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(data.address, Direction::TX);
//        uint16_t myAddress = this->dsmeAdaptionLayer.getMAC_PIB().macShortAddress;
//
//        if(flipEnabled && i == flipLink && myAddress == platform.par("flipNode").longValue()) {
//            std::cout << "trace: ";
//            std::cout << omnetpp::simTime() << ",";
//            std::cout << global_multisuperframe << ",";
//            std::cout << myAddress << ",";
//            std::cout << data.address << ",";
//            std::cout << data.slotTarget << ",";
//            std::cout << slots << ",";
//            std::cout << data.messagesInLastMultisuperframe << ",";
//            std::cout << data.messagesOutLastMultisuperframe << ",";
//            std::cout << data.queueingHistory[data.historyPosition] << ",";
//            std::cout << std::endl;
//        }
//
//        if(flipEnabled && i == flipLink && myAddress == platform.par("flipNode").longValue() && global_multisuperframe >= platform.par("flipTime").longValue()) {
//            if(global_multisuperframe == platform.par("flipTime").longValue()) {
//                std::cout << "state: ";
//                std::cout << myAddress << ",";
//                std::cout << data.address << ",";
//                std::cout << slots << ",";
//                std::cout << platform.par("flipOption").longValue() << ",";
//                std::cout << platform.par("evaluationDuration").longValue() << ",";
//
//                uint8_t k = data.historyPosition;
//                for(uint8_t j = 0; j < HISTORY_LENGTH; j++) {
//                    std::cout << data.queueingHistory[k] << ",";
//                    // std::cout << data.messagesIn[k] << ",";
//                    // std::cout << data.messagesOut[k] << ",";
//
//                    k++;
//                    if(k >= HISTORY_LENGTH) {
//                        k = 0;
//                    }
//                }
//                std::cout << std::endl;
//
//                data.slotTarget = platform.par("flipOption").longValue();
//            } else {
//                /* do not change control value any more */
//            }
//        } else {
//            float control_input_array[HISTORY_LENGTH + 1];
//            control_input_array[0] = slots;
//            uint8_t k = data.historyPosition;
//            for(uint8_t j = 0; j < HISTORY_LENGTH; j++) {
//                control_input_array[j + 1] = data.queueingHistory[k];
//                k++;
//                if(k >= HISTORY_LENGTH) {
//                    k = 0;
//                }
//            }
//            quicknet::vector_t input{HISTORY_LENGTH + 1, control_input_array};
//
//            /* input: slots | l0 | l1 | l2 | l3 | l4 | l5 | l6 | l7 */
//            quicknet::vector_t& output = this->network.feedForward(input);
//            /* output: 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 */
//
//            data.slotTarget = quicknet::idmax(output);
//
//            //data.control = platform.getRandom() % 9;
//
//            std::cout << "network: ";
//            std::cout << myAddress << ",";
//            std::cout << data.slotTarget << ",";
//            std::cout << std::endl;
//        }
//
//        uint16_t currentQueueSize = data.queueingHistory[data.historyPosition];
//        ++data.historyPosition;
//        if(data.historyPosition >= HISTORY_LENGTH) {
//            data.historyPosition = 0;
//        }
//        data.queueingHistory[data.historyPosition] = currentQueueSize;
//        data.messagesInLastMultisuperframe = 0;
//        data.messagesOutLastMultisuperframe = 0;
//        i++;
//    }
//}

} /* namespace dsme */
