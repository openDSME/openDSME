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


MLControllerData::MLControllerData() : error_sum(0), last_error(0),  avgIn(0), multisuperframesSinceLastPacket(0) {
    for(int i=0; i<HISTORY_LENGTH; i++) {
        queueLevel[i] = 0;
        transmissionRate[i] = 0;
        receptionRate[i] = 0;
        slots[i] = 0;
    }
}

void MLController::multisuperframeEvent() {
    static float maxTr= 0;
    static float maxRr= 0;
    DSMEPlatform& platform = *dynamic_cast<DSMEPlatform*>(&(this->dsmeAdaptionLayer.getDSME().getPlatform()));

    uint8_t lastHistoryPosition = historyPosition;
    historyPosition = (historyPosition+1) % HISTORY_LENGTH;
    for(MLControllerData& data : this->txLinks) {
        maxRr = data.messagesInLastMultisuperframe > maxRr ? data.messagesInLastMultisuperframe : maxRr;
        maxTr = data.messagesOutLastMultisuperframe > maxTr ? data.messagesOutLastMultisuperframe : maxTr;
	    
        data.transmissionRate[historyPosition] = data.messagesOutLastMultisuperframe;
        data.receptionRate[historyPosition] = data.messagesInLastMultisuperframe;
        data.queueLevel[historyPosition] = data.queueLevel[lastHistoryPosition] + data.messagesInLastMultisuperframe - data.messagesOutLastMultisuperframe;
        if(data.queueLevel[historyPosition] > 22) data.queueLevel[historyPosition] = 22;
        data.slots[historyPosition] = dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(data.address, Direction::TX); 
    }

    
    if(platform.par("learning").boolValue()) {
       doTPS(0.1, 28);
        //doPID();
    } else {
        for(MLControllerData& data : this->txLinks) {
            float inputArray[4 * HISTORY_LENGTH];
            for(int i=0; i<HISTORY_LENGTH; i++) {
                inputArray[i] = data.transmissionRate[i] / maxTr;
                inputArray[HISTORY_LENGTH + i] = data.receptionRate[i] / maxRr;
                inputArray[2*HISTORY_LENGTH + i] = data.queueLevel[i] / 22.0f;
                inputArray[3*HISTORY_LENGTH + i] = data.slots[i] / 14.0f;
            }

            quicknet::Vector<float> input{4*HISTORY_LENGTH, inputArray};
            quicknet::Vector<float>& output = this->network.feedForward(input);
            data.slotTarget = quicknet::idmax(output);
            data.messagesInLastMultisuperframe = 0;
            data.messagesOutLastMultisuperframe = 0;
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
        std::cout << "\"s_i\" : " << dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(data.address, Direction::RX) << ", ";
        for(int i=0; i<HISTORY_LENGTH; i++) {
            uint8_t n = (historyPosition - i);
            if(n<0) n = HISTORY_LENGTH - (i - historyPosition); 
            std::cout << "\"rr" << (int)i << "\" : " << data.receptionRate[n] << ", ";
            std::cout << "\"tr" << (int)i << "\" : " <<  data.transmissionRate[n] << ", ";
            std::cout << "\"q" << (int)i << "\" : " << data.queueLevel[n] << ", ";
            std::cout << "\"s_o" << (int)i << "\" : " << data.slots[n] << ", "; 
        }
        std::cout << "\"slot_target\" : " <<  data.slotTarget << "}" << std::endl;
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

} /* namespace dsme */
