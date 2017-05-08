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

#include "./PIDScheduling.h"

#include "../../../dsme_platform.h"
#include "../../mac_services/dataStructures/IEEE802154MacAddress.h"
#include "../DSMEAdaptionLayer.h"
#include "../../mac_services/pib/MAC_PIB.h"

constexpr int16_t K_P_POS = 0;
constexpr int16_t K_I_POS = 30;
constexpr int16_t K_D_POS = 26;

constexpr int16_t K_P_NEG = 50;
constexpr int16_t K_I_NEG = 30;
constexpr int16_t K_D_NEG = 38;

constexpr uint16_t SCALING = 128;

namespace dsme {

PIDSchedulingData::PIDSchedulingData() : error_sum(0), last_error(0) {
}

void PIDScheduling::multisuperframeEvent() {
    for(PIDSchedulingData& data : this->links) {
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

        uint16_t slots = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedTxGTS(data.address);
        data.slotTarget = slots + u;

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

} /* namespace dsme */
