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

#ifndef DAS_H_
#define DAS_H_

#include "./GTSScheduling.h"

namespace dsme {

class DSMEAdaptionLayer;

struct DASTxData : GTSSchedulingData {  // Werte die f�r tx link gespeichert werdenm avg hier berechnen
    DASTxData();
    int pastQValues[3]{0};
    int messagesInLastMultisuperframe;
    int incomingPacketsHistory[3]{0};
    float avgIn; // TODO no float!
    int dynamicSlots{1};
};

class DAS : public GTSSchedulingImpl<DASTxData, GTSRxData> {
public:
    DAS(DSMEAdaptionLayer& dsmeAdaptionLayer) : GTSSchedulingImpl(dsmeAdaptionLayer), alpha{0} {
    }

    virtual void multisuperframeEvent() override;
    // virtual uint8_t registerIncomingMessage(uint16_t address) override;
    void incCounter();
    void setAlpha(float alpha);
    void setUseMultiplePacketsPerGTS(bool useMultiplePackets);
    virtual GTSSchedulingDecision getNextSchedulingAction(uint16_t address) override;
    virtual GTSSchedulingDecision getNextSchedulingActionRx(uint8_t prefSF) override;

private:
    float alpha = 0.5; // paramater f�r packetallocation zwecks Q abbau
    bool useMultiplePacketsPerGTS{true};
    uint8_t counter = 0;
};

} /* namespace dsme */

#endif /* TPS_H_ */
