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
 * This file: 
 *          Florian Meyer <fl.meyer@tuhh.de>
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

#ifndef STATICSCHEDULING_H_
#define STATICSCHEDULING_H_

#include "./GTSScheduling.h"
#include <vector>

namespace dsme {

class DSMEAdaptionLayer;


class StaticScheduling : public GTSSchedulingImpl<GTSSchedulingData, GTSRxData> {
public:
    StaticScheduling(DSMEAdaptionLayer& dsmeAdaptionLayer) : GTSSchedulingImpl(dsmeAdaptionLayer), newMsf(false), negotiateChannels(true) {
    }

    virtual void multisuperframeEvent();
    virtual GTSSchedulingDecision getNextSchedulingAction(uint16_t address);

    void setNegotiateChannels(bool negotiateChannels);    
    void allocateGTS(uint8_t superframeID, uint8_t slotID, uint8_t channelID, Direction direction, uint16_t address);

private:
    /* save schedule and allocate slots */
    std::vector<uint8_t> superframes;
    std::vector<uint8_t> slots; 
    std::vector<uint16_t> addresses;
    bool newMsf; 
    bool negotiateChannels;
};

} /* namespace dsme */

#endif /* STATICSCHEDULING_H_ */
