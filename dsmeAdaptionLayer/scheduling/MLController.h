/*
 * openDSME
 *
 * Implementation of the Deterministic & Synchronous Multi-channel Extension (DSME)
 * introduced in the IEEE 802.15.4e-2012 standard
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

#ifndef GTSCONTROLLER_H_
#define GTSCONTROLLER_H_

#include "GTSScheduling.h"
#include "../NeuralNetwork.h"
#include "../DSMEAdaptionLayer.h"

namespace dsme {

class DSMEAdaptionLayer;

uint8_t const HISTORY_LENGTH = 7;

struct MLControllerData : GTSSchedulingData {
    MLControllerData();

    int16_t error_sum;
    int16_t last_error;
    uint16_t queueLevel[HISTORY_LENGTH];
    uint16_t transmissionRate[HISTORY_LENGTH];
    uint16_t receptionRate[HISTORY_LENGTH];
    uint16_t slots[HISTORY_LENGTH];
    float avgIn; // TODO no float!
    uint16_t multisuperframesSinceLastPacket;
};

class MLController : public GTSSchedulingImpl<MLControllerData, GTSRxData> {
public:
    MLController(DSMEAdaptionLayer& dsmeAdaptionLayer) : GTSSchedulingImpl(dsmeAdaptionLayer) {
    }

    virtual void multisuperframeEvent();
    void finish();   
 
private:
    NeuralNetwork<float> network;
    uint8_t historyPosition;

    void doPID();
    void doTPS(float alpha, uint16_t minFreshness);
};

} /* namespace dsme */

#endif /* GTSCONTROLLER_H_ */
