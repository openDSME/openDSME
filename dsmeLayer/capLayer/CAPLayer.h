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

#ifndef CAPLAYER_H_
#define CAPLAYER_H_

#include "../../../dsme_settings.h"
#include "../../helper/DSMEBufferedFSM.h"
#include "../../helper/DSMEFSM.h"
#include "../../helper/DSMEQueue.h"
#include "../../helper/Integers.h"
#include "../../mac_services/DSME_Common.h"
#include "../ackLayer/AckLayer.h"

namespace dsme {

class IDSMEMessage;

class CSMAEvent : public FSMEvent {
public:
    void fill(uint16_t signal) {
        this->signal = signal;
    }

    enum : uint8_t { MSG_PUSHED = USER_SIGNAL_START, TIMER_FIRED, CCA_FAILURE, CCA_SUCCESS, SEND_SUCCESSFUL, SEND_FAILED, SEND_ABORTED };
};

class DSMELayer;

class CAPLayer : private DSMEBufferedFSM<CAPLayer, CSMAEvent, 4> {
public:
    explicit CAPLayer(DSMELayer& dsme);

    void reset();
    bool pushMessage(IDSMEMessage* msg);
    uint16_t getQueueLevel() const;
    void dispatchTimerEvent();
    void dispatchCCAResult(bool success);
    void setQAgent(bool hasQAgent) {
        this->hasQAgent = hasQAgent;
    }

private:
    /**
     * States
     */
    fsmReturnStatus stateIdle(CSMAEvent& event);
    fsmReturnStatus stateBackoff(CSMAEvent& vent);
    fsmReturnStatus stateCCA(CSMAEvent& event);
    fsmReturnStatus stateSending(CSMAEvent& event);

    /*
     * External interfaces for use through callbacks
     */
    void sendDone(AckLayerResponse response, IDSMEMessage* msg);

    /**
     * Actions
     */
    void actionStartBackoffTimer();
    void actionPopMessage(DataStatus::Data_Status);

    /**
     * Internal helper
     */
    fsmReturnStatus choiceRebackoff();
    bool enoughTimeLeft();
    uint16_t symbolsRequired();

    /*
     * Attributes
     */
    DSMELayer& dsme;
    uint8_t NB;
    uint8_t NR;
    uint8_t totalNBs;
    AckLayer::done_callback_t doneCallback;
    DSMEQueue<IDSMEMessage*, CAP_QUEUE_SIZE> queue;
    uint32_t lastWaitTime = 0;
    bool ccaFailed{false};
    bool txFailed{false};
    bool txSuccess{false};
    bool hasQAgent{false};
};

} /* namespace dsme */

#endif /* CAPLAYER_H_ */
