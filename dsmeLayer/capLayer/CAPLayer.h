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

#ifndef DSMECSMA_H
#define DSMECSMA_H

#include "../../helper/DSMEFSM.h"
#include "../../helper/DSMEQueue.h"
#include "../ackLayer/AckLayer.h"

namespace dsme {

class CSMAEvent : public FSMEvent {
public:
    enum : uint8_t {
        MSG_PUSHED = USER_SIGNAL_START,
        TIMER_FIRED,
        CCA_FAILURE,
        CCA_SUCCESS,
        SEND_SUCCESSFUL,
        SEND_FAILED
    };
};

class DSMELayer;

class CAPLayer : private FSM<CAPLayer, CSMAEvent> {
public:
    CAPLayer(DSMELayer& dsme);

    bool pushMessage(DSMEMessage* msg);

    void dispatchTimerEvent();
    void dispatchCCAResult(bool success);

private:
    DSMELayer& dsme;

    uint8_t NB;
    uint8_t NR;
    uint8_t totalNBs;

    AckLayer::done_callback_t doneCallback;

    void sendDone(enum AckLayerResponse response, DSMEMessage* msg);
    void startBackoffTimer();
    bool enoughTimeLeft();
    uint16_t symbolsRequired();
    void popMessage(DataStatus::Data_Status status);

    fsmReturnStatus gateBackoffIfPending();
    fsmReturnStatus gateRetry();

    fsmReturnStatus stateIdle(CSMAEvent& event);
    fsmReturnStatus stateBackoff(CSMAEvent& event);
    fsmReturnStatus stateCCA(CSMAEvent& event);
    fsmReturnStatus stateSending(CSMAEvent& event);

    DSMEQueue<DSMEMessage*, CAP_QUEUE_SIZE> queue;
};

}

#endif
