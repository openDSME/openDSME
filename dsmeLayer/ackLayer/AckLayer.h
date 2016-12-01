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

#ifndef ACKLAYER_H
#define ACKLAYER_H

#include "../../helper/DSMEDelegate.h"
#include "../../helper/DSMEFSM.h"

namespace dsme {

enum AckLayerResponse {
    SEND_FAILED,
    NO_ACK_REQUESTED,
    ACK_FAILED,
    ACK_SUCCESSFUL
};

class DSMEMessage;
class DSMELayer;

class AckEvent : public FSMEvent {
public:
    enum : uint8_t {
        SEND_REQUEST = USER_SIGNAL_START,
        RECEIVE_REQUEST,
        SEND_DONE,
        TIMER_FIRED,
        ACK_RECEIVED,
    };

    bool success; // only valid for SEND_DONE
    uint8_t seqNum; // only valid for ACK_RECEIVED
};

class AckLayer : private FSM<AckLayer, AckEvent> {
public:
    typedef Delegate<void(enum AckLayerResponse, DSMEMessage* msg)> done_callback_t;

    explicit AckLayer(DSMELayer &dsme);

    void reset();

    /**
     * Only waits for an acknowledgment if the AR bit in the frame control field is set.
     * @return true, if message was accepted and the doneCallback is pending
     */
    bool sendButKeep(DSMEMessage* msg, done_callback_t doneCallback);
    void sendAdditionalAck(uint8_t seqNum);
    void receive(DSMEMessage* msg);
    void dispatchTimer();
    void setNextSequenceNumber(uint8_t nextSequenceNumber);

private:
    void sendDone(bool success);
    fsmReturnStatus stateIdle(AckEvent& event);
    fsmReturnStatus stateTx(AckEvent& event);
    fsmReturnStatus stateWaitForAck(AckEvent& event);
    fsmReturnStatus stateTxAck(AckEvent& event);
    fsmReturnStatus transitionToIdle();
    fsmReturnStatus catchAll(AckEvent& event);

    DSMELayer& dsme;

    // Indicates if the layer is currently busy (i.e. in another state than idle)
    bool busy;

    DSMEMessage* pendingMessage;
    uint8_t pendingSequenceNumber;

    uint8_t ackSeqNum;
    uint8_t nextSequenceNumber;

    done_callback_t externalDoneCallback;
    Delegate<void(bool)> internalDoneCallback;
};

}

#endif
