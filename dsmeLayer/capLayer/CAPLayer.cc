/*
 * openDSME
 *
 * Implementation of the Deterministic & Synchronous Multi-channel Extension (DSME)
 * described in the IEEE 802.15.4-2015 standard
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

#include "CAPLayer.h"

#include "../../../dsme_platform.h"
#include "../DSMELayer.h"

namespace dsme {

CAPLayer::CAPLayer(DSMELayer& dsme)
    : DSMEBufferedFSM<CAPLayer, CSMAEvent, 4>(&CAPLayer::stateIdle), dsme(dsme), NB(0), NR(0), totalNBs(0), doneCallback(DELEGATE(&CAPLayer::sendDone, *this)) {
}

void CAPLayer::reset() {
    transition(&CAPLayer::stateIdle);
    this->NB = 0;
    this->totalNBs = 0;
    this->NR = 0;

    while(!this->queue.empty()) {
        this->queue.pop();
    }
}

/*****************************
 * External interfaces
 *****************************/
void CAPLayer::dispatchTimerEvent() {
    bool dispatchSuccessful = dispatch(CSMAEvent::TIMER_FIRED);
    DSME_ASSERT(dispatchSuccessful);
}

void CAPLayer::dispatchCCAResult(bool success) {
    bool dispatchSuccessful = dispatch(success ? CSMAEvent::CCA_SUCCESS : CSMAEvent::CCA_FAILURE);
    DSME_ASSERT(dispatchSuccessful);
}

void CAPLayer::sendDone(AckLayerResponse response, IDSMEMessage* msg) {
    uint8_t signal;
    switch(response) {
        case AckLayerResponse::NO_ACK_REQUESTED:
        case AckLayerResponse::ACK_SUCCESSFUL:
            signal = CSMAEvent::SEND_SUCCESSFUL;
            break;
        case AckLayerResponse::ACK_FAILED:
        case AckLayerResponse::SEND_FAILED:
            signal = CSMAEvent::SEND_FAILED;
            break;
        case AckLayerResponse::SEND_ABORTED:
            signal = CSMAEvent::SEND_ABORTED;
            break;
        default:
            DSME_ASSERT(false);
            return;
    }
    bool dispatchSuccessful = dispatch(signal);
    DSME_ASSERT(dispatchSuccessful);
}

bool CAPLayer::pushMessage(IDSMEMessage* msg) {
    LOG_DEBUG("push " << (uint64_t)msg);

    bool pushed = false;

    dsme_atomicBegin();
    if(queue.full()) {
        pushed = false;
    } else {
        queue.push(msg);
        pushed = true;
    }
    dsme_atomicEnd();

    if(pushed) {
        dispatch(CSMAEvent::MSG_PUSHED);
    }

    return pushed;
}

/*****************************
 * Choices
 *****************************/
fsmReturnStatus CAPLayer::choiceRebackoff() {
    NB++;
    totalNBs++;
    if(NB > dsme.getMAC_PIB().macMaxCSMABackoffs) {
        actionPopMessage(DataStatus::CHANNEL_ACCESS_FAILURE);
        return transition(&CAPLayer::stateIdle);
    } else {
        return transition(&CAPLayer::stateBackoff);
    }
}

/*****************************
 * States
 *****************************/
fsmReturnStatus CAPLayer::stateIdle(CSMAEvent& event) {
    LOG_DEBUG_PURE("Ci" << (uint16_t)event.signal << LOG_ENDL);
    if(event.signal == CSMAEvent::ENTRY_SIGNAL) {
        NB = 0;
        NR = 0;
        totalNBs = 0;

        if(!queue.empty()) {
            return transition(&CAPLayer::stateBackoff);
        } else {
            return FSM_HANDLED;
        }
    } else if(event.signal == CSMAEvent::MSG_PUSHED) {
        LOG_INFO("A CSMA message was pushed.");
        DSME_ASSERT(!queue.empty());
        return transition(&CAPLayer::stateBackoff);
    } else {
        if(event.signal >= CSMAEvent::USER_SIGNAL_START) {
            LOG_ERROR((uint16_t)event.signal);
            DSME_ASSERT(false);
        }
        return FSM_IGNORED;
    }
}

fsmReturnStatus CAPLayer::stateBackoff(CSMAEvent& event) {
    LOG_DEBUG_PURE("Cb" << (uint16_t)event.signal << LOG_ENDL);
    if(event.signal == CSMAEvent::ENTRY_SIGNAL) {
        actionStartBackoffTimer();
        return FSM_HANDLED;
    } else if(event.signal == CSMAEvent::MSG_PUSHED) {
        return FSM_IGNORED;
    } else if(event.signal == CSMAEvent::TIMER_FIRED) {
        if(enoughTimeLeft()) {
            return transition(&CAPLayer::stateCCA);
        } else {
            DSME_SIM_ASSERT(false);
            // should only happen in rare cases (e.g. resync)
            // normally the backoff is chosen large enough beforehand
            // TODO check how often this really happens
            return transition(&CAPLayer::stateBackoff);
        }
    } else {
        if(event.signal >= CSMAEvent::USER_SIGNAL_START) {
            LOG_INFO((uint16_t)event.signal);
            DSME_ASSERT(false);
        }
        return FSM_IGNORED;
    }
}

fsmReturnStatus CAPLayer::stateCCA(CSMAEvent& event) {
    LOG_DEBUG_PURE("Cc" << (uint16_t)event.signal << LOG_ENDL);
    if(event.signal == CSMAEvent::ENTRY_SIGNAL) {
        if(!dsme.getPlatform().startCCA()) {
            return choiceRebackoff();
        } else {
            return FSM_HANDLED;
        }
    } else if(event.signal == CSMAEvent::MSG_PUSHED) {
        return FSM_IGNORED;
    } else if(event.signal == CSMAEvent::CCA_FAILURE) {
        return choiceRebackoff();
    } else if(event.signal == CSMAEvent::CCA_SUCCESS) {
        return transition(&CAPLayer::stateSending);
    } else {
        if(event.signal >= CSMAEvent::USER_SIGNAL_START) {
            LOG_INFO((uint16_t)event.signal);
            DSME_ASSERT(false);
        }
        return FSM_IGNORED;
    }
}

fsmReturnStatus CAPLayer::stateSending(CSMAEvent& event) {
    LOG_DEBUG_PURE("Cs" << (uint16_t)event.signal << LOG_ENDL);
    if(event.signal == CSMAEvent::ENTRY_SIGNAL) {
        if(!dsme.getAckLayer().prepareSendingCopy(queue.front(), doneCallback)) {
            // currently receiving external interference
            return choiceRebackoff();
        }
        dsme.getAckLayer().sendNowIfPending();

        return FSM_HANDLED;
    } else if(event.signal == CSMAEvent::MSG_PUSHED) {
        return FSM_IGNORED;
    } else if(event.signal == CSMAEvent::SEND_SUCCESSFUL) {
        actionPopMessage(DataStatus::SUCCESS);
        return transition(&CAPLayer::stateIdle);
    } else if(event.signal == CSMAEvent::SEND_FAILED) {
        /* check if a sending should by retries */
        if(NR > dsme.getMAC_PIB().macMaxFrameRetries) {
            actionPopMessage(DataStatus::Data_Status::NO_ACK);
            return transition(&CAPLayer::stateIdle);
        } else {
            NB = 0;
            NR++;
            return transition(&CAPLayer::stateBackoff);
        }
    } else if(event.signal == CSMAEvent::SEND_ABORTED) {
        actionPopMessage(DataStatus::Data_Status::TRANSACTION_EXPIRED);
        return transition(&CAPLayer::stateIdle);
    } else {
        if(event.signal >= CSMAEvent::USER_SIGNAL_START) {
            LOG_INFO((uint16_t)event.signal);
            DSME_ASSERT(false);
        }
        return FSM_IGNORED;
    }
}

/*****************************
 * Actions & Helpers
 *****************************/
uint16_t CAPLayer::symbolsRequired() {
    IDSMEMessage* msg = queue.front();
    uint16_t symbols = 0;
    symbols += 8; // CCA
    symbols += msg->getTotalSymbols();
    symbols += dsme.getMAC_PIB().helper.getAckWaitDuration(); // ACK
    symbols += 10;                                            // processing (arbitrary) TODO ! verify that the callback is always called before slots begin
    return symbols;
}

void CAPLayer::actionStartBackoffTimer() {
    uint8_t backoffExp = dsme.getMAC_PIB().macMinBE + NB;
    uint8_t maxBE = dsme.getMAC_PIB().macMaxBE;
    backoffExp = backoffExp <= maxBE ? backoffExp : maxBE;
    uint16_t unitBackoffPeriods = dsme.getPlatform().getRandom() % (1 << (uint16_t)backoffExp);

    uint16_t backoff = aUnitBackoffPeriod * (unitBackoffPeriods + 1); // +1 to avoid scheduling in the past
    uint32_t now = dsme.getPlatform().getSymbolCounter();

    uint32_t symbolsPerSlot = dsme.getMAC_PIB().helper.getSymbolsPerSlot();
    uint32_t symbolsSinceCAPStart = dsme.getSymbolsSinceSuperframeStart(now, symbolsPerSlot); // including backoff
    uint32_t backoffSinceCAPStart = symbolsSinceCAPStart + backoff;
    uint32_t CAPStart = now - symbolsSinceCAPStart;

    uint16_t blockedEnd = symbolsRequired() + PRE_EVENT_SHIFT;
    uint8_t fullCAPs = backoffSinceCAPStart / (dsme.getMAC_PIB().helper.getFinalCAPSlot() * symbolsPerSlot - blockedEnd);
    uint16_t blockedSymbolsPerSuperframe = ((dsme.getMAC_PIB().helper.getNumGTSlots() + 1) * symbolsPerSlot + blockedEnd);

    backoffSinceCAPStart += fullCAPs * blockedSymbolsPerSuperframe;

    DSME_ASSERT(CAPStart + backoffSinceCAPStart >= now + backoff);
    DSME_ASSERT(dsme.isWithinCAP(CAPStart + backoffSinceCAPStart, symbolsRequired()));

    dsme.getEventDispatcher().setupCSMATimer(CAPStart + backoffSinceCAPStart);

    LOG_DEBUG(now << " " << (CAPStart + backoffSinceCAPStart));
}

bool CAPLayer::enoughTimeLeft() {
    // TODO test
    return dsme.isWithinCAP(dsme.getPlatform().getSymbolCounter(), symbolsRequired());
}

void CAPLayer::actionPopMessage(DataStatus::Data_Status status) {
    IDSMEMessage* msg = queue.front();
    queue.pop();

    LOG_DEBUG("pop " << (uint64_t)msg);
    dsme.getMessageDispatcher().onCSMASent(msg, status, totalNBs);
}

} /* dsme */
