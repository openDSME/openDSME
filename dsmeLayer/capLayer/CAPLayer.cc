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
: FSM<CAPLayer,CSMAEvent>(&CAPLayer::stateIdle), dsme(dsme), doneCallback(DELEGATE(&CAPLayer::sendDone,*this)) {
}

void CAPLayer::dispatchTimerEvent() {
    CSMAEvent e;
    e.signal = CSMAEvent::TIMER_FIRED;
    dispatch(e);
}

void CAPLayer::dispatchCCAResult(bool success) {
    CSMAEvent e;
    if(success) {
        e.signal = CSMAEvent::CCA_SUCCESS;
    }
    else {
        e.signal = CSMAEvent::CCA_FAILURE;
    }
    dispatch(e);
}

void CAPLayer::sendDone(enum AckLayerResponse response, DSMEMessage* msg) {
    CSMAEvent e;

    switch(response) {
    case AckLayerResponse::NO_ACK_REQUESTED:
    case AckLayerResponse::ACK_SUCCESSFUL:
        e.signal = CSMAEvent::SEND_SUCCESSFUL;
        break;
    case AckLayerResponse::ACK_FAILED:
    case AckLayerResponse::SEND_FAILED:
        e.signal = CSMAEvent::SEND_FAILED;
        break;
    default:
        DSME_ASSERT(false);
    }

    dispatch(e);
}

bool CAPLayer::pushMessage(DSMEMessage* msg)
{
    LOG_DEBUG("pushMessage " << (uint64_t)msg);

    if(queue.full()) {
        return false;
    }
    else {
        queue.push(msg);
        CSMAEvent e;
        e.signal = CSMAEvent::MSG_PUSHED;
        dispatch(e);
        return true;
    }
}

uint16_t CAPLayer::symbolsRequired() {
    DSMEMessage* msg = queue.front();
    uint16_t symbols = 0;
    symbols += 8; // CCA
    symbols += msg->getTotalSymbols();
    symbols += dsme.getMAC_PIB().macAckWaitDuration; // ACK
    symbols += 10; // processing (arbitrary) TODO ! verify that the callback is always called before slots begin
    return symbols;
}

void CAPLayer::startBackoffTimer() {
    uint8_t backoffExp = dsme.getMAC_PIB().macMinBE + NB;
    uint8_t maxBE = dsme.getMAC_PIB().macMaxBE;
    backoffExp = backoffExp <= maxBE ? backoffExp : maxBE;
    uint16_t unitBackoffPeriods = dsme.getPlatform().getRandom() % (1 << (uint16_t)backoffExp);

    uint16_t backoff = aUnitBackoffPeriod*(unitBackoffPeriods + 1); // +1 to avoid scheduling in the past
    uint32_t now = dsme.getPlatform().getSymbolCounter();

    uint16_t symbolsSinceCAPStart = dsme.getSymbolsSinceSuperframeStart(now, dsme.getMAC_PIB().helper.getSymbolsPerSlot()); // including backoff
    uint16_t backoffSinceCAPStart = symbolsSinceCAPStart + backoff;
    uint32_t CAPStart = now - symbolsSinceCAPStart;

    uint16_t blockedEnd = symbolsRequired() + PRE_EVENT_SHIFT;
    uint8_t fullCAPs = backoffSinceCAPStart/(dsme.getMAC_PIB().helper.getFinalCAPSlot() * dsme.getMAC_PIB().helper.getSymbolsPerSlot() - blockedEnd);
    uint16_t blockedSymbolsPerSuperframe = ((dsme.getMAC_PIB().helper.getNumGTSlots()+1) * dsme.getMAC_PIB().helper.getSymbolsPerSlot() + blockedEnd);

    backoffSinceCAPStart += fullCAPs * blockedSymbolsPerSuperframe;

    DSME_ASSERT(CAPStart+backoffSinceCAPStart >= now+backoff);
    DSME_ASSERT(dsme.isWithinCAP(CAPStart+backoffSinceCAPStart, symbolsRequired()));

    dsme.getEventDispatcher().setupCSMATimer(CAPStart+backoffSinceCAPStart);
}

bool CAPLayer::enoughTimeLeft() {
    // TODO test
    return dsme.isWithinCAP(dsme.getPlatform().getSymbolCounter(), symbolsRequired());
}

void CAPLayer::popMessage(DataStatus::Data_Status status) {
    DSMEMessage* msg = queue.front();
    queue.pop();

    LOG_DEBUG("popMessage " << (uint64_t)msg);
    dsme.getMessageDispatcher().onCSMASent(msg, status, totalNBs);
}

fsmReturnStatus CAPLayer::gateBackoffIfPending() {
    if(queue.empty()) {
        return transition(&CAPLayer::stateIdle);
    }
    else {
        NB = 0;
        NR = 0;
        totalNBs = 0;
        BE = dsme.getMAC_PIB().macMinBE;
        startBackoffTimer();
        return transition(&CAPLayer::stateBackoff);
    }
}

fsmReturnStatus CAPLayer::stateIdle(CSMAEvent& event) {
    if(event.signal == CSMAEvent::MSG_PUSHED) {
        LOG_INFO("A CSMA message was pushed.");
        return gateBackoffIfPending();
    }
    else {
        return FSM_IGNORED;
    }
}

fsmReturnStatus CAPLayer::stateBackoff(CSMAEvent& event) {
    if(event.signal == CSMAEvent::TIMER_FIRED) {
        if(enoughTimeLeft()) {
            dsme.getPlatform().startCCA();
            return transition(&CAPLayer::stateCCA);
        }
        else {
            // should only happen in rare cases (e.g. resync)
            // normally the backoff is chosen large enough beforehand
            // TODO test if this is the case
            DSME_ASSERT(false);
            startBackoffTimer();
            return transition(&CAPLayer::stateBackoff);
        }
    }
    else {
        return FSM_IGNORED;
    }
}

fsmReturnStatus CAPLayer::stateCCA(CSMAEvent& event) {
    if(event.signal == CSMAEvent::CCA_FAILURE) {
        NB++;
        totalNBs++;
        uint8_t macMaxBE = dsme.getMAC_PIB().macMaxBE;
        if(BE+1 < macMaxBE) {
            BE = BE+1;
        }
        else {
            BE = macMaxBE;
        }

        if(NB > dsme.getMAC_PIB().macMaxCSMABackoffs) {
            popMessage(DataStatus::CHANNEL_ACCESS_FAILURE);
            return gateBackoffIfPending();
        }
        else {
            startBackoffTimer();
            return transition(&CAPLayer::stateBackoff);
        }
    }
    else if(event.signal == CSMAEvent::CCA_SUCCESS) {
        //LOG_INFO("CSMA msg sent")
        if(!dsme.getAckLayer().sendButKeep(queue.front(),doneCallback)) {
            // message could not be sent (another transmission or reception is active)
            // should only happen in rare cases (e.g. resync)

            // TODO check how often this really happens
            return gateRetry();
        }
        else {
            return transition(&CAPLayer::stateSending);
        }
    }
    else {
        return FSM_IGNORED;
    }
}

fsmReturnStatus CAPLayer::stateSending(CSMAEvent& event) {
    if(event.signal == CSMAEvent::SEND_SUCCESSFUL) {
        popMessage(DataStatus::SUCCESS);
        return gateBackoffIfPending();
    }
    else if(event.signal == CSMAEvent::SEND_FAILED) {
        return gateRetry();
    }
    else {
        return FSM_IGNORED;
    }
}

fsmReturnStatus CAPLayer::gateRetry() {
    if(NR > dsme.getMAC_PIB().macMaxFrameRetries) {
        popMessage(DataStatus::Data_Status::NO_ACK);
        return gateBackoffIfPending();
    }
    else {
        NB = 0;
        NR++;
        BE = dsme.getMAC_PIB().macMinBE;
        startBackoffTimer();
        return transition(&CAPLayer::stateBackoff);
    }
}

}
