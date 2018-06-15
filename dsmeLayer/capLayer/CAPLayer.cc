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

#include "./CAPLayer.h"

#include "../../../dsme_platform.h"
#include "../../helper/DSMEAtomic.h"
#include "../../helper/DSMEDelegate.h"
#include "../../interfaces/IDSMEMessage.h"
#include "../../interfaces/IDSMEPlatform.h"
#include "../../mac_services/pib/MAC_PIB.h"
#include "../../mac_services/pib/PIBHelper.h"
#include "../../mac_services/pib/dsme_mac_constants.h"
#include "../DSMEEventDispatcher.h"
#include "../DSMELayer.h"
#include "../messageDispatcher/MessageDispatcher.h"

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
        actionPopMessage(DataStatus::Data_Status::TRANSACTION_EXPIRED);
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
    LOG_DEBUG("push");

    bool pushed = false;

    DSME_ATOMIC_BLOCK {
        if(this->queue.full()) {
            pushed = false;
        } else {
            this->queue.push(msg);
            pushed = true;
        }
    }

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
    } else if(event.signal == CSMAEvent::CCA_SUCCESS || event.signal == CSMAEvent::CCA_FAILURE) {
        /* '-> only possible after reset */
        return FSM_IGNORED;
    } else if(event.signal == CSMAEvent::SEND_ABORTED) {
        // After a RESET, events might arrive for
        // messages already signaled as expired to upper layer
        DSME_ASSERT(this->queue.empty());
        return FSM_IGNORED;
    } else {
        if(event.signal >= CSMAEvent::USER_SIGNAL_START) {
            LOG_ERROR((uint16_t)event.signal);
            DSME_ASSERT(false);
        }
        return FSM_IGNORED;
    }
}

fsmReturnStatus CAPLayer::stateBackoff(CSMAEvent& event) {
    if(event.signal == CSMAEvent::ENTRY_SIGNAL) {
        actionStartBackoffTimer();
        return FSM_HANDLED;
    } else if(event.signal == CSMAEvent::MSG_PUSHED) {
        return FSM_IGNORED;
    } else if(event.signal == CSMAEvent::TIMER_FIRED) {
        if(enoughTimeLeft()) {
            return transition(&CAPLayer::stateCCA);
        } else {
            // This only happens in rare cases (e.g. resync).
            // Normally the backoff is chosen large enough beforehand.
            // It also can occur in simulation when sending Beacon Requests
            // and a beacon is heared before the message is sent.
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
        if(NR >= dsme.getMAC_PIB().macMaxFrameRetries) {
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
    totalNBs++;

    uint8_t backoffExp = this->dsme.getMAC_PIB().macMinBE + NB;
    const uint8_t maxBE = this->dsme.getMAC_PIB().macMaxBE;
    backoffExp = backoffExp <= maxBE ? backoffExp : maxBE;

    const uint16_t unitBackoffPeriods = this->dsme.getPlatform().getRandom() % (1 << (uint16_t)backoffExp);

    const uint16_t backoff = aUnitBackoffPeriod * (unitBackoffPeriods + 1); // +1 to avoid scheduling in the past
    const uint32_t symbolsPerSlot = this->dsme.getMAC_PIB().helper.getSymbolsPerSlot();
    const uint16_t blockedEnd = symbolsRequired() + PRE_EVENT_SHIFT;
    const uint32_t capPhaseLength = dsme.getMAC_PIB().helper.getFinalCAPSlot(0) * symbolsPerSlot;
    const uint32_t usableCapPhaseLength = capPhaseLength - blockedEnd;
    const uint32_t usableCapPhaseEnd = usableCapPhaseLength + symbolsPerSlot;

    DSME_ATOMIC_BLOCK {
        const uint32_t now = this->dsme.getPlatform().getSymbolCounter();
        const uint32_t symbolsSinceCapFrameStart = this->dsme.getSymbolsSinceCapFrameStart(now);
        const uint32_t CAPStart = now + symbolsPerSlot - symbolsSinceCapFrameStart;

        uint32_t backoffFromCAPStart;
        if(symbolsSinceCapFrameStart < symbolsPerSlot) {
            /* '-> currently in beacon slot before CAP */
            backoffFromCAPStart = backoff;

        } else if(symbolsSinceCapFrameStart < usableCapPhaseEnd) {
            /* '-> currently inside CAP */
            backoffFromCAPStart = backoff + symbolsSinceCapFrameStart - symbolsPerSlot;

        } else {
            /* '-> after CAP */
            backoffFromCAPStart = backoff + usableCapPhaseLength;
        }

        uint32_t backOfTimeLeft = backoffFromCAPStart;

        const uint16_t superFramesPerMultiSuperframe = 1 << (this->dsme.getMAC_PIB().macMultiSuperframeOrder - this->dsme.getMAC_PIB().macSuperframeOrder);

        const uint16_t startingSuperframe = 1;
        uint16_t superframeIterator = startingSuperframe;
        while(backOfTimeLeft > usableCapPhaseLength) {
            if(!this->dsme.getMAC_PIB().macCapReduction || superframeIterator % superFramesPerMultiSuperframe == 0) {
                /* '-> this superframe contains a CAP phase */
                backOfTimeLeft -= usableCapPhaseLength;
            }
            superframeIterator++;
        }

        const uint16_t superframesToWait = superframeIterator - startingSuperframe;
        const uint32_t superFrameDuration = this->dsme.getMAC_PIB().helper.getSymbolsPerSlot() * aNumSuperframeSlots;
        const uint32_t totalWaitTime = backOfTimeLeft + superFrameDuration * superframesToWait;

        const uint32_t timerEndTime = CAPStart + totalWaitTime;

        DSME_ASSERT(timerEndTime >= now + backoff);
        if(!this->dsme.isWithinCAP(timerEndTime, symbolsRequired())) {
            LOG_INFO("Not within CAP-phase in superframe " << dsme.getCurrentSuperframe());
            LOG_ERROR("now: " << now << ", CAPStart: " << CAPStart << ", totalWaitTime: " << totalWaitTime << ", symbolsRequired: " << symbolsRequired());
            DSME_ASSERT(false);
        }

        this->dsme.getEventDispatcher().setupCSMATimer(timerEndTime);
    }
}

bool CAPLayer::enoughTimeLeft() {
    return dsme.isWithinCAP(dsme.getPlatform().getSymbolCounter(), symbolsRequired());
}

void CAPLayer::actionPopMessage(DataStatus::Data_Status status) {
    IDSMEMessage* msg = queue.front();
    queue.pop();

    uint8_t transmissionAttempts = NR+1;

    LOG_DEBUG("pop 0x" << HEXOUT << msg->getHeader().getDestAddr().getShortAddress() << DECOUT << " " << (int16_t)status << " " << (uint16_t)totalNBs << " " << (uint16_t)NR << " " << (uint16_t)NB << " " << (uint16_t)transmissionAttempts);
    dsme.getMessageDispatcher().onCSMASent(msg, status, totalNBs, transmissionAttempts);
}

} /* namespace dsme */
