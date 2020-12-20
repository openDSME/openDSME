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
    : DSMEBufferedFSM<CAPLayer, CSMAEvent, 4>(&CAPLayer::stateIdle), dsme(dsme), NB(0), NR(0), totalNBs(0), CW(CW0), batteryLifeExt(false), slottedCSMA(false), useQAgent(false), sentPackets(0), failedPackets(0), successPackets(0), failedCCAs(0), doneCallback(DELEGATE(&CAPLayer::sendDone, *this)) {
        if(!slottedCSMA) {
            batteryLifeExt = false;
        }
}

void CAPLayer::reset() {
    if(useQAgent) {
	 transition(&CAPLayer::stateQAgentEvaluation);
    } else {
	 transition(&CAPLayer::stateIdle);
    }
    this->NB = 0;
    this->totalNBs = 0;
    this->NR = 0;
    this->CW = CW0;

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

uint16_t CAPLayer::getQueueLevel() {
    return queue.getSize();
}

/*****************************
 * Choices
 *****************************/
fsmReturnStatus CAPLayer::choiceRebackoff() {
    NB++;
    CW = CW0;
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
        CW = CW0;

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
    } else if(event.signal == CSMAEvent::TIMER_FIRED) {
        return FSM_HANDLED;
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
        failedCCAs++;
        return choiceRebackoff();
    } else if(event.signal == CSMAEvent::CCA_SUCCESS) {
        if(slottedCSMA) {
            return transition(&CAPLayer::stateContention);
        }
        else {
            return transition(&CAPLayer::stateSending);
        }
    } else if(event.signal == CSMAEvent::TIMER_FIRED) {
        return FSM_HANDLED;
    } else {
        if(event.signal >= CSMAEvent::USER_SIGNAL_START) {
            LOG_INFO((uint16_t)event.signal);
            DSME_ASSERT(false);
        }
        return FSM_IGNORED;
    }
}

fsmReturnStatus CAPLayer::stateContention(CSMAEvent& event) {
    uint32_t now = this->dsme.getPlatform().getSymbolCounter();
    uint32_t symbolsSinceCapFrameStart = this->dsme.getSymbolsSinceCapFrameStart(now);
    uint32_t timerEndTime = now;

    if(event.signal == CSMAEvent::ENTRY_SIGNAL) {
        if(CW > 0) {
            timerEndTime -= symbolsSinceCapFrameStart % aUnitBackoffPeriod;
            timerEndTime += aUnitBackoffPeriod;
            this->dsme.getEventDispatcher().setupCSMATimer(timerEndTime);
            return FSM_HANDLED;
        }
        else {
            return transition(&CAPLayer::stateSending);
        }
    }
    else if(event.signal == CSMAEvent::TIMER_FIRED) {
        CW--;
        return transition(&CAPLayer::stateCCA);
    }
    else if(event.signal == CSMAEvent::MSG_PUSHED) {
        return FSM_IGNORED;
    }
    else {
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
        sentPackets++;
        return FSM_HANDLED;
    } else if(event.signal == CSMAEvent::MSG_PUSHED) {
        return FSM_IGNORED;
    } else if(event.signal == CSMAEvent::SEND_SUCCESSFUL) {
        actionPopMessage(DataStatus::SUCCESS);
	    successPackets++;
        return transition(&CAPLayer::stateIdle);
    } else if(event.signal == CSMAEvent::SEND_FAILED) {
        failedPackets++;
        /* check if a sending should by retries */
        if(NR >= dsme.getMAC_PIB().macMaxFrameRetries) {
            actionPopMessage(DataStatus::Data_Status::NO_ACK);
            return transition(&CAPLayer::stateIdle);
        } else {
            NB = 0;
            CW=CW0;
            NR++;
            return transition(&CAPLayer::stateBackoff);
        }
    } else if(event.signal == CSMAEvent::SEND_ABORTED) {
        actionPopMessage(DataStatus::Data_Status::TRANSACTION_EXPIRED);
        return transition(&CAPLayer::stateIdle);
    } else if(event.signal == CSMAEvent::TIMER_FIRED) {
        return FSM_HANDLED;
    } else {
        if(event.signal >= CSMAEvent::USER_SIGNAL_START) {
            LOG_INFO((uint16_t)event.signal);
            DSME_ASSERT(false);
        }
        return FSM_IGNORED;
    }
}

/*****************************
 * Q-States
 *****************************/

fsmReturnStatus CAPLayer::stateQAgentDecision(CSMAEvent& event) {
    if(event.signal == CSMAEvent::ENTRY_SIGNAL) {
        if(!queue.empty() && enoughTimeLeft()) {
            QAction action = (QAction)dsme.getQAgent().selectAction();
            switch(action) {
                case QAction::BACKOFF:
                    return transition(&CAPLayer::stateQAgentEvaluation);
                    return FSM_HANDLED;
                case QAction::CCA:
                    return transition(&CAPLayer::stateQAgentCCA);
                case QAction::SEND:
                    return transition(&CAPLayer::stateQAgentSending);
                default:
                    DSME_ASSERT(false);
            }
        }
        return FSM_HANDLED;
    } else if(event.signal == CSMAEvent::MSG_PUSHED) {
        return transition(&CAPLayer::stateQAgentDecision);
    } else if(event.signal == CSMAEvent::TIMER_FIRED) {
        return transition(&CAPLayer::stateQAgentDecision);
    } else {
        if(event.signal >= CSMAEvent::USER_SIGNAL_START) {
            DSME_ASSERT(false);
        }
        return FSM_IGNORED;
    }
}

fsmReturnStatus CAPLayer::stateQAgentCCA(CSMAEvent& event) {
    if(event.signal == CSMAEvent::ENTRY_SIGNAL) {
        dsme.getPlatform().startCCA();
        return FSM_HANDLED;
    } else if(event.signal == CSMAEvent::CCA_SUCCESS) {
        dsme.getQAgent().getFeatureManager().getState().getFeature<CCASuccessFeature>().update(true);
        return transition(&CAPLayer::stateQAgentSending);
    } else if(event.signal == CSMAEvent::CCA_FAILURE) {
        if(++NB >= dsme.getMAC_PIB().macMaxCSMABackoffs) {
            actionPopMessage(DataStatus::CHANNEL_ACCESS_FAILURE);
        }
        dsme.getQAgent().getFeatureManager().getState().getFeature<CCASuccessFeature>().update(false);
        return transition(&CAPLayer::stateQAgentEvaluation);
    } else if(event.signal == CSMAEvent::MSG_PUSHED) {
        return FSM_IGNORED;
    } else if(event.signal == CSMAEvent::TIMER_FIRED) {
        return transition(&CAPLayer::stateQAgentDecision);
    } else {
        if(event.signal >= CSMAEvent::USER_SIGNAL_START) {
            return transition(&CAPLayer::stateQAgentDecision);
            //DSME_ASSERT(false);
        }
        return FSM_IGNORED;
    }
}

fsmReturnStatus CAPLayer::stateQAgentSending(CSMAEvent& event) {
    if(event.signal == CSMAEvent::ENTRY_SIGNAL) {
        if(!dsme.getAckLayer().prepareSendingCopy(queue.front(), doneCallback)) {
            actionQAgentStartBackoffTimer(1);
            return FSM_HANDLED;
        }
        dsme.getAckLayer().sendNowIfPending();
        dsme.getQAgent().getFeatureManager().getState().getFeature<SentPacketsFeature>().update(1);
        return FSM_HANDLED;
    } else if(event.signal == CSMAEvent::SEND_SUCCESSFUL) {
        actionPopMessage(DataStatus::SUCCESS);
        dsme.getQAgent().getFeatureManager().getState().getFeature<SuccessFeature>().update(true);
        dsme.getQAgent().getFeatureManager().getState().getFeature<TxSuccessFeature>().update(1);
        return transition(&CAPLayer::stateQAgentEvaluation);
    } else if(event.signal == CSMAEvent::SEND_FAILED) {
        if(++NR >= dsme.getMAC_PIB().macMaxFrameRetries) {
            actionPopMessage(DataStatus::NO_ACK);
        }
        NB = 0;
        dsme.getQAgent().getFeatureManager().getState().getFeature<SuccessFeature>().update(false);
        dsme.getQAgent().getFeatureManager().getState().getFeature<TxFailedFeature>().update(1);
        return transition(&CAPLayer::stateQAgentEvaluation);
    } else if(event.signal == CSMAEvent::SEND_ABORTED) {
        if(++NR >= dsme.getMAC_PIB().macMaxFrameRetries) {
            actionPopMessage(DataStatus::NO_ACK);
        }
        NB = 0;
        dsme.getQAgent().getFeatureManager().getState().getFeature<SuccessFeature>().update(false);
        dsme.getQAgent().getFeatureManager().getState().getFeature<TxFailedFeature>().update(1);
        return transition(&CAPLayer::stateQAgentEvaluation);
    } else if(event.signal == CSMAEvent::MSG_PUSHED) {
        return FSM_IGNORED;
    } else if(event.signal == CSMAEvent::TIMER_FIRED) {
        LOG_DEBUG("Failure during TX -> backup to next subslot");
        return transition(&CAPLayer::stateQAgentDecision);
    } else {
        if(event.signal >= CSMAEvent::USER_SIGNAL_START) {
            DSME_ASSERT(false);
        }
        return FSM_IGNORED;
    }
}

fsmReturnStatus CAPLayer::stateQAgentEvaluation(CSMAEvent& event) {
    if(event.signal == CSMAEvent::ENTRY_SIGNAL) {
        dsme.getQAgent().update();
        actionQAgentStartBackoffTimer(4);
        return FSM_HANDLED;
    } else if(event.signal == CSMAEvent::MSG_PUSHED) {
        return FSM_IGNORED;
    } else if(event.signal == CSMAEvent::TIMER_FIRED) {
        return transition(&CAPLayer::stateQAgentDecision);
    } else if(event.signal == CSMAEvent::EXIT_SIGNAL) {
        dsme.getQAgent().getFeatureManager().getState().getFeature<SuccessFeature>().update(false);
        dsme.getQAgent().getFeatureManager().getState().getFeature<CCASuccessFeature>().update(false);
        return FSM_HANDLED;
    } else {
        if(event.signal >= CSMAEvent::USER_SIGNAL_START) {
            return FSM_HANDLED;
            //DSME_ASSERT(false);
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
    if(slottedCSMA)
        symbols += CW0 * aUnitBackoffPeriod; // Contention Window
    symbols += msg->getTotalSymbols();
    symbols += dsme.getMAC_PIB().helper.getAckWaitDuration(); // ACK
    symbols += 10;                                            // processing (arbitrary) TODO ! verify that the callback is always called before slots begin
    return symbols;
}

void CAPLayer::handleStartOfCFP() {
    this->dsme.getPlatform().signalPRRCAP(((double)(sentPackets - failedPackets) / sentPackets));
    this->dsme.getPlatform().signalFailedPacketsPerCAP(failedPackets);
    failedPackets = 0;
    this->dsme.getPlatform().signalPacketsPerCAP(sentPackets);
    sentPackets = 0;
    this->dsme.getPlatform().signalFailedCCAs(failedCCAs);
    failedCCAs = 0;
    this->dsme.getPlatform().signalSuccessPacketsCAP(successPackets);
    successPackets = 0;
}

void CAPLayer::setSlottedCSMA(bool slotted) {
    slottedCSMA = slotted;
}

void CAPLayer::setBLE(bool ble) {
    batteryLifeExt = ble;
}

void CAPLayer::setUseQAgent(bool useQAgent) {
    this->useQAgent = useQAgent;
    reset();
}

void CAPLayer::actionStartBackoffTimer() {
    totalNBs++;

    uint8_t backoffExp;

    if((int)this->dsme.getMAC_PIB().macMinBE < 2 || !batteryLifeExt || !slottedCSMA) {
        backoffExp = this->dsme.getMAC_PIB().macMinBE + NB;
    } else {
        backoffExp = 2 + NB;
    }

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

        uint32_t backOffTimeLeft = backoffFromCAPStart;

        // Backoff period synchronisation
        if(slottedCSMA && backOffTimeLeft % aUnitBackoffPeriod != 0) {
            backOffTimeLeft -= backOffTimeLeft % aUnitBackoffPeriod;
            backOffTimeLeft += aUnitBackoffPeriod; // To avoid scheduling earlier than initial backoff
        }

        const uint16_t superFramesPerMultiSuperframe = 1 << (this->dsme.getMAC_PIB().macMultiSuperframeOrder - this->dsme.getMAC_PIB().macSuperframeOrder);

        const uint16_t startingSuperframe = 1;
        uint16_t superframeIterator = startingSuperframe;
        while(backOffTimeLeft > usableCapPhaseLength) {
            if(!this->dsme.getMAC_PIB().macCapReduction || superframeIterator % superFramesPerMultiSuperframe == 0) {
                /* '-> this superframe contains a CAP phase */
                backOffTimeLeft -= usableCapPhaseLength;
                if(slottedCSMA) {
                    /* Resync to backoff period */
                    backOffTimeLeft -= backOffTimeLeft % aUnitBackoffPeriod;
                    backOffTimeLeft += aUnitBackoffPeriod;
                }
            }
            superframeIterator++;
        }

        const uint16_t superframesToWait = superframeIterator - startingSuperframe;
        const uint32_t superFrameDuration = this->dsme.getMAC_PIB().helper.getSymbolsPerSlot() * aNumSuperframeSlots;
        const uint32_t totalWaitTime = backOffTimeLeft + superFrameDuration * superframesToWait;

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

void CAPLayer::actionQAgentStartBackoffTimer(uint16_t unitBackoffPeriods) {
    DSME_ASSERT(unitBackoffPeriods > 0);

    uint32_t backoff = aUnitBackoffPeriod * unitBackoffPeriods;
    DSME_ATOMIC_BLOCK {
        const uint32_t now = this->dsme.getPlatform().getSymbolCounter();
        const uint32_t symbolsSinceCapFrameStart = this->dsme.getSymbolsSinceCapFrameStart(now);
        const uint32_t offsetLastSubslot = symbolsSinceCapFrameStart % backoff;
        const uint32_t timerEndTime = now + backoff - offsetLastSubslot;

        this->dsme.getEventDispatcher().setupCSMATimer(timerEndTime);
    }
}

bool CAPLayer::enoughTimeLeft() {
    return dsme.isWithinCAP(dsme.getPlatform().getSymbolCounter(), symbolsRequired());
}

void CAPLayer::actionPopMessage(DataStatus::Data_Status status) {
    IDSMEMessage* msg = queue.front();
    queue.pop();

    uint8_t transmissionAttempts = NR + 1;

    LOG_DEBUG("pop 0x" << HEXOUT << msg->getHeader().getDestAddr().getShortAddress() << DECOUT << " " << (int16_t)status << " " << (uint16_t)totalNBs << " "
                       << (uint16_t)NR << " " << (uint16_t)NB << " " << (uint16_t)transmissionAttempts);
    dsme.getMessageDispatcher().onCSMASent(msg, status, totalNBs, transmissionAttempts);
    dsme.getPlatform().signalCSMAResult(status == DataStatus::SUCCESS, NR+1, NB+1);

    if(useQAgent) {
    	NR = 0;
    	NB = 0;
    }
}

} /* namespace dsme */
