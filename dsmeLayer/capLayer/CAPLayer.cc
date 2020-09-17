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

uint16_t CAPLayer::getQueueLevel() {
    return queue.getSize();
}

/*****************************
 * States
 *****************************/
fsmReturnStatus CAPLayer::stateIdle(CSMAEvent& event) {
    if(event.signal == CSMAEvent::ENTRY_SIGNAL) {
        return FSM_HANDLED;
    } else if(event.signal == CSMAEvent::MSG_PUSHED) {
        /*if(queue.front()->getHeader().getFrameType() == IEEE802154eMACHeader::FrameType::COMMAND) {
            actionStartBackoffTimerRandom();
        }*/
        return FSM_IGNORED;
    } else if(event.signal == CSMAEvent::TIMER_FIRED) {
        /*if(!queue.empty() && queue.front()->getHeader().getFrameType() == IEEE802154eMACHeader::FrameType::COMMAND) {
            return transition(&CAPLayer::stateCCA);
        }*/
        return transition(&CAPLayer::stateQAgentEvaluation);
    } else if(event.signal == CSMAEvent::EXIT_SIGNAL) {
        //if(!queue.empty() && queue.front()->getHeader().getFrameType() != IEEE802154eMACHeader::FrameType::COMMAND) {
            if(dsme.isWithinCAP(dsme.getPlatform().getSymbolCounter(), 16 * aUnitBackoffPeriod + PRE_EVENT_SHIFT)) {
                LOG_INFO("CL: Starting timer");
                actionStartBackoffTimer(16);
            }
        //}
        return FSM_HANDLED;
    } else {
        if(event.signal >= CSMAEvent::USER_SIGNAL_START) {
            DSME_ASSERT(false);
        }
        return FSM_IGNORED;
    }
}

fsmReturnStatus CAPLayer::stateQAgentDecision(CSMAEvent& event) {
    if(event.signal == CSMAEvent::ENTRY_SIGNAL) {
        QAction action = (QAction)dsme.getQAgent().selectAction();
        switch(action) {
            case QAction::BACKOFF:
                return transition(&CAPLayer::stateIdle);
            case QAction::CCA:
                return transition(&CAPLayer::stateCCA);
            case QAction::SEND:
                return transition(&CAPLayer::stateSending);
            default:
                DSME_ASSERT(false);
        }
    } else if(event.signal == CSMAEvent::MSG_PUSHED) {
        return FSM_IGNORED;
    } else if(event.signal == CSMAEvent::TIMER_FIRED) {
        LOG_ERROR("CL: QAgent could not make a decision within subslot");
        DSME_ASSERT(false);
    } else {
        if(event.signal >= CSMAEvent::USER_SIGNAL_START) {
            DSME_ASSERT(false);
        }
        return FSM_IGNORED;
    }
}

fsmReturnStatus CAPLayer::stateCCA(CSMAEvent& event) {
    if(event.signal == CSMAEvent::ENTRY_SIGNAL) {
        if(queue.empty() || !enoughTimeLeft() || !dsme.getPlatform().startCCA()) {
            return transition(&CAPLayer::stateIdle);
        }
        return FSM_HANDLED;
    } else if(event.signal == CSMAEvent::CCA_SUCCESS) {
        dsme.getQAgent().getFeatureManager().getState().getFeature<CCASuccessFeature>().update(true);
        return transition(&CAPLayer::stateSending);
    } else if(event.signal == CSMAEvent::CCA_FAILURE) {
        if(++NB >= dsme.getMAC_PIB().macMaxCSMABackoffs) {
            actionPopMessage(DataStatus::CHANNEL_ACCESS_FAILURE);
        }
        dsme.getQAgent().getFeatureManager().getState().getFeature<CCASuccessFeature>().update(false);
        return transition(&CAPLayer::stateIdle);
    } else if(event.signal == CSMAEvent::MSG_PUSHED) {
        return FSM_IGNORED;
    } else if(event.signal == CSMAEvent::TIMER_FIRED) {
        LOG_ERROR("CL: CCA did not finish within subslot");
        DSME_ASSERT(false);
        return FSM_IGNORED;
    } else {
        if(event.signal >= CSMAEvent::USER_SIGNAL_START) {
            DSME_ASSERT(false);
        }
        return FSM_IGNORED;
    }
}

fsmReturnStatus CAPLayer::stateSending(CSMAEvent& event) {
    if(event.signal == CSMAEvent::ENTRY_SIGNAL) {
        if(queue.empty() || !enoughTimeLeft() || !dsme.getAckLayer().prepareSendingCopy(queue.front(), doneCallback)) {
            LOG_DEBUG("CL: Failed to prepare sending copy");
            return transition(&CAPLayer::stateIdle);
        }
        dsme.getAckLayer().sendNowIfPending();
        dsme.getQAgent().getFeatureManager().getState().getFeature<SentPacketsFeature>().update(1);
        return FSM_HANDLED;
    } else if(event.signal == CSMAEvent::SEND_SUCCESSFUL) {
        actionPopMessage(DataStatus::SUCCESS);
        dsme.getQAgent().getFeatureManager().getState().getFeature<SuccessFeature>().update(true);
        dsme.getQAgent().getFeatureManager().getState().getFeature<TxSuccessFeature>().update(1);
        return transition(&CAPLayer::stateIdle);
    } else if(event.signal == CSMAEvent::SEND_FAILED) {
        if(++NR >= dsme.getMAC_PIB().macMaxFrameRetries) {
            actionPopMessage(DataStatus::NO_ACK);
        }
        NB = 0;
        dsme.getQAgent().getFeatureManager().getState().getFeature<SuccessFeature>().update(false);
        dsme.getQAgent().getFeatureManager().getState().getFeature<TxFailedFeature>().update(1);
        return transition(&CAPLayer::stateIdle);
    } else if(event.signal == CSMAEvent::SEND_ABORTED) {
        if(++NR >= dsme.getMAC_PIB().macMaxFrameRetries) {
            actionPopMessage(DataStatus::NO_ACK);
        }
        NB = 0;
        dsme.getQAgent().getFeatureManager().getState().getFeature<SuccessFeature>().update(false);
        dsme.getQAgent().getFeatureManager().getState().getFeature<TxFailedFeature>().update(1);
        return transition(&CAPLayer::stateIdle);
    } else if(event.signal == CSMAEvent::MSG_PUSHED) {
        return FSM_IGNORED;
    } else if(event.signal == CSMAEvent::TIMER_FIRED) {
        LOG_ERROR("CL: Transmission did not finish within subslot");
        DSME_ASSERT(false);
        /*if(dsme.isWithinCAP(dsme.getPlatform().getSymbolCounter(), 8 * aUnitBackoffPeriod + PRE_EVENT_SHIFT)) {
            LOG_INFO("CL: Starting timer");
            actionStartBackoffTimer(8);
        }*/
        return FSM_HANDLED;
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
        return transition(&CAPLayer::stateQAgentDecision);
    } else if(event.signal == CSMAEvent::MSG_PUSHED) {
        return FSM_IGNORED;
    } else if(event.signal == CSMAEvent::TIMER_FIRED) {
        LOG_ERROR("CL: Evaluating action and updating QAgent did not finish within subslot");
        DSME_ASSERT(false);
    } else if(event.signal == CSMAEvent::EXIT_SIGNAL) {
        dsme.getQAgent().getFeatureManager().getState().getFeature<SuccessFeature>().update(false);
        dsme.getQAgent().getFeatureManager().getState().getFeature<CCASuccessFeature>().update(false);
        return FSM_HANDLED;
    } else {
        if(event.signal >= CSMAEvent::USER_SIGNAL_START) {
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

void CAPLayer::actionStartBackoffTimer(uint16_t unitBackoffPeriods) {
    DSME_ASSERT(unitBackoffPeriods > 0);

    uint32_t backoff = aUnitBackoffPeriod * unitBackoffPeriods;
    DSME_ATOMIC_BLOCK {
        const uint32_t now = this->dsme.getPlatform().getSymbolCounter();
        const uint32_t timerEndTime = now + backoff;
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

    NR = 0;
    NB = 0;
}

} /* namespace dsme */
