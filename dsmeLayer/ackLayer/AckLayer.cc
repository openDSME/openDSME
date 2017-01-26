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

#include "AckLayer.h"

#include "../../../dsme_platform.h"
#include "../DSMELayer.h"

namespace dsme {

AckLayer::AckLayer(DSMELayer& dsme)
    : FSM<AckLayer, AckEvent>(&AckLayer::stateIdle),
      dsme(dsme),
      busy(false),
      pendingMessage(nullptr),
      internalDoneCallback(DELEGATE(&AckLayer::sendDone, *this)) {
}

void AckLayer::reset() {
    ASSERT(!isDispatchBusy());
    AckEvent e;
    e.signal = AckEvent::RESET;
    dispatch(e);
}

bool AckLayer::prepareSendingCopy(DSMEMessage* msg, done_callback_t doneCallback) {
    dsme_atomicBegin();
    if(busy) {
        dsme_atomicEnd();
        return false;
    }
    busy = true;
    dsme_atomicEnd();

    DSME_ASSERT(pendingMessage == nullptr);

    this->pendingMessage = msg;
    this->externalDoneCallback = doneCallback;
    AckEvent e;
    e.signal = AckEvent::PREPARE_SEND_REQUEST;
    dispatch(e);
    return true;
}

void AckLayer::sendNowIfPending() {
    if(this->pendingMessage) {
        DSME_ASSERT(busy);
        AckEvent e;
        e.signal = AckEvent::START_TRANSMISSION;
        dispatch(e);
    }
}

void AckLayer::abortPreparedTransmission() {
    DSME_ASSERT(this->pendingMessage);

    AckEvent e;
    e.signal = AckEvent::ABORT_TRANSMISSION;
    dispatch(e);
}

void AckLayer::receive(DSMEMessage* msg) {
    IEEE802154eMACHeader& header = msg->getHeader();

    /*
     * TODO
     * GTS allocations should also be heard before the association
     * Better would be a mechanism to collect used slots after the association
    if(this->dsme.getBeaconManager().isScanning() && header.getFrameType() != IEEE802154eMACHeader::BEACON) {
        dsme.getPlatform().releaseMessage(msg);
        return;
    }
    */

    /* if message is an ACK, directly dispatch the event */
    if(header.getFrameType() == IEEE802154eMACHeader::ACKNOWLEDGEMENT) {
        LOG_DEBUG("ACK_RECEIVED with seq num " << (uint16_t)header.getSequenceNumber());
        AckEvent e;
        e.signal = AckEvent::ACK_RECEIVED;
        e.seqNum = header.getSequenceNumber();
        dsme.getPlatform().releaseMessage(msg);
        dispatch(e);
        return;
    }

    /* filter messages not for this device */
    bool throwawayMessage = false;
    if(this->dsme.getMAC_PIB().macAssociatedPANCoord && header.hasDestinationPANId() && header.getDstPANId() != this->dsme.getMAC_PIB().macPANId &&
       header.getDstPANId() != IEEE802154eMACHeader::BROADCAST_PAN) {
        LOG_DEBUG("Mismatching PAN-ID: " << header.getDstPANId() << " instead of " << this->dsme.getMAC_PIB().macPANId << " from "
                                         << header.getSrcAddr().getShortAddress());
        throwawayMessage = true;
    } else if(!header.getDestAddr().isBroadcast()) {
        if(header.getDstAddrMode() == SHORT_ADDRESS && header.getDestAddr().getShortAddress() != this->dsme.getMAC_PIB().macShortAddress) {
            throwawayMessage = true;
        } else if(header.getDstAddrMode() == EXTENDED_ADDRESS && header.getDestAddr() != this->dsme.getMAC_PIB().macExtendedAddress) {
            throwawayMessage = true;
        }
    }

    if(throwawayMessage) {
        this->dsme.getPlatform().releaseMessage(msg);
        return;
    }

    /* also throw away the packet if the FSM is busy */
    bool wasBusy;
    dsme_atomicBegin();
    if(busy) {
        wasBusy = true;
        LOG_DEBUG("Throwing away packet, ACKLayer was busy.");
        // DSME_SIM_ASSERT(false);
    } else {
        wasBusy = false;
        busy = true;
    }
    dsme_atomicEnd();

    if(wasBusy) {
        this->dsme.getPlatform().releaseMessage(msg);
        return;
    }

    this->pendingMessage = msg;
    AckEvent e;
    e.signal = AckEvent::RECEIVE_REQUEST;
    dispatch(e);

    return;
}

void AckLayer::dispatchTimer() {
    AckEvent e;
    e.signal = AckEvent::TIMER_FIRED;
    dispatch(e);
}

void AckLayer::sendDone(bool success) {
    AckEvent e;
    e.signal = AckEvent::SEND_DONE;
    e.success = success;
    dispatch(e);
}

//////////////////////////////// STATES ////////////////////////////////

fsmReturnStatus AckLayer::catchAll(AckEvent& event) {
    switch(event.signal) {
        case AckEvent::ACK_RECEIVED:
        case AckEvent::ENTRY_SIGNAL:
        case AckEvent::EXIT_SIGNAL:
            return FSM_IGNORED;
        default:
            DSME_ASSERT(false);
            return FSM_IGNORED;
    }
}

fsmReturnStatus AckLayer::stateIdle(AckEvent& event) {
    switch(event.signal) {
        case AckEvent::ENTRY_SIGNAL:
            pendingMessage = nullptr;
            dsme_atomicBegin();
            busy = false;
            dsme_atomicEnd();
            return FSM_HANDLED;

        case AckEvent::RESET:
            return FSM_HANDLED;

        case AckEvent::PREPARE_SEND_REQUEST: {
            if(pendingMessage->getHeader().hasSequenceNumber()) {
                pendingMessage->getHeader().setSequenceNumber(this->dsme.getMAC_PIB().macDsn++);
            }

            if(dsme.getPlatform().prepareSendingCopy(pendingMessage, internalDoneCallback)) {
                return transition(&AckLayer::statePreparingTx);
            } else {
                /* '-> currently busy (e.g. recent reception) */
                externalDoneCallback(SEND_FAILED, pendingMessage);
                pendingMessage = nullptr; // owned by upper layer now
                dsme_atomicBegin();
                busy = false;
                dsme_atomicEnd();
                return FSM_HANDLED;
            }
        }

        case AckEvent::RECEIVE_REQUEST:
            if(!dsme.getPlatform().isReceptionFromAckLayerPossible()) {
                dsme.getPlatform().releaseMessage(pendingMessage);
                pendingMessage = nullptr;
                dsme_atomicBegin();
                busy = false;
                dsme_atomicEnd();
                return FSM_HANDLED;
            }

            // according to 5.2.1.1.4, the ACK shall be sent anyway even with broadcast address, but this can not work for GTS replies (where the AR bit has to
            // be set 5.3.11.5.2)
            if(pendingMessage->getHeader().isAckRequested() && !pendingMessage->getHeader().getDestAddr().isBroadcast()) {
                LOG_DEBUG("sending ACK");

                // keep the received message and set up the acknowledgement as new pending message
                DSMEMessage* receivedMessage = pendingMessage;
                pendingMessage = dsme.getPlatform().getEmptyMessage();
                if(pendingMessage == nullptr) {
                    DSME_ASSERT(false);
                    dsme_atomicBegin();
                    busy = false;
                    dsme_atomicEnd();
                    return FSM_HANDLED;
                }

                IEEE802154eMACHeader& ackHeader = pendingMessage->getHeader();
                ackHeader.setFrameType(IEEE802154eMACHeader::ACKNOWLEDGEMENT);
                ackHeader.setSequenceNumber(receivedMessage->getHeader().getSequenceNumber());

                ackHeader.setDstAddr(receivedMessage->getHeader().getSrcAddr()); // TODO remove, this is only for the sequence diagram

                /* platform has to handle delaying the ACK to obey aTurnaroundTime */
                bool success = dsme.getPlatform().sendDelayedAck(pendingMessage, receivedMessage, internalDoneCallback);

                /* let upper layer handle the received message after the ACK has been transmitted */
                dsme.getPlatform().handleReceivedMessageFromAckLayer(receivedMessage);

                if(success) {
                    return transition(&AckLayer::stateTxAck);
                } else {
                    DSME_SIM_ASSERT(false);

                    dsme.getPlatform().releaseMessage(pendingMessage);
                    pendingMessage = nullptr;
                    dsme_atomicBegin();
                    busy = false;
                    dsme_atomicEnd();
                    return FSM_HANDLED;
                }
            } else {
                dsme.getPlatform().handleReceivedMessageFromAckLayer(pendingMessage);
                pendingMessage = nullptr; // owned by upper layer now
                dsme_atomicBegin();
                busy = false;
                dsme_atomicEnd();
                return FSM_HANDLED;
            }

        default:
            return catchAll(event);
    }
}

fsmReturnStatus AckLayer::statePreparingTx(AckEvent& event) {
    switch(event.signal) {
        case AckEvent::START_TRANSMISSION: {
            bool result = dsme.getPlatform().sendNow();
            ASSERT(result);
            return transition(&AckLayer::stateTx);
        }
        case AckEvent::RESET:
        case AckEvent::ABORT_TRANSMISSION:
            externalDoneCallback(SEND_ABORTED, pendingMessage);
            pendingMessage = nullptr;

            dsme.getPlatform().abortPreparedTransmission();
            return transition(&AckLayer::stateAbort);
        default:
            return catchAll(event);
    }
}

fsmReturnStatus AckLayer::stateTx(AckEvent& event) {
    switch(event.signal) {
        case AckEvent::SEND_DONE:
            if(!event.success) {
                externalDoneCallback(SEND_FAILED, pendingMessage);
                return transition(&AckLayer::stateIdle);
            } else {
                // ACK requested?
                if(pendingMessage->getHeader().isAckRequested() && !pendingMessage->getHeader().getDestAddr().isBroadcast()) {
                    // according to 5.2.1.1.4, the ACK shall be sent anyway even with broadcast address, but this can not work for GTS replies (where the AR bit
                    // has to be set 5.3.11.5.2)
                    // unless an acknowledgment shall be sent from an upper layer (can not be interfered from the standard)

                    return transition(&AckLayer::stateWaitForAck);
                } else {
                    externalDoneCallback(NO_ACK_REQUESTED, pendingMessage);
                    return transition(&AckLayer::stateIdle);
                }
            }

        case AckEvent::RESET:
            externalDoneCallback(SEND_ABORTED, pendingMessage);
            pendingMessage = nullptr;
            return transition(&AckLayer::stateAbort);

        default:
            return catchAll(event);
    }
}

fsmReturnStatus AckLayer::stateWaitForAck(AckEvent& event) {
    switch(event.signal) {
        case AckEvent::ENTRY_SIGNAL:
            dsme.getEventDispatcher().setupACKTimer();
            return FSM_HANDLED;
        case AckEvent::ACK_RECEIVED:
            if(event.seqNum == pendingMessage->getHeader().getSequenceNumber()) {
                dsme.getEventDispatcher().stopACKTimer();
                externalDoneCallback(ACK_SUCCESSFUL, pendingMessage);
                return transition(&AckLayer::stateIdle);
            } else {
                /* '-> if sequence number does not match, ignore this ACK */
                return FSM_HANDLED;
            }

        case AckEvent::RESET:
        case AckEvent::TIMER_FIRED:
            dsme.getEventDispatcher().stopACKTimer();
            LOG_DEBUG("ACK timer fired for seqNum: " << (uint16_t)pendingMessage->getHeader().getSequenceNumber() << " dstAddr "
                                                     << pendingMessage->getHeader().getDestAddr().getShortAddress());
            externalDoneCallback(ACK_FAILED, pendingMessage);
            return transition(&AckLayer::stateIdle);

        default:
            return catchAll(event);
    }
}

fsmReturnStatus AckLayer::stateTxAck(AckEvent& event) {
    switch(event.signal) {
        case AckEvent::SEND_DONE:
            dsme.getPlatform().releaseMessage(pendingMessage);
            return transition(&AckLayer::stateIdle);

        case AckEvent::RESET:
            return transition(&AckLayer::stateAbort);

        default:
            return catchAll(event);
    }
}

fsmReturnStatus AckLayer::stateAbort(AckEvent& event) {
    switch(event.signal) {
        case AckEvent::SEND_DONE:
            // external callback was already called if message was no ACK
            if(pendingMessage) {
                dsme.getPlatform().releaseMessage(pendingMessage);
            }
            return transition(&AckLayer::stateIdle);

        default:
            return catchAll(event);
    }
}
}
