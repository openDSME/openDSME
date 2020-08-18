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

#include "./AckLayer.h"

#include "../../../dsme_platform.h"
#include "../../helper/DSMEAtomic.h"
#include "../../helper/DSMEDelegate.h"
#include "../../helper/DSMEFSM.h"
#include "../../interfaces/IDSMEMessage.h"
#include "../../interfaces/IDSMEPlatform.h"
#include "../../mac_services/DSME_Common.h"
#include "../../mac_services/dataStructures/IEEE802154MacAddress.h"
#include "../../mac_services/pib/MAC_PIB.h"
#include "../DSMEEventDispatcher.h"
#include "../DSMELayer.h"
#include "../messages/IEEE802154eMACHeader.h"
#include "../messages/GackCmd.h"

namespace dsme {

AckLayer::AckLayer(DSMELayer& dsme)
    : DSMEBufferedFSM<AckLayer, AckEvent, 2>(&AckLayer::stateIdle), dsme(dsme), internalDoneCallback(DELEGATE(&AckLayer::sendDone, *this)) {
    GackMap.initialize(7*20, false); //dsme.getMAC_PIB().sizeGackMap
}

void AckLayer::reset() {
    bool dispatchSuccessful = dispatch(AckEvent::RESET);
    DSME_ASSERT(dispatchSuccessful);
}

bool AckLayer::prepareSendingCopy(IDSMEMessage* msg, done_callback_t doneCallback) {
    DSME_ATOMIC_BLOCK {
        if(this->busy) {
            return false;
        } else {
            this->busy = true;
        }
    }

    DSME_ASSERT(this->pendingMessage == nullptr);

    this->pendingMessage = msg;
    this->externalDoneCallback = doneCallback;
    DSME_ASSERT(!isDispatchBusy());
    bool dispatchSuccessful = dispatch(AckEvent::PREPARE_SEND_REQUEST);
    DSME_ASSERT(dispatchSuccessful);
    return true;
}

void AckLayer::sendNowIfPending() {
    if(this->pendingMessage) {
        DSME_ASSERT(busy);
        DSME_ASSERT(!isDispatchBusy());
        bool dispatchSuccessful = dispatch(AckEvent::START_TRANSMISSION);
        DSME_ASSERT(dispatchSuccessful);
    }
}

void AckLayer::abortPreparedTransmission() {
    if(this->pendingMessage) {
        DSME_ASSERT(!isDispatchBusy());
        bool dispatchSuccessful = dispatch(AckEvent::ABORT_TRANSMISSION);
        DSME_ASSERT(dispatchSuccessful);
    }
}

void AckLayer::receive(IDSMEMessage* msg) {
    IEEE802154eMACHeader& header = msg->getHeader();

    if(header.getGack() == true && header.getSequenceNumber() == 255 && header.getFrameType() == IEEE802154eMACHeader::ACKNOWLEDGEMENT){
        LOG_INFO("GACK message received");
        GackCmd Gack;
        Gack.decapsulateFrom(msg);

        LOG_INFO("GACK MAP RECEIVED: ");
            int count = 0;
            for(int i = 0; i < dsme.getMAC_PIB().sizeGackMap; i++){
                LOG_INFO("slotID: " << i << " status: " << Gack.getGackMap().get(i));
                if(Gack.getGackMap().get(i) == true){
                    count++;
                }
            }
            LOG_INFO("Number one: " << count);
    }
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
        uint8_t seqNum = header.getSequenceNumber();
        dsme.getPlatform().releaseMessage(msg);
        DSME_ASSERT(!isDispatchBusy());
        bool dispatchSuccessful = dispatch(AckEvent::ACK_RECEIVED, seqNum);
        DSME_ASSERT(dispatchSuccessful);
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
    DSME_ATOMIC_BLOCK {
        if(busy) {
            LOG_DEBUG("Throwing away packet, ACKLayer was busy.");
            this->dsme.getPlatform().releaseMessage(msg);
            // DSME_SIM_ASSERT(false);
            return;
        } else {
            busy = true;
        }
    }

    this->pendingMessage = msg;
    DSME_ASSERT(!isDispatchBusy());
    bool dispatchSuccessful = dispatch(AckEvent::RECEIVE_REQUEST);
    DSME_ASSERT(dispatchSuccessful);

    return;
}

void AckLayer::dispatchTimer() {
    if(isDispatchBusy()) {
        return; // already processing (e.g. ACK arrived just in time)
    }
    bool dispatchSuccessful = dispatch(AckEvent::TIMER_FIRED);
    DSME_ASSERT(dispatchSuccessful);
}

// ifMsgPending(): retrieves true if there are message pending to be sent. False, otherwise
bool AckLayer::ifMsgPending(){
    bool pending = false;
    if(this->pendingMessage != nullptr) {
        pending = true;
    }
return pending;
}

void AckLayer::sendDone(bool success) {
    DSME_ASSERT(!isDispatchBusy());
    bool dispatchSuccessful = dispatch(AckEvent::SEND_DONE, success);
    DSME_ASSERT(dispatchSuccessful);
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
            DSME_ATOMIC_BLOCK {
                this->busy = false;
            }
            return FSM_HANDLED;

        case AckEvent::RESET:
            return FSM_HANDLED;

        case AckEvent::PREPARE_SEND_REQUEST: {
            if(pendingMessage->getHeader().hasSequenceNumber()) {
                if(pendingMessage->getRetryCounter() == 0) {
                    if(pendingMessage->getHeader().getFrameType() != IEEE802154eMACHeader::ACKNOWLEDGEMENT){
                        pendingMessage->getHeader().setSequenceNumber(this->dsme.getMAC_PIB().macDsn++);
                    }
                } else {
                    /* message is a retransmit, keeps sequence number from previous try */
                }
            }

            if(dsme.getPlatform().prepareSendingCopy(pendingMessage, internalDoneCallback)) {
                return transition(&AckLayer::statePreparingTx);
            } else {
                /* '-> currently busy (e.g. recent reception) */
                signalResult(SEND_FAILED);
                DSME_ATOMIC_BLOCK {
                    this->busy = false;
                }
                return FSM_HANDLED;
            }
        }

        case AckEvent::RECEIVE_REQUEST:
            if(!dsme.getPlatform().isReceptionFromAckLayerPossible()) {
                dsme.getPlatform().releaseMessage(pendingMessage);
                pendingMessage = nullptr;
                DSME_ATOMIC_BLOCK {
                    this->busy = false;
                }
                return FSM_HANDLED;
            }

            // according to 5.2.1.1.4, the ACK shall be sent anyway even with broadcast address, but this can not work for GTS replies (where the AR bit has to
            // be set 5.3.11.5.2)

            //command
            // if groupack
            // fill bitmap
            // print bitmap
            if(pendingMessage->getHeader().getFrameControl().frameType == IEEE802154eMACHeader::DATA && pendingMessage->getHeader().getGack() == true){ // check in GTS
                GackUsed = true;
                if(newSuperframe == true){
                    newSuperframe = false;
                    lastSeqNum = pendingMessage->getHeader().getSequenceNumber() - 1;
                    GackMapIterator = 0;
                    lastGTSID = 0;
                    GackMap.initialize(dsme.getMAC_PIB().sizeGackMap, false);
                }

                if(lastGTSID < dsme.getMessageDispatcher().currentACTElement.node()->content.getGTSlotID()){
                    lastGTSID = dsme.getMessageDispatcher().currentACTElement.node()->content.getGTSlotID();
                    GackMapIterator = 0;
                    lastSeqNum = pendingMessage->getHeader().getSequenceNumber() - 1; //wird die SeqNum immer um eins erhöht?
                }

                if(lastSeqNum + 1 == pendingMessage->getHeader().getSequenceNumber()){
                    GackMap.set(lastGTSID*(dsme.getMAC_PIB().sizeGackMap/7)+GackMapIterator, true);
                    GackMapIterator++;
                } else if(lastSeqNum + 1 < pendingMessage->getHeader().getSequenceNumber()){
                    for(int i = lastSeqNum + 1; i < pendingMessage->getHeader().getSequenceNumber(); i++){
                        GackMap.set(lastGTSID*(dsme.getMAC_PIB().sizeGackMap/7)+GackMapIterator, false);
                        GackMapIterator++;
                    }
                    GackMap.set(lastGTSID*(dsme.getMAC_PIB().sizeGackMap/7)+GackMapIterator, true);
                }
                lastSeqNum = pendingMessage->getHeader().getSequenceNumber();
            }

            if(pendingMessage->getHeader().isAckRequested() && !pendingMessage->getHeader().getDestAddr().isBroadcast()) {
                LOG_DEBUG("sending ACK");

                // keep the received message and set up the acknowledgement as new pending message
                IDSMEMessage* receivedMessage = pendingMessage;
                pendingMessage = dsme.getPlatform().getEmptyMessage();
                //pendingMessage.prependFrom(BitmapMessageElement)
                if(pendingMessage == nullptr) {
                    DSME_ASSERT(false);
                    DSME_ATOMIC_BLOCK {
                        this->busy = false;
                    }
                    return FSM_HANDLED;
                }

                IEEE802154eMACHeader& ackHeader = pendingMessage->getHeader(); //TODO remove IEQueue
                ackHeader.setFrameType(IEEE802154eMACHeader::ACKNOWLEDGEMENT);
                ackHeader.setSequenceNumber(receivedMessage->getHeader().getSequenceNumber());
                ackHeader.setGack(false);

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
                    DSME_ATOMIC_BLOCK {
                        this->busy = false;
                    }
                    return FSM_HANDLED;
                }
            } else {
                dsme.getPlatform().handleReceivedMessageFromAckLayer(pendingMessage);
                pendingMessage = nullptr; // owned by upper layer now
                DSME_ATOMIC_BLOCK {
                    this->busy = false;
                }
                return FSM_HANDLED;
            }

        default:
            return catchAll(event);
    }
}

fsmReturnStatus AckLayer::statePreparingTx(AckEvent& event) {
    switch(event.signal) {
        case AckEvent::START_TRANSMISSION: {
            bool result = this->dsme.getPlatform().sendNow();
            DSME_ASSERT(result);
            return transition(&AckLayer::stateTx);
        }
        case AckEvent::RESET:
        case AckEvent::ABORT_TRANSMISSION:
            DSME_ASSERT(this->pendingMessage);
            signalResult(SEND_ABORTED);
            pendingMessage = nullptr;
            dsme.getPlatform().abortPreparedTransmission();
            return transition(&AckLayer::stateIdle);

        case AckEvent::SEND_DONE:
            // only out of abortPreparedTransmission
            return FSM_HANDLED;

        default:
            return catchAll(event);
    }
}

fsmReturnStatus AckLayer::stateTx(AckEvent& event) {
    switch(event.signal) {
        case AckEvent::SEND_DONE:
            if(!event.success) {
                signalResult(SEND_FAILED);
                return transition(&AckLayer::stateIdle);
            } else {
                // ACK requested?
                if(this->pendingMessage->getHeader().isAckRequested() && !this->pendingMessage->getHeader().getDestAddr().isBroadcast()) {
                    // according to 5.2.1.1.4, the ACK shall be sent anyway even with broadcast address, but this can not work for GTS replies (where the AR bit
                    // has to be set 5.3.11.5.2)
                    // unless an acknowledgment shall be sent from an upper layer (can not be interfered from the standard)

                    return transition(&AckLayer::stateWaitForAck);
                } else {
                    signalResult(NO_ACK_REQUESTED);
                    return transition(&AckLayer::stateIdle);
                }
            }

        case AckEvent::RESET:
            signalResult(SEND_ABORTED);
            this->pendingMessage = nullptr;
            return transition(&AckLayer::stateAbort);

        default:
            return catchAll(event);
    }
}

fsmReturnStatus AckLayer::stateWaitForAck(AckEvent& event) {
    switch(event.signal) {
        case AckEvent::ENTRY_SIGNAL:
            this->dsme.getEventDispatcher().setupACKTimer();
            return FSM_HANDLED;
        case AckEvent::ACK_RECEIVED:
            if(event.seqNum == pendingMessage->getHeader().getSequenceNumber()) {
                dsme.getEventDispatcher().stopACKTimer();
                signalResult(ACK_SUCCESSFUL);
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
            signalResult(ACK_FAILED);
            return transition(&AckLayer::stateIdle);

        default:
            return catchAll(event);
    }
}

fsmReturnStatus AckLayer::stateTxAck(AckEvent& event) {
    switch(event.signal) {
        case AckEvent::SEND_DONE:{
            dsme.getMessageDispatcher().handleAckTransmitted();
            dsme.getPlatform().releaseMessage(pendingMessage);
            pendingMessage = nullptr;
            return transition(&AckLayer::stateIdle);
        }
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
                pendingMessage = nullptr;
            }
            return transition(&AckLayer::stateIdle);

        default:
            return catchAll(event);
    }
}

void AckLayer::signalResult(enum AckLayerResponse response) {
    auto addr = pendingMessage->getHeader().getDestAddr();
    externalDoneCallback(response, pendingMessage);
    pendingMessage = nullptr; // owned by upper layer now
}

//JND:
void AckLayer::handleStartofCFP(){
    lastSeqNum = 0;
    lastSfID = 0;
    lastGTSID = 0;
    newSuperframe = true;
    GackUsed = false;
}

void AckLayer::handleStartofCAP(){
    if(GackUsed == true){
        GackCmd GackMessageElement(GackMap);

        IDSMEMessage* GackMessage = dsme.getPlatform().getEmptyMessage();
        GackMessageElement.prependTo(GackMessage);

        IEEE802154eMACHeader& ackHeader = GackMessage->getHeader(); //TODO remove IEQueue
        ackHeader.setFrameType(IEEE802154eMACHeader::ACKNOWLEDGEMENT);
        ackHeader.setSequenceNumber(255);
        ackHeader.setGack(true);

        ackHeader.setDstAddr(IEEE802154MacAddress(IEEE802154MacAddress::SHORT_BROADCAST_ADDRESS));
        ackHeader.setSrcAddrMode(SHORT_ADDRESS);
        ackHeader.setSrcAddr(IEEE802154MacAddress(dsme.getMAC_PIB().macShortAddress));

        dsme.getMessageDispatcher().sendInCAP(GackMessage);
    }
}

} /* namespace dsme */
