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

#include "./MessageHelper.h"

#include "../../dsme_platform.h"
#include "../dsmeLayer/DSMELayer.h" // TODO: remove cross-layer reference
#include "../interfaces/IDSMEMessage.h"
#include "../mac_services/dataStructures/PANDescriptor.h"
#include "../mac_services/mcps_sap/MCPS_SAP.h"
#include "../mac_services/mlme_sap/MLME_SAP.h"
#include "../mac_services/pib/MAC_PIB.h"
#include "./DSMEAdaptionLayer.h"

namespace dsme {

MessageHelper::MessageHelper(DSMEAdaptionLayer& dsmeAdaptionLayer)
    : dsmeAdaptionLayer(dsmeAdaptionLayer),

      scanOrSyncInProgress(false),
      associationInProgress(false) {
}

void MessageHelper::initialize() {
    this->dsmeAdaptionLayer.getMCPS_SAP().getDATA().indication(DELEGATE(&MessageHelper::handleDataIndication, *this));
    this->dsmeAdaptionLayer.getMCPS_SAP().getDATA().confirm(DELEGATE(&MessageHelper::handleDataConfirm, *this));
    return;
}

void MessageHelper::setIndicationCallback(indicationCallback_t callback_indication) {
    this->callback_indication = callback_indication;
    return;
}

void MessageHelper::setConfirmCallback(confirmCallback_t callback_confirm) {
    this->callback_confirm = callback_confirm;
    return;
}

void MessageHelper::receiveIndication(IDSMEMessage* msg) {
    if(this->callback_indication) {
        callback_indication(msg);
    }
    return;
}

void MessageHelper::startAssociation() {
    if(!this->associationInProgress) {
        if(!this->scanOrSyncInProgress) {
            this->scanOrSyncInProgress = true;
            this->dsmeAdaptionLayer.getScanHelper().startScan();
        } else {
            LOG_INFO("Scan already in progress.");
        }
    } else {
        LOG_INFO("Association already in progress.");
    }
}

void MessageHelper::sendRetryBuffer() {
    if(this->retryBuffer.hasCurrent()) {
        DSMEAdaptionLayerBufferEntry* oldestEntry = this->retryBuffer.current();
        do {
            IDSMEMessage* currentMessage = this->retryBuffer.current()->message;
            DSME_ASSERT(!currentMessage->getCurrentlySending());

            this->retryBuffer.advanceCurrent();
            sendMessageDown(currentMessage, false);
        } while(this->retryBuffer.hasCurrent() && this->retryBuffer.current() != oldestEntry);
    }
}

void MessageHelper::sendMessage(IDSMEMessage* msg) {
    LOG_INFO("Sending DATA message");
    sendMessageDown(msg, true);
}

void MessageHelper::sendMessageDown(IDSMEMessage* msg, bool newMessage) {
    if(msg == nullptr) {
        /* '-> Error! */
        DSME_ASSERT(false);
        return;
    }
    DSME_ASSERT(!msg->getCurrentlySending());
    msg->setCurrentlySending(true);

    msg->getHeader().setSrcAddr(this->dsmeAdaptionLayer.getMAC_PIB().macExtendedAddress);
    IEEE802154MacAddress& dst = msg->getHeader().getDestAddr();

    if(!this->dsmeAdaptionLayer.getMAC_PIB().macAssociatedPANCoord) {
        startAssociation();
        LOG_INFO("Discarding message for " << dst.getShortAddress() << ".");
        callback_confirm(msg, DataStatus::TRANSACTION_OVERFLOW);
        return;
    }

    LOG_DEBUG("Sending DATA message to " << dst.getShortAddress() << " via MCPS.");

    if(dst.getShortAddress() == this->dsmeAdaptionLayer.getMAC_PIB().macShortAddress) {
        /* '-> loopback */
        LOG_INFO("Loopback message received.");
        receiveIndication(msg);
    } else {
        mcps_sap::DATA::request_parameters params;

        msg->getHeader().setSrcAddrMode(SHORT_ADDRESS);
        msg->getHeader().setDstAddrMode(SHORT_ADDRESS);
        msg->getHeader().setDstAddr(dst);

        msg->getHeader().setSrcPANId(this->dsmeAdaptionLayer.getMAC_PIB().macPANId);
        msg->getHeader().setDstPANId(this->dsmeAdaptionLayer.getMAC_PIB().macPANId);

        // Both PAN IDs are equal and we are using short addresses
        // so suppress the PAN ID -> This should not be the task
        // for the user of the MCPS, but it is specified like this... TODO
        params.panIdSuppressed = true;

        params.msdu = msg;
        params.msduHandle = 0; // TODO
        params.ackTx = true;

        /* TODO
        if(dsme.getDSMESettings().optimizations) {
            params.gtsTX = !msg->firstTry && !dst.isBroadcast();
        }
        else {
        */
        params.gtsTx = !dst.isBroadcast();
        //}

        params.indirectTx = false;
        params.ranging = NON_RANGING;
        params.uwbPreambleSymbolRepetitions = 0;
        params.dataRate = 0; // DSSS -> 0

        params.seqNumSuppressed = false;

        params.sendMultipurpose = false;

        if(params.gtsTx) {
            uint16_t srcAddr = this->dsmeAdaptionLayer.getMAC_PIB().macShortAddress;
            if(srcAddr == 0xfffe) {
                LOG_ERROR("No short address allocated -> cannot request GTS!");
            } else if(srcAddr == 0xffff) {
                LOG_INFO("Association required before slot allocation.");
                DSME_ASSERT(false);
            }

            if(!this->dsmeAdaptionLayer.getDSME().getMessageDispatcher().neighborExists(dst)) {
                // TODO implement neighbor management / routing
                this->dsmeAdaptionLayer.getDSME().getMessageDispatcher().addNeighbor(dst);
            }

            if(newMessage) {
                msg->setStartOfFrameDelimiterSymbolCounter(this->dsmeAdaptionLayer.getDSME().getPlatform().getSymbolCounter()); // MISSUSE for statistics
                msg->queueAtCreation = this->dsmeAdaptionLayer.getGTSHelper().indicateIncomingMessage(dst.getShortAddress());
                this->dsmeAdaptionLayer.getGTSHelper().checkAllocationForPacket(dst.getShortAddress());
            }

            LOG_INFO("Preparing transmission in CFP.");
        } else {
            LOG_INFO("Preparing transmission in CAP.");
        }

        this->dsmeAdaptionLayer.getMCPS_SAP().getDATA().request(params);
    }
    return;
}

void MessageHelper::handleDataIndication(mcps_sap::DATA_indication_parameters& params) {
    LOG_INFO("Received DATA message from MCPS.");
    this->dsmeAdaptionLayer.getGTSHelper().indicateReceivedMessage(params.msdu->getHeader().getSrcAddr().getShortAddress());
    receiveIndication(params.msdu);
    return;
}

bool MessageHelper::queueMessageIfPossible(IDSMEMessage* msg) {
    if(!this->retryBuffer.hasNext()) {
        DSME_ASSERT(this->retryBuffer.hasCurrent());

        DSMEAdaptionLayerBufferEntry* oldestEntry = this->retryBuffer.current();
        if(oldestEntry->message == msg) {
            this->retryBuffer.advanceCurrent();
            DSME_ASSERT(this->retryBuffer.hasNext());
            this->retryBuffer.next()->message = msg;
            this->retryBuffer.next()->initialSymbolCounter = this->dsmeAdaptionLayer.getDSME().getPlatform().getSymbolCounter();
            return true;
        }

        if(!oldestEntry->message->getCurrentlySending()) {
            uint32_t currentSymbolCounter = this->dsmeAdaptionLayer.getDSME().getPlatform().getSymbolCounter();
            LOG_DEBUG("DROPPED->" << oldestEntry->message->getHeader().getDestAddr().getShortAddress() << ": Retry-Queue overflow ("
                                  << currentSymbolCounter - oldestEntry->initialSymbolCounter << " symbols old)");
            DSME_ASSERT(callback_confirm);
            callback_confirm(oldestEntry->message, DataStatus::Data_Status::INVALID_GTS); // TODO change if queue is used for retransmissions
            this->retryBuffer.advanceCurrent();
        }
    }
    if(this->retryBuffer.hasNext()) {
        this->retryBuffer.next()->message = msg;
        this->retryBuffer.next()->initialSymbolCounter = this->dsmeAdaptionLayer.getDSME().getPlatform().getSymbolCounter();
        this->retryBuffer.advanceNext();
        return true; /* Do NOT release current message yet */
    }
    return false;
}

void MessageHelper::handleDataConfirm(mcps_sap::DATA_confirm_parameters& params) {
    LOG_DEBUG("Received DATA confirm from MCPS");
    IDSMEMessage* msg = params.msduHandle;

    DSME_ASSERT(msg->getCurrentlySending());
    msg->setCurrentlySending(false);

    if(params.status != DataStatus::SUCCESS) {
        if(params.gtsTX) {
            if(params.status == DataStatus::INVALID_GTS) {
                if(queueMessageIfPossible(msg)) {
                    return;
                }

                // GTS slot not yet allocated
                LOG_DEBUG("DROPPED->" << params.msduHandle->getHeader().getDestAddr().getShortAddress() << ": No GTS allocated + queue full");
            } else if(params.status == DataStatus::NO_ACK) {
                // This should not happen, but might be the case for temporary inconsistent slots
                LOG_DEBUG("DROPPED->" << params.msduHandle->getHeader().getDestAddr().getShortAddress() << ": No ACK");
            } else if(params.status == DataStatus::CHANNEL_ACCESS_FAILURE) {
                // This should not happen, but might be the case if a foreign packet is received and the phy is therefore busy
                DSME_SIM_ASSERT(false);
                LOG_DEBUG("DROPPED->" << params.msduHandle->getHeader().getDestAddr().getShortAddress() << ": Channel Access Failure");
            } else if(params.status == DataStatus::TRANSACTION_OVERFLOW) {
                // Queue is full
                LOG_DEBUG("DROPPED->" << params.msduHandle->getHeader().getDestAddr().getShortAddress() << ": Queue full");
            } else if(params.status == DataStatus::TRANSACTION_EXPIRED) {
                // Transaction expired, e.g. for RESET
                LOG_DEBUG("DROPPED->" << params.msduHandle->getHeader().getDestAddr().getShortAddress() << ": Expired");
            } else {
                // Should not occur
                DSME_ASSERT(false);
            }
        }
        // TODO specialize!
        // TODO verify that Ack in GTS is always successful for simulation
    }

    if(params.gtsTX) {
        int32_t serviceTime = (int32_t)this->dsmeAdaptionLayer.getDSME().getPlatform().getSymbolCounter() - (int32_t)msg->getStartOfFrameDelimiterSymbolCounter();
        this->dsmeAdaptionLayer.getGTSHelper().indicateOutgoingMessage(params.msduHandle->getHeader().getDestAddr().getShortAddress(), params.status == DataStatus::SUCCESS,
                                                    serviceTime, params.msduHandle->queueAtCreation);
    }

    DSME_ASSERT(callback_confirm);
    callback_confirm(params.msduHandle, params.status);
}

void MessageHelper::handleScanAndSyncComplete(PANDescriptor* panDescriptor) {
    DSME_ASSERT(this->scanOrSyncInProgress);
    DSME_ASSERT(!this->associationInProgress);

    if(panDescriptor != nullptr) {
        this->associationInProgress = true;
        this->scanOrSyncInProgress = false;
        this->dsmeAdaptionLayer.getAssociationHelper().associate(panDescriptor->coordPANId, panDescriptor->coordAddrMode, panDescriptor->coordAddress,
                                                                  panDescriptor->channelNumber);
    } else {
        LOG_INFO("Trying scan again!");
        this->dsmeAdaptionLayer.getScanHelper().startScan();
    }
    return;
}

void MessageHelper::handleAssociationComplete(AssociationStatus::Association_Status status) {
    this->associationInProgress = false;
    return;
}

} /* namespace dsme */
