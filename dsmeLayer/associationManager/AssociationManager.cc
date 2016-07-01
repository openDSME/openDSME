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

#include "AssociationManager.h"

#include "../../mac_services/DSME_Common.h"
#include "../DSMELayer.h"
#include "../messages/AssociateReplyCmd.h"
#include "../messages/MACCommand.h"

namespace dsme {

AssociationManager::AssociationManager(DSMELayer& dsme) :
        dsme(dsme) {
}

void AssociationManager::sendAssociationRequest(AssociateRequestCmd &req, mlme_sap::ASSOCIATE::request_parameters &params) {
    LOG_INFO("Requesting association to " << params.coordAddress.getShortAddress() << ".");

    ASSERT(!associationPending);

    associationPending = true;
    associationSent = false;

    DSMEMessage* msg = dsme.getPlatform().getEmptyMessage();
    req.prependTo(msg);
    MACCommand cmd;
    cmd.setCmdId(CommandFrameIdentifier::ASSOCIATION_REQUEST);
    cmd.prependTo(msg);

    //Header compliant to specification (IEEE802.15.4-2012 5.3.11.2.1).
    msg->getHeader().setDstAddr(params.coordAddress);
    msg->getHeader().setSrcAddr(dsme.getMAC_PIB().macExtendedAddress);
    msg->getHeader().getFrameControl().srcAddrMode = EXTENDED_ADDRESS;
    msg->getHeader().getFrameControl().dstAddrMode = params.coordAddrMode;
    msg->getHeader().getFrameControl().framePending = 0;
    msg->getHeader().setDstPANId(params.coordPANId);
    msg->getHeader().setSrcPANId(BROADCAST_PAN_ID);
    msg->getHeader().setAckRequest(true);
    msg->getHeader().setFrameType(IEEE802154eMACHeader::FrameType::COMMAND);

    if(!dsme.getMessageDispatcher().sendInCAP(msg)) {
        associationPending = false;
        mlme_sap::ASSOCIATE_confirm_parameters params;
        params.assocShortAddress = 0xFFFF;
        params.status = AssociationStatus::CHANNEL_ACCESS_FAILURE;
        this->dsme.getMLME_SAP().getASSOCIATE().notify_confirm(params);
        dsme.getPlatform().releaseMessage(msg);
    }
}

void AssociationManager::handleAssociationRequest(DSMEMessage *msg) {

    AssociateRequestCmd req;
    req.decapsulateFrom(msg);

    mlme_sap::ASSOCIATE_indication_parameters params;
    params.deviceAddress = msg->getHeader().getSrcAddr();
    params.capabilityInformation = req.getCapabilityInformation();

    this->dsme.getMLME_SAP().getASSOCIATE().notify_indication(params);
}

void AssociationManager::sendAssociationReply(AssociateReplyCmd &reply, IEEE802154MacAddress &deviceAddress) {
    LOG_INFO("Replying to association request from " << deviceAddress.getShortAddress() << ".");

    DSMEMessage* msg = dsme.getPlatform().getEmptyMessage();
    reply.prependTo(msg);
    MACCommand cmd;
    cmd.setCmdId(CommandFrameIdentifier::ASSOCIATION_RESPONSE);
    cmd.prependTo(msg);

    //Header compliant to specification (IEEE802.15.4-2012 5.3.11.3.1).
    msg->getHeader().getFrameControl().srcAddrMode = AddrMode::EXTENDED_ADDRESS;
    msg->getHeader().getFrameControl().dstAddrMode = AddrMode::EXTENDED_ADDRESS;
    msg->getHeader().getFrameControl().framePending = 0;
    msg->getHeader().setAckRequest(true);
    msg->getHeader().getFrameControl().panIDCompression = 1;
    msg->getHeader().setDstPANId(dsme.getMAC_PIB().macPANId);
    msg->getHeader().setDstAddr(deviceAddress);
    msg->getHeader().setSrcAddr(dsme.getMAC_PIB().macExtendedAddress);

    msg->getHeader().setFrameType(IEEE802154eMACHeader::FrameType::COMMAND);

    if(!dsme.getMessageDispatcher().sendInCAP(msg)) {
        // TODO
        dsme.getPlatform().releaseMessage(msg);
    }
}

void AssociationManager::handleAssociationReply(DSMEMessage *msg) {
    AssociateReplyCmd reply;
    reply.decapsulateFrom(msg);

    mlme_sap::ASSOCIATE_confirm_parameters params;
    params.assocShortAddress = reply.getShortAddr();
    params.status = reply.getStatus();

    this->dsme.getMLME_SAP().getASSOCIATE().notify_confirm(params);

    if ((params.status == AssociationStatus::SUCCESS) || (params.status == AssociationStatus::FASTA_SUCCESSFUL)) {
        dsme.getMAC_PIB().macPANId = msg->getHeader().getSrcPANId();
        dsme.getMAC_PIB().macShortAddress = params.assocShortAddress;
        dsme.getMAC_PIB().macAssociatedPANCoord = true;
    } else {
        /* default in case of failure */
        dsme.getMAC_PIB().macPANId = 0xffff;
        dsme.getMAC_PIB().macShortAddress = 0xffff;
    }
}

/*****************************************************************************
 * Disassociation
 ****************************************************************************/
void AssociationManager::sendDisassociationRequest(DisassociationNotifyCmd &req, mlme_sap::DISASSOCIATE::request_parameters &params) {

    DSMEMessage* msg = dsme.getPlatform().getEmptyMessage();
    req.prependTo(msg);
    MACCommand cmd;
    cmd.setCmdId(CommandFrameIdentifier::DISASSOCIATION_NOTIFICATION);
    cmd.prependTo(msg);

    //Header compliant to specification (IEEE802.15.4-2011 5.3.3.1).
    msg->getHeader().getFrameControl().dstAddrMode = params.deviceAddrMode;
    msg->getHeader().getFrameControl().srcAddrMode = AddrMode::EXTENDED_ADDRESS;
    msg->getHeader().getFrameControl().framePending = 0;
    msg->getHeader().setAckRequest(true);
    msg->getHeader().getFrameControl().panIDCompression = 1;
    msg->getHeader().setDstPANId(dsme.getMAC_PIB().macPANId);

    if (req.getReason() == DisassociateReason::COORD_WISH_DEVICE_TO_LEAVE) {
        msg->getHeader().setDstAddr(params.deviceAddress);
    } else {
        if (params.deviceAddrMode == AddrMode::SHORT_ADDRESS) {
            msg->getHeader().setDstAddr(dsme.getMAC_PIB().macCoordShortAddress);
        } else {
            msg->getHeader().setDstAddr(dsme.getMAC_PIB().macCoordExtendedAddress);
        }
    }
    msg->getHeader().setSrcAddr(dsme.getMAC_PIB().macExtendedAddress);
    msg->getHeader().setFrameType(IEEE802154eMACHeader::FrameType::COMMAND);

    if(!dsme.getMessageDispatcher().sendInCAP(msg)) {
        // TODO
        dsme.getPlatform().releaseMessage(msg);
    }
}

void AssociationManager::handleDisassociationRequest(DSMEMessage *msg) {
    DisassociationNotifyCmd req;
    req.decapsulateFrom(msg);

    mlme_sap::DISASSOCIATE_indication_parameters params;
    params.deviceAddress = msg->getHeader().getSrcAddr();
    params.disassociateReason = req.getReason();

    this->dsme.getMLME_SAP().getDISASSOCIATE().notify_indication(params);
}


/**
 * Gets called when CSMA Message was sent down to the PHY
 */
void AssociationManager::onCSMASent(DSMEMessage* msg, CommandFrameIdentifier cmdId, DataStatus::Data_Status status, uint8_t numBackoffs) {
    mlme_sap::ASSOCIATE_confirm_parameters params;
    params.assocShortAddress = 0xFFFF;

    if(cmdId == ASSOCIATION_REQUEST) {
        if(status != DataStatus::Data_Status::SUCCESS) {
            associationPending = false;
        }

        switch(status) {
        case DataStatus::Data_Status::SUCCESS:
            ASSERT(associationPending);
            associationSent = true;
            superframesSinceAssociationSent = 0;
            break;
        case DataStatus::Data_Status::TRANSACTION_OVERFLOW:
        case DataStatus::Data_Status::TRANSACTION_EXPIRED:
        case DataStatus::Data_Status::CHANNEL_ACCESS_FAILURE:
            params.status = AssociationStatus::CHANNEL_ACCESS_FAILURE;
            this->dsme.getMLME_SAP().getASSOCIATE().notify_confirm(params);
            break;
        case DataStatus::Data_Status::COUNTER_ERROR:
            params.status = AssociationStatus::COUNTER_ERROR;
            this->dsme.getMLME_SAP().getASSOCIATE().notify_confirm(params);
            break;
        case DataStatus::Data_Status::NO_ACK:
            params.status = AssociationStatus::NO_ACK;
            this->dsme.getMLME_SAP().getASSOCIATE().notify_confirm(params);
            break;
        case DataStatus::Data_Status::INVALID_PARAMETER:
            params.status = AssociationStatus::INVALID_PARAMETER;
            this->dsme.getMLME_SAP().getASSOCIATE().notify_confirm(params);
            break;
        case DataStatus::Data_Status::UNSUPPORTED_SECURITY:
            params.status = AssociationStatus::UNSUPPORTED_SECURITY;
            this->dsme.getMLME_SAP().getASSOCIATE().notify_confirm(params);
            break;
        case DataStatus::Data_Status::UNAVAILABLE_KEY:
            params.status = AssociationStatus::UNAVAILABLE_KEY;
            this->dsme.getMLME_SAP().getASSOCIATE().notify_confirm(params);
            break;
        case DataStatus::Data_Status::FRAME_TOO_LONG:
            params.status = AssociationStatus::FRAME_TOO_LONG;
            this->dsme.getMLME_SAP().getASSOCIATE().notify_confirm(params);
            break;
        case DataStatus::Data_Status::INVALID_ADDRESS:
        case DataStatus::Data_Status::INVALID_GTS:
        case DataStatus::Data_Status::ACK_RCVD_NODSN_NOSA:
            ASSERT(false);
        }
    }

    dsme.getPlatform().releaseMessage(msg);
}

void AssociationManager::handleSlotEvent(uint8_t slot, uint8_t superframe) {
    if(slot == dsme.getMAC_PIB().helper.getFinalCAPSlot() + 1) {
        if(associationPending && associationSent) {
            superframesSinceAssociationSent++;
            // macResponseWaitTime is given in aBaseSuperframeDurations (that do not include the superframe order)
            if(superframesSinceAssociationSent*(1 << dsme.getMAC_PIB().macSuperframeOrder) > dsme.getMAC_PIB().macResponseWaitTime) {
                associationPending = false;
                mlme_sap::ASSOCIATE_confirm_parameters params;
                params.assocShortAddress = 0xFFFF;
                params.status = AssociationStatus::NO_DATA;
                this->dsme.getMLME_SAP().getASSOCIATE().notify_confirm(params);
            }
        }
    }
}

}

