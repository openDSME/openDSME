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

#include "AssociationManager.h"

#include "../../helper/DSMEAtomic.h"
#include "../../mac_services/DSME_Common.h"
#include "../DSMELayer.h"
#include "../messages/AssociateReplyCmd.h"
#include "../messages/MACCommand.h"

namespace dsme {

AssociationManager::AssociationManager(DSMELayer& dsme)
    : dsme(dsme), messageSent(false), currentAction(CommandFrameIdentifier::ASSOCIATION_REQUEST), superframesSinceAssociationSent(0) {
}

void AssociationManager::reset() {
    this->dsme.getMAC_PIB().macPANId = 0xffff;
    this->dsme.getMAC_PIB().macShortAddress = 0xffff;
    this->dsme.getMAC_PIB().macAssociatedPANCoord = false;
    this->dsme.getMAC_PIB().macCoordExtendedAddress = IEEE802154MacAddress::UNSPECIFIED;
    this->dsme.getMAC_PIB().macCoordShortAddress = IEEE802154MacAddress::NO_SHORT_ADDRESS;

    this->dsme.getMAC_PIB().macIsCoord = false;
}

void AssociationManager::sendAssociationRequest(AssociateRequestCmd& req, mlme_sap::ASSOCIATE::request_parameters& params) {
    LOG_INFO("Requesting association to " << params.coordAddress.getShortAddress() << ".");

    DSME_ATOMIC_BLOCK {
        if(this->actionPending) {
            mlme_sap::ASSOCIATE_confirm_parameters params;

            // TODO: This should be a TRANSACTION_OVERFLOW, but the standard does not support this
            params.status = AssociationStatus::CHANNEL_ACCESS_FAILURE;
            this->dsme.getMLME_SAP().getASSOCIATE().notify_confirm(params);
            return;
        }

        this->currentAction = CommandFrameIdentifier::ASSOCIATION_REQUEST;
        this->actionPending = true;
    }

    messageSent = false;

    IDSMEMessage* msg = dsme.getPlatform().getEmptyMessage();
    req.prependTo(msg);
    MACCommand cmd;
    cmd.setCmdId(CommandFrameIdentifier::ASSOCIATION_REQUEST);
    cmd.prependTo(msg);

    // Header compliant to specification (IEEE802.15.4-2012 5.3.11.2.1).
    msg->getHeader().setDstAddr(params.coordAddress);
    msg->getHeader().setSrcAddr(dsme.getMAC_PIB().macExtendedAddress);
    msg->getHeader().setSrcAddrMode(EXTENDED_ADDRESS);
    msg->getHeader().setDstAddrMode(params.coordAddrMode);
    msg->getHeader().setDstPANId(params.coordPANId);
    msg->getHeader().setSrcPANId(BROADCAST_PAN_ID);
    msg->getHeader().setAckRequest(true);
    msg->getHeader().setFrameType(IEEE802154eMACHeader::FrameType::COMMAND);

    if(!dsme.getMessageDispatcher().sendInCAP(msg)) {
        actionPending = false;
        mlme_sap::ASSOCIATE_confirm_parameters params;
        params.assocShortAddress = 0xFFFF;

        // TODO: This should be a TRANSACTION_OVERFLOW, but the standard does not support this
        params.status = AssociationStatus::CHANNEL_ACCESS_FAILURE;
        this->dsme.getMLME_SAP().getASSOCIATE().notify_confirm(params);
        dsme.getPlatform().releaseMessage(msg);
    }
}

void AssociationManager::handleAssociationRequest(IDSMEMessage* msg) {
    AssociateRequestCmd req;
    req.decapsulateFrom(msg);

    mlme_sap::ASSOCIATE_indication_parameters params;
    params.deviceAddress = msg->getHeader().getSrcAddr();
    params.capabilityInformation = req.getCapabilityInformation();

    this->dsme.getMLME_SAP().getASSOCIATE().notify_indication(params);
}

void AssociationManager::sendAssociationReply(AssociateReplyCmd& reply, IEEE802154MacAddress& deviceAddress) {
    DSME_ATOMIC_BLOCK {
        if(this->actionPending) {
            return;
        } else {
            this->currentAction = CommandFrameIdentifier::ASSOCIATION_RESPONSE;
            this->actionPending = true;
            DSME_ASSERT(!this->messageSent);
        }
    }

    LOG_INFO("Replying to association request from " << deviceAddress.getShortAddress() << ".");

    IDSMEMessage* msg = dsme.getPlatform().getEmptyMessage();
    reply.prependTo(msg);
    MACCommand cmd;
    cmd.setCmdId(CommandFrameIdentifier::ASSOCIATION_RESPONSE);
    cmd.prependTo(msg);

    // Header compliant to specification (IEEE802.15.4-2012 5.3.11.3.1).
    msg->getHeader().setSrcAddrMode(AddrMode::EXTENDED_ADDRESS);
    msg->getHeader().setDstAddrMode(AddrMode::EXTENDED_ADDRESS);
    msg->getHeader().setAckRequest(true);
    msg->getHeader().setDstPANId(dsme.getMAC_PIB().macPANId);
    msg->getHeader().setDstAddr(deviceAddress);
    msg->getHeader().setSrcPANId(dsme.getMAC_PIB().macPANId);
    msg->getHeader().setSrcAddr(dsme.getMAC_PIB().macExtendedAddress);

    msg->getHeader().setFrameType(IEEE802154eMACHeader::FrameType::COMMAND);

    if(!dsme.getMessageDispatcher().sendInCAP(msg)) {
        // TODO
        dsme.getPlatform().releaseMessage(msg);
    }
    return;
}

void AssociationManager::handleAssociationReply(IDSMEMessage* msg) {
    DSME_ATOMIC_BLOCK {
        if(!this->actionPending || this->currentAction != CommandFrameIdentifier::ASSOCIATION_REQUEST) {
            // No association pending, for example because of an ACK timeout
            return;
        }
        this->actionPending = false;
        this->messageSent = false;
    }

    AssociateReplyCmd reply;
    reply.decapsulateFrom(msg);

    mlme_sap::ASSOCIATE_confirm_parameters params;
    params.assocShortAddress = reply.getShortAddr();
    params.status = reply.getStatus();

    this->dsme.getMLME_SAP().getASSOCIATE().notify_confirm(params);

    if((params.status == AssociationStatus::SUCCESS) || (params.status == AssociationStatus::FASTA_SUCCESSFUL)) {
        this->dsme.getMAC_PIB().macPANId = msg->getHeader().getDstPANId();
        this->dsme.getMAC_PIB().macShortAddress = params.assocShortAddress;
        this->dsme.getMAC_PIB().macAssociatedPANCoord = true;

        this->dsme.startTrackingBeacons();

        this->dsme.getPlatform().updateVisual();
    } else {
        /* default in case of failure */
        this->dsme.getMAC_PIB().macPANId = 0xffff;
        this->dsme.getMAC_PIB().macShortAddress = 0xffff;
    }
    return;
}

/*****************************************************************************
 * Disassociation
 ****************************************************************************/
void AssociationManager::sendDisassociationRequest(DisassociationNotifyCmd& req, mlme_sap::DISASSOCIATE::request_parameters& params) {
    DSME_ATOMIC_BLOCK {
        if(this->actionPending) {
            mlme_sap::DISASSOCIATE_confirm_parameters params;

            // TODO: This should be a TRANSACTION_OVERFLOW, but the standard does not support this
            params.status = DisassociationStatus::CHANNEL_ACCESS_FAILURE;
            this->dsme.getMLME_SAP().getDISASSOCIATE().notify_confirm(params);
            return;
        }

        this->currentAction = CommandFrameIdentifier::DISASSOCIATION_NOTIFICATION;
        this->actionPending = true;
        DSME_ASSERT(!this->messageSent);
    }
    messageSent = false;

    IDSMEMessage* msg = dsme.getPlatform().getEmptyMessage();
    req.prependTo(msg);
    MACCommand cmd;
    cmd.setCmdId(CommandFrameIdentifier::DISASSOCIATION_NOTIFICATION);
    cmd.prependTo(msg);

    // Header compliant to specification (IEEE802.15.4-2011 5.3.3.1).
    msg->getHeader().setDstAddrMode(params.deviceAddrMode);
    msg->getHeader().setSrcAddrMode(AddrMode::EXTENDED_ADDRESS);
    msg->getHeader().setAckRequest(true);
    msg->getHeader().setDstPANId(dsme.getMAC_PIB().macPANId);

    if(req.getReason() == DisassociateReason::COORD_WISH_DEVICE_TO_LEAVE) {
        msg->getHeader().setDstAddr(params.deviceAddress);
    } else {
        if(params.deviceAddrMode == AddrMode::SHORT_ADDRESS) {
            msg->getHeader().setDstAddr(IEEE802154MacAddress(dsme.getMAC_PIB().macCoordShortAddress));
        } else {
            msg->getHeader().setDstAddr(dsme.getMAC_PIB().macCoordExtendedAddress);
        }
    }
    msg->getHeader().setSrcAddr(dsme.getMAC_PIB().macExtendedAddress);
    msg->getHeader().setFrameType(IEEE802154eMACHeader::FrameType::COMMAND);

    if(!dsme.getMessageDispatcher().sendInCAP(msg)) {
        actionPending = false;
        mlme_sap::DISASSOCIATE_confirm_parameters params;

        // TODO: This should be a TRANSACTION_OVERFLOW, but the standard does not support this
        params.status = DisassociationStatus::CHANNEL_ACCESS_FAILURE;
        this->dsme.getMLME_SAP().getDISASSOCIATE().notify_confirm(params);
        dsme.getPlatform().releaseMessage(msg);
    }
}

void AssociationManager::handleDisassociationRequest(IDSMEMessage* msg) {
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
void AssociationManager::onCSMASent(IDSMEMessage* msg, CommandFrameIdentifier cmdId, DataStatus::Data_Status status, uint8_t numBackoffs) {
    DSME_ATOMIC_BLOCK {
        if(!this->actionPending) {
            // Already received a response
            this->dsme.getPlatform().releaseMessage(msg);
            return;
        }

        DSME_ASSERT(this->currentAction == cmdId);

        if(cmdId == CommandFrameIdentifier::ASSOCIATION_RESPONSE) {
            this->actionPending = false;
            this->messageSent = false;
            this->superframesSinceAssociationSent = 0;
            this->dsme.getPlatform().releaseMessage(msg);
            return;
        }

        if(status != DataStatus::Data_Status::SUCCESS) {
            this->actionPending = false;
        }
    }

    mlme_sap::ASSOCIATE_confirm_parameters associate_params;
    mlme_sap::DISASSOCIATE_confirm_parameters disassociate_params;
    associate_params.assocShortAddress = 0xFFFF;

    switch(status) {
        case DataStatus::Data_Status::SUCCESS:
            DSME_ASSERT(actionPending);
            messageSent = true;
            superframesSinceAssociationSent = 0;
            dsme.getPlatform().releaseMessage(msg);
            return;
        case DataStatus::Data_Status::TRANSACTION_OVERFLOW:
        case DataStatus::Data_Status::TRANSACTION_EXPIRED:
        case DataStatus::Data_Status::CHANNEL_ACCESS_FAILURE:
            associate_params.status = AssociationStatus::CHANNEL_ACCESS_FAILURE;
            disassociate_params.status = DisassociationStatus::CHANNEL_ACCESS_FAILURE;
            break;
        case DataStatus::Data_Status::COUNTER_ERROR:
            associate_params.status = AssociationStatus::COUNTER_ERROR;
            disassociate_params.status = DisassociationStatus::COUNTER_ERROR;
            break;
        case DataStatus::Data_Status::NO_ACK:
            associate_params.status = AssociationStatus::NO_ACK;
            disassociate_params.status = DisassociationStatus::NO_ACK;
            break;
        case DataStatus::Data_Status::INVALID_PARAMETER:
            associate_params.status = AssociationStatus::INVALID_PARAMETER;
            disassociate_params.status = DisassociationStatus::INVALID_PARAMETER;
            break;
        case DataStatus::Data_Status::UNSUPPORTED_SECURITY:
            associate_params.status = AssociationStatus::UNSUPPORTED_SECURITY;
            disassociate_params.status = DisassociationStatus::UNSUPPORTED_SECURITY;
            break;
        case DataStatus::Data_Status::UNAVAILABLE_KEY:
            associate_params.status = AssociationStatus::UNAVAILABLE_KEY;
            disassociate_params.status = DisassociationStatus::UNAVAILABLE_KEY;
            break;
        case DataStatus::Data_Status::FRAME_TOO_LONG:
            associate_params.status = AssociationStatus::FRAME_TOO_LONG;
            disassociate_params.status = DisassociationStatus::FRAME_TOO_LONG;
            break;
        case DataStatus::Data_Status::INVALID_ADDRESS:
        case DataStatus::Data_Status::INVALID_GTS:
        case DataStatus::Data_Status::ACK_RCVD_NODSN_NOSA:
            DSME_ASSERT(false);
    }

    switch(cmdId) {
        case CommandFrameIdentifier::ASSOCIATION_REQUEST:
            this->dsme.getMLME_SAP().getASSOCIATE().notify_confirm(associate_params);
            break;
        case CommandFrameIdentifier::DISASSOCIATION_NOTIFICATION:
            this->dsme.getMLME_SAP().getDISASSOCIATE().notify_confirm(disassociate_params);
            break;
        default:
            DSME_ASSERT(false);
    }

    dsme.getPlatform().releaseMessage(msg);
}

void AssociationManager::handleStartOfCFP(uint8_t superframe) {
    bool confirmPending = false;
    CommandFrameIdentifier pendingAction = CommandFrameIdentifier::ASSOCIATION_REQUEST;

    DSME_ATOMIC_BLOCK {
        if(this->actionPending && this->messageSent) {
            this->superframesSinceAssociationSent++;
            // macResponseWaitTime is given in aBaseSuperframeDurations (that do not include the superframe order)
            if(this->superframesSinceAssociationSent * (1 << this->dsme.getMAC_PIB().macSuperframeOrder) > this->dsme.getMAC_PIB().macResponseWaitTime) {
                this->actionPending = false;
                this->messageSent = false;
                confirmPending = true;
                pendingAction = this->currentAction;
            }
        }
    }

    if(confirmPending) {
        if(pendingAction == CommandFrameIdentifier::ASSOCIATION_REQUEST) {
            mlme_sap::ASSOCIATE_confirm_parameters associate_params;
            associate_params.assocShortAddress = 0xFFFF;
            associate_params.status = AssociationStatus::NO_DATA;
            this->dsme.getMLME_SAP().getASSOCIATE().notify_confirm(associate_params);
        } else if(pendingAction == CommandFrameIdentifier::DISASSOCIATION_NOTIFICATION) {
            mlme_sap::DISASSOCIATE_confirm_parameters disassociate_params;
            disassociate_params.status = DisassociationStatus::SUCCESS; /* According to the standard, all failures mean disassociation */
            this->dsme.getMLME_SAP().getDISASSOCIATE().notify_confirm(disassociate_params);
        } else {
            LOG_ERROR((int)pendingAction);
            DSME_ASSERT(false);
        }
    }
}

} /* namespace dsme */
