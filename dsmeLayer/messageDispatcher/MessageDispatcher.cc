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

#include "./MessageDispatcher.h"

#include "../messages/MACCommand.h"
#include "../DSMELayer.h"
#include "../../mac_services/mcps_sap/DATA.h"
#include "../../mac_services/mcps_sap/MCPS_SAP.h"

namespace dsme {

MessageDispatcher::MessageDispatcher(DSMELayer &dsme) :
                dsme(dsme),
                doneGTS(DELEGATE(&MessageDispatcher::sendDoneGTS, *this)),
                dsmeAckFrame(nullptr),
                lastSendGTSNeighbor(neighborQueue.end())
{
}

MessageDispatcher::~MessageDispatcher() {
    for(NeighborQueue<MAX_NEIGHBORS>::iterator it = neighborQueue.begin(); it != neighborQueue.end(); ++it) {
        while(! this->neighborQueue.isQueueEmpty(it)) {
            DSMEMessage *msg = neighborQueue.popFront(it);
            this->dsme.getPlatform().releaseMessage(msg);
        }
    }
}

void MessageDispatcher::initialize(void) {
    currentACTElement = dsme.getMAC_PIB().macDSMEACT.end();
    return;
}

void MessageDispatcher::reset(void) {
    currentACTElement = dsme.getMAC_PIB().macDSMEACT.end();

    for(NeighborQueue<MAX_NEIGHBORS>::iterator it = neighborQueue.begin(); it != neighborQueue.end(); ++it) {
        while(! this->neighborQueue.isQueueEmpty(it)) {
            DSMEMessage *msg = neighborQueue.popFront(it);
            mcps_sap::DATA_confirm_parameters params;
            params.msduHandle = msg;
            params.Timestamp = 0;
            params.RangingReceived = false;
            params.gtsTX = true;
            params.status = DataStatus::TRANSACTION_EXPIRED;
            params.numBackoffs = 0;
            params.dsn = msg->getHeader().getSequenceNumber();
            params.AckPayload = nullptr;
            this->dsme.getMCPS_SAP().getDATA().notify_confirm(params);
        }
    }
    while(this->neighborQueue.getNumNeighbors() > 0) {
        NeighborQueue<MAX_NEIGHBORS>::iterator it = this->neighborQueue.begin();
        this->neighborQueue.eraseNeighbor(it);
    }

    return;
}

bool MessageDispatcher::handlePreSlotEvent(uint8_t nextSlot, uint8_t nextSuperframe) {
    // Prepare next slot
    // Switch to next slot channel and radio mode

    DSMEAllocationCounterTable& act = dsme.getMAC_PIB().macDSMEACT;

    if (nextSlot == 0) {
        // next slot will be the beacon frame
        uint8_t commonChannel = dsme.getPHY_PIB().phyCurrentChannel;
        dsme.getPlatform().setChannelNumber(commonChannel);
        currentACTElement = act.end();
    }
    else if (nextSlot > dsme.getMAC_PIB().helper.getFinalCAPSlot()) { // TODO correct?
        unsigned nextGTS = nextSlot - (dsme.getMAC_PIB().helper.getFinalCAPSlot() + 1);
        if (act.isAllocated(nextSuperframe, nextGTS)) {
            currentACTElement = act.find(nextSuperframe, nextGTS);
            DSME_ASSERT(currentACTElement != act.end());

            // For RX also if INVALID or UNCONFIRMED!
            if((currentACTElement->getState() == VALID) || (currentACTElement->getDirection() == Direction::RX)) {
                dsme.getPlatform().setChannelNumber(MIN_CHANNEL + currentACTElement->getChannel());
            }

            // statistic
            if (currentACTElement->getDirection() == RX) {
                numUnusedRxGts++;   // gets PURGE.cc decremented on actual reception
            }
        }
        else {
            currentACTElement = act.end();
        }
    } else {
        currentACTElement = act.end();
    }

    return true;
}

bool MessageDispatcher::handleSlotEvent(uint8_t slot, uint8_t superframe) {
    if (slot > dsme.getMAC_PIB().helper.getFinalCAPSlot()) {
        handleGTS();
    }
    return true;
}

void MessageDispatcher::receive(DSMEMessage* msg) {
    IEEE802154eMACHeader macHdr = msg->getHeader();

    switch (macHdr.getFrameType()) {
    case IEEE802154eMACHeader::FrameType::BEACON: {
        LOG_INFO("BEACON from " << macHdr.getSrcAddr().getShortAddress() << ".");
        this->dsme.getBeaconManager().handleBeacon(msg);
        this->dsme.getPlatform().releaseMessage(msg);
        break;
    }

    case IEEE802154eMACHeader::FrameType::COMMAND: {
        MACCommand cmd;
        cmd.decapsulateFrom(msg);
        switch (cmd.getCmdId()) {
        case CommandFrameIdentifier::DSME_GTS_REQUEST:
            LOG_INFO("DSME-GTS-REQUEST from " << macHdr.getSrcAddr().getShortAddress() << ".");
            dsme.getGTSManager().handleGTSRequest(msg);
            break;
        case CommandFrameIdentifier::DSME_GTS_REPLY:
            LOG_INFO("DSME-GTS-REPLY from " << macHdr.getSrcAddr().getShortAddress() << ".");
            dsme.getGTSManager().handleGTSResponse(msg);
            break;
        case CommandFrameIdentifier::DSME_GTS_NOTIFY:
            LOG_INFO("DSME-GTS-NOTIFY from " << macHdr.getSrcAddr().getShortAddress() << ".");
            dsme.getGTSManager().handleGTSNotify(msg);
            break;
        case CommandFrameIdentifier::ASSOCIATION_REQUEST:
            LOG_INFO("ASSOCIATION-REQUEST from " << macHdr.getSrcAddr().getShortAddress() << ".");
            dsme.getAssociationManager().handleAssociationRequest(msg);
            break;
        case CommandFrameIdentifier::ASSOCIATION_RESPONSE:
            LOG_INFO("ASSOCIATION-RESPONSE from " << macHdr.getSrcAddr().getShortAddress() << ".");
            dsme.getAssociationManager().handleAssociationReply(msg);
            break;
        case CommandFrameIdentifier::DISASSOCIATION_NOTIFICATION:
            LOG_INFO("DISASSOCIATION-NOTIFICATION from " << macHdr.getSrcAddr().getShortAddress() << ".");
            dsme.getAssociationManager().handleDisassociationRequest(msg);
            break;
        case CommandFrameIdentifier::DATA_REQUEST:
            /* Not implemented */
            break;
        case CommandFrameIdentifier::DSME_BEACON_ALLOCATION_NOTIFICATION:
            LOG_INFO("DSME-BEACON-ALLOCATION-NOTIFICATION from " << macHdr.getSrcAddr().getShortAddress() << ".");
            dsme.getBeaconManager().handleBeaconAllocation(msg);
            break;
        case CommandFrameIdentifier::DSME_BEACON_COLLISION_NOTIFICATION:
            LOG_INFO("DSME-BEACON-COLLISION-NOTIFICATION from " << macHdr.getSrcAddr().getShortAddress() << ".");
            dsme.getBeaconManager().handleBeaconCollision(msg);
            break;
        case CommandFrameIdentifier::BEACON_REQUEST:
            LOG_INFO("BEACON_REQUEST from " << macHdr.getSrcAddr().getShortAddress() << ".");
            dsme.getBeaconManager().handleBeaconRequest(msg);
            break;
        default:
            DSME_ASSERT(false);
        }
        dsme.getPlatform().releaseMessage(msg);
        break;
    }

    case IEEE802154eMACHeader::FrameType::DATA: {
        if(currentACTElement != dsme.getMAC_PIB().macDSMEACT.end()) {
            handleGTSFrame(msg);
        } else {
            createDataIndication(msg);
        }
        break;
    }

    default: {
        DSME_ASSERT(false); // TODO handle other frame types
    }
    }
    return;
}

void MessageDispatcher::createDataIndication(DSMEMessage* msg) {
    IEEE802154eMACHeader& header = msg->getHeader();

    mcps_sap::DATA_indication_parameters params;

    params.msdu = msg;

    params.mpduLinkQuality = 0; // TODO link quality?
    params.dsn = header.getSequenceNumber();
    params.timestamp = msg->getStartOfFrameDelimiterSymbolCounter();
    params.securityLevel = header.isSecurityEnabled();

    params.keyIdMode = 0;
    params.keySource = nullptr;
    params.keyIndex = 0;
    params.uwbprf = PRF_OFF;
    params.uwbPreambleSymbolRepetitions = 0;
    params.dataRate = 0; // DSSS -> 0

    params.rangingReceived = NO_RANGING_REQUESTED;
    params.rangingCounterStart = 0;
    params.rangingCounterStop = 0;
    params.rangingTrackingInterval = 0;
    params.rangingOffset = 0;
    params.rangingFOM = 0;

    this->dsme.getMCPS_SAP().getDATA().notify_indication(params);
}


bool MessageDispatcher::sendInGTS(DSMEMessage* msg, NeighborQueue<MAX_NEIGHBORS>::iterator destIt) {
    DSME_ASSERT(!msg->getHeader().getDestAddr().isBroadcast());
    DSME_ASSERT(this->dsme.getMAC_PIB().macAssociatedPANCoord);
    DSME_ASSERT(destIt != neighborQueue.end());

    numUpperPacketsForGTS++;

    if (!neighborQueue.isQueueFull()) {
        /* push into queue */
        // TODO implement TRANSACTION_EXPIRED
        uint16_t totalSize = 0;
        for(NeighborQueue<MAX_NEIGHBORS>::iterator it = neighborQueue.begin(); it != neighborQueue.end(); ++it) {
            totalSize += it->queueSize;
        }
        LOG_INFO("NeighborQueue is at " << totalSize << "/" << TOTAL_GTS_QUEUE_SIZE << ".");
        neighborQueue.pushBack(destIt, msg);
        return true;
    } else {
        /* queue full */
        LOG_WARN("NeighborQueue is full!");
        numUpperPacketsDroppedFullQueue++;
        return false;
    }
}

bool MessageDispatcher::sendInCAP(DSMEMessage* msg) {
    numUpperPacketsForCAP++;
    LOG_INFO("Inserting message into CAP queue.");
    if (msg->getHeader().getSrcAddrMode() != EXTENDED_ADDRESS && !(this->dsme.getMAC_PIB().macAssociatedPANCoord)) {
        LOG_WARN("Message dropped due to missing association!");
        // TODO document this behaviour
        // TODO send appropriate MCPS confirm or better remove this handling and implement TRANSACTION_EXPIRED
        return false;
    }

    if (!this->dsme.getCapLayer().pushMessage(msg)) {
        LOG_INFO("CAP queue full!");
        return false;
    }

    return true;
}


void MessageDispatcher::handleGTS() {
    if (currentACTElement != dsme.getMAC_PIB().macDSMEACT.end() && currentACTElement->getSuperframeID() == dsme.getCurrentSuperframe()
            && currentACTElement->getGTSlotID() == dsme.getCurrentSlot() - (dsme.getMAC_PIB().helper.getFinalCAPSlot() + 1)) {

        if (currentACTElement->getDirection() == RX) { // also if INVALID or UNCONFIRMED!
            //LOG_INFO("Waiting to receive from " << currentACTElement->getAddress())
        } else if (currentACTElement->getState() == VALID) {
            // transmit from gtsQueue
            DSME_ASSERT(lastSendGTSNeighbor == neighborQueue.end());

            lastSendGTSNeighbor = neighborQueue.findByAddress(IEEE802154MacAddress(currentACTElement->getAddress()));
            if (neighborQueue.isQueueEmpty(lastSendGTSNeighbor)) {
                lastSendGTSNeighbor = neighborQueue.end();
                numUnusedTxGts++;
            } else {
                DSMEMessage* msg = neighborQueue.front(lastSendGTSNeighbor);

                //LOG_INFO("send in GTS " << msg->getHeader().getDestAddr().getShortAddress());

                DSME_ASSERT(dsme.getMAC_PIB().helper.getSymbolsPerSlot() >= msg->getTotalSymbols());

                if (!dsme.getAckLayer().sendButKeep(msg, doneGTS)) {
                    // message could not be sent
                    DSME_ASSERT(false); // the ACK layer is still busy, but we have an assigned slot, something is wrong, probably DSMECSMA::symbolsRequired()
                    neighborQueue.popFront(lastSendGTSNeighbor);
                    lastSendGTSNeighbor = neighborQueue.end();
                }

                // statistics
                numTxGtsFrames++;
            }
        }
    }
}

void MessageDispatcher::handleGTSFrame(DSMEMessage* msg) {
    DSME_ASSERT(currentACTElement != dsme.getMAC_PIB().macDSMEACT.end());

    numRxGtsFrames++;
    numUnusedRxGts--;

    if (currentACTElement->getSuperframeID() == dsme.getCurrentSuperframe()
            && currentACTElement->getGTSlotID() == dsme.getCurrentSlot() - (dsme.getMAC_PIB().helper.getFinalCAPSlot() + 1)) {
        // According to 5.1.10.5.3
        currentACTElement->resetIdleCounter();
    }

    createDataIndication(msg);
}

void MessageDispatcher::onCSMASent(DSMEMessage* msg, DataStatus::Data_Status status, uint8_t numBackoffs) {
    if (msg->receivedViaMCPS) {
        mcps_sap::DATA_confirm_parameters params;
        params.msduHandle = msg;
        params.Timestamp = 0; // TODO
        params.RangingReceived = false;
        params.status = status;
        params.numBackoffs = numBackoffs;
        params.dsn = msg->getHeader().getSequenceNumber();
        params.AckPayload = nullptr;
        params.gtsTX = false;
        this->dsme.getMCPS_SAP().getDATA().notify_confirm(params);
    } else {
        if (msg->getHeader().getFrameType() == IEEE802154eMACHeader::FrameType::COMMAND) {
            MACCommand cmd;
            cmd.decapsulateFrom(msg);

            LOG_DEBUG("cmdID " << (uint16_t)cmd.getCmdId());

            switch (cmd.getCmdId()) {
            case ASSOCIATION_REQUEST:
            case ASSOCIATION_RESPONSE:
            case DISASSOCIATION_NOTIFICATION:
                this->dsme.getAssociationManager().onCSMASent(msg, cmd.getCmdId(), status, numBackoffs);
                break;
            case DATA_REQUEST:
            case DSME_ASSOCIATION_REQUEST:
            case DSME_ASSOCIATION_RESPONSE:
                DSME_ASSERT(false);
                // TODO handle correctly
                this->dsme.getPlatform().releaseMessage(msg);
                break;
            case BEACON_REQUEST:
            case DSME_BEACON_ALLOCATION_NOTIFICATION:
            case DSME_BEACON_COLLISION_NOTIFICATION:
                this->dsme.getBeaconManager().onCSMASent(msg, cmd.getCmdId(), status, numBackoffs);
                break;
            case DSME_GTS_REQUEST:
            case DSME_GTS_REPLY:
            case DSME_GTS_NOTIFY:
                this->dsme.getGTSManager().onCSMASent(msg, cmd.getCmdId(), status, numBackoffs);
                break;
            }
        } else {
            this->dsme.getPlatform().releaseMessage(msg);
        }
    }
}


void MessageDispatcher::sendDoneGTS(enum AckLayerResponse response, DSMEMessage* msg) {
    LOG_DEBUG("sendDoneGTS");

    DSME_ASSERT(lastSendGTSNeighbor != neighborQueue.end());
    DSME_ASSERT(msg == neighborQueue.front(lastSendGTSNeighbor));
    neighborQueue.popFront(lastSendGTSNeighbor);
    lastSendGTSNeighbor = neighborQueue.end();

    mcps_sap::DATA_confirm_parameters params;
    params.msduHandle = msg;
    params.Timestamp = 0; // TODO
    params.RangingReceived = false;
    params.gtsTX = true;

    switch (response) {
        case AckLayerResponse::NO_ACK_REQUESTED:
        case AckLayerResponse::ACK_SUCCESSFUL:
            params.status = DataStatus::SUCCESS;
            break;
        case AckLayerResponse::ACK_FAILED:
            currentACTElement->incrementIdleCounter();
            params.status = DataStatus::NO_ACK;
            break;
        case AckLayerResponse::SEND_FAILED:
            LOG_DEBUG("SEND_FAILED during GTS");
            currentACTElement->incrementIdleCounter();
            params.status = DataStatus::CHANNEL_ACCESS_FAILURE;
            break;
        default:
            DSME_ASSERT(false);
    }

    params.numBackoffs = 0;
    params.dsn = msg->getHeader().getSequenceNumber();
    params.AckPayload = nullptr;
    this->dsme.getMCPS_SAP().getDATA().notify_confirm(params);
}

} /* dsme */
