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

#include "./MessageDispatcher.h"

#include "../../../dsme_platform.h"
#include "../../../dsme_settings.h"
#include "../../helper/DSMEDelegate.h"
#include "../../helper/Integers.h"
#include "../../interfaces/IDSMEMessage.h"
#include "../../interfaces/IDSMEPlatform.h"
#include "../../mac_services/DSME_Common.h"
#include "../../mac_services/MacDataStructures.h"
#include "../../mac_services/dataStructures/DSMEAllocationCounterTable.h"
#include "../../mac_services/dataStructures/IEEE802154MacAddress.h"
#include "../../mac_services/mcps_sap/DATA.h"
#include "../../mac_services/mcps_sap/MCPS_SAP.h"
#include "../../mac_services/pib/dsme_mac_constants.h"
#include "../../mac_services/pib/MAC_PIB.h"
#include "../../mac_services/pib/PHY_PIB.h"
#include "../../mac_services/pib/PIBHelper.h"
#include "../DSMELayer.h"
#include "../ackLayer/AckLayer.h"
#include "../associationManager/AssociationManager.h"
#include "../beaconManager/BeaconManager.h"
#include "../capLayer/CAPLayer.h"
#include "../gtsManager/GTSManager.h"
#include "../messages/IEEE802154eMACHeader.h"
#include "../messages/MACCommand.h"
#include "../../helper/GackHelper.h"

namespace dsme {

MessageDispatcher::MessageDispatcher(DSMELayer& dsme)
    : dsme(dsme),
      currentACTElement(nullptr, nullptr),
      doneGTS(DELEGATE(&MessageDispatcher::sendDoneGTS, *this)),
      dsmeAckFrame(nullptr),
      lastSendGTSNeighbor(neighborQueue.end()),
      gackHelper()
      {
}

MessageDispatcher::~MessageDispatcher() {
    for(NeighborQueue<MAX_NEIGHBORS>::iterator it = neighborQueue.begin(); it != neighborQueue.end(); ++it) {
        while(!this->neighborQueue.isQueueEmpty(it)) {
            IDSMEMessage* msg = neighborQueue.popFront(it);
            this->dsme.getPlatform().releaseMessage(msg);
        }
    }
    for(NeighborQueue<MAX_NEIGHBORS>::iterator it = retransmissionQueue.begin(); it != retransmissionQueue.end(); ++it) {
        while(!this->retransmissionQueue.isQueueEmpty(it)) {
            IDSMEMessage* msg = retransmissionQueue.popFront(it);
            this->dsme.getPlatform().releaseMessage(msg);
        }
    }
}

void MessageDispatcher::initialize(void) {
    currentACTElement = dsme.getMAC_PIB().macDSMEACT.end();
    gackHelper.init(dsme.getMAC_PIB().macSuperframeOrder);
    return;
}

void MessageDispatcher::reset(void) {
    currentACTElement = dsme.getMAC_PIB().macDSMEACT.end();

    for(NeighborQueue<MAX_NEIGHBORS>::iterator it = neighborQueue.begin(); it != neighborQueue.end(); ++it) {
        while(!this->neighborQueue.isQueueEmpty(it)) {
            IDSMEMessage* msg = neighborQueue.popFront(it);
            mcps_sap::DATA_confirm_parameters params;
            params.msduHandle = msg;
            params.timestamp = 0;
            params.rangingReceived = false;
            params.gtsTX = true;
            params.status = DataStatus::TRANSACTION_EXPIRED;
            params.numBackoffs = 0;
            this->dsme.getMCPS_SAP().getDATA().notify_confirm(params);
        }
    }
    for(NeighborQueue<MAX_NEIGHBORS>::iterator it = retransmissionQueue.begin(); it != retransmissionQueue.end(); ++it) {
        while(!this->retransmissionQueue.isQueueEmpty(it)) {
            IDSMEMessage* msg = retransmissionQueue.popFront(it);
            mcps_sap::DATA_confirm_parameters params;
            params.msduHandle = msg;
            params.timestamp = 0;
            params.rangingReceived = false;
            params.gtsTX = true;
            params.status = DataStatus::TRANSACTION_EXPIRED;
            params.numBackoffs = 0;
            this->dsme.getMCPS_SAP().getDATA().notify_confirm(params);
        }
    }
    while(this->neighborQueue.getNumNeighbors() > 0) {
        NeighborQueue<MAX_NEIGHBORS>::iterator it = this->neighborQueue.begin();
        this->neighborQueue.eraseNeighbor(it);
        it = this->retransmissionQueue.begin();
        this->retransmissionQueue.eraseNeighbor(it);
    }

    return;
}


void MessageDispatcher::sendDoneGTS(enum AckLayerResponse response, IDSMEMessage* msg) {
    LOG_DEBUG("sendDoneGTS");

    DSME_ASSERT(lastSendGTSNeighbor != neighborQueue.end());
    DSME_ASSERT(msg == neighborQueue.front(lastSendGTSNeighbor));

    DSMEAllocationCounterTable& act = this->dsme.getMAC_PIB().macDSMEACT;
    DSME_ASSERT(this->currentACTElement != act.end());

    this->dsme.getEventDispatcher().setupIFSTimer(msg->getTotalSymbols() > aMaxSIFSFrameSize);

    if(response != AckLayerResponse::NO_ACK_REQUESTED && response != AckLayerResponse::ACK_SUCCESSFUL) {
        currentACTElement->incrementIdleCounter();

        // not successful -> retry?
        if(msg->getRetryCounter() < dsme.getMAC_PIB().macMaxFrameRetries) {
            msg->increaseRetryCounter();
            //finalizeGTSTransmission();
            LOG_DEBUG("sendDoneGTS - retry");
            return; // will stay at front of queue
        }
    }

    if(response == AckLayerResponse::ACK_FAILED || response == AckLayerResponse::ACK_SUCCESSFUL) {
        this->dsme.getPlatform().signalAckedTransmissionResult(response == AckLayerResponse::ACK_SUCCESSFUL, msg->getRetryCounter() + 1, msg->getHeader().getDestAddr());
    }

    bool signalUpperLayer = false;
    if(!retransmissionQueue.isQueueFull()) {
        IDSMEMessage* msg = neighborQueue.popFront(lastSendGTSNeighbor);
        const IEEE802154MacAddress &addr = lastSendGTSNeighbor->address;

        /* Keep track of how many packets have been send in each GTS ths SF*/
        this->gackHelper.transmittedPacketsGTS[currentACTElement.node()->content.getGTSlotID()]++;

        NeighborQueue<MAX_NEIGHBORS>::iterator retransmissionQueueNeighbor = retransmissionQueue.findByAddress(addr);
        retransmissionQueue.pushBack(retransmissionQueueNeighbor, msg); // JND
    } else {
        neighborQueue.popFront(lastSendGTSNeighbor);
        signalUpperLayer = true;
    }
    this->preparedMsg = nullptr;

    /* STATISTICS */
    uint16_t totalSize = 0;
    for(NeighborQueue<MAX_NEIGHBORS>::iterator it = neighborQueue.begin(); it != neighborQueue.end(); ++it) {
        totalSize += it->queueSize;
    }
    this->dsme.getPlatform().signalQueueLength(totalSize);
    /* END STATISTICS */

    mcps_sap::DATA_confirm_parameters params;
    params.msduHandle = msg;
    params.timestamp = 0; // TODO
    params.rangingReceived = false;
    params.gtsTX = true;

    switch(response) {
        case AckLayerResponse::NO_ACK_REQUESTED:
        case AckLayerResponse::ACK_SUCCESSFUL:
            LOG_DEBUG("sendDoneGTS - success");
            params.status = DataStatus::SUCCESS;
            break;
        case AckLayerResponse::ACK_FAILED:
            DSME_ASSERT(this->currentACTElement != this->dsme.getMAC_PIB().macDSMEACT.end());
            currentACTElement->incrementIdleCounter();
            params.status = DataStatus::NO_ACK;
            break;
        case AckLayerResponse::SEND_FAILED:
            LOG_DEBUG("SEND_FAILED during GTS");
            DSME_ASSERT(this->currentACTElement != this->dsme.getMAC_PIB().macDSMEACT.end());
            currentACTElement->incrementIdleCounter();
            params.status = DataStatus::CHANNEL_ACCESS_FAILURE;
            break;
        case AckLayerResponse::SEND_ABORTED:
            LOG_DEBUG("SEND_ABORTED during GTS");
            params.status = DataStatus::TRANSACTION_EXPIRED;
            break;
        default:
            DSME_ASSERT(false);
    }

    params.numBackoffs = 0;
    if(signalUpperLayer || !msg->getHeader().getGack()) {
        this->dsme.getMCPS_SAP().getDATA().notify_confirm(params);
    }

    if(!this->multiplePacketsPerGTS || !prepareNextMessageIfAny()) {
        /* '-> prepare next frame for transmission after one IFS */
        finalizeGTSTransmission();
    }
}

void MessageDispatcher::finalizeGTSTransmission() {
    LOG_DEBUG("Finalizing transmission for " << this->currentACTElement->getGTSlotID() << " " << this->currentACTElement->getSuperframeID() << " " << this->currentACTElement->getChannel());
    transceiverOffIfAssociated();
    this->dsme.getEventDispatcher().stopIFSTimer();
    this->preparedMsg = nullptr;    // TODO correct here?
    this->lastSendGTSNeighbor = this->neighborQueue.end();
    this->currentACTElement = this->dsme.getMAC_PIB().macDSMEACT.end();
    if(this->numTxGtsFrames > 0) this->dsme.getPlatform().signalPacketsTXPerSlot(this->numTxGtsFrames);
    if(this->numRxGtsFrames > 0) this->dsme.getPlatform().signalPacketsRXPerSlot(this->numRxGtsFrames);
    this->numTxGtsFrames = 0;
    this->numRxGtsFrames = 0;
}

void MessageDispatcher::onCSMASent(IDSMEMessage* msg, DataStatus::Data_Status status, uint8_t numBackoffs, uint8_t transmissionAttempts) {
    if(status == DataStatus::Data_Status::NO_ACK || status == DataStatus::Data_Status::SUCCESS) {
        if(msg->getHeader().isAckRequested() && !msg->getHeader().getDestAddr().isBroadcast()) {
            this->dsme.getPlatform().signalAckedTransmissionResult(status == DataStatus::Data_Status::SUCCESS, transmissionAttempts,
                                                                   msg->getHeader().getDestAddr());
        }
    }

    if(msg->getReceivedViaMCPS()) {
        mcps_sap::DATA_confirm_parameters params;
        params.msduHandle = msg;
        params.timestamp = 0; // TODO
        params.rangingReceived = false;
        params.status = status;
        params.numBackoffs = numBackoffs;
        params.gtsTX = false;
        this->dsme.getMCPS_SAP().getDATA().notify_confirm(params);
    } else {
        if(msg->getHeader().getFrameType() == IEEE802154eMACHeader::FrameType::COMMAND) {
            MACCommand cmd;
            cmd.decapsulateFrom(msg);

            LOG_DEBUG("cmdID " << (uint16_t)cmd.getCmdId());

            switch(cmd.getCmdId()) {
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

bool MessageDispatcher::sendInGTS(IDSMEMessage* msg, NeighborQueue<MAX_NEIGHBORS>::iterator destIt) {
    DSME_ASSERT(!msg->getHeader().getDestAddr().isBroadcast());
    DSME_ASSERT(this->dsme.getMAC_PIB().macAssociatedPANCoord);
    DSME_ASSERT(destIt != neighborQueue.end());

    numUpperPacketsForGTS++;

    if(!neighborQueue.isQueueFull()) {
        /* push into queue */
        // TODO implement TRANSACTION_EXPIRED
        uint16_t totalSize = 0;
        for(NeighborQueue<MAX_NEIGHBORS>::iterator it = neighborQueue.begin(); it != neighborQueue.end(); ++it) {
            totalSize += it->queueSize;
        }
        LOG_INFO("NeighborQueue is at " << totalSize << "/" << TOTAL_GTS_QUEUE_SIZE << ".");
        neighborQueue.pushBack(destIt, msg);
        this->dsme.getPlatform().signalQueueLength(totalSize+1);
        return true;
    } else {
        /* queue full */
        LOG_INFO("NeighborQueue is full!");
        numUpperPacketsDroppedFullQueue++;
        return false;
    }
}

bool MessageDispatcher::sendInCAP(IDSMEMessage* msg) {
    LOG_INFO("Inserting message into CAP queue.");
    if(msg->getHeader().getSrcAddrMode() != EXTENDED_ADDRESS && !(this->dsme.getMAC_PIB().macAssociatedPANCoord)) {
        LOG_INFO("Message dropped due to missing association!");
        // TODO document this behaviour
        // TODO send appropriate MCPS confirm or better remove this handling and implement TRANSACTION_EXPIRED
        return false;
    }
    if(!this->dsme.getCapLayer().pushMessage(msg)) {
        LOG_INFO("CAP queue full!");
        return false;
    }
    if(msg->getHeader().getFrameType() == IEEE802154eMACHeader::FrameType::COMMAND) {
        numUpperPacketsForCAP++;
    }
    return true;
}

void MessageDispatcher::receive(IDSMEMessage* msg) {
    IEEE802154eMACHeader macHdr = msg->getHeader();

    switch(macHdr.getFrameType()) {
        case IEEE802154eMACHeader::FrameType::BEACON: {
            LOG_INFO("BEACON from " << macHdr.getSrcAddr().getShortAddress() << " " << macHdr.getSrcPANId() << " " << dsme.getCurrentSuperframe() << ".");
            this->dsme.getBeaconManager().handleBeacon(msg);
            this->dsme.getPlatform().releaseMessage(msg);
            break;
        }

        case IEEE802154eMACHeader::FrameType::COMMAND: {
            MACCommand cmd;
            cmd.decapsulateFrom(msg);
            switch(cmd.getCmdId()) {
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
                    LOG_ERROR("Invalid cmd ID " << (uint16_t)cmd.getCmdId());
                    // DSME_ASSERT(false);
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
            LOG_ERROR((uint16_t)macHdr.getFrameType());
            dsme.getPlatform().releaseMessage(msg);
        }
    }
    return;
}

bool MessageDispatcher::handlePreSlotEvent(uint8_t nextSlot, uint8_t nextSuperframe, uint8_t nextMultiSuperframe) {
    // Prepare next slot
    // Switch to next slot channel and radio mode
    DSMEAllocationCounterTable& act = this->dsme.getMAC_PIB().macDSMEACT;
    if(this->currentACTElement != act.end()) {
        if(this->currentACTElement->getDirection() == Direction::RX) {
            this->currentACTElement = act.end();
        } else {
            // Rarely happens, only if the sendDoneGTS is delayed
            // Then skip this preSlotEvent
            LOG_DEBUG("Previous slot did not finish until preslot event: slot " << (int)nextSlot << " SF " << (int)nextSuperframe);
            DSME_SIM_ASSERT(false);
            return false;
        }
    }

    if(nextSlot > this->dsme.getMAC_PIB().helper.getFinalCAPSlot(nextSuperframe)) {
        /* '-> next slot will be GTS */

        unsigned nextGTS = nextSlot - (this->dsme.getMAC_PIB().helper.getFinalCAPSlot(nextSuperframe) + 1);
        if(act.isAllocated(nextSuperframe, nextGTS)) {
            /* '-> this slot might be used */

            this->currentACTElement = act.find(nextSuperframe, nextGTS);
            DSME_ASSERT(this->currentACTElement != act.end());
            // For TX currentACTElement will be reset in finalizeGTSTransmission, called by
            // either handleGTS if nothing is to send or by sendDoneGTS.
            // For RX it is reset in the next handlePreSlotEvent.   TODO: is the reset actually required?

            // For RX also if INVALID or UNCONFIRMED!
            if((this->currentACTElement->getState() == VALID) || (this->currentACTElement->getDirection() == Direction::RX)) {
                this->dsme.getPlatform().turnTransceiverOn();

                if(dsme.getMAC_PIB().macChannelDiversityMode == Channel_Diversity_Mode::CHANNEL_ADAPTATION) {
                    this->dsme.getPlatform().setChannelNumber(this->dsme.getMAC_PIB().helper.getChannels()[this->currentACTElement->getChannel()]);
                } else {
                    uint8_t channel = nextHoppingSequenceChannel(nextSlot, nextSuperframe, nextMultiSuperframe);
                    this->dsme.getPlatform().setChannelNumber(channel);
                }
            }

            // statistic
            if(this->currentACTElement->getDirection() == RX) {
                this->numUnusedRxGts++; // gets PURGE.cc decremented on actual reception
            }
        } else {
            /* '-> nothing to do during this slot */
            DSME_ASSERT(this->currentACTElement == act.end());
            transceiverOffIfAssociated();
        }
    } else if(nextSlot == 0) {
        /* '-> beacon slots are handled by the BeaconManager */
        DSME_ASSERT(this->currentACTElement == act.end());
    } else if(nextSlot == 1) {
        /* '-> next slot will be CAP */

        if(!this->dsme.getMAC_PIB().macCapReduction || nextSuperframe == 0) {
            /* '-> active CAP slot */

            this->dsme.getPlatform().turnTransceiverOn();
            this->dsme.getPlatform().setChannelNumber(this->dsme.getPHY_PIB().phyCurrentChannel);
        } else {
            /* '-> CAP reduction */
            transceiverOffIfAssociated();
        }
    }

    return true;
}

uint8_t MessageDispatcher::nextHoppingSequenceChannel(uint8_t nextSlot, uint8_t nextSuperframe, uint8_t nextMultiSuperframe) {
    uint16_t hoppingSequenceLength = this->dsme.getMAC_PIB().macHoppingSequenceLength;
    uint8_t ebsn = 0; // this->dsme.getMAC_PIB().macPanCoordinatorBsn;    //TODO is this set correctly
    uint16_t sdIndex = nextSuperframe + this->dsme.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe() * nextMultiSuperframe;
    uint8_t numGTSlots = this->dsme.getMAC_PIB().helper.getNumGTSlots(sdIndex);

    uint8_t slotId = this->currentACTElement->getGTSlotID();
    uint16_t channelOffset = this->currentACTElement->getChannel();

    uint8_t channel =
        this->dsme.getMAC_PIB().macHoppingSequenceList[(sdIndex * numGTSlots + slotId + channelOffset + ebsn) % hoppingSequenceLength];
    LOG_INFO("Using channel " << channel << " - numGTSlots: " << numGTSlots << " EBSN: " << ebsn << " sdIndex: " << sdIndex
                              << " slot: " << slotId << " Superframe " << nextSuperframe << " channelOffset: " << channelOffset
                              << " Direction: " << currentACTElement->getDirection());
    return channel;
}

bool MessageDispatcher::handleSlotEvent(uint8_t slot, uint8_t superframe, int32_t lateness) {
    if(slot > dsme.getMAC_PIB().helper.getFinalCAPSlot(superframe)) {
        handleGTS(lateness);
    }
    return true;
}

bool MessageDispatcher::handleIFSEvent(int32_t lateness) {
    /* Neighbor and slot have to be valid at this point */
    DSME_ASSERT(this->lastSendGTSNeighbor != this->neighborQueue.end());
    DSME_ASSERT(this->currentACTElement != this->dsme.getMAC_PIB().macDSMEACT.end());
    DSME_ASSERT(this->currentACTElement->getSuperframeID() == this->dsme.getCurrentSuperframe() && this->currentACTElement->getGTSlotID()
      == this->dsme.getCurrentSlot() - (this->dsme.getMAC_PIB().helper.getFinalCAPSlot(this->dsme.getCurrentSuperframe())+1));

    if(!sendPreparedMessage()) {
        finalizeGTSTransmission();
    }

    return true;
}


void MessageDispatcher::handleGTS(int32_t lateness) {
    if(this->currentACTElement != this->dsme.getMAC_PIB().macDSMEACT.end() && this->currentACTElement->getSuperframeID() == this->dsme.getCurrentSuperframe() &&
       this->currentACTElement->getGTSlotID() ==
           this->dsme.getCurrentSlot() - (this->dsme.getMAC_PIB().helper.getFinalCAPSlot(dsme.getCurrentSuperframe()) + 1)) {
        /* '-> this slot matches the prepared ACT element */

        if(this->currentACTElement->getDirection() == RX) { // also if INVALID or UNCONFIRMED!
            /* '-> a message may be received during this slot */

        } else if(this->currentACTElement->getState() == VALID) {
            /* '-> if any messages are queued for this link, send one */

            DSME_ASSERT(this->lastSendGTSNeighbor == this->neighborQueue.end());

            IEEE802154MacAddress adr = IEEE802154MacAddress(this->currentACTElement->getAddress());
            this->lastSendGTSNeighbor = this->neighborQueue.findByAddress(IEEE802154MacAddress(this->currentACTElement->getAddress()));
            if(this->lastSendGTSNeighbor == this->neighborQueue.end()) {
                /* '-> the neighbor associated with the current slot does not exist */

                LOG_ERROR("neighborQueue.size: " << ((uint8_t) this->neighborQueue.getNumNeighbors()));
                LOG_ERROR("neighbor address: " << HEXOUT << adr.a1() << ":" << adr.a2() << ":" << adr.a3() << ":" << adr.a4() << DECOUT);
                for(auto it : this->neighborQueue) {
                    LOG_ERROR("neighbor address: " << HEXOUT << it.address.a1() << ":" << it.address.a2() << ":" << it.address.a3() << ":" << it.address.a4()
                                                   << DECOUT);
                }
                DSME_ASSERT(false);
            }

            bool success = prepareNextMessageIfAny();
            // JND: preparedMsg->getHeader().setAckRequest(true);
            // JND: preparedMsg->getHeader().setGroupAckRequest(true);
            LOG_DEBUG(success);
            if(success) {
                /* '-> a message is queued for transmission */
                success = sendPreparedMessage();
            }
            LOG_DEBUG(success);

            if(success == false) {
                /* '-> no message to be sent */
                LOG_DEBUG("MessageDispatcher: Could not transmit any packet in GTS");
                this->numUnusedTxGts++;
                finalizeGTSTransmission();
            }
        } else {
            finalizeGTSTransmission();
        }
    }
}
void MessageDispatcher::handleGTSFrame(IDSMEMessage* msg) {
    DSME_ASSERT(currentACTElement != dsme.getMAC_PIB().macDSMEACT.end());

    numRxGtsFrames++;
    numUnusedRxGts--;

    if(currentACTElement->getSuperframeID() == dsme.getCurrentSuperframe() &&
       currentACTElement->getGTSlotID() == dsme.getCurrentSlot() - (dsme.getMAC_PIB().helper.getFinalCAPSlot(dsme.getCurrentSuperframe()) + 1)) {
        // According to 5.1.10.5.3
        currentACTElement->resetIdleCounter();
    }


    //Checks if message contains lastMessageIE,
    InformationElement* iePointer = nullptr;
    if(msg->getHeader().getIEListPresent() == true){
        if(msg->getHeader().ieQueue.getIEByID(0x10, iePointer)){
            if(dynamic_cast<lastMessageIE*>(iePointer)->isLastMessage){
                   turnOff = true;
                   LOG_INFO("Last Message true");//TODO: remove
           }
        }
        LOG_INFO("Information Element present");//TODO: remove
    }


    createDataIndication(msg);
}

void MessageDispatcher::handleAckTransmitted(){
    LOG_INFO("handleAckTransmitted");
    if(currentACTElement != dsme.getMAC_PIB().macDSMEACT.end()) {
        if(turnOff) {
            LOG_INFO("turnOFF");
            turnOff = false;
            finalizeGTSTransmission(); //dsme.getMessageDispatcher().
            dsme.getPlatform().delayedTurnTransceiverOff();
        }
   }
}

void MessageDispatcher::handleGACK(IEEE802154eMACHeader& header, GackCmd& gack) {
    DSMEAllocationCounterTable& act = this->dsme.getMAC_PIB().macDSMEACT;
    uint16_t currentSuperframe = ((dsme.getCurrentSuperframe()-1) + dsme.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe()) % dsme.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe();
    uint16_t cfpSlotsPerMsf = dsme.getMAC_PIB().helper.getNumGTSlots(currentSuperframe); // Sf
    DSME_ASSERT(gack.getGackMap().length() % cfpSlotsPerMsf == 0);
    uint8_t packetsPerGTS = gack.getGackMap().length() / cfpSlotsPerMsf;

    for(uint8_t gtsId=0; gtsId<cfpSlotsPerMsf; gtsId++) {
        if(act.isAllocated(currentSuperframe, gtsId)) {
            IEEE802154MacAddress addr = IEEE802154MacAddress(act.find(currentSuperframe, gtsId)->getAddress());
            if(addr == header.getSrcAddr()) {

            /* '-> we transmitted in this gts to the device that sent the gack */
            const IEEE802154MacAddress &addr = header.getSrcAddr();
            NeighborQueue<MAX_NEIGHBORS>::iterator neighborQueueNeighbor = neighborQueue.findByAddress(addr);
            NeighborQueue<MAX_NEIGHBORS>::iterator retransmissionQueueNeighbor = retransmissionQueue.findByAddress(addr);
            DSME_ASSERT(neighborQueueNeighbor != neighborQueue.end() && retransmissionQueueNeighbor != retransmissionQueue.end());

            for(bit_vector_size_t i = packetsPerGTS * gtsId; i<packetsPerGTS * (gtsId) + gackHelper.transmittedPacketsGTS[gtsId]; i++) {
                /* '-> check for all bits of the bitmap */

                if(retransmissionQueue.isQueueEmpty(retransmissionQueueNeighbor)) {
                    break;
                }

                IDSMEMessage* msg = retransmissionQueue.popFront(retransmissionQueueNeighbor);
                mcps_sap::DATA_confirm_parameters params;
                params.msduHandle = msg;
                params.timestamp = 0; // TODO
                params.rangingReceived = false;
                params.gtsTX = true;

                if(gack.getGackMap().get(i) == true) {
                    /* '-> successful transmission */
                    params.status = DataStatus::SUCCESS;

                } else {
                    /* '-> failed transmission */
                    if(msg->getRetryCounter() < dsme.getMAC_PIB().macMaxFrameRetries) {
                        msg->increaseRetryCounter();
                        LOG_DEBUG("handleGACK - retry");
                        if(!neighborQueue.isQueueFull()) {
                            DSME_ASSERT(msg != nullptr);
                            neighborQueue.pushBack(neighborQueueNeighbor, msg);
                        }
                        preparedMsg = nullptr;
                        continue;
                    } else {
                        // should not happen MessageHelper.cc 269
                        params.status = DataStatus::CHANNEL_ACCESS_FAILURE;
                    }
                }

                this->dsme.getMCPS_SAP().getDATA().notify_confirm(params);
            }
            }
        }
    }
}


// Function to determine if there is any message that can be transmitted or not.
// Returns: True: 1. If there is a pending message (e.g. from failed transmission) to transmit
//                2. If there exits any message in the queue to transmit.
//          False otherwise
bool MessageDispatcher::prepareNextMessageIfAny() {
    bool result = false;
    bool checkTimeToSendMessage = false;

    // check if there exists a pending Message
    if(this->preparedMsg) {
        checkTimeToSendMessage = true; // if true, set a flag to to check if pending message can be sent in remaining slot time
    } else if (this->neighborQueue.isQueueEmpty(this->lastSendGTSNeighbor)) {  // there is no pending message, then check if the queue is empty (i.e. there is not any message to transmit to a target neighbor)
        this->preparedMsg = nullptr; // reset value of pending Message
        checkTimeToSendMessage = false;
        result = false;
    } else { // if there is a message to send retrieve a copy of it from the queue and set the flag to check if possible to send the message
        checkTimeToSendMessage = true;
        // JND if(!retransmissionQueue.empty())  {
        //      this->preparedMsg = retransmissionQueue.front(this->lastSendGTSNeighbor);
        //      } else  {
        this->preparedMsg = neighborQueue.front(this->lastSendGTSNeighbor);
    }

    if(this->preparedMsg != nullptr){
        if(this->preparedMsg->getHeader().getFrameControl().frameType == IEEE802154eMACHeader::DATA){
            this->preparedMsg->getHeader().setGack(true);
            this->preparedMsg->getHeader().setAckRequest(false);
        }
    }

    if(this->neighborQueue.getPacketsInQueue(this->lastSendGTSNeighbor) == 1){
        //preparedMsg->getHeader().setIEListPresent(true);
        LOG_INFO("last Packet in queue");
        //lastMessageIE lmIE;
        //lmIE.isLastMessage = true;
        //preparedMsg->getHeader().ieQueue.push(lmIE);
        //LOG_INFO("lastMessageIE added to queue");
    }

    if(checkTimeToSendMessage) {//if the timming for transmission must be checked
        // determined how long the transmission of the preparedMessage will take.
        uint8_t ifsSymbols = this->preparedMsg->getTotalSymbols() <= aMaxSIFSFrameSize ? const_redefines::macSIFSPeriod : const_redefines::macLIFSPeriod;
        uint32_t duration = this->preparedMsg->getTotalSymbols() + this->dsme.getMAC_PIB().helper.getAckWaitDuration() + ifsSymbols;
        // check if the remaining slot time is enough to transmit the prepared packet
        if(!this->dsme.isWithinTimeSlot(this->dsme.getPlatform().getSymbolCounter(), duration)) {
            LOG_DEBUG("No packet prepared (remaining slot time insufficient)");
            this->preparedMsg = nullptr; // reset value of pending Message
            result = false; // there is no enough time, no transmission will take place
        } else {
            result = true; // there is time, then proceed
        }
    }
    return result;
}

bool MessageDispatcher::sendPreparedMessage() {
    DSME_ASSERT(this->preparedMsg);
    DSME_ASSERT(this->dsme.getMAC_PIB().helper.getSymbolsPerSlot() >= this->preparedMsg->getTotalSymbols() + this->dsme.getMAC_PIB().helper.getAckWaitDuration() + 10 /* arbitrary processing delay */ + PRE_EVENT_SHIFT);

    uint8_t ifsSymbols = this->preparedMsg->getTotalSymbols() <= aMaxSIFSFrameSize ? const_redefines::macSIFSPeriod : const_redefines::macLIFSPeriod;
    uint32_t duration = this->preparedMsg->getTotalSymbols() + this->dsme.getMAC_PIB().helper.getAckWaitDuration() + ifsSymbols;
    /* '-> Duration for the transmission of the next frame */

    if(this->dsme.isWithinTimeSlot(this->dsme.getPlatform().getSymbolCounter(), duration)) {
        /* '-> Sufficient time to send message in remaining slot time */
        if (this->dsme.getAckLayer().prepareSendingCopy(this->preparedMsg, this->doneGTS)) {
            /* '-> Message transmission can be attempted */
            this->dsme.getAckLayer().sendNowIfPending();
            this->numTxGtsFrames++;
        } else {
            /* '-> Message could not be sent via ACKLayer (FAILED) */
            sendDoneGTS(AckLayerResponse::SEND_FAILED, this->preparedMsg);
        }
        return true;
    }
    LOG_DEBUG("No packet sent (remaining slot time insufficient)");
    return false;
}


void MessageDispatcher::createDataIndication(IDSMEMessage* msg) {
    IEEE802154eMACHeader& header = msg->getHeader();

    mcps_sap::DATA_indication_parameters params;

    params.msdu = msg;

    params.mpduLinkQuality = 0; // TODO link quality?
    params.dsn = header.getSequenceNumber();
    params.timestamp = msg->getStartOfFrameDelimiterSymbolCounter();
    params.securityLevel = header.isSecurityEnabled();

    params.dataRate = 0; // DSSS -> 0

    params.rangingReceived = NO_RANGING_REQUESTED;
    params.rangingCounterStart = 0;
    params.rangingCounterStop = 0;
    params.rangingTrackingInterval = 0;
    params.rangingOffset = 0;
    params.rangingFom = 0;

    this->dsme.getMCPS_SAP().getDATA().notify_indication(params);
}

void MessageDispatcher::transceiverOffIfAssociated() {
    LOG_INFO("Tranceiver turned off"); //TODO: remove
    if(this->dsme.getMAC_PIB().macAssociatedPANCoord) {
        this->dsme.getPlatform().turnTransceiverOff();
    } else {
        /* '-> do not turn off the transceiver while we might be scanning */
    }
}
}
