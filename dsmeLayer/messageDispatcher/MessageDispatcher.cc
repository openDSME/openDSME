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
#include "../messages/GTSGackCmd.h"

namespace dsme {

MessageDispatcher::MessageDispatcher(DSMELayer& dsme)
    : dsme(dsme),
      currentACTElement(nullptr, nullptr),
      doneGTS(DELEGATE(&MessageDispatcher::sendDoneGTS, *this)),
      dsmeAckFrame(nullptr),
      lastSendGTSNeighbor(neighborQueue.end()) {
}

MessageDispatcher::~MessageDispatcher() {
    for(NeighborQueue<MAX_NEIGHBORS>::iterator it = neighborQueue.begin(); it != neighborQueue.end(); ++it) {
        while(!this->neighborQueue.isQueueEmpty(it)) {
            IDSMEMessage* currentMsg = neighborQueue.popFront(it);
            this->dsme.getPlatform().releaseMessage(currentMsg);
            currentMsg = nullptr;
        }
    }
    for(NeighborQueue<MAX_NEIGHBORS>::iterator it = retransmissionQueue.begin(); it != retransmissionQueue.end(); ++it) {
        while(!this->retransmissionQueue.isQueueEmpty(it)) {
            IDSMEMessage* currentMsg = retransmissionQueue.popFront(it);
            this->dsme.getPlatform().releaseMessage(currentMsg);
            currentMsg = nullptr;
        }
    }
}

void MessageDispatcher::initialize(void) {
    currentACTElement = dsme.getMAC_PIB().macDSMEACT.end();
    //gackHelper.init(this->dsme.getMAC_PIB().helper.getNumberSuperframesPerGroupAckSlot(), this->dsme.getMAC_PIB().macSuperframeOrder, this->dsme.getMAC_PIB().macCapReduction?15:7); //15 GTSlots if CAPReduction is active
    //gackHelper.init(2,7,7); //TODO: fix initialization
    return;
}

void MessageDispatcher::reset(void) {
    currentACTElement = dsme.getMAC_PIB().macDSMEACT.end();

    for(NeighborQueue<MAX_NEIGHBORS>::iterator it = neighborQueue.begin(); it != neighborQueue.end(); ++it) {
        while(!this->neighborQueue.isQueueEmpty(it)) {
            IDSMEMessage* currentMsg = neighborQueue.popFront(it);
            mcps_sap::DATA_confirm_parameters params;
            params.msduHandle = currentMsg;
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
            IDSMEMessage* currentMsg = retransmissionQueue.popFront(it);
            mcps_sap::DATA_confirm_parameters params;
            params.msduHandle = currentMsg;
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
    }
    while(this->retransmissionQueue.getNumNeighbors() > 0) {
        NeighborQueue<MAX_NEIGHBORS>::iterator it = this->retransmissionQueue.begin();
        this->retransmissionQueue.eraseNeighbor(it);
    }
    gackBitmap.reset();
    return;
}


void MessageDispatcher::sendDoneGTS(enum AckLayerResponse response, IDSMEMessage* msg) {
    LOG_DEBUG("sendDoneGTS");

    if(this->currentACTElement->isGackGTS()){   //if it is a GACK-GTS
        this->dsme.getPlatform().releaseMessage(msg);
        finalizeGTSTransmission();
        return;
    }

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
    /*Insert sent message into retransmissionQueue for eventual retransmission*/
    if(currentACTElement->isGackEnabled()) {
        if(retransmissionQueue.isQueueFull()) {
            LOG_ERROR("RetransmissionQueue is full!");
            numRetransmissionPacketsDroppedFullQueue++;
            this->dsme.getPlatform().signalNumDroppedRetransmissionPackets(numRetransmissionPacketsDroppedFullQueue);
            neighborQueue.popFront(lastSendGTSNeighbor);
            signalUpperLayer = true;
        }else{
            IDSMEMessage* msg = neighborQueue.popFront(lastSendGTSNeighbor);
            const IEEE802154MacAddress &addr = lastSendGTSNeighbor->address;
            msg->txSlotId = currentACTElement->getGTSlotID();
            msg->txSuperframeId = currentACTElement->getSuperframeID();

            NeighborQueue<MAX_NEIGHBORS>::iterator retransmissionQueueNeighbor = retransmissionQueue.findByAddress(addr);
            DSME_ASSERT(retransmissionQueueNeighbor != retransmissionQueue.end());

            retransmissionQueue.pushBack(retransmissionQueueNeighbor, msg);
        }

    } else {  /*if not gackEnabled, release Message*/
        this->dsme.getPlatform().signalCFPAckDelay(this->dsme.getPlatform().getSymbolCounter()-msg->getHeader().getCreationTime()); //signal Ack Delay for statistics
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
    totalSize = 0;
    for(NeighborQueue<MAX_NEIGHBORS>::iterator it = retransmissionQueue.begin(); it != retransmissionQueue.end(); ++it) {
        totalSize += it->queueSize;
    }
    this->dsme.getPlatform().signalRetransmissionQueueLength(totalSize);
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
    if(signalUpperLayer){
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

    this->dsme.getPlatform().signalCAPAckDelay(this->dsme.getPlatform().getSymbolCounter()-msg->getHeader().getCreationTime()); //signal Ack Delay for statistics
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
                default:
                    DSME_ASSERT(false); //not handled!
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

    msg->getHeader().setCreationTime(this->dsme.getPlatform().getSymbolCounter()); //set Message Creation Time for statistics
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
        LOG_ERROR("NeighborQueue is full!");
        numUpperPacketsDroppedFullQueue++;
        this->dsme.getPlatform().signalNumDroppedPackets(numUpperPacketsDroppedFullQueue);
        return false;
    }
}

bool MessageDispatcher::sendInCAP(IDSMEMessage* msg) {
    LOG_INFO("Inserting message into CAP queue.");
    msg->getHeader().setCreationTime(this->dsme.getPlatform().getSymbolCounter()); //set Message Creation Time for statistics
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
    IEEE802154eMACHeader &macHdr = msg->getHeader();

    //if(currentACTElement->isGackEnabled()){
    if(macHdr.getIEList().contains(InformationElement::ID_gack)){   //msg with gack=true received, signal gackHelper
        //gackHelper.registerReceivedMessage(macHdr.getSequenceNumber(), currentACTElement->getSuperframeID(), currentACTElement->getGTSlotID());
        gackBitmap.registerPacket(macHdr.getSrcAddr(), macHdr.getSequenceNumber());
    }

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
                case CommandFrameIdentifier::DSME_GTS_GACK:
                    LOG_INFO("DSME-GTS-GACK from " << macHdr.getSrcAddr().getShortAddress() << ".");
                    handleGackReception(msg);
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
        //gackHelper.handleNewSuperframe(nextSuperframe, nextMultiSuperframe);
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

            if(this->currentACTElement->isGackGTS()){   //TX GACK
                bool success = prepareGackCommand();    //send GACK here, broadcast address
                LOG_DEBUG(success);
                if(success) {
                    /* '-> a message is queued for transmission */
                    success = sendPreparedMessage();
                }
                LOG_DEBUG(success);

                if(success == false) {
                    /* '-> no message to be sent */
                    LOG_DEBUG("MessageDispatcher: Could not transmit any packet in GACK GTS");
                    this->numUnusedTxGts++;
                    finalizeGTSTransmission();
                }
            }else{

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
    if(msg->getHeader().getIEListPresent()){
        if(msg->getHeader().getIEList().contains(InformationElement::ID_lastMessage)){
           turnOff = true;
           LOG_INFO("Last Message true");//TODO: remove
        }
        LOG_INFO("Information Element present");//TODO: remove
    }


    createDataIndication(msg);
}

void MessageDispatcher::handleAckTransmitted(){
    LOG_INFO("handleAckTransmitted");
    dsme.getPlatform().signalAckSent();
    if(currentACTElement != dsme.getMAC_PIB().macDSMEACT.end()) {
        if(turnOff) {
            LOG_INFO("turnOFF");
            turnOff = false;
            finalizeGTSTransmission(); //dsme.getMessageDispatcher().
            dsme.getPlatform().delayedTurnTransceiverOff();
        }
   }
}

bool MessageDispatcher::handleGackReception(IDSMEMessage* msg) {
    DSMEGACKBitmap bitmap;
    GTSGackCmd gackCmd(bitmap);
    gackCmd.decapsulateFrom(msg);
    char buf[5];
    sprintf(buf,"%i",gackCmd.getSerializationLength());
    LOG_INFO("GACK: received, size: "<< buf <<" Bytes");

    IEEE802154MacAddress srcAddr = msg->getHeader().getSrcAddr();

    return handleGackBitmap(bitmap, srcAddr);
}

bool MessageDispatcher::handleGackBitmap(DSMEGACKBitmap &bitmap, IEEE802154MacAddress &srcAddr) {
    //debug vars
    uint16_t numAckedPackets = 0, numRetransmittedPackets = 0;
    DSMEAllocationCounterTable& act = this->dsme.getMAC_PIB().macDSMEACT;

    NeighborQueue<MAX_NEIGHBORS>::iterator neighborQueueNeighbor = neighborQueue.findByAddress(srcAddr);
    NeighborQueue<MAX_NEIGHBORS>::iterator retransmissionQueueNeighbor = retransmissionQueue.findByAddress(srcAddr);

    if(retransmissionQueueNeighbor == retransmissionQueue.end()){
        LOG_INFO("GACK: neighbor does not exist yet. discarding.");
        return true;
    }

    IEEE802154MacAddress ownAddress = IEEE802154MacAddress(dsme.getMAC_PIB().macShortAddress);

    uint8_t acknowledgedPackets = bitmap.getNumberOfPackets(ownAddress);
    this->dsme.getPlatform().signalAcksInGack(acknowledgedPackets);
    LOG_INFO("GACK: ackedPackets:"<< (int)acknowledgedPackets);
    for(uint8_t packet=0; packet<acknowledgedPackets; packet++) {
        /* -> remove all packets that were delivered successfully */

        /* retrieve sequence number */
        uint8_t sequenceNumber = bitmap.getNextSequenceNumber(ownAddress);
        LOG_INFO("GACK: p:"<< (int)packet);
        LOG_INFO("GACK: seqNr:"<< (int)sequenceNumber);

        IDSMEMessage* queuedMsg = retransmissionQueue.popBySequenceNumber(retransmissionQueueNeighbor, sequenceNumber);
        if(queuedMsg == nullptr) {
            LOG_INFO("GACK: not found in queue");
            continue;
        }
        LOG_INFO("GACK: removed from queue");

        /* reset idle counter of the GTS */
        DSMEAllocationCounterTable::iterator actIter = act.find(queuedMsg->txSuperframeId, queuedMsg->txSlotId);
        if(actIter != act.end()) {
            actIter->resetIdleCounter();
        }

        numAckedPackets++;
        uint16_t totalSize = 0;
        for(NeighborQueue<MAX_NEIGHBORS>::iterator it = retransmissionQueue.begin(); it != retransmissionQueue.end(); ++it) {
            totalSize += it->queueSize;
        }
        this->dsme.getPlatform().signalRetransmissionQueueLength(totalSize);

        this->dsme.getPlatform().signalCFPAckDelay(this->dsme.getPlatform().getSymbolCounter()-queuedMsg->getHeader().getCreationTime()); //signal Ack Delay for statistics
        mcps_sap::DATA_confirm_parameters params;
        params.msduHandle = queuedMsg;
        params.timestamp = 0; // TODO
        params.rangingReceived = false;
        params.gtsTX = true;
        params.status = DataStatus::SUCCESS;
        this->dsme.getMCPS_SAP().getDATA().notify_confirm(params);
    }

    /* -> retransmit all packets that are not acknowledged yet until neighborQueue full */
    LOG_INFO("GACK: not acknowledged:");

    while(!retransmissionQueue.isQueueEmpty(retransmissionQueueNeighbor) && !neighborQueue.isQueueFull()) {
        IDSMEMessage* queuedMsg = retransmissionQueue.popFront(retransmissionQueueNeighbor);
        LOG_INFO("GACK: seqNr:"<< (int)queuedMsg->getHeader().getSequenceNumber());
        LOG_INFO("GACK: retry:"<< (int)queuedMsg->getRetryCounter());
        numRetransmittedPackets++;

        /* Increment idle counter of the GTS */
        DSMEAllocationCounterTable::iterator actIter = act.find(queuedMsg->txSuperframeId, queuedMsg->txSlotId);
        if(actIter != act.end()) {
            actIter->incrementIdleCounter();
        }

        if(queuedMsg->getRetryCounter() < dsme.getMAC_PIB().macMaxFrameRetries) {
            queuedMsg->increaseRetryCounter();
            LOG_INFO("GACK: resend:"<< (int)queuedMsg->getHeader().getSequenceNumber());
            neighborQueue.pushBack(neighborQueueNeighbor, queuedMsg);
            preparedMsg = nullptr;
        } else {
            /* -> inform the upper layer about the unsuccessful transmission */
            mcps_sap::DATA_confirm_parameters params;
            params.msduHandle = queuedMsg;
            params.timestamp = 0; // TODO
            params.rangingReceived = false;
            params.gtsTX = true;
            params.status = DataStatus::NO_ACK;
            this->dsme.getMCPS_SAP().getDATA().notify_confirm(params);
        }
    }

    this->dsme.getPlatform().signalPacketRetransmissionRate(((double)numRetransmittedPackets)/((double)(numAckedPackets+numRetransmittedPackets)));

    gackBitmap.reset();
    return true;
}

/*bool MessageDispatcher::handleGackReception(IDSMEMessage* msg) {

    GTSGackCmd gackCmd(gackBitmap);
    gackCmd.decapsulateFrom(msg);

    LOG_INFO("GACK MAP RECEIVED: ");
    int count = 0;
    for(int i = 0; i < gackCmd.getGackVector().length(); i++){
        LOG_INFO("slotID: " << i << " status: " << gackCmd.getGackVector().get(i));
        if(gackCmd.getGackVector().get(i) == true){
            count++;
        }
    }
    //parse received gackVector and retransmit messages

    DSMEAllocationCounterTable& act = this->dsme.getMAC_PIB().macDSMEACT;
    uint16_t cfpSlotsPerSuperframe = this->dsme.getMAC_PIB().macCapReduction?15:7;
    uint8_t superframesPerGroupAckSlot = this->dsme.getMAC_PIB().helper.getNumberSuperframesPerGroupAckSlot();
    DSME_ASSERT(gackCmd.getGackVector().length() % cfpSlotsPerSuperframe == 0);
    uint8_t packetsPerGTS = gackCmd.getGackVector().length() / cfpSlotsPerSuperframe / superframesPerGroupAckSlot;

    const IEEE802154MacAddress srcAddr = msg->getHeader().getSrcAddr();
    NeighborQueue<MAX_NEIGHBORS>::iterator neighborQueueNeighbor = neighborQueue.findByAddress(srcAddr);
    NeighborQueue<MAX_NEIGHBORS>::iterator retransmissionQueueNeighbor = retransmissionQueue.findByAddress(srcAddr);
    DSME_ASSERT(neighborQueueNeighbor != neighborQueue.end() && retransmissionQueueNeighbor != retransmissionQueue.end());

    for(uint8_t superframeID = 0; superframeID < superframesPerGroupAckSlot;superframeID++){ //for every superframe in the msg
        for(uint8_t gtsId=0; gtsId<cfpSlotsPerSuperframe; gtsId++) { //for every GTSlot
            uint16_t vectorPtr = superframeID*cfpSlotsPerSuperframe*packetsPerGTS + gtsId*packetsPerGTS;
            if(act.isAllocated(superframeID, gtsId)) {
                if(IEEE802154MacAddress(act.find(superframeID, gtsId)->getAddress()) == srcAddr) {
                    for(uint16_t i = 0; i<packetsPerGTS; i++) {

                        if(retransmissionQueue.isQueueEmpty(retransmissionQueueNeighbor)) {
                            break;
                        }
                        IDSMEMessage* queuedMsg = retransmissionQueue.popFront(retransmissionQueueNeighbor);
                        mcps_sap::DATA_confirm_parameters params;
                        params.msduHandle = queuedMsg;
                        params.timestamp = 0; // TODO
                        params.rangingReceived = false;
                        params.gtsTX = true;

                        if(gackCmd.getGackVector().get(vectorPtr+i) == true) {
                            params.status = DataStatus::SUCCESS;
                        } else {
                            if(queuedMsg->getRetryCounter() < dsme.getMAC_PIB().macMaxFrameRetries) {
                                queuedMsg->increaseRetryCounter();
                                LOG_DEBUG("handleGACK - retry");
                                if(!neighborQueue.isQueueFull()) {
                                    neighborQueue.pushBack(neighborQueueNeighbor, queuedMsg);
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
    //this->gackHelper.resetTransmittedPacketsGTS();
    return true;
} */

bool MessageDispatcher::prepareGackCommand(){
    bool result = false;
    bool checkTimeToSendMessage = false;

    // check if there exists a pending Message
    DSME_ASSERT(this->preparedMsg == nullptr);
    this->preparedMsg = dsme.getPlatform().getEmptyMessage();
    DSME_ASSERT(this->preparedMsg != nullptr);

    GTSGackCmd gackCmd(gackBitmap); //prepare gackCmd here
    char buf[5];
    sprintf(buf,"%i",gackCmd.getSerializationLength());
    dsme.getPlatform().signalGackSize(gackCmd.getSerializationLength());
    dsme.getPlatform().signalAckSent();
    LOG_INFO("GACK MAP SENT: "<< buf << " bytes");

    gackCmd.prependTo(this->preparedMsg);

    MACCommand cmd;
    cmd.setCmdId(DSME_GTS_GACK);
    cmd.prependTo(this->preparedMsg);

    this->preparedMsg->getHeader().setDstAddr(IEEE802154MacAddress(IEEE802154MacAddress::SHORT_BROADCAST_ADDRESS));
    this->preparedMsg->getHeader().setSrcAddrMode(AddrMode::SHORT_ADDRESS);
    this->preparedMsg->getHeader().setSrcAddr(IEEE802154MacAddress(dsme.getMAC_PIB().macShortAddress));
    this->preparedMsg->getHeader().setDstAddrMode(AddrMode::SHORT_ADDRESS);

    this->preparedMsg->getHeader().setSrcPANId(this->dsme.getMAC_PIB().macPANId);
    this->preparedMsg->getHeader().setDstPANId(this->dsme.getMAC_PIB().macPANId);

    this->preparedMsg->getHeader().setAckRequest(false);   //No ACK for gACK as it is broadcasted
    this->preparedMsg->getHeader().setFrameType(IEEE802154eMACHeader::FrameType::COMMAND);

    /* STATISTICS (START) */
    this->preparedMsg->getHeader().setCreationTime(dsme.getPlatform().getSymbolCounter());
    /* STATISTICS (END) */


    checkTimeToSendMessage = true;
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
        this->preparedMsg = neighborQueue.front(this->lastSendGTSNeighbor);
    }

    if(this->preparedMsg){
        //if(currentACTElement->isGackEnabled() && this->preparedMsg->getHeader().getFrameControl().frameType == IEEE802154eMACHeader::DATA){
        if(dsme.getPlatform().isGackEnabled() && this->preparedMsg->getHeader().getFrameControl().frameType == IEEE802154eMACHeader::DATA){
            //if gackEnabled in the current GTS and its a data message -> get gackEnabled flag
            gackIE *gIE = new gackIE();
            DSME_ASSERT(gIE != nullptr);
            preparedMsg->getHeader().getIEList().insert(gIE);
            preparedMsg->getHeader().setAckRequest(false);
        }
    }

    if(this->neighborQueue.getPacketsInQueue(this->lastSendGTSNeighbor) == 1){
        LOG_INFO("last Packet in queue");
        lastMessageIE *lmIE = new lastMessageIE();
        DSME_ASSERT(lmIE != nullptr);
        lmIE->isLastMessage = true;
        preparedMsg->getHeader().getIEList().insert(lmIE);
        LOG_INFO("lastMessageIE added to queue");
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




} /* namespace dsme */


///*
// * openDSME
// *
// * Implementation of the Deterministic & Synchronous Multi-channel Extension (DSME)
// * described in the IEEE 802.15.4-2015 standard
// *
// * Authors: Florian Kauer <florian.kauer@tuhh.de>
// *          Maximilian Koestler <maximilian.koestler@tuhh.de>
// *          Sandrina Backhauss <sandrina.backhauss@tuhh.de>
// *
// * Based on
// *          DSME Implementation for the INET Framework
// *          Tobias Luebkert <tobias.luebkert@tuhh.de>
// *
// * Copyright (c) 2015, Institute of Telematics, Hamburg University of Technology
// * All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions
// * are met:
// * 1. Redistributions of source code must retain the above copyright
// *    notice, this list of conditions and the following disclaimer.
// * 2. Redistributions in binary form must reproduce the above copyright
// *    notice, this list of conditions and the following disclaimer in the
// *    documentation and/or other materials provided with the distribution.
// * 3. Neither the name of the Institute nor the names of its contributors
// *    may be used to endorse or promote products derived from this software
// *    without specific prior written permission.
// *
// * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
// * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
// * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
// * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
// * SUCH DAMAGE.
// */
//
//#include "./MessageDispatcher.h"
//
//#include "../../../dsme_platform.h"
//#include "../../../dsme_settings.h"
//#include "../../helper/DSMEDelegate.h"
//#include "../../helper/Integers.h"
//#include "../../interfaces/IDSMEMessage.h"
//#include "../../interfaces/IDSMEPlatform.h"
//#include "../../mac_services/DSME_Common.h"
//#include "../../mac_services/MacDataStructures.h"
//#include "../../mac_services/dataStructures/DSMEAllocationCounterTable.h"
//#include "../../mac_services/dataStructures/IEEE802154MacAddress.h"
//#include "../../mac_services/mcps_sap/DATA.h"
//#include "../../mac_services/mcps_sap/MCPS_SAP.h"
//#include "../../mac_services/pib/dsme_mac_constants.h"
//#include "../../mac_services/pib/MAC_PIB.h"
//#include "../../mac_services/pib/PHY_PIB.h"
//#include "../../mac_services/pib/PIBHelper.h"
//#include "../DSMELayer.h"
//#include "../ackLayer/AckLayer.h"
//#include "../associationManager/AssociationManager.h"
//#include "../beaconManager/BeaconManager.h"
//#include "../capLayer/CAPLayer.h"
//#include "../gtsManager/GTSManager.h"
//#include "../messages/IEEE802154eMACHeader.h"
//#include "../messages/MACCommand.h"
//
//namespace dsme {
//
//MessageDispatcher::MessageDispatcher(DSMELayer& dsme)
//    : dsme(dsme),
//      currentACTElement(nullptr, nullptr),
//      doneGTS(DELEGATE(&MessageDispatcher::sendDoneGTS, *this)),
//      dsmeAckFrame(nullptr),
//      lastSendGTSNeighbor(neighborQueue.end()) {
//}
//
//MessageDispatcher::~MessageDispatcher() {
//    for(NeighborQueue<MAX_NEIGHBORS>::iterator it = neighborQueue.begin(); it != neighborQueue.end(); ++it) {
//        while(!this->neighborQueue.isQueueEmpty(it)) {
//            IDSMEMessage* msg = neighborQueue.popFront(it);
//            this->dsme.getPlatform().releaseMessage(msg);
//        }
//    }
//}
//
//void MessageDispatcher::initialize(void) {
//    currentACTElement = dsme.getMAC_PIB().macDSMEACT.end();
//    return;
//}
//
//void MessageDispatcher::reset(void) {
//    currentACTElement = dsme.getMAC_PIB().macDSMEACT.end();
//
//    for(NeighborQueue<MAX_NEIGHBORS>::iterator it = neighborQueue.begin(); it != neighborQueue.end(); ++it) {
//        while(!this->neighborQueue.isQueueEmpty(it)) {
//            IDSMEMessage* msg = neighborQueue.popFront(it);
//            mcps_sap::DATA_confirm_parameters params;
//            params.msduHandle = msg;
//            params.timestamp = 0;
//            params.rangingReceived = false;
//            params.gtsTX = true;
//            params.status = DataStatus::TRANSACTION_EXPIRED;
//            params.numBackoffs = 0;
//            this->dsme.getMCPS_SAP().getDATA().notify_confirm(params);
//        }
//    }
//    while(this->neighborQueue.getNumNeighbors() > 0) {
//        NeighborQueue<MAX_NEIGHBORS>::iterator it = this->neighborQueue.begin();
//        this->neighborQueue.eraseNeighbor(it);
//    }
//
//    return;
//}
//
//
//void MessageDispatcher::sendDoneGTS(enum AckLayerResponse response, IDSMEMessage* msg) {
//    LOG_DEBUG("sendDoneGTS");
//
//    DSME_ASSERT(lastSendGTSNeighbor != neighborQueue.end());
//    DSME_ASSERT(msg == neighborQueue.front(lastSendGTSNeighbor));
//
//    DSMEAllocationCounterTable& act = this->dsme.getMAC_PIB().macDSMEACT;
//    DSME_ASSERT(this->currentACTElement != act.end());
//
//    this->dsme.getEventDispatcher().setupIFSTimer(msg->getTotalSymbols() > aMaxSIFSFrameSize);
//
//    if(response != AckLayerResponse::NO_ACK_REQUESTED && response != AckLayerResponse::ACK_SUCCESSFUL) {
//        currentACTElement->incrementIdleCounter();
//
//        // not successful -> retry?
//        if(msg->getRetryCounter() < dsme.getMAC_PIB().macMaxFrameRetries) {
//            msg->increaseRetryCounter();
//            //finalizeGTSTransmission();
//            LOG_DEBUG("sendDoneGTS - retry");
//            return; // will stay at front of queue
//        }
//    }
//
//    if(response == AckLayerResponse::ACK_FAILED || response == AckLayerResponse::ACK_SUCCESSFUL) {
//        this->dsme.getPlatform().signalAckedTransmissionResult(response == AckLayerResponse::ACK_SUCCESSFUL, msg->getRetryCounter() + 1, msg->getHeader().getDestAddr());
//    }
//
//    //TODO check flag and turn of transceiver
//    bool turnOff = false;
//    InformationElement* iePointer = nullptr;
//    if(msg->getHeader().getIEListPresent() == true){
//        if(msg->getHeader().ieQueue.getIEByID(0x10, iePointer)){  //check for lastMessageIE
//            if(dynamic_cast<lastMessageIE*>(iePointer)->isLastMessage){
//                   turnOff = true;
//                   LOG_INFO("Last Message true");
//           }
//        }
//        LOG_INFO("Information Element present");
//    }
//
//    neighborQueue.popFront(lastSendGTSNeighbor);
//    this->preparedMsg = nullptr;
//
//    /* STATISTICS */
//    uint16_t totalSize = 0;
//    for(NeighborQueue<MAX_NEIGHBORS>::iterator it = neighborQueue.begin(); it != neighborQueue.end(); ++it) {
//        totalSize += it->queueSize;
//    }
//    this->dsme.getPlatform().signalQueueLength(totalSize);
//    /* END STATISTICS */
//
//    mcps_sap::DATA_confirm_parameters params;
//    params.msduHandle = msg;
//    params.timestamp = 0; // TODO
//    params.rangingReceived = false;
//    params.gtsTX = true;
//
//    switch(response) {
//        case AckLayerResponse::NO_ACK_REQUESTED:
//        case AckLayerResponse::ACK_SUCCESSFUL:
//            LOG_DEBUG("sendDoneGTS - success");
//            params.status = DataStatus::SUCCESS;
//            break;
//        case AckLayerResponse::ACK_FAILED:
//            DSME_ASSERT(this->currentACTElement != this->dsme.getMAC_PIB().macDSMEACT.end());
//            currentACTElement->incrementIdleCounter();
//            params.status = DataStatus::NO_ACK;
//            break;
//        case AckLayerResponse::SEND_FAILED:
//            LOG_DEBUG("SEND_FAILED during GTS");
//            DSME_ASSERT(this->currentACTElement != this->dsme.getMAC_PIB().macDSMEACT.end());
//            currentACTElement->incrementIdleCounter();
//            params.status = DataStatus::CHANNEL_ACCESS_FAILURE;
//            break;
//        case AckLayerResponse::SEND_ABORTED:
//            LOG_DEBUG("SEND_ABORTED during GTS");
//            params.status = DataStatus::TRANSACTION_EXPIRED;
//            break;
//        default:
//            DSME_ASSERT(false);
//    }
//
//    params.numBackoffs = 0;
//    this->dsme.getMCPS_SAP().getDATA().notify_confirm(params);
//
//
//    if(turnOff || !this->multiplePacketsPerGTS || !prepareNextMessageIfAny()) {
//        /* '-> prepare next frame for transmission after one IFS */
//        finalizeGTSTransmission();
//    }
//}
//
//void MessageDispatcher::finalizeGTSTransmission() {
//    LOG_DEBUG("Finalizing transmission for slot " << (int)this->currentACTElement->getGTSlotID() << " " << (int)this->currentACTElement->getSuperframeID() << " " << (int)this->currentACTElement->getChannel());
//    transceiverOffIfAssociated();
//    this->dsme.getEventDispatcher().stopIFSTimer();
//    this->preparedMsg = nullptr;    // TODO correct here?
//    this->lastSendGTSNeighbor = this->neighborQueue.end();
//    this->currentACTElement = this->dsme.getMAC_PIB().macDSMEACT.end();
//    if(this->numTxGtsFrames > 0) this->dsme.getPlatform().signalPacketsTXPerSlot(this->numTxGtsFrames);
//    if(this->numRxGtsFrames > 0) this->dsme.getPlatform().signalPacketsRXPerSlot(this->numRxGtsFrames);
//    this->numTxGtsFrames = 0;
//    this->numRxGtsFrames = 0;
//}
//
//void MessageDispatcher::onCSMASent(IDSMEMessage* msg, DataStatus::Data_Status status, uint8_t numBackoffs, uint8_t transmissionAttempts) {
//    if(status == DataStatus::Data_Status::NO_ACK || status == DataStatus::Data_Status::SUCCESS) {
//        if(msg->getHeader().isAckRequested() && !msg->getHeader().getDestAddr().isBroadcast()) {
//            this->dsme.getPlatform().signalAckedTransmissionResult(status == DataStatus::Data_Status::SUCCESS, transmissionAttempts,
//                                                                   msg->getHeader().getDestAddr());
//        }
//    }
//
//    if(msg->getReceivedViaMCPS()) {
//        mcps_sap::DATA_confirm_parameters params;
//        params.msduHandle = msg;
//        params.timestamp = 0; // TODO
//        params.rangingReceived = false;
//        params.status = status;
//        params.numBackoffs = numBackoffs;
//        params.gtsTX = false;
//        this->dsme.getMCPS_SAP().getDATA().notify_confirm(params);
//    } else {
//        if(msg->getHeader().getFrameType() == IEEE802154eMACHeader::FrameType::COMMAND) {
//            MACCommand cmd;
//            cmd.decapsulateFrom(msg);
//
//            LOG_DEBUG("cmdID " << (uint16_t)cmd.getCmdId());
//
//            switch(cmd.getCmdId()) {
//                case ASSOCIATION_REQUEST:
//                case ASSOCIATION_RESPONSE:
//                case DISASSOCIATION_NOTIFICATION:
//                    this->dsme.getAssociationManager().onCSMASent(msg, cmd.getCmdId(), status, numBackoffs);
//                    break;
//                case DATA_REQUEST:
//                case DSME_ASSOCIATION_REQUEST:
//                case DSME_ASSOCIATION_RESPONSE:
//                    DSME_ASSERT(false);
//                    // TODO handle correctly
//                    this->dsme.getPlatform().releaseMessage(msg);
//                    break;
//                case BEACON_REQUEST:
//                case DSME_BEACON_ALLOCATION_NOTIFICATION:
//                case DSME_BEACON_COLLISION_NOTIFICATION:
//                    this->dsme.getBeaconManager().onCSMASent(msg, cmd.getCmdId(), status, numBackoffs);
//                    break;
//                case DSME_GTS_REQUEST:
//                case DSME_GTS_REPLY:
//                case DSME_GTS_NOTIFY:
//                    this->dsme.getGTSManager().onCSMASent(msg, cmd.getCmdId(), status, numBackoffs);
//                    break;
//            }
//        } else {
//            this->dsme.getPlatform().releaseMessage(msg);
//        }
//    }
//}
//
//bool MessageDispatcher::sendInGTS(IDSMEMessage* msg, NeighborQueue<MAX_NEIGHBORS>::iterator destIt) {
//    DSME_ASSERT(!msg->getHeader().getDestAddr().isBroadcast());
//    DSME_ASSERT(this->dsme.getMAC_PIB().macAssociatedPANCoord);
//    DSME_ASSERT(destIt != neighborQueue.end());
//
//    numUpperPacketsForGTS++;
//
//    if(!neighborQueue.isQueueFull()) {
//        /* push into queue */
//        // TODO implement TRANSACTION_EXPIRED
//        uint16_t totalSize = 0;
//        for(NeighborQueue<MAX_NEIGHBORS>::iterator it = neighborQueue.begin(); it != neighborQueue.end(); ++it) {
//            totalSize += it->queueSize;
//        }
//        LOG_INFO("NeighborQueue is at " << totalSize << "/" << TOTAL_GTS_QUEUE_SIZE << ".");
//        neighborQueue.pushBack(destIt, msg);
//        this->dsme.getPlatform().signalQueueLength(totalSize+1);
//        return true;
//    } else {
//        /* queue full */
//        LOG_INFO("NeighborQueue is full!");
//        numUpperPacketsDroppedFullQueue++;
//        return false;
//    }
//}
//
//bool MessageDispatcher::sendInCAP(IDSMEMessage* msg) {
//    LOG_INFO("Inserting message into CAP queue.");
//    if(msg->getHeader().getSrcAddrMode() != EXTENDED_ADDRESS && !(this->dsme.getMAC_PIB().macAssociatedPANCoord)) {
//        LOG_INFO("Message dropped due to missing association!");
//        // TODO document this behaviour
//        // TODO send appropriate MCPS confirm or better remove this handling and implement TRANSACTION_EXPIRED
//        return false;
//    }
//    if(!this->dsme.getCapLayer().pushMessage(msg)) {
//        LOG_INFO("CAP queue full!");
//        return false;
//    }
//    if(msg->getHeader().getFrameType() == IEEE802154eMACHeader::FrameType::COMMAND) {
//        numUpperPacketsForCAP++;
//    }
//    return true;
//}
//
//void MessageDispatcher::receive(IDSMEMessage* msg) {
//    IEEE802154eMACHeader macHdr = msg->getHeader();
//
//    switch(macHdr.getFrameType()) {
//        case IEEE802154eMACHeader::FrameType::BEACON: {
//            LOG_INFO("BEACON from " << macHdr.getSrcAddr().getShortAddress() << " " << macHdr.getSrcPANId() << " " << dsme.getCurrentSuperframe() << ".");
//            this->dsme.getBeaconManager().handleBeacon(msg);
//            this->dsme.getPlatform().releaseMessage(msg);
//            break;
//        }
//
//        case IEEE802154eMACHeader::FrameType::COMMAND: {
//            MACCommand cmd;
//            cmd.decapsulateFrom(msg);
//            switch(cmd.getCmdId()) {
//                case CommandFrameIdentifier::DSME_GTS_REQUEST:
//                    LOG_INFO("DSME-GTS-REQUEST from " << macHdr.getSrcAddr().getShortAddress() << ".");
//                    dsme.getGTSManager().handleGTSRequest(msg);
//                    break;
//                case CommandFrameIdentifier::DSME_GTS_REPLY:
//                    LOG_INFO("DSME-GTS-REPLY from " << macHdr.getSrcAddr().getShortAddress() << ".");
//                    dsme.getGTSManager().handleGTSResponse(msg);
//                    break;
//                case CommandFrameIdentifier::DSME_GTS_NOTIFY:
//                    LOG_INFO("DSME-GTS-NOTIFY from " << macHdr.getSrcAddr().getShortAddress() << ".");
//                    dsme.getGTSManager().handleGTSNotify(msg);
//                    break;
//                case CommandFrameIdentifier::ASSOCIATION_REQUEST:
//                    LOG_INFO("ASSOCIATION-REQUEST from " << macHdr.getSrcAddr().getShortAddress() << ".");
//                    dsme.getAssociationManager().handleAssociationRequest(msg);
//                    break;
//                case CommandFrameIdentifier::ASSOCIATION_RESPONSE:
//                    LOG_INFO("ASSOCIATION-RESPONSE from " << macHdr.getSrcAddr().getShortAddress() << ".");
//                    dsme.getAssociationManager().handleAssociationReply(msg);
//                    break;
//                case CommandFrameIdentifier::DISASSOCIATION_NOTIFICATION:
//                    LOG_INFO("DISASSOCIATION-NOTIFICATION from " << macHdr.getSrcAddr().getShortAddress() << ".");
//                    dsme.getAssociationManager().handleDisassociationRequest(msg);
//                    break;
//                case CommandFrameIdentifier::DATA_REQUEST:
//                    /* Not implemented */
//                    break;
//                case CommandFrameIdentifier::DSME_BEACON_ALLOCATION_NOTIFICATION:
//                    LOG_INFO("DSME-BEACON-ALLOCATION-NOTIFICATION from " << macHdr.getSrcAddr().getShortAddress() << ".");
//                    dsme.getBeaconManager().handleBeaconAllocation(msg);
//                    break;
//                case CommandFrameIdentifier::DSME_BEACON_COLLISION_NOTIFICATION:
//                    LOG_INFO("DSME-BEACON-COLLISION-NOTIFICATION from " << macHdr.getSrcAddr().getShortAddress() << ".");
//                    dsme.getBeaconManager().handleBeaconCollision(msg);
//                    break;
//                case CommandFrameIdentifier::BEACON_REQUEST:
//                    LOG_INFO("BEACON_REQUEST from " << macHdr.getSrcAddr().getShortAddress() << ".");
//                    dsme.getBeaconManager().handleBeaconRequest(msg);
//                    break;
//                default:
//                    LOG_ERROR("Invalid cmd ID " << (uint16_t)cmd.getCmdId());
//                    // DSME_ASSERT(false);
//            }
//            dsme.getPlatform().releaseMessage(msg);
//            break;
//        }
//
//        case IEEE802154eMACHeader::FrameType::DATA: {
//            if(currentACTElement != dsme.getMAC_PIB().macDSMEACT.end()) {
//                handleGTSFrame(msg);
//            } else {
//                createDataIndication(msg);
//            }
//            break;
//        }
//
//        default: {
//            LOG_ERROR((uint16_t)macHdr.getFrameType());
//            dsme.getPlatform().releaseMessage(msg);
//        }
//    }
//    return;
//}
//
//bool MessageDispatcher::handlePreSlotEvent(uint8_t nextSlot, uint8_t nextSuperframe, uint8_t nextMultiSuperframe) {
//    LOG_DEBUG("NEXT SLOT: " << (int)nextSlot << " NEXT SF: " << (int)nextSuperframe);
//    // Prepare next slot
//    // Switch to next slot channel and radio mode
//    DSMEAllocationCounterTable& act = this->dsme.getMAC_PIB().macDSMEACT;
//    if(this->currentACTElement != act.end()) {
//        if(true) { //this->currentACTElement->getDirection() == Direction::RX) {
//            this->currentACTElement = act.end();
//        } else {
//            // Rarely happens, only if the sendDoneGTS is delayed
//            // Then skip this preSlotEvent
//            LOG_DEBUG("Previous slot did not finish until preslot event: slot " << (int)nextSlot << " SF " << (int)nextSuperframe);
//            //DSME_SIM_ASSERT(false);
//            return false;
//        }
//    }
//
//    if(nextSlot == 0) {
//        /* '-> beacon slots are handled by the BeaconManager */
//        DSME_ASSERT(this->currentACTElement == act.end());
//    } else if(nextSlot >= 1 && nextSlot <= 8) {
//        /* '-> next slot will be CAP */
//
//        if(!this->dsme.getMAC_PIB().macCapReduction || nextSuperframe == 0) {
//            /* '-> active CAP slot */
//
//            this->dsme.getPlatform().turnTransceiverOn();
//            this->dsme.getPlatform().setChannelNumber(this->dsme.getPHY_PIB().phyCurrentChannel);
//        } else {
//            /* '-> CAP reduction */
//            transceiverOffIfAssociated();
//        }
//    }
//
//    if(nextSlot > this->dsme.getMAC_PIB().helper.getFinalCAPSlot(nextSuperframe)) {
//        /* '-> next slot will be GTS */
//
//        unsigned nextGTS = nextSlot - (this->dsme.getMAC_PIB().helper.getFinalCAPSlot(nextSuperframe) + 1);
//        if(act.isAllocated(nextSuperframe, nextGTS)) {
//            /* '-> this slot might be used */
//            LOG_DEBUG("Current slot is allocated as GTS");
//
//            this->currentACTElement = act.find(nextSuperframe, nextGTS);
//            DSME_ASSERT(this->currentACTElement != act.end());
//            // For TX currentACTElement will be reset in finalizeGTSTransmission, called by
//            // either handleGTS if nothing is to send or by sendDoneGTS.
//            // For RX it is reset in the next handlePreSlotEvent.   TODO: is the reset actually required?
//
//            // For RX also if INVALID or UNCONFIRMED!
//            if((this->currentACTElement->getState() == VALID) || (this->currentACTElement->getDirection() == Direction::RX)) {
//                this->dsme.getPlatform().turnTransceiverOn();
//
//                if(dsme.getMAC_PIB().macChannelDiversityMode == Channel_Diversity_Mode::CHANNEL_ADAPTATION) {
//                    LOG_DEBUG("Switching to channel " << (int)this->currentACTElement->getChannel());
//                    this->dsme.getPlatform().setChannelNumber(this->dsme.getMAC_PIB().helper.getChannels()[this->currentACTElement->getChannel()]);
//                } else {
//                    uint8_t channel = nextHoppingSequenceChannel(nextSlot, nextSuperframe, nextMultiSuperframe);
//                    this->dsme.getPlatform().setChannelNumber(channel);
//                }
//            }
//
//            // statistic
//            if(this->currentACTElement->getDirection() == RX) {
//                LOG_DEBUG("Current GTS is RX");
//                this->numUnusedRxGts++; // gets PURGE.cc decremented on actual reception
//            }
//        } else {
//            /* '-> nothing to do during this slot */
//            DSME_ASSERT(this->currentACTElement == act.end());
//            //transceiverOffIfAssociated();
//        }
//    }
//
//    return true;
//}
//
//uint8_t MessageDispatcher::nextHoppingSequenceChannel(uint8_t nextSlot, uint8_t nextSuperframe, uint8_t nextMultiSuperframe) {
//    uint16_t hoppingSequenceLength = this->dsme.getMAC_PIB().macHoppingSequenceLength;
//    uint8_t ebsn = 0; // this->dsme.getMAC_PIB().macPanCoordinatorBsn;    //TODO is this set correctly
//    uint16_t sdIndex = nextSuperframe + this->dsme.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe() * nextMultiSuperframe;
//    uint8_t numGTSlots = this->dsme.getMAC_PIB().helper.getNumGTSlots(sdIndex);
//
//    uint8_t slotId = this->currentACTElement->getGTSlotID();
//    uint16_t channelOffset = this->currentACTElement->getChannel();
//
//    uint8_t channel =
//        this->dsme.getMAC_PIB().macHoppingSequenceList[(sdIndex * numGTSlots + slotId + channelOffset + ebsn) % hoppingSequenceLength];
//    LOG_INFO("Using channel " << channel << " - numGTSlots: " << numGTSlots << " EBSN: " << ebsn << " sdIndex: " << sdIndex
//                              << " slot: " << slotId << " Superframe " << nextSuperframe << " channelOffset: " << channelOffset
//                              << " Direction: " << currentACTElement->getDirection());
//    return channel;
//}
//
//bool MessageDispatcher::handleSlotEvent(uint8_t slot, uint8_t superframe, int32_t lateness) {
//    if(slot > dsme.getMAC_PIB().helper.getFinalCAPSlot(superframe)) {
//        handleGTS(lateness);
//    }
//    return true;
//}
//
//bool MessageDispatcher::handleIFSEvent(int32_t lateness) {
//    /* Neighbor and slot have to be valid at this point */
//    DSME_ASSERT(this->lastSendGTSNeighbor != this->neighborQueue.end());
//    DSME_ASSERT(this->currentACTElement != this->dsme.getMAC_PIB().macDSMEACT.end());
//    DSME_ASSERT(this->currentACTElement->getSuperframeID() == this->dsme.getCurrentSuperframe() && this->currentACTElement->getGTSlotID()
//      == this->dsme.getCurrentSlot() - (this->dsme.getMAC_PIB().helper.getFinalCAPSlot(this->dsme.getCurrentSuperframe())+1));
//
//    if(!sendPreparedMessage()) {
//        finalizeGTSTransmission();
//    }
//
//    return true;
//}
//
//void MessageDispatcher::handleGTS(int32_t lateness) {
//    if(this->currentACTElement != this->dsme.getMAC_PIB().macDSMEACT.end() && this->currentACTElement->getSuperframeID() == this->dsme.getCurrentSuperframe() &&
//       this->currentACTElement->getGTSlotID() == this->dsme.getCurrentSlot() - (this->dsme.getMAC_PIB().helper.getFinalCAPSlot(dsme.getCurrentSuperframe()) + 1)) {
//        /* '-> this slot matches the prepared ACT element */
//
//        if(this->currentACTElement->getDirection() == RX) { // also if INVALID or UNCONFIRMED!
//            /* '-> a message may be received during this slot */
//
//        } else if(this->currentACTElement->getState() == VALID) {
//            /* '-> if any messages are queued for this link, send one */
//
//            DSME_ASSERT(this->lastSendGTSNeighbor == this->neighborQueue.end());
//
//            IEEE802154MacAddress adr = IEEE802154MacAddress(this->currentACTElement->getAddress());
//            this->lastSendGTSNeighbor = this->neighborQueue.findByAddress(IEEE802154MacAddress(this->currentACTElement->getAddress()));
//            if(this->lastSendGTSNeighbor == this->neighborQueue.end()) {
//                /* '-> the neighbor associated with the current slot does not exist */
//
//                LOG_ERROR("neighborQueue.size: " << ((uint8_t) this->neighborQueue.getNumNeighbors()));
//                LOG_ERROR("neighbor address: " << HEXOUT << adr.a1() << ":" << adr.a2() << ":" << adr.a3() << ":" << adr.a4() << DECOUT);
//                for(auto it : this->neighborQueue) {
//                    LOG_ERROR("neighbor address: " << HEXOUT << it.address.a1() << ":" << it.address.a2() << ":" << it.address.a3() << ":" << it.address.a4()
//                                                   << DECOUT);
//                }
//                DSME_ASSERT(false);
//            }
//
//            bool success = prepareNextMessageIfAny();
//            if(success) {
//                /* '-> a message is queued for transmission */
//                success = sendPreparedMessage();
//            }
//
//            if(!success) {
//                /* '-> no message to be sent */
//                LOG_DEBUG("MessageDispatcher: Could not transmit any packet in GTS");
//                this->numUnusedTxGts++;
//                finalizeGTSTransmission();
//            }
//        } else {
//            finalizeGTSTransmission();
//        }
//    }
//}
//
//void MessageDispatcher::handleGTSFrame(IDSMEMessage* msg) {
//    DSME_ASSERT(currentACTElement != dsme.getMAC_PIB().macDSMEACT.end());
//
//    numRxGtsFrames++;
//    numUnusedRxGts--;
//
//    if(currentACTElement->getSuperframeID() == dsme.getCurrentSuperframe() &&
//       currentACTElement->getGTSlotID() == dsme.getCurrentSlot() - (dsme.getMAC_PIB().helper.getFinalCAPSlot(dsme.getCurrentSuperframe()) + 1)) {
//        // According to 5.1.10.5.3
//        currentACTElement->resetIdleCounter();
//    }
//
//    createDataIndication(msg);
//}
//
//
//// Function to determine if there is any message that can be transmitted or not.
//// Returns: True: 1. If there is a pending message (e.g. from failed transmission) to transmit
////                2. If there exits any message in the queue to transmit.
////          False otherwise
//bool MessageDispatcher::prepareNextMessageIfAny() {
//    bool result = false;
//    bool checkTimeToSendMessage = false;
//
//    // check if there exists a pending Message
//    if(this->preparedMsg) {
//        checkTimeToSendMessage = true; // if true, set a flag to to check if pending message can be sent in remaining slot time
//    } else if (this->neighborQueue.isQueueEmpty(this->lastSendGTSNeighbor)) {  // there is no pending message, then check if the queue is empty (i.e. there is not any message to transmit to a target neighbor)
//        this->preparedMsg = nullptr; // reset value of pending Message
//        checkTimeToSendMessage = false;
//        result = false;
//    } else { // if there is a message to send retrieve a copy of it from the queue and set the flag to check if possible to send the message
//        checkTimeToSendMessage = true;
//        this->preparedMsg = neighborQueue.front(this->lastSendGTSNeighbor);
//    }
//
//    if(this->neighborQueue.getPacketsInQueue(this->lastSendGTSNeighbor) == 1){
//        LOG_INFO("last Packet in queue");
//        lastMessageIE lmIE;
//        lmIE.isLastMessage = true;
//        preparedMsg->getHeader().ieQueue.push(lmIE);
//        LOG_INFO("lastMessageIE added to queue");
//    }
//
//    if(checkTimeToSendMessage) {//if the timming for transmission must be checked
//        // determined how long the transmission of the preparedMessage will take.
//        uint8_t ifsSymbols = this->preparedMsg->getTotalSymbols() <= aMaxSIFSFrameSize ? const_redefines::macSIFSPeriod : const_redefines::macLIFSPeriod;
//        uint32_t duration = this->preparedMsg->getTotalSymbols() + this->dsme.getMAC_PIB().helper.getAckWaitDuration() + ifsSymbols;
//        // check if the remaining slot time is enough to transmit the prepared packet
//        if(!this->dsme.isWithinTimeSlot(this->dsme.getPlatform().getSymbolCounter(), duration)) {
//            LOG_DEBUG("No packet prepared (remaining slot time insufficient)");
//            this->preparedMsg = nullptr; // reset value of pending Message
//            result = false; // there is no enough time, no transmission will take place
//        } else {
//            result = true; // there is time, then proceed
//        }
//    }
//    return result;
//}
//
//bool MessageDispatcher::sendPreparedMessage() {
//    DSME_ASSERT(this->preparedMsg);
//    DSME_ASSERT(this->dsme.getMAC_PIB().helper.getSymbolsPerSlot() >= this->preparedMsg->getTotalSymbols() + this->dsme.getMAC_PIB().helper.getAckWaitDuration() + 10 /* arbitrary processing delay */ + PRE_EVENT_SHIFT);
//
//    uint8_t ifsSymbols = this->preparedMsg->getTotalSymbols() <= aMaxSIFSFrameSize ? const_redefines::macSIFSPeriod : const_redefines::macLIFSPeriod;
//    uint32_t duration = this->preparedMsg->getTotalSymbols() + this->dsme.getMAC_PIB().helper.getAckWaitDuration() + ifsSymbols;
//    /* '-> Duration for the transmission of the next frame */
//
//    if(this->dsme.isWithinTimeSlot(this->dsme.getPlatform().getSymbolCounter(), duration)) {
//        /* '-> Sufficient time to send message in remaining slot time */
//        if (this->dsme.getAckLayer().prepareSendingCopy(this->preparedMsg, this->doneGTS)) {
//            /* '-> Message transmission can be attempted */
//            this->dsme.getAckLayer().sendNowIfPending();
//            this->numTxGtsFrames++;
//        } else {
//            /* '-> Message could not be sent via ACKLayer (FAILED) */
//            sendDoneGTS(AckLayerResponse::SEND_FAILED, this->preparedMsg);
//        }
//        return true;
//    }
//    LOG_DEBUG("No packet sent (remaining slot time insufficient)");
//    return false;
//}
//
//
//void MessageDispatcher::createDataIndication(IDSMEMessage* msg) {
//    IEEE802154eMACHeader& header = msg->getHeader();
//
//    mcps_sap::DATA_indication_parameters params;
//
//    params.msdu = msg;
//
//    params.mpduLinkQuality = 0; // TODO link quality?
//    params.dsn = header.getSequenceNumber();
//    params.timestamp = msg->getStartOfFrameDelimiterSymbolCounter();
//    params.securityLevel = header.isSecurityEnabled();
//
//    params.dataRate = 0; // DSSS -> 0
//
//    params.rangingReceived = NO_RANGING_REQUESTED;
//    params.rangingCounterStart = 0;
//    params.rangingCounterStop = 0;
//    params.rangingTrackingInterval = 0;
//    params.rangingOffset = 0;
//    params.rangingFom = 0;
//
//    this->dsme.getMCPS_SAP().getDATA().notify_indication(params);
//}
//
//void MessageDispatcher::transceiverOffIfAssociated() {
//    LOG_INFO("Tranceiver turned off");
//    if(this->dsme.getMAC_PIB().macAssociatedPANCoord) {
//      this->dsme.getPlatform().turnTransceiverOff();
//    } else {
//        /* '-> do not turn off the transceiver while we might be scanning */
//    }
//}
//
//
//
//
//} /* namespace dsme */
