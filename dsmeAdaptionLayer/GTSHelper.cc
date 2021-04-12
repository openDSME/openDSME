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

#include "./GTSHelper.h"

#include "../../dsme_platform.h"
#include "../dsmeLayer/DSMELayer.h" // TODO: remove cross-layer reference
#include "../helper/DSMEDelegate.h"
#include "../interfaces/IDSMEPlatform.h"
#include "../mac_services/dataStructures/DSMEAllocationCounterTable.h"
#include "../mac_services/dataStructures/DSMESABSpecification.h"
#include "../mac_services/dataStructures/DSMESlotAllocationBitmap.h"
#include "../mac_services/dataStructures/IEEE802154MacAddress.h"
#include "../mac_services/mcps_sap/MCPS_SAP.h"
#include "../mac_services/mlme_sap/DSME_GTS.h"
#include "../mac_services/mlme_sap/MLME_SAP.h"
#include "../mac_services/pib/MAC_PIB.h"
#include "../mac_services/pib/PIBHelper.h"
#include "./DSMEAdaptionLayer.h"

namespace dsme {

GTSHelper::GTSHelper(DSMEAdaptionLayer& dsmeAdaptionLayer) : dsmeAdaptionLayer(dsmeAdaptionLayer), gtsConfirmPending(false) {
}

void GTSHelper::initialize(GTSScheduling* scheduling) {
    this->gtsScheduling = scheduling;

    this->dsmeAdaptionLayer.getDSME().setStartOfCFPDelegate(DELEGATE(&GTSHelper::handleStartOfCFP, *this)); /* BAD cross-layer hack */

    this->dsmeAdaptionLayer.getMLME_SAP().getDSME_GTS().indication(DELEGATE(&GTSHelper::handleDSME_GTS_indication, *this));
    this->dsmeAdaptionLayer.getMLME_SAP().getDSME_GTS().confirm(DELEGATE(&GTSHelper::handleDSME_GTS_confirm, *this));
    this->dsmeAdaptionLayer.getMLME_SAP().getCOMM_STATUS().indication(DELEGATE(&GTSHelper::handleCOMM_STATUS_indication, *this));
    return;
}

void GTSHelper::reset() {
    this->gtsConfirmPending = false;
    this->gtsScheduling->reset();
}

uint8_t GTSHelper::indicateIncomingMessage(uint16_t address) {
    return this->gtsScheduling->registerIncomingMessage(address);
}

void GTSHelper::indicateOutgoingMessage(uint16_t address, bool success, int32_t serviceTime, uint8_t queueAtCreation) {
    this->gtsScheduling->registerOutgoingMessage(address, success, serviceTime, queueAtCreation);
}

void GTSHelper::indicateReceivedMessage(uint16_t address) {
    return this->gtsScheduling->registerReceivedMessage(address);
}

void GTSHelper::handleStartOfCFP() {
    if(this->dsmeAdaptionLayer.getDSME().getCurrentSuperframe() == 0) {
        this->gtsScheduling->multisuperframeEvent();
    }

    /* Check allocation at random superframe in multi-superframe */
    uint8_t num_superframes = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe();
    uint8_t random_frame = this->dsmeAdaptionLayer.getDSME().getPlatform().getRandom() % num_superframes;
    //if(this->dsmeAdaptionLayer.getDSME().getCurrentSuperframe() == random_frame) {
    performSchedulingAction(this->gtsScheduling->getNextSchedulingAction());
    //}

    //debug logging
    DSMEAllocationCounterTable& macDSMEACT = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT;
    DSMEAllocationCounterTable::iterator it = macDSMEACT.begin();
    uint8_t currentGTSCount = 0, currentGackGTSCount = 0;
    while(it != macDSMEACT.end()) {
        if(it->isGackGTS()){    //get all already allocated GTS
            currentGackGTSCount++;
        }else{
            currentGTSCount++;
        }
        it++;
    }
    this->dsmeAdaptionLayer.getDSME().getPlatform().signalGTSCount(currentGTSCount);
    this->dsmeAdaptionLayer.getDSME().getPlatform().signalGackGTSCount(currentGackGTSCount);
    return;
}

void GTSHelper::checkAllocationForPacket(uint16_t address) {
    performSchedulingAction(this->gtsScheduling->getNextSchedulingAction(address));
    return;
}

void GTSHelper::performSchedulingAction(GTSSchedulingDecision decision) {
    if(decision.numSlot == 0) {
        LOG_DEBUG("NO SCHEDULING ACTION");
        DSME_ASSERT(decision.deviceAddress == IEEE802154MacAddress::NO_SHORT_ADDRESS);
        return;
    }
    LOG_DEBUG("GTSHelper: performSchedulingAction");

    if(decision.managementType == ManagementType::ALLOCATION) {
        checkAndAllocateGTS(decision);
    } else if(decision.managementType == ManagementType::DEALLOCATION) {
        checkAndDeallocateSingeleGTS(decision.deviceAddress);
    } else {
        DSME_ASSERT(false);
    }
    return;
}

void GTSHelper::checkAndAllocateGTS(GTSSchedulingDecision decision) {
    DSME_ATOMIC_BLOCK {
        if(gtsConfirmPending) {
            LOG_INFO("GTS allocation still active (trying with 0x" << HEXOUT << decision.deviceAddress << DECOUT << ")");
            return;
        }

        gtsConfirmPending = true;
    }

    DSMEAllocationCounterTable& macDSMEACT = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT;
    DSMESlotAllocationBitmap& macDSMESAB = this->dsmeAdaptionLayer.getMAC_PIB().macDSMESAB;

    uint8_t numChannels = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumChannels();

    //The scheduling decision is always in TX direction. Therefore, we only need one GTS and no GACK-GTS here
    GTS preferredGTS = getNextFreeGTS(decision.preferredSuperframeId, decision.preferredSlotId, nullptr, nullptr);

    if(preferredGTS == GTS::UNDEFINED) {
        LOG_ERROR("No free GTS found! (trying with 0x" << HEXOUT << decision.deviceAddress << DECOUT << ")");
        gtsConfirmPending = false;
        return;
    }

    mlme_sap::DSME_GTS::request_parameters params;
    params.deviceAddress = decision.deviceAddress;
    params.managementType = ManagementType::ALLOCATION;
    params.direction = decision.direction;
    params.prioritizedChannelAccess = Priority::LOW;
    params.numSlot = decision.numSlot;
    params.preferredSuperframeId = preferredGTS.superframeID;
    params.preferredSlotId = preferredGTS.slotID;
    if(this->dsmeAdaptionLayer.getDSME().getPlatform().isGackEnabled()){    //request a GACK-GTS, if enabled
        params.gackGTS = true;
    }
    else{
        params.gackGTS = false;
    }

    params.dsmeSabSpecification.setSubBlockLengthBytes(this->dsmeAdaptionLayer.getMAC_PIB().helper.getSubBlockLengthBytes(preferredGTS.superframeID));
    params.dsmeSabSpecification.setSubBlockIndex(preferredGTS.superframeID);
    macDSMESAB.getOccupiedSubBlock(params.dsmeSabSpecification, preferredGTS.superframeID);

    LOG_INFO("ALLOCATING slot " << preferredGTS.slotID << " " << preferredGTS.superframeID << " " << (uint16_t)preferredGTS.channel << " with 0x" << HEXOUT
                                << params.deviceAddress << DECOUT << ".");

    /* mark all impossible slots that are in use in other channels, too */
    for(DSMEAllocationCounterTable::iterator it = macDSMEACT.begin(); it != macDSMEACT.end(); ++it) {
        if(it->getSuperframeID() == preferredGTS.superframeID) {
            for(uint8_t channel = 0; channel < numChannels; channel++) {
                params.dsmeSabSpecification.getSubBlock().set(it->getGTSlotID() * numChannels + channel, true);
            }
        }
    }

    this->dsmeAdaptionLayer.getMLME_SAP().getDSME_GTS().request(params);
    return;
}

void GTSHelper::checkAndDeallocateSingeleGTS(uint16_t address) {
    DSMEAllocationCounterTable& act = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT;
    int16_t highestIdleCounter = -1;
    DSMEAllocationCounterTable::iterator toDeallocate = act.end();
    for(auto it = act.begin(); it != act.end(); ++it) {
        if(it->getDirection() == Direction::TX && it->getAddress() == address) {
            if(it->getState() == ACTState::VALID && it->getIdleCounter() > highestIdleCounter && it->isGackGTS() == false) {
                //ignore GACK-GTS here, as they will be deallocated together with its GTS
                highestIdleCounter = it->getIdleCounter();
                toDeallocate = it;
            }
        }
    }

    if(toDeallocate != act.end()) {
        LOG_INFO("DEALLOCATING slot " << toDeallocate->getSuperframeID() << "/" << toDeallocate->getGTSlotID() << " with 0x" << HEXOUT
                                      << toDeallocate->getAddress() << DECOUT);

        DSMESABSpecification dsmeSABSpecification;
        uint8_t subBlockLengthBytes = this->dsmeAdaptionLayer.getMAC_PIB().helper.getSubBlockLengthBytes(toDeallocate->getSuperframeID());
        dsmeSABSpecification.setSubBlockLengthBytes(subBlockLengthBytes);
        dsmeSABSpecification.setSubBlockIndex(toDeallocate->getSuperframeID());
        dsmeSABSpecification.getSubBlock().fill(false);
        dsmeSABSpecification.getSubBlock().set(
            toDeallocate->getGTSlotID() * this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumChannels() + toDeallocate->getChannel(), true);

        sendDeallocationRequest(toDeallocate->getAddress(), toDeallocate->getDirection(), dsmeSABSpecification);
    }
}

void GTSHelper::handleCOMM_STATUS_indication(mlme_sap::COMM_STATUS_indication_parameters& params) {
    LOG_INFO("COMM_STATUS indication handled.");

    // TODO should we do anything here? especially for failures?

    return;
}

void GTSHelper::handleDSME_GTS_indication(mlme_sap::DSME_GTS_indication_parameters& params) {
    if(!this->dsmeAdaptionLayer.getMAC_PIB().macAssociatedPANCoord) {
        LOG_INFO("Not associated, discarding incoming GTS request from " << params.deviceAddress << ".");
        return;
    }

    LOG_INFO("GTS request handled.");

    mlme_sap::DSME_GTS::response_parameters responseParams;
    responseParams.deviceAddress = params.deviceAddress;
    responseParams.managementType = params.managementType;
    responseParams.direction = params.direction;
    responseParams.prioritizedChannelAccess = params.prioritizedChannelAccess;


    bool sendReply = true;

    switch(params.managementType) {
        case ALLOCATION: {
            responseParams.dsmeSabSpecification.setSubBlockLengthBytes(params.dsmeSabSpecification.getSubBlockLengthBytes());
            responseParams.dsmeSabSpecification.setSubBlockIndex(params.dsmeSabSpecification.getSubBlockIndex());

            DSME_ASSERT(params.dsmeSabSpecification.getSubBlockIndex() == params.preferredSuperframeId);

            GTS gackGTS = GTS::UNDEFINED;
            if(params.gackGTS){  //if GACK-GTS requested
                findFreeSlots(params.dsmeSabSpecification, responseParams.dsmeSabSpecification, params.numSlot, params.preferredSuperframeId,
                              params.preferredSlotId, &gackGTS);
                DSME_ASSERT(gackGTS != GTS::UNDEFINED);
            }
            else{
                findFreeSlots(params.dsmeSabSpecification, responseParams.dsmeSabSpecification, params.numSlot, params.preferredSuperframeId,
                              params.preferredSlotId, nullptr);
            }
            responseParams.gackGTS = gackGTS;

            responseParams.channelOffset = dsmeAdaptionLayer.getMAC_PIB().macChannelOffset;
            if(responseParams.dsmeSabSpecification.getSubBlock().isZero()) {
                LOG_ERROR("Unable to allocate GTS.");
                responseParams.status = GTSStatus::DENIED;
            } else {
                responseParams.status = GTSStatus::SUCCESS;
            }
            break;
        }
        case DUPLICATED_ALLOCATION_NOTIFICATION: {
            // TODO what shall we do here? With this the deallocation could be to aggressive
            uint16_t address = IEEE802154MacAddress::NO_SHORT_ADDRESS;
            Direction direction;
            responseParams.status = verifyDeallocation(params.dsmeSabSpecification, address, direction);

            if(responseParams.status == GTSStatus::SUCCESS) {
                // Now handled by the ACTUpdater this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.setACTState(params.dsmeSABSpecification, INVALID);
            } else {
                // the deallocated slot is not allocated, so send back as DENIED
            }

            sendReply = false; // TODO correct?

            break;
        }
        case DEALLOCATION: {
            Direction directionUnused;
            responseParams.status = verifyDeallocation(params.dsmeSabSpecification, params.deviceAddress, directionUnused);

            if(responseParams.status == GTSStatus::GTS_Status::SUCCESS) {
                // TODO remove
                // only set to INVALID here and remove them not before NOTIFY
                // if anything goes wrong, the slot will be deallocated later again
                // Now handled by the ACTUpdater this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.setACTState(params.dsmeSABSpecification, INVALID);
            }

            responseParams.dsmeSabSpecification = params.dsmeSabSpecification;
            break;
        }
        case EXPIRATION:
            // In this implementation EXPIRATION is only issued while no confirm is pending
            // DSME_ASSERT(!gtsConfirmPending);

            // TODO is this required?
            // this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.setACTState(params.dsmeSABSpecification, DEALLOCATED);

            sendDeallocationRequest(params.deviceAddress, params.direction, params.dsmeSabSpecification);
            sendReply = false;
            break;
        default:
            DSME_ASSERT(false);
            break;
    }

    if(sendReply) {
        this->dsmeAdaptionLayer.getMLME_SAP().getDSME_GTS().response(responseParams);
    }

    return;
}

void GTSHelper::sendDeallocationRequest(uint16_t address, Direction direction, DSMESABSpecification& sabSpecification) {
    mlme_sap::DSME_GTS::request_parameters params;
    params.deviceAddress = address;
    params.managementType = ManagementType::DEALLOCATION;
    params.direction = direction;
    params.prioritizedChannelAccess = Priority::LOW;

    params.numSlot = 0;               // ignored
    params.preferredSuperframeId = 0; // ignored
    params.preferredSlotId = 0;       // ignored

    params.dsmeSabSpecification = sabSpecification;

    LOG_INFO("Deallocating slot with " << params.deviceAddress << ".");

    this->dsmeAdaptionLayer.getMLME_SAP().getDSME_GTS().request(params);

    return;
}

const char* printStatus(GTSStatus::GTS_Status status) {
    switch(status) {
        case GTSStatus::SUCCESS:
            return "SUCCESS";
        case GTSStatus::DENIED:
            return "DENIED";
        case GTSStatus::INVALID_PARAMETER:
            return "INVALID_PARAMETER";
        case GTSStatus::NO_ACK:
            return "NO_ACK";
        case GTSStatus::NO_DATA:
            return "NO_DATA";
        case GTSStatus::CHANNEL_ACCESS_FAILURE:
            return "CHANNEL_ACCESS_FAILURE";
        case GTSStatus::NO_SHORT_ADDRESS:
            return "NO_SHORT_ADDRESS";
        case GTSStatus::TRANSACTION_OVERFLOW:
            return "TRANSACTION_OVERFLOW";
    }

    DSME_ASSERT(false);
    return "UNKNOWN";
}

void GTSHelper::handleDSME_GTS_confirm(mlme_sap::DSME_GTS_confirm_parameters& params) {
    LOG_DEBUG("GTS confirmation handled (Status: " << printStatus(params.status) << ").");

    // TODO handle channel access failure! retransmission?

    if(params.managementType == ManagementType::ALLOCATION) {
        gtsConfirmPending = false;
        LOG_DEBUG("gtsConfirmPending = false");
        if(params.status == GTSStatus::SUCCESS) {
            this->dsmeAdaptionLayer.getMessageHelper().sendRetryBuffer();
        }
        if(params.status != GTSStatus::TRANSACTION_OVERFLOW) {
            performSchedulingAction(this->gtsScheduling->getNextSchedulingAction());
        }
    }
    return;
}

GTS GTSHelper::getNextFreeGTSBefore(uint16_t superframeID, uint8_t slotID, uint8_t lowestSlotID, const DSMESABSpecification* sabSpec)
{
    DSMEAllocationCounterTable& macDSMEACT = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT;
    DSMESlotAllocationBitmap& macDSMESAB = this->dsmeAdaptionLayer.getMAC_PIB().macDSMESAB;
    uint8_t numChannels = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumChannels();
    BitVector<MAX_CHANNELS> occupied;
    BitVector<MAX_CHANNELS> remoteOccupied; // only used if sabSpec != nullptr
    occupied.setLength(numChannels);
    remoteOccupied.setLength(numChannels);

    GTS currentGTS(superframeID, slotID-1, 0);
    for(; currentGTS.slotID >= lowestSlotID; currentGTS.slotID--) {
        LOG_INFO("Checking SlotID " << currentGTS.slotID << " in superframeID " << currentGTS.superframeID);
        if(!macDSMEACT.isAllocated(currentGTS.superframeID, currentGTS.slotID)) {   //if free slot available
            macDSMESAB.getOccupiedChannels(occupied, currentGTS.superframeID, currentGTS.slotID); //find a free channel
            if(sabSpec != nullptr) {    //if remoteSAB is available, merge with ownSAB
                remoteOccupied.copyFrom(sabSpec->getSubBlock(), currentGTS.slotID * numChannels);
                occupied.setOperationJoin(remoteOccupied);
            }
            currentGTS.channel = this->dsmeAdaptionLayer.getDSME().getPlatform().getRandom() % numChannels;
            for(uint8_t i = 0; i < numChannels; i++) {
                if(!occupied.get(currentGTS.channel)) {    /* found one */
                    return currentGTS;
                    break;
                }
                currentGTS.channel = (currentGTS.channel+1)%numChannels;
            }
        }
        if(currentGTS.slotID == 0){ //for loop has to be stopped here to prevent underflow
            break;
        }
    }
    return GTS::UNDEFINED;
}


GTS GTSHelper::getNextFreeGTS(uint16_t preferredSuperframeID, uint8_t preferredSlotID, const DSMESABSpecification* sabSpec, GTS *closestGackGTS)
{
    DSMEAllocationCounterTable& macDSMEACT = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT;
    DSMESlotAllocationBitmap& macDSMESAB = this->dsmeAdaptionLayer.getMAC_PIB().macDSMESAB;

    uint8_t numChannels = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumChannels();
    uint8_t numSuperFramesPerMultiSuperframe = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe();
    uint16_t slotsToCheck = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(0) +
                            (this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe() - 1) *
                                this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(1);
    uint8_t numGackGTSPerMultiSuperframe = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumberGroupAckSlotsPerMultiSuperframe();
    uint8_t maxSlotShift = 3; //maximum difference between last slotID of a CFP and GACK-GTS SlotID

    BitVector<MAX_CHANNELS> occupied;
    BitVector<MAX_CHANNELS> remoteOccupied; // only used if sabSpec != nullptr
    occupied.setLength(numChannels);

    if(sabSpec != nullptr) {
        DSME_ASSERT(sabSpec->getSubBlockIndex() == preferredSuperframeID);
        remoteOccupied.setLength(numChannels);
    }
    GTS currentGTS(0, 0, 0);

    if(closestGackGTS != nullptr)   //if GACK-GTS requested, try to find a slot in front of a GACK-GTS
    {
        LOG_DEBUG("GACK: getNextFreeGTS, Gack-GTS requested");
        if(*closestGackGTS != GTS::UNDEFINED)  //if a GACK-GTS is given, it has to be used
        {
            LOG_DEBUG("GACK: Gack-GTS given: Slot" << (int)closestGackGTS->slotID << " in SF" << (int)closestGackGTS->superframeID);
            currentGTS = getNextFreeGTSBefore(closestGackGTS->superframeID, closestGackGTS->slotID, 0, sabSpec);
            LOG_DEBUG("GACK: Found GTS: Slot" << (int)currentGTS.slotID << " in SF" << (int)currentGTS.superframeID);
            return currentGTS; //either a found slot or a GTS::UNDEFINED
        }

        //check closestGackGTS to preferred superframe
        uint8_t GackGTSSpacing = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumberSuperframesPerGroupAckSlot();
        uint16_t closestGackGTSDistance = GTS::UNDEFINED.superframeID;
        DSMEAllocationCounterTable::iterator it = macDSMEACT.begin();
        LOG_DEBUG("GACK: searching closestGackGTS");
        LOG_DEBUG("GACK: log all current GTS:");
        while(it != macDSMEACT.end()) {
            LOG_DEBUG("GACK: SF:"<<(int)it->getSuperframeID()<<" Slot:"<<(int)it->getGTSlotID()<<" GACK-GTS: "<<(it->isGackGTS()?"true":"false")<<" Direction: "<<(it->getDirection()?"RX":"TX"));
            if(it->isGackGTS()){    //get all already allocated GACK-GTS
                uint8_t currentDistance = abs((int8_t)it->getSuperframeID()-preferredSuperframeID);
                LOG_DEBUG("GACK: Slot" << (int)it->getGTSlotID() << " in SF" << (int)it->getSuperframeID() << " distance:" << (int)currentDistance);
                //The requested gackGTS is always in TX direction
                if(currentDistance <= closestGackGTSDistance && it->getDirection() == TX){ //if distance is lower or equal to lowest GackGTSDistance
                    closestGackGTSDistance = currentDistance;
                    closestGackGTS->slotID = it->getGTSlotID();
                    closestGackGTS->superframeID = it->getSuperframeID();
                    closestGackGTS->channel = it->getChannel();
                }
            }
            it++;
        }
        LOG_DEBUG("GACK: log done.");
        LOG_DEBUG("GACK: superframesPerMSF: " << (int)numSuperFramesPerMultiSuperframe);
        LOG_DEBUG("GACK: gackGTSPerMSF: " << (int)numGackGTSPerMultiSuperframe);
        if(*closestGackGTS == GTS::UNDEFINED){
            LOG_DEBUG("GACK: no GACKGTS available yet");
        }else{
            LOG_DEBUG("GACK: closest Slot" << (int)closestGackGTS->slotID << " in SF" << (int)closestGackGTS->superframeID << " distance:" << (int)closestGackGTSDistance);
            LOG_DEBUG("GACK: closestGackGTS: " << (int)closestGackGTSDistance << " >= GackGTSSpacing: " << (int)GackGTSSpacing << "?");
        }

        if(closestGackGTSDistance >= GackGTSSpacing){
            LOG_DEBUG("GACK: true! allocate new GACK-GTS");
            //allocate new GackGTS, calculate next superframeID
            uint16_t nextSuperframeID = (closestGackGTS->superframeID + GackGTSSpacing)%numSuperFramesPerMultiSuperframe;
            uint8_t lastSlotID = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(nextSuperframeID)-1;
            LOG_DEBUG("GACK: not close enough, allocate new GACK-GTS in SF" << (int)nextSuperframeID);
            *closestGackGTS = getNextFreeGTSBefore(nextSuperframeID, lastSlotID, lastSlotID-maxSlotShift, sabSpec);
            if(*closestGackGTS == GTS::UNDEFINED){
                LOG_DEBUG("GACK: no GACK-GTS slot found, abort!");
                return GTS::UNDEFINED; //GACK-GTS allocation not successful, maybe increasing maxSlotShift?
            }
            LOG_DEBUG("GACK: found GACK-GTS in slot " << (int)closestGackGTS->slotID);
        }else{
            LOG_DEBUG("GACK: false! return previous GACK-GTS");
        }
        //if distance lower than GackGTSSpacing, use closestGackGTS as chosen GackGTS
        //check in front of it, if a slot is available, running backwards from Gack-SlotID
        /*currentGTS = getNextFreeGTSBefore(closestGackGTS->superframeID, closestGackGTS->slotID, 0, sabSpec);
        if(currentGTS != GTS::UNDEFINED)
        {
            LOG_DEBUG("GACK: found GTS in Slot" << (int)currentGTS.slotID << " in SF" << (int)currentGTS.superframeID);
            return currentGTS; //found a slot
        }else{
            LOG_DEBUG("GACK: no GTS slot found, abort!");
            return GTS::UNDEFINED;
        }*/
    }
    //No GACK-GTS requested
    for(currentGTS.superframeID = preferredSuperframeID; slotsToCheck > 0; currentGTS.superframeID = (currentGTS.superframeID + 1) % numSuperFramesPerMultiSuperframe) {
        if(sabSpec != nullptr && currentGTS.superframeID != preferredSuperframeID) {
            /* currently per convention a sub block holds exactly one superframe */
            return GTS::UNDEFINED;
        }

        uint8_t numGTSlots = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(currentGTS.superframeID);
        LOG_INFO("Checking " << numGTSlots << " in superframe " << currentGTS.superframeID);
        for(currentGTS.slotID = preferredSlotID % numGTSlots; slotsToCheck > 0; currentGTS.slotID = (currentGTS.slotID + 1) % numGTSlots) {
            if(!macDSMEACT.isAllocated(currentGTS.superframeID, currentGTS.slotID)) {
                uint8_t startChannel = this->dsmeAdaptionLayer.getDSME().getPlatform().getRandom() % numChannels;
                macDSMESAB.getOccupiedChannels(occupied, currentGTS.superframeID, currentGTS.slotID);
                if(sabSpec != nullptr) {
                    remoteOccupied.copyFrom(sabSpec->getSubBlock(), currentGTS.slotID * numChannels);
                    occupied.setOperationJoin(remoteOccupied);
                }

                currentGTS.channel = startChannel;
                for(uint8_t i = 0; i < numChannels; i++) {
                    if(!occupied.get(currentGTS.channel)) {
                        /* found one */
                        return currentGTS;
                    }

                    currentGTS.channel++;
                    if(currentGTS.channel == numChannels) {
                        currentGTS.channel = 0;
                    }
                }
            }
            slotsToCheck--;
            if((currentGTS.slotID+1)%numGTSlots == preferredSuperframeID) {
                break;
            }
        }
    }
    return GTS::UNDEFINED;
}

GTSStatus::GTS_Status GTSHelper::verifyDeallocation(DSMESABSpecification& requestSABSpec, uint16_t& deviceAddress, Direction& direction) {
    GTSStatus::GTS_Status result;

    DSMEAllocationCounterTable& macDSMEACT = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT;

    uint8_t numChannels = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumChannels();

    bool foundGts = false;
    bool gtsDifferentAddresses = false;

    for(DSMEAllocationCounterTable::iterator it = macDSMEACT.begin(); it != macDSMEACT.end(); ++it) {
        abs_slot_idx_t idx = it->getGTSlotID();
        idx *= numChannels;
        idx += it->getChannel();

        if(it->getSuperframeID() != requestSABSpec.getSubBlockIndex() || !requestSABSpec.getSubBlock().get(idx)) {
            continue; // no deallocation requested
        }

        if(deviceAddress == IEEE802154MacAddress::NO_SHORT_ADDRESS) {
            deviceAddress = it->getAddress();
            direction = it->getDirection();
            foundGts = true;
        } else if(deviceAddress == it->getAddress()) {
            foundGts = true;
        } else {
            DSME_ASSERT(!foundGts); // currently deallocation is only possible with one slot
            gtsDifferentAddresses = true;
        }
    }

    if(gtsDifferentAddresses) {
        // TODO handle multiple requests or also send (INVALID_PARAMETER)?!
        // DSME_ASSERT(false); // TODO ?
        // TODO This could also mean that the slot is in use with another node, better DENIED?
        return GTSStatus::DENIED;
    }

    if(foundGts) {
        result = GTSStatus::SUCCESS;
    } else {
        result = GTSStatus::DENIED;
    }

    return result;
}

void GTSHelper::findFreeSlots(DSMESABSpecification& requestSABSpec, DSMESABSpecification& replySABSpec, uint8_t numSlots, uint16_t preferredSuperframe,
                              uint8_t preferredSlot, GTS *selectedGackGTS) {
    const uint8_t numChannels = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumChannels();

    //GACK-GTS Allocation procedure
    //check if distance to previous GACK-GTS higher than distance to next predetermined GACK-GTS
    //yes -> allocate new GACK-GTS at predetermined superframe
    //no -> return previous GACK-GTS

    for(uint8_t i = 0; i < numSlots; i++) {
        GTS gts = getNextFreeGTS(preferredSuperframe, preferredSlot, &requestSABSpec, selectedGackGTS);

        if(gts == GTS::UNDEFINED) {
            break;
        }

        /* mark slot as allocated. Dont mark GACK-GTS Slot. Will be handled differently*/
        replySABSpec.getSubBlock().set(gts.slotID * numChannels + gts.channel, true);
        //replySABSpec.getSubBlock().set(selectedGackGTS->slotID * numChannels + selectedGackGTS->channel, true);

        if(i < numSlots - 1) {
            /* mark already allocated slots as occupied for next round */
            for(uint8_t channel = 0; channel < numChannels; channel++) {
                requestSABSpec.getSubBlock().set(gts.slotID * numChannels + channel, true);
            }

            preferredSuperframe = gts.superframeID;
            preferredSlot = gts.slotID;
        }
    }
    return;
}

} /* namespace dsme */
