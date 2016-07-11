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

#include "../../DSMEPlatform.h"
#include "../dsmeLayer/DSMELayer.h" // TODO: remove cross-layer reference
#include "../mac_services/dataStructures/DSMESABSpecification.h"
#include "../mac_services/pib/dsme_mac_constants.h"
#include "DSMEAdaptionLayer.h"
#include "GTSHelper.h"

namespace dsme {

GTSHelper::GTSHelper(DSMEAdaptionLayer& dsmeAdaptionLayer) :
        dsmeAdaptionLayer(dsmeAdaptionLayer),
        gtsConfirmPending(false) {
}

void GTSHelper::initialize() {
    this->dsmeAdaptionLayer.getMLME_SAP().getDSME_GTS().indication(DELEGATE(&GTSHelper::handleDSME_GTS_indication, *this));
    this->dsmeAdaptionLayer.getMLME_SAP().getDSME_GTS().confirm(DELEGATE(&GTSHelper::handleDSME_GTS_confirm, *this));
    this->dsmeAdaptionLayer.getMLME_SAP().getCOMM_STATUS().indication(DELEGATE(&GTSHelper::handleCOMM_STATUS_indication, *this));
    return;
}

void GTSHelper::checkAndAllocateSingleGTS(uint16_t address) {
    if (gtsConfirmPending) {
        LOG_INFO("GTS allocation still active.");
        return;
    }

    DSMEAllocationCounterTable& macDSMEACT = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT;
    DSMESlotAllocationBitmap& macDSMESAB = this->dsmeAdaptionLayer.getMAC_PIB().macDSMESAB;

    uint16_t numAllocatedSlots = macDSMEACT.getNumAllocatedTxGTS(address);
    uint16_t numPacketsInQueue = this->dsmeAdaptionLayer.getMCPS_SAP().getMessageCount(address);
    LOG_INFO("Currently " << numAllocatedSlots << " slots are allocated for " << address << ".");

    LOG_INFO("Currently " << (uint16_t) numPacketsInQueue << " packets are queued for " << address << ".");

    if (numAllocatedSlots == numPacketsInQueue + 1) {
        LOG_INFO("Enough slots are already reserved.");
        return;
    } else if (numAllocatedSlots > numPacketsInQueue + 3) {
        LOG_INFO("Possibly too many slots are reserved.");
        //checkAndDeallocateSingeleGTS(address); /TODO: readd after testing
        return;
    }

    LOG_INFO("More slots have to be reserved.");

    uint8_t numChannels = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumChannels();
    uint8_t numGTSlots = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots();
    uint8_t subBlockLengthBytes = this->dsmeAdaptionLayer.getMAC_PIB().helper.getSubBlockLengthBytes();

    /* select random or contiguous slot */
    GTS preferredGTS = GTS::UNDEFINED;
    switch (this->dsmeAdaptionLayer.settings.allocationScheme) {
        case DSMEAdaptionLayerSettings::ALLOC_RANDOM:
            preferredGTS = this->getRandomFreeGTS();
            break;
        case DSMEAdaptionLayerSettings::ALLOC_CONTIGUOUS_SLOT:
            preferredGTS = getContiguousFreeGTS();
            break;
    }

    if (preferredGTS == GTS::UNDEFINED) {
        LOG_WARN("No free GTS found!");
        return;
    }

    mlme_sap::DSME_GTS::request_parameters params;
    params.deviceAddress = address;
    params.managmentType = ManagementType::ALLOCATION;
    params.direction = Direction::TX;
    params.prioritizedChannelAccess = Priority::LOW;
    params.numSlot = 1;
    params.preferredSuperframeID = preferredGTS.superframeID;
    params.preferredSlotID = preferredGTS.slotID;

    params.dsmeSABSpecification.setSubBlockLengthBytes(subBlockLengthBytes);
    params.dsmeSABSpecification.setSubBlockIndex(preferredGTS.superframeID);
    macDSMESAB.getOccupiedSubBlock(params.dsmeSABSpecification, preferredGTS.superframeID * numGTSlots * numChannels);

    params.securityLevel = 0;
    params.keyIdMode = 0;
    params.keySource = nullptr;
    params.keyIndex = 0;

    LOG_INFO(
            "Requesting slot " << preferredGTS.superframeID << "/" << preferredGTS.slotID << "/" << (uint16_t)preferredGTS.channel << " for transmission to " << params.deviceAddress << ".");

    /* mark all impossible slots that are in use in other channels, too */
    DSMESABSpecification::SABSubBlock& subBlock = params.dsmeSABSpecification.getSubBlock();
    for (DSMEAllocationCounterTable::iterator it = macDSMEACT.begin(); it != macDSMEACT.end(); it++) {
        if (it->getSuperframeID() == preferredGTS.superframeID) {
            for (uint8_t channel = 0; channel < numChannels; channel++) {
                subBlock.set(it->getGTSlotID() * numChannels + channel, true);
            }
        }
    }

    superframesSinceGtsRequestSent = 0;
    DSME_ASSERT(!gtsConfirmPending);
    gtsConfirmPending = true;
    LOG_DEBUG("gtsConfirmPending = true");
    this->dsmeAdaptionLayer.getMLME_SAP().getDSME_GTS().request(params);
    return;
}

void GTSHelper::checkAndDeallocateSingeleGTS(uint16_t address) {
    DSME_ASSERT(!gtsConfirmPending);

    DSMEAllocationCounterTable& act = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT;
    uint16_t highestIdleCounter = 0;
    DSMEAllocationCounterTable::iterator toDeallocate;
    for (auto it = act.begin(); it != act.end(); ++it) {
        if (it->getDirection() == Direction::TX && it->getAddress() == address) {
            if (it->getIdleCounter() > highestIdleCounter) {
                highestIdleCounter = it->getIdleCounter();
                toDeallocate = it;
            }
        }
    }

    if (highestIdleCounter != 0) {
        LOG_INFO("#Now deallocate " << toDeallocate->getSuperframeID() << "/" << toDeallocate->getGTSlotID() << ".");

        DSMESABSpecification dsmeSABSpecification;
        uint8_t subBlockLengthBytes = this->dsmeAdaptionLayer.getMAC_PIB().helper.getSubBlockLengthBytes();
        dsmeSABSpecification.setSubBlockLengthBytes(subBlockLengthBytes);
        dsmeSABSpecification.setSubBlockIndex(toDeallocate->getSuperframeID());
        dsmeSABSpecification.getSubBlock().fill(false);
        dsmeSABSpecification.getSubBlock().set(
                toDeallocate->getGTSlotID() * this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumChannels() + toDeallocate->getChannel(),
                true);

        sendDeallocationRequest(toDeallocate->getAddress(), toDeallocate->getDirection(), dsmeSABSpecification);
    }
}

void GTSHelper::handleCOMM_STATUS_indication(mlme_sap::COMM_STATUS_indication_parameters &params) {
    LOG_INFO("COMM_STATUS indication handled.");

    // TODO should we do anything here? especially for failures?

    return;
}

void GTSHelper::handleDSME_GTS_indication(mlme_sap::DSME_GTS_indication_parameters &params) {
    if (!this->dsmeAdaptionLayer.getMAC_PIB().macAssociatedPANCoord) {
        LOG_INFO("Not associated, discarding incoming GTS request from " << params.deviceAddress << ".");
        return;
    }

    LOG_INFO("GTS request handled.");

    mlme_sap::DSME_GTS::response_parameters responseParams;
    responseParams.deviceAddress = params.deviceAddress;
    responseParams.managmentType = params.managmentType;
    responseParams.direction = params.direction;
    responseParams.prioritizedChannelAccess = params.prioritizedChannelAccess;
    responseParams.channelOffset = 0;

    bool sendReply = true;

    switch (params.managmentType) {
        case ALLOCATION: {
            responseParams.dsmeSABSpecification.setSubBlockLengthBytes(params.dsmeSABSpecification.getSubBlockLengthBytes());
            responseParams.dsmeSABSpecification.setSubBlockIndex(params.dsmeSABSpecification.getSubBlockIndex());

            findFreeSlots(params.dsmeSABSpecification, responseParams.dsmeSABSpecification, params.numSlot, params.preferredSuperframeID,
                    params.preferredSlotID);

            if (responseParams.dsmeSABSpecification.getSubBlock().isZero()) {
                LOG_WARN("Unable to allocate GTS.");
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
            responseParams.status = verifyDeallocation(params.dsmeSABSpecification, address, direction);

            if (responseParams.status == GTSStatus::SUCCESS) {
                // Now handled by the ACTUpdater this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.setACTState(params.dsmeSABSpecification, INVALID);
            } else {
                // the deallocated slot is not allocated, so send back as DENIED
            }

            sendReply = false; // TODO correct?

            break;
        }
        case DEALLOCATION: {
            Direction directionUnused;
            responseParams.status = verifyDeallocation(params.dsmeSABSpecification, params.deviceAddress, directionUnused);

            if (responseParams.status == GTSStatus::GTS_Status::SUCCESS) {
                // TODO remove
                // only set to INVALID here and remove them not before NOTIFY
                // if anything goes wrong, the slot will be deallocated later again
                // Now handled by the ACTUpdater this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.setACTState(params.dsmeSABSpecification, INVALID);
            }

            responseParams.dsmeSABSpecification = params.dsmeSABSpecification;
            break;
        }
        case EXPIRATION:
            // In this implementation EXPIRATION is only issued while no confirm is pending
            DSME_ASSERT(!gtsConfirmPending)
            ;

            // TODO is this required?
            //this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.setACTState(params.dsmeSABSpecification, DEALLOCATED);

            sendDeallocationRequest(params.deviceAddress, params.direction, params.dsmeSABSpecification);
            sendReply = false;
            break;
        default:
            DSME_ASSERT(false)
            ;
            break;
    }

    if (sendReply) {
        this->dsmeAdaptionLayer.getMLME_SAP().getDSME_GTS().response(responseParams);
    }

    return;
}

void GTSHelper::sendDeallocationRequest(uint16_t address, Direction direction, DSMESABSpecification& sabSpecification) {
    mlme_sap::DSME_GTS::request_parameters params;
    params.deviceAddress = address;
    params.managmentType = ManagementType::DEALLOCATION;
    params.direction = direction;
    params.prioritizedChannelAccess = Priority::LOW;

    params.dsmeSABSpecification = sabSpecification;

    params.securityLevel = 0;
    params.keyIdMode = 0;
    params.keySource = nullptr;
    params.keyIndex = 0;

    LOG_INFO("Deallocating slot with " << params.deviceAddress << ".");

    superframesSinceGtsRequestSent = 0;
    gtsConfirmPending = true;
    LOG_DEBUG("gtsConfirmPending = true");
    this->dsmeAdaptionLayer.getMLME_SAP().getDSME_GTS().request(params);

    return;
}

const char* printStatus(GTSStatus::GTS_Status status) {
    switch (status) {
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

void GTSHelper::handleDSME_GTS_confirm(mlme_sap::DSME_GTS_confirm_parameters &params) {
    LOG_INFO("GTS confirmation handled.");

    // TODO handle channel access failure! retransmission?

    gtsConfirmPending = false;
    LOG_DEBUG("gtsConfirmPending = false");
    return;
}

GTS GTSHelper::getContiguousFreeGTS() {
    uint8_t initialSuperframeID = this->dsmeAdaptionLayer.getDSME().getCurrentSuperframe();
    uint8_t initialSlotID = 0;

    return getNextFreeGTS(initialSuperframeID, initialSlotID);
}

GTS GTSHelper::getRandomFreeGTS() {
    uint8_t numSuperFramesPerMultiSuperframe = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe();
    uint8_t numGTSlots = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots();

    uint8_t initialSuperframeID = this->dsmeAdaptionLayer.getDSME().getPlatform().getRandom() % numSuperFramesPerMultiSuperframe;
    uint8_t initialSlotID = this->dsmeAdaptionLayer.getDSME().getPlatform().getRandom() % numGTSlots;

    return getNextFreeGTS(initialSuperframeID, initialSlotID);
}

GTS GTSHelper::getNextFreeGTS(uint16_t initialSuperframeID, uint8_t initialSlotID, const DSMESABSpecification* sabSpec) {
    DSMEAllocationCounterTable& macDSMEACT = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT;
    DSMESlotAllocationBitmap& macDSMESAB = this->dsmeAdaptionLayer.getMAC_PIB().macDSMESAB;

    uint8_t numChannels = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumChannels();
    uint8_t numGTSlots = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots();
    uint8_t numSuperFramesPerMultiSuperframe = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe();
    uint16_t slotsToCheck = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe() * numGTSlots;

    GTS gts(0, 0, 0);

    if (sabSpec != nullptr) {
        DSME_ASSERT(sabSpec->getSubBlockIndex() == initialSuperframeID);
    }

    for (gts.superframeID = initialSuperframeID; slotsToCheck > 0;
            gts.superframeID = (gts.superframeID + 1) % numSuperFramesPerMultiSuperframe) {

        if (sabSpec != nullptr && gts.superframeID != initialSuperframeID) {
            /* currently per convention a sub block holds exactly one superframe */
            return GTS::UNDEFINED;
        }

        for (gts.slotID = initialSlotID; slotsToCheck > 0; gts.slotID = (gts.slotID + 1) % numGTSlots) {
            if (!macDSMEACT.isAllocated(gts.superframeID, gts.slotID)) {
                for (gts.channel = 0; gts.channel < numChannels; gts.channel++) {
                    if (!macDSMESAB.isOccupied(gts.absoluteIndex(numGTSlots, numChannels))) {
                        if (sabSpec == nullptr || !sabSpec->getSubBlock().get(gts.slotID * numChannels + gts.channel)) {
                            /* found one */
                            // LOG_INFO("Next free GTS is " << gts.superframeID << "/" << gts.slotID << "/" << (uint16_t)gts.channel << ".");
                            return gts;
                        }
                    }
                }
            }
            slotsToCheck--;
        }
    }

    return GTS::UNDEFINED;
}

GTSStatus::GTS_Status GTSHelper::verifyDeallocation(DSMESABSpecification &requestSABSpec, uint16_t &deviceAddress, Direction& direction) {
    GTSStatus::GTS_Status result;

    DSMEAllocationCounterTable& macDSMEACT = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT;

    uint8_t numChannels = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumChannels();

    bool foundGts = false;
    bool gtsDifferentAddresses = false;

    for (DSMEAllocationCounterTable::iterator it = macDSMEACT.begin(); it != macDSMEACT.end(); it++) {
        abs_slot_idx_t idx = it->getGTSlotID();
        idx *= numChannels;
        idx += it->getChannel();

        if (it->getSuperframeID() != requestSABSpec.getSubBlockIndex() || !requestSABSpec.getSubBlock().get(idx)) {
            continue;			// no deallocation requested
        }

        if (deviceAddress == IEEE802154MacAddress::NO_SHORT_ADDRESS) {
            deviceAddress = it->getAddress();
            direction = it->getDirection();
            foundGts = true;
        } else if (deviceAddress == it->getAddress()) {
            foundGts = true;
        } else {
            DSME_ASSERT(!foundGts); // currently deallocation is only possible with one slot
            gtsDifferentAddresses = true;
        }
    }

    if (gtsDifferentAddresses) {
        // TODO handle multiple requests or also send (INVALID_PARAMETER)?!
        //EV_ERROR << "DSME deallocateGTSlots: slots belong to different addresses!" << endl;
        //DSME_ASSERT(false); // TODO ?
        // TODO This could also mean that the slot is in use with another node, better DENIED?
        return GTSStatus::DENIED;
    }

    if (foundGts) {
        result = GTSStatus::SUCCESS;
    } else {
        result = GTSStatus::DENIED;
        //EV_WARN << "DSME deallocatedGTSlots: no allocated Slots were found (" << dest << ") -> DISAPPROVED" << endl;
    }

    return result;
}

void GTSHelper::findFreeSlots(DSMESABSpecification &requestSABSpec, DSMESABSpecification &replySABSpec, uint8_t numSlots,
        uint16_t preferredSuperframe, uint8_t preferredSlot) {
    const uint8_t numChannels = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumChannels();

    for (uint8_t i = 0; i < numSlots; i++) {
        GTS gts = getNextFreeGTS(preferredSuperframe, preferredSlot, &requestSABSpec);

        if (gts == GTS::UNDEFINED) {
            break;
        }

        /* mark slot as allocated */
        replySABSpec.getSubBlock().set(gts.slotID * numChannels + gts.channel, true);

        if (i < numSlots - 1) {
            /* mark already allocated slots as occupied for next round */
            for (uint8_t channel = 0; channel < numChannels; channel++) {
                requestSABSpec.getSubBlock().set(gts.slotID * numChannels + channel, true);
            }

            preferredSuperframe = gts.superframeID;
            preferredSlot = gts.slotID;
        }
    }
    return;
}

void GTSHelper::handleSlotEvent() {
    if (this->dsmeAdaptionLayer.getDSME().getCurrentSlot() == 0) {
        if (gtsConfirmPending) {
            superframesSinceGtsRequestSent++;

            // If gtsAllocationSent status was not reset (reply never received) for too long, reset it now
            // TODO make this configurable or think of different handling
            // TODO probably remove it since the MAC layer has this timeout, too
            // macResponseWaitTime is given in aBaseSuperframeDurations (that do not include the superframe order)
            if (superframesSinceGtsRequestSent * (1 << this->dsmeAdaptionLayer.getDSME().getMAC_PIB().macSuperframeOrder)
                    > this->dsmeAdaptionLayer.getDSME().getMAC_PIB().macResponseWaitTime * 2) {
                LOG_WARN("gtsConfirmPending status was not reset for a long time -> reset");
                DSME_ASSERT(false); // this should not happen since the MLME has a timeout itself!
                gtsConfirmPending = false;
                LOG_DEBUG("gtsConfirmPending = false");
            }
        }
    }
}

void GTSHelper::print(const char* name, const DSMESABSpecification &spec) {
    const BitVector<MAX_GTSLOTS * MAX_CHANNELS * MAX_SAB_UNITS> &subblock = spec.getSubBlock();
    const uint8_t numChannels = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumChannels();
    const uint8_t numGTSlots = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots();

    LOG_INFO_PREFIX
;
LOG_INFO_PURE("Slot allocation Bitmap[");
LOG_INFO_PURE(spec.getSubBlockIndex());
LOG_INFO_PURE("] (");
LOG_INFO_PURE(name);
LOG_INFO_PURE(")\n");

for (uint8_t c = 0; c < numChannels; c++) {
    if (c < 10) {
        LOG_INFO_PURE(" ");
    }
    LOG_INFO_PURE((uint16_t )c);
    LOG_INFO_PURE(": ");
    for (uint8_t s = 0; s < numGTSlots; s++) {
        LOG_INFO_PURE(subblock.get(c + s * numChannels));LOG_INFO_PURE(" ");
    }LOG_INFO_PURE("\n");
}
return;
}

void GTSHelper::print(const char* name, const DSMEAllocationCounterTable &act) {
const uint8_t numGTSlots = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots();
const uint8_t numSuperFramesPerMultiSuperframe = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe();

LOG_INFO_PREFIX
;
LOG_INFO_PURE("macDSMEACT (");
LOG_INFO_PURE(name);
LOG_INFO_PURE(")\n");

for (uint8_t m = 0; m < numSuperFramesPerMultiSuperframe; m++) {
if (m < 10) {
    LOG_INFO_PURE(" ");
}
LOG_INFO_PURE((uint16_t )m);
LOG_INFO_PURE(": ");
for (uint8_t s = 0; s < numGTSlots; s++) {
    LOG_INFO_PURE(act.isAllocated(m, s));LOG_INFO_PURE(" ");
}LOG_INFO_PURE("\n");
}
return;
}

} /* dsme */
