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

#include "BeaconManager.h"

#include "../../../dsme_platform.h"
#include "../../mac_services/dataStructures/PANDescriptor.h"
#include "../DSMELayer.h"
#include "../messages/BeaconNotificationCmd.h"
#include "../messages/MACCommand.h"

namespace dsme {

BeaconManager::BeaconManager(DSMELayer& dsme) :
        dsme(dsme),
        doneCallback(DELEGATE(&BeaconManager::sendDone, *this)),
        scanning(
        false) {
}

void BeaconManager::initialize() {
    dsmePANDescriptor.getBeaconBitmap().setSDBitmapLengthBytes(
            BITVECTOR_BYTE_LENGTH(dsme.getMAC_PIB().helper.getNumberSuperframesPerBeaconInterval()),
            false);
    heardBeacons.setSDBitmapLengthBytes(BITVECTOR_BYTE_LENGTH(dsme.getMAC_PIB().helper.getNumberSuperframesPerBeaconInterval()),
    false);
    neighborOrOwnHeardBeacons.setSDBitmapLengthBytes(
            BITVECTOR_BYTE_LENGTH(dsme.getMAC_PIB().helper.getNumberSuperframesPerBeaconInterval()),
            false);

    isBeaconAllocated = dsme.getDSMESettings().isPANCoordinator;
    isBeaconAllocationSent = false;

    // PAN Coordinator starts network with beacon
    if (dsme.getDSMESettings().isPANCoordinator) {
        dsmePANDescriptor.getBeaconBitmap().setSDIndex(0);
        dsmePANDescriptor.getBeaconBitmap().fill(false);
        dsmePANDescriptor.getBeaconBitmap().set(0, true);
    } else if (dsme.getDSMESettings().isCoordinator) {
        dsmePANDescriptor.getBeaconBitmap().fill(false);
    }

    lastKnownBeaconIntervalStart = dsme.getPlatform().getSymbolCounter();
    //lastHeardBeaconSymbolCounter = dsme.getPlatform().getSymbolCounter();
    lastHeardBeaconTimestamp = 0;
}

void BeaconManager::superframeEvent(uint16_t currentSuperframe, uint16_t currentMultiSuperframe, uint32_t lateness) {
    if (isBeaconAllocated) {
        uint16_t currentSDIndex = currentSuperframe
                + dsme.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe() * currentMultiSuperframe;
        if (currentSDIndex == dsmePANDescriptor.getBeaconBitmap().getSDIndex()) {
            sendEnhancedBeacon(lateness);
        }
    }
}

void BeaconManager::sendEnhancedBeacon(uint32_t lateness) {
    DSMEMessage* msg = dsme.getPlatform().getEmptyMessage();

    dsmePANDescriptor.getTimeSyncSpec().setBeaconTimestampMicroSeconds(0); // TODO !!!
    dsmePANDescriptor.getTimeSyncSpec().setBeaconOffsetTimestampMicroSeconds(lateness * symbolDurationInMicroseconds);
    dsmePANDescriptor.prependTo(msg); // TODO this should be implemented as IE

    msg->getHeader().setDstAddr(IEEE802154MacAddress::SHORT_BROADCAST_ADDRESS);
    msg->getHeader().setDstAddrMode(SHORT_ADDRESS);
    msg->getHeader().setSrcAddr(dsme.getMAC_PIB().macExtendedAddress);
    msg->getHeader().setFrameType(IEEE802154eMACHeader::BEACON);

    if (dsme.getDSMESettings().isPANCoordinator) {
        // TODO only for PAN coordinator or for all coordinators?
        // the PAN coordinator is the reference, so set the sent beacon as heard beacon to advance in time
        // TODO timing?
        lastKnownBeaconIntervalStart = dsme.getPlatform().getSymbolCounter();
        lastHeardBeaconTimestamp = dsmePANDescriptor.getTimeSyncSpec().getBeaconTimestampMicroSeconds();
        dsme.beaconSentOrReceived(dsmePANDescriptor.getBeaconBitmap().getSDIndex());
    }

    LOG_DEBUG("Send beacon");

    if (!dsme.getAckLayer().sendButKeep(msg, doneCallback)) {
        // message could not be sent
        dsme.getPlatform().releaseMessage(msg);
    }

    return;
}

void BeaconManager::sendEnhancedBeaconRequest() {
    DSMEMessage* msg = dsme.getPlatform().getEmptyMessage();

    MACCommand cmd;
    cmd.setCmdId(CommandFrameIdentifier::BEACON_REQUEST);
    cmd.prependTo(msg);

    //TODO Header IEEE802.15.4-2012 5.3.7.2.1 p.97
    msg->getHeader().setDstAddrMode(AddrMode::SHORT_ADDRESS);
    msg->getHeader().setDstAddr(IEEE802154MacAddress::SHORT_BROADCAST_ADDRESS);

    msg->getHeader().setSrcAddrMode(AddrMode::EXTENDED_ADDRESS);
    msg->getHeader().setSrcAddr(dsme.getMAC_PIB().macExtendedAddress);

    msg->getHeader().setFrameType(IEEE802154eMACHeader::COMMAND);

    msg->getHeader().getFrameControl().frameVersion = IEEE802154eMACHeader::FrameVersion::IEEE802154_2012; //0b10 in specification
    msg->getHeader().getFrameControl().framePending = 0;
    msg->getHeader().setAckRequest(false);

    msg->getHeader().getFrameControl().securityEnabled = 0;

    if(!dsme.getMessageDispatcher().sendInCAP(msg)) {
        // TODO
        dsme.getPlatform().releaseMessage(msg);
    }
}

void BeaconManager::handleEnhancedBeacon(DSMEMessage* msg, DSMEPANDescriptor& descr) {
    if (dsme.getDSMESettings().isPANCoordinator) {
        /* '-> This function should not be called for PAN-coordinators */
        DSME_ASSERT(false);
    }

    if(this->scanning) {
        /* '-> Do not handle beacon as PAN coordinator or while scanning */
        LOG_INFO("Currently scanning for PANs -> discard.");
        return;
    }

    // TODO should this be done for every beacon or only for parent beacons?
    // TODO do this on lower layer to gain accuracy and include offset in calculation
    uint16_t lastHeardBeaconSDIndex = descr.getBeaconBitmap().getSDIndex();

    // -8 symbols for preamble
    // -2 symbols for SFD
    lastKnownBeaconIntervalStart = msg->getStartOfFrameDelimiterSymbolCounter()
            - lastHeardBeaconSDIndex * aNumSuperframeSlots * dsme.getMAC_PIB().helper.getSymbolsPerSlot() - 8 - 2;
    lastHeardBeaconTimestamp = descr.getTimeSyncSpec().getBeaconTimestampMicroSeconds();
    //EV_DETAIL << "Received EnhancedBeacon @ " << lastHeardBeaconSDIndex << "(" << lastHeardBeaconTimestamp << ")" << endl;

    // time sync -> reschedule nextSlotTimer

    dsme.beaconSentOrReceived(lastHeardBeaconSDIndex);

    LOG_DEBUG("Updating heard Beacons, index is " << descr.getBeaconBitmap().getSDIndex() << ".");
    // update heardBeacons and neighborHeardBeacons
    heardBeacons.set(descr.getBeaconBitmap().getSDIndex(), true);
    neighborOrOwnHeardBeacons.set(descr.getBeaconBitmap().getSDIndex(), true);
    neighborOrOwnHeardBeacons.orWith(descr.getBeaconBitmap());
    if (dsme.getDSMESettings().isCoordinator) {
        dsmePANDescriptor.getBeaconBitmap().set(descr.getBeaconBitmap().getSDIndex(), true);
    }

    //EV_DEBUG << "heardBeacons: " << heardBeacons.getAllocatedCount() << ": " << heardBeacons.SDBitmap.toString() << ", ";
    //EV << "neighborsBeacons: " << neighborHeardBeacons.getAllocatedCount() << ": " << neighborHeardBeacons.SDBitmap.toString() << endl;

    // Coordinator device request free beacon slots
    LOG_DEBUG("Checking if beacon has to be allocated: "
            << "isCoordinator:" << dsme.getDSMESettings().isCoordinator
            << ", isBeaconAllocated:" << isBeaconAllocated
            << ", isBeaconAllocationSent:" << isBeaconAllocationSent
            << ".");
    if (dsme.getDSMESettings().isCoordinator && !isBeaconAllocated && !isBeaconAllocationSent) {
        LOG_INFO("Attempting to reserve slot for own BEACON.");
        if(!(dsme.getMAC_PIB().macAssociatedPANCoord)) {
            LOG_INFO("Device is not associated, cannot reserve BEACON slot.");
        } else {
            // Lookup free slot within all neighbors and broadcast beacon allocation request
            int32_t i = neighborOrOwnHeardBeacons.getRandomFreeSlot(dsme.getPlatform().getRandom());
            if (i >= 0) {
                sendBeaconAllocationNotification(i);
            } else {
                LOG_INFO("No window for a BEACON could be found.");
            }
        }
    }

    // TODO if isAllocationsent and allocated Index now is allocated -> cancel!

}

void BeaconManager::sendBeaconAllocationNotification(uint16_t beaconSDIndex) {
    LOG_INFO("Attempting to allocate BEACON at index " << beaconSDIndex << ".");
    DSMEMessage* msg = dsme.getPlatform().getEmptyMessage();

    BeaconNotificationCmd cmd;
    cmd.setBeaconSDIndex(beaconSDIndex);
    cmd.prependTo(msg);

    MACCommand macCmd;
    macCmd.setCmdId(CommandFrameIdentifier::DSME_BEACON_ALLOCATION_NOTIFICATION);
    macCmd.prependTo(msg);

    msg->getHeader().setFrameType(IEEE802154eMACHeader::COMMAND);
    msg->getHeader().setSrcAddr(dsme.getMAC_PIB().macExtendedAddress);
    msg->getHeader().setDstAddr(IEEE802154MacAddress::SHORT_BROADCAST_ADDRESS);
    msg->getHeader().setDstAddrMode(SHORT_ADDRESS);
    msg->getHeader().setAckRequest(false);

    // Update PANDDescription
    dsmePANDescriptor.getBeaconBitmap().setSDIndex(beaconSDIndex);
    dsmePANDescriptor.getBeaconBitmap().copyBitsFrom(heardBeacons);

    isBeaconAllocationSent = true;

    if(!dsme.getMessageDispatcher().sendInCAP(msg)) {
        isBeaconAllocationSent = false;
        dsme.getPlatform().releaseMessage(msg);
    }
}

void BeaconManager::handleBeaconAllocation(DSMEMessage* msg) {
    // TODO check if self has sent allocation to same slot -> abort CSMA transmission
    BeaconNotificationCmd beaconAlloc;
    beaconAlloc.decapsulateFrom(msg);

    if (heardBeacons.get(beaconAlloc.getBeaconSDIndex())) {
        sendBeaconCollisionNotification(beaconAlloc.getBeaconSDIndex(), msg->getHeader().getSrcAddr());
    } else {
        heardBeacons.set(beaconAlloc.getBeaconSDIndex(), true);
        // TODO when to remove heardBeacons in case of collision elsewhere?
    }
}

void BeaconManager::handleBeaconRequest(DSMEMessage* msg) {
    //TODO
    if(!this->dsme.getDSMESettings().isCoordinator && this->dsme.getMAC_PIB().macAssociatedPANCoord) {
        LOG_INFO("Received BEACON-REQUEST, turning into a coordinator now.");
        this->dsme.getDSMESettings().isCoordinator = true;
        this->dsme.getPlatform().updateVisual();
    }
}

void BeaconManager::sendBeaconCollisionNotification(uint16_t beaconSDIndex, const IEEE802154MacAddress& addr) {
    LOG_INFO("Informing about BEACON-collision at index " << beaconSDIndex << ".");
    numBeaconCollision++;

    DSMEMessage* msg = dsme.getPlatform().getEmptyMessage();

    BeaconNotificationCmd cmd;
    cmd.setBeaconSDIndex(beaconSDIndex);
    cmd.prependTo(msg);

    MACCommand macCmd;
    macCmd.setCmdId(CommandFrameIdentifier::DSME_BEACON_COLLISION_NOTIFICATION);
    macCmd.prependTo(msg);

    msg->getHeader().setFrameType(IEEE802154eMACHeader::FrameType::COMMAND);
    msg->getHeader().setDstAddr(addr);
    msg->getHeader().setSrcAddr(dsme.getMAC_PIB().macExtendedAddress);
    msg->getHeader().setAckRequest(true);

    if(!dsme.getMessageDispatcher().sendInCAP(msg)) {
        // TODO
        dsme.getPlatform().releaseMessage(msg);
    }
}

void BeaconManager::handleBeaconCollision(DSMEMessage* msg) {
    BeaconNotificationCmd cmd;
    cmd.decapsulateFrom(msg);

    LOG_INFO("Got informed about BEACON-collision at index " << cmd.getBeaconSDIndex() << ".");
    isBeaconAllocated = false;
    neighborOrOwnHeardBeacons.set(cmd.getBeaconSDIndex(), true);
}

void BeaconManager::onCSMASent(DSMEMessage* msg, CommandFrameIdentifier cmdId, DataStatus::Data_Status status, uint8_t numBackoffs) {
    if(cmdId == DSME_BEACON_ALLOCATION_NOTIFICATION && status == DataStatus::SUCCESS) {
        if (dsme.getDSMESettings().isCoordinator && isBeaconAllocationSent) {
            isBeaconAllocated = true;
            isBeaconAllocationSent = false;
        }
    }

    dsme.getPlatform().releaseMessage(msg);
}

uint16_t BeaconManager::getNumHeardBeacons() {
    return heardBeacons.getAllocatedCount();
}

void BeaconManager::sendDone(enum AckLayerResponse result, DSMEMessage* msg) {
    dsme.getPlatform().releaseMessage(msg);
}

void BeaconManager::handleBeacon(DSMEMessage* msg) {
    if (dsme.getDSMESettings().isPANCoordinator) {
        //* '-> do not handle beacon as PAN coordinator */
        LOG_INFO("A PAN-coordinator does not handle BEACONS -> discard");
        return;
    }

    if ((!msg->hasPayload()) && (this->dsme.getMAC_PIB().macAutoRequest)) {
        /* '-> Nothing to do for this beacon */
        LOG_INFO("This BEACON does not have to be indicated -> discard");
        return;
    }

    /* Data exist or no macAutoRequest -> create indication */

    mlme_sap::BEACON_NOTIFY_indication_parameters params;

    params.bsn = msg->getHeader().getSequenceNumber();
    params.panDescriptor.coordAddrMode = msg->getHeader().getSrcAddrMode();
    params.panDescriptor.coordPANId = msg->getHeader().getSrcPANId();
    params.panDescriptor.coordAddress = msg->getHeader().getSrcAddr();
    params.panDescriptor.channelNumber = this->dsme.getPHY_PIB().phyCurrentChannel;
    params.panDescriptor.channelPage = this->dsme.getPHY_PIB().phyCurrentPage;
    params.panDescriptor.timestamp = msg->getReceptionSymbolCounter();

    //  TODO fill in the other indication_parameters,
    //  some of the info is already included in the PANDesriptor.
    //    params.pendAddrSpec;
    //    params.addrList;
    //    params.sduLength;
    //    params.sdu;
    params.eBSN = msg->getHeader().getSequenceNumber();
    params.beaconType = msg->getHeader().isEnhancedBeacon();

    if (msg->getHeader().isEnhancedBeacon()) {
        params.panDescriptor.dsmePANDescriptor.decapsulateFrom(msg);
        this->dsme.getMLME_SAP().getBEACON_NOTIFY().notify_indication(params);
        handleEnhancedBeacon(msg, params.panDescriptor.dsmePANDescriptor);
    } else {
        DSME_ASSERT(false); // not implemented!
        this->dsme.getMLME_SAP().getBEACON_NOTIFY().notify_indication(params);
    }

    if (this->scanning) {
        switch (this->scanType) {
            case PASSIVE:
                singleBeaconScanPassiveReceived(params.panDescriptor);
                break;
            case ENHANCEDACTIVESCAN:
                singleBeaconScanEnhancedActiveReceived(params.panDescriptor);
                break;
            default:
                break;
        }
    }
}

bool BeaconManager::isScanning() {
    return this->scanning;
}

void BeaconManager::setScanDuration(uint16_t scanDuration) {
    /*
     * Calculate time to scan each channel
     *  T_scan = aBaseSuperframeDuration * (2^n + 1) (IEEE 802.15.4e-2012 6.2.10.1)
     *         = 16 * aBaseSlotDuration * (2^n + 1)
     *
     *  T_superframe = 16 * aBaseSlotDuration * 2^(SO)
     *
     *  S = T_scan / T_superframe = (2^n + 1) / (2^(SO))
     *  2^(n-SO) < S < 2^(n-SO)+1
     */
    uint16_t superframes = 1;

    // Scanning less than one superframe does not make any sense
    if(scanDuration > this->dsme.getMAC_PIB().macSuperframeOrder) {
        superframes = (1 << (scanDuration-this->dsme.getMAC_PIB().macSuperframeOrder)) + 1;
    }

    this->superframesForEachChannel = superframes;
}

void BeaconManager::startScanEnhancedActive(uint16_t scanDuration, channelList_t scanChannels) {
    DSME_ASSERT(!this->scanning);
    DSME_ASSERT(scanChannels.size() > 0);

    this->scanType = ScanType::ENHANCEDACTIVESCAN;
    this->scanning = true;

    this->scanChannels = scanChannels;
    this->storedMacPANId = this->dsme.getMAC_PIB().macPANId;
    this->dsme.getMAC_PIB().macPANId = 0xffff;

    setScanDuration(scanDuration);

    this->panDescriptorList.clear();

    /*
     * Channels are scanned in order from the lowest channel number to the highest. (IEEE802.15.4-2011 5.1.2.1)
     */
    this->currentScanChannelIndex = 0;

    this->dsme.getPlatform().setChannelNumber(this->scanChannels[this->currentScanChannelIndex]);
    this->sendEnhancedBeaconRequest();
    this->superframesLeftForScan = this->superframesForEachChannel;
    return;
}

void BeaconManager::startScanPassive(uint16_t scanDuration, channelList_t scanChannels) {
    DSME_ASSERT(!this->scanning);
    DSME_ASSERT(scanChannels.size() > 0);

    this->scanType = ScanType::PASSIVE;
    this->scanning = true;

    this->scanChannels = scanChannels;
    this->storedMacPANId = this->dsme.getMAC_PIB().macPANId;
    this->dsme.getMAC_PIB().macPANId = 0xffff;

    setScanDuration(scanDuration);

    this->panDescriptorList.clear();

    /*
     * Channels are scanned in order from the lowest channel number to the highest. (IEEE802.15.4-2011 5.1.2.1)
     */
    this->currentScanChannelIndex = 0;

    this->dsme.getPlatform().setChannelNumber(this->scanChannels[this->currentScanChannelIndex]);
    this->superframesLeftForScan = this->superframesForEachChannel;
}

void BeaconManager::handleStartOfCFP() {
    if (this->scanning) {
        this->superframesLeftForScan--;

        if (this->superframesLeftForScan == 0) {
            switch (this->scanType) {
            case PASSIVE:
                channelScanPassiveComplete();
                break;
            case ENHANCEDACTIVESCAN:
                channelScanEnhancedActiveComplete();
                break;
            default:
                LOG_WARN("Event during unknown scan!");
                break;
            }
        }
    }

    return;
}

void BeaconManager::scanCurrentChannel() {
    this->dsme.getPlatform().setChannelNumber(this->currentScanChannel);

    sendEnhancedBeaconRequest();

    this->superframesLeftForScan = this->superframesForEachChannel;
    return;
}

void BeaconManager::channelScanPassiveComplete() {
    DSME_ASSERT(this->scanning);

    LOG_INFO("Channel scan completed for channel " << (uint16_t) this->scanChannels[this->currentScanChannelIndex] << ".");
    if (this->panDescriptorList.full() || this->currentScanChannelIndex >= this->scanChannels.size() - 1) {
        this->scanning = false;
        this->dsme.getMAC_PIB().macPANId = this->storedMacPANId;

        mlme_sap::SCAN_confirm_parameters params;

        if (this->dsme.getMAC_PIB().macAutoRequest) {
            params.panDescriptorList = panDescriptorList;
            params.resultListSize = panDescriptorList.size();
        } else {
            params.resultListSize = 0;
        }

        params.status = ScanStatus::SUCCESS;
        params.scanType = this->scanType;

        LOG_INFO("Notify confirm");
        this->dsme.getMLME_SAP().getSCAN().notify_confirm(params);
    } else {
        LOG_INFO("Check next");
        this->currentScanChannelIndex++;
        this->dsme.getPlatform().setChannelNumber(this->scanChannels[this->currentScanChannelIndex]);
        this->superframesLeftForScan = this->superframesForEachChannel;
    }
    return;
}

void BeaconManager::singleBeaconScanPassiveReceived(PANDescriptor &panDescr) {
    if (this->dsme.getMAC_PIB().macAutoRequest) {
        LOG_INFO("Beacon registered during passive scan.");
        this->panDescriptorList.add(panDescr);
        if (this->panDescriptorList.full()) {
            channelScanPassiveComplete();
        }
    }
    return;
}

void BeaconManager::channelScanEnhancedActiveComplete() {
    DSME_ASSERT(this->scanning);

    LOG_INFO("Channel scan completed for channel " << (uint16_t) this->scanChannels[this->currentScanChannelIndex] << ".");
    if (this->panDescriptorList.full() || this->currentScanChannelIndex >= this->scanChannels.size() - 1) {
        this->scanning = false;
        this->dsme.getMAC_PIB().macPANId = this->storedMacPANId;

        mlme_sap::SCAN_confirm_parameters params;

        if (this->dsme.getMAC_PIB().macAutoRequest) {
            params.panDescriptorList = panDescriptorList;
            params.resultListSize = panDescriptorList.size();
        } else {
            params.resultListSize = 0;
        }

        params.status = ScanStatus::SUCCESS;
        params.scanType = this->scanType;

        this->dsme.getMLME_SAP().getSCAN().notify_confirm(params);
    } else {
        this->currentScanChannelIndex++;
        this->dsme.getPlatform().setChannelNumber(this->scanChannels[this->currentScanChannelIndex]);
        this->sendEnhancedBeaconRequest();
        this->superframesLeftForScan = this->superframesForEachChannel;
    }
    return;
}

void BeaconManager::singleBeaconScanEnhancedActiveReceived(PANDescriptor &panDescr) {
    if (this->dsme.getMAC_PIB().macAutoRequest) {
        LOG_INFO("Beacon registered during enhanced active scan.");
        this->panDescriptorList.add(panDescr);
        if (this->panDescriptorList.full()) {
            channelScanEnhancedActiveComplete();
        }
    }
}

}
