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

BeaconManager::BeaconManager(DSMELayer& dsme)
    : dsme(dsme),

      isBeaconAllocationSent(false),
      isBeaconAllocated(false),
      lastKnownBeaconIntervalStart(0),

      numBeaconCollision(0),
      missedBeacons(0),
      doneCallback(DELEGATE(&BeaconManager::sendDone, *this)),

      currentScanChannel(0),
      scanning(false),
      scanType(ScanType::ED),
      storedMacPANId(0),
      currentScanChannelIndex(0),
      superframesForEachChannel(0),
      superframesLeftForScan(0),
      transmissionPending(false) {
}

void BeaconManager::initialize() {
    dsmePANDescriptor.getBeaconBitmap().setSDBitmapLengthBytes(BITVECTOR_BYTE_LENGTH(dsme.getMAC_PIB().helper.getNumberSuperframesPerBeaconInterval()), false);
    this->dsme.getMAC_PIB().macSdBitmap.setSDBitmapLengthBytes(BITVECTOR_BYTE_LENGTH(dsme.getMAC_PIB().helper.getNumberSuperframesPerBeaconInterval()), false);
    neighborOrOwnHeardBeacons.setSDBitmapLengthBytes(BITVECTOR_BYTE_LENGTH(dsme.getMAC_PIB().helper.getNumberSuperframesPerBeaconInterval()), false);

    isBeaconAllocated = false;
    isBeaconAllocationSent = false;

    // PAN Coordinator starts network with beacon
    if(dsme.getMAC_PIB().macIsPANCoord) {
        dsmePANDescriptor.getBeaconBitmap().setSDIndex(0);
        dsmePANDescriptor.getBeaconBitmap().fill(false);
        dsmePANDescriptor.getBeaconBitmap().set(0, true);
    } else if(dsme.getMAC_PIB().macIsCoord) {
        dsmePANDescriptor.getBeaconBitmap().fill(false);
    }

    lastKnownBeaconIntervalStart = dsme.getPlatform().getSymbolCounter();
}

void BeaconManager::reset() {
    isBeaconAllocated = false;
    isBeaconAllocationSent = false;
    missedBeacons = 0;

    if(dsme.getMAC_PIB().macIsPANCoord) {
        dsmePANDescriptor.getBeaconBitmap().setSDIndex(0);
        dsmePANDescriptor.getBeaconBitmap().fill(false);
        dsmePANDescriptor.getBeaconBitmap().set(0, true);
    } else if(dsme.getMAC_PIB().macIsCoord) {
        dsmePANDescriptor.getBeaconBitmap().fill(false);
    }

    lastKnownBeaconIntervalStart = dsme.getPlatform().getSymbolCounter();

    this->dsme.getMAC_PIB().macSdBitmap.fill(false);
    this->neighborOrOwnHeardBeacons.fill(false);
}

void BeaconManager::preSuperframeEvent(uint16_t nextSuperframe, uint16_t nextMultiSuperframe, uint32_t startSlotTime) {
    if(isBeaconAllocated || dsme.getMAC_PIB().macIsPANCoord) {
        uint16_t nextSDIndex = nextSuperframe + dsme.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe() * nextMultiSuperframe;
        if(nextSDIndex == dsmePANDescriptor.getBeaconBitmap().getSDIndex()) {
            prepareEnhancedBeacon(startSlotTime);
        }
    }
}

void BeaconManager::superframeEvent(int32_t lateness, uint32_t currentSlotTime) {
    if(transmissionPending) {
        if(lateness > 1) {
            dsme.getAckLayer().abortPreparedTransmission();
        } else {
            dsme.getAckLayer().sendNowIfPending();
        }

        if(dsme.getMAC_PIB().macIsPANCoord) {
            lastKnownBeaconIntervalStart = currentSlotTime;
        }
    }
}

void BeaconManager::prepareEnhancedBeacon(uint32_t nextSlotTime) {
    DSME_ASSERT(!transmissionPending);
    IDSMEMessage* msg = dsme.getPlatform().getEmptyMessage();

    dsmePANDescriptor.getTimeSyncSpec().setBeaconTimestampMicroSeconds(nextSlotTime * aSymbolDuration);
    dsmePANDescriptor.getTimeSyncSpec().setBeaconOffsetTimestampMicroSeconds(0);
    dsmePANDescriptor.prependTo(msg); // TODO this should be implemented as IE

    msg->getHeader().setDstAddr(IEEE802154MacAddress(IEEE802154MacAddress::SHORT_BROADCAST_ADDRESS));
    msg->getHeader().setDstAddrMode(SHORT_ADDRESS);

    msg->getHeader().setSrcAddr(dsme.getMAC_PIB().macExtendedAddress);
    msg->getHeader().setFrameType(IEEE802154eMACHeader::BEACON);

    msg->getHeader().setSrcPANId(this->dsme.getMAC_PIB().macPANId);
    msg->getHeader().setDstPANId(this->dsme.getMAC_PIB().macPANId);

    transmissionPending = true;
    if(!dsme.getAckLayer().prepareSendingCopy(msg, doneCallback)) {
        // message could not be sent
        dsme.getPlatform().releaseMessage(msg);
    } else {
        LOG_DEBUG("Beacon sent");
    }

    return;
}

void BeaconManager::sendEnhancedBeaconRequest() {
    IDSMEMessage* msg = dsme.getPlatform().getEmptyMessage();

    MACCommand cmd;
    cmd.setCmdId(CommandFrameIdentifier::BEACON_REQUEST);
    cmd.prependTo(msg);

    // TODO Header IEEE802.15.4-2012 5.3.7.2.1 p.97
    msg->getHeader().setDstAddrMode(AddrMode::SHORT_ADDRESS);
    msg->getHeader().setDstAddr(IEEE802154MacAddress(IEEE802154MacAddress::SHORT_BROADCAST_ADDRESS));

    msg->getHeader().setSrcAddrMode(AddrMode::EXTENDED_ADDRESS);
    msg->getHeader().setSrcAddr(dsme.getMAC_PIB().macExtendedAddress);

    msg->getHeader().setFrameType(IEEE802154eMACHeader::COMMAND);

    msg->getHeader().setAckRequest(false);

    if(!dsme.getMessageDispatcher().sendInCAP(msg)) {
        // TODO
        dsme.getPlatform().releaseMessage(msg);
    }
}

bool BeaconManager::handleEnhancedBeacon(IDSMEMessage* msg, DSMEPANDescriptor& descr) {
    if(dsme.getMAC_PIB().macIsPANCoord) {
        /* '-> This function should not be called for PAN-coordinators */
        DSME_ASSERT(false);
    }

    if(this->scanning) {
        /* '-> Do not handle beacons while scanning */
        LOG_INFO("Currently scanning for PANs -> don't handle as enhanced beacon.");
        return false;
    }

    LOG_DEBUG("Updating heard Beacons, index is " << descr.getBeaconBitmap().getSDIndex() << ".");
    this->dsme.getMAC_PIB().macSdBitmap.set(descr.getBeaconBitmap().getSDIndex(), true);
    neighborOrOwnHeardBeacons.set(descr.getBeaconBitmap().getSDIndex(), true);
    neighborOrOwnHeardBeacons.orWith(descr.getBeaconBitmap());

    if(dsme.getMAC_PIB().macAssociatedPANCoord && msg->getHeader().getSrcAddr().getShortAddress() != this->dsme.getMAC_PIB().macCoordShortAddress) {
        LOG_INFO("Only track beacons by coordinator -> discard");
        return true;
    }

    /* Reset the number of missed beacons */
    this->missedBeacons = 0;

    // TODO should this be done for every beacon or only for parent beacons?
    // TODO do this on lower layer to gain accuracy and include offset in calculation
    uint16_t lastHeardBeaconSDIndex = descr.getBeaconBitmap().getSDIndex();

    // -8 symbols for preamble
    // -2 symbols for SFD
    uint32_t offset = descr.getTimeSyncSpec().getBeaconOffsetTimestampMicroSeconds() / aSymbolDuration;
    lastKnownBeaconIntervalStart = msg->getStartOfFrameDelimiterSymbolCounter() -
                                   lastHeardBeaconSDIndex * aNumSuperframeSlots * dsme.getMAC_PIB().helper.getSymbolsPerSlot() - 8 - 2 - offset;

    // Coordinator device request free beacon slots
    LOG_DEBUG("Checking if beacon has to be allocated: "
              << "isCoordinator:" << dsme.getMAC_PIB().macIsCoord << ", isBeaconAllocated:" << isBeaconAllocated
              << ", isBeaconAllocationSent:" << isBeaconAllocationSent << ".");
    if(dsme.getMAC_PIB().macIsCoord && !isBeaconAllocated && !isBeaconAllocationSent && !dsme.getMAC_PIB().macIsPANCoord) {
        LOG_INFO("Attempting to reserve slot for own BEACON.");
        if(!(dsme.getMAC_PIB().macAssociatedPANCoord)) {
            LOG_INFO("Device is not associated, cannot reserve BEACON slot.");
        } else {
            // Lookup free slot within all neighbors and broadcast beacon allocation request
            int32_t i = neighborOrOwnHeardBeacons.getRandomFreeSlot(dsme.getPlatform().getRandom());
            if(i >= 0) {
                sendBeaconAllocationNotification(i);
            } else {
                LOG_INFO("No window for a BEACON could be found.");
            }
        }
    }

    return false;
}

void BeaconManager::sendBeaconAllocationNotification(uint16_t beaconSDIndex) {
    LOG_INFO("Attempting to allocate BEACON at index " << beaconSDIndex << ".");
    IDSMEMessage* msg = dsme.getPlatform().getEmptyMessage();

    BeaconNotificationCmd cmd;
    cmd.setBeaconSDIndex(beaconSDIndex);
    cmd.prependTo(msg);

    MACCommand macCmd;
    macCmd.setCmdId(CommandFrameIdentifier::DSME_BEACON_ALLOCATION_NOTIFICATION);
    macCmd.prependTo(msg);

    msg->getHeader().setFrameType(IEEE802154eMACHeader::COMMAND);
    msg->getHeader().setSrcAddr(dsme.getMAC_PIB().macExtendedAddress);
    msg->getHeader().setDstAddr(IEEE802154MacAddress(IEEE802154MacAddress::SHORT_BROADCAST_ADDRESS));
    msg->getHeader().setDstAddrMode(SHORT_ADDRESS);

    msg->getHeader().setSrcPANId(this->dsme.getMAC_PIB().macPANId);
    msg->getHeader().setDstPANId(this->dsme.getMAC_PIB().macPANId);

    msg->getHeader().setAckRequest(false);

    // Update PANDDescription
    dsmePANDescriptor.getBeaconBitmap().setSDIndex(beaconSDIndex);
    dsmePANDescriptor.getBeaconBitmap().copyBitsFrom(this->dsme.getMAC_PIB().macSdBitmap);

    isBeaconAllocationSent = true;

    if(!dsme.getMessageDispatcher().sendInCAP(msg)) {
        isBeaconAllocationSent = false;
        dsme.getPlatform().releaseMessage(msg);
    }
}

void BeaconManager::handleBeaconAllocation(IDSMEMessage* msg) {
    // TODO check if self has sent allocation to same slot -> abort CSMA transmission
    BeaconNotificationCmd beaconAlloc;
    beaconAlloc.decapsulateFrom(msg);

    uint16_t ownBeaconSDIndex = this->dsmePANDescriptor.getBeaconBitmap().getSDIndex();
    uint16_t heardBeaconSDIndex = beaconAlloc.getBeaconSDIndex();

    bool collidesWithOwnBeacon = this->isBeaconAllocated && (ownBeaconSDIndex == heardBeaconSDIndex);
    bool collidesWithHeardBeacon = this->dsme.getMAC_PIB().macSdBitmap.get(heardBeaconSDIndex);

    if(collidesWithOwnBeacon || collidesWithHeardBeacon) {
        sendBeaconCollisionNotification(heardBeaconSDIndex, msg->getHeader().getSrcAddr());
    } else {
        this->dsme.getMAC_PIB().macSdBitmap.set(heardBeaconSDIndex, true);
        this->neighborOrOwnHeardBeacons.set(heardBeaconSDIndex, true);
        // TODO when to remove heardBeacons in case of collision elsewhere?
    }
}

void BeaconManager::handleBeaconRequest(IDSMEMessage* msg) {
    // TODO
    if(!this->dsme.getMAC_PIB().macIsCoord && this->dsme.getMAC_PIB().macAssociatedPANCoord) {
        LOG_INFO("Received BEACON-REQUEST, turning into a coordinator now.");
        this->dsme.getMAC_PIB().macIsCoord = true;
        this->dsme.getPlatform().updateVisual();
    }
}

void BeaconManager::sendBeaconCollisionNotification(uint16_t beaconSDIndex, const IEEE802154MacAddress& addr) {
    LOG_INFO("Informing about BEACON-collision at index " << beaconSDIndex << ".");
    numBeaconCollision++;

    IDSMEMessage* msg = dsme.getPlatform().getEmptyMessage();

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

    msg->getHeader().setSrcPANId(this->dsme.getMAC_PIB().macPANId);
    msg->getHeader().setDstPANId(this->dsme.getMAC_PIB().macPANId);

    if(!dsme.getMessageDispatcher().sendInCAP(msg)) {
        // TODO
        dsme.getPlatform().releaseMessage(msg);
    }
}

void BeaconManager::handleBeaconCollision(IDSMEMessage* msg) {
    BeaconNotificationCmd cmd;
    cmd.decapsulateFrom(msg);

    LOG_INFO("Got informed about BEACON-collision at index " << cmd.getBeaconSDIndex() << ".");
    isBeaconAllocated = false;
    neighborOrOwnHeardBeacons.set(cmd.getBeaconSDIndex(), true);
}

void BeaconManager::onCSMASent(IDSMEMessage* msg, CommandFrameIdentifier cmdId, DataStatus::Data_Status status, uint8_t numBackoffs) {
    if(cmdId == DSME_BEACON_ALLOCATION_NOTIFICATION && status == DataStatus::SUCCESS) {
        if(dsme.getMAC_PIB().macIsCoord && isBeaconAllocationSent) {
            isBeaconAllocated = true;
            isBeaconAllocationSent = false;
        }
    }

    dsme.getPlatform().releaseMessage(msg);
}

void BeaconManager::sendDone(enum AckLayerResponse result, IDSMEMessage* msg) {
    dsme.getPlatform().releaseMessage(msg);
    transmissionPending = false;
}

void BeaconManager::handleBeacon(IDSMEMessage* msg) {
    if(dsme.getMAC_PIB().macIsPANCoord) {
        //* '-> do not handle beacon as PAN coordinator */
        LOG_INFO("A PAN-coordinator does not handle BEACONS -> discard");
        return;
    }

    if((!msg->hasPayload()) && (this->dsme.getMAC_PIB().macAutoRequest)) {
        /* '-> Nothing to do for this beacon */
        LOG_INFO("This BEACON does not have to be indicated -> discard");
        return;
    }

    if(!msg->getHeader().isEnhancedBeacon()) {
        LOG_INFO("We only want enhanced beacons -> discard");
        return;
    }

    /* Data exist or no macAutoRequest -> create indication */

    mlme_sap::BEACON_NOTIFY_indication_parameters params;
    params.panDescriptor.dsmePANDescriptor.decapsulateFrom(msg);

    bool beaconDiscarded = handleEnhancedBeacon(msg, params.panDescriptor.dsmePANDescriptor);

    if(!beaconDiscarded) {
        params.bsn = msg->getHeader().getSequenceNumber();
        params.panDescriptor.coordAddrMode = msg->getHeader().getSrcAddrMode();
        params.panDescriptor.coordPANId = msg->getHeader().getDstPANId();
        params.panDescriptor.coordAddress = msg->getHeader().getSrcAddr();
        params.panDescriptor.channelNumber = this->dsme.getPHY_PIB().phyCurrentChannel;
        params.panDescriptor.channelPage = this->dsme.getPHY_PIB().phyCurrentPage;
        params.panDescriptor.timestamp = msg->getReceptionSymbolCounter();
        params.panDescriptor.linkQuality = msg->getLQI();
        //  TODO fill in the other indication_parameters,
        //  some of the info is already included in the PANDesriptor.
        //    params.pendAddrSpec;
        //    params.addrList;
        //    params.sduLength;
        //    params.sdu;
        params.eBSN = msg->getHeader().getSequenceNumber();
        params.beaconType = msg->getHeader().isEnhancedBeacon();

        this->dsme.getMLME_SAP().getBEACON_NOTIFY().notify_indication(params);
    }

    if(this->scanning) {
        switch(this->scanType) {
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

bool BeaconManager::isScanning() const {
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
        superframes = (1 << (scanDuration - this->dsme.getMAC_PIB().macSuperframeOrder)) + 1;
    }

    this->superframesForEachChannel = superframes;
}

void BeaconManager::startScanEnhancedActive(uint16_t scanDuration, const channelList_t& scanChannels) {
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

void BeaconManager::startScanPassive(uint16_t scanDuration, const channelList_t& scanChannels) {
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

void BeaconManager::handleStartOfCFP(uint16_t currentSuperframe, uint16_t currentMultiSuperframe) {
    if(this->scanning) {
        this->superframesLeftForScan--;

        if(this->superframesLeftForScan == 0) {
            switch(this->scanType) {
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

    if(this->dsme.isTrackingBeacons() && currentMultiSuperframe == 0 && currentSuperframe == 0) {
        /* Increment the number of missed beacons. This gets reset whenever a beacon is received */
        ++(this->missedBeacons);
        if(this->missedBeacons > aMaxLostBeacons) {
            mlme_sap::SYNC_LOSS_indication_parameters params;
            MAC_PIB& mac_pip = this->dsme.getMAC_PIB();
            PHY_PIB& phy_pip = this->dsme.getPHY_PIB();

            params.lossReason = LossReason::BEACON_LOST;
            params.panId = mac_pip.macPANId;
            params.channelNumber = phy_pip.phyCurrentChannel;
            params.channelPage = phy_pip.phyCurrentPage;
            params.securityLevel = 0;
            params.keyIdMode = 0;
            params.keySource = nullptr;
            params.keyIndex = 0;

            this->dsme.stopTrackingBeacons();
            this->dsme.getMLME_SAP().getSYNC_LOSS().notify_indication(params);
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

    LOG_INFO("Scan complete, chan " << (uint16_t) this->scanChannels[this->currentScanChannelIndex]);
    if(this->panDescriptorList.full() || this->currentScanChannelIndex >= this->scanChannels.size() - 1) {
        this->scanning = false;
        this->dsme.getMAC_PIB().macPANId = this->storedMacPANId;

        mlme_sap::SCAN_confirm_parameters params;

        if(this->dsme.getMAC_PIB().macAutoRequest) {
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

void BeaconManager::singleBeaconScanPassiveReceived(PANDescriptor& panDescr) {
    if(this->dsme.getMAC_PIB().macAutoRequest) {
        LOG_INFO("Beacon registered during passive scan.");
        this->panDescriptorList.add(panDescr);
        if(this->panDescriptorList.full()) {
            channelScanPassiveComplete();
        }
    }
    return;
}

void BeaconManager::channelScanEnhancedActiveComplete() {
    DSME_ASSERT(this->scanning);

    LOG_INFO("Scan complete, chan " << (uint16_t) this->scanChannels[this->currentScanChannelIndex]);
    if(this->panDescriptorList.full() || this->currentScanChannelIndex >= this->scanChannels.size() - 1) {
        this->scanning = false;
        this->dsme.getMAC_PIB().macPANId = this->storedMacPANId;

        mlme_sap::SCAN_confirm_parameters params;

        if(this->dsme.getMAC_PIB().macAutoRequest) {
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

void BeaconManager::singleBeaconScanEnhancedActiveReceived(PANDescriptor& panDescr) {
    if(this->dsme.getMAC_PIB().macAutoRequest) {
        LOG_INFO("Beacon registered during enhanced active scan.");
        this->panDescriptorList.add(panDescr);
        if(this->panDescriptorList.full()) {
            channelScanEnhancedActiveComplete();
        }
    }
}
}
