/*
 * openDSME
 *
 * Implementation of the Deterministic & Synchronous Multi-channel Extension (DSME)
 * introduced in the IEEE 802.15.4e-2012 standard
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

#ifndef BEACON_MANAGER_H
#define BEACON_MANAGER_H

#include <stdint.h>

#include "../../mac_services/dataStructures/BeaconBitmap.h"
#include "../../mac_services/dataStructures/DSMEPANDescriptor.h"
#include "../../mac_services/dataStructures/IEEE802154MacAddress.h"
#include "../../mac_services/mlme_sap/MLME_SAP.h"
#include "../ackLayer/AckLayer.h"

namespace dsme {

class DSMELayer;

class BeaconManager {
    friend class mlme_sap::SCAN;

public:
    explicit BeaconManager(DSMELayer& dsme);

    void initialize();

    void reset();

    /**
     * Called on reception of an EnhancedBeacon
     */
    bool handleEnhancedBeacon(IDSMEMessage* msg, DSMEPANDescriptor& descr);

    uint32_t getLastKnownBeaconIntervalStart() const {
        return lastKnownBeaconIntervalStart;
    }

    void preSuperframeEvent(uint16_t nextSuperframe, uint16_t nextMultiSuperframe, uint32_t nextSlotTime);
    void superframeEvent(int32_t lateness, uint32_t currentSlotTime);

    void handleBeacon(IDSMEMessage* msg);

    bool isScanning() const;

    void startScanPassive(uint16_t scanDuration, const channelList_t& scanChannels);

    /**
     * Starts an enhanced active scan (See IEEE 802.15.4e-2012 6.2.10.1)
     */
    void startScanEnhancedActive(uint16_t scanDuration, const channelList_t& scanChannels);

    void handleStartOfCFP(uint16_t currentSuperframe, uint16_t currentMultiSuperframe);

    /**
     * Gets called when CSMA Message was sent down to the PHY
     */
    void onCSMASent(IDSMEMessage* msg, CommandFrameIdentifier cmdId, DataStatus::Data_Status status, uint8_t numBackoffs);

    /**
     * Handle reception of beacon collision notification.
     * Update beacon allocation
     */
    void handleBeaconCollision(IDSMEMessage*);

    /**
     * Called on reception of an BeaconAllocationNotification
     */
    void handleBeaconAllocation(IDSMEMessage*);

    /**
     * Called on reception of an BeaconRequest
     */
    void handleBeaconRequest(IDSMEMessage*);

protected:
    DSMELayer& dsme;

    bool isBeaconAllocationSent;
    bool isBeaconAllocated;

    uint32_t lastKnownBeaconIntervalStart;

    BeaconBitmap neighborOrOwnHeardBeacons;

    DSMEPANDescriptor dsmePANDescriptor;

    long numBeaconCollision;

    uint8_t missedBeacons;

    /**
     * Send an enhanced Beacon directly
     */
    void prepareEnhancedBeacon(uint32_t startSlotTime);

    /**
     * Send an enhanced Beacon request in scan primitive
     */
    void sendEnhancedBeaconRequest();

    /**
     * Send beacon allocation notification
     */
    void sendBeaconAllocationNotification(uint16_t beaconSDIndex);

    /**
     * Send beacon collision notification
     */
    void sendBeaconCollisionNotification(uint16_t beaconSDIndex, const IEEE802154MacAddress& addr);

    void sendDone(enum AckLayerResponse result, IDSMEMessage* msg);

    AckLayer::done_callback_t doneCallback;

private:
    /* HELPER METHODS FOR SCANNING */
    void scanCurrentChannel();
    void setScanDuration(uint16_t scanDuration);

    /* CALLBACKS FOR SCANNING */
    void channelScanEnhancedActiveComplete();
    void singleBeaconScanEnhancedActiveReceived(PANDescriptor& panDescr);

    void channelScanPassiveComplete();
    void singleBeaconScanPassiveReceived(PANDescriptor& panDescr);

    /* VARIABLES FOR SCANNING */
    uint16_t currentScanChannel;

    PanDescriptorList panDescriptorList;

    bool scanning;
    ScanType scanType;
    channelList_t scanChannels;

    uint16_t storedMacPANId;
    uint8_t currentScanChannelIndex;

    uint16_t superframesForEachChannel;
    uint16_t superframesLeftForScan;
    bool transmissionPending;
};

} /* dsme */

#endif
