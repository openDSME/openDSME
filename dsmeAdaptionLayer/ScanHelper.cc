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

#include "./ScanHelper.h"

#include "../../dsme_platform.h"
#include "../dsmeLayer/DSMELayer.h" // TODO: remove cross-layer reference
#include "../interfaces/IDSMEPlatform.h"
#include "../mac_services/DSME_Common.h"
#include "../mac_services/dataStructures/IEEE802154MacAddress.h"
#include "../mac_services/mlme_sap/BEACON_NOTIFY.h"
#include "../mac_services/mlme_sap/MLME_SAP.h"
#include "../mac_services/mlme_sap/SCAN.h"
#include "../mac_services/mlme_sap/START.h"
#include "../mac_services/mlme_sap/SYNC.h"
#include "../mac_services/mlme_sap/SYNC_LOSS.h"
#include "../mac_services/pib/MAC_PIB.h"
#include "./DSMEAdaptionLayer.h"

namespace dsme {

ScanHelper::ScanHelper(DSMEAdaptionLayer& dsmeAdaptionLayer)
    : dsmeAdaptionLayer(dsmeAdaptionLayer), recordedPanDescriptors(0), passiveScanCounter(0), syncActive(false) {
}

void ScanHelper::initialize(channelList_t& scanChannels) {
    this->scanChannels = scanChannels;
    this->dsmeAdaptionLayer.getMLME_SAP().getBEACON_NOTIFY().indication(DELEGATE(&ScanHelper::handleBEACON_NOTIFY_indication, *this));
    this->dsmeAdaptionLayer.getMLME_SAP().getSCAN().confirm(DELEGATE(&ScanHelper::handleSCAN_confirm, *this));
    this->dsmeAdaptionLayer.getMLME_SAP().getSYNC_LOSS().indication(DELEGATE(&ScanHelper::handleSyncLossIndication, *this));
    return;
}

void ScanHelper::setScanAndSyncCompleteDelegate(scanAndSyncCompleteDelegate_t delegate) {
    this->scanAndSyncCompleteDelegate = delegate;
    return;
}

void ScanHelper::setSyncLossAfterSyncedDelegate(syncLossAfterSyncedDelegate_t delegate) {
    this->syncLossAfterSyncedDelegate = delegate;
    return;
}

void ScanHelper::startScan() {
    DSME_ASSERT(!this->syncActive);

    this->recordedPanDescriptors.clear();

    mlme_sap::SCAN::request_parameters params;

    uint16_t random_value = this->dsmeAdaptionLayer.getDSME().getPlatform().getRandom() % 128;
    if(((uint16_t) this->passiveScanCounter) > random_value) {
        LOG_INFO("Initiating enhanced active scan");
        params.scanType = ScanType::ENHANCEDACTIVESCAN;
    } else {
        LOG_INFO("Initiating passive scan");
        params.scanType = ScanType::PASSIVE;
    }
    this->passiveScanCounter++;

    params.scanChannels = scanChannels;
    params.scanDuration = 6;
    params.channelPage = this->dsmeAdaptionLayer.getPHY_PIB().phyCurrentPage;
    params.linkQualityScan = false;

    this->dsmeAdaptionLayer.getMLME_SAP().getSCAN().request(params);
    return;
}

void ScanHelper::handleSyncLossIndication(mlme_sap::SYNC_LOSS_indication_parameters& params) {
    if(params.lossReason == LossReason::BEACON_LOST) {
        LOG_ERROR("Beacon tracking lost.");
        // DSME_SIM_ASSERT(false); TODO
    } else {
        LOG_ERROR("Tracking lost for unsupported reason: " << (uint16_t)params.lossReason);
        DSME_ASSERT(false);
    }

    if(this->syncActive) {
        this->syncActive = false;
        this->scanAndSyncCompleteDelegate(nullptr);
    } else {
        this->syncLossAfterSyncedDelegate();
    }
    return;
}

void ScanHelper::handleBEACON_NOTIFY_indication(mlme_sap::BEACON_NOTIFY_indication_parameters& params) {
    if(!this->dsmeAdaptionLayer.getMAC_PIB().macAutoRequest) {
        LOG_INFO("Beacon registered in upper layer.");
        this->recordedPanDescriptors.add(params.panDescriptor);
    }

    uint16_t shortAddress = params.panDescriptor.coordAddress.getShortAddress();
    if(!heardCoordinators.contains(shortAddress)) {
        heardCoordinators.add(shortAddress);
    }

    if(this->syncActive) {
        // TODO check if the beacon is actually from the PAN described in the activePanDescriptor
        this->syncActive = false;
        this->scanAndSyncCompleteDelegate(&this->panDescriptorToSyncTo);
    }

    // TODO CROSS-LAYER-CALLS, no interface for this information
    LOG_INFO("Checking whether to become a coordinator: "
             << "isAssociated:" << this->dsmeAdaptionLayer.getMAC_PIB().macAssociatedPANCoord << ", isCoordinator:"
             << this->dsmeAdaptionLayer.getMAC_PIB().macIsCoord << ", numHeardCoordinators:" << ((uint16_t)heardCoordinators.getLength()) << ".");
    if(this->dsmeAdaptionLayer.getMAC_PIB().macAssociatedPANCoord && !this->dsmeAdaptionLayer.getMAC_PIB().macIsCoord && heardCoordinators.getLength() < 2) {
        uint16_t random_value = this->dsmeAdaptionLayer.getDSME().getPlatform().getRandom() % 3;
        if(random_value < 1) {
            mlme_sap::START::request_parameters request_params;
            request_params.panCoordinator = false;
            // TODO: fill rest;

            LOG_INFO("Turning into a coordinator now.");
            this->dsmeAdaptionLayer.getMLME_SAP().getSTART().request(request_params);

            mlme_sap::START_confirm_parameters confirm_params;
            bool confirmed = this->dsmeAdaptionLayer.getMLME_SAP().getSTART().confirm(&confirm_params);
            DSME_ASSERT(confirmed);
            DSME_ASSERT(confirm_params.status == StartStatus::SUCCESS);
        }
    }

    return;
}

void ScanHelper::handleSCAN_confirm(mlme_sap::SCAN_confirm_parameters& params) {
    DSME_ASSERT(!this->syncActive);

    PanDescriptorList* list;

    if(!this->dsmeAdaptionLayer.getMAC_PIB().macAutoRequest) {
        list = &this->recordedPanDescriptors;
    } else {
        list = &params.panDescriptorList;
    }

    if(list->size() == 0) {
        this->scanAndSyncCompleteDelegate(nullptr);
        return;
    }

    uint8_t bestLQI = 0;
    uint8_t bestLQIIdx = 0xFF;

    for(uint8_t i = 0; i < list->size(); i++) {
        if((*list)[i].linkQuality > bestLQI) {
            bestLQI = (*list)[i].linkQuality;
            bestLQIIdx = i;
        }
    }

    if(bestLQI < this->dsmeAdaptionLayer.getDSME().getPlatform().getMinCoordinatorLQI()) {
        this->scanAndSyncCompleteDelegate(nullptr);
        return;
    }

    this->panDescriptorToSyncTo = (*list)[bestLQIIdx];

    mlme_sap::SYNC::request_parameters syncParams;
    syncParams.channelNumber = this->panDescriptorToSyncTo.channelNumber;
    syncParams.channelPage = this->panDescriptorToSyncTo.channelPage;
    syncParams.trackBeacon = true;
    this->syncActive = true;
    this->dsmeAdaptionLayer.getMLME_SAP().getSYNC().request(syncParams);

    return;
}

} /* namespace dsme */
