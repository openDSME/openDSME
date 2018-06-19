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

#define DSME_ADAPTION_LAYER

#include "./DSMEAdaptionLayer.h"

#include "../dsmeLayer/DSMELayer.h"
#include "../dsmeLayer/messages/IEEE802154eMACHeader.h"
#include "../interfaces/IDSMEMessage.h"
#include "../interfaces/IDSMEPlatform.h"
#include "../mac_services/mcps_sap/DATA.h"
#include "../mac_services/mcps_sap/MCPS_SAP.h"
#include "../mac_services/mlme_sap/MLME_SAP.h"
#include "../mac_services/mlme_sap/RESET.h"
#include "../mac_services/pib/MAC_PIB.h"

namespace dsme {

DSMEAdaptionLayer::DSMEAdaptionLayer(DSMELayer& dsme)
    : dsme(dsme),
      associationHelper(*this),
      messageHelper(*this),
      gtsHelper(*this),
      scanHelper(*this),

      mcps_sap(nullptr),
      mlme_sap(nullptr),
      phy_pib(nullptr),
      mac_pib(nullptr) {
}

void DSMEAdaptionLayer::initialize(channelList_t& scanChannels, uint8_t scanDuration, GTSScheduling* scheduling) {
    this->mcps_sap = &(this->dsme.getMCPS_SAP());
    this->mlme_sap = &(this->dsme.getMLME_SAP());
    this->phy_pib = &(this->dsme.getPHY_PIB());
    this->mac_pib = &(this->dsme.getMAC_PIB());

    this->associationHelper.initialize();
    this->messageHelper.initialize();
    this->gtsHelper.initialize(scheduling);
    this->scanHelper.initialize(scanChannels, scanDuration);

    this->scanHelper.setScanAndSyncCompleteDelegate(DELEGATE(&DSMEAdaptionLayer::handleScanAndSyncComplete, *this));
    this->scanHelper.setSyncLossAfterSyncedDelegate(DELEGATE(&DSMEAdaptionLayer::handleSyncLossAfterSynced, *this));
    this->associationHelper.setAssociationCompleteDelegate(DELEGATE(&DSMEAdaptionLayer::handleAssociationComplete, *this));
    this->associationHelper.setDisassociationCompleteDelegate(DELEGATE(&DSMEAdaptionLayer::handleDisassociationComplete, *this));
    this->getMLME_SAP().getRESET().confirm(DELEGATE(&DSMEAdaptionLayer::handleResetComplete, *this));

    return;
}

DSMELayer& DSMEAdaptionLayer::getDSME() {
    return this->dsme;
}

mcps_sap::MCPS_SAP& DSMEAdaptionLayer::getMCPS_SAP() {
    return *(this->mcps_sap);
}

mlme_sap::MLME_SAP& DSMEAdaptionLayer::getMLME_SAP() {
    return *(this->mlme_sap);
}

PHY_PIB& DSMEAdaptionLayer::getPHY_PIB() {
    return *(this->phy_pib);
}

MAC_PIB& DSMEAdaptionLayer::getMAC_PIB() {
    return *(this->mac_pib);
}

AssociationHelper& DSMEAdaptionLayer::getAssociationHelper() {
    return this->associationHelper;
}

GTSHelper& DSMEAdaptionLayer::getGTSHelper() {
    return this->gtsHelper;
}

MessageHelper& DSMEAdaptionLayer::getMessageHelper() {
    return this->messageHelper;
}

ScanHelper& DSMEAdaptionLayer::getScanHelper() {
    return this->scanHelper;
}

void DSMEAdaptionLayer::setIndicationCallback(indicationCallback_t callback_indication) {
    this->messageHelper.setIndicationCallback(callback_indication);
    return;
}

void DSMEAdaptionLayer::setConfirmCallback(confirmCallback_t callback_confirm) {
    this->messageHelper.setConfirmCallback(callback_confirm);
    return;
}

void DSMEAdaptionLayer::sendMessage(IDSMEMessage* msg) {
    this->messageHelper.sendMessage(msg);
}

void DSMEAdaptionLayer::startAssociation() {
    if(!getMAC_PIB().macAssociatedPANCoord) {
        LOG_DEBUG("Device is not associated with PAN.");
        this->messageHelper.startAssociation();
    }
}

uint16_t DSMEAdaptionLayer::getRandom() {
    return getDSME().getPlatform().getRandom();
}

void DSMEAdaptionLayer::handleSyncLossAfterSynced() {
    if(this->mac_pib->macAssociatedPANCoord) {
        this->associationHelper.disassociate();
    } else {
        reset();
    }
}

void DSMEAdaptionLayer::handleScanAndSyncComplete(PANDescriptor* panDescriptor) {
    this->messageHelper.handleScanAndSyncComplete(panDescriptor);

    if(panDescriptor != nullptr) {
        LOG_INFO("PAN found on channel " << (uint16_t)panDescriptor->channelNumber << ", coordinator is " << panDescriptor->coordAddress.getShortAddress()
                                         << " on PAN " << panDescriptor->coordPANId << ".");
    } else {
        LOG_DEBUG("Channel scan did not yield any PANs.");
    }
    return;
}

void DSMEAdaptionLayer::handleAssociationComplete(AssociationStatus::Association_Status status) {
    if(status == AssociationStatus::SUCCESS) {
        LOG_INFO("Association completed successfully.");
    } else {
        LOG_ERROR("Association failed.");
    }

    this->messageHelper.handleAssociationComplete(status);
    return;
}

void DSMEAdaptionLayer::reset() {
    mlme_sap::RESET::request_parameters params;
    params.setDefaultPib = false;

    getMLME_SAP().getRESET().request(params);
    return;
}

void DSMEAdaptionLayer::handleResetComplete(mlme_sap::RESET_confirm_parameters& parameters) {
    bool confirmed = getMLME_SAP().getRESET().confirm(&parameters);
    DSME_ASSERT(parameters.status == ResetStatus::SUCCESS);

    this->gtsHelper.reset();
}

void DSMEAdaptionLayer::handleDisassociationComplete(DisassociationStatus::Disassociation_Status status) {
    if(status == DisassociationStatus::SUCCESS) {
        LOG_DEBUG("Disassociation completed successfully.");
    } else {
        LOG_ERROR("Disassociation failed.");
    }

    /*
     * A disassociation is currently only started after loosing beacon tracking.
     * Afterwards, reset the entire MLME.
     */
    reset();
}

} /* namespace dsme */
