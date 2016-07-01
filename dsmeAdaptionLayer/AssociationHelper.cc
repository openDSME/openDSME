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

#include "../../dsme_platform.h"
#include "../mac_services/mlme_sap/MLME_SAP.h"
#include "AssociationHelper.h"
#include "DSMEAdaptionLayer.h"

namespace dsme {

AssociationHelper::AssociationHelper(DSMEAdaptionLayer& dsmeAdaptionLayer) :
        dsmeAdaptionLayer(dsmeAdaptionLayer) {
}

void AssociationHelper::initialize() {
    this->dsmeAdaptionLayer.getMLME_SAP().getASSOCIATE().indication(DELEGATE(&AssociationHelper::handleASSOCIATION_indication, *this));
    this->dsmeAdaptionLayer.getMLME_SAP().getASSOCIATE().confirm(DELEGATE(&AssociationHelper::handleASSOCIATION_confirm, *this));
    return;
}

void AssociationHelper::setAssociationCompleteDelegate(associationCompleteDelegate_t delegate) {
    this->associationCompleteDelegate = delegate;
    return;
}

void AssociationHelper::associate(uint16_t coordPANId, AddrMode addrMode, IEEE802154MacAddress coordAddress, uint8_t channel) {
    CapabilityInformation capabilityInformation;
    capabilityInformation.alternatePANCoordinator = false;
    capabilityInformation.deviceType = 1;
    capabilityInformation.powerSource = 0;
    capabilityInformation.receiverOnWhenIdle = 1;
    capabilityInformation.associationType = 1; // TODO: FastA? 1 -> yes, 0 -> no
    capabilityInformation.reserved = 0;
    capabilityInformation.securityCapability = 0;
    capabilityInformation.allocateAddress = 1;

    mlme_sap::ASSOCIATE::request_parameters params;
    params.channelNumber = channel;
    params.channelPage = this->dsmeAdaptionLayer.getPHY_PIB().phyCurrentPage;
    params.coordAddrMode = SHORT_ADDRESS;
    params.coordPANId = coordPANId;
    params.coordAddress = coordAddress;
    params.capabilityInformation = capabilityInformation;
    params.securityLevel = 0;
    params.keyIdMode = 0;
    params.keySource = nullptr;
    params.keyIndex = 0;
    params.lowLatencyNetworkInfo = nullptr;
    params.channelOffset = 0;
    params.hoppingSequenceID = 0;

    // TODO start timer for macMaxFrameTotalWaitTime, report NO_DATA on timeout
    this->dsmeAdaptionLayer.getMLME_SAP().getASSOCIATE().request(params);
    return;
}

bool AssociationHelper::isAssociatedDevice(IEEE802154MacAddress address) {
    // TODO check if device is associated to this coordinator.
    return true;
}

void AssociationHelper::handleASSOCIATION_indication(mlme_sap::ASSOCIATE_indication_parameters &params) {
    LOG_INFO("Association requested from 0x" << params.deviceAddress.getShortAddress() << ".");

    mlme_sap::ASSOCIATE::response_parameters response_params;

    response_params.deviceAddress = params.deviceAddress;
    response_params.assocShortAddress = params.deviceAddress.a4();
    response_params.status = AssociationStatus::SUCCESS;
    response_params.securityLevel = 0;
    response_params.keyIdMode = 0;
    response_params.keySource = nullptr;
    response_params.keyIndex = 0;
    response_params.lowLatencyNetworkInfo = nullptr;
    response_params.channelOffset = 0;
    response_params.hoppingSequenceLength = 0;
    response_params.hoppingSequence = nullptr;

    // TODO update list of associated devices

    this->dsmeAdaptionLayer.getMLME_SAP().getASSOCIATE().response(response_params);
    return;
}

void AssociationHelper::handleASSOCIATION_confirm(mlme_sap::ASSOCIATE_confirm_parameters &params) {
    if (params.status == AssociationStatus::SUCCESS) {

    }
    this->associationCompleteDelegate(params.status);
    return;
}

} /* dsme */
