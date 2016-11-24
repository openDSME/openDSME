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

#ifndef DSMEADAPTIONLAYER_H_
#define DSMEADAPTIONLAYER_H_

#include "AssociationHelper.h"
#include "GTSHelper.h"
#include "ScanHelper.h"

#include "../helper/DSMERingbuffer.h"

namespace dsme {

class DSMELayer;

namespace mlme_sap {
class MLME_SAP;
struct SYNC_LOSS_indication_parameters;
}

namespace mcps_sap {
class MCPS_SAP;
struct DATA_indication_parameters;
struct DATA_confirm_parameters;
}

class MAC_PIB;
class PHY_PIB;

struct DSMEAdaptionLayerSettings {
public:
    enum gts_allocation_scheme {
        ALLOC_RANDOM, ALLOC_CONTIGUOUS_SLOT
    };

    DSMEAdaptionLayerSettings() :
            allocationScheme(ALLOC_CONTIGUOUS_SLOT) {

    }

    gts_allocation_scheme allocationScheme;
};

struct DSMEAdaptionLayerBufferEntry {
public:
    DSMEMessage* message;
    uint32_t initialSymbolCounter;
};

/**
 * This class provides a simplified abstraction of the DSME mac-layer. It takes and returns data messages and performs all
 * management tasks automatically.
 */
class DSMEAdaptionLayer {
public:
    typedef Delegate<void(DSMEMessage* msg)> indicationCallback_t;
    typedef Delegate<void(DSMEMessage* msg, DataStatus::Data_Status dataStatus)> confirmCallback_t;

    DSMEAdaptionLayer(DSMELayer&);

    void initialize();

    mcps_sap::MCPS_SAP& getMCPS_SAP();

    mlme_sap::MLME_SAP& getMLME_SAP();

    PHY_PIB& getPHY_PIB();

    MAC_PIB& getMAC_PIB();

    DSMELayer& getDSME();

    void sendMessage(DSMEMessage* msg);

    void setIndicationCallback(indicationCallback_t);

    void setConfirmCallback(confirmCallback_t);

    void sendRetryBuffer();

    void startAssociation();

public:
    DSMEAdaptionLayerSettings settings;

private:
    void sendMessageDown(DSMEMessage* msg, bool newMessage);

    void receiveIndication(DSMEMessage* msg);

    bool queueMessageIfPossible(DSMEMessage* msg);

    void handleDataIndication(mcps_sap::DATA_indication_parameters &params);

    void handleDataConfirm(mcps_sap::DATA_confirm_parameters &params);

    void handleSyncLossIndication(mlme_sap::SYNC_LOSS_indication_parameters &params);

    void handleScanComplete(PANDescriptor* panDescriptor);

    void handleAssociationComplete(AssociationStatus::Association_Status status);

    DSMELayer& dsme;

    AssociationHelper associationHelper;
    GTSHelper gtsAllocationHelper;
    ScanHelper scanHelper;

    mcps_sap::MCPS_SAP* mcps_sap;
    mlme_sap::MLME_SAP* mlme_sap;
    PHY_PIB* phy_pib;
    MAC_PIB* mac_pib;

    indicationCallback_t callback_indication;
    confirmCallback_t callback_confirm;

    bool scanInProgress;
    bool associationInProgress;

    DSMERingBuffer<DSMEAdaptionLayerBufferEntry, UPPER_LAYER_QUEUE_SIZE> retryBuffer;
};

} /* dsme */

#endif /* DSMEADAPTIONLAYER_H_ */
