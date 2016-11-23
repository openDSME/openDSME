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

#ifndef DSMELAYER_H
#define DSMELAYER_H

#include <stdint.h>

#include "../../dsme_platform.h"
#include "../helper/DSMEFSM.h"
#include "../interfaces/IDSMEMessage.h"
#include "../interfaces/IDSMEPlatform.h"
#include "../mac_services/dataStructures/BeaconBitmap.h"
#include "../mac_services/dataStructures/DSMEPANDescriptor.h"
#include "../mac_services/dataStructures/DSMESuperframeSpecification.h"
#include "../mac_services/dataStructures/IEEE802154MacAddress.h"
#include "../mac_services/dataStructures/SuperframeSpecification.h"
#include "../mac_services/pib/MAC_PIB.h"
#include "../mac_services/pib/PHY_PIB.h"
#include "../mac_services/pib/dsme_mac_constants.h"
#include "DSMEEventDispatcher.h"
#include "ackLayer/AckLayer.h"
#include "associationManager/AssociationManager.h"
#include "beaconManager/BeaconManager.h"
#include "capLayer/CAPLayer.h"
#include "gtsManager/GTSManager.h"
#include "messageDispatcher/MessageDispatcher.h"


namespace dsme {

/*
 * Forward declarations for MAC services:
 */
namespace mlme_sap {
class MLME_SAP;
}

namespace mcps_sap {
class MCPS_SAP;
}

class DSMEPlatform;

class DSMESettings {
public:
    bool isPANCoordinator;
    bool isCoordinator;
    unsigned commonChannel;

    bool optimizations;
};

/**
 * IEEE802.15.4
 * Deterministic and synchronous multi-channel extension
 */
class DSMELayer {
    friend class DSMEPlatform;

public:
    DSMELayer();
    ~DSMELayer();

    /* MAC SERVICES -------------------------------------------------------> */
private:
    PHY_PIB *phy_pib;
    MAC_PIB *mac_pib;

    mcps_sap::MCPS_SAP *mcps_sap;
    mlme_sap::MLME_SAP *mlme_sap;

public:
    PHY_PIB& getPHY_PIB() {
        return *(phy_pib);
    }

    MAC_PIB& getMAC_PIB() {
        return *(mac_pib);
    }

    mcps_sap::MCPS_SAP& getMCPS_SAP() {
        return *(this->mcps_sap);
    }

    mlme_sap::MLME_SAP& getMLME_SAP() {
        return *(this->mlme_sap);
    }

    void setPHY_PIB(PHY_PIB* phy_pib) {
        this->phy_pib = phy_pib;
        return;
    }

    void setMAC_PIB(MAC_PIB* mac_pib) {
        this->mac_pib = mac_pib;
        return;
    }

    void setMCPS(mcps_sap::MCPS_SAP *mcps_sap) {
        this->mcps_sap = mcps_sap;
        return;
    }

    void setMLME(mlme_sap::MLME_SAP *mlme_sap) {
        this->mlme_sap = mlme_sap;
        return;
    }
    /* <------------------------------------------------------- MAC SERVICES */

    GTSManager& getGTSManager() {
        return gtsManager;
    }

    AssociationManager& getAssociationManager() {
        return this->associationManager;
    }

    BeaconManager& getBeaconManager() {
        return beaconManager;
    }

    void setStartOfCFPDelegate(Delegate<void()> delegate) {
        startOfCFPDelegate = delegate;
    }

    const DSMESettings& getDSMESettings() const {
        return settings;
    }

    DSMESettings& getDSMESettings() {
        return settings;
    }

    DSMEEventDispatcher& getEventDispatcher() {
        return eventDispatcher;
    }

    AckLayer& getAckLayer() {
        return ackLayer;
    }

    CAPLayer& getCapLayer() {
        return capLayer;
    }

    IDSMEPlatform& getPlatform() {
        return *platform;
    }

    MessageDispatcher& getMessageDispatcher() {
        return messageDispatcher;
    }

    void start(DSMESettings& dsmeSettings, IDSMEPlatform* platform);

    void dispatchCCAResult(bool success) {
        messageDispatcher.dispatchCCAResult(success);
    }

    void preSlotEvent(void);
    void slotEvent(int32_t lateness);

    uint16_t getSymbolsSinceSuperframeStart(uint32_t time, uint16_t shift);

    bool isWithinCAP(uint32_t time, uint16_t duration);

    unsigned getCurrentSuperframe() const {
        return currentSuperframe;
    }

    unsigned getCurrentSlot() const {
        return currentSlot;
    }

    // TODO data size
    void beaconSentOrReceived(uint16_t SDIndex);

    void handleStartOfCFP();

    void startTrackingBeacons();
    void stopTrackingBeacons();

protected:
    DSMESettings settings;

    CAPLayer capLayer;

    DSMEEventDispatcher eventDispatcher;

    AckLayer ackLayer;

    MessageDispatcher messageDispatcher;

    IDSMEPlatform* platform;

    GTSManager gtsManager;
    BeaconManager beaconManager;
    AssociationManager associationManager;

    // TODO remove one or use better abstraction
    Delegate<void()> startOfCFPDelegate;
    //Delegate<void(uint8_t slot, uint8_t superframe)> slotEventDelegateB;
    // TODO size!
    uint16_t currentSlot;
    uint16_t currentSuperframe;
    uint16_t currentMultiSuperframe;
    uint16_t nextSlot;
    uint16_t nextSuperframe;
    uint16_t nextMultiSuperframe;

    uint16_t slotsSinceLastKnownBeaconIntervalStart;

    bool trackingBeacons;
    uint32_t lastSlotTime;

    /**
     * Called every slot to display node status in GUI
     * TODO currently platform specific!
     */
    void updateDisplay();
};

}

#endif
