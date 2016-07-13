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

#ifndef GTSALLOCATIONHELPER_H_
#define GTSALLOCATIONHELPER_H_

#include "../mac_services/dataStructures/DSMEAllocationCounterTable.h"
#include "../mac_services/dataStructures/DSMESABSpecification.h"
#include "../mac_services/dataStructures/GTS.h"
#include "../mac_services/dataStructures/IEEE802154MacAddress.h"

namespace dsme {

class DSMEAdaptionLayer;

namespace mlme_sap {
    class DSME_GTS_indication_parameters;
    class DSME_GTS_confirm_parameters;
    class COMM_STATUS_indication_parameters;
}

class GTSHelper {
public:
    GTSHelper(DSMEAdaptionLayer&);

    void initialize();

    void checkAndAllocateSingleGTS(uint16_t address);

    void handleSlotEvent();

private:
    /* MLME handlers */

    void handleDSME_GTS_indication(mlme_sap::DSME_GTS_indication_parameters&);

    void handleDSME_GTS_confirm(mlme_sap::DSME_GTS_confirm_parameters&);

    void handleCOMM_STATUS_indication(mlme_sap::COMM_STATUS_indication_parameters&);

    /* Helper methods */

    void checkAndDeallocateSingeleGTS(uint16_t address);

    GTS getContiguousFreeGTS();

    GTS getRandomFreeGTS();

    GTS getNextFreeGTS(uint16_t initialSuperframeID, uint8_t initialSlotID, const DSMESABSpecification *sabSpec = nullptr);

    GTSStatus::GTS_Status verifyDeallocation(DSMESABSpecification &requestSABSpec, uint16_t &deviceAddress, Direction& direction);

    void findFreeSlots(DSMESABSpecification &requestSABSpec, DSMESABSpecification& replySABSpec, uint8_t numSlots,
            uint16_t preferredSuperframe, uint8_t preferredSlot);

    void sendDeallocationRequest(uint16_t address, Direction direction, DSMESABSpecification& sabSpecification);

    /* Debug methods */

    void print(const char* name, const DSMESABSpecification &spec);

    void print(const char* name, const DSMEAllocationCounterTable &act);

private:
    DSMEAdaptionLayer& dsmeAdaptionLayer;

    bool gtsConfirmPending;
    uint8_t superframesSinceGtsRequestSent;
};

} /* dsme */

#endif /* GTSALLOCATIONHELPER_H_ */