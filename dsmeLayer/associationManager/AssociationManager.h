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

#ifndef ASSOCIATIONMANAGER_H_
#define ASSOCIATIONMANAGER_H_

#include "../../../dsme_platform.h"
#include "../../mac_services/mlme_sap/MLME_SAP.h"
#include "../messages/AssociateReplyCmd.h"
#include "../messages/AssociateRequestCmd.h"
#include "../messages/DisassociationNotifyCmd.h"

namespace dsme {

class DSMELayer;

/*
 * Manager for MLME Associate and MLME Disassociate
 */

class AssociationManager {
public:
    AssociationManager(DSMELayer& dsme);
    void sendAssociationRequest(AssociateRequestCmd &associateRequestCmd, mlme_sap::ASSOCIATE::request_parameters &params);
    void handleAssociationRequest(DSMEMessage *msg);
    void sendAssociationReply(AssociateReplyCmd &reply, IEEE802154MacAddress &deviceAddress);
    void handleAssociationReply(DSMEMessage *msg);

    void sendDisassociationRequest(DisassociationNotifyCmd &req, mlme_sap::DISASSOCIATE::request_parameters &params);
    void handleDisassociationRequest(DSMEMessage *msg);

    /**
     * Gets called when CSMA Message was sent down to the PHY
     */
    void onCSMASent(DSMEMessage* msg, CommandFrameIdentifier cmdId, DataStatus::Data_Status status, uint8_t numBackoffs);

    void handleSlotEvent(uint8_t slot, uint8_t superframe);

private:
    DSMELayer& dsme;
    static constexpr uint16_t BROADCAST_PAN_ID = 0xffff;

    bool associationPending = false;
    bool associationSent = false;
    uint8_t superframesSinceAssociationSent;
};

}

#endif /* ASSOCIATIONMANAGER_H_ */