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

#include "ASSOCIATE.h"

#include "../../dsmeLayer/DSMELayer.h"
#include "../../dsmeLayer/associationManager/AssociationManager.h"
#include "../../dsmeLayer/messages/AssociateReplyCmd.h"
#include "../../dsmeLayer/messages/AssociateRequestCmd.h"

namespace dsme {
namespace mlme_sap {

ASSOCIATE::ASSOCIATE(DSMELayer& dsme) :
    dsme(dsme) {
}

/* IEEE802.15.4-2011 6.2.2.1 */
void ASSOCIATE::request(request_parameters& params) {

    //update PHY and MAC PIB attributes
    dsme.getPlatform().setChannelNumber(params.channelNumber); // TODO Move -> AssociationManager
    dsme.getPHY_PIB().phyCurrentPage = params.channelPage;
    dsme.getMAC_PIB().macPANId = params.coordPANId;
    if (params.coordAddrMode == AddrMode::SHORT_ADDRESS) {
        dsme.getMAC_PIB().macCoordShortAddress = params.coordAddress.getShortAddress();
    } else {
        dsme.getMAC_PIB().macCoordExtendedAddress = params.coordAddress;
    }

    AssociateRequestCmd associateRequestCmd(params.capabilityInformation);
    AssociationManager& associationManager = dsme.getAssociationManager();
    associationManager.sendAssociationRequest(associateRequestCmd, params);

}

void ASSOCIATE::response(response_parameters& params) {

    AssociateReplyCmd reply(params.assocShortAddress, params.status);
    AssociationManager& associationManager = dsme.getAssociationManager();

    if (params.status != AssociationStatus::FASTA_SUCCESSFUL) {
        /* TODO handle! (IEEE 802.15.4-2011 5.1.3.1)
         * response is added to a list of pending transactions
         */
    }
    associationManager.sendAssociationReply(reply, params.deviceAddress);

}

} /* mlme_sap */
} /* dsme */
