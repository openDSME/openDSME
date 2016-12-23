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

#include "DATA.h"

#include "../../dsmeLayer/DSMELayer.h"
#include "../../dsmeLayer/messages/IEEE802154eMACHeader.h"

namespace dsme {
namespace mcps_sap {

DATA::DATA(DSMELayer& dsme) : dsme(dsme) {
}

/*
 * IEEE802.15.4-2011 6.3.1
 * On receipt of the MCPS-DATA.request primitive, the MAC sublayer entity begins the transmission of the supplied MSDU.
 */
void DATA::request(request_parameters& params) {
    DSMEMessage* dsmemsg = params.msdu;
    IDSMEMessage* msg = dsmemsg;

    if(!(dsme.getMAC_PIB().macAssociatedPANCoord)) {
        mcps_sap::DATA_confirm_parameters confirmParams;
        confirmParams.msduHandle = dsmemsg;
        confirmParams.Timestamp = 0;
        confirmParams.RangingReceived = false;
        confirmParams.status = DataStatus::INVALID_GTS; // TODO better status?
        confirmParams.AckPayload = nullptr;
        confirmParams.gtsTX = params.gtsTX;
        notify_confirm(confirmParams);
        return;
    }

    dsmemsg->receivedViaMCPS = true;

    IEEE802154eMACHeader& header = msg->getHeader();

    header.setFrameType(IEEE802154eMACHeader::DATA);

    header.setSrcPANId(this->dsme.getMAC_PIB().macPANId);
    header.setSrcAddr(this->dsme.getMAC_PIB().macExtendedAddress);

    header.setAckRequest(params.ackTX);

    header.setSecurityEnabled(params.securityLevel);

    header.overridePanIDCompression(params.frameControlOption_pan_id_suppressed);
    header.setIEListPresent(params.frameControlOption_ies_included);
    header.setSeqNumSuppression(params.frameControlOption_seq_num_suppressed);

    if(params.gtsTX) {
        // TODO use short address!
        IEEE802154MacAddress& dest = msg->getHeader().getDestAddr();
        NeighborQueue<MAX_NEIGHBORS>::iterator destIt = dsme.getMessageDispatcher().getNeighborQueue().findByAddress(dest);

        DSMEAllocationCounterTable& macDSMEACT = dsme.getMAC_PIB().macDSMEACT;
        uint16_t numAllocatedSlots = macDSMEACT.getNumAllocatedTxGTS(dest.getShortAddress());

        if(destIt == dsme.getMessageDispatcher().getNeighborQueue().end() || numAllocatedSlots == 0) {
            mcps_sap::DATA_confirm_parameters confirmParams;
            confirmParams.msduHandle = dsmemsg;
            confirmParams.Timestamp = 0;
            confirmParams.RangingReceived = false;
            confirmParams.status = DataStatus::INVALID_GTS;
            confirmParams.AckPayload = nullptr;
            confirmParams.gtsTX = params.gtsTX;
            notify_confirm(confirmParams);
            return;
        }

        if(!this->dsme.getMessageDispatcher().sendInGTS(dsmemsg, destIt)) {
            mcps_sap::DATA_confirm_parameters confirmParams;
            confirmParams.msduHandle = dsmemsg;
            confirmParams.Timestamp = 0;
            confirmParams.RangingReceived = false;
            confirmParams.status = DataStatus::TRANSACTION_OVERFLOW;
            confirmParams.AckPayload = nullptr;
            confirmParams.gtsTX = params.gtsTX;
            notify_confirm(confirmParams);
        }
    } else {
        if(!this->dsme.getMessageDispatcher().sendInCAP(dsmemsg)) {
            mcps_sap::DATA_confirm_parameters confirmParams;
            confirmParams.msduHandle = dsmemsg;
            confirmParams.Timestamp = 0;
            confirmParams.RangingReceived = false;
            confirmParams.status = DataStatus::TRANSACTION_OVERFLOW;
            confirmParams.AckPayload = nullptr;
            confirmParams.gtsTX = params.gtsTX;
            notify_confirm(confirmParams);
        }
    }
    return;
}

} /* mcps_sap */
} /* dsme */
