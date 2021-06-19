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

#include "./DSME_GTS.h"

#include "../../dsmeLayer/DSMELayer.h"
#include "../../dsmeLayer/gtsManager/GTSManager.h"
#include "../../dsmeLayer/messages/GTSManagement.h"
#include "../../dsmeLayer/messages/GTSReplyNotifyCmd.h"
#include "../../dsmeLayer/messages/GTSRequestCmd.h"
#include "../DSME_Common.h"
#include "../dataStructures/DSMESuperframeSpecification.h"
#include "../pib/MAC_PIB.h"

namespace dsme {
namespace mlme_sap {

template <typename T>
static void notify_busy_confirm(T& params, DSME_GTS& dsme_gts) {
    DSME_GTS_confirm_parameters busyConfirm;

    busyConfirm.deviceAddress = params.deviceAddress;
    busyConfirm.direction = params.direction;
    busyConfirm.dsmeSabSpecification = params.dsmeSabSpecification;
    busyConfirm.managementType = params.managementType;
    busyConfirm.prioritizedChannelAccess = params.prioritizedChannelAccess;
    busyConfirm.status = GTSStatus::TRANSACTION_OVERFLOW;
    dsme_gts.notify_confirm(busyConfirm);
    return;
}

DSME_GTS::DSME_GTS(DSMELayer& dsme) : dsme(dsme) {
}
/*
 * Request command(IEEE 802.15.4e-2012 5.3.11.4) will be generated with the request_parameters from higher layer.
 */
void DSME_GTS::request(request_parameters& params) {
    uint16_t srcAddr = this->dsme.getMAC_PIB().macShortAddress;
    if((srcAddr == 0xfffe) || (srcAddr == 0xffff)) {
        DSME_GTS_confirm_parameters reply;
        reply.status = GTSStatus::NO_SHORT_ADDRESS;
        this->notify_confirm(reply);
    } else {
        GTSManager& gtsManager = dsme.getGTSManager();

        GTSManagement gtsManagement(params.managementType, params.direction, params.prioritizedChannelAccess);
        GTSRequestCmd gtsRequestCmd(params.numSlot, params.preferredSuperframeId, params.preferredSlotId, params.preferredChannelId, params.dsmeSabSpecification);
        bool busy = !gtsManager.handleMLMERequest(params.deviceAddress, gtsManagement, gtsRequestCmd);

        if(busy) {
            notify_busy_confirm(params, *this);
        }
    }
}

void DSME_GTS::response(response_parameters& params) {
    /* (IEEE 802.15.4e-2012 6.2.21.1.3):
     - generate reply command (IEEE 802.15.4e-2012 5.3.11.5)
     */
    GTSManager& gtsManager = dsme.getGTSManager();
    GTSManagement gtsManagement(params.managementType, params.direction, params.prioritizedChannelAccess, params.status);

    bool busy;
    if(dsme.getMAC_PIB().macChannelDiversityMode == Channel_Diversity_Mode::CHANNEL_ADAPTATION) {
        // channel adaption mode
        GTSReplyNotifyCmd gtsReply(params.deviceAddress, params.dsmeSabSpecification);
        busy = !gtsManager.handleMLMEResponse(gtsManagement, gtsReply);
    } else {
        // channel hopping mode
        GTSReplyNotifyCmd gtsReply(params.deviceAddress, params.channelOffset, params.dsmeSabSpecification);
        busy = !gtsManager.handleMLMEResponse(gtsManagement, gtsReply);
    }
    if(busy) {
        notify_busy_confirm(params, *this);
    }
}

} /* namespace mlme_sap */
} /* namespace dsme */
