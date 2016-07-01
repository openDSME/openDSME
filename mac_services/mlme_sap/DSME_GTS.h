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

#ifndef DSME_GTS_H_
#define DSME_GTS_H_

#include <stdint.h>

#include "../../dsmeLayer/messages/GTSManagement.h"
#include "../../dsmeLayer/messages/GTSReplyNotifyCmd.h"
#include "../../dsmeLayer/messages/GTSRequestCmd.h"
#include "../ConfirmBase.h"
#include "../DSME_Common.h"
#include "../IndicationBase.h"
#include "../dataStructures/DSMESABSpecification.h"
#include "../dataStructures/IEEE802154MacAddress.h"

namespace dsme {
class DSMELayer;

namespace mlme_sap {

struct DSME_GTS_indication_parameters {
    uint16_t deviceAddress;
    ManagementType managmentType;
    Direction direction;
    Priority prioritizedChannelAccess;
    uint8_t numSlot;
    uint16_t preferredSuperframeID;
    uint8_t preferredSlotID;
    DSMESABSpecification dsmeSABSpecification;
};

struct DSME_GTS_confirm_parameters {
    uint16_t deviceAddress;
    ManagementType managmentType;
    Direction direction;
    Priority prioritizedChannelAccess;
    uint16_t channelOffset;
    DSMESABSpecification dsmeSABSpecification;
    GTSStatus::GTS_Status status;
};

/*
 * The MLME-SAP DSME-GTS management primitives define how DSME-GTSs are requested and maintained.
 * A device wishing to use these primitives and DSME-GTSs in general will already be tracking the
 * beacons of its coordinator. (IEEE 802.15.4e-2012 6.2.21.1)
 */
class DSME_GTS : public IndicationBase<DSME_GTS_indication_parameters>,
                 public ConfirmBase<DSME_GTS_confirm_parameters> {
public:
    DSME_GTS(DSMELayer& dsme);

    struct request_parameters {
        uint16_t deviceAddress;
        ManagementType managmentType;
        Direction direction;
        Priority prioritizedChannelAccess;
        uint8_t numSlot;
        uint16_t preferredSuperframeID;
        uint8_t preferredSlotID;
        DSMESABSpecification dsmeSABSpecification;
        uint8_t securityLevel;
        uint8_t keyIdMode;
        uint8_t* keySource;
        uint8_t keyIndex;
    };

    struct response_parameters {
        uint16_t deviceAddress;
        ManagementType managmentType;
        Direction direction;
        Priority prioritizedChannelAccess;
        uint16_t channelOffset;
        DSMESABSpecification dsmeSABSpecification;
        GTSStatus::GTS_Status status;
    };

    void request(request_parameters&);

    void response(response_parameters&);

private:
    DSMELayer& dsme;
};

} /* mlme_sap */
} /* dsme */

#endif /* DSME_GTS_H_ */
