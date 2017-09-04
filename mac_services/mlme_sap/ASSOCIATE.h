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

#ifndef ASSOCIATE_H_
#define ASSOCIATE_H_

#include "../ConfirmBase.h"
#include "../DSME_Common.h"
#include "../IndicationBase.h"
#include "../dataStructures/IEEE802154MacAddress.h"

namespace dsme {
class DSMELayer;

namespace mlme_sap {

struct ASSOCIATE_indication_parameters {
    IEEE802154MacAddress deviceAddress;
    CapabilityInformation capabilityInformation;
    NOT_IMPLEMENTED_t securityLevel;
    NOT_IMPLEMENTED_t keyIdMode;
    NOT_IMPLEMENTED_t keySource;
    NOT_IMPLEMENTED_t keyIndex;
    uint16_t channelOffset;
    uint8_t hoppingSequenceId;
    NOT_IMPLEMENTED_t dsmeAssociation;
    NOT_IMPLEMENTED_t direction;
    NOT_IMPLEMENTED_t allocationOrder;
    bool hoppingSequenceRequest;
};

struct ASSOCIATE_confirm_parameters {
    uint16_t assocShortAddress;
    AssociationStatus::Association_Status status;
    NOT_IMPLEMENTED_t securityLevel;
    NOT_IMPLEMENTED_t keyIdMode;
    NOT_IMPLEMENTED_t keySource;
    NOT_IMPLEMENTED_t keyIndex;
    uint16_t channelOffset;
    uint8_t  *hoppingSequence; //TODO
    NOT_IMPLEMENTED_t dsmeAssociation;
    NOT_IMPLEMENTED_t allocationOrder;
    NOT_IMPLEMENTED_t biIndex;
    NOT_IMPLEMENTED_t superframeId;
    NOT_IMPLEMENTED_t slotId;
    NOT_IMPLEMENTED_t channelIndex;
};

/*
 * These primitives are used when a device becomes associated with a PAN (IEEE 802.15.4-2011 6.2.2 and IEEE 802.15.4e-2012 updates )
 */
class ASSOCIATE : public IndicationBase<ASSOCIATE_indication_parameters>, public ConfirmBase<ASSOCIATE_confirm_parameters> {
public:
    explicit ASSOCIATE(DSMELayer& dsme);

    struct request_parameters {
        uint8_t channelNumber;
        uint8_t channelPage;
        AddrMode coordAddrMode;
        uint16_t coordPanId;
        IEEE802154MacAddress coordAddress;
        CapabilityInformation capabilityInformation;
        NOT_IMPLEMENTED_t securityLevel;
        NOT_IMPLEMENTED_t keyIdMode;
        NOT_IMPLEMENTED_t keySource;
        NOT_IMPLEMENTED_t keyIndex;
        uint16_t channelOffset;     // To be ignored, when in channel adaption mode
        uint8_t hoppingSequenceId; // To be ignored, when in channel adaption mode
        NOT_IMPLEMENTED_t dsmeAssociation;
        NOT_IMPLEMENTED_t direction;
        NOT_IMPLEMENTED_t allocationOrder;
        bool hoppingSequenceRequest;
    };

    struct response_parameters {
        IEEE802154MacAddress deviceAddress;
        uint16_t assocShortAddress;
        NOT_IMPLEMENTED_t securityLevel;
        NOT_IMPLEMENTED_t keyIdMode;
        NOT_IMPLEMENTED_t keySource;
        NOT_IMPLEMENTED_t keyIndex;
        uint16_t channelOffset;
        uint8_t *hoppingSequence; //TODO
        NOT_IMPLEMENTED_t dsmeAssociation;
        NOT_IMPLEMENTED_t allocationOrder;
        NOT_IMPLEMENTED_t biIndex;
        NOT_IMPLEMENTED_t superframeId;
        NOT_IMPLEMENTED_t slotId;
        NOT_IMPLEMENTED_t channelIndex;
        AssociationStatus::Association_Status status;
    };

    void request(request_parameters&);

    void response(response_parameters&);

private:
    DSMELayer& dsme;
};

} /* namespace mlme_sap */
} /* namespace dsme */

#endif /* ASSOCIATE_H_ */
