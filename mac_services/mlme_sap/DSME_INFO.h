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

#ifndef DSME_INFO_H_
#define DSME_INFO_H_

#include "../ConfirmBase.h"
#include "../DSME_Common.h"
#include "../IndicationBase.h"
#include "../dataStructures/DSMEPANDescriptor.h"
#include "../dataStructures/DSMESABSpecification.h"
#include "../dataStructures/IEEE802154MacAddress.h"

namespace dsme {
namespace mlme_sap {

struct DSME_INFO_indication_parameters {
    uint64_t deviceAddress;
    Info info;
};

struct DSME_INFO_confirm_parameters {
    Info info;
    uint32_t timestamp;
    uint16_t superframeID;
    uint8_t slotID;
    DSMESABSpecification dsmeSABSpecification;
    DSMEPANDescriptor dsmePANDescriptor;
    InfoStatus::Info_Status status;

};

/*
 * MLME SAP DSME information primitives define how a device can acquire DSME information. (IEEE 802.15.4e-2012 6.2.21.2)
 */
class DSME_INFO : public IndicationBase<DSME_INFO_indication_parameters>,
    public ConfirmBase<DSME_INFO_confirm_parameters> {
public:

    struct request_parameters {
        AddrMode dstAddrMode;
        IEEE802154MacAddress dstAddr;
        Info info;
        uint16_t dsmeSABSubblockLength;
        uint8_t dsmeSABSubblockIndex;
    };

    void request(request_parameters);

};

} /* mlme_sap */
} /* dsme */

#endif /* DSME_INFO_H_ */
