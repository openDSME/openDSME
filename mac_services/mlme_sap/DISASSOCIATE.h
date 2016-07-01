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

#ifndef DISASSOCIATE_H_
#define DISASSOCIATE_H_

#include "../ConfirmBase.h"
#include "../DSME_Common.h"
#include "../IndicationBase.h"
#include "../dataStructures/IEEE802154MacAddress.h"

namespace dsme {
class DSMELayer;

namespace mlme_sap {

struct DISASSOCIATE_indication_parameters {
    IEEE802154MacAddress deviceAddress;
    DisassociateReason disassociateReason;
    uint8_t securityLevel;
    uint8_t keyIdMode;
    uint8_t *keySource;
    uint8_t keyIndex;
};

struct DISASSOCIATE_confirm_parameters {
    DisassociationStatus::Disassociation_Status status;
    AddrMode deviceAddrMode;
    uint16_t devicePANId;
    IEEE802154MacAddress deviceAddress;

};

/*
 * These primitives are used by a device to disassociate from a PAN or by the coordinator to disassociate a
 * device from a PAN. (IEEE 802.15.4-2011 6.2.3)
 */
class DISASSOCIATE : public IndicationBase<DISASSOCIATE_indication_parameters>,
                     public ConfirmBase<DISASSOCIATE_confirm_parameters> {
public:
    DISASSOCIATE(DSMELayer& dsme);
    struct request_parameters {
        AddrMode deviceAddrMode;
        uint16_t devicePANId;
        IEEE802154MacAddress deviceAddress;
        DisassociateReason disassociateReason;
        bool txIndirect; //TRUE if the disassociation notification command is to be sent indirectly.
        uint8_t securityLevel;
        uint8_t keyIdMode;
        uint8_t *keySource;
        uint8_t keyIndex;
    };

    void request(request_parameters&);

private:
    DSMELayer& dsme;
};

} /* mlme_sap */
} /* dsme */

#endif /* DISASSOCIATE_H_ */
