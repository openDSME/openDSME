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

#ifndef START_H_
#define START_H_

#include "../ConfirmBase.h"
#include "../DSME_Common.h"
#include "../dataStructures/BeaconBitmap.h"
#include "../dataStructures/DSMESuperframeSpecification.h"

namespace dsme {
class DSMELayer;

namespace mlme_sap {

struct START_confirm_parameters {
    StartStatus::Start_Status status;
};

/*
 * These primitives are used by an FFD to initiate a PAN, to begin using a new superframe configuration, or to
 * stop transmitting beacons.
 * (IEEE 802.15.4-2011 6.2.10 and IEEE 802.15.4e-2012 updates)
 */
class START : public ConfirmBase<START_confirm_parameters> {
public:
    explicit START(DSMELayer &dsme);

    struct request_parameters {
        uint16_t panId;
        uint8_t channelNumber;
        uint8_t channelPage;
        uint32_t startTime;
        uint16_t beaconOrder;
        uint16_t superframeOrder;
        bool panCoordinator; // true = become PanCoord of new Pan, false = begin using a new superframe configuration
        bool batteryLifeExtension;
        bool coordRealignment;
        uint8_t coordRealignSecurityLevel;
        uint8_t coordRealignKeyIdMode;
        uint8_t *coordRealignKeySource;
        uint8_t coordRealignKeyIndex;
        uint8_t beaconSecurityLevel;
        uint8_t beaconKeyIdMode;
        uint8_t *beaconKeySource;
        uint8_t beaconKeyIndex;
        DSMESuperframeSpecification dsmeSuperframeSpecification;
        BeaconBitmap beaconBitmap;
        HoppingDescriptor hoppingDescriptor;
    };

    void request(request_parameters&);

private:
    DSMELayer& dsme;

};

} /* mlme_sap */
} /* dsme */

#endif /* START_H_ */
