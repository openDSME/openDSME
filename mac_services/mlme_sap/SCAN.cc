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

#include "SCAN.h"

#include "../../dsmeLayer/DSMELayer.h"
#include "../../dsmeLayer/beaconManager/BeaconManager.h"

namespace dsme {
namespace mlme_sap {
SCAN::SCAN(DSMELayer& dsme) :
    dsme(dsme) {
}

void SCAN::request(request_parameters& params) {
    switch (params.scanType) {
        /*
         * IEEE802.15.4-2012 6.2.10.1
         */
        case ScanType::ED:
            break;
        case ScanType::ACTIVE:

            break;

        case ScanType::PASSIVE:
            dsme.getBeaconManager().startScanPassive(params.scanDuration, params.scanChannels);
            break;

        case ScanType::ORPHAN:
            break;

        case ScanType::ASYMMETRICMULTICHANNELACTIVE:
            if (params.linkQualityScan) {
                // not implemented
            } else {
                // not implemented
            }
            break;

        case ScanType::CHANNELPROBE:
            break;

        case ScanType::MULTICHANNELHELLO:
            break;

        case ScanType::ENHANCEDACTIVESCAN:
            dsme.getBeaconManager().startScanEnhancedActive(params.scanDuration, params.scanChannels);
            break;
    }

}

} /* mlme_sap */
} /* dsme */
