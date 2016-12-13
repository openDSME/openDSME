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

#include "MAC_PIB.h"

#include "dsme_mac_constants.h"

namespace dsme {

template<typename T>
inline T min(T a, T b) {
    return (a < b) ? a : b;
}

template<typename T, typename F>
inline T sum(uint8_t min, uint8_t max, F func) {
    T result = 0;
    for (uint8_t k = min; k <= max; k++) {
        result += func(k);
    }
    return result;
}

MAC_PIB::MAC_PIB(PHY_PIB& phy_pib) :
    helper(phy_pib, *this),
    phy_pib(phy_pib),

    macExtendedAddress(0, 0, 0, 0),
    macAckWaitDuration(aUnitBackoffPeriod + aTurnaroundTimeSymbols + phy_pib.phySHRDuration + 6 * phy_pib.phySymbolsPerOctet),
    macAssociatedPANCoord(false),
    macAssociationPermit(false),
    macAutoRequest(false),
    macCoordExtendedAddress(0, 0, 0, 0),
    macBeaconOrder(15),
    macCoordShortAddress(0xffff),
    macMaxBE(5),
    macMaxCSMABackoffs(4),
    macMaxFrameRetries(3),
    macMinBE(3),
    macMaxFrameTotalWaitTime(0),
    macPANId(0xffff),
    macShortAddress(0xffff),
    macSuperframeOrder(15),
    macResponseWaitTime(32),

    macChannelDiversityMode(0x00),
    macMultiSuperframeOrder(15),
    macDSMESAB(),
    macDSMEACT(),
    macDSMEGTSExpirationTime(7) {
    recalculateDependentProperties();
}

void MAC_PIB::recalculateDependentProperties() {
    /* macMaxFrameTotalWaitTime (IEEE 802.15.4-2011 6.4.3.) */
    uint8_t m = min(((uint8_t) (this->macMaxBE - this->macMinBE)), this->macMaxCSMABackoffs);
    uint16_t partialSum = sum<uint16_t>(0, m - 1, [&] (uint8_t k) {
        return 1 << (this->macMinBE + k);
    });
    this->macMaxFrameTotalWaitTime = (partialSum + ((1 << this->macMinBE) - 1) * (this->macMaxCSMABackoffs - m)) * aUnitBackoffPeriod
                                     + this->phy_pib.phyMaxFrameDuration;

    return;
}

} /* dsme */
