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

#include "PHY_PIB.h"

#include "dsme_mac_constants.h"

namespace dsme {

PHY_PIB::PHY_PIB(uint8_t phySHRDuration) :
        phyCurrentChannel(11),
        phyChannelsSupported(),
        phyCurrentPage(0),
        phySHRDuration(phySHRDuration),
        phySymbolsPerOctet(2),
        phyMaxFrameDuration(phySHRDuration + ((aMaxPHYPacketSize + 1) * phySymbolsPerOctet)) {

    /* 11 <= phyCurrentChannel <= 26 for 2450 MHz band DSSS */
    channelList_t DSSS2450_channels(16);
    for (uint8_t i = 0; i < 16; i++) {
        DSSS2450_channels[i] = 11 + i;
    }
    MacTuple<uint8_t, channelList_t> *DSSS2450_page0 = new MacTuple<uint8_t, channelList_t>(0, DSSS2450_channels);

    phyChannelsSupported.setLength(1);
    phyChannelsSupported[0] = DSSS2450_page0;

}

PHY_PIB::~PHY_PIB() {
    for (uint8_t i = 0; i < phyChannelsSupported.getLength(); i++) {
        delete phyChannelsSupported[0];
    }
}

} /* dsme */