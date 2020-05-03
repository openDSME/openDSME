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

#include "./PIBHelper.h"

#include "../../../dsme_platform.h"
#include "../MacDataStructures.h"
#include "./MAC_PIB.h"
#include "./PHY_PIB.h"
#include "./dsme_mac_constants.h"
#include "./dsme_phy_constants.h"

namespace dsme {

PIBHelper::PIBHelper(PHY_PIB& phy_pib, MAC_PIB& mac_pib) : phy_pib(phy_pib), mac_pib(mac_pib) {
    return;
}

uint8_t PIBHelper::getNumberSuperframesPerMultiSuperframe() const {
    /* 2^(MO-SO) */
    return 1 << (uint8_t)(this->mac_pib.macMultiSuperframeOrder - this->mac_pib.macSuperframeOrder);
}

unsigned PIBHelper::getNumberSuperframesPerBeaconInterval() const {
    /* 2^(BO-SO) */
    return 1 << (unsigned)(this->mac_pib.macBeaconOrder - this->mac_pib.macSuperframeOrder);
}

unsigned PIBHelper::getNumberMultiSuperframesPerBeaconInterval() const {
    /*  2^(BO-MO) */
    return 1 << (unsigned)(this->mac_pib.macBeaconOrder - this->mac_pib.macMultiSuperframeOrder);
}

uint8_t PIBHelper::getFinalCAPSlot(uint8_t superframeId) const {
    if((mac_pib.macCapReduction == false) || (superframeId == 0)) {
        return 8;
    } else {
        return 0;
    }
}

uint32_t PIBHelper::getSymbolsPerSlot() const {
    /* aBaseSlotDuration * 2^(SO) */
    return aBaseSlotDuration * (1 << (uint32_t) this->mac_pib.macSuperframeOrder);
}

uint8_t PIBHelper::getNumGTSlots(uint8_t superframeId) const {
    //return (aNumSuperframeSlots - 1 - getFinalCAPSlot(superframeId));
    // proof of concept capOff capOn
    // idea is return always the same number of gts slots per superframes different to 0 ?  is this correct
    // if superframe ID == 0 numGTSSlots = 7 else 15
    if (superframeId == 0){
        return (aNumSuperframeSlots - 9);
    }else if (mac_pib.macCapReduction == true){
        return (aNumSuperframeSlots - 1);
    }else if (mac_pib.macCapReduction == false){
        return (aNumSuperframeSlots - 9);
    }
}

uint8_t PIBHelper::getNumChannels() const {
    for(uint8_t i = 0; i < phy_pib.phyChannelsSupported.getLength(); i++) {
        if(phy_pib.phyChannelsSupported[i]->key == phy_pib.phyCurrentPage) {
            return phy_pib.phyChannelsSupported[i]->value.getLength();
        }
    }
    DSME_ASSERT(false);
    return 0;
}

const channelList_t& PIBHelper::getChannels() const {
    static channelList_t emptyList(0);

    for(uint8_t i = 0; i < phy_pib.phyChannelsSupported.getLength(); i++) {
        if(phy_pib.phyChannelsSupported[i]->key == phy_pib.phyCurrentPage) {
            return phy_pib.phyChannelsSupported[i]->value;
        }
    }
    return emptyList;
}

uint8_t PIBHelper::getSubBlockLengthBytes(uint8_t superframeId) const {
//    return (getNumGTSlots(superframeId) * getNumChannels() - 1) / 8 + 1; ORIGINAL CODE
    if (superframeId == 0){
        return (getNumGTSlots(superframeId) * getNumChannels() - 1) / 8 + 1;
    }else{
        return (15 * getNumChannels() - 1) / 8 + 1;
    }
}

uint16_t PIBHelper::getAckWaitDuration() const {
    return aUnitBackoffPeriod + aTurnaroundTime + phy_pib.phySHRDuration + 6 * phy_pib.phySymbolsPerOctet + ADDITIONAL_ACK_WAIT_DURATION;
}// 12 + 20 + 12 + 12

} /* namespace dsme */
