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

#ifndef PANDESCRIPTOR_H_
#define PANDESCRIPTOR_H_

#include "./DSMEPANDescriptor.h"

namespace dsme {

class PANDescriptor {
public:
    /* IEEE 802.15.4-2011 */
    AddrMode coordAddrMode;
    uint16_t coordPANId;
    IEEE802154MacAddress coordAddress;
    uint8_t channelNumber;
    uint8_t channelPage;
    bool GTSPermit;
    uint8_t linkQuality;
    int8_t rssi; // not available in standard!
    uint32_t timestamp;
    SecurityStatus::Security_Status securityStatus;
    uint8_t securityLevel;
    uint8_t keyIdMode;
    uint8_t* keySource;
    uint8_t keyIndex;
    uint8_t* codeList;

    /* IEEE 802.15.4e-2012 */
    DSMEPANDescriptor dsmePANDescriptor;

public:
    PANDescriptor& operator=(const PANDescriptor& other) {
        this->coordAddrMode = other.coordAddrMode;
        this->coordPANId = other.coordPANId;
        this->coordAddress = other.coordAddress;
        this->channelNumber = other.channelNumber;
        this->channelPage = other.channelPage;
        this->GTSPermit = other.GTSPermit;
        this->linkQuality = other.linkQuality;
        this->timestamp = other.timestamp;
        this->securityStatus = other.securityStatus;
        this->securityLevel = other.securityLevel;
        this->keyIdMode = other.keyIdMode;
        this->keySource = other.keySource;
        this->keyIndex = other.keyIndex;
        this->codeList = other.codeList;

        this->dsmePANDescriptor = other.dsmePANDescriptor;
        return *this;
    }
};

} /* namespace dsme */

#endif /* PANDESCRIPTOR_H_ */
