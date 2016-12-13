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

#ifndef GTSREQUESTCMD_H
#define GTSREQUESTCMD_H

#include "../../mac_services/dataStructures/DSMEMessageElement.h"
#include "../../mac_services/dataStructures/DSMESABSpecification.h"

namespace dsme {

class GTSRequestCmd : public DSMEMessageElement {
private:
    uint8_t numSlots;
    uint16_t preferredSuperframeID;
    uint8_t preferredSlotID;
    DSMESABSpecification SABSpec;

public:
    GTSRequestCmd(int8_t numSlots, uint16_t preferredSuperframeID, uint8_t preferredSlotID, const DSMESABSpecification& SABSpec) :
        numSlots(numSlots), preferredSuperframeID(preferredSuperframeID), preferredSlotID(preferredSlotID), SABSpec(SABSpec) {
    }

    GTSRequestCmd() {

    }

    uint8_t getNumSlots() const {
        return numSlots;
    }

    void setNumSlots(uint8_t numSlots) {
        this->numSlots = numSlots;
    }

    uint16_t getPreferredSuperframeID() const {
        return preferredSuperframeID;
    }

    void setPreferredSuperframeID(uint16_t preferredSuperframeID) {
        this->preferredSuperframeID = preferredSuperframeID;
    }

    uint8_t getPreferredSlotID() const {
        return preferredSlotID;
    }

    void setPreferredSlotID(uint8_t preferredSlotID) {
        this->preferredSlotID = preferredSlotID;
    }

    DSMESABSpecification& getSABSpec() {
        return SABSpec;
    }

public:
    virtual uint8_t getSerializationLength() const {
        uint8_t size = 0;
        size += 1; // number of slots
        size += 2; // preferred superframe ID
        size += 1; // preferred slot ID
        size += SABSpec.getSerializationLength();
        return size;
    }

    virtual void serialize(Serializer& serializer) {
        serializer << numSlots;
        serializer << preferredSuperframeID;
        serializer << preferredSlotID;
        serializer << SABSpec;
    }
};

}

#endif
