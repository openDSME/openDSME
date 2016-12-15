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

#ifndef GTSREPLYNOTIFYCMD_H
#define GTSREPLYNOTIFYCMD_H

#include "../../mac_services/dataStructures/DSMESABSpecification.h"
#include "GTSManagement.h"

namespace dsme {

class GTSReplyNotifyCmd: public DSMEMessageElement {
protected:
    /*
     * Last variables of DSME GTS reply command. (IEEE 802.15.4e-2012 5.3.11.5)
     * Same for DSME GTS notify command.(IEEE 802.15.4e-2012 5.3.11.6)
     */
    uint16_t destinationAddress;
    uint16_t channelOffset;
    DSMESABSpecification SABSpec;

public:
    GTSReplyNotifyCmd() :
        destinationAddress(0),
        channelOffset(0) {
    }

    //in channel hopping mode
    GTSReplyNotifyCmd(uint16_t destinationAddress, uint16_t channelOffset, const DSMESABSpecification& SABSpec) :
        destinationAddress(destinationAddress),
        channelOffset(channelOffset),
        SABSpec(SABSpec) {
    }

    //in channel adaption mode
    GTSReplyNotifyCmd(uint16_t destinationAddress, const DSMESABSpecification& SABSpec) :
        destinationAddress(destinationAddress),
        channelOffset(0),
        SABSpec(SABSpec) {
    }

    uint16_t getDestinationAddress() const {
        return destinationAddress;
    }

    void setDestinationAddress(uint16_t destinationShortAddress) {
        this->destinationAddress = destinationShortAddress;
    }

    DSMESABSpecification& getSABSpec() {
        return SABSpec;
    }

public:
    virtual uint8_t getSerializationLength() {
        uint8_t size = 0;
        size += 2; // Destination Address
        size += 0; // Channel hopping is not supported
        size += SABSpec.getSerializationLength();
        return size;
    }

    virtual void serialize(Serializer& serializer) {
        serializer << destinationAddress;
        serializer << SABSpec;
    }
};

}

#endif
