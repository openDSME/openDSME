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

#ifndef DSMEASSOCIATIONREQUESTCMD_H_
#define DSMEASSOCIATIONREQUESTCMD_H_

#include "./AssociateRequestCmd.h"

namespace dsme {
class DSMEAssociationRequestCmd : public AssociateRequestCmd {
public:
    DSMEAssociationRequestCmd() : hoppingSequenceId(0), channelOffset(0) {
    }

    explicit DSMEAssociationRequestCmd(CapabilityInformation capabilityInformation, uint8_t hoppingSequenceId, uint16_t channelOffset,
                                       NOT_IMPLEMENTED_t extendedDsmeGtsAllocation)
        : AssociateRequestCmd(capabilityInformation), hoppingSequenceId(hoppingSequenceId), channelOffset(channelOffset) {
    }

    uint8_t getHoppingSequenceId() const {
        return this->hoppingSequenceId;
    }

    uint16_t getChannelOffset() const {
        return this->channelOffset;
    }

    NOT_IMPLEMENTED_t getExtendedDsmeGtsAllocation() const {
        return this->extendedDsmeGtsAllocation;
    }

    virtual uint8_t getSerializationLength() {
        uint8_t size = 0;
        size += 1; // capabilityInformation
        size += 1; // hoppingSequenceId
        size += 2; // channelOffset
        return size;
    }

    virtual void serialize(Serializer& serializer) {
        if(serializer.getType() == SERIALIZATION) { // TODO
            uint8_t info = 0;
            info |= capabilityInformation.alternatePANCoordinator;
            info |= capabilityInformation.deviceType << 1;
            info |= capabilityInformation.powerSource << 2;
            info |= capabilityInformation.receiverOnWhenIdle << 3;
            info |= capabilityInformation.associationType << 4;
            info |= capabilityInformation.securityCapability << 6;
            info |= capabilityInformation.allocateAddress << 7;
            serializer << info;
        } else {
            uint8_t info;
            serializer << info;
            capabilityInformation.alternatePANCoordinator = info & (1 << 0);
            capabilityInformation.deviceType = info & (1 << 1);
            capabilityInformation.powerSource = info & (1 << 2);
            capabilityInformation.receiverOnWhenIdle = info & (1 << 3);
            capabilityInformation.associationType = info & (1 << 4);
            capabilityInformation.securityCapability = info & (1 << 6);
            capabilityInformation.allocateAddress = info & (1 << 7);
        }
        serializer << hoppingSequenceId;
        serializer << channelOffset;
    }

private:
    uint8_t hoppingSequenceId;
    uint16_t channelOffset;
    NOT_IMPLEMENTED_t extendedDsmeGtsAllocation;
};

} /* namespace dsme */

#endif /* DSMEASSOCIATIONREQUESTCMD_H_ */
