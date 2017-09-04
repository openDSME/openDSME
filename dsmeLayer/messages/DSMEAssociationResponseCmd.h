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

#ifndef ASSOCIATEREPLYCMD_H_
#define ASSOCIATEREPLYCMD_H_

#include "../../mac_services/DSME_Common.h"
#include "../../mac_services/dataStructures/DSMEMessageElement.h"

namespace dsme {
class DSMEAssociationResponseCmd : public DSMEMessageElement {
public:
    DSMEAssociationResponseCmd() : shortAddr(0), status(AssociationStatus::Association_Status::SUCCESS), hoppingSequenceLength(0), hoppingSequence(nullptr) {
    }

    DSMEAssociationResponseCmd(uint16_t shortAddr, AssociationStatus::Association_Status status, uint8_t hoppingSequenceLength, uint8_t *hoppingSequence, NOT_IMPLEMENTED_t allocationOrder, NOT_IMPLEMENTED_t biIdx, NOT_IMPLEMENTED_t superframeId, NOT_IMPLEMENTED_t slotId, NOT_IMPLEMENTED_t channelIdx) : shortAddr(shortAddr), status(status), hoppingSequenceLength(hoppingSequenceLength), hoppingSequence(hoppingSequence), allocationOrder(allocationOrder), biIdx(biIdx), superframeId(superframeId), slotId(slotId), channelIdx(channelIdx) { //TODO
    }

    uint16_t getShortAddr() const {
        return this->shortAddr;
    }

    AssociationStatus::Association_Status getStatus() const {
        return this->status;
    }

    uint8_t getHoppingSequenceLength() const {
        return this->hoppingSequenceLength;
    }

    uint8_t* getHoppingSequence() const {  //TODO
        return this->hoppingSequence;
    }

    NOT_IMPLEMENTED_t getAllocationOrder() const {
        return this->allocationOrder;
    }

    NOT_IMPLEMENTED_t getBiIndex() const {
        return this->biIdx;
    }
        
    NOT_IMPLEMENTED_t getSuperframeId() const {
        return this->superframeId;
    }

    NOT_IMPLEMENTED_t getSlotId() const {
        return this->slotId;
    }

    NOT_IMPLEMENTED_t getChannelIdx() const {
        return this->channelIdx;
    }

    virtual uint8_t getSerializationLength() {
        uint8_t size = 0;
        size += 2; // shortAddr
        size += 1; // status
        size += 1; // hoppingSequenceLength
        size += hoppingSequenceLength; //hoppingSequence
        //size += 1; // allocationOrder -> not implemented
        //size += 1; // biIndex -> not implemented
        //size += 2; // superframeId -> not implemented
        //size += 1; // slotId -> not implemented
        //size += 2; // channelIdx -> not implemented
        return size;
    }

    virtual void serialize(Serializer& serializer) { //TODO dependent on parameters
        serializer << shortAddr;
        uint8_t stat = (uint8_t)status;
        serializer << stat;
        status = (AssociationStatus::Association_Status)stat;
        serializer << hoppingSequenceLength;
        for(int i=0; i<hoppingSequenceLength; i++) {
            serializer << hoppingSequence[i];
        }
    }

private:
    uint16_t shortAddr;
    AssociationStatus::Association_Status status;
    uint8_t hoppingSequenceLength;
    uint8_t *hoppingSequence; //TODO
    NOT_IMPLEMENTED_t allocationOrder;
    NOT_IMPLEMENTED_t biIdx;
    NOT_IMPLEMENTED_t superframeId;
    NOT_IMPLEMENTED_t slotId;
    NOT_IMPLEMENTED_t channelIdx;
};

} /* namespace dsme */

#endif /* ASSOCIATEREPLYCMD_H_ */
