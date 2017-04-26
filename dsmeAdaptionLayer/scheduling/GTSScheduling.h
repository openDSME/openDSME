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

#ifndef GTSSCHEDULING_H_
#define GTSSCHEDULING_H_

#include "../../mac_services/dataStructures/IEEE802154MacAddress.h"
#include "../../mac_services/dataStructures/RBTree.h"

namespace dsme {

class DSMEAdaptionLayer;

struct GTSSchedulingData {
    GTSSchedulingData() : address(0xffff), messagesInLastMultisuperframe(0), messagesOutLastMultisuperframe(0), slotTarget(1) {
    }

    uint16_t address;

    uint16_t messagesInLastMultisuperframe;
    uint16_t messagesOutLastMultisuperframe;

    int16_t slotTarget;
};

class GTSScheduling {
public:
    GTSScheduling(DSMEAdaptionLayer& dsmeAdaptionLayer) : dsmeAdaptionLayer(dsmeAdaptionLayer) {
    }

    virtual ~GTSScheduling() = default;
    virtual void reset() = 0;
    virtual uint8_t registerIncomingMessage(uint16_t address) = 0;
    virtual void registerOutgoingMessage(uint16_t address, bool success, int32_t serviceTime, uint8_t queueAtCreation) = 0;
    virtual void multisuperframeEvent() = 0;
    virtual int16_t getSlotTarget(uint16_t address) = 0;
    virtual uint16_t getPriorityLink() = 0;

protected:
    DSMEAdaptionLayer& dsmeAdaptionLayer;
};

template <typename SchedulingData>
class GTSSchedulingImpl : public GTSScheduling {
public:
    typedef typename RBTree<SchedulingData, uint16_t>::iterator iterator;

    GTSSchedulingImpl(DSMEAdaptionLayer& dsmeAdaptionLayer) : GTSScheduling(dsmeAdaptionLayer) {
    }

    virtual ~GTSSchedulingImpl() = default;

    virtual void reset() {
        while(this->links.size() > 0) {
            auto it = this->links.begin();
            this->links.remove(it);
        }
    }

    virtual uint8_t registerIncomingMessage(uint16_t address) {
        queueLevel++;
        iterator it = this->links.find(address);
        if(it == this->links.end()) {
            SchedulingData data;
            data.address = address;
            data.messagesInLastMultisuperframe++;
            this->links.insert(data, address);
        } else {
            it->messagesInLastMultisuperframe++;
        }
        return queueLevel;
    }

    virtual void registerOutgoingMessage(uint16_t address, bool success, int32_t serviceTime, uint8_t queueAtCreation) {
        queueLevel--;
        iterator it = this->links.find(address);
        if(it != this->links.end()) {
            it->messagesOutLastMultisuperframe++;
        }

        return;
    }

    virtual int16_t getSlotTarget(uint16_t address) {
        iterator it = this->links.find(address);

        return it->slotTarget;
    }

    static uint16_t abs(int16_t v) {
        if(v > 0) {
            return v;
        } else {
            return -v;
        }
    }

    uint16_t getPriorityLink() {
        uint16_t address = IEEE802154MacAddress::NO_SHORT_ADDRESS;
        int16_t difference = 0;
        for(const SchedulingData& d : this->links) {
            uint16_t slots = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedTxGTS(d.address);

            if(abs(difference) < abs(d.slotTarget - slots)) {
                difference = d.slotTarget - slots;
                address = d.address;
            }
        }
        return address;
    }

protected:
    RBTree<SchedulingData, uint16_t> links;
    uint8_t queueLevel = 0;
};

} /* namespace dsme */

#endif /* GTSSCHEDULING_H_ */
