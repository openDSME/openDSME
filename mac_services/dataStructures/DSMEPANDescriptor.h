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

#ifndef DSMEPANDESCRIPTOR_H_
#define DSMEPANDESCRIPTOR_H_

#include "../DSME_Common.h"
#include "./BeaconBitmap.h"
#include "./ChannelHoppingSpecification.h"
#include "./DSMEMessageElement.h"
#include "./DSMESuperframeSpecification.h"
#include "./IEEE802154MacAddress.h"
#include "./PendingAddresses.h"
#include "./SuperframeSpecification.h"
#include "./TimeSyncSpecification.h"

namespace dsme {

struct DSMEPANDescriptor : public DSMEMessageElement {
    SuperframeSpecification superframeSpec;
    PendingAddresses pendingAddresses;
    DSMESuperframeSpecification dsmeSuperframeSpec;
    TimeSyncSpecification timeSyncSpec;
    BeaconBitmap beaconBitmap;
    ChannelHoppingSpecification channelHoppingSpecification;

    BeaconBitmap& getBeaconBitmap() {
        return beaconBitmap;
    }

    DSMEPANDescriptor& operator=(const DSMEPANDescriptor& other) {
        this->superframeSpec = other.superframeSpec;
        this->pendingAddresses = other.pendingAddresses;
        this->dsmeSuperframeSpec = other.dsmeSuperframeSpec;
        this->timeSyncSpec = other.timeSyncSpec;
        this->beaconBitmap.setNumberOfBeaconSlots(other.beaconBitmap.getSDBitmapLengthBytes()*8, false);
        this->beaconBitmap.copyBitsFrom(other.beaconBitmap);
        return *this;
    }

    TimeSyncSpecification& getTimeSyncSpec() {
        return timeSyncSpec;
    }

    virtual uint8_t getSerializationLength() {
        uint8_t size = 0;
        size += 2;                                                    // Superframe Specification
        size += pendingAddresses.getSerializationLength();            // Pending Address
        size += 1;                                                    // DSME Superframe Specification
        size += 8;                                                    // Time Synchronization Specification
        size += beaconBitmap.getSerializationLength();                // Beacon Bitmap
        size += channelHoppingSpecification.getSerializationLength(); // Channel Hopping Specification //TODO only if channel hopping
        size += 0;                                                    // Group ACK Specification (not yet implemented)
        return size;
    }

    virtual void serialize(Serializer& serializer) {
        serializer << superframeSpec;
        serializer << pendingAddresses;
        serializer << dsmeSuperframeSpec;
        serializer << timeSyncSpec;
        serializer << beaconBitmap;
        serializer << channelHoppingSpecification; // TODO only if channel hopping
    }
};

} /* namespace dsme */

#endif /* DSMEPANDESCRIPTOR_H_ */
