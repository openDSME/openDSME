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

#ifndef GTSMANAGEMENT_H
#define GTSMANAGEMENT_H

#include "../../mac_services/DSME_Common.h"
#include "../../mac_services/dataStructures/DSMEMessageElement.h"

namespace dsme {

class GTSManagement : public DSMEMessageElement {
public:
    GTSManagement(ManagementType type, Direction direction, Priority prioritizedChannelAccess) :
        type(type),
        direction(direction),
        prioritizedChannelAccess(prioritizedChannelAccess),
        status(GTSStatus::GTS_Status::SUCCESS) {
    }

    GTSManagement(ManagementType type, Direction direction, Priority prioritizedChannelAccess,
                  GTSStatus::GTS_Status status) :
        type(type),
        direction(direction),
        prioritizedChannelAccess(prioritizedChannelAccess),
        status(status) {
    }

    GTSManagement() :
        type(DEALLOCATION),
        direction(RX),
        prioritizedChannelAccess(HIGH),
        status(GTSStatus::SUCCESS) {
    }

    ManagementType type;
    Direction direction;
    Priority prioritizedChannelAccess;
    GTSStatus::GTS_Status status;

public:
    virtual uint8_t getSerializationLength() const {
        return 1;
    }

    virtual void serialize(Serializer& serializer);
};

}

#endif
