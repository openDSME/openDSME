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

#ifndef IDSMEMESSAGE_H_
#define IDSMEMESSAGE_H_

#include "../dsmeLayer/messages/IEEE802154eMACHeader.h"
#include "../helper/Integers.h"
#include "../mac_services/dataStructures/DSMEMessageElement.h"

namespace dsme {

class IDSMEMessage {
public:
    virtual ~IDSMEMessage() = default;

    virtual void prependFrom(DSMEMessageElement* msg) = 0;

    virtual void decapsulateTo(DSMEMessageElement* msg) = 0;

    virtual bool hasPayload() = 0;

    virtual uint32_t getStartOfFrameDelimiterSymbolCounter() = 0;

    virtual void setStartOfFrameDelimiterSymbolCounter(uint32_t) = 0;

    virtual uint16_t getTotalSymbols() = 0;

    virtual IEEE802154eMACHeader& getHeader() = 0;

    virtual uint8_t getLQI() = 0;

    virtual bool getReceivedViaMCPS() = 0;

    virtual void setReceivedViaMCPS(bool receivedViaMCPS) = 0;

    virtual bool getCurrentlySending() = 0;

    virtual void setCurrentlySending(bool currentlySending) = 0;

    virtual void increaseRetryCounter() = 0;

    virtual uint8_t getRetryCounter() = 0;

    uint8_t queueAtCreation = -1;
};

} /* namespace dsme */

#endif /* IDSMEMESSAGE_H_ */
