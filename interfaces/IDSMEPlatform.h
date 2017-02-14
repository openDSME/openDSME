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

#ifndef IDSMEPLATFORM_H_
#define IDSMEPLATFORM_H_

#include "../helper/DSMEDelegate.h"
#include "../helper/Integers.h"

namespace dsme {

class IDSMEMessage;

class IDSMEPlatform {
public:
    typedef Delegate<void(IDSMEMessage* msg)> receive_delegate_t;

    /**
     * Sets the current channel for transmitting / receiving
     */
    virtual bool setChannelNumber(uint8_t channel) = 0;

    /**
     * Directly send packet without delay and without CSMA
     * but keep the message (the caller has to ensure that the message is eventually released)
     * This might lead to an additional memory copy in the platform
     */
    virtual bool prepareSendingCopy(IDSMEMessage* msg, Delegate<void(bool)> txEndCallback) = 0;

    virtual bool sendNow() = 0;

    virtual void abortPreparedTransmission() = 0;

    /**
     * Send an ACK message, delay until aTurnaRoundTime after reception_time has expired
     */
    virtual bool sendDelayedAck(IDSMEMessage* ackMsg, IDSMEMessage* receivedMsg, Delegate<void(bool)> txEndCallback) = 0;

    virtual void setReceiveDelegate(receive_delegate_t receiveDelegate) = 0;

    virtual bool isReceptionFromAckLayerPossible() = 0;

    virtual void handleReceivedMessageFromAckLayer(IDSMEMessage* message) = 0;

    virtual IDSMEMessage* getEmptyMessage() = 0;

    virtual void releaseMessage(IDSMEMessage* msg) = 0;

    virtual bool startCCA() = 0;

    virtual void startTimer(uint32_t symbolCounterValue) = 0;

    virtual uint32_t getSymbolCounter() = 0;

    virtual uint16_t getRandom() = 0;

    virtual void updateVisual() = 0;

    virtual void scheduleStartOfCFP() = 0;

    // Beacons with LQI lower than this will not be considered when deciding for a coordinator to associate to
    virtual uint8_t getMinCoordinatorLQI() = 0;
};

} /* namespace dsme */

#endif /* IDSMEPLATFORM_H_ */
