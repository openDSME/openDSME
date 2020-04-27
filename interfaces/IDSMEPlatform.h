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
#include "../mac_services/DSME_Common.h"
#include "../mac_services/dataStructures/IEEE802154MacAddress.h"
#include "./IDSMERadio.h"

namespace dsme {

class IDSMEMessage;

class IDSMEPlatform : public IDSMERadio {
public:
    /*
     * Returns whether the ACK-layer may currently pass a message up to the DSME-layer
     */
    virtual bool isReceptionFromAckLayerPossible() = 0;

    /*
     * Pass a message up to the DSME-layer and decuple it from the ISR control flow
     */
    virtual void handleReceivedMessageFromAckLayer(IDSMEMessage* message) = 0;

    /*
     * Allocate a new DSMEMessage
     */
    virtual IDSMEMessage* getEmptyMessage() = 0;

    /*
     * Release a DSMEMessage
     */
    virtual void releaseMessage(IDSMEMessage* msg) = 0;

    /*
     * Start a timer at symbolCounterValue symbols
     */
    virtual void startTimer(uint32_t symbolCounterValue) = 0;

    /*
     * Gets the current time in symbols
     */
    virtual uint32_t getSymbolCounter() = 0;

    /*
     * Returns a random integer
     */
    virtual uint16_t getRandom() = 0;

    /*
     * Can be used to force the update of the visual representation during simulation
     */
    virtual void updateVisual() = 0;

    /*
     * Allows the platform to inform the DSME-layer about the start of a CFP while decoupling from the ISR control flow
     */
    virtual void scheduleStartOfCFP() = 0;

    /*
     * Beacons with LQI lower than this will not be considered when deciding for a coordinator to associate to
     */
    virtual uint8_t getMinCoordinatorLQI() = 0;

    /*
     * Signal result of messages with requested acknowledgment sent by the MAC layers.
     * Can be used to generate link statistics (i.e. ETX).
     */
    virtual void signalAckedTransmissionResult(bool success, uint8_t transmissionAttempts, IEEE802154MacAddress receiver) {
    }

    /*
     * Signal GTS allocation or deallocation (indication is cumulative)
     */
    virtual void signalGTSChange(bool deallocation, IEEE802154MacAddress counterpart) {
    }



    //ALLOCATIONS

    // NOTIFIY
 /*    virtual void signalNotifyInitialized() { // DONE
     }
     virtual void signalNotifyBackoffs(uint8_t backoffs) { // DONE
     }
     virtual void signalNotifySendSuccess() { // DONE
     }
     virtual void signalNotifySendFailedChannelAccess() { // DONE
     }
     virtual void signalNotifySendFailedTransactionOverflow() { // DONE
     }*/

    //DEALLOCATIONS

/*    // DEALLOCATION NOTIFY
    virtual void signalDeallocationNotifyInitialized() { // DONE
    }
    virtual void signalDeallocationNotifyBackoffs(uint8_t backoffs) { // DONE
    }
    virtual void signalDeallocationNotifySendSuccess() { // DONE
    }
    virtual void signalDeallocationNotifySendFailedChannelAccess() { // DONE
    }
    virtual void signalDeallocationNotifySendFailedTransactionOverflow() { // DONE
    }*/

    //PER SF
    virtual void signalSuperframe(bool limits){//signal to define the begin and end of a SF
    }

    virtual void signalGTSRequestsTotal(uint16_t allocations) {//signal to retrieve the number of GTS requests
    }
    virtual void signalGTSNotifySuccess(uint16_t allocations) {//signal to retrieve successful handshake GTS allocations
    }
 /*   virtual void signalGTSRequestsFailed(uint16_t allocations) {
    }
    virtual void signalGTSRequestsFailedNoAck(uint16_t allocations) {
    }
    virtual void signalGTSRequestsFailedChannelAccess(uint16_t allocations) {
    }
    virtual void signalGTSRequestsFailedTransactionOverflow(uint16_t allocations) {
    }
    virtual void signalGTSRequestsFailedTimeout(uint16_t allocations) {
    }
    virtual void signalGTSRequestsFailedDenied(uint16_t allocations) {
    }
    virtual void signalGTSRequestsFailedQueue(uint16_t allocations) {
    }
    virtual void signalGTSRequestsFailedDeallocated(uint16_t allocations) {
    }*/

    // QUEUE LEVEL

    virtual void signalGTSQueueLevel(bool push) {
    }

    /*
     * Signal current queue length
     */
    virtual void signalQueueLength(uint32_t length) {
    }

    // QUEUE LEVEL PER MSF
    virtual void signalGTSQueueLevelMSF(uint8_t queueLevel){
    }
};

} /* namespace dsme */

#endif /* IDSMEPLATFORM_H_ */
