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

#ifndef ACTELEMENT_H_
#define ACTELEMENT_H_

#include "../DSME_Common.h"
#include "../../helper/DSMELinkedList.h"

namespace dsme {

enum ACTState { VALID, UNCONFIRMED, INVALID, DEALLOCATED, REMOVED };

class ACTElement {
    friend class DSMEAllocationCounterTable;

public:
    uint16_t getIdleCounter() const {
        return idleCounter;
    }

    bool contains(uint16_t address) {
        return addressList.contains(address);
    }

    uint16_t getAddress(){
        return addressList.getLast();
    }

    void addAddress(uint16_t address) {
        addressList.insertLast(address);
    }

    bool removeAddress(uint16_t address) {
        if(addressList.contains(address))
        {
            addressList.deleteItemIfExists(address);
            return true;
        }
        return false;
    }

    uint16_t getSuperframeID() const {
        return superframeID;
    }

    uint8_t getGTSlotID() const {
        return slotID;
    }

    uint8_t getChannel() const {
        return channel;
    }

    Direction getDirection() const {
        return direction;
    }

    bool isGack() {
        return gack;
    }

    void incrementIdleCounter() {
        idleCounter++;
    }

    void resetIdleCounter() {
        idleCounter = 0;
    }

    bool operator>(const ACTElement& other) const {
        if(this->superframeID > other.superframeID) {
            return true;
        }
        if(this->superframeID < other.superframeID) {
            return false;
        }
        if(this->slotID > other.slotID) {
            return true;
        }
        if(this->slotID < other.slotID) {
            return false;
        }
        return this->channel > other.channel;
    }

    bool operator<(const ACTElement& other) const {
        if(this->superframeID < other.superframeID) {
            return true;
        }
        if(this->superframeID > other.superframeID) {
            return false;
        }
        if(this->slotID < other.slotID) {
            return true;
        }
        if(this->slotID > other.slotID) {
            return false;
        }
        return this->channel < other.channel;
    }

    ACTState getState() {
        return state;
    }

    void setState(ACTState newState) {
        state = newState;
    }

private:
    ACTElement(uint16_t superframeID, uint8_t slotID, uint8_t channel, Direction direction, uint16_t address, ACTState state, bool gack = false)
        : superframeID(superframeID), slotID(slotID), channel(channel), direction(direction), idleCounter(0), state(state), gack(gack) {
        addressList.insertLast(address);
    }

    uint16_t superframeID;
    uint8_t slotID;
    uint8_t channel;
    Direction direction;
    DSMELinkedList<uint16_t> addressList;
    uint16_t idleCounter;
    bool gack;

    // Slot state
    // TODO implementation specific, not handled by the standard
    //
    // After a duplicate notification, the slot is marked as INVALID.
    //
    // During sending of a DEALLOCATION request, the ACTElement is not removed immediately, but only
    // set to deallocated. It is removed after reception of the reply or after a timeout (NO_DATA).
    //
    // In the invalid or deallocated state, the slot is still used for reception but not for transmission,
    // since a transmission might induce collisions, but a reception could collect pending messages if
    // the slot on the other device is still VALID.
    ACTState state;
};

} /* namespace dsme */

#endif /* ACTELEMENT_H_ */
