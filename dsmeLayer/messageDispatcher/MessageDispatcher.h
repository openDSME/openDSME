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

#ifndef MESSAGEDISPATCHER_H_
#define MESSAGEDISPATCHER_H_

#include "../../../dsme_platform.h"
#include "../../helper/Integers.h"
#include "../../mac_services/dataStructures/DSMEAllocationCounterTable.h"
#include "../ackLayer/AckLayer.h"
#include "../neighbors/NeighborQueue.h"

namespace dsme {

class DSMELayer;

class MessageDispatcher {
public:
    explicit MessageDispatcher(DSMELayer& dsme);
    ~MessageDispatcher();

    void initialize(void);
    void reset(void);

private:
    DSMELayer& dsme;

public:
    void sendDoneGTS(enum AckLayerResponse response, IDSMEMessage* msg);

    /**
     * Gets called when CSMA Message was sent down to the PHY
     */
    void onCSMASent(IDSMEMessage* msg, DataStatus::Data_Status status, uint8_t numBackoffs, uint8_t transmissionAttempts);

    bool sendInGTS(IDSMEMessage* msg, NeighborQueue<MAX_NEIGHBORS>::iterator destIt);

    bool sendInCAP(IDSMEMessage* msg);

    void receive(IDSMEMessage* msg);

    NeighborQueue<MAX_NEIGHBORS>& getNeighborQueue() {
        return neighborQueue;
    }

    void addNeighbor(const IEEE802154MacAddress& address) {
        Neighbor n(address);
        neighborQueue.addNeighbor(n);
    }

    bool neighborExists(const IEEE802154MacAddress& address) {
        return neighborQueue.findByAddress(address) != neighborQueue.end();
    }

    long getNumUpperPacketsDroppedFullQueue() const {
        return numUpperPacketsDroppedFullQueue;
    }

    long getNumUpperPacketsForCAP() const {
        return numUpperPacketsForCAP;
    }

    long getNumUpperPacketsForGTS() const {
        return numUpperPacketsForGTS;
    }

    /**
     * This shall be called shortly before the start of every slot to allow for setting up the transceiver.
     *
     * @param nextSlot The upcoming slot number
     * @param nextSuperframe The upcoming superframe number
     * @param nextMultiSuperframe The upcoming multisuperframe number
     *
     * @return false if the MessageDispatcher is busy and can not handle the event, true otherwise
     */
    bool handlePreSlotEvent(uint8_t nextSlot, uint8_t nextSuperframe, uint8_t nextMultiSuperframe);

    /**
     * This shall be called at the start of every slot.
     *
     * @param slot The new slot number
     * @param superframe The new superframe number
     *
     * @return false if the MessageDispatcher is busy and can not handle the event, true otherwise
     */
    bool handleSlotEvent(uint8_t slot, uint8_t superframe, int32_t lateness);

    /**
     * This shall be called one SIFS or LIFS after the reception of an acknowledgement,
     * depending on the length of the transmitted frame. Transmits the next frame
     * AFTER a frame has already been transmitted.
     */
    bool handleIFSEvent(int32_t lateness);


protected:
    DSMEAllocationCounterTable::iterator currentACTElement;

    AckLayer::done_callback_t doneGTS;

    IDSMEMessage* dsmeAckFrame;

    /**
     * Called on start of every GTSlot.
     * Switch channel for reception or transmit from queue in allocated slots. TODO: correct?
     */
    void handleGTS(int32_t lateness);

    /**
     * Called on reception of a GTS frame. Send Ack and send payload to upper layer.
     */
    void handleGTSFrame(IDSMEMessage* msg);

    /**
     * Prepares the transmission of a single message if the queue is not empty
     *  and no message has been prepared yet.
     */
    bool prepareNextMessageIfAny();

    /**
     * Sends the prepared message if there is sufficient time for transmission.
     */
    void sendPreparedMessage();

    NeighborQueue<MAX_NEIGHBORS> neighborQueue;
    NeighborQueue<MAX_NEIGHBORS>::iterator lastSendGTSNeighbor;
    IDSMEMessage *preparedMsg{nullptr};

    void createDataIndication(IDSMEMessage* msg);

    void finalizeGTSTransmission();
    void transceiverOffIfAssociated();

    /* Statistics */
    long numTxGtsFrames = 0;
    long numRxAckFrames = 0;
    long numRxGtsFrames = 0;
    long numUnusedTxGts = 0;
    long numUnusedRxGts = 0;
    long numUpperPacketsDroppedFullQueue = 0;
    long numUpperPacketsForCAP = 0;
    long numUpperPacketsForGTS = 0;
    bool recordGtsUpdates = false;
};

} /* namespace dsme */

#endif /* MESSAGEDISPATCHER_H_ */
