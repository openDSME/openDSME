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
//#include "../../helper/GackHelper.h"
#include "../../mac_services/dataStructures/DSMEGACKBitmap.h"

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
    bool multiplePacketsPerGTS{false};
    bool turnOff{false};
    DSMEGACKBitmap gackBitmap; // TODO: this should be owned by the ack layer, the message dispatcher should not care about acks

public:
    /*! Queues a message for transmission during a GTS.
     *
     * \param msg The message to transmit
     * \param destIt The destination device
     * \return false if the GTS queue is full, true otherwise
     */
    bool sendInGTS(IDSMEMessage* msg, NeighborQueue<MAX_NEIGHBORS>::iterator destIt);

    /*! Queues a message for transmission during the CAP.
     *
     * \param msg The message to transmit
     * \return true if the message was pushed to the #CAPLayer, false otherwise
     */
    bool sendInCAP(IDSMEMessage* msg);


    inline NeighborQueue<MAX_NEIGHBORS>& getNeighborQueue() {
        return neighborQueue;
    }

    inline void addNeighbor(const IEEE802154MacAddress& address) {
        Neighbor n(address);
        neighborQueue.addNeighbor(n);
        retransmissionQueue.addNeighbor(n);
    }

    inline bool neighborExists(const IEEE802154MacAddress& address) {
        return neighborQueue.findByAddress(address) != neighborQueue.end();
    }

    inline void setSendMultiplePacketsPerGTS(bool multiplePacketsPerGTS) {
        this->multiplePacketsPerGTS = multiplePacketsPerGTS;
    }


/* Event handlers (START) ----------------------------------------------------*/
    /*! This shall be called shortly before the start of every slot to allow for setting up the transceiver.
     *
     * \param nextSlot The upcoming slot number
     * \param nextSuperframe The upcoming superframe number
     * \param nextMultiSuperframe The upcoming multisuperframe number
     *
     * \return false if the MessageDispatcher is busy and can not handle the event, true otherwise
     */
    bool handlePreSlotEvent(uint8_t nextSlot, uint8_t nextSuperframe, uint8_t nextMultiSuperframe);

    /*! This shall be called at the start of every slot.
     *
     * \param slot The new slot number
     * \param superframe The new superframe number
     *
     * \return false if the MessageDispatcher is busy and can not handle the event, true otherwise
     */
    bool handleSlotEvent(uint8_t slot, uint8_t superframe, int32_t lateness);

    /*!
     * This shall be called one SIFS or LIFS after the reception of an acknowledgement,
     * depending on the length of the transmitted frame. Transmits the next frame
     * AFTER a frame has already been transmitted.
     */
    bool handleIFSEvent(int32_t lateness);

    //TODO JND
    void handleAckTransmitted();

    /*! This shall be called when CSMA Message was sent down to the physical layer.
     *
     * \param msg The sent message
     * \param status The status of the transmission
     * \param numBackoffs The number of required backoffs
     * \param transmissionAttempts The number of required transmission attempts
     */
    void onCSMASent(IDSMEMessage* msg, DataStatus::Data_Status status, uint8_t numBackoffs, uint8_t transmissionAttempts);

    /*! This shall be send after a GTS transmission.
     *
     * \param response The status of the transmission
     * \param msg The sent message
     */
    void sendDoneGTS(enum AckLayerResponse response, IDSMEMessage* msg);

    /*! This shall be called to receive a message after it has been decoupled from
     * the ISR control flow.
     *
     * \param msg The message to received
     */
    void receive(IDSMEMessage* msg);

    bool handleGackBitmap(DSMEGACKBitmap &bitmap, IEEE802154MacAddress &srcAddr);

/* Event handlers (END) ------------------------------------------------------*/

protected:
    DSMEAllocationCounterTable::iterator currentACTElement;

    AckLayer::done_callback_t doneGTS;

    IDSMEMessage* dsmeAckFrame;

    NeighborQueue<MAX_NEIGHBORS> neighborQueue;
    NeighborQueue<MAX_NEIGHBORS> retransmissionQueue;

    NeighborQueue<MAX_NEIGHBORS>::iterator lastSendGTSNeighbor;

    IDSMEMessage *preparedMsg{nullptr};

    /*!
     * Called on start of every GTSlot.
     * Switch channel for reception or transmit from queue in allocated slots. TODO: correct?
     */
    void handleGTS(int32_t lateness);

    /*!
     * Called on reception of a GTS frame. Send Ack and send payload to upper layer.
     */
    void handleGTSFrame(IDSMEMessage* msg);


    bool prepareGackCommand();

    bool handleGackReception(IDSMEMessage* msg);

    /*! Prepares the next GTS message from the packet queue for transmission.
     *\return true if a message was prepared, false otherwise, i.e., if there is
     *        no packet in the queue or the remaining time is not sufficient for
     *        transmission.
     */
    bool prepareNextMessageIfAny();

    /*! Transmits the prepared GTS message by passing the message to the ACKLayer.
     *  The MessageDispatcher maintains ownership of the packet so it must not be
     *  deleted before the ACKLayer finishes the transmission. The callback-function
     *  sendDoneGTS is called after every attempted transmission.
     *\brief Transmits the prepared GTS message.
     *\return true if the transmission is attempted, false otherwise.
     */
    bool sendPreparedMessage();

    void createDataIndication(IDSMEMessage* msg);

    /*! Finalizes the current GTS. Turns off the transceiver if transmitting,
     *  resets the neighbor associated with the time slot and ensures there
     *  is no message pending.
     */
    void finalizeGTSTransmission();

    void transceiverOffIfAssociated();

    /*! Returns the next channel in the hopping sequence.
     */
    uint8_t nextHoppingSequenceChannel(uint8_t nextSlot, uint8_t nextSuperframe, uint8_t nextMultiSuperframe);




/* Statistics (START) ------------------------------------------------------- */
public:
    long getNumUpperPacketsDroppedFullQueue() const {
        return numUpperPacketsDroppedFullQueue;
    }

    long getNumUpperPacketsForCAP() const {
        return numUpperPacketsForCAP;
    }

    long getNumUpperPacketsForGTS() const {
        return numUpperPacketsForGTS;
    }

    long getNumUnusedTxGTS() const {
        return this->numUnusedTxGts;
    }

    long getNumUnusedRxGTS() const {
        return this->numUnusedRxGts;
    }

private:
    long numTxGtsFrames = 0;
    long numRxAckFrames = 0;
    long numRxGtsFrames = 0;
    long numUnusedTxGts = 0;
    long numUnusedRxGts = 0;
    long numRetransmissionPacketsDroppedFullQueue = 0;
    long numUpperPacketsDroppedFullQueue = 0;
    long numUpperPacketsForCAP = 0;
    long numUpperPacketsForGTS = 0;
    bool recordGtsUpdates = false;
/* Statistics (END) --------------------------------------------------------- */
};

} /* namespace dsme */

#endif /* MESSAGEDISPATCHER_H_ */
