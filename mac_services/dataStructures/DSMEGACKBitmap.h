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

#ifndef DSMEGACKBITMAP_H_
#define DSMEGACKBITMAP_H_

/* INCLDUDES *****************************************************************/

#include "../../helper/Integers.h"
#include "./IEEE802154MacAddress.h"
#include "./Serializer.h"
#include "./RBTree.h"
#include "../../helper/DSMELinkedList.h"    //TODO: move to data structures
#include "DSMEGACKBitmapFragment.h"

class Serializer;


/* CLASSES *******************************************************************/

namespace dsme {

class DSMEGACKBitmap {
public:
    DSMEGACKBitmap() = default;
    ~DSMEGACKBitmap() = default;

    /** Returns the sequence number of the next acknowledged packet.
     *
     * @param addr The source address of the received packet
     *
     * @return The sequence number of the next acknowledged packet
     */
    auto getNextSequenceNumber(IEEE802154MacAddress const& addr) -> uint8_t;


    /** Returns the number of acknowledged packets in the GACK bitmap for the given source.
     *
     * @param addr The source address of the received packets
     *
     * @return The number of acknowledged packets in the bitmap
     */
    auto getNumberOfPackets(IEEE802154MacAddress const& addr) -> uint8_t;

    /** Registers a received packet in the GACK bitmap.
     *
     * @param addr The source address of the received packet
     * @param sequenceNumber The sequence number of the received packet
     */
    auto registerPacket(IEEE802154MacAddress const& addr, uint8_t const sequenceNumber) -> void;

    /** Resets the GACK bitmap.
     * Calling this function during normal operation is not necessary because it resets during serialization
     */
    auto reset() -> void;

    virtual auto getSerializationLength() -> uint8_t;

protected:
    friend auto operator<<(Serializer& serializer, DSMEGACKBitmap& gack) -> Serializer&;

private:
    RBTree<DSMEGACKBitmapFragment*, IEEE802154MacAddress> bitmap; // TODO do not store seqNbr here but some encoded format
};

} /* namespace dsme */

#endif /* DSMEGACKBITMAP_H_ */
