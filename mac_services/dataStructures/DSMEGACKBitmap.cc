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

/* INCLDUDES *****************************************************************/

#include "./DSMEGACKBitmap.h"
#include "./Serializer.h"
#include "../../../dsme_platform.h"
#include "DSMEBitVector.h"

namespace dsme {

/* PUBLIC METHODS ************************************************************/


auto DSMEGACKBitmap::registerPacket(IEEE802154MacAddress const& addr, uint8_t const sequenceNumber) -> void { //TODO
    auto bitmapIterator = this->bitmap.find(addr);
    if(bitmapIterator != this->bitmap.end()) {
        /* -> some packets from this addr already registered */
        DSMEGACKBitmapFragment *frag = *bitmapIterator;

        while(sequenceNumber < frag->sequenceNumber) {
            DSMEGACKBitmapFragment *prev = new DSMEGACKBitmapFragment(frag->sequenceNumber - 8);
            prev->next = frag;
            frag = prev;
            *bitmapIterator = frag;
        }
        while(frag->getLastSequenceNumber() < sequenceNumber) {     // TODO wraparound from 255 to 0 ????
            if(frag->next == nullptr) {
                /* -> new fragment required */
                frag->next = new DSMEGACKBitmapFragment(frag->getLastSequenceNumber()+1);
            }
            frag = frag->next;
        }

        /* -> mark bit in bitmap fragment */
        frag->setBit(sequenceNumber - frag->sequenceNumber, true);
    } else {
        /* -> Add the first fragment for this address */
        this->bitmap.insert(new DSMEGACKBitmapFragment(sequenceNumber, 1), addr);
    }
}


auto DSMEGACKBitmap::getNumberOfPackets(IEEE802154MacAddress const& addr) -> uint8_t {
    uint8_t packets = 0;
    auto bitmapIterator = bitmap.find(addr);
    if(bitmapIterator != bitmap.end()) {
        DSMEGACKBitmapFragment *bitmapFragment = *bitmapIterator;
        while(bitmapFragment != nullptr) {
            packets += bitmapFragment->getNumSetBits();
            bitmapFragment = bitmapFragment->next;
        }
    } // else: no packets registered yet

    return packets;
}

auto DSMEGACKBitmap::getNextSequenceNumber(IEEE802154MacAddress const& addr) -> uint8_t {
    auto bitmapIterator = bitmap.find(addr);
    DSME_ASSERT(bitmapIterator != bitmap.end());
    DSMEGACKBitmapFragment *frag = *bitmapIterator;

    /* retrieve and delete next sequence number */
    uint8_t bitmapIndex = frag->getFirstSetBitIndex();
    uint8_t sequenceNumber = frag->sequenceNumber + bitmapIndex;
    frag->setBit(bitmapIndex, false);

    if(frag->getNumSetBits() == 0) {
        /* we removed all packets from fragment so delete it */
        if(frag->next == nullptr) {
            bitmap.remove(bitmapIterator);
        } else {
            *bitmapIterator = frag->next;
        }
        delete frag;
    }

    return sequenceNumber;
}

uint8_t DSMEGACKBitmap::getSerializationLength() {
    uint8_t serializationLength = 1; // number of addresses
    for(auto bitmapIterator = bitmap.begin(); bitmapIterator != bitmap.end(); ++bitmapIterator) {
        serializationLength += 2; // TODO use short address for now but this should be generalized
        serializationLength += 1; // bitmap length
        serializationLength += 1; // sequence number

        DSMEGACKBitmapFragment *frag = *bitmapIterator;
        while(frag != nullptr) {
            serializationLength += 1; // bitmap fragment
            frag = frag->next;
        }
    }

    return serializationLength;
}

auto operator<<(Serializer& serializer, DSMEGACKBitmap& gack) -> Serializer& {
    if(serializer.getType() == SERIALIZATION) {
        uint8_t numAddresses = gack.bitmap.size();
        serializer << numAddresses;

        for(auto bitmapIterator = gack.bitmap.begin(); bitmapIterator != gack.bitmap.end(); ++bitmapIterator) {
            IEEE802154MacAddress& addr = bitmapIterator.node()->getKey();
            uint16_t short_addr = addr.getShortAddress();
            serializer << short_addr;

            uint8_t numBitmapFragments = 0;
            DSMEGACKBitmapFragment *frag = *bitmapIterator;
            while(frag != nullptr) {
                numBitmapFragments += 1;
                frag = frag->getNext();
            }
            serializer << numBitmapFragments;

            frag = *bitmapIterator;
            uint8_t sequenceNumber = frag->getSequenceNumber();
            serializer << sequenceNumber;

            frag = *bitmapIterator;
            while(frag != nullptr) {
                uint8_t bits = frag->getBits();
                serializer << bits;
                frag = frag->getNext();
            }

            //DIRTY
            uint8_t size = gack.getNumberOfPackets(addr);
            for(uint8_t i=0; i<size; i++) {
                gack.getNextSequenceNumber(addr); //throwaway
            }
        }
    } else { // DESERIALIZE
        uint8_t numAdresses = 0;
        serializer << numAdresses;

        for(uint8_t i=0; i<numAdresses; i++) {
            uint16_t short_addr;
            serializer << short_addr;
            IEEE802154MacAddress addr(short_addr);

            uint8_t numBitmapFragments = 0;
            serializer << numBitmapFragments;

            uint8_t sequenceNumber = 0;
            serializer << sequenceNumber;

            for(uint8_t fragment=0; fragment<numBitmapFragments; fragment++) {
                uint8_t bits = 0;
                serializer << bits;

                if(fragment == 0) {
                    /* Insert first fragment */
                    gack.bitmap.insert(new DSMEGACKBitmapFragment(sequenceNumber, bits), addr);
                } else {
                    /* Append another fragment */
                    auto bitmapIterator = gack.bitmap.find(addr);
                    DSME_ASSERT(bitmapIterator != gack.bitmap.end());
                    DSMEGACKBitmapFragment *frag = *bitmapIterator;

                    /* skip to last fragment */
                    while(frag->getNext() != nullptr) frag = frag->getNext();

                    frag->setNext(new DSMEGACKBitmapFragment(frag->getLastSequenceNumber()+1, bits));
                }
            }
        }
    }

    return serializer;
}

auto DSMEGACKBitmap::reset() -> void {
    for(auto bitmapIterator = bitmap.begin(); bitmapIterator != bitmap.end(); ++bitmapIterator) {
        DSMEGACKBitmapFragment *bitmapFragment = *bitmapIterator;
        DSMEGACKBitmapFragment *nextBitmapFragment = bitmapFragment->next;

        while(nextBitmapFragment != nullptr) {
            delete bitmapFragment;
            bitmapFragment = nextBitmapFragment;
            nextBitmapFragment = bitmapFragment->next;
        }
        delete bitmapFragment;
        bitmap.remove(bitmapIterator);
    }
}

} /* namespace dsme */
