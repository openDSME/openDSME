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

#define DSME_ALLOCATION_COUNTER_TABLE

#include "./DSMEAllocationCounterTable.h"

#include "../../../dsme_platform.h"
#include "../../dsmeLayer/DSMELayer.h"
#include "../../helper/Integers.h"
#include "../DSME_Common.h"
#include "../pib/MAC_PIB.h"
#include "./ACTElement.h"
#include "./BitVectorIterator.h"
#include "./DSMEBitVector.h"
#include "./DSMESABSpecification.h"
#include "./GTS.h"
#include "./RBTree.h"

using namespace dsme;

DSMEAllocationCounterTable::DSMEAllocationCounterTable()
    : numSuperFramesPerMultiSuperframe(0), numGTSlotsFirstSuperframe(0), numGTSlotsLatterSuperframes(0), numChannels(0) {
}

void DSMEAllocationCounterTable::initialize(uint16_t numSuperFramesPerMultiSuperframe, uint8_t numGTSlotsFirstSuperframe, uint8_t numGTSlotsLatterSuperframes,
                                            uint8_t numChannels, DSMELayer* dsme) {
    this->numSuperFramesPerMultiSuperframe = numSuperFramesPerMultiSuperframe;
    this->numGTSlotsFirstSuperframe = numGTSlotsFirstSuperframe;
    this->numGTSlotsLatterSuperframes = numGTSlotsLatterSuperframes;
    this->numChannels = numChannels;
    bitmap.initialize((numGTSlotsFirstSuperframe + (numSuperFramesPerMultiSuperframe - 1) * numGTSlotsLatterSuperframes), false);
    this->dsme = dsme;
}

uint16_t DSMEAllocationCounterTable::getBitmapPosition(uint8_t superframeID, uint8_t slotID) const {
    if(superframeID == 0) {
        return slotID;
    } else {
        return (numGTSlotsFirstSuperframe + (superframeID - 1) * numGTSlotsLatterSuperframes) + slotID;
    }
}

DSMEAllocationCounterTable::iterator DSMEAllocationCounterTable::begin() {
    return act.begin();
}

DSMEAllocationCounterTable::iterator DSMEAllocationCounterTable::end() {
    return act.end();
}

void DSMEAllocationCounterTable::clear() {
    while(this->act.size() != 0) {
        auto it = this->act.begin();
        this->act.remove(it);
    }

    for(int i = 0; i < 2; i++) {
        while(this->numAllocatedSlots[i].size() != 0) {
            auto it = this->numAllocatedSlots[i].begin();
            this->numAllocatedSlots[i].remove(it);
        }
    }

    this->bitmap.fill(false);
}

DSMEAllocationCounterTable::iterator DSMEAllocationCounterTable::find(uint16_t superframeID, uint8_t gtSlotID) {
    ACTPosition pos;
    pos.superframeID = superframeID;
    pos.gtSlotID = gtSlotID;
    return act.find(pos);
}

void DSMEAllocationCounterTable::printChange(const char* type, uint16_t superframeID, uint8_t gtSlotID, uint8_t channel, bool direction, uint16_t address) {
    LOG_INFO_PREFIX;
    LOG_INFO_PURE(DECOUT << type << " " << palId_id());
    if(direction == TX) {
        LOG_INFO_PURE(">");
    } else {
        LOG_INFO_PURE("<");
    }
    LOG_INFO_PURE(address << " " << (uint16_t)(gtSlotID + 9) << "," << superframeID << "," << (uint16_t)channel << LOG_ENDL);
}

bool DSMEAllocationCounterTable::add(uint16_t superframeID, uint8_t gtSlotID, uint8_t channel, Direction direction, uint16_t address, ACTState state) {
    if(state == ACTState::REMOVED) {
        LOG_DEBUG("Slot to be REMOVED is not in ACT -> Do nothing.");
        // DSME_ASSERT(false);
        return true;
    }
    printChange("alloc", superframeID, gtSlotID, channel, direction, address);

    if(isAllocated(superframeID, gtSlotID)) {
        DSME_ASSERT(false);
    }
    DSME_ASSERT(!isAllocated(superframeID, gtSlotID));

    uint8_t numSlots = 1;

    bool success = false;
    for (uint8_t i=0; i<numSlots ; i++){
        ACTPosition pos;
        pos.superframeID = superframeID;
        pos.gtSlotID = gtSlotID;



        success = act.insert(ACTElement(superframeID, gtSlotID, channel, direction, address, state), pos);

        if(success) {

            this->dsme->getPlatform().signalGTSChange(false, IEEE802154MacAddress(address));

            int d = (direction == TX) ? 0 : 1;
            RBTree<float, uint16_t>::iterator numSlotIt = numAllocatedSlots[d].find(address);
            if(numSlotIt == numAllocatedSlots[d].end()) {
                LOG_DEBUG("Inserting 0x" << HEXOUT << address << DECOUT << " into numAllocatedSlots[" << d << ".");

                //IAMG Proof of concept capOn capOff. idea -> in case of CAP GTS the number of allocated GTS will be one for two CAP GTS.
                if(
                        //((superframeID !=0) && (gtSlotID < 8) && (gtSlotID % 2 == 0)) ||
                        ((superframeID ==0) && (gtSlotID < 7)) || ((superframeID !=0) && (gtSlotID > 7) && (gtSlotID < 15))){
                    //IAMG
                    LOG_DEBUG("IAMG. Adding 0x" << HEXOUT << address << DECOUT << " into numAllocatedSlots[" << d << ".");
                    numAllocatedSlots[d].insert(1, address);
                }else  if((superframeID !=0) && (gtSlotID < 8)){
                    //IAMG
                    LOG_DEBUG("IAMG. Adding 0x" << HEXOUT << address << DECOUT << " into numAllocatedSlots[" << d << ".");
                    //numAllocatedSlots[d].insert(0.5, address);
                    numAllocatedSlots[d].insert(1, address);
                }


            } else {

                //IAMG Proof of concept capOn capOff. idea -> in case of CAP GTS the number of allocated GTS will be one for two CAP GTS.
                if(
                        //((superframeID !=0) && (gtSlotID < 8) && (gtSlotID % 2 == 0)) ||

                        ((superframeID ==0) && (gtSlotID < 7)) || ((superframeID !=0) && (gtSlotID > 7) && (gtSlotID < 15))){
                    LOG_DEBUG("IAMG. count in RBTree already exist. Adding 0x" << HEXOUT << address << DECOUT << " into numAllocatedSlots[" << d << ".");
                    (*numSlotIt)++;
                }else  if((superframeID !=0) && (gtSlotID < 8)){
                    //IAMG
                    LOG_DEBUG("IAMG. Adding 0x" << HEXOUT << address << DECOUT << " into numAllocatedSlots[" << d << ".");
                    //(*numSlotIt) = (*numSlotIt) + 0.5;
                    (*numSlotIt) = (*numSlotIt) + 1;
                }


                LOG_DEBUG("Incrementing slot count " << d << HEXOUT << " for 0x" << address << DECOUT << " (now at " << *numSlotIt << ").");
            }

            bitmap.set(getBitmapPosition(superframeID, gtSlotID), true);
        }


    }
    return success;
}

void DSMEAllocationCounterTable::remove(DSMEAllocationCounterTable::iterator it) {
    DSME_ASSERT(it != act.end());

    uint16_t superframeID = it->getSuperframeID();
    uint8_t gtSlotID = it->getGTSlotID();

    printChange("dealloc", it->superframeID, it->slotID, it->channel, it->direction, it->address);

    this->dsme->getPlatform().signalGTSChange(true, IEEE802154MacAddress(it->address));

    DSME_ASSERT(isAllocated(superframeID, gtSlotID));

    bitmap.set(getBitmapPosition(superframeID, gtSlotID), false);



    int d = (it->direction == TX) ? 0 : 1;
    RBTree<float, uint16_t>::iterator numSlotIt = numAllocatedSlots[d].find(it->address);

    if (numSlotIt != numAllocatedSlots[d].end()){
    DSME_ASSERT(numSlotIt != numAllocatedSlots[d].end());
    }
    //IAMG Proof of concept capOn capOff. idea -> in case of CAP GTS the number of allocated GTS will be one for two CAP GTS.
    if((superframeID ==0) && (gtSlotID < 7)){
        (*numSlotIt)--;
    }else if ((superframeID !=0) && (gtSlotID > 7) && (gtSlotID < 15)){
        (*numSlotIt)--;
    }else if ((superframeID !=0) && (gtSlotID < 8)){
        (*numSlotIt)--;
/*        if(gtSlotID % 2 == 0){
            //if (!isAllocated(superframeID, ((gtSlotID+1)%8))){ // first version from capOncapOff
                //(*numSlotIt)--;
            //}
            LOG_DEBUG("IAMG: VALUE numSlot in remove when deallocation of CAP slot -0.5 ");
            //DSME_ASSERT(false);
            (*numSlotIt) = (*numSlotIt) - 0.5;
        }else if (gtSlotID % 2 != 0){
            if (!isAllocated(superframeID, ((gtSlotID-1)%8))){
                (*numSlotIt)--;
            } // first version from capOncapOff
            (*numSlotIt) = (*numSlotIt) - 0.5;
        }*/
    }


    LOG_DEBUG("Decrementing slot count for " << it->address << DECOUT << " (now at " << *numSlotIt << ").");
    if((*numSlotIt) == 0) {
        numAllocatedSlots[d].remove(numSlotIt);
    }

    act.remove(it);
}

bool DSMEAllocationCounterTable::isAllocated(uint16_t superframeID, uint8_t gtSlotID) const {
    //DSME_ASSERT(gtSlotID < dsme->getMAC_PIB().helper.getNumGTSlots(superframeID));
    return bitmap.get(getBitmapPosition(superframeID, gtSlotID));
}

uint16_t DSMEAllocationCounterTable::getNumAllocatedGTS(uint16_t address, Direction direction) {
    int d = (direction == TX) ? 0 : 1;
    RBTree<float, uint16_t>::iterator numSlotIt = numAllocatedSlots[d].find(address);
    if(numSlotIt == numAllocatedSlots[d].end()) {
        return 0;
    } else {
        return *numSlotIt;
    }
}

void DSMEAllocationCounterTable::setACTStateIfExists(DSMESABSpecification& subBlock, ACTState state, uint16_t channelOffset) {
    Direction ignoredDirection = TX;
    setACTState(subBlock, state, ignoredDirection, 0xFFFF, channelOffset, false);
}

static const char* stateToString(ACTState state) {
    switch(state) {
        case VALID:
            return "VALID";
        case UNCONFIRMED:
            return "UNCONFIRMED";
        case INVALID:
            return "INVALID";
        case DEALLOCATED:
            return "DEALLOCATED";
        case REMOVED:
            return "REMOVED";
        default:
            return "UNKNOWN STATE";
    };
}

void DSMEAllocationCounterTable::setACTState(DSMESABSpecification& subBlock, ACTState state, Direction direction, uint16_t deviceAddress,
                                             uint16_t channelOffset, bool useChannelOffset, bool checkAddress) {
    setACTState(subBlock, state, direction, deviceAddress, channelOffset, useChannelOffset, [](ACTElement e) { return true; }, checkAddress);
}

void DSMEAllocationCounterTable::setACTState(DSMESABSpecification& subBlock, ACTState state, Direction direction, uint16_t deviceAddress,
                                             uint16_t channelOffset, bool useChannelOffset, condition_t condition, bool checkAddress) {
    // Supporting more than one slot allocation induces many open issues and is probably not needed most of the time.
    bit_vector_size_t count = 0;
    count = subBlock.getSubBlock().count(true);

    if(subBlock.getSubBlock().count(true) < 1) {
        return;
    }
    LOG_DEBUG("IAMG. Inside DSMEALlocationCounterTable. setACTState. retrieve the count of sbublock: "<< count);
    //DSME_ASSERT(subBlock.getSubBlock().count(true) < 3); //IAMG CAP ON CAP OFF. IDea to trigger assert if count (i.e. number of slots > 3)

    for(DSMESABSpecification::SABSubBlock::iterator it = subBlock.getSubBlock().beginSetBits(); it != subBlock.getSubBlock().endSetBits(); ++it) {
        // this calculation assumes there is always exactly one superframe in the subblock
        GTS gts(subBlock.getSubBlockIndex(), (*it) / numChannels, (*it) % numChannels);

        LOG_DEBUG("search slot " << (uint16_t)gts.slotID << " " << (uint16_t)gts.superframeID << " (" << (uint16_t)gts.channel << ")");
        DSMEAllocationCounterTable::iterator actit = find(gts.superframeID, gts.slotID);

        if(actit != end()) {
            if((!useChannelOffset &&
                (actit->getChannel() != gts.channel))) { // For channel hopping the channel is irrelevant because the channel offset is saved in the act
                LOG_DEBUG("Request too late, GTS"
                          << "used on other channel (useChannelOffset: " << useChannelOffset << ")");
                // DSME_ASSERT(false);
                continue;
            }
            if(checkAddress && actit->getAddress() != deviceAddress) {
                LOG_DEBUG("Request too late, GTS "
                          << "used towards other device");
                continue;
            }
            if(!condition(*actit)) {
                LOG_DEBUG("Request too late, GTS "
                          << "does not fulfill condition");
                continue;
            }
        }

        if(actit != end() && state == REMOVED) {
            remove(actit);
            continue;
        }

        if(actit == end()) {
            /* '-> does not yet exist */
            if(deviceAddress != 0xFFFF) {
                uint16_t channel = useChannelOffset ? channelOffset : gts.channel;
                LOG_DEBUG("ch " << channelOffset << " " << gts.channel);
                bool added = add(gts.superframeID, gts.slotID, channel, direction, deviceAddress, state);
                DSME_ASSERT(added);
                LOG_DEBUG("add slot " << (uint16_t)gts.slotID << " " << (uint16_t)gts.superframeID << " " << channel << " as " << stateToString(state)
                                      << " useChannelOffset: " << useChannelOffset << " nDirection: " << direction);
            } else {
                /* setACTStateIfExists(...) was called */
            }
        } else {
            LOG_DEBUG("set slot " << (uint16_t)actit->getGTSlotID() << " " << (uint16_t)actit->getSuperframeID() << " " << (uint16_t)actit->getChannel()
                                  << " to " << stateToString(state));
            actit->setState(state);
        }
    }
}
