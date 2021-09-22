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

#include "./DAS.h"

#include <cmath>
#include "../../../dsme_platform.h"
#include "../../dsmeLayer/DSMELayer.h"
#include "../../mac_services/dataStructures/IEEE802154MacAddress.h"
#include "../../mac_services/pib/MAC_PIB.h"
#include "../DSMEAdaptionLayer.h"

static bool header = false;

namespace dsme {

DASTxData::DASTxData() : pastQValues{0},  messagesInLastMultisuperframe{0}, incomingPacketsHistory{0}, avgIn{0},dynamicSlots{0},counter{0} {
}


void DAS::setUseMultiplePacketsPerGTS(bool useMultiplePackets) {
    this->useMultiplePacketsPerGTS = useMultiplePackets;
}

void DAS::multisuperframeEvent() {
    for(DASTxData& data : this->txLinks) {
        NeighborQueue<MAX_NEIGHBORS>& queue = dsmeAdaptionLayer.getDSME().getMessageDispatcher().getNeighborQueue();
        NeighborQueue<MAX_NEIGHBORS>::iterator neighbor = queue.findByAddress(IEEE802154MacAddress(data.address));
        data.incomingPacketsHistory[data.counter%5] = data.messagesInLastMultisuperframe;
        data.pastQValues[data.counter%3] = queue.getPacketsInQueue(neighbor);
        data.avgIn = (data.incomingPacketsHistory[0]+data.incomingPacketsHistory[1]+data.incomingPacketsHistory[2]+data.incomingPacketsHistory[3]+data.incomingPacketsHistory[4])/5.0;
        if(data.avgIn <= 0){ data.avgIn = 1;}
        if((data.pastQValues[data.counter%3] > data.pastQValues[(data.counter-2)%3]) &&
           (data.pastQValues[data.counter%3]-(data.dynamicSlots + this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(data.address, Direction::TX))) > 0) {
               data.dynamicSlots++;
               }
        if(this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(data.address, Direction::TX) > (data.avgIn + data.dynamicSlots) && data.dynamicSlots != 0) {
               data.dynamicSlots--;
        }
        if(data.pastQValues[data.counter%3] < ( 2 + data.pastQValues[(data.counter-2)%3])){data.dynamicSlots--;}
        if(data.avgIn + 4 < data.dynamicSlots) {data.dynamicSlots-=2;}
        if(data.dynamicSlots < 0) {data.dynamicSlots = 0;}
        //uint8_t slots = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(data.address, Direction::TX);

        // LengthFrameInSymbols = Preamble +SFD + PHR + PSDU (PHYPayload)
        // Preamble = 8 symbols;
        // SFD = 2 symbols;
        // PHR = 2 symbols;
        // PSDU = MHR + MACPayload + MFR;
        uint8_t packets_per_slot = 1;
        if(useMultiplePacketsPerGTS) {
            packets_per_slot = (this->dsmeAdaptionLayer.getMAC_PIB().helper.getSymbolsPerSlot() - PRE_EVENT_SHIFT) /
                               ((6 + 127) * 2 + this->dsmeAdaptionLayer.getMAC_PIB().helper.getAckWaitDuration() + const_redefines::macLIFSPeriod);
            /* '-> calculate number of packets per slot with assumption of maximum packet size and maximum acknowledgement wait duration -> THIS CAN BE DONE
             * MUCH BETTER */
        }

        LOG_DEBUG("Packets per slot: " << (int)packets_per_slot);
        // TODO
        data.slotTarget = ceilf(data.avgIn + data.dynamicSlots); // + change;
        LOG_DEBUG("DAS target: " << data.slotTarget);
        LOG_DEBUG("DAS avgIN: " << data.avgIn);
        LOG_DEBUG("DAS dynamic: " << data.dynamicSlots);
        LOG_DEBUG("QSize: " << (int)queue.getPacketsInQueue(neighbor));
        LOG_DEBUG("QSize akutell: " << data.pastQValues[data.counter%3]);
        LOG_DEBUG("QSize vor 1: " << data.pastQValues[(data.counter-1)%3]);
        LOG_DEBUG("QSize vor 2: " << data.pastQValues[(data.counter-2)%3]);
        LOG_DEBUG("DAS counter: " << data.counter);
        LOG_DEBUG("Currently allocated: " << (int)(this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(data.address, Direction::TX)));
        data.messagesInLastMultisuperframe = 0;
        data.messagesOutLastMultisuperframe = 0;
        data.counter++;
        if(data.counter < 0) {data.counter = 0;}
    }

}

GTSSchedulingDecision DAS::getNextSchedulingAction(uint16_t address) {
    uint16_t numAllocatedSlots = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(address, Direction::TX);
    int16_t target = getSlotTarget(address);
    if(target > numAllocatedSlots) {
        uint8_t numSuperFramesPerMultiSuperframe = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe();
        uint8_t targetSF = (this->dsmeAdaptionLayer.getDSME().getCurrentSuperframe()+1) % numSuperFramesPerMultiSuperframe;
        uint8_t numGTSlots = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(targetSF);
        DSMESlotAllocationBitmap& macDSMESAB = this->dsmeAdaptionLayer.getMAC_PIB().macDSMESAB;
        DSMEAllocationCounterTable& macDSMEACT = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT;
        uint8_t maxSlot = 14;
        uint8_t distance[8][16]{0};
        uint8_t timeslot = 0;
        distance[0][15] = 255;
        int outputTimeslot = 15;
        uint8_t offset = 0;
        uint8_t temp = 1;
        uint8_t outputSF = 0;
        uint8_t threshold = 4;

        for(; offset < numSuperFramesPerMultiSuperframe; offset++) { // f�r alle Superframes
            LOG_DEBUG("Versuche in superframe " << (int)((targetSF + offset)% numSuperFramesPerMultiSuperframe) << " frei paare zu finden");
            timeslot = 0;
            if((targetSF + offset) % numSuperFramesPerMultiSuperframe != 0) {
                timeslot = 8;
                maxSlot = 14;
            } else {
                timeslot = 0;
                maxSlot = 6;
            }
            for(; timeslot < maxSlot; timeslot++) { // Schreibt abst�nde zum n�chsten freien slot in array
                if(!macDSMEACT.isAllocated((targetSF + offset) % numSuperFramesPerMultiSuperframe, timeslot)) {
                    LOG_DEBUG("Slot " << (int)timeslot << " ist frei");
                    timeslot++;
                    while(macDSMEACT.isAllocated((targetSF + offset) % numSuperFramesPerMultiSuperframe, timeslot) &&
                          timeslot <= maxSlot) { // z�hlt Schritte zum n�chsten freien slot
                        temp++;
                        timeslot++;
                        if(timeslot >= maxSlot){break;}
                    }
                    LOG_DEBUG("Sein partner ist slot " << (int)timeslot << " mit distance von " << (int)(temp));
                    distance[offset][timeslot - temp] = temp;
                    if(timeslot >= maxSlot) {
                        distance[offset][timeslot - temp] = 0;
                    }
                    timeslot--;
                    temp = 1;
                } // ende if timeslot frei
            } // ende for timeslot
            for(uint8_t i = 0; i < maxSlot; i++) { // sucht kleinste Zahl aus distance array
                   if(distance[offset][i] > 0 && (distance[offset][i] < distance[outputSF][outputTimeslot])){
                          outputTimeslot = i;
                          outputSF = offset;
                          LOG_DEBUG("outputSF und slot " << (int)((targetSF + outputSF)%numSuperFramesPerMultiSuperframe) << "und " << (int)outputTimeslot);
                          LOG_DEBUG("Die distance hier ist " << (int)distance[outputSF][outputTimeslot]);
                   }
            }
        } // ende for offset
        if(distance[outputSF][outputTimeslot] < threshold) {
                LOG_DEBUG("gebe folgenden output: outputSF = " << (int)((targetSF + outputSF)%numSuperFramesPerMultiSuperframe) << " outputTimeslot = " << outputTimeslot);
               return GTSSchedulingDecision{address, ManagementType::ALLOCATION, Direction::TX, 1,
                   (uint8_t)((targetSF + outputSF)%numSuperFramesPerMultiSuperframe), (uint8_t)((outputTimeslot))};
        }

        if(distance[outputSF][outputTimeslot] >= threshold){
            LOG_DEBUG("distance zu hoch, versuche in cap reduction zu suchen");
            offset = 0;
            for(; offset < numSuperFramesPerMultiSuperframe; offset++) { // f�r alle Superframes
                        timeslot = 0;
                        if((targetSF + offset) % numSuperFramesPerMultiSuperframe != 0) {
                            maxSlot = 14;
                        } else {
                            timeslot = 0;
                            maxSlot = 6;
                        }
                        for(; timeslot < maxSlot; timeslot++) { // Schreibt abst�nde zum n�chsten freien slot in array
                            if(!macDSMEACT.isAllocated((targetSF + offset) % numSuperFramesPerMultiSuperframe, timeslot)) { // f�r alle freien Slots
                                timeslot++;
                                while(macDSMEACT.isAllocated((targetSF + offset) % numSuperFramesPerMultiSuperframe, timeslot) &&
                                      timeslot <= maxSlot ) { // z�hlt Schritte zum n�chsten freien slot
                                    temp++;
                                    timeslot++;
                                    if(timeslot >= maxSlot){break;}
                                }
                                distance[offset][timeslot - temp] = temp;
                                if(timeslot > maxSlot) {
                                    distance[offset][timeslot - temp] = 0;
                                }
                                timeslot--;
                                temp = 1;
                            } // ende if timeslot frei
                        } // ende for timeslot
                    } //ende for offset
            for(uint8_t i = 0; i < 15; i++) { // sucht kleinste Zahl aus distance array
                if((distance[offset][i] > 0) && (distance[offset][i] <= distance[outputSF][outputTimeslot])){
                    outputTimeslot = i;
                    outputSF = offset;
                }
            } // for finde lowest disntace
        } // ende if distanz > threshold


        if(distance[outputSF][outputTimeslot] == 255) {return NO_SCHEDULING_ACTION;}
        return GTSSchedulingDecision{address, ManagementType::ALLOCATION, Direction::TX, 1,(uint8_t)((targetSF + outputSF)%numSuperFramesPerMultiSuperframe),
            (uint8_t)outputTimeslot};
           // return GTSSchedulingDecision{address, ManagementType::ALLOCATION, Direction::TX, 1, randomSuperframeID, randomSlotID};
    } else if(target < numAllocatedSlots && numAllocatedSlots > 1) {
        /* TODO: slot and superframe ID are currently ignored for DEALLOCATION */
        return GTSSchedulingDecision{address, ManagementType::DEALLOCATION, Direction::TX, 1, 0, 0};
    } else {return NO_SCHEDULING_ACTION;}
} // ende funktion

GTSSchedulingDecision DAS::getNextSchedulingActionRx(uint8_t prefSF) {
    uint8_t numSuperFramesPerMultiSuperframe = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe();
    uint8_t currentSF = this->dsmeAdaptionLayer.getDSME().getCurrentSuperframe();
    uint8_t numGTSlots = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumGTSlots(prefSF);
    DSMESlotAllocationBitmap& macDSMESAB = this->dsmeAdaptionLayer.getMAC_PIB().macDSMESAB;
    uint8_t numChannels = this->dsmeAdaptionLayer.getMAC_PIB().helper.getNumChannels();
    DSMEAllocationCounterTable& macDSMEACT = this->dsmeAdaptionLayer.getMAC_PIB().macDSMEACT;
    //  auswahl superframe und GTS
    uint8_t threshold = 4;
    uint8_t distance[16]{0};
    distance[15] = 255;
    uint8_t temp = 1;
    uint8_t timeslot = 0;
    uint8_t outputTimeslot = 15;
    uint8_t maxSlot = 14;
    LOG_DEBUG("RX in mit superframe "<<(int)prefSF);

    if(prefSF != 0) {
        timeslot = 8;
        maxSlot = 14;
    } else {
        timeslot = 0;
        maxSlot = 6;
    }
    LOG_DEBUG("Versuche in superframe " << (int)prefSF << " freie paare zu finden maxslot ist also :" << (int)maxSlot << " " << (int)numGTSlots << " timeslot: " << (int)timeslot);

    for(; timeslot < maxSlot; timeslot++) {
        LOG_DEBUG("FOR Timeslot betreten");
        if(!macDSMEACT.isAllocated((prefSF) % numSuperFramesPerMultiSuperframe, timeslot)) {// f�r alle freien Slots
            LOG_DEBUG("RX Slot " << (int)timeslot << " ist frei");
            timeslot++;
            while(macDSMEACT.isAllocated(prefSF, timeslot) &&
                  timeslot <= maxSlot ) { // z�hlt Schritte zum n�chsten freien slot
                temp++;
                timeslot++;
                if(timeslot >= maxSlot){break;}
            }
            LOG_DEBUG("Sein partner ist slot " << (int)timeslot << " mit distance von " << (int)(temp));
            distance[timeslot - temp] = temp;
            LOG_DEBUG("RX Schreibe in distance[" << (int)(timeslot-temp) << "] den wert: "<< (int)temp);
            if(timeslot > maxSlot) {
                distance[timeslot - temp] = 0;
            }
            timeslot--;
            temp = 1;
        } // ende if timeslot frei
    } // ende for timeslot

    for(uint8_t i = 0; i < 15; i++) { // sucht kleinste Zahl aus distance array
        if(distance[i] > 0 && distance[i] < distance[outputTimeslot]){
            LOG_DEBUG("RX der erstbeste outputtimeslot = " << (int)i);
            LOG_DEBUG("er ist besser als " << (int)outputTimeslot);
            outputTimeslot = i;
        }
    }

    if(distance[outputTimeslot] >= threshold && prefSF != 0){
        LOG_DEBUG("RX distanz groesser als "<< (int)threshold << "aktiviere cap reduction");
            uint8_t timeslot = 0;
            if(prefSF != 0) {
                   timeslot = 0;
                   maxSlot = 14;
            } else {
                   timeslot = 0;
                   maxSlot = 6;
            }
            for(; timeslot < maxSlot; timeslot++) {                                                    // Schreibt abst�nde zum n�chsten freien slot in array
                   if(!macDSMEACT.isAllocated((prefSF) % numSuperFramesPerMultiSuperframe, timeslot)) { // f�r alle freien Slots
                          LOG_DEBUG("Slot " << (int)timeslot << " ist frei");
                          timeslot++;
                          while(macDSMEACT.isAllocated(prefSF, timeslot) &&
                                 timeslot <= maxSlot ) { // z�hlt Schritte zum n�chsten freien slot
                                     temp++;
                                     timeslot++;
                                     if(timeslot >= maxSlot){break;}
                           }
                           distance[timeslot - temp] = temp;
                           if(timeslot > maxSlot) {
                               distance[timeslot - temp] = 0;
                           }
                           LOG_DEBUG("Sein partner ist slot " << (int)timeslot << " mit distance von " << (int)(temp));
                           LOG_DEBUG("RX Schreibe in distance[" << (int)(timeslot-temp) << "] den wert: "<< (int)temp);
                           timeslot--;
                           temp = 1;
                   } // ende if timeslot frei
             } // ende for timeslot

             for(uint8_t i = 0; i < 8; i++) { // sucht kleinste Zahl aus distance array
                   if(distance[i] > 0 && distance[i] <= distance[outputTimeslot]){
                       LOG_DEBUG("RX CR der erstbeste outputtimeslot = " << (int)i);
                       LOG_DEBUG("er CR ist besser als " << (int)outputTimeslot);
                          outputTimeslot = i;
                   }
            }

      } // ende if distanz > treshold
    if(outputTimeslot >= 15) {return NO_SCHEDULING_ACTION;}
    LOG_DEBUG("RX returns decisions mit Timeslot "<< (int)outputTimeslot << "und SF " << (int)prefSF);
    return GTSSchedulingDecision{0,ManagementType::ALLOCATION, Direction::TX, 1, (uint8_t)(prefSF),(uint8_t)(outputTimeslot%numGTSlots)};
}// ende funktion

} /* namespace dsme */
