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

#include "DSMELayer.h"

#include "../../dsme_platform.h"
#include "../../DSMEPlatform.h"
#include "../helper/DSMEAtomic.h"
#include "messages/IEEE802154eMACHeader.h"
#include "messages/MACCommand.h"

extern dsme::DSMEPlatform m_dsme;

namespace dsme {

typedef Delegate<void(IDSMEMessage* msg)> receive_delegate_t;

DSMELayer::DSMELayer()
    : phy_pib(nullptr),
      mac_pib(nullptr),
      mcps_sap(nullptr),
      mlme_sap(nullptr),

      platform(nullptr),
      eventDispatcher(*this),

      ackLayer(*this),
      capLayer(*this),
      associationManager(*this),
      beaconManager(*this),
      gtsManager(*this),
      messageDispatcher(*this),

      currentSlot(0),
      currentSuperframe(0),
      currentMultiSuperframe(0),
      nextSlot(0),
      nextSuperframe(0),
      nextMultiSuperframe(0),
      trackingBeacons(false),
      nextSlotTime(0),
      resetPending(false) {
}

void DSMELayer::initialize(IDSMEPlatform* platform) {
    this->platform = platform;
    LOG_INFO("platfrom points to " << HEXOUT << platform << DECOUT << LOG_ENDL);
    LOG_INFO("m_dsme points to " << HEXOUT << &m_dsme << DECOUT << LOG_ENDL);
    receive_delegate_t recDel = DELEGATE(&MessageDispatcher::receive, this->messageDispatcher);
    asm volatile("bkpt 1");
    ((DSMEPlatform*) this->platform)->bla();
    m_dsme.setReceiveDelegate(recDel);
    DSMEPlatform* plat = dynamic_cast<DSMEPlatform*>(this->platform);
    LOG_INFO("plat points to " << HEXOUT << plat << DECOUT << LOG_ENDL);
    plat->setReceiveDelegate(recDel);


    this->currentSlot = 0;
    this->currentSuperframe = 0;
    this->currentMultiSuperframe = 0;

    this->eventDispatcher.initialize();
    this->gtsManager.initialize();
    this->messageDispatcher.initialize();
    this->beaconManager.initialize();

    this->mac_pib->macDsn = platform->getRandom();
}

void DSMELayer::start() {
    if(getMAC_PIB().macIsPANCoord) {
        getMAC_PIB().macIsCoord = true;
    }

    /* start the timer initially */
    this->nextSlotTime = this->eventDispatcher.setupSlotTimer(this->platform->getSymbolCounter());
}

void DSMELayer::reset() {
    DSME_ASSERT(!resetPending);
    resetPending = true;
}

void DSMELayer::doReset() {
    DSME_ASSERT(resetPending);
    LOG_WARN("Performing a complete reset of the DSME MLME.");

    DSME_ATOMIC_BLOCK {
        this->ackLayer.reset();

        /* stop all timers */
        this->eventDispatcher.reset();

        this->beaconManager.reset();
        this->associationManager.reset();
        this->gtsManager.reset();
        this->messageDispatcher.reset();

        this->capLayer.reset();

        this->mac_pib->macDsn = platform->getRandom();

        this->currentSlot = 0;
        this->currentSuperframe = 0;
        this->currentMultiSuperframe = 0;

        this->trackingBeacons = false;
    }

    /* restart slot timer */
    this->nextSlotTime = this->eventDispatcher.setupSlotTimer(this->platform->getSymbolCounter());

    resetPending = false;
}

void DSMELayer::preSlotEvent(void) {
    if(resetPending) {
        doReset();
        return;
    }

    // calculate time within next slot
    uint32_t cnt = platform->getSymbolCounter() - beaconManager.getLastKnownBeaconIntervalStart() + PRE_EVENT_SHIFT + 1;

    // calculate slot position
    uint16_t slotsSinceLastKnownBeaconIntervalStart = cnt / getMAC_PIB().helper.getSymbolsPerSlot();
    nextSlot = slotsSinceLastKnownBeaconIntervalStart % aNumSuperframeSlots;
    uint16_t superframe = slotsSinceLastKnownBeaconIntervalStart / aNumSuperframeSlots;
    nextSuperframe = superframe % getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe();
    superframe /= getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe();
    nextMultiSuperframe = superframe % getMAC_PIB().helper.getNumberMultiSuperframesPerBeaconInterval();

    if(nextSlot == 0) {
        beaconManager.preSuperframeEvent(nextSuperframe, nextMultiSuperframe, nextSlotTime);
    }

    messageDispatcher.handlePreSlotEvent(nextSlot, nextSuperframe);
}

void DSMELayer::slotEvent(int32_t lateness) {
    if(resetPending) {
        doReset();
        return;
    }

    currentSlot = nextSlot;
    currentSuperframe = nextSuperframe;
    currentMultiSuperframe = nextMultiSuperframe;

    if(getMAC_PIB().macShortAddress == 1) {
        LOG_DEBUG(DECOUT << currentSlot << " " << currentSuperframe << " " << currentMultiSuperframe);
    }

    if(lateness > 100) { // TODO reduce
        LOG_ERROR("lateness " << lateness);
        DSME_ASSERT(false);
    }

    uint32_t currentSlotTime;

    // TODO set timer to next relevant slot only!
    // TODO in that case currentSlot might be used even if no slotEvent was called before -> calculate then
    if(this->trackingBeacons) {
        auto now = platform->getSymbolCounter();
        currentSlotTime = now - (now - beaconManager.getLastKnownBeaconIntervalStart()) % getMAC_PIB().helper.getSymbolsPerSlot();
    } else {
        currentSlotTime = this->nextSlotTime;
    }

    this->nextSlotTime = eventDispatcher.setupSlotTimer(currentSlotTime);

    /* handle slot */
    if(currentSlot == 0) {
        beaconManager.superframeEvent(lateness, currentSlotTime);
    }

    messageDispatcher.handleSlotEvent(currentSlot, currentSuperframe, lateness);

    if(currentSlot == getMAC_PIB().helper.getFinalCAPSlot() + 1) {
        platform->scheduleStartOfCFP();
    }
}

void DSMELayer::handleStartOfCFP() {
#ifdef STATISTICS_MONITOR_LATENESS
    if(latenessStatisticsCount++ % 10 == 0) {
        this->eventDispatcher.printLatenessHistogram();
    }
#endif

    if(this->startOfCFPDelegate) {
        this->startOfCFPDelegate();
    }

    this->gtsManager.handleStartOfCFP(this->currentSuperframe);
    this->associationManager.handleStartOfCFP(this->currentSuperframe);
    this->beaconManager.handleStartOfCFP(this->currentSuperframe, this->currentMultiSuperframe);
}

uint16_t DSMELayer::getSymbolsSinceSuperframeStart(uint32_t time, uint16_t shift) {
    // Add one superframe to account for times where the shifted time is larger than the current time
    return (time - (beaconManager.getLastKnownBeaconIntervalStart() + shift) + aNumSuperframeSlots * getMAC_PIB().helper.getSymbolsPerSlot()) %
           (aNumSuperframeSlots * getMAC_PIB().helper.getSymbolsPerSlot());
}

bool DSMELayer::isWithinCAP(uint32_t time, uint16_t duration) {
    uint32_t symbolsPerSlot = getMAC_PIB().helper.getSymbolsPerSlot();
    uint16_t symbolsSinceSuperframeStart = getSymbolsSinceSuperframeStart(time, 0);
    uint16_t capStart = symbolsPerSlot; // after beacon slot
    uint32_t capEnd = symbolsPerSlot * (getMAC_PIB().helper.getFinalCAPSlot() + 1) - PRE_EVENT_SHIFT;
    // LOG_INFO(capStart << " " << symbolsSinceSuperframeStart << " " << duration << " " << capEnd);
    return (symbolsSinceSuperframeStart >= capStart)              // after beacon slot
           && (symbolsSinceSuperframeStart + duration <= capEnd); // before pre-event of first GTS

    // Unrealistic, but helpful example for
    //   symbolsPerSlot  = 2
    //   finalCAPSlot    = 3
    //   PRE_EVENT_SHIFT = 1
    //   duration        = 2
    //             B   C   C   C   G
    // Slot ID:    0   1   2   3   4
    // Symbol IDs: 0-1 2-3 4-5 6-7 8-9
    //
    // 0: (0 >= 2) && (0+2 <= 2*4-1) -> not ok
    // 1: (1 >= 2) && (1+2 <= 2*4-1) -> not ok
    // 2: (2 >= 2) && (2+2 <= 2*4-1) -> ok
    // 3: (3 >= 2) && (3+2 <= 2*4-1) -> ok
    // 4: (4 >= 2) && (4+2 <= 2*4-1) -> ok
    // 5: (5 >= 2) && (5+2 <= 2*4-1) -> ok
    // 6: (6 >= 2) && (6+2 <= 2*4-1) -> not ok
    // 7: (7 >= 2) && (7+2 <= 2*4-1) -> not ok
    // 8: (8 >= 2) && (8+2 <= 2*4-1) -> not ok
}

void DSMELayer::startTrackingBeacons() {
    this->trackingBeacons = true;
    return;
}

void DSMELayer::stopTrackingBeacons() {
    this->trackingBeacons = false;
    return;
}

bool DSMELayer::isTrackingBeacons() const {
    return this->trackingBeacons;
}

} /* namespace dsme */
