/*
 * openDSME
 *
 * Implementation of the Deterministic & Synchronous Multi-channel Extension (DSME)
 * described in the IEEE 802.15.4-2015 standard
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

#include "DSMEEventDispatcher.h"
#include "DSMELayer.h"
#include "../../dsme_platform.h"

namespace dsme {

DSMEEventDispatcher::DSMEEventDispatcher(DSMELayer& dsme) :
        dsme(dsme) {
    for (uint8_t i = 0; i < TIMER_COUNT; i++) {
        symsUntil[i] = -1;
    }
}

void DSMEEventDispatcher::initialize() {
    lastSymCnt = dsme.getPlatform().getSymbolCounter();
}

void DSMEEventDispatcher::timerInterrupt() {
    dispatchEvents();
    setupTimer();
}

void DSMEEventDispatcher::dispatchEvents() {
    uint32_t currentSymCnt = dsme.getPlatform().getSymbolCounter();

    // The difference also works if there was a wrap around since lastSymCnt (modulo by casting to uint32_t).
    // Since the timer fires at least once every superframe, the results fits into a int16_t.
    int16_t symsSinceLastDispatch = (uint32_t) (currentSymCnt - lastSymCnt);

    // TODO dispatch outside of IRQ context (e.g. TinyOS Tasklet concept)
    // this should also hold for events not related to the timer (e.g. CCA)

    // TODO reduce the number of timers, e.g. for less time critical timers check only in (pre-) slot events

    // Execute events
    if (0 < symsUntil[NEXT_PRE_SLOT] && symsUntil[NEXT_PRE_SLOT] <= symsSinceLastDispatch) {
        dsme.preSlotEvent();
    }

    if (0 < symsUntil[NEXT_SLOT] && symsUntil[NEXT_SLOT] <= symsSinceLastDispatch) {
        dsme.slotEvent();
    }

    if (0 < symsUntil[CSMA_TIMER] && symsUntil[CSMA_TIMER] <= symsSinceLastDispatch) {
        dsme.getMessageDispatcher().dispatchCSMATimerEvent();
    }

    if (0 < symsUntil[ACK_TIMER] && symsUntil[ACK_TIMER] <= symsSinceLastDispatch) {
        dsme.getAckLayer().dispatchTimer();
    }

    if (0 < symsUntil[SCAN_TIMER] && symsUntil[SCAN_TIMER] <= symsSinceLastDispatch) {
        dsme.getBeaconManager().dispatchScanTimer();
    }

    // Decrease times
    for (uint8_t i = 0; i < TIMER_COUNT; i++) {
        if (symsUntil[i] > 0) {
            symsUntil[i] -= symsSinceLastDispatch;
        }
    }

    lastSymCnt = currentSymCnt;
}

void DSMEEventDispatcher::setupTimer() {
    int16_t symsUntilNextEvent = dsme.getMAC_PIB().helper.getSymbolsPerSlot() * aNumSuperframeSlots;

    for (uint8_t i = 0; i < TIMER_COUNT; i++) {
        if (0 < symsUntil[i] && symsUntil[i] < symsUntilNextEvent) {
            symsUntilNextEvent = symsUntil[i];
        }

    }

    dsme.getPlatform().startTimer(lastSymCnt + symsUntilNextEvent);
}

void DSMEEventDispatcher::setupSlotTimer(uint32_t lastHeardBeaconSymbolCounter, uint16_t slotsSinceLastHeardBeacon) {
    symsUntil[NEXT_SLOT] = (slotsSinceLastHeardBeacon + 1) * dsme.getMAC_PIB().helper.getSymbolsPerSlot()
            - ((uint32_t) (lastSymCnt - lastHeardBeaconSymbolCounter));
    symsUntil[NEXT_PRE_SLOT] = symsUntil[NEXT_SLOT] - PRE_EVENT_SHIFT;

    setupTimer();
}

void DSMEEventDispatcher::setupCSMATimer(uint32_t absSymCnt) {
    symsUntil[CSMA_TIMER] = absSymCnt - lastSymCnt;

    setupTimer();
}

void DSMEEventDispatcher::setupACKTimer(uint32_t symbols) {
    symsUntil[ACK_TIMER] = dsme.getPlatform().getSymbolCounter() + symbols - lastSymCnt;

    setupTimer();
}

void DSMEEventDispatcher::stopACKTimer() {
    symsUntil[ACK_TIMER] = -1;

    setupTimer();
}

void DSMEEventDispatcher::setupScanTimer(uint32_t symbols) {
    symsUntil[SCAN_TIMER] = dsme.getPlatform().getSymbolCounter() + symbols - lastSymCnt;

    setupTimer();
}

void DSMEEventDispatcher::stopScanTimer() {
    symsUntil[SCAN_TIMER] = -1;

    setupTimer();
}

}
