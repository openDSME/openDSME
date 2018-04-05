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

#ifndef DSMEEVENTDISPATCHER_H_
#define DSMEEVENTDISPATCHER_H_

#include "../helper/Integers.h"
#include "../interfaces/IDSMEPlatform.h"
#include "./TimerAbstractions.h"
#include "./TimerMultiplexer.h"

namespace dsme {

class DSMELayer;
class IDSMEPlatform;

enum EventTimers {
    NEXT_PRE_SLOT,
    NEXT_SLOT,
    CSMA_TIMER,
    ACK_TIMER,
    TIMER_COUNT /* always last element */
};

class DSMEEventDispatcher;

typedef TimerMultiplexer<EventTimers, DSMEEventDispatcher, IDSMEPlatform, IDSMEPlatform> DSMETimerMultiplexer;

class DSMEEventDispatcher : private DSMETimerMultiplexer {
public:
    explicit DSMEEventDispatcher(DSMELayer& dsme);
    void initialize();
    void reset();

    void timerInterrupt();

    uint32_t setupSlotTimer(uint32_t lastSlotTime, uint8_t skippedSlots);
    void setupCSMATimer(uint32_t absSymCnt);
    void setupACKTimer();
    void stopACKTimer();

private:
    DSMELayer& dsme;

    void firePreSlotTimer(int32_t lateness);
    void fireSlotTimer(int32_t lateness);
    void fireCSMATimer(int32_t lateness);
    void fireACKTimer(int32_t lateness);

    ReadonlyTimerAbstraction<IDSMEPlatform> NOW;
    WriteonlyTimerAbstraction<IDSMEPlatform> TIMER;

#ifdef STATISTICS_MONITOR_LATENESS
public:
    void printLatenessHistogram() {
        for(uint8_t i = 0; i < EventTimers::TIMER_COUNT; ++i) {
            LOG_ERROR_PREFIX;
            LOG_ERROR_PURE(static_cast<uint16_t>(i) << ": ");
            for(uint16_t j = 0; j < BIN_COUNT; ++j) {
                LOG_ERROR_PURE(this->lateness_histogram[i][j] << ",");
            }
            LOG_ERROR_PURE(LOG_ENDL);
        }
        return;
    }
#endif
};

} /* namespace dsme */

#endif /* DSMEEVENTDISPATCHER_H_ */
