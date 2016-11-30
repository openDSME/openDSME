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

#ifndef TIMERMULTIPLEXER_H
#define TIMERMULTIPLEXER_H

#include <stdint.h>
#include "EventHistory.h"
#include "TimerAbstractions.h"
#include "dsme_platform.h"

namespace dsme {

template<typename T, typename R, typename G, typename S>
class TimerMultiplexer {
protected:
    typedef T timer_t;
    typedef void (R::*handler_t)(int32_t lateness);

    TimerMultiplexer(R* instance, ReadonlyTimerAbstraction<G> &now, WriteonlyTimerAbstraction<S> &timer) :
            instance(instance),
            _NOW(now),
            _TIMER(timer) {
        for (uint8_t i = 0; i < timer_t::TIMER_COUNT; ++i) {
            this->symbols_until[i] = -1;
            this->handlers[i] = nullptr;
        }
    }

    void _initialize() {
        this->lastDispatchSymbolCounter = _NOW;
    }

    void _reset() {
        this->lastDispatchSymbolCounter = _NOW;
        for (uint8_t i = 0; i < timer_t::TIMER_COUNT; ++i) {
            this->symbols_until[i] = -1;
            this->handlers[i] = nullptr;
        }

        _TIMER = _NOW - 1;
        return;
    }

    void _timerInterrupt() {
        dsme_atomicBegin();

        dispatchEvents();
        _scheduleTimer();

        dsme_atomicEnd();
    }

    template<T E>
    inline void _startTimer(uint32_t nextEventSymbolCounter, handler_t handler) {
        this->history.addEvent(nextEventSymbolCounter, E);

        if (nextEventSymbolCounter <= this->lastDispatchSymbolCounter) {
            /* '-> an event was scheduled too far in the past */
            uint32_t now = _NOW;
            LOG_ERROR("now:" << now
                    << ", nextEvent: "<< nextEventSymbolCounter
                    << ", lastDispatch: " << this->lastDispatchSymbolCounter
                    << ", Event " << (uint16_t)E);
            history.printEvents();
            DSME_ASSERT(false);
        }

        this->symbols_until[E] = nextEventSymbolCounter - this->lastDispatchSymbolCounter;
        this->handlers[E] = handler;
        return;
    }

    template<T E>
    inline void _stopTimer() {
        this->symbols_until[E] = -1;
        return;
    }

    void _scheduleTimer() {
        uint32_t symsUntilNextEvent = -1;

        for (uint8_t i = 0; i < timer_t::TIMER_COUNT; ++i) {
            if (0 < this->symbols_until[i] && static_cast<uint32_t>(this->symbols_until[i]) < symsUntilNextEvent) {
                symsUntilNextEvent = symbols_until[i];
            }
        }

        if(symsUntilNextEvent == -1) {
            return;
        }

        uint32_t timer = this->lastDispatchSymbolCounter + symsUntilNextEvent;

        uint32_t currentSymCnt = _NOW;
        if (timer < currentSymCnt + 2) {
            timer = currentSymCnt + 2;
        }
        _TIMER = timer;

        uint32_t now = _NOW;
        if (timer <= now) {
            LOG_ERROR("now: " << now << " timer: " << timer);
            DSME_ASSERT(false);
        }

        return;
    }

private:
    void dispatchEvents() {
        uint32_t currentSymbolCounter = _NOW;

        /* The difference also works if there was a wrap around since lastSymCnt (modulo by casting to uint32_t). */
        int32_t symbolsSinceLastDispatch = (uint32_t) (currentSymbolCounter - this->lastDispatchSymbolCounter);

        for (uint8_t i = 0; i < timer_t::TIMER_COUNT; ++i) {
            if (0 < this->symbols_until[i] && this->symbols_until[i] <= symbolsSinceLastDispatch) {
                int32_t lateness = symbolsSinceLastDispatch - this->symbols_until[i];
                DSME_ASSERT(this->handlers[i] != nullptr);
                (this->instance->*(this->handlers[i]))(lateness);
            }
        }

        for (uint8_t i = 0; i < timer_t::TIMER_COUNT; ++i) {
            if (this->symbols_until[i] > 0) {
                this->symbols_until[i] -= symbolsSinceLastDispatch;
            }
        }
        this->lastDispatchSymbolCounter = currentSymbolCounter;
        return;
    }

private:
    /**
     * The timestamp in symbols of the last call to dispatchEvents();
     */
    uint32_t lastDispatchSymbolCounter;

    /**
     * Values >  0: timer is activated
     * Values <= 0: timer has expired or is deactivated
     */
    int32_t symbols_until[timer_t::TIMER_COUNT];

    /**
     * Stores handles to methods of a subclass that get called once their associated timer expires
     */
    handler_t handlers[timer_t::TIMER_COUNT];

    /**
     * Handle to the instance of the TimerMultiplexer as the subclass which implements the handlers
     */
    R* instance;

    /**
     * Reading this as uint32_t returns the current system's symbol counter
     */
    ReadonlyTimerAbstraction<G> &_NOW;

    /**
     * Writing an uint32_t to this schedules a timer at the given symbol counter
     */
    WriteonlyTimerAbstraction<S> &_TIMER;

    /**
     * For debuging only, records the last scheduled events
     */
    EventHistory<T, 8> history;
};

} /* dsme */

#endif /* TIMERMULTIPLEXER_H */
