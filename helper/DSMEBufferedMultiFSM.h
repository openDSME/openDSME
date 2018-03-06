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

#ifndef DSMEBUFFEREDMULTIFSM_H_
#define DSMEBUFFEREDMULTIFSM_H_

#include "./DSMEFSM.h"
#include "./DSMERingbuffer.h"
#include "./Integers.h"

namespace dsme {

class MultiFSMEvent : public FSMEvent {
public:
    MultiFSMEvent() : fsmId(-1) {
    }

    int8_t getFsmId() {
        DSME_ASSERT(fsmId >= 0);
        return fsmId;
    }

    void setFsmId(int8_t fsmId) {
        this->fsmId = fsmId;
        return;
    }

private:
    int8_t fsmId;
};

/**
 * Template for implementing a finite state machine (FSM).
 */
template <typename C, typename E, uint8_t N, ringbuffer_size_t S>
class DSMEBufferedMultiFSM {
public:
    typedef fsmReturnStatus (C::*state_t)(E& event);

    /**
     * Created FSM is put into initial state
     */
    explicit DSMEBufferedMultiFSM(const state_t& initial, const state_t& busy) : dispatchBusy(false) {
        for(uint8_t i = 0; i < N; i++) {
            states[i] = initial;
        }
        states[N] = busy;
    }

    template <typename... Args>
    bool dispatch(int8_t fsmId, uint16_t signal, Args&... args) {
        DSME_ASSERT(fsmId >= 0 && fsmId <= N);

        bool canAdd;
        bool isBusy;
        bool returnValue;

        DSME_ATOMIC_BLOCK {
            canAdd = !this->eventBuffer.isFull();
            DSME_ASSERT(canAdd); // TODO: Remove after testing? Should never trigger when S is chosen correctly.

            if(canAdd) {
                this->eventBuffer.freeElement()->setFsmId(fsmId);
                this->eventBuffer.freeElement()->fill(signal, args...);
                this->eventBuffer.pushFreeElement();
            }

            isBusy = this->dispatchBusy;
            if(isBusy) {
                returnValue = canAdd;
            } else {
                DSME_ASSERT(canAdd);
                returnValue = true;
                this->dispatchBusy = true;
            }
        }

        if(!isBusy) {
            runUntilFinished();
        }

        return returnValue;
    }

    inline fsmReturnStatus transition(int8_t fsmId, state_t next) {
        DSME_ASSERT(fsmId >= 0 && fsmId < N);

        DSME_ATOMIC_BLOCK {
            this->states[fsmId] = next;
        }
        return FSM_TRANSITION;
    }

    const state_t& getState(int8_t fsmId) const {
        DSME_ASSERT(fsmId >= 0 && fsmId < N);

        return states[fsmId];
    }

private:
    void runUntilFinished() {
        while(!eventBuffer.isEmpty()) {
            E* currentEvent = this->eventBuffer.front();

            int8_t fsmId = currentEvent->getFsmId();
            state_t state = states[fsmId];

            state_t s = state;
            fsmReturnStatus r = (((C*)this)->*state)(*currentEvent);

            while(r == FSM_TRANSITION) {
                /* call the exit action from last state, reuse the already processed 'currentEvent' to deliver this */
                currentEvent->signal = E::EXIT_SIGNAL;
                r = (((C*)this)->*s)(*currentEvent);
                DSME_ASSERT(r != FSM_TRANSITION);

                state = states[fsmId];
                s = state;

                /* call entry action of new state, reuse the already processed 'currentEvent' to deliver this */
                currentEvent->signal = E::ENTRY_SIGNAL;
                r = (((C*)this)->*state)(*currentEvent);
            }
            this->eventBuffer.pop();
        }
        dispatchBusy = false;
        return;
    }

    state_t states[N + 1];
    bool dispatchBusy;
    DSMERingBuffer<E, S> eventBuffer;
};

} /* namespace dsme */

#endif /* DSMEBUFFEREDMULTIFSM_H_ */
