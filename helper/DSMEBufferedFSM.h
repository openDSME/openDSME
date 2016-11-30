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

#ifndef DSMEBUFFEREDFSM_H
#define DSMEBUFFEREDFSM_H

#include <stdint.h>
#include "DSMEFSM.h"
#include "DSMERingbuffer.h"

namespace dsme {

/**
 * Template for implementing a finite state machine (FSM).
 */
template<typename C, typename E, ringbuffer_size_t S>
class DSMEBufferedFSM {
public:
	typedef fsmReturnStatus (C::*state_t)(E& event);

	/**
	 * Created FSM is put into initial state
	 */
	DSMEBufferedFSM(const state_t& initial) :
			state(initial), dispatchBusy(false) {
	}

	template <typename ...Args>
	int dispatch(uint16_t signal, Args & ... args)
	{
	    bool canAdd;
	    bool isBusy;
	    bool returnValue;

	    dsme_atomicBegin();

	    canAdd = this->eventBuffer.hasNext();
        DSME_ASSERT(canAdd); // TODO: Remove after testing? Should never trigger when S is chosen correctly.

        if(canAdd) {
            this->eventBuffer.next()->fill(signal, args...);
            this->eventBuffer.advanceNext();
        }

        isBusy = this->dispatchBusy;
        if (isBusy) {
            returnValue = canAdd;
        } else {
            DSME_ASSERT(canAdd);
            returnValue = true;
            this->dispatchBusy = true;
        }

        dsme_atomicEnd();

        if(!isBusy) {
            runUntilFinished();
        }

        return returnValue;
	}

	inline fsmReturnStatus transition(state_t next) {
        dsme_atomicBegin();
		state = next;
        dsme_atomicEnd();
        return FSM_TRANSITION;
	}

	const state_t& getState() {
		return state;
	}

private:
	void runUntilFinished() {
        while (eventBuffer.hasCurrent()) {
            E *currentEvent = this->eventBuffer.current();

            state_t s = state;
            fsmReturnStatus r = (((C*) this)->*state)(*currentEvent);

            while (r == FSM_TRANSITION) {
                /* call the exit action from last state, reuse the already processed 'currentEvent' to deliver this */
                currentEvent->signal = E::EXIT_SIGNAL;
                r = (((C* )this)->*s)(*currentEvent);
                DSME_ASSERT(r != FSM_TRANSITION);

                s = state;

                /* call entry action of new state, reuse the already processed 'currentEvent' to deliver this */
                currentEvent->signal = E::ENTRY_SIGNAL;
                r = (((C*) this)->*state)(*currentEvent);
            }
            this->eventBuffer.advanceCurrent();
        }
        dispatchBusy = false;
        return;
	}


	state_t state;
	bool dispatchBusy;
	DSMERingBuffer<E,S> eventBuffer;
};

} /* dsme */

#endif /* DSMEBUFFEREDFSM_H */
