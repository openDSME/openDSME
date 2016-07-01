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

#ifndef DSME_FSM_H
#define DSME_FSM_H

#include <stdint.h>
#include "../../dsme_platform.h"

namespace dsme {

class FSMEvent {
public:
    enum : uint8_t {
        EMPTY_SIGNAL = 0,
        ENTRY_SIGNAL,
        EXIT_SIGNAL,
        USER_SIGNAL_START
    };

    FSMEvent(uint8_t signal)
    : signal(signal) {
    }

    FSMEvent()
    : signal(EMPTY_SIGNAL) {
    }

    uint8_t signal;
};

typedef enum : uint8_t {
    FSM_HANDLED,    //< event was handled
    FSM_IGNORED,    //< event was ignored; not used in this implementation
    FSM_TRANSITION, //< event was handled and a state transition occurred
} fsmReturnStatus;

/**
 * Template for implementing a finite state machine (FSM).
 */
template<typename C, typename E>
class FSM {
public:
	typedef fsmReturnStatus (C::*state_t)(E& event);

	/**
	 * Created FSM is put into initial state
	 */
	FSM(const state_t& initial) :
			state(initial), dispatchBusy(false) {
	}

    bool dispatch(E& event) {
        bool wasBusy;
        dsme_atomicBegin();
        wasBusy = dispatchBusy;
        dispatchBusy = true;
        dsme_atomicEnd();

        if(wasBusy) {
            return false;
        }

        E entryEvent;
        entryEvent.signal = FSMEvent::ENTRY_SIGNAL;
        E exitEvent;
        exitEvent.signal = FSMEvent::EXIT_SIGNAL;

        state_t s = state;
        fsmReturnStatus r = (((C*)this)->*state)(event);

        while (r == FSM_TRANSITION) {
            /* call the exit action from last state */
            ASSERT((((C*)this)->*s)(exitEvent) != FSM_TRANSITION);
            s = state;

            /* call entry action of new state */
            r = (((C*)this)->*state)(entryEvent);
        }

        dispatchBusy = false;
        return true;
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
	state_t state;
	bool dispatchBusy;
};


}
#endif
