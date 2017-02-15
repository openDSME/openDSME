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

#ifndef DSMERINGBUFFER_H_
#define DSMERINGBUFFER_H_

#include <stdint.h>
#include "../helper/DSMEAtomic.h"

namespace dsme {

typedef uint8_t ringbuffer_size_t;

template <typename T, ringbuffer_size_t N>
class DSMERingBuffer {
private:
    T buffer[N];
    ringbuffer_size_t head;
    ringbuffer_size_t tail;

    ringbuffer_size_t next(ringbuffer_size_t current) {
        return (current + 1) % N;
    }

public:
    DSMERingBuffer() : buffer{}, head(0), tail(0) {
    }

    virtual ~DSMERingBuffer() {}; // = default;

    bool hasCurrent() {
        bool result;
        DSME_ATOMIC_BLOCK {
            result = this->tail != this->head;
        }
        return result;
    }

    bool hasNext() {
        bool result;
        DSME_ATOMIC_BLOCK {
            result = next(this->tail) != this->head;
        }
        return result;
    }

    T* current() {
        T* result;
        DSME_ATOMIC_BLOCK {
            result = &(this->buffer[this->head]);
        }
        return result;
    }

    T* next() {
        T* result;
        DSME_ATOMIC_BLOCK {
            result = &(this->buffer[this->tail]);
        }
        return result;
    }

    void advanceCurrent() {
        DSME_ATOMIC_BLOCK {
            this->head = next(this->head);
        }
        return;
    }

    void advanceNext() {
        DSME_ATOMIC_BLOCK {
            this->tail = next(this->tail);
        }
        return;
    }
};

} /* namespace dsme */

#endif /* DSMERINGBUFFER_H_ */
