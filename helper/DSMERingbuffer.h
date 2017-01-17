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

#ifndef DSMERINGBUFFER_H
#define DSMERINGBUFFER_H

#include <stdint.h>
#include "../../dsme_atomic.h"

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

    virtual ~DSMERingBuffer() = default;

    bool hasCurrent() {
        bool result;
        dsme_atomicBegin();
        result = this->tail != this->head;
        dsme_atomicEnd();
        return result;
    }

    bool hasNext() {
        bool result;
        dsme_atomicBegin();
        result = next(this->tail) != this->head;
        dsme_atomicEnd();
        return result;
    }

    T* current() {
        T* result;
        dsme_atomicBegin();
        result = &(this->buffer[this->head]);
        dsme_atomicEnd();
        return result;
    }

    T* next() {
        T* result;
        dsme_atomicBegin();
        result = &(this->buffer[this->tail]);
        dsme_atomicEnd();
        return result;
    }

    void advanceCurrent() {
        dsme_atomicBegin();
        this->head = next(this->head);
        dsme_atomicEnd();
        return;
    }

    void advanceNext() {
        dsme_atomicBegin();
        this->tail = next(this->tail);
        dsme_atomicEnd();
        return;
    }
};

} /* dsme */

#endif /* DSMERINGBUFFER_H */
