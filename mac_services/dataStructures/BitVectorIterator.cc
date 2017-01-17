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

#include "DSMEBitVector.h"

namespace dsme {

BitVectorIterator::BitVectorIterator(BitVectorBase* instance, bit_vector_size_t position, bool value) : instance(instance), position(position), value(value) {
}

BitVectorIterator::BitVectorIterator(const BitVectorIterator& other) : instance(other.instance), position(other.position), value(other.value) {
}

BitVectorIterator::BitVectorIterator(BitVectorIterator&& other) : instance(other.instance), position(other.position), value(other.value) {
    other.instance = nullptr;
}

BitVectorIterator& BitVectorIterator::operator=(const BitVectorIterator& other) {
    this->instance = other.instance;
    this->position = other.position;
    this->value = other.value;
    return *this;
}

BitVectorIterator& BitVectorIterator::operator=(BitVectorIterator&& other) {
    this->instance = other.instance;
    this->position = other.position;
    this->value = other.value;

    other.instance = nullptr;

    return *this;
}

BitVectorIterator& BitVectorIterator::operator++() {
    uint16_t max = instance->bitSize;
    if(this->position == max) {
        return *this;
    }
    this->position++;
    for(; this->position != max && instance->get(this->position) != value; this->position++) {
    }
    return *this;
}

BitVectorIterator BitVectorIterator::operator++(int) {
    BitVectorIterator old = *this;
    ++(*this);
    return old;
}

uint16_t BitVectorIterator::operator*() const {
    return this->position;
}

bool operator==(const BitVectorIterator& lhs, const BitVectorIterator& rhs) {
    return (lhs.instance == rhs.instance && lhs.position == rhs.position && lhs.value == rhs.value);
}

bool operator!=(const BitVectorIterator& lhs, const BitVectorIterator& rhs) {
    return !(lhs == rhs);
}
}
