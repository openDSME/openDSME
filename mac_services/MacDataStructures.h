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

#ifndef MACDATASTRUCTURES_H_
#define MACDATASTRUCTURES_H_

#include <stdint.h>

namespace dsme {

template<typename TK, typename TV>
class MacTuple {
public:
    MacTuple(const TK& key, const TV& value) :
        key(key),
        value(value) {
    }

    TK key;
    TV value;
};

template<typename T, uint8_t S>
class MacStaticList {
public:
    MacStaticList() :
        length(0),
        array{} {
    }

    explicit MacStaticList(const MacStaticList& other) :
        length(other.length) {
        for(uint8_t i = 0; i < length; i++) {
            this->array[i] = other.array[i];
        }
    }

    MacStaticList& operator=(const MacStaticList& other) {
        this->length = other.length;
        for(uint8_t i = 0; i < length; i++) {
            this->array[i] = other.array[i];
        }
        return *this;
    }

    explicit MacStaticList(uint8_t length) :
        length(length) {
    }

    ~MacStaticList() {
    }

    uint8_t getLength() const {
        return this->length;
    }

    void setLength(uint8_t length) {
        this->length = length;
    }

    const T& operator[](uint8_t pos) const {
        return this->array[pos];
    }

    T& operator[](uint8_t pos) {
        return this->array[pos];
    }

    void add(const T& element) {
        if(this->length < S) {
            this->array[length++] = element;
        }
    }

    bool contains(const T& element) {
        for(uint8_t i = 0; i < length; i++) {
            if(this->array[i] == element) {
                return true;
            }
        }
        return false;
    }

    void clear() {
        this->length = 0;
    }

    uint8_t size() const {
        return this->length;
    }

    uint8_t full() const {
        return (this->length == S);
    }

private:
    uint8_t length;
    T array[S];
};

}
/* dsme */

#endif /* MACDATASTRUCTURES_H_ */
