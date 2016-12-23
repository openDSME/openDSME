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

#ifndef TIMERABSTRACTIONS_H
#define TIMERABSTRACTIONS_H

#include <stdint.h>

namespace dsme {

template <typename T>
class ReadonlyTimerAbstraction {
public:
    typedef uint32_t((T::*getter_t)());

    ReadonlyTimerAbstraction() : getter_instance(nullptr), getter(nullptr) {
    }

    void initialize(T* instance, getter_t getter) {
        this->getter_instance = instance;
        this->getter = getter;
    }

    inline operator uint32_t() const volatile {
        return (getter_instance->*getter)();
    }

private:
    T* getter_instance;
    getter_t getter;
};

template <typename T>
class WriteonlyTimerAbstraction {
public:
    typedef void (T::*setter_t)(uint32_t);

    WriteonlyTimerAbstraction() : setter_instance(nullptr), setter(nullptr) {
    }

    void initialize(T* instance, setter_t setter) {
        setter_instance = instance;
        this->setter = setter;
    }

    inline void operator=(uint32_t value) volatile {
        (setter_instance->*setter)(value);
        return;
    }

private:
    T* setter_instance;
    setter_t setter;
};

} /* dsme */

#endif /* TIMERABSTRACTIONS_H */
