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

#include "./Math.h"

#include <math.h>

#include "../../helper/Integers.h"

namespace dsme {

namespace quicknet {

void quick_linear(vector_t& vector) {
    return;
}

void quick_sigmoid(vector_t& vector) {
    for(uint8_t j = 0; j < vector.length(); j++) {
        vector(j) = (1.0 + (tanh(vector(j) / 2.0))) / 2.0;
    }
}

void quick_softmax(vector_t& vector) {
    weight_t sum = 0;
    for(uint8_t j = 0; j < vector.length(); j++) {
        vector(j) = exp(vector(j));
        sum += vector(j);
    }

    for(uint8_t j = 0; j < vector.length(); j++) {
        vector(j) = vector(j) / sum;
    }
    return;
}

void quick_tanh(vector_t& vector) {
    for(uint8_t j = 0; j < vector.length(); j++) {
        vector(j) = tanh(vector(j));
    }
    return;
}

uint8_t idmax(const vector_t& vector) {
    uint8_t max_index = 0;
    weight_t max_score = 0.0;

    for(uint8_t i = 0; i < vector.length(); i++) {
        if(vector(i) > max_score) {
            max_index = i;
            max_score = vector(i);
        }
    }
    return max_index;
}

} /* namespace quicknet */

} /* namespace dsme */
