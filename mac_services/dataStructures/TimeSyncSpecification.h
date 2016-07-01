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

#ifndef TIMESYNCSPECIFICATION_H
#define TIMESYNCSPECIFICATION_H

namespace dsme {

struct TimeSyncSpecification
{
public:
    void setBeaconTimestampMicroSeconds(uint64_t beaconTimestamp) {
        this->beaconTimestamp = beaconTimestamp;
    }

    void setBeaconOffsetTimestampMicroSeconds(uint16_t beaconOffsetTimestamp) {
        this->beaconOffsetTimestamp = beaconOffsetTimestamp;
    }

    uint64_t getBeaconTimestampMicroSeconds() {
        return beaconTimestamp;
    }

    uint16_t getBeaconOffsetTimestampMicroSeconds() {
        return beaconOffsetTimestamp;
    }

    friend Serializer& operator<<(Serializer& serializer, TimeSyncSpecification& spec);

private:
    union {
        uint64_t beaconTimestamp; // in microseconds, only 6 bytes are used
        uint8_t beaconTimestampBytes[8];
    };
    uint16_t beaconOffsetTimestamp; // in microseconds


};

inline Serializer& operator<<(Serializer& serializer, TimeSyncSpecification& spec) {
    serializer << spec.beaconTimestampBytes[5];
    serializer << spec.beaconTimestampBytes[4];
    serializer << spec.beaconTimestampBytes[3];
    serializer << spec.beaconTimestampBytes[2];
    serializer << spec.beaconTimestampBytes[1];
    serializer << spec.beaconTimestampBytes[0];

    serializer << spec.beaconOffsetTimestamp;
    return serializer;
}

}

#endif
