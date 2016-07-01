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

#include "IEEE802154MacAddress.h"

namespace dsme {

const IEEE802154MacAddress IEEE802154MacAddress::UNSPECIFIED(0, 0, 0, 0);
const uint16_t IEEE802154MacAddress::SHORT_BROADCAST_ADDRESS(0xffff);
const uint16_t IEEE802154MacAddress::NO_SHORT_ADDRESS(0xfffe);

IEEE802154MacAddress::IEEE802154MacAddress(const IEEE802154MacAddress& other) :
        addr { other.a1(), other.a2(), other.a3(), other.a4() } {
}

IEEE802154MacAddress::IEEE802154MacAddress() {
    *this = UNSPECIFIED;
}

IEEE802154MacAddress::IEEE802154MacAddress(uint16_t shortPart) :
        addr { 0, 0x00ff, 0xfe00, shortPart } {
}

IEEE802154MacAddress::IEEE802154MacAddress(uint16_t a1, uint16_t a2,
        uint16_t a3, uint16_t a4) :
        addr { a1, a2, a3, a4 } {
}

IEEE802154MacAddress::IEEE802154MacAddress(const uint16_t* a) :
        addr { a[0], a[1], a[2], a[3] } {
}

bool IEEE802154MacAddress::operator<=(const IEEE802154MacAddress& other) const {
    if (this->addr[0] > other.addr[0])
        return false;
    if (this->addr[0] < other.addr[0])
        return true;

    if (this->addr[1] > other.addr[1])
        return false;
    if (this->addr[1] < other.addr[1])
        return true;

    if (this->addr[2] > other.addr[2])
        return false;
    if (this->addr[2] < other.addr[2])
        return true;

    if (this->addr[3] > other.addr[3])
        return false;
    if (this->addr[3] < other.addr[3])
        return true;

    return true;
}

bool IEEE802154MacAddress::operator>=(const IEEE802154MacAddress& other) const {
    if (this->addr[0] > other.addr[0])
        return true;
    if (this->addr[0] < other.addr[0])
        return false;

    if (this->addr[1] > other.addr[1])
        return true;
    if (this->addr[1] < other.addr[1])
        return false;

    if (this->addr[2] > other.addr[2])
        return true;
    if (this->addr[2] < other.addr[2])
        return false;

    if (this->addr[3] > other.addr[3])
        return true;
    if (this->addr[3] < other.addr[3])
        return false;

    return true;
}

bool IEEE802154MacAddress::operator>(const IEEE802154MacAddress& other) const {
    return !((*this) <= other);
}

bool IEEE802154MacAddress::operator<(const IEEE802154MacAddress& other) const {
    return !((*this) >= other);
}

bool IEEE802154MacAddress::operator==(const IEEE802154MacAddress& other) const {
    return (this->a1() == other.a1() && this->a2() == other.a2()
            && this->a3() == other.a3() && this->a4() == other.a4());
}

bool IEEE802154MacAddress::operator!=(const IEEE802154MacAddress& other) const {
    return !((*this) == other);
}

}

