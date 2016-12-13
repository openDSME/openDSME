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

#ifndef IEEE802514MACADDRESS_H
#define IEEE802514MACADDRESS_H

#include <stdint.h>
#include "Serializer.h"

namespace dsme {

class IEEE802154MacAddress {
public:
    IEEE802154MacAddress(const IEEE802154MacAddress& other);

    explicit IEEE802154MacAddress();

    explicit IEEE802154MacAddress(uint16_t shortPart);

    explicit IEEE802154MacAddress(const uint16_t* a);

    explicit IEEE802154MacAddress(uint16_t a1, uint16_t a2, uint16_t a3, uint16_t a4);

    static const IEEE802154MacAddress UNSPECIFIED;
    static const uint16_t SHORT_BROADCAST_ADDRESS; // there is no extended broadcast address (IEEE 802.15.4-2011 5.1.6.2)
    static const uint16_t NO_SHORT_ADDRESS;

    uint16_t a1() const {
        return addr[0];
    }
    uint16_t a2() const {
        return addr[1];
    }
    uint16_t a3() const {
        return addr[2];
    }
    uint16_t a4() const {
        return addr[3];
    }

    void setA1(uint16_t a) {
        addr[0] = a;
    }
    void setA2(uint16_t a) {
        addr[1] = a;
    }
    void setA3(uint16_t a) {
        addr[2] = a;
    }
    void setA4(uint16_t a) {
        addr[3] = a;
    }

    IEEE802154MacAddress& operator=(IEEE802154MacAddress const& other) {
        addr[0] = other.a1();
        addr[1] = other.a2();
        addr[2] = other.a3();
        addr[3] = other.a4();
        return *this;
    }

    bool operator<=(const IEEE802154MacAddress& other) const;

    bool operator>=(const IEEE802154MacAddress& other) const;

    bool operator>(const IEEE802154MacAddress& other) const;

    bool operator<(const IEEE802154MacAddress& other) const;

    bool operator==(const IEEE802154MacAddress& other) const;

    bool operator!=(const IEEE802154MacAddress& other) const;

    bool isUnspecified() const {
        return (*this) == UNSPECIFIED;
    }

    bool isBroadcast() const {
        return (*this) == IEEE802154MacAddress(SHORT_BROADCAST_ADDRESS);
    }

    // TODO should be determined via association
    void setShortAddress(uint16_t shortAddr) {
        (*this) = IEEE802154MacAddress(shortAddr);
    }

    // TODO should be determined via association
    uint16_t getShortAddress() const {
        return addr[3];
    }

    uint64_t getExtendedAdress() const = delete;

    friend Serializer& operator<<(Serializer& serializer, IEEE802154MacAddress& addr);

    friend uint8_t* operator<<(uint8_t*& buffer, const IEEE802154MacAddress& addr);
    friend const uint8_t* operator>>(const uint8_t*& buffer, IEEE802154MacAddress& addr);

private:
    uint16_t addr[4];
};

inline Serializer& operator<<(Serializer& serializer, IEEE802154MacAddress& addr) {
    serializer << addr.addr[0];
    serializer << addr.addr[1];
    serializer << addr.addr[2];
    serializer << addr.addr[3];
    return serializer;
}

/* NEW FAST SERIALISATION **************************************************************/

inline uint8_t* operator<<(uint8_t*& buffer, const IEEE802154MacAddress& addr) {
    *(buffer++) = addr.addr[0] & 0xFF;
    *(buffer++) = addr.addr[0] >> 8;
    *(buffer++) = addr.addr[1] & 0xFF;
    *(buffer++) = addr.addr[1] >> 8;
    *(buffer++) = addr.addr[2] & 0xFF;
    *(buffer++) = addr.addr[2] >> 8;
    *(buffer++) = addr.addr[3] & 0xFF;
    *(buffer++) = addr.addr[3] >> 8;

    return buffer;
}

inline const uint8_t* operator>>(const uint8_t*& buffer, IEEE802154MacAddress& addr) {
    addr.addr[0] = *(buffer) | (*(buffer + 1) << 8);
    buffer += 2;

    addr.addr[1] = *(buffer) | (*(buffer + 1) << 8);
    buffer += 2;

    addr.addr[2] = *(buffer) | (*(buffer + 1) << 8);
    buffer += 2;

    addr.addr[3] = *(buffer) | (*(buffer + 1) << 8);
    buffer += 2;

    return buffer;
}

}

#endif
