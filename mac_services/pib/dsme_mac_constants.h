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

#ifndef DSME_MAC_CONSTANTS_H_
#define DSME_MAC_CONSTANTS_H_

#include "dsme_phy_constants.h"

namespace dsme {

/**
 * This file contains all MAC constants as defined in IEEE 802.15.4-2015, 8.4.1, Table 8-80
 */

/** The number of symbols forming a superframe slot when the superframe order is equal to zero, as described in 6.2.1. */
constexpr uint8_t aBaseSlotDuration{60};

/** The number of slots contained in any superframe. */
constexpr uint8_t aNumSuperframeSlots{16};

/** The number of symbols forming a superframe when the superframe order is equal to zero. */
constexpr uint16_t aBaseSuperframeDuration{aBaseSlotDuration * aNumSuperframeSlots};

/** The number of superframes in which a GTS descriptor exists in the beacon frame of the PAN coordinator. */
constexpr uint8_t aGtsDescPersistenceTime{4};

/** The number of consecutive lost beacons that will cause the MAC sublayer of a receiving device to declare a loss of synchronization. */
constexpr uint8_t aMaxLostBeacons{4};

/** The maximum size of an MPDU, in octets, that can be followed by a SIFS period. */
constexpr uint8_t aMaxSIFSFrameSize{18};

/** The minimum number of symbols forming the CAP. This ensures that MAC commands can still be transferred to devices when GTSs are being used.
 * An exception to this minimum shall be allowed for the accommodation of the temporary increase in the beacon frame length needed to perform
 * GTS maintenance, as described in 7.3.1.4. Additional restrictions apply when PCA is enabled, as described in 6.2.5.4. */
constexpr uint16_t aMinCAPLength{440};

/** The number of symbols forming the basic time period used by the CSMA-CA algorithm. */
constexpr uint8_t aUnitBackoffPeriod{aTurnaroundTime + aCcaTime};

/** The number of symbols forming an RCCN superframe slot. */
constexpr uint8_t aRccnBaseSlotDuration{60};

} /* namespace dsme */

#endif /* DSME_MAC_CONSTANTS_H_ */
