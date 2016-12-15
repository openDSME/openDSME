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

#ifndef MAC_PIB_H_
#define MAC_PIB_H_

#include "../dataStructures/DSMEAllocationCounterTable.h"
#include "../dataStructures/DSMESlotAllocationBitmap.h"
#include "../dataStructures/IEEE802154MacAddress.h"
#include "PHY_PIB.h"
#include "PIBHelper.h"

namespace dsme {

/*
 * This class contains MAC PIB attributes as defined in IEEE 802.15.4-2011, 6.4.2, Table 52
 * and IEEE 802.15.4e-2012, 6.4.2, Table 52 and 802.15.4e-2012, 6.4.3.6, Table 52h-i.
 */
class MAC_PIB {
public:
    explicit MAC_PIB(PHY_PIB&);

    MAC_PIB() = delete;
    MAC_PIB(const MAC_PIB&) = delete;
    MAC_PIB& operator=(const MAC_PIB&) = delete;

    void recalculateDependentProperties();

    PIBHelper helper;

    PHY_PIB& phy_pib;

    /* GENERAL PART FROM HERE ON ------------------------------------------> */

    /* The extended address assigned to the device. */
    IEEE802154MacAddress macExtendedAddress;

    /* The maximum number of symbols to wait for an acknowledgment frame to arrive following a transmitted data frame.
     * This value is dependent on the supported PHY, which determines both the selected channel and channel page.
     * The calculated value is the time to commence transmitting the ACK plus the length of the ACK frame.
     * The commencement time is described in 5.1.6.4.2.*/
    const uint16_t macAckWaitDuration;

    /* Indication of whether the device is the PAN coordinator.
     * TODO Not covered by the standard! */
    bool macIsPANCoord;

    /* Indication of whether the device is a coordinator.
     * TODO Not covered by the standard! */
    bool macIsCoord;

    /* Indication of whether the device is associated to the PAN through the PAN coordinator.
     * A value of TRUE indicates the device has associated through the PAN coordinator. Otherwise, the value is set to FALSE. */
    bool macAssociatedPANCoord;

    /* Indication of whether a coordinator is currently allowing association. A value of TRUE indicates that association is permitted. */
    bool macAssociationPermit;

    /* Indication of whether a device automatically sends a data request command if its address is listed in the beacon frame.
     * A value of TRUE indicates that the data request command is automatically sent. This attribute also affects
     * the generation of the MLME-BEACON-NOTIFY.indication primitive, as described in 6.2.4.1. */
    bool macAutoRequest;

    /* The address of the coordinator through which the device is associated. */
    IEEE802154MacAddress macCoordExtendedAddress;

    /* Indicates the frequency with which the beacon is transmitted, as defined in 5.1.1.1. */
    uint8_t macBeaconOrder;

    /* The short address assigned to the coordinator through which the device is associated. A value of 0xfffe indicates that the
     * coordinator is only using its extended address. A value of 0xffff indicates that this value is unknown. */
    uint16_t macCoordShortAddress;

    /* The sequence number added to the transmitted Data frame or MAC command. */
    uint8_t macDsn;

    /* The maximum value of the backoff exponent, BE, in the CSMA-CA algorithm, as defined in 5.1.1.4. */
    uint8_t macMaxBE;

    /* The maximum number of backoffs the CSMA-CA algorithm will attempt before declaring a channel access failure. */
    uint8_t macMaxCSMABackoffs;

    /* The maximum number of retries allowed after a transmission failure. */
    uint8_t macMaxFrameRetries;

    /* The minimum value of the backoff exponent (BE) in the CSMA-CA algorithm, as described in 5.1.1.4. */
    uint8_t macMinBE;

    /* The maximum time to wait either for a frame intended as a response to a data request frame or for a broadcast frame
     * following a beacon with the Frame Pending field set to one. */
    uint16_t macMaxFrameTotalWaitTime;

    /* The identifier of the PAN on which the device is operating. If this value is 0xffff, the device is not associated. */
    uint16_t macPANId;

    /* The address that the device uses to communicate in the PAN. If the device is the PAN coordinator, this value shall be
     * chosen before a PAN is started. Otherwise, the short address is allocated by a coordinator during association.
     * A value of 0xfffe indicates that the device has associated but has not been allocated an address. A value of 0xffff
     * indicates that the device does not have a short address. */
    uint16_t macShortAddress;

    /* The length of the active portion of the outgoing superframe, including the beacon frame, as defined in 5.1.1.1 */
    uint8_t macSuperframeOrder;

    /* The maximum time to wait for a response that might be sent over multiple hops in multiples of aBaseSuperframeDuration */
    uint8_t macResponseWaitTime;

    /* DSME SPECIFIC PART FROM HERE ON ------------------------------------> */

    /* Indicates the method of channel diversity:
     0x00 = Channel Adaptation
     0x01 = Channel Hopping
     This value is not valid for a nonbeacon-enabled PAN. */
    uint8_t macChannelDiversityMode;

    /* The length of a multi-superframe, which is a cycle of the repeated superframes. */
    uint8_t macMultiSuperframeOrder;

    /* The slot allocation bitmap table of the DSME-GTS schedule. */
    DSMESlotAllocationBitmap macDSMESAB;

    /* The allocation counter table of the DSME-GTS allocated to the device. */
    DSMEAllocationCounterTable macDSMEACT;

    /* The number of idle incidents before expiring a DSME-GTS.*/
    uint8_t macDSMEGTSExpirationTime;
};

} /* dsme */

#endif /* MAC_PIB_H_ */
