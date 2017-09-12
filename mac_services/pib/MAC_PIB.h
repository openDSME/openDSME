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

#include "../dataStructures/BeaconBitmap.h"
#include "../dataStructures/DSMEAllocationCounterTable.h"
#include "../dataStructures/DSMESlotAllocationBitmap.h"
#include "../dataStructures/IEEE802154MacAddress.h"
#include "./PHY_PIB.h"
#include "./PIBHelper.h"

namespace dsme {

/*
 * This class contains MAC PIB attributes as defined in IEEE 802.15.4-2011, 6.4.2, Table 52
 * and IEEE 802.15.4e-2012, 6.4.2, Table 52 and 802.15.4e-2012, 6.4.3.6, Table 52h-i.
 */
class PHY_PIB;

class MAC_PIB {
public:
    explicit MAC_PIB(PHY_PIB&);
    PIBHelper helper;

    /* NON-STANDARD PART FROM HERE ON -------------------------------------> */

    /* Indication of whether the device is the PAN coordinator. */
    bool macIsPANCoord{false};

    /* Indication of whether the device is a coordinator. */
    bool macIsCoord{false};

    /* GENERAL PART FROM HERE ON ------------------------------------------> */

    /** The extended address assigned to the device. */
    IEEE802154MacAddress macExtendedAddress{0xffff, 0xffff, 0xffff, 0xffff};

    /** Indication of whether the device is associated to the PAN through the PAN coordinator. A value of TRUE indicates the device has associated through the
     * PAN coordinator. Otherwise, the value is set to FALSE. */
    bool macAssociatedPANCoord{false};

    /** Indication of whether a coordinator is currently allowing association. A value of TRUE indicates that association is permitted. */
    bool macAssociationPermit{false};

    /** Indication of whether a device automatically sends a Data Request command if its address is listed in the beacon frame. A value of TRUE indicates that
     * the Data Request command is automatically sent. This attribute also affects the generation of the MLME-BEACON-NOTIFY.indication primitive, as described
     * in 8.2.5.1 */
    bool macAutoRequest{true};

    /** Indication of whether BLE, through the reduction of coordinator receiver operation time during the CAP, is enabled. A value of TRUE indicates that it is
     * enabled. The effect of this attribute on the backoff exponent in the CSMA-CA algorithm is described in 6.2.5.1. */
    bool macBattLifeExt{false};

    /** In BLE mode, the number of backoff periods during which the receiver is enabled after the IFS following a beacon. This value is dependent on the
     * supported PHY and is the sum of three terms:
     * Term 1: The value 2 x – 1, where x is the maximum value of macMinBe in BLE mode (equal to two). This term is thus equal to three backoff periods.
     * Term 2: The duration of the initial contention window length, as described in 6.2.5.1.
     * Term 3: The Preamble field length and the SFD field length of the supported PHY summed together and rounded up (if necessary) to an integer number of
     * backoff periods. */
    uint16_t macBattLifeExtPeriods{0};

    /** A sequence of zero or more octets to be transmitted in the Beacon Payload field. */
    uint8_t* macBeaconPayload{nullptr};

    /** Indicates the frequency with which the beacon is transmitted, as defined in 6.2.1. */
    uint8_t macBeaconOrder{15};

    /** The time that the device transmitted its last beacon frame, in symbol periods. The measurement shall be taken at the same symbol boundary within every
     * transmitted beacon frame, the location of which is implementation specific. The precision of this value shall be a minimum of 20 bits, with the lowest
     * four bits being the least significant. */
    uint32_t macBeaconTxTime{0x000000};

    /** The sequence number added to the transmitted beacon frame. */
    uint8_t macBsn{0x00};

    /** The address of the coordinator through which the device is associated. */
    IEEE802154MacAddress macCoordExtendedAddress{0xffff, 0xffff, 0xffff, 0xffff};

    /** The short address assigned to the coordinator through which the device is associated. A value of 0xfffe indicates that the coordinator is only using its
     * extended address. A value of 0xffff indicates that this value is unknown. */
    uint16_t macCoordShortAddress{0xffff};

    /** The sequence number added to the transmitted Data frame or MAC command. */
    uint8_t macDsn{0x00};

    /** TRUE if the PAN coordinator is to accept GTS requests. FALSE otherwise. */
    bool macGtsPermit{true};

    /** The maximum value of the backoff exponent, BE, in the CSMA-CA algorithm, as defined in 6.2.5.1. */
    uint8_t macMaxBE{5};

    /** The maximum number of backoffs the CSMA-CA algorithm will attempt before declaring a channel access failure. */
    uint8_t macMaxCSMABackoffs{4};

    /** The maximum number of retries allowed after a transmission failure. */
    uint8_t macMaxFrameRetries{3};

    /** The minimum value of the backoff exponent (BE) in the CSMA-CA algorithm, as described in 6.2.5.1. */
    uint8_t macMinBE{3};

    /** The identifier of the PAN on which the device is operating. If this value is 0xffff, the device is not associated. */
    uint16_t macPANId{0xffff};

    /** Indication of whether the MAC sublayer is in a promiscuous (receive all) mode. A value of TRUE indicates that the MAC sublayer accepts all frames
     * received from the PHY. */
    bool macPromiscuousMode{false};

    /** This indicates whether the MAC sublayer supports the optional ranging features. */
    bool macRangingSupported{false};

    /** The maximum time, in multiples of aBaseSuperframeDuration, a device shall wait for a response command to be available following a request command. */
    uint8_t macResponseWaitTime{32};

    /** Indication of whether the MAC sublayer is to enable its receiver during idle periods. For a beacon-enabled PAN, this attribute is relevant only during
     * the CAP of the incoming superframe. For a nonbeacon-enabled PAN, this attribute is relevant at all times. */
    bool macRxOnWhenIdle{false};

    /** Indication of whether the MAC sublayer has security enabled. A value of TRUE indicates that security is enabled, while a value of FALSE indicates that
     * security is disabled. */
    bool macSecurityEnabled{false};

    /** The address that the device uses to communicate in the PAN. If the device is the PAN coordinator, this value shall be chosen before a PAN is started.
     * Otherwise, the short address is allocated by a coordinator during association. */
    uint16_t macShortAddress{0xffff};

    /** The length of the active portion of the outgoing superframe, including the beacon frame, as defined in 6.2.1 */
    uint8_t macSuperframeOrder{15};

    /** The offset, measured in symbols, between the symbol boundary at which the MLME captures the timestamp of each transmitted or received frame, and the
     * onset of the first symbol past the SFD. */
    uint16_t macSyncSymbolOffset{0};

    /** Specification of how often the coordinator transmits an Enhanced Beacon frame. If macEnhancedBeaconOrder = 15, no periodic Enhanced Beacon frame will be
     * transmitted. */
    uint8_t macEnhancedBeaconOrder{0};

    /** An indication of whether the Coexistence Specification IE, as defined in 7.4.4.9, is to be included in the Enhanced Beacon frame. If this value is TRUE,
     * the Enhanced Beacon frame will include the Coexistence Specification IE. If this value is FALSE, the Enhanced Beacon frame will not include the
     * Coexistence Specification IE */
    bool macMpmIe{false};

    /** Specification of how often the coordinator transmits an Enhanced Beacon frame in a nonbeacon-enabled PAN (i.e., macBeaconOrder = 15). If
     * macNbPanEnhancedBeaconOrder = 16383, no Enhanced Beacon frame will be transmitted. */
    uint16_t macNbPanEnhancedBeaconOrder{16383};

    /** The offset between the start of the periodic beacon transmission and the start of the following Enhanced Beacon frame transmission expressed in
     * superframe time slots. */
    uint8_t macOffsetTimeSlot{15};

    /** The type of the FCS, as defined in 7.2.10. A value of zero indicates a 4-octet FCS. A value of one indicates a 2-octet FCS. This attribute is only valid
     * for LECIM, TVWS, and SUN PHYs. */
    uint8_t macFcsType{0};

    /** Indication of whether the MAC sublayer supports the optional timestamping feature for incoming and outgoing Data frames */
    bool macTimestampSupported{true};

    /** The maximum time (in unit periods) that a transaction is stored by a coordinator and indicated in its beacon. The unit period is governed by
     * macBeaconOrder, BO, as follows:
     * For 0 <= BO <= 14, the unit period will be aBase-SuperframeDuration * 2 BO.
     * For BO = 15, the unit period will be aBaseSuperframeDuration */
    uint16_t macTransactionPersistenceTime{0x01f4};

    /** Indicates whether frames without a destination PAN ID and a destination address are to treated as though they are addressed to the broadcast PAN ID and
     * broadcast short address. */
    bool macImplicitBroadcast{false};

    /** The number of symbols for backoff when PCA backoff algorithm is in use, as defined in 6.2.5.5. */
    uint8_t macLecimAlohaUnitBackoffPeriod{0};

    /** The value of the constant backoff exponent for priority messages using CCA Mode 4 (ALOHA), as described in 6.2.5.5. */
    uint8_t macLecimAlohaBe{2};

    /** Indicates whether PCA is enabled.A value of TRUE indicates that it is enabled, while a value of FALSE indicates that it is disabled. */
    bool macPriorityChannelAccess{false};

    /** Indicates the PCA allocation rate per superframe. A value of TRUE indicates one or more allocations per superframe. A value of FALSE indicates less than
     * one allocation per superframe. */
    bool macPcaAllocationSuperRate{true};

    /** The PCA allocation rate. If macPcaAllocationSuperRate is TRUE, the value is the number of allocations per superframe. If macPcaAllocationSuperRate is
     * FALSE, the value is the number of superframes per PCA allocation. */
    uint8_t macPcaAllocationRate{1};

    /** The maximum transaction delay, in milliseconds, for a critical event message before issuing an MCPS-DATA.confirm with Status set to TRANSACTION_EXPIRED,
     * as defined in 7.4.4.15. */
    uint32_t macCritMsgDelayTol{15000};

    /** Frequency in kilohertz indicating the lower edge of the band. */
    uint32_t macStartBandEdge{608000};

    /** Frequency in kilohertz indicating the upper edge of the band. */
    uint32_t macEndBandEdge{614000};

    /** Enables the reception of EUI-64 group addresses. */
    bool macGroupRxMode{false};

    /** The extended length of the active portion of the superframe, as defined in 6.2.8. */
    uint8_t macTmctpExtendedOrder{0};

    /* FUNCTIONAL ORGANIZATION  PART FROM HERE ON -------------------------> */

    /** If TRUE, the device is capable of functionality specific to TSCH. */
    bool macTschCapable{false};

    /** If TRUE, the device is capable of functionality specific to DSME. */
    bool macDsmeCapable{true};

    /** If TRUE, the device is capable of functionality specific to low energy. */
    bool macLeCapable{false};

    /** If TRUE, the device is capable of unslotted channel hopping. */
    bool macHoppingCapable{false};

    /** If TRUE, the device is capable of providing additional MAC metrics. */
    bool macMetricsCapable{false};

    /** If TRUE, the device is using functionality specific to TSCH. */
    bool macTschEnabled{false};

    /** If TRUE, the device is using functionality specific to DSME. */
    bool macDsmeEnabled{true};

    /** If TRUE, the device is using functionality specific to low energy. */
    bool macLeEnabled{false};

    /** If TRUE, the device is using unslotted channel hopping. */
    bool macHoppingEnabled{false};

    /** If TRUE, the device is providing additional MAC metrics. */
    bool macMetricsEnabled{false};

    /** A value of TRUE indicates that the device is capable of functionality specific to ExtendedDSME. A value of FALSE indicates that the device is not
     * capable of functionality specific to ExtendedDSME. */
    bool macExtendedDsmeCapable{false};

    /** A value of TRUE indicates that the device is using functionality specific to ExtendedDSME. A value of FALSE indicates that the device is not using
     * functionality specific to ExtendedDSME. */
    bool macExtendedDsmeEnabled{false};

    /** A value of TRUE indicates that the device is using functionality specific to low-energy handshake. A value of FALSE indicates that the low-energy
     * handshake is not used. */
    bool macLeHsEnabled{false};

    /** If TRUE, the device is using TRLE relaying mode, as defined in F.3. If FALSE, the device is not using TRLE relaying mode. */
    bool macRelayingMode{false};

    /** If TRUE, the device is using functionality specific to TRLE. If FALSE, the device is not operating as a TRLE PAN. */
    bool macTrleEnabled{false};

    /** An indication of whether the device is using functionality specific to an RCCN. If TRUE, the device is using this functionality. If FALSE, it is not. */
    bool macRccnEnabled{false};

    /** The number of timeslots within a superframe, excluding the timeslot for beacon frame and management timeslots. */
    uint8_t macRccnNumTimeSlots{48};

    /** The number of management timeslots. */
    uint8_t macRccnNumMgmtTs{4};

    /** The number of GTS timeslots. */
    uint8_t macRccnNumGtsTs{24};

    /* A network specific identification. */
    uint8_t* macRccnNetId{nullptr};

    /** Indicates the type of RCCN device. It may be one of the following device types: a PAN coordinator, a mobile device that is not a PAN coordinator, or a
     * fixed device that is not a PAN coordinator. */
    enum { RCCPANC, MOBILE, FIXED } macRccnDevType{FIXED};

    /* The PHY capabilities supported. */
    uint8_t* macRccnCap{nullptr};

    /* HOPPING SEQUENCE SPECIFIC PART FROM HERE ON ------------------------> */

    /** The unique ID of the hopping sequence. */
    uint8_t macHoppingSequenceId{0};

    /** The channel page for the hopping sequence. Note this may not correspond to the current channelPage. */
    uint8_t macChannelPage{0};

    /** Number of channels supported by the PHY on this channelPage. */
    uint16_t macNumberOfChannels{0};

    /** For channel pages other than 9 and 10, the 27 LSBs (b0, b1, …, b26) indicate the status (1 = to be used, 0 = not to be used) for each of the up to 27
     * channels available to the PHY. For pages 9 and 10, the 27 LSBs indicate the configuration of the PHY, and the channel list is contained in the Extended
     * Bitmap field. */
    uint32_t macPhyConfiguration{0};

    /** The number of channels in the Hopping Sequence. Does not necessarily equal macNumberOfChannels. */
    uint8_t macHoppingSequenceLength{0}; //TODO this should be uint16_t

    /** A macHoppingSequenceLengthelement set of channels to be hopped over. */
    MacStaticList<uint8_t, 30> macHoppingSequenceList; //TODO this should be uint16_t

    /** For unslotted channel hopping modes, this field is the channel dwell time, in units of 10 µs. For other modes, the field is empty. */
    uint16_t macHopDwellTime{0};

    /* DSME SPECIFIC PART FROM HERE ON ------------------------------------> */

    /** Specifies the channel index of the channel's DSME link reported by the source device. */
    uint8_t macChannelIndex{0};

    /** A characterization of the link quality between a source device and a destination device on the channel defined by Channel Index, the measurement shall
     * be performed for each received packet during a period of LinkStatusStatisticPeriod. */
    uint8_t macAvgLqi{0};

    /** Average RSSI. */
    uint8_t macAvgRssi{0};

    /** The time interval between two times of link status statistics, which is defined as LinkStatusStatisticPeriod = aBaseSuperframeDuration * 2 MO symbols.
     * If the parameter equals to 0x000000, link status statistic is not allowed. */
    uint32_t macLinkStatusStatisticPeriod{0};

    /** Indication of whether the CAP reduction is enabled. A value of TRUE indicates that the CAP reduction is enabled. */
    bool macCapReduction{false};

    /** Indicates the method of channel diversity in a beacon-enabled PAN, either channel adaptation or channel hopping. */
    enum { ADAPTATION, HOPPING } macChannelDiversityMode{HOPPING};

    /** The length of a multi-superframe, which is a cycle of the repeated superframes. */
    uint8_t macMultiSuperframeOrder{15};

    /** Indication of whether the device is a Connection Device or not. If this attribute is TRUE, the device is a Connection Device. This attribute shall be
     * set to FALSE if the device is not a Connection Device. */
    bool macConnecDev{false};

    /** The slot allocation bitmap of the DSME-GTS schedule. */
    DSMESlotAllocationBitmap macDSMESAB{};

    /** A list of allocation counter tables of the DSME GTSs allocated to the device. */
    DSMEAllocationCounterTable macDSMEACT{};

    /** Specifies the allocating SD index number for beacon frame. */
    uint16_t macSdIndex{0x0000};

    /** Indicates the beacon frame allocation information of neighbor nodes. This field is expressed in bitmap format that orderly represents the schedule of
     * beacons, with corresponding bit shall be set to one if a beacon is allocated in that SD. */
    BeaconBitmap macSdBitmap{};

    /** ChannelOffset is the offset value of Hopping Sequence. */
    uint16_t macChannelOffset{0};

    /** Indication of whether the device uses CCA before transmitting beacon frame. A value of TRUE indicates that the device uses CCA before transmitting
     * beacon frame. */
    bool macDeferredBeaconUsed{false};

    /** The extended address of the coordinator through which the device is synchronized. */
    IEEE802154MacAddress macSyncParentExtendedAddress{0xffff, 0xffff, 0xffff, 0xffff};

    /** The short address assigned to the coordinator through which the device is synchronized. A value of 0xfffe indicates that the coordinator is only using
     * its extended address. A value of 0xffff indicates that this value is unknown. */
    uint16_t macSyncParentShortAddress{0xffff};

    /** Indication of SD index the synchronized parent used. */
    uint16_t macSyncParentSdIndex{0};

    /** Link report for each used channel. */
    void* macChannelStatus{nullptr};

    /** The number of symbols forming a beacon slot. */
    uint16_t macBeaconSlotLength{60};

    /** The number of idle incidents before expiring a DSME-GTS.*/
    uint8_t macDSMEGTSExpirationTime{7};

    /** Specifies the length of ChannelOffsetBitmap in octets. */
    uint8_t macChannelOffsetBitmapLength{0};

    /** A bitmap that represents whether the corresponding channel offset is used. If the corresponding channel offset is used, the bit value shall be set to
     * one. Otherwise, it shall be set to zero.
     * For instance, if the 1st, 2nd, 4th channels offset are used with ChannelOffsetBitmapLength of 16, ChannelOffsetBitmap shall be 0b0110100000000000. */
    uint8_t* macChannelOffsetBitmap{nullptr};

    /** The sequence number added to the transmitted beacon frame of a PAN coordinator. */
    uint8_t macPanCoordinatorBsn{0};

    /** A table of the neighbor device's information entries. */
    void* macNeighborInformationTable{nullptr};

    /** As defined in 7.5.13. If MO < BO, the value shall be set to zero. */
    uint8_t macAllocationOrder{0};

    /** As defined in 7.5.13. */
    uint8_t macBiIndex{0};

    /** Indicates whether DSME GTSs are allocated during the association procedure. This attribute is set to TRUE if a device requests assignment of a DSME GTS
     * during association. */
    bool macDsmeAssociation{true};
};

} /* namespace dsme */

#endif /* MAC_PIB_H_ */
