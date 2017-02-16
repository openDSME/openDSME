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

#ifndef DATA_H_
#define DATA_H_

#include "../ConfirmBase.h"
#include "../DSME_Common.h"
#include "../IndicationBase.h"
#include "../dataStructures/IEEE802154MacAddress.h"

namespace dsme {
class DSMELayer;
class IDSMEMessage;

namespace mcps_sap {

/*
 * IEEE 802.15.4-2011 6.3.2, Table 47,  IEEE 802.15.4e-2012 6.3.2, Table 47
 */
struct DATA_confirm_parameters {
    IDSMEMessage* msduHandle;
    uint32_t timestamp;
    bool rangingReceived;
    uint32_t rangingCounterStart;
    uint32_t rangingCounterStop;
    uint32_t rangingTrackingInterval;
    uint32_t rangingOffset;
    uint8_t rangingFom;
    uint8_t numBackoffs;
    NOT_IMPLEMENTED_t ackPayload;
    DataStatus::Data_Status status;

    bool gtsTX; // TODO not covered by the standard, but very useful anyway
};

/*
 * IEEE 802.15.4-2011 6.3.3, Table 48,  IEEE 802.15.4e-2012 6.3.3, Table 48
 */

/* These are all included in the msdu
AddrMode srcAddrMode;
uint16_t srcPANId;
IEEE802154MacAddress srcAddr;
AddrMode dstAddrMode;
uint16_t dstPANId;
IEEE802154MacAddress dstAddr;
uint8_t msduLength;
*/
struct DATA_indication_parameters {
    IDSMEMessage* msdu;
    NOT_IMPLEMENTED_t headerIeList;
    NOT_IMPLEMENTED_t payloadIeList;
    uint8_t mpduLinkQuality;
    uint8_t dsn;
    uint32_t timestamp;
    uint8_t securityLevel;
    NOT_IMPLEMENTED_t keyIdMode;
    NOT_IMPLEMENTED_t keySource;
    NOT_IMPLEMENTED_t keyIndex;
    ReceivedRangingMode rangingReceived;
    uint32_t rangingCounterStart;
    uint32_t rangingCounterStop;
    uint32_t rangingTrackingInterval;
    uint32_t rangingOffset;
    uint8_t rangingFom;
    NOT_IMPLEMENTED_t angleOfArrivalAzimuth;
    NOT_IMPLEMENTED_t angleOfArrivalElevation;
    NOT_IMPLEMENTED_t angleOfArrivalSupported;
    uint8_t dataRate;
    NOT_IMPLEMENTED_t rssi;
};

/*
 * These primitives support the transport of data (IEEE 802.15.4-2011 6.3.1 and IEEE 802.15.4e-2012 updates).
 */
class DATA : public ConfirmBase<DATA_confirm_parameters>, public IndicationBase<DATA_indication_parameters> {
public:
    explicit DATA(DSMELayer& dsme);

    /*
     * IEEE 802.15.4-2011 6.3.1, Table 46,  IEEE 802.15.4e-2012 6.3.1, Table 46
     */

    /* These are all included in the msdu
    AddrMode srcAddrMode;
    AddrMode dstAddrMode;
    uint16_t dstPANId;
    IEEE802154MacAddress dstAddr;
    uint8_t msduLength;
    */
    struct request_parameters {
        IDSMEMessage* msdu;
        uint8_t msduHandle;
        NOT_IMPLEMENTED_t headerIeList;
        NOT_IMPLEMENTED_t payloadIeList;
        NOT_IMPLEMENTED_t headerIeIdList;
        NOT_IMPLEMENTED_t nestedIeSubIdList;
        bool ackTx;
        bool gtsTx;
        bool indirectTx;
        NOT_IMPLEMENTED_t securityLevel;
        NOT_IMPLEMENTED_t keyIdMode;
        NOT_IMPLEMENTED_t keySource;
        NOT_IMPLEMENTED_t keyIndex;
        NOT_IMPLEMENTED_t uwbPrf;
        RangingMode ranging;
        uint16_t uwbPreambleSymbolRepetitions;
        uint8_t dataRate;
        NOT_IMPLEMENTED_t locationEnhancingInformationPostamble;
        NOT_IMPLEMENTED_t locationEnhancingInformationPostambleLength;
        bool panIdSuppressed;
        bool seqNumSuppressed;
        bool sendMultipurpose;
        NOT_IMPLEMENTED_t frakPolicy;
        NOT_IMPLEMENTED_t criticalEventMessage;
    };

    void request(request_parameters&);

private:
    DSMELayer& dsme;
};

} /* namespace mcps_sap */
} /* namespace dsme */

#endif /* DATA_H_ */
