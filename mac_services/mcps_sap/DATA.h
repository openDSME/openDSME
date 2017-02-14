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

#include "../../mac_services/DSME_Common.h"
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
    uint32_t Timestamp;
    bool RangingReceived;
    uint32_t RangingCounterStart;
    uint32_t RangingCounterStop;
    uint32_t RangingTrackingInterval;
    uint32_t RangingOffset;
    uint8_t RangingFOM;
    DataStatus::Data_Status status;

    uint8_t numBackoffs;
    uint8_t dsn;
    uint8_t* AckPayload;

    bool gtsTX; // TODO not covered by the standard, but very useful anyway
};

/*
 * IEEE 802.15.4-2011 6.3.3, Table 48,  IEEE 802.15.4e-2012 6.3.3, Table 48
 */
struct DATA_indication_parameters {
    /*
    These are all included in the msdu
    AddrMode srcAddrMode;
    uint16_t srcPANId;
    IEEE802154MacAddress srcAddr;
    AddrMode dstAddrMode;
    uint16_t dstPANId;
    IEEE802154MacAddress dstAddr;
    uint8_t msduLength;
    */

    IDSMEMessage* msdu;
    uint8_t mpduLinkQuality;
    uint8_t dsn;
    uint32_t timestamp;
    uint8_t securityLevel;
    uint8_t keyIdMode;
    uint8_t* keySource;
    uint8_t keyIndex;
    UWBPRFMode uwbprf;
    uint16_t uwbPreambleSymbolRepetitions;
    uint8_t dataRate;
    ReceivedRangingMode rangingReceived;
    uint32_t rangingCounterStart;
    uint32_t rangingCounterStop;
    uint32_t rangingTrackingInterval;
    uint32_t rangingOffset;
    uint8_t rangingFOM;
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
    struct request_parameters {
        /*
        These are all included in the msdu
        AddrMode srcAddrMode;
        AddrMode dstAddrMode;
        uint16_t dstPANId;
        IEEE802154MacAddress dstAddr;
        uint8_t msduLength;
        */

        IDSMEMessage* msdu;
        uint8_t msduHandle;
        bool ackTX;
        bool gtsTX;
        bool indirectTX;
        uint8_t securityLevel;
        uint8_t keyIdMode;
        uint8_t* keySource;
        uint8_t keyIndex;
        UWBPRFMode uwbprf;
        RangingMode ranging;
        uint16_t uwbPreambleSymbolRepetitions;
        uint8_t dataRate;

        bool frameControlOption_pan_id_suppressed;
        bool frameControlOption_ies_included;
        bool frameControlOption_seq_num_suppressed;
        void* headerIElist;  // TODO
        void* payloadIElist; // TODO
        bool sendMultipurpose;
    };

    void request(request_parameters&);

private:
    DSMELayer& dsme;
};

} /* namespace mcps_sap */
} /* namespace dsme */

#endif /* DATA_H_ */
