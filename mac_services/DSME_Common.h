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

#ifndef DSME_COMMON_H_
#define DSME_COMMON_H_

#include "../helper/DSMEDelegate.h"
#include "../helper/Integers.h"

namespace dsme {

class NOT_IMPLEMENTED_t {
    template <typename T>
    void operator=(T) = delete;
};

enum CommandFrameIdentifier {
    ASSOCIATION_REQUEST = 0x01,
    ASSOCIATION_RESPONSE = 0x02,
    DISASSOCIATION_NOTIFICATION = 0x03,
    DATA_REQUEST = 0x04,
    BEACON_REQUEST = 0x07,
    DSME_ASSOCIATION_REQUEST = 0x13,
    DSME_ASSOCIATION_RESPONSE = 0x14,
    DSME_GTS_REQUEST = 0x15,
    DSME_GTS_REPLY = 0x16,
    DSME_GTS_NOTIFY = 0x17,
    DSME_BEACON_ALLOCATION_NOTIFICATION = 0x1a,
    DSME_BEACON_COLLISION_NOTIFICATION = 0x1b
};

struct CapabilityInformation {
    bool alternatePANCoordinator : 1;
    bool deviceType : 1; // FFD = 1 or RFD = 0
    bool powerSource : 1;
    bool receiverOnWhenIdle : 1;
    bool associationType : 1;
    uint8_t reserved : 1;
    bool securityCapability : 1;
    bool allocateAddress : 1;
};

struct HoppingDescriptor {
    uint8_t hoppingSequenceID;
    uint16_t hoppingSequenceLength;
    uint8_t* hoppingSequence;
    uint16_t channelOffset;
    uint16_t channelOffsetBitmapLength;
    uint8_t* channelOffsetBitmap;
};

enum ManagementType { DEALLOCATION = 0x00, ALLOCATION = 0x01, DUPLICATED_ALLOCATION_NOTIFICATION = 0x02, REDUCE = 0x03, RESTART = 0x04, EXPIRATION = 0x05 };

enum Direction { TX = 0x00, RX = 0x01 };

enum Priority { LOW = 0x00, HIGH = 0x01 };

struct GTSStatus {
    enum GTS_Status {
        SUCCESS,
        DENIED,
        INVALID_PARAMETER,
        NO_ACK,
        NO_DATA,
        CHANNEL_ACCESS_FAILURE,
        /* (table 44t IEEE 802.15.4e-2012 6.2.21.1.4) */
        NO_SHORT_ADDRESS,

        // TODO not covered by the standard
        // A request should be replied by a confirm, so TRANSACTION_OVERFLOW should not be signaled by COMM_STATUS,
        // but instead by a MLME-DSME-GTS.confirm with TRANSACTION_OVERFLOW
        TRANSACTION_OVERFLOW
    };
};

struct InfoStatus {
    enum Info_Status {
        SUCCESS,
        CHANNEL_ACCESS_FAILURE,
        NO_ACK,
        NO_DATA,
        COUNTER_ERROR,
        FRAME_TOO_LONG,
        UNAVAILABLE_KEY,
        UNSUPPORTED_SECURITY,
        INVALID_PARAMETER
    };
};

enum LinkStatusRPT_Status { CHANNEL_ACCESS_FAILURE, NO_ACK, SUCCESS };

struct AssociationStatus {
    enum Association_Status {
        SUCCESS = 0x00,
        PAN_AT_CAPACITY = 0x01,
        PAN_ACCESS_DENIED = 0x02,
        CHANNEL_ACCESS_FAILURE,
        NO_ACK,
        NO_DATA,
        COUNTER_ERROR,
        FRAME_TOO_LONG,
        IMPROPER_KEY_TYPE,
        IMPROPER_SECURITY_LEVEL,
        SECURITY_ERROR,
        UNAVAILABLE_KEY,
        UNSUPPORTED_LEGACY,
        UNSUPPORTED_SECURITY,
        UNSUPPORTED_FEATURE,
        INVALID_PARAMETER,
        FASTA_SUCCESSFUL = 0x80
    };
};

struct DisassociationStatus {
    enum Disassociation_Status {
        SUCCESS,
        NO_ACK,
        TRANSACTION_OVERFLOW,
        TRANSACTION_EXPIRED,
        CHANNEL_ACCESS_FAILURE,
        COUNTER_ERROR,
        FRAME_TOO_LONG,
        UNAVAILABLE_KEY,
        UNSUPPORTED_SECURITY,
        INVALID_PARAMETER
    };
};
struct CommStatus {
    enum Comm_Status {
        SUCCESS,
        TRANSACTION_OVERFLOW,
        TRANSACTION_EXPIRED,
        CHANNEL_ACCESS_FAILURE,
        NO_ACK,
        COUNTER_ERROR,
        FRAME_TOO_LONG,
        IMPROPER_KEY_TYPE,
        IMPROPER_SECURITY_LEVEL,
        SECURITY_ERROR,
        UNAVAILABLE_KEY,
        UNSUPPORTED_LEGACY,
        UNSUPPORTED_SECURITY,
        INVALID_PARAMETER
    };
};

struct GetStatus {
    enum Get_Status { SUCCESS, UNSUPPORTED_ATTRIBUTE };
};

struct ResetStatus {
    enum Reset_Status { SUCCESS };
};

struct ScanStatus {
    enum Scan_Status {
        SUCCESS,
        LIMIT_REACHED,
        NO_BEACON,
        SCAN_IN_PROGRESS,
        COUNTER_ERROR,
        FRAME_TOO_LONG,
        UNAVAILABLE_KEY,
        UNSUPPORTED_SECURITY,
        BAD_CHANNEL,
        INVALID_PARAMETER
    };
};

struct SetStatus {
    enum Set_Status { SUCCESS, READ_ONLY, UNSUPPORTED_ATTRIBUTE, INVALID_INDEX, INVALID_PARAMETER };
};

struct PollStatus {
    enum Poll_Status {
        SUCCESS,
        CHANNEL_ACCESS_FAILURE,
        NO_ACK,
        NO_DATA,
        COUNTER_ERROR,
        FRAME_TOO_LONG,
        UNAVAILABLE_KEY,
        UNSUPPORTED_SECURITY,
        INVALID_PARAMETER
    };
};

struct StartStatus {
    enum Start_Status {
        SUCCESS,
        NO_SHORT_ADDRESS,
        SUPERFRAME_OVERLAP,
        TRACKING_OFF,
        INVALID_PARAMETER,
        COUNTER_ERROR,
        FRAME_TOO_LONG,
        UNAVAILABLE_KEY,
        UNSUPPORTED_SECURITY,
        CHANNEL_ACCESS_FAILURE
    };
};

struct SecurityStatus {
    enum Security_Status {
        SUCCESS,
        COUNTER_ERROR,
        IMPROPER_KEY_TYPE,
        IMPROPER_SECURITY_LEVEL,
        SECURITY_ERROR,
        UNAVAILABLE_KEY,
        UNSUPPORTED_LEGACY,
        UNSUPPORTED_SECURITY
    };
};

enum ScanType {
    ED = 0x00,
    ACTIVE = 0x01,
    PASSIVE = 0x02,
    ORPHAN = 0x03,
    ASYMMETRICMULTICHANNELACTIVE = 0x04,
    CHANNELPROBE = 0x05,
    MULTICHANNELHELLO = 0x06,
    ENHANCEDACTIVESCAN = 0x07
};

struct LossReason {
    enum Loss_Reason { PAN_ID_CONFLICT, REALIGNMENT, BEACON_LOST, SUPERFRAME_OVERLAP };
};

enum AddrMode { NO_ADDRESS = 0x00, SIMPLE_ADDRESS = 0x01, SHORT_ADDRESS = 0x02, EXTENDED_ADDRESS = 0x03 };

enum FrameControlOptionFlags { PAN_ID_SUPPRESSED = 0x00, IES_INCLUDED = 0x01, SEQ_NO_SUPPRESSED = 0x02 };

enum DisassociateReason {
    COORD_WISH_DEVICE_TO_LEAVE = 0x01,
    DEVICE_WISH_TO_LEAVE = 0x02

};

enum RangingMode { NON_RANGING, ALL_RANGING, PHY_HEADER_ONLY };

enum ReceivedRangingMode { NO_RANGING_REQUESTED, RANGING_ACTIVE, RANGING_REQUESTED_BUT_NOT_SUPPORTED };

struct DataStatus {
    enum Data_Status {
        SUCCESS,
        TRANSACTION_OVERFLOW,
        TRANSACTION_EXPIRED,
        CHANNEL_ACCESS_FAILURE,
        INVALID_ADDRESS,
        INVALID_GTS,
        NO_ACK,
        COUNTER_ERROR,
        FRAME_TOO_LONG,
        UNAVAILABLE_KEY,
        UNSUPPORTED_SECURITY,
        INVALID_PARAMETER,
        ACK_RCVD_NODSN_NOSA
    };
};

struct PurgeStatus {
    enum Purge_status { SUCCESS, INVALID_HANDLE };
};

} /* namespace dsme */

#endif /* DSME_COMMON_H_ */
