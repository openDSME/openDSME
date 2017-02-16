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

#ifndef SCAN_H_
#define SCAN_H_

#include "../ConfirmBase.h"
#include "../DSME_Common.h"
#include "../dataStructures/DSMEPANDescriptor.h"
#include "../pib/PHY_PIB.h"
#include "./helper/PanDescriptorList.h"

namespace dsme {
class DSMELayer;

namespace mlme_sap {

struct SCAN_confirm_parameters {
    ScanType scanType;
    uint8_t channelPage;
    NOT_IMPLEMENTED_t unscannedChannels;
    uint8_t resultListSize;
    NOT_IMPLEMENTED_t energyDetectList;
    PanDescriptorList panDescriptorList;
    uint8_t detectedCategory;
    NOT_IMPLEMENTED_t hrpUwbEnergyDetectList;
    ScanStatus::Scan_Status status;
};

/*
 * These primitives are used to either find PANs in a channel or measure the energy in the channel.
 * (IEEE 802.15.4-2011 6.2.10 and IEEE 802.15.4e-2012 updates)
 */
class SCAN : public ConfirmBase<SCAN_confirm_parameters> {
public:
    explicit SCAN(DSMELayer& dsme);
    struct request_parameters {
        ScanType scanType;
        channelList_t scanChannels;
        uint16_t scanDuration;
        uint8_t channelPage;
        NOT_IMPLEMENTED_t securityLevel;
        NOT_IMPLEMENTED_t keyIdMode;
        NOT_IMPLEMENTED_t keySource;
        NOT_IMPLEMENTED_t keyIndex;
        bool linkQualityScan;
        NOT_IMPLEMENTED_t panIdSuppressed;
        NOT_IMPLEMENTED_t seqNumSuppressed;
        NOT_IMPLEMENTED_t headerIeList;
        NOT_IMPLEMENTED_t payloadIeList;
        NOT_IMPLEMENTED_t headerIeIdList;
        NOT_IMPLEMENTED_t nestedIeSubIdList;
        NOT_IMPLEMENTED_t mpmScanDurationBPan;
        NOT_IMPLEMENTED_t mpmScanDurationNbPan;
        NOT_IMPLEMENTED_t mpmScan;
        NOT_IMPLEMENTED_t mpmScanType;
    };

    void request(request_parameters&);

private:
    DSMELayer& dsme;
};

} /* namespace mlme_sap */
} /* namespace dsme */

#endif /* SCAN_H_ */
