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

#ifndef MESSAGEHELPER_H_
#define MESSAGEHELPER_H_

#include "../../dsme_settings.h"
#include "../helper/DSMEDelegate.h"
#include "../helper/DSMERingbuffer.h"
#include "../mac_services/DSME_Common.h"

namespace dsme {

class DSMEAdaptionLayer;
class IDSMEMessage;
class PANDescriptor;

namespace mcps_sap {
class MCPS_SAP;
struct DATA_confirm_parameters;
struct DATA_indication_parameters;
} /* namespace mcps_sap */

struct DSMEAdaptionLayerBufferEntry {
public:
    IDSMEMessage* message;
    uint32_t initialSymbolCounter;
};

class MessageHelper {
public:
    typedef Delegate<void(IDSMEMessage* msg)> indicationCallback_t;
    typedef Delegate<void(IDSMEMessage* msg, DataStatus::Data_Status dataStatus)> confirmCallback_t;

    explicit MessageHelper(DSMEAdaptionLayer&);

    void initialize();

    void setIndicationCallback(indicationCallback_t);
    void setConfirmCallback(confirmCallback_t);

    void sendMessage(IDSMEMessage* msg);
    void sendRetryBuffer();

    void startAssociation();
    void handleAssociationComplete(AssociationStatus::Association_Status status);
    void handleScanAndSyncComplete(PANDescriptor* panDescriptor);

    bool isAssociated();

private:
    void handleDataConfirm(mcps_sap::DATA_confirm_parameters& params);
    void handleDataIndication(mcps_sap::DATA_indication_parameters& params);

    void receiveIndication(IDSMEMessage* msg);

    void sendMessageDown(IDSMEMessage* msg, bool newMessage);
    bool queueMessageIfPossible(IDSMEMessage* msg);

    DSMEAdaptionLayer& dsmeAdaptionLayer;

    indicationCallback_t callback_indication;
    confirmCallback_t callback_confirm;

    bool scanOrSyncInProgress;
    bool associationInProgress;

    DSMERingBuffer<DSMEAdaptionLayerBufferEntry, UPPER_LAYER_QUEUE_SIZE> retryBuffer;
};

} /* namespace dsme */

#endif /* MESSAGEHELPER_H_ */
