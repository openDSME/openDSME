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

#ifndef MLME_SAP_H_
#define MLME_SAP_H_

#include "ASSOCIATE.h"
#include "BEACON_NOTIFY.h"
#include "COMM_STATUS.h"
#include "DISASSOCIATE.h"
#include "DSME_GTS.h"
#include "POLL.h"
#include "RESET.h"
#include "SCAN.h"
#include "START.h"
#include "SYNC.h"
#include "SYNC_LOSS.h"

namespace dsme {
class DSMELayer;

namespace mlme_sap {

class MLME_SAP {
public:
    explicit MLME_SAP(DSMELayer& dsme);

    ASSOCIATE& getASSOCIATE();
    BEACON_NOTIFY& getBEACON_NOTIFY();
    COMM_STATUS& getCOMM_STATUS();
    DISASSOCIATE& getDISASSOCIATE();
    DSME_GTS& getDSME_GTS();
    POLL& getPOLL();
    RESET& getRESET();
    SCAN& getSCAN();
    START& getSTART();
    SYNC& getSYNC();
    SYNC_LOSS& getSYNC_LOSS();

private:
    DSMELayer& dsme;

    ASSOCIATE associate;
    BEACON_NOTIFY beacon_notify;
    COMM_STATUS comm_status;
    DISASSOCIATE disassociate;
    DSME_GTS dsme_gts;
    POLL poll;
    RESET reset;
    SCAN scan;
    START start;
    SYNC sync;
    SYNC_LOSS sync_loss;
};

} /* namespace mlme_sap */
} /* namespace dsme */

#endif /* MLME_SAP_H_ */
