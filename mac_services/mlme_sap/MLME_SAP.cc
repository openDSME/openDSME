/*
 * openDSME
 *
 * Implementation of the Deterministic & Synchronous Multi-channel Extension (DSME)
 * described in the IEEE 802.15.4-2015 standard
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

#include "MLME_SAP.h"

#include "../../dsmeLayer/DSMELayer.h"

namespace dsme {
namespace mlme_sap {

MLME_SAP::MLME_SAP(DSMELayer& dsme) :
    dsme(dsme),
    associate(dsme),
    disassociate(dsme),
    dsme_gts(dsme),
    poll(dsme),
    reset(dsme),
    scan(dsme),
    start(dsme),
    sync(dsme) {
    this->dsme.setMLME(this);
}

ASSOCIATE& MLME_SAP::getASSOCIATE() {
    return this->associate;
}

BEACON_NOTIFY& MLME_SAP::getBEACON_NOTIFY() {
    return this->beacon_notify;
}

COMM_STATUS& MLME_SAP::getCOMM_STATUS() {
    return this->comm_status;
}

DISASSOCIATE& MLME_SAP::getDISASSOCIATE() {
    return this->disassociate;
}

DSME_GTS& MLME_SAP::getDSME_GTS() {
    return this->dsme_gts;
}

POLL& MLME_SAP::getPOLL() {
    return this->poll;
}

RESET& MLME_SAP::getRESET() {
    return this->reset;
}

SCAN& MLME_SAP::getSCAN() {
    return this->scan;
}

SYNC& MLME_SAP::getSYNC() {
    return this->sync;
}

START& MLME_SAP::getSTART() {
    return this->start;
}

SYNC_LOSS& MLME_SAP::getSYNC_LOSS() {
    return this->sync_loss;
}

} /* mlme_sap */
} /* dsme */
