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

#ifndef ACTUPDATER_H
#define ACTUPDATER_H

#include <stdint.h>

#include "../../mac_services/DSME_Common.h"
#include "../messages/GTSManagement.h"
#include "../../mac_services/dataStructures/DSMESABSpecification.h"

namespace dsme {

class DSMELayer;

class ACTUpdater {
public:

    ACTUpdater(DSMELayer& dsme);

    void requestAccessFailure(DSMESABSpecification& sabSpec, GTSManagement& management, uint16_t deviceAddr);
    void requestNoAck(DSMESABSpecification& sabSpec, GTSManagement& management, uint16_t deviceAddr);
    void responseTimeout(DSMESABSpecification& sabSpec, GTSManagement& management, uint16_t deviceAddr);
    void approvalQueued(DSMESABSpecification& sabSpec, GTSManagement& management, uint16_t deviceAddr);
    void approvalReceived(DSMESABSpecification& sabSpec, GTSManagement& management, uint16_t deviceAddr);
    void disapproved(DSMESABSpecification& sabSpec, GTSManagement& management, uint16_t deviceAddr);
    void notifyAccessFailure(DSMESABSpecification& sabSpec, GTSManagement& management, uint16_t deviceAddr);
    void notifyDelivered(DSMESABSpecification& sabSpec, GTSManagement& management, uint16_t deviceAddr);

    void approvalDelivered(DSMESABSpecification& sabSpec, GTSManagement& management, uint16_t deviceAddr);
    void approvalAccessFailure(DSMESABSpecification& sabSpec, GTSManagement& management, uint16_t deviceAddr);
    void notifyTimeout(DSMESABSpecification& sabSpec, GTSManagement& management, uint16_t deviceAddr);
    void notifyReceived(DSMESABSpecification& sabSpec, GTSManagement& management, uint16_t deviceAddr);
    void disapprovalAccessFailure(DSMESABSpecification& sabSpec, GTSManagement& management, uint16_t deviceAddr);
    void disapprovalNoAck(DSMESABSpecification& sabSpec, GTSManagement& management, uint16_t deviceAddr);
    void disapprovalDelivered(DSMESABSpecification& sabSpec, GTSManagement& management, uint16_t deviceAddr);

    void duplicateAllocation(DSMESABSpecification& sabSpec);
    void expired(DSMESABSpecification& sabSpec);

private:
    DSMELayer& dsme;
    Direction invert(Direction direction);
};

}

#endif
