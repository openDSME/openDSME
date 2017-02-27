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

#ifndef GTSCONTROLLER_H_
#define GTSCONTROLLER_H_

#include "../mac_services/dataStructures/RBTree.h"

namespace dsme {

constexpr uint8_t CONTROL_HISTORY_LENGTH = 8;

class DSMEAdaptionLayer;

struct GTSControllerData {
    uint16_t address{0xffff};

    uint16_t messagesIn[CONTROL_HISTORY_LENGTH]{};
    uint16_t messagesOut[CONTROL_HISTORY_LENGTH]{};
    uint8_t history_position{0};

    int16_t queue_size{0};

    int16_t control{0};
};

class GTSController {
public:
    typedef RBTree<GTSControllerData, uint16_t>::iterator iterator;

    GTSController(DSMEAdaptionLayer& dsmeAdaptionLayer);

    void reset();

    void registerIncomingMessage(uint16_t address);

    void registerOutgoingMessage(uint16_t address);

    void superframeEvent();

    int16_t getControl(uint16_t address);

    void indicateChange(uint16_t address, int16_t change);

    uint16_t getPriorityLink();

private:
    DSMEAdaptionLayer& dsmeAdaptionLayer;
    RBTree<GTSControllerData, uint16_t> links;

    uint32_t global_superframe{0};
};

} /* namespace dsme */

#endif /* GTSCONTROLLER_H_ */
