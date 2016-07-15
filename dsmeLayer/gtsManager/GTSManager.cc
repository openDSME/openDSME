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

#include "GTSManager.h"

#include "../../../dsme_platform.h"
#include "../DSMELayer.h"
#include "../messages/GTSReplyNotifyCmd.h"
#include "../messages/GTSRequestCmd.h"
#include "../messages/GTSRequestCmd.h"
#include "../messages/MACCommand.h"

namespace dsme {

GTSManager::GTSManager(DSMELayer& dsme) :
        DSMEBufferedFSM<GTSManager, GTSEvent, 4>(&GTSManager::stateIdle),
        dsme(dsme),
        actUpdater(dsme) {
}

void GTSManager::initialize() {
    dsme.getMAC_PIB().macDSMESAB.initialize(dsme.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe(),
            dsme.getMAC_PIB().helper.getNumGTSlots(),
            dsme.getMAC_PIB().helper.getNumChannels());
    dsme.getMAC_PIB().macDSMEACT.initialize(dsme.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe(),
            dsme.getMAC_PIB().helper.getNumGTSlots(),
            dsme.getMAC_PIB().helper.getNumChannels());
    return;
}

/*****************************
 * States
 *****************************/
fsmReturnStatus GTSManager::stateIdle(GTSEvent& event) {
    LOG_DEBUG("GTS Event handled: '" << signalToString(event.signal) << "' (" << stateToString(&GTSManager::stateIdle) << ")");
    switch(event.signal) {
    case GTSEvent::ENTRY_SIGNAL:
    case GTSEvent::EXIT_SIGNAL:
        return FSM_IGNORED;
    case GTSEvent::MLME_REQUEST_ISSUED: {
        preparePendingConfirm(event);

        DSMEMessage* msg = dsme.getPlatform().getEmptyMessage();

        event.requestCmd.prependTo(msg);

        if(!sendGTSCommand(msg, event.management, CommandFrameIdentifier::DSME_GTS_REQUEST, event.deviceAddr)) {
            dsme.getPlatform().releaseMessage(msg);

            LOG_DEBUG("TRANSACTION_OVERFLOW");
            pendingConfirm.status = GTSStatus::TRANSACTION_OVERFLOW;

            this->dsme.getMLME_SAP().getDSME_GTS().notify_confirm(pendingConfirm);
            return FSM_HANDLED;
        }
        else {
            return transition(&GTSManager::stateSending);
        }
    }

    case GTSEvent::MLME_RESPONSE_ISSUED: {
        preparePendingConfirm(event);

        DSMEMessage* msg = dsme.getPlatform().getEmptyMessage();
        event.replyNotifyCmd.prependTo(msg);

        uint16_t destinationShortAddress;

        if (event.management.status == GTSStatus::SUCCESS) {
            LOG_INFO("Sending a positive response to a GTS-REQUEST to " << event.replyNotifyCmd.getDestinationAddress());

            destinationShortAddress = IEEE802154MacAddress::SHORT_BROADCAST_ADDRESS;
        } else {
            LOG_INFO("Sending a negative response to a GTS-REQUEST to " << event.replyNotifyCmd.getDestinationAddress());

            destinationShortAddress = event.replyNotifyCmd.getDestinationAddress();
        }

        if(!sendGTSCommand(msg, event.management, CommandFrameIdentifier::DSME_GTS_REPLY, destinationShortAddress)) {
            LOG_DEBUG("Could not send REPLY");
            dsme.getPlatform().releaseMessage(msg);

            mlme_sap::COMM_STATUS_indication_parameters params;
            // TODO also fill other fields
            params.status = CommStatus::Comm_Status::TRANSACTION_OVERFLOW;
            this->dsme.getMLME_SAP().getCOMM_STATUS().notify_indication(params);
            return FSM_HANDLED;
        }
        else {
            return transition(&GTSManager::stateSending);
        }
    }

    case GTSEvent::RESPONSE_CMD_FOR_ME:
        // This was received too late, probably because the ACK got lost. Throw it away!
        // TODO What will happen at the sender and at its neighbouring devices?
        return FSM_IGNORED;
    case GTSEvent::NOTIFY_CMD_FOR_ME:
        // This was received too late, probably because the ACK got lost. Throw it away!
        // TODO What will happen at the sender and at its neighbouring devices?
        return FSM_IGNORED;

    case GTSEvent::CFP_STARTED: {
        LOG_DEBUG("SLOT event during IDLE");
        // check if a slot should be deallocated, only if no reply or notify is pending
        for (DSMEAllocationCounterTable::iterator it = dsme.getMAC_PIB().macDSMEACT.begin();
                it != dsme.getMAC_PIB().macDSMEACT.end(); it++) {
            // Since no reply is pending, this slot should have been removed already and is no longer in the ACT
            // This should be even the case for timeouts (NO_DATA indication for upper layer)
            DSME_ASSERT(it->getState() != DEALLOCATED);
            DSME_ASSERT(it->getState() != REMOVED);

            LOG_DEBUG("check slot " << (uint16_t)it->getGTSlotID() << " " << (uint16_t)it->getSuperframeID() << " " << (uint16_t)it->getChannel());

            // TODO Since INVALID is not included in the standard, use the EXPIRATION type for INVALID, too.
            //      The effect should be the same.
            if(it->getState() == INVALID || it->getState() == UNCONFIRMED || it->getIdleCounter() > dsme.getMAC_PIB().macDSMEGTSExpirationTime) {
            // TODO remove if(it->getState() == INVALID || it->getState() == UNCONFIRMED) {

                if(it->getState() == INVALID) {
                    LOG_DEBUG("DEALLOCATE: Due to state INVALID");
                }
                else if(it->getState() == UNCONFIRMED) {
                    LOG_DEBUG("DEALLOCATE: Due to state UNCONFIRMED");
                }
                else if(it->getIdleCounter() > dsme.getMAC_PIB().macDSMEGTSExpirationTime) {
                    LOG_DEBUG("DEALLOCATE: Due to expiration");
                    //continue; // TODO: Remove when it works again!
                }

                LOG_DEBUG("slot to deallocate found!");

                mlme_sap::DSME_GTS_indication_parameters params;
                params.deviceAddress = it->getAddress();

                params.managmentType = EXPIRATION;
                params.direction = it->getDirection();
                params.prioritizedChannelAccess = Priority::LOW;
                params.numSlot = 1;

                uint8_t subBlockLengthBytes = dsme.getMAC_PIB().helper.getSubBlockLengthBytes();
                params.dsmeSABSpecification.setSubBlockLengthBytes(subBlockLengthBytes);
                params.dsmeSABSpecification.setSubBlockIndex(it->getSuperframeID());
                params.dsmeSABSpecification.getSubBlock().fill(false);
                params.dsmeSABSpecification.getSubBlock().set(it->getGTSlotID() * dsme.getMAC_PIB().helper.getNumChannels() + it->getChannel(), true);

                this->dsme.getMLME_SAP().getDSME_GTS().notify_indication(params);
                break;
            }
        }

        return FSM_HANDLED;
    }
    case GTSEvent::SEND_COMPLETE:
        LOG_DEBUG("Outdated message"); // see onCSMASent
        return FSM_IGNORED;
    default:
        DSME_ASSERT(false);
        return FSM_IGNORED;
    }
}

fsmReturnStatus GTSManager::stateSending(GTSEvent& event) {
    LOG_DEBUG("GTS Event handled: '" << signalToString(event.signal) << "' (" << stateToString(&GTSManager::stateSending) << ")");
    switch(event.signal) {
    case GTSEvent::ENTRY_SIGNAL:
    case GTSEvent::EXIT_SIGNAL:
        return FSM_IGNORED;
    case GTSEvent::MLME_REQUEST_ISSUED:
        actionReportBusyNotify(event);
        return FSM_HANDLED;
    case GTSEvent::MLME_RESPONSE_ISSUED:
        actionReportBusyCommStatus(event);
        return FSM_HANDLED;
    case GTSEvent::RESPONSE_CMD_FOR_ME:
        // This was received too late, probably because the ACK got lost. Throw it away!
        // TODO What will happen at the sender and at its neighbouring devices?
        return FSM_IGNORED;
    case GTSEvent::NOTIFY_CMD_FOR_ME:
        // This was received too late, probably because the ACK got lost. Throw it away!
        // TODO What will happen at the sender and at its neighbouring devices?
        return FSM_IGNORED;
    case GTSEvent::CFP_STARTED:
        return FSM_IGNORED;
    case GTSEvent::SEND_COMPLETE: {
        DSME_ASSERT(event.cmdId == DSME_GTS_REQUEST || event.cmdId == DSME_GTS_REPLY || event.cmdId == DSME_GTS_NOTIFY);
        DSME_ASSERT(event.cmdId == cmdToSend);

        if(event.cmdId == DSME_GTS_NOTIFY) {
            actUpdater.notifyDelivered(event.replyNotifyCmd.getSABSpec(), event.management, event.deviceAddr);
            return transition(&GTSManager::stateIdle);
        }
        else if(event.cmdId == DSME_GTS_REQUEST) {
            if(event.dataStatus != DataStatus::Data_Status::SUCCESS) {
                LOG_DEBUG("GTSManager sending request failed " << (uint16_t)event.dataStatus);

                switch(event.dataStatus) {
                case DataStatus::NO_ACK:
                    actUpdater.requestNoAck(event.requestCmd.getSABSpec(), event.management, event.deviceAddr);
                    pendingConfirm.status = GTSStatus::NO_ACK;
                    break;
                case DataStatus::CHANNEL_ACCESS_FAILURE:
                    actUpdater.requestAccessFailure(event.requestCmd.getSABSpec(), event.management, event.deviceAddr);
                    pendingConfirm.status = GTSStatus::CHANNEL_ACCESS_FAILURE;
                    break;
                default:
                    DSME_ASSERT(false);
                }

                this->dsme.getMLME_SAP().getDSME_GTS().notify_confirm(pendingConfirm);
                return transition(&GTSManager::stateIdle);
            }
            else {
                // REQUEST_SUCCESS
                return transition(&GTSManager::stateWaitForResponse);
            }
        }
        else if(event.cmdId == DSME_GTS_REPLY) {
            if(event.dataStatus != DataStatus::Data_Status::SUCCESS) {
                mlme_sap::COMM_STATUS_indication_parameters params;
                // TODO also fill other fields

                switch(event.dataStatus) {
                case DataStatus::NO_ACK:
                    // An ACK is only expected for disapprovals
                    DSME_ASSERT(event.management.status == GTSStatus::GTS_Status::DENIED);
                    actUpdater.disapprovalNoAck(event.replyNotifyCmd.getSABSpec(), event.management, event.deviceAddr);

                    params.status = CommStatus::Comm_Status::NO_ACK;
                    break;
                case DataStatus::CHANNEL_ACCESS_FAILURE:
                    if(event.management.status == GTSStatus::SUCCESS) {
                        actUpdater.approvalAccessFailure(event.replyNotifyCmd.getSABSpec(), event.management, event.deviceAddr);
                    }
                    else if(event.management.status == GTSStatus::DENIED) {
                        actUpdater.disapprovalAccessFailure(event.replyNotifyCmd.getSABSpec(), event.management, event.deviceAddr);
                    }
                    else {
                        DSME_ASSERT(false);
                    }

                    params.status = CommStatus::Comm_Status::CHANNEL_ACCESS_FAILURE;
                    break;
                default:
                    DSME_ASSERT(false);
                }

                this->dsme.getMLME_SAP().getCOMM_STATUS().notify_indication(params);
                return transition(&GTSManager::stateIdle);
            }
            else {
                if(event.management.status == GTSStatus::SUCCESS) {
                    actUpdater.approvalDelivered(event.replyNotifyCmd.getSABSpec(), event.management, event.deviceAddr);
                    return transition(&GTSManager::stateWaitForNotify);
                }
                else if(event.management.status == GTSStatus::DENIED) {
                    actUpdater.disapprovalDelivered(event.replyNotifyCmd.getSABSpec(), event.management, event.deviceAddr);

                    // for disapprovals, no notify is expected
                    return transition(&GTSManager::stateIdle);
                }
                else {
                    DSME_ASSERT(false);
                }
            }
        }

        DSME_ASSERT(false);
        return FSM_IGNORED;
    }
    default:
        DSME_ASSERT(false);
        return FSM_IGNORED;
    }
}

fsmReturnStatus GTSManager::stateWaitForResponse(GTSEvent& event) {
    LOG_DEBUG("GTS Event handled: '" << signalToString(event.signal) << "' (" << stateToString(&GTSManager::stateWaitForResponse) << ")");
    switch(event.signal) {
    case GTSEvent::ENTRY_SIGNAL:
        superframesInCurrentState = 0;
        return FSM_HANDLED;
    case GTSEvent::EXIT_SIGNAL:
        return FSM_IGNORED;
    case GTSEvent::MLME_REQUEST_ISSUED:
        actionReportBusyNotify(event);
        return FSM_HANDLED;
    case GTSEvent::MLME_RESPONSE_ISSUED:
        actionSendImmediateNegativeResponse(event);
        actionReportBusyCommStatus(event);
        return FSM_HANDLED;
    case GTSEvent::RESPONSE_CMD_FOR_ME: {
        mlme_sap::DSME_GTS_confirm_parameters params;
        params.deviceAddress = event.deviceAddr;
        params.managmentType = event.management.type;
        params.direction = event.management.direction;
        params.prioritizedChannelAccess = event.management.prioritizedChannelAccess;
        params.dsmeSABSpecification = event.replyNotifyCmd.getSABSpec();

        // TODO // if the ACK gets lost, the reply might be sent anyway, so we might be in SENDING_REQUEST
        // TODO DSME_ASSERT((state == State::SENDING && cmdToSend == DSME_GTS_REQUEST) || state == State::WAIT_FOR_REPLY);

        DSME_ASSERT(pendingConfirm.deviceAddress == params.deviceAddress);
        DSME_ASSERT(pendingConfirm.managmentType == params.managmentType);
        DSME_ASSERT(pendingConfirm.direction == params.direction);

        params.status = event.management.status;

        this->dsme.getMLME_SAP().getDSME_GTS().notify_confirm(params);

        if(event.management.status == GTSStatus::SUCCESS) {
            if (event.management.type == ALLOCATION) {
                if (checkAndHandleGTSDuplicateAllocation(event.replyNotifyCmd.getSABSpec(), event.deviceAddr, true)) { // TODO issue #3
                    uint8_t numSlotsOk = event.replyNotifyCmd.getSABSpec().getSubBlock().count(true);

                    if (numSlotsOk == 0) {
                        event.management.status = GTSStatus::DENIED;
                    } else {
                        DSME_ASSERT(false); /* This case is not handled properly, better use only one slot per request */
                    }
                }

/* TODO do this after delivering the notify
                this->dsme.getMAC_PIB().macDSMEACT.addNewAllocations(event.replyNotifyCmd.getSABSpec(), event.management.direction, event.deviceAddr, ACTState::VALID);
                //this->dsme.getMAC_PIB().macDSMESAB.addOccupiedSlots(reply.getSABSpec()); //TODO Standard does not demand this.
*/
            }

            /* the requesting node has to notify its one hop neighbors */
            DSMEMessage* msg_notify = dsme.getPlatform().getEmptyMessage();
            event.replyNotifyCmd.setDestinationAddress(event.deviceAddr);
            event.replyNotifyCmd.prependTo(msg_notify);
            if(!sendGTSCommand(msg_notify, event.management, CommandFrameIdentifier::DSME_GTS_NOTIFY, IEEE802154MacAddress::SHORT_BROADCAST_ADDRESS)) {
                // TODO should this be signaled to the upper layer?
                LOG_DEBUG("NOTIFY could not be sent");
                actUpdater.notifyAccessFailure(event.replyNotifyCmd.getSABSpec(), event.management, event.deviceAddr);
                dsme.getPlatform().releaseMessage(msg_notify);
                return transition(&GTSManager::stateIdle);
            }
            else {
                return transition(&GTSManager::stateSending);
            }
        }
        else {
            actUpdater.disapproved(event.replyNotifyCmd.getSABSpec(), event.management, event.deviceAddr);
            return transition(&GTSManager::stateIdle);
        }
    }

    case GTSEvent::NOTIFY_CMD_FOR_ME:
        // This was received too late, probably because the ACK got lost. Throw it away!
        // TODO What will happen at the sender and at its neighbouring devices?
        return FSM_IGNORED;

    case GTSEvent::CFP_STARTED: {
        if(isTimeoutPending()) {
            LOG_DEBUG("GTS timeout for response");
            actUpdater.responseTimeout(pendingConfirm.dsmeSABSpecification, pendingManagement, pendingConfirm.deviceAddress);
            pendingConfirm.status = GTSStatus::GTS_Status::NO_DATA;
            this->dsme.getMLME_SAP().getDSME_GTS().notify_confirm(pendingConfirm);
            return transition(&GTSManager::stateIdle);
        }
        else {
            return FSM_HANDLED;
        }
    }
    case GTSEvent::SEND_COMPLETE:
        LOG_DEBUG("Outdated message"); // see onCSMASent
        return FSM_IGNORED;
    default:
        DSME_ASSERT(false);
        return FSM_IGNORED;
    }
}

fsmReturnStatus GTSManager::stateWaitForNotify(GTSEvent& event) {
    LOG_DEBUG("GTS Event handled: '" << signalToString(event.signal) << "' (" << stateToString(&GTSManager::stateWaitForNotify) << ")");
    switch(event.signal) {
    case GTSEvent::ENTRY_SIGNAL:
        superframesInCurrentState = 0;
        return FSM_HANDLED;
    case GTSEvent::EXIT_SIGNAL:
        return FSM_IGNORED;
    case GTSEvent::MLME_REQUEST_ISSUED:
        actionReportBusyNotify(event);
        return FSM_HANDLED;
    case GTSEvent::MLME_RESPONSE_ISSUED:
        actionSendImmediateNegativeResponse(event);
        actionReportBusyCommStatus(event);
        return FSM_HANDLED;
    case GTSEvent::RESPONSE_CMD_FOR_ME:
        // This was received too late, probably because the ACK got lost. Throw it away!
        // TODO What will happen at the sender and at its neighbouring devices?
        return FSM_IGNORED;

    case GTSEvent::NOTIFY_CMD_FOR_ME: {
        // TODO! DSME_ASSERT((state == State::SENDING && cmdToSend == DSME_GTS_REPLY) || state == State::WAIT_FOR_NOTIFY); // TODO what if the notify comes too late, probably send a deallocation again???
        actUpdater.notifyReceived(event.replyNotifyCmd.getSABSpec(), event.management, event.deviceAddr);

        /* If the DSME-GTS Destination address is the same as the macShortAddress, the device shall notify the next higher
         * layer of the receipt of the DSME-GTS notify command frame using MLME-COMM- STATUS.indication */
        // TODO also for DEALLOCATION?
        mlme_sap::COMM_STATUS_indication_parameters params;
        params.pANId = event.header.getSrcPANId();
        params.srcAddrMode = event.header.getFrameControl().srcAddrMode;
        params.srcAddr = event.header.getSrcAddr();
        params.dstAddrMode = event.header.getFrameControl().dstAddrMode;
        params.dstAddr = event.header.getDestAddr(); //TODO: Header destination address of GTS destination address?
        params.status = CommStatus::SUCCESS;

        dsme.getMLME_SAP().getCOMM_STATUS().notify_indication(params);

        return transition(&GTSManager::stateIdle);
    }

    case GTSEvent::CFP_STARTED: {
        if(isTimeoutPending()) {
            LOG_DEBUG("GTS timeout for notify");
            actUpdater.notifyTimeout(pendingConfirm.dsmeSABSpecification, pendingManagement, pendingConfirm.deviceAddress);

            mlme_sap::COMM_STATUS_indication_parameters params;
            // TODO also fill other fields
            params.status = CommStatus::Comm_Status::TRANSACTION_EXPIRED;
            this->dsme.getMLME_SAP().getCOMM_STATUS().notify_indication(params);
            return transition(&GTSManager::stateIdle);
        }
        else {
            return FSM_HANDLED;
        }
    }
    case GTSEvent::SEND_COMPLETE:
        LOG_DEBUG("Outdated message"); // see onCSMASent
        return FSM_IGNORED;
    default:
        DSME_ASSERT(false);
        return FSM_IGNORED;
    }
}

/*****************************
 * Actions
 *****************************/

const char* GTSManager::signalToString(uint8_t signal) {
    switch (signal) {
        case GTSEvent::EMPTY_SIGNAL:
            return "EMPTY_SIGNAL";
        case GTSEvent::ENTRY_SIGNAL:
            return "ENTRY_SIGNAL";
        case GTSEvent::EXIT_SIGNAL:
            return "EXIT_SIGNAL";
        case GTSEvent::MLME_REQUEST_ISSUED:
            return "MLME_REQUEST_ISSUED";
        case GTSEvent::MLME_RESPONSE_ISSUED:
            return "MLME_RESPONSE_ISSUED";
        case GTSEvent::RESPONSE_CMD_FOR_ME:
            return "RESPONSE_CMD_FOR_ME";
        case GTSEvent::NOTIFY_CMD_FOR_ME:
            return "NOTIFY_CMD_FOR_ME";
        case GTSEvent::CFP_STARTED:
            return "CFP_STARTED";
        case GTSEvent::SEND_COMPLETE:
            return "SEND_COMPLETE";
        default:
            return "UNKNOWN";
    }
}

const char* GTSManager::stateToString(DSMEBufferedFSM<GTSManager, GTSEvent, 4>::state_t state) {
    if(state == &GTSManager::stateIdle) {
        return "IDLE";
    } else if(state == &GTSManager::stateSending) {
        return "SENDING";
    } else if(state == &GTSManager::stateWaitForResponse) {
        return "WAITFORRESPONSE";
    } else if(state == &GTSManager::stateWaitForNotify) {
        return "WAITFORNOTIFY";
    } else {
        return "UNKNOWN";
    }
}

void GTSManager::actionSendImmediateNegativeResponse(GTSEvent& event) {
    DSME_ASSERT(event.signal == GTSEvent::MLME_RESPONSE_ISSUED);

    DSMEMessage* msg = dsme.getPlatform().getEmptyMessage();
    event.replyNotifyCmd.prependTo(msg);

    LOG_INFO(
            "Sending a negative response to a GTS-REQUEST to " << event.replyNotifyCmd.getDestinationAddress() << " due to a TRANSACTION_OVERFLOW");
    uint16_t destinationShortAddress = event.replyNotifyCmd.getDestinationAddress();
    event.management.status = GTSStatus::DENIED;
    if (!sendGTSCommand(msg, event.management, CommandFrameIdentifier::DSME_GTS_REPLY, destinationShortAddress)) {
        LOG_DEBUG("Could not send REPLY");
        dsme.getPlatform().releaseMessage(msg);
    }
}


void GTSManager::actionReportBusyNotify(GTSEvent& event) {
    LOG_DEBUG("BusyNotify on event '" << signalToString(event.signal) << "' (" << stateToString(this->getState()) << ")");

    mlme_sap::DSME_GTS_confirm_parameters busyConfirm;
    busyConfirm.deviceAddress = event.deviceAddr;
    busyConfirm.managmentType = event.management.type;
    busyConfirm.direction = event.management.direction;
    busyConfirm.prioritizedChannelAccess = event.management.prioritizedChannelAccess;
    busyConfirm.dsmeSABSpecification = event.requestCmd.getSABSpec();
    busyConfirm.status = GTSStatus::TRANSACTION_OVERFLOW;
    this->dsme.getMLME_SAP().getDSME_GTS().notify_confirm(busyConfirm);
}

void GTSManager::actionReportBusyCommStatus(GTSEvent& event) {
    LOG_DEBUG("BusyCommstatus on event '" << signalToString(event.signal) << "' (" << stateToString(this->getState()) << ")");

    mlme_sap::COMM_STATUS_indication_parameters params;
    // TODO also fill other fields
    params.status = CommStatus::Comm_Status::TRANSACTION_OVERFLOW;
    this->dsme.getMLME_SAP().getCOMM_STATUS().notify_indication(params);
    return;
}

/*****************************
 * External interfaces
 *****************************/

bool GTSManager::handleMLMERequest(uint16_t deviceAddr, GTSManagement &man, GTSRequestCmd &cmd) {
    return dispatch(GTSEvent::MLME_REQUEST_ISSUED, deviceAddr, man, cmd);
}

bool GTSManager::handleMLMEResponse(GTSManagement man, GTSReplyNotifyCmd reply) {
    uint16_t destinationAddress = reply.getDestinationAddress();
    return dispatch(GTSEvent::MLME_RESPONSE_ISSUED, destinationAddress, man, reply);
}

bool GTSManager::handleGTSRequest(DSMEMessage *msg) {
    // This can be directly passed to the upper layer.
    // There is no need to go over the state machine!
    uint16_t sourceAddr = msg->getHeader().getSrcAddr().getShortAddress();
    GTSManagement man;
    man.decapsulateFrom(msg);
    GTSRequestCmd req;
    req.decapsulateFrom(msg);

    mlme_sap::DSME_GTS_indication_parameters params;
    params.deviceAddress = sourceAddr;
    params.managmentType = man.type;
    params.direction = man.direction;
    params.prioritizedChannelAccess = man.prioritizedChannelAccess;
    params.numSlot = req.getNumSlots();
    params.preferredSuperframeID = req.getPreferredSuperframeID();
    params.preferredSlotID = req.getPreferredSlotID();
    params.dsmeSABSpecification = req.getSABSpec();

    if(man.type == ManagementType::DUPLICATED_ALLOCATION_NOTIFICATION) {
        dsme.getMAC_PIB().macDSMESAB.addOccupiedSlots(req.getSABSpec());
        actUpdater.duplicateAllocation(req.getSABSpec());
    }

    this->dsme.getMLME_SAP().getDSME_GTS().notify_indication(params);
    return true;
}

bool GTSManager::handleGTSResponse(DSMEMessage *msg) {
    GTSManagement management;
    GTSReplyNotifyCmd replyNotifyCmd;
    management.decapsulateFrom(msg);
    replyNotifyCmd.decapsulateFrom(msg);

    if (replyNotifyCmd.getDestinationAddress() == dsme.getMAC_PIB().macShortAddress) {
        return dispatch(GTSEvent::RESPONSE_CMD_FOR_ME, msg, management, replyNotifyCmd);
    }
    else if(management.status == GTSStatus::SUCCESS) {
        // Response overheared -> Add to the SAB regardless of the current state
        DSMESlotAllocationBitmap &macDSMESAB = this->dsme.getMAC_PIB().macDSMESAB;
        if (management.type == ManagementType::ALLOCATION) {
            if (!checkAndHandleGTSDuplicateAllocation(replyNotifyCmd.getSABSpec(), msg->getHeader().getSrcAddr().getShortAddress(), false)) {
                // If there is no conflict, the device shall update macDSMESAB according to the DSMESABSpecification in this
                // command frame to reflect the neighbor's newly allocated DSME-GTSs
                macDSMESAB.addOccupiedSlots(replyNotifyCmd.getSABSpec());
            }
        } else if (management.type == ManagementType::DEALLOCATION) {
            macDSMESAB.removeOccupiedSlots(replyNotifyCmd.getSABSpec());
        }
    }
    else {
        // A denied request should not be sent via broadcast!
        DSME_ASSERT(false);
    }

    return true;
}

bool GTSManager::handleGTSNotify(DSMEMessage* msg) {
    GTSManagement management;


    management.decapsulateFrom(msg);

    if(management.type != ManagementType::ALLOCATION && management.type != ManagementType::DEALLOCATION) {
        return true;
    }

    GTSReplyNotifyCmd replyNotifyCmd;
    replyNotifyCmd.decapsulateFrom(msg);

    if (replyNotifyCmd.getDestinationAddress() == dsme.getMAC_PIB().macShortAddress) {
        return dispatch(GTSEvent::NOTIFY_CMD_FOR_ME, msg, management, replyNotifyCmd);
    } else {
        // Notify overheared -> Add to the SAB regardless of the current state
        DSMESlotAllocationBitmap &macDSMESAB = this->dsme.getMAC_PIB().macDSMESAB;
        if (management.type == ManagementType::ALLOCATION) {
            if (!checkAndHandleGTSDuplicateAllocation(replyNotifyCmd.getSABSpec(), msg->getHeader().getSrcAddr().getShortAddress(), false)) {
                // If there is no conflict, the device shall update macDSMESAB according to the DSMESABSpecification in this
                // command frame to reflect the neighbor's newly allocated DSME-GTSs
                macDSMESAB.addOccupiedSlots(replyNotifyCmd.getSABSpec());
            }
        } else if (management.type == ManagementType::DEALLOCATION) {
            macDSMESAB.removeOccupiedSlots(replyNotifyCmd.getSABSpec());
        }
    }
    return true;
}

bool GTSManager::handleSlotEvent(uint8_t slot, uint8_t superframe) {
    if(slot == dsme.getMAC_PIB().helper.getFinalCAPSlot() + 1) {
        superframesInCurrentState++;
        LOG_DEBUG("superframe event");

        // also execute this during non-idle phases
        if(superframe == 0) {
            // check if a slot should be deallocated, only if no reply or notify is pending
            for (DSMEAllocationCounterTable::iterator it = dsme.getMAC_PIB().macDSMEACT.begin();
                it != dsme.getMAC_PIB().macDSMEACT.end(); it++) {
                // New multi-superframe started, so increment the idle counter according to 5.1.10.5.3
                // TODO for TX this should be the link quality counter, but this is redundant
                it->incrementIdleCounter(); // gets reset to zero on RX or TX
            }
        }

        return dispatch(GTSEvent::CFP_STARTED);
    }
    else {
        return true;
    }
}

bool GTSManager::onCSMASent(DSMEMessage* msg, CommandFrameIdentifier cmdId, DataStatus::Data_Status status, uint8_t numBackoffs) {
    GTSManagement management;
    management.decapsulateFrom(msg);

    bool returnStatus;
    if (management.type == ManagementType::DUPLICATED_ALLOCATION_NOTIFICATION) {
        // Sending the duplicate allocation is not handled by the state machine, since
        // it is just a state-less notification (at least for our interpretation of the standard)
        LOG_DEBUG("DUPLICATED_ALLOCATION_NOTIFICATION sent");
        returnStatus = true;
    } else if (msg != msgToSend) {
        // If the ACK was lost, but the message itself was delivered successfully,
        // the RESPONSE or NOTIFY might already have been handled properly.
        // Same holds if the current state is not sending (see there)
        // TODO What about the states of the receiver and the neighbours?
        LOG_DEBUG("Outdated message");
        returnStatus = true;
    } else {
        LOG_DEBUG("GTSManager::onCSMASent " << status);
        returnStatus = dispatch(GTSEvent::SEND_COMPLETE, msg, management, cmdId, status);
    }

    dsme.getPlatform().releaseMessage(msg);
    return returnStatus;
}


/*****************************
 * Internal helpers
 *****************************/

bool GTSManager::checkAndHandleGTSDuplicateAllocation(DSMESABSpecification& sabSpec, uint16_t addr, bool allChannels) {
    DSMEAllocationCounterTable &macDSMEACT = this->dsme.getMAC_PIB().macDSMEACT;

    bool duplicateFound = false;

    GTSRequestCmd dupReq;
    dupReq.getSABSpec().setSubBlockLengthBytes(sabSpec.getSubBlockLengthBytes()); // = new DSME_GTSRequestCmd("gts-request-duplication");
    dupReq.getSABSpec().setSubBlockIndex(sabSpec.getSubBlockIndex());

    for (DSMESABSpecification::SABSubBlock::iterator it = sabSpec.getSubBlock().beginSetBits(); it != sabSpec.getSubBlock().endSetBits();
            it++) {

        DSMEAllocationCounterTable::iterator actElement = macDSMEACT.find(sabSpec.getSubBlockIndex(), (*it) / dsme.getMAC_PIB().helper.getNumChannels());
        if (actElement != macDSMEACT.end() && (allChannels || actElement->getChannel() == (*it) % dsme.getMAC_PIB().helper.getNumChannels())) {
            LOG_INFO("Duplicate allocation " << (uint16_t)(actElement->getGTSlotID()+9) << " " << (uint16_t)sabSpec.getSubBlockIndex()
                    << " " << (uint16_t)actElement->getChannel());

            duplicateFound = true;
            dupReq.getSABSpec().getSubBlock().set(*it, true);

            // clear bit so the sabSpec can be used in a notification
            sabSpec.getSubBlock().set(*it, false);
        }
    }

    if (duplicateFound) {
        LOG_INFO("Duplicate allocation detected, informing originating device.");
        DSMEMessage* msg = dsme.getPlatform().getEmptyMessage();
        dupReq.prependTo(msg);
        GTSManagement man;
        man.type = ManagementType::DUPLICATED_ALLOCATION_NOTIFICATION;
        man.status = GTSStatus::SUCCESS;

        // this request expects no reply, so do not use usual request command
        // also do not handle this via the state machine
        if(!sendGTSCommand(msg, man, CommandFrameIdentifier::DSME_GTS_REQUEST, addr)) {
            // TODO should this be signaled to the upper layer?
            LOG_DEBUG("Could not send DUPLICATED_ALLOCATION_NOTIFICATION");
            dsme.getPlatform().releaseMessage(msg);
        }
    }

    return duplicateFound;
}

bool GTSManager::isTimeoutPending() {
    // According to the IEEE 802.15.4e standard, the macMaxFrameTotalWaitTime should be used for the timeout.
    // This is not enough, for example due to queuing of the reply and not considering the GTS times.
    // It was changed in the IEEE 802.15.4-2015 standard to macResponseWaitTime (see e.g. Figure 6-57).
    // macResponseWaitTime is given in aBaseSuperframeDurations (that do not include the superframe order)

    return (superframesInCurrentState*(1 << dsme.getMAC_PIB().macSuperframeOrder) > dsme.getMAC_PIB().macResponseWaitTime);
}

bool GTSManager::sendGTSCommand(DSMEMessage* msg, GTSManagement& man, CommandFrameIdentifier commandId, uint16_t dst) {
    man.prependTo(msg);

    MACCommand cmd;
    cmd.setCmdId(commandId);
    cmd.prependTo(msg);

    msg->getHeader().setDstAddr(dst);
    msg->getHeader().setSrcAddrMode(AddrMode::SHORT_ADDRESS);
    msg->getHeader().setSrcAddr(dsme.getMAC_PIB().macShortAddress);
    msg->getHeader().setDstAddrMode(AddrMode::SHORT_ADDRESS);
    msg->getHeader().setAckRequest(true);
    msg->getHeader().setFrameType(IEEE802154eMACHeader::FrameType::COMMAND);

    // The DUPLICATED_ALLOCATION_NOTIFICATION will be sent regardless of the current state and expects no response.
    // For example a DISALLOW REPSPONE will only be sent during BUSY, but the fact that it do not expect a notify
    // is handled inside of the onCSMASent.
    if(man.type != ManagementType::DUPLICATED_ALLOCATION_NOTIFICATION) {
        cmdToSend = commandId;
        msgToSend = msg;
    }

    return dsme.getMessageDispatcher().sendInCAP(msg);
}

void GTSManager::preparePendingConfirm(GTSEvent& event) {
    pendingManagement = event.management;
    pendingConfirm.deviceAddress = event.deviceAddr;
    pendingConfirm.managmentType = event.management.type;
    pendingConfirm.direction = event.management.direction;
    pendingConfirm.prioritizedChannelAccess = event.management.prioritizedChannelAccess;
    if(event.signal == GTSEvent::MLME_REQUEST_ISSUED) {
        pendingConfirm.dsmeSABSpecification = event.requestCmd.getSABSpec();
    }
    else if(event.signal == GTSEvent::MLME_RESPONSE_ISSUED) {
        pendingConfirm.dsmeSABSpecification = event.replyNotifyCmd.getSABSpec();
    }
    else {
        DSME_ASSERT(false);
    }
}


} /* dsme */
