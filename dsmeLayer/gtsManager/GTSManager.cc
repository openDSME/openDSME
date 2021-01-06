/*
 * openDSME
 *
 * Implementation of the Deterministic & Synchronous Multi-channel Extension (DSME)
 * described in the IEEE 802.15.4-2015 standard
 *
 * Authors: Florian Kauer <florian.kauer@tuhh.de>
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

#include "./GTSManager.h"

#include "../../../dsme_platform.h"
#include "../../helper/DSMEAtomic.h"
#include "../../interfaces/IDSMEMessage.h"
#include "../../interfaces/IDSMEPlatform.h"
#include "../../mac_services/dataStructures/DSMEAllocationCounterTable.h"
#include "../../mac_services/dataStructures/DSMESABSpecification.h"
#include "../../mac_services/dataStructures/IEEE802154MacAddress.h"
#include "../../mac_services/mlme_sap/COMM_STATUS.h"
#include "../../mac_services/mlme_sap/DSME_GTS.h"
#include "../../mac_services/pib/MAC_PIB.h"
#include "../../mac_services/pib/PIBHelper.h"
#include "../DSMELayer.h"
#include "../messageDispatcher/MessageDispatcher.h"
#include "../messages/GTSReplyNotifyCmd.h"
#include "../messages/GTSRequestCmd.h"
#include "../messages/MACCommand.h"

namespace dsme {

void GTSEvent::fill(void) {
}

void GTSEvent::fill(IDSMEMessage* msg, GTSManagement& management, CommandFrameIdentifier cmdId, DataStatus::Data_Status dataStatus) {
    switch(cmdId) {
        case CommandFrameIdentifier::DSME_GTS_REQUEST:
            this->requestCmd.decapsulateFrom(msg);
            this->deviceAddr = msg->getHeader().getDestAddr().getShortAddress();
            break;
        case CommandFrameIdentifier::DSME_GTS_REPLY:
        case CommandFrameIdentifier::DSME_GTS_NOTIFY:
            this->replyNotifyCmd.decapsulateFrom(msg);
            this->deviceAddr = this->replyNotifyCmd.getDestinationAddress();
            break;
        default:
            this->deviceAddr = msg->getHeader().getDestAddr().getShortAddress();
            break;
    }
    this->management = management;
    this->cmdId = cmdId;
    this->dataStatus = dataStatus;
}

void GTSEvent::fill(uint16_t& deviceAddr, GTSManagement& management, GTSReplyNotifyCmd& replyNotifyCmd) {
    this->deviceAddr = deviceAddr;
    this->management = management;
    this->replyNotifyCmd = replyNotifyCmd;
}

void GTSEvent::fill(uint16_t& deviceAddr, GTSManagement& management, GTSRequestCmd& requestCmd) {
    this->deviceAddr = deviceAddr;
    this->management = management;
    this->requestCmd = requestCmd;
}

void GTSEvent::fill(IDSMEMessage* msg, GTSManagement& management, GTSReplyNotifyCmd& replyNotifyCmd) {
    this->deviceAddr = msg->getHeader().getSrcAddr().getShortAddress();
    this->header = msg->getHeader();
    this->management = management;
    this->replyNotifyCmd = replyNotifyCmd;
}

GTSManager::GTSManager(DSMELayer& dsme) : GTSManagerFSM_t(&GTSManager::stateIdle, &GTSManager::stateBusy), dsme(dsme), actUpdater(dsme) {
}

void GTSManager::initialize() {
    dsme.getMAC_PIB().macDSMESAB.initialize(dsme.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe(), dsme.getMAC_PIB().helper.getNumGTSlots(0),
                                            dsme.getMAC_PIB().helper.getNumGTSlots(1), dsme.getMAC_PIB().helper.getNumChannels());
    dsme.getMAC_PIB().macDSMEACT.initialize(dsme.getMAC_PIB().helper.getNumberSuperframesPerMultiSuperframe(), dsme.getMAC_PIB().helper.getNumGTSlots(0),
                                            dsme.getMAC_PIB().helper.getNumGTSlots(1), dsme.getMAC_PIB().helper.getNumChannels(), &dsme);
    return;
}

void GTSManager::reset() {
    DSME_ATOMIC_BLOCK {
        for(uint8_t i = 0; i < GTS_STATE_MULTIPLICITY; ++i) {
            transition(i, &GTSManager::stateIdle);
            this->data[i].msgToSend = nullptr;
        }
    }

    this->dsme.getMAC_PIB().macDSMESAB.clear();
    this->dsme.getMAC_PIB().macDSMEACT.clear();
}

/*****************************
 * States
 *****************************/
fsmReturnStatus GTSManager::stateBusy(GTSEvent& event) {
    int8_t fsmId = event.getFsmId();
    DSME_ASSERT(fsmId == GTS_STATE_MULTIPLICITY);

    LOG_DEBUG(signalToString(event.signal) << "-" << stateToString(&GTSManager::stateBusy) << "[" << (uint16_t)fsmId << "]");

    switch(event.signal) {
        case GTSEvent::ENTRY_SIGNAL:
            return FSM_IGNORED;
        case GTSEvent::EXIT_SIGNAL:
            DSME_ASSERT(false);
            return FSM_IGNORED;
        case GTSEvent::MLME_REQUEST_ISSUED:
            actionReportBusyNotify(event);
            return FSM_IGNORED;
        case GTSEvent::MLME_RESPONSE_ISSUED:
            actionSendImmediateNegativeResponse(event);
            actionReportBusyCommStatus(event);
            return FSM_HANDLED;
        case GTSEvent::SEND_COMPLETE:
            LOG_INFO("Outdated message");
            return FSM_IGNORED;
        default:
            return FSM_IGNORED;
    }
}

fsmReturnStatus GTSManager::stateIdle(GTSEvent& event) {
    int8_t fsmId = event.getFsmId();
    if(event.signal != GTSEvent::CFP_STARTED) {
        LOG_DEBUG(signalToString(event.signal) << "-" << stateToString(&GTSManager::stateIdle) << "[" << (uint16_t)fsmId << "]");
    }

    switch(event.signal) {
        case GTSEvent::ENTRY_SIGNAL:
        case GTSEvent::EXIT_SIGNAL:
            return FSM_IGNORED;
        case GTSEvent::MLME_REQUEST_ISSUED: {
            preparePendingConfirm(event);

            IDSMEMessage* msg = dsme.getPlatform().getEmptyMessage();

            //add gackGTS parameters to Header IEList
            if(event.requestCmd.isGackGTS()){
                gackEnabledIE *gackIE = new gackEnabledIE();
                DSME_ASSERT(gackIE != nullptr);
                gackIE->gackEnabled = true;
                msg->getHeader().getIEList().insert(gackIE);
            }
            event.requestCmd.prependTo(msg);

            if(!sendGTSCommand(fsmId, msg, event.management, CommandFrameIdentifier::DSME_GTS_REQUEST, event.deviceAddr)) {
                dsme.getPlatform().releaseMessage(msg);

                LOG_INFO("TRANSACTION_OVERFLOW");
                data[fsmId].pendingConfirm.status = GTSStatus::TRANSACTION_OVERFLOW;

                this->dsme.getMLME_SAP().getDSME_GTS().notify_confirm(data[fsmId].pendingConfirm);
                return FSM_HANDLED;
            } else {
                return transition(fsmId, &GTSManager::stateSending);
            }
        }

        case GTSEvent::MLME_RESPONSE_ISSUED: {
            preparePendingConfirm(event);

            IDSMEMessage* msg = dsme.getPlatform().getEmptyMessage();

            event.replyNotifyCmd.prependTo(msg);

            //add gackGTS parameters to Header IEList
            if(event.replyNotifyCmd.getGackGTS()!=GTS::UNDEFINED){
                gackResponseIE *gackRspIE = new gackResponseIE();
                DSME_ASSERT(gackRspIE != nullptr);
                gackRspIE->superframeID = event.replyNotifyCmd.getGackGTS().superframeID;
                gackRspIE->slotID = event.replyNotifyCmd.getGackGTS().slotID;
                gackRspIE->channelIndex = event.replyNotifyCmd.getGackGTS().channel;
                msg->getHeader().getIEList().insert(gackRspIE);
            }

            uint16_t destinationShortAddress;

            if(event.management.status == GTSStatus::SUCCESS) {
                LOG_INFO("Positive GTS response " << event.replyNotifyCmd.getDestinationAddress());

                destinationShortAddress = IEEE802154MacAddress::SHORT_BROADCAST_ADDRESS;
            } else {
                LOG_INFO("Negative GTS response " << event.replyNotifyCmd.getDestinationAddress());

                destinationShortAddress = event.replyNotifyCmd.getDestinationAddress();
            }

            if(!sendGTSCommand(fsmId, msg, event.management, CommandFrameIdentifier::DSME_GTS_REPLY, destinationShortAddress)) {
                LOG_INFO("Could not send REPLY");
                dsme.getPlatform().releaseMessage(msg);

                mlme_sap::COMM_STATUS_indication_parameters params;
                // TODO also fill other fields
                params.status = CommStatus::Comm_Status::TRANSACTION_OVERFLOW;
                this->dsme.getMLME_SAP().getCOMM_STATUS().notify_indication(params);
                return FSM_HANDLED;
            } else {
                if(event.management.status == GTSStatus::SUCCESS) {
                    actUpdater.approvalQueued(event.replyNotifyCmd.getSABSpec(), event.management, event.deviceAddr, dsme.getMAC_PIB().macChannelOffset);
                    //TODO: transmit gackGTS information to actUpdater here
                }
                return transition(fsmId, &GTSManager::stateSending);
            }
        }

        case GTSEvent::RESPONSE_CMD_FOR_ME:
        case GTSEvent::NOTIFY_CMD_FOR_ME:
        case GTSEvent::SEND_COMPLETE:
            DSME_ASSERT(false);
            return FSM_IGNORED;

        case GTSEvent::CFP_STARTED: {
            // check if a slot should be deallocated, only if no reply or notify is pending
            for(DSMEAllocationCounterTable::iterator it = dsme.getMAC_PIB().macDSMEACT.begin(); it != dsme.getMAC_PIB().macDSMEACT.end(); ++it) {
                // Since no reply is pending, this slot should have been removed already and is no longer in the ACT
                // This should be even the case for timeouts (NO_DATA indication for upper layer)
                DSME_ASSERT(it->getState() != DEALLOCATED);
                DSME_ASSERT(it->getState() != REMOVED);

                if(it->isGackGTS()){   //skip GACK-GTS
                    continue;
                }

                LOG_DEBUG("check slot " << (uint16_t)it->getGTSlotID() << " " << it->getSuperframeID() << " " << (uint16_t)it->getChannel() << " ["
                                        << this->dsme.getMAC_PIB().macShortAddress << (const char*)((it->getDirection() == Direction::TX) ? ">" : "<")
                                        << it->getAddress() << ", " << it->getIdleCounter() << "]");

                // TODO Since INVALID is not included in the standard, use the EXPIRATION type for INVALID, too.
                //      The effect should be the same.
                if(it->getState() == INVALID || it->getState() == UNCONFIRMED || it->getIdleCounter() > dsme.getMAC_PIB().macDSMEGTSExpirationTime) {
                    if(it->getState() == INVALID) {
                        LOG_INFO("DEALLOCATE: Due to state INVALID");
                    } else if(it->getState() == UNCONFIRMED) {
                        if(hasBusyFsm()) {
                            continue;
                        }
                        LOG_INFO("DEALLOCATE: Due to state UNCONFIRMED");
                    } else if(it->getIdleCounter() > dsme.getMAC_PIB().macDSMEGTSExpirationTime) {
                        it->resetIdleCounter();
                        LOG_INFO("DEALLOCATE: Due to expiration");
                    } else {
                        DSME_ASSERT(false);
                    }

                    mlme_sap::DSME_GTS_indication_parameters params;
                    params.deviceAddress = it->getAddress();

                    params.managementType = EXPIRATION;
                    params.direction = it->getDirection();
                    params.prioritizedChannelAccess = Priority::LOW;
                    params.numSlot = 1;

                    uint8_t subBlockLengthBytes = dsme.getMAC_PIB().helper.getSubBlockLengthBytes(it->getSuperframeID());
                    params.dsmeSabSpecification.setSubBlockLengthBytes(subBlockLengthBytes);
                    params.dsmeSabSpecification.setSubBlockIndex(it->getSuperframeID());
                    params.dsmeSabSpecification.getSubBlock().fill(false);
                    params.dsmeSabSpecification.getSubBlock().set(it->getGTSlotID() * dsme.getMAC_PIB().helper.getNumChannels() + it->getChannel(), true);

                    this->dsme.getMLME_SAP().getDSME_GTS().notify_indication(params);
                    break;
                }
            }

            return FSM_HANDLED;
        }

        default:
            DSME_ASSERT(false);
            return FSM_IGNORED;
    }
}

fsmReturnStatus GTSManager::stateSending(GTSEvent& event) {
    int8_t fsmId = event.getFsmId();
    LOG_DEBUG(signalToString(event.signal) << "-" << stateToString(&GTSManager::stateSending) << "[" << (uint16_t)fsmId << "]");

    switch(event.signal) {
        case GTSEvent::ENTRY_SIGNAL:
        case GTSEvent::EXIT_SIGNAL:
            return FSM_IGNORED;

        case GTSEvent::CFP_STARTED:
            // pending SEND_COMPLETE (e.g. delay due to serial output)
            LOG_ERROR("CFP during sending");
            return FSM_IGNORED;

        case GTSEvent::MLME_REQUEST_ISSUED:
        case GTSEvent::MLME_RESPONSE_ISSUED:
        case GTSEvent::RESPONSE_CMD_FOR_ME:
        case GTSEvent::NOTIFY_CMD_FOR_ME:
            LOG_ERROR(signalToString(event.signal) << "-" << stateToString(&GTSManager::stateSending) << "[" << (uint16_t)fsmId << "]");
            DSME_ASSERT(false);
            return FSM_IGNORED;

        case GTSEvent::SEND_COMPLETE: {
            DSME_ASSERT(event.cmdId == DSME_GTS_REQUEST || event.cmdId == DSME_GTS_REPLY || event.cmdId == DSME_GTS_NOTIFY);
            DSME_ASSERT(event.cmdId == data[fsmId].cmdToSend);

            if(event.cmdId == DSME_GTS_NOTIFY) {
                actUpdater.notifyDelivered(event.replyNotifyCmd.getSABSpec(), event.management, event.deviceAddr, event.replyNotifyCmd.getChannelOffset(), event.replyNotifyCmd.getGackGTS());
                return transition(fsmId, &GTSManager::stateIdle);
            } else if(event.cmdId == DSME_GTS_REQUEST) {
                if(event.dataStatus != DataStatus::Data_Status::SUCCESS) {
                    LOG_DEBUG("GTSManager sending request failed " << (uint16_t)event.dataStatus);

                    switch(event.dataStatus) {
                        case DataStatus::NO_ACK:
                            actUpdater.requestNoAck(event.requestCmd.getSABSpec(), event.management, event.deviceAddr);
                            data[fsmId].pendingConfirm.status = GTSStatus::NO_ACK;
                            break;
                        case DataStatus::CHANNEL_ACCESS_FAILURE:
                            actUpdater.requestAccessFailure(event.requestCmd.getSABSpec(), event.management, event.deviceAddr);
                            data[fsmId].pendingConfirm.status = GTSStatus::CHANNEL_ACCESS_FAILURE;
                            break;
                        case DataStatus::TRANSACTION_EXPIRED:
                            actUpdater.requestAccessFailure(event.requestCmd.getSABSpec(), event.management, event.deviceAddr);
                            data[fsmId].pendingConfirm.status = GTSStatus::TRANSACTION_OVERFLOW; // TODO TRANSACTION_EXPIRED not available!
                            break;
                        default:
                            DSME_ASSERT(false);
                    }

                    this->dsme.getMLME_SAP().getDSME_GTS().notify_confirm(data[fsmId].pendingConfirm);
                    return transition(fsmId, &GTSManager::stateIdle);
                } else {
                    // REQUEST_SUCCESS
                    data[fsmId].responsePartnerAddress = event.deviceAddr;
                    return transition(fsmId, &GTSManager::stateWaitForResponse);
                }
            } else if(event.cmdId == DSME_GTS_REPLY) {
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
                        case DataStatus::TRANSACTION_EXPIRED:
                        case DataStatus::CHANNEL_ACCESS_FAILURE:
                            if(event.management.status == GTSStatus::SUCCESS) {
                                actUpdater.approvalAccessFailure(event.replyNotifyCmd.getSABSpec(), event.management, event.deviceAddr);
                            } else if(event.management.status == GTSStatus::DENIED) {
                                actUpdater.disapprovalAccessFailure(event.replyNotifyCmd.getSABSpec(), event.management, event.deviceAddr);
                            } else {
                                DSME_ASSERT(false);
                            }

                            if(event.dataStatus == DataStatus::CHANNEL_ACCESS_FAILURE) {
                                params.status = CommStatus::Comm_Status::TRANSACTION_EXPIRED;
                            } else {
                                params.status = CommStatus::Comm_Status::CHANNEL_ACCESS_FAILURE;
                            }
                            break;
                        default:
                            DSME_ASSERT(false);
                    }

                    this->dsme.getMLME_SAP().getCOMM_STATUS().notify_indication(params);
                    return transition(fsmId, &GTSManager::stateIdle);
                } else {
                    if(event.management.status == GTSStatus::SUCCESS) {
                        actUpdater.approvalDelivered(event.replyNotifyCmd.getSABSpec(), event.management, event.deviceAddr, dsme.getMAC_PIB().macChannelOffset);
                        data[fsmId].notifyPartnerAddress = event.deviceAddr;
                        return transition(fsmId, &GTSManager::stateWaitForNotify);
                    } else if(event.management.status == GTSStatus::DENIED) {
                        actUpdater.disapprovalDelivered(event.replyNotifyCmd.getSABSpec(), event.management, event.deviceAddr);

                        // for disapprovals, no notify is expected
                        return transition(fsmId, &GTSManager::stateIdle);
                    } else {
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
    int8_t fsmId = event.getFsmId();
    LOG_DEBUG(signalToString(event.signal) << "-" << stateToString(&GTSManager::stateWaitForResponse) << "[" << (uint16_t)fsmId << "]");

    switch(event.signal) {
        case GTSEvent::ENTRY_SIGNAL:
            data[fsmId].superframesInCurrentState = 0;
            return FSM_HANDLED;
        case GTSEvent::EXIT_SIGNAL:
            return FSM_IGNORED;

        case GTSEvent::MLME_REQUEST_ISSUED:
        case GTSEvent::MLME_RESPONSE_ISSUED:
        case GTSEvent::NOTIFY_CMD_FOR_ME:
        case GTSEvent::SEND_COMPLETE:
            DSME_ASSERT(false);
            return FSM_IGNORED;

        case GTSEvent::RESPONSE_CMD_FOR_ME: {
            mlme_sap::DSME_GTS_confirm_parameters params;
            params.deviceAddress = event.deviceAddr;
            params.managementType = event.management.type;
            params.direction = event.management.direction;
            params.prioritizedChannelAccess = event.management.prioritizedChannelAccess;
            params.dsmeSabSpecification = event.replyNotifyCmd.getSABSpec();
            params.channelOffset = event.replyNotifyCmd.getChannelOffset();

            // TODO // if the ACK gets lost, the reply might be sent anyway, so we might be in SENDING_REQUEST
            // TODO DSME_ASSERT((state == State::SENDING && cmdToSend == DSME_GTS_REQUEST) || state == State::WAIT_FOR_REPLY);

            if(data[fsmId].pendingConfirm.deviceAddress != params.deviceAddress) {
                LOG_INFO("Wrong response handled! Got address " << params.deviceAddress << " instead of " << data[fsmId].pendingConfirm.deviceAddress);
                // DSME_ASSERT(false);
                return FSM_HANDLED;
            }
            if(data[fsmId].pendingConfirm.managementType != params.managementType) {
                LOG_INFO("Wrong response handled! Got type " << (int16_t)params.managementType << " instead of "
                                                             << (int16_t)data[fsmId].pendingConfirm.managementType);
                // DSME_ASSERT(false);
                return FSM_HANDLED;
            }
            if(data[fsmId].pendingConfirm.direction != params.direction) {
                LOG_INFO("Wrong response handled! Got direction " << (int16_t)params.direction << " instead of "
                                                                  << (int16_t)data[fsmId].pendingConfirm.direction);
                // DSME_ASSERT(false);
                return FSM_HANDLED;
            }

            params.status = event.management.status;

            if(event.management.status == GTSStatus::SUCCESS) {
                if(event.management.type == ALLOCATION) {
                    if(checkAndHandleGTSDuplicateAllocation(event.replyNotifyCmd.getSABSpec(), event.deviceAddr, true)) { // TODO issue #3
                        uint8_t numSlotsOk = event.replyNotifyCmd.getSABSpec().getSubBlock().count(true);

                        if(numSlotsOk == 0) {
                            event.management.status = GTSStatus::DENIED;
                        } else {
                            DSME_ASSERT(false); /* This case is not handled properly, better use only one slot per request */
                        }
                    } else {
                        actUpdater.approvalReceived(event.replyNotifyCmd.getSABSpec(), event.management, event.deviceAddr,
                                                    event.replyNotifyCmd.getChannelOffset());
                    }
                }
            }

            this->dsme.getMLME_SAP().getDSME_GTS().notify_confirm(params);

            if(event.management.status == GTSStatus::SUCCESS) {
                /* the requesting node has to notify its one hop neighbors */
                IDSMEMessage* msg_notify = dsme.getPlatform().getEmptyMessage();
                event.replyNotifyCmd.setDestinationAddress(event.deviceAddr);
                event.replyNotifyCmd.prependTo(msg_notify);

                //add gackGTS parameters to Header IEList
                if(event.replyNotifyCmd.getGackGTS()!=GTS::UNDEFINED){
                    gackResponseIE *gackRspIE = new gackResponseIE();
                    DSME_ASSERT(gackRspIE != nullptr);
                    gackRspIE->superframeID = event.replyNotifyCmd.getGackGTS().superframeID;
                    gackRspIE->slotID = event.replyNotifyCmd.getGackGTS().slotID;
                    gackRspIE->channelIndex = event.replyNotifyCmd.getGackGTS().channel;
                    msg_notify->getHeader().getIEList().insert(gackRspIE);
                }

                if(!sendGTSCommand(fsmId, msg_notify, event.management, CommandFrameIdentifier::DSME_GTS_NOTIFY,
                                   IEEE802154MacAddress::SHORT_BROADCAST_ADDRESS)) {
                    // TODO should this be signaled to the upper layer?
                    LOG_INFO("NOTIFY could not be sent");
                    actUpdater.notifyAccessFailure(event.replyNotifyCmd.getSABSpec(), event.management, event.deviceAddr);
                    dsme.getPlatform().releaseMessage(msg_notify);
                    return transition(fsmId, &GTSManager::stateIdle);
                } else {
                    return transition(fsmId, &GTSManager::stateSending);
                }
            } else if(event.management.status == GTSStatus::NO_DATA) { // misuse NO_DATA to signal that the destination was busy
                // actUpdater.requestAccessFailure(event.requestCmd.getSABSpec(), event.management, event.deviceAddr);
                actUpdater.responseTimeout(event.requestCmd.getSABSpec(), event.management, event.deviceAddr);
                return transition(fsmId, &GTSManager::stateIdle);
            } else {
                DSME_ASSERT(event.management.status == GTSStatus::DENIED);
                actUpdater.disapproved(event.replyNotifyCmd.getSABSpec(), event.management, event.deviceAddr, event.replyNotifyCmd.getChannelOffset(), event.replyNotifyCmd.getGackGTS());
                return transition(fsmId, &GTSManager::stateIdle);
            }
        }

        case GTSEvent::CFP_STARTED: {
            if(isTimeoutPending(fsmId)) {
                LOG_INFO("GTS timeout for response");
                mlme_sap::DSME_GTS_confirm_parameters& pendingConfirm = data[fsmId].pendingConfirm;

                actUpdater.responseTimeout(pendingConfirm.dsmeSabSpecification, data[fsmId].pendingManagement, pendingConfirm.deviceAddress);
                pendingConfirm.status = GTSStatus::GTS_Status::NO_DATA;
                this->dsme.getMLME_SAP().getDSME_GTS().notify_confirm(pendingConfirm);
                return transition(fsmId, &GTSManager::stateIdle);
            } else {
                return FSM_HANDLED;
            }
        }
        default:
            DSME_ASSERT(false);
            return FSM_IGNORED;
    }
}

fsmReturnStatus GTSManager::stateWaitForNotify(GTSEvent& event) {
    int8_t fsmId = event.getFsmId();
    LOG_DEBUG(signalToString(event.signal) << "-" << stateToString(&GTSManager::stateWaitForNotify) << "[" << (uint16_t)fsmId << "]");

    switch(event.signal) {
        case GTSEvent::ENTRY_SIGNAL:
            data[fsmId].superframesInCurrentState = 0;
            return FSM_HANDLED;
        case GTSEvent::EXIT_SIGNAL:
            return FSM_IGNORED;

        case GTSEvent::MLME_REQUEST_ISSUED:
        case GTSEvent::MLME_RESPONSE_ISSUED:
        case GTSEvent::RESPONSE_CMD_FOR_ME:
        case GTSEvent::SEND_COMPLETE:
            DSME_ASSERT(false);
            return FSM_IGNORED;

        case GTSEvent::NOTIFY_CMD_FOR_ME: {
            // TODO! DSME_ASSERT((state == State::SENDING && cmdToSend == DSME_GTS_REPLY) || state == State::WAIT_FOR_NOTIFY); // TODO what if the notify comes
            // too late, probably send a deallocation again???
            actUpdater.notifyReceived(event.replyNotifyCmd.getSABSpec(), event.management, event.deviceAddr, event.replyNotifyCmd.getChannelOffset(), event.replyNotifyCmd.getGackGTS());

            /* If the DSME-GTS Destination address is the same as the macShortAddress, the device shall notify the next higher
             * layer of the receipt of the DSME-GTS notify command frame using MLME-COMM- STATUS.indication */
            // TODO also for DEALLOCATION?
            mlme_sap::COMM_STATUS_indication_parameters params;
            params.panId = event.header.getSrcPANId();
            params.srcAddrMode = event.header.getFrameControl().srcAddrMode;
            params.srcAddr = event.header.getSrcAddr();
            params.dstAddrMode = event.header.getFrameControl().dstAddrMode;
            params.dstAddr = event.header.getDestAddr(); // TODO: Header destination address of GTS destination address?
            params.status = CommStatus::SUCCESS;

            dsme.getMLME_SAP().getCOMM_STATUS().notify_indication(params);

            return transition(fsmId, &GTSManager::stateIdle);
        }

        case GTSEvent::CFP_STARTED: {
            if(isTimeoutPending(fsmId)) {
                LOG_INFO("GTS timeout for notify");
                actUpdater.notifyTimeout(data[fsmId].pendingConfirm.dsmeSabSpecification, data[fsmId].pendingManagement,
                                         data[fsmId].pendingConfirm.deviceAddress, data[fsmId].pendingConfirm.channelOffset);

                mlme_sap::COMM_STATUS_indication_parameters params;
                // TODO also fill other fields
                params.status = CommStatus::Comm_Status::TRANSACTION_EXPIRED;
                this->dsme.getMLME_SAP().getCOMM_STATUS().notify_indication(params);
                return transition(fsmId, &GTSManager::stateIdle);
            } else {
                return FSM_HANDLED;
            }
        }

        default:
            DSME_ASSERT(false);
            return FSM_IGNORED;
    }
}

/*****************************
 * Actions
 *****************************/

const char* GTSManager::signalToString(uint8_t signal) {
    switch(signal) {
        case GTSEvent::EMPTY_SIGNAL:
            // return "EMPTY_SIGNAL";
            return "EM";
        case GTSEvent::ENTRY_SIGNAL:
            // return "ENTRY_SIGNAL";
            return "EN";
        case GTSEvent::EXIT_SIGNAL:
            // return "EXIT_SIGNAL";
            return "EX";
        case GTSEvent::MLME_REQUEST_ISSUED:
            // return "MLME_REQUEST_ISSUED";
            return "REQ";
        case GTSEvent::MLME_RESPONSE_ISSUED:
            // return "MLME_RESPONSE_ISSUED";
            return "RES";
        case GTSEvent::RESPONSE_CMD_FOR_ME:
            // return "RESPONSE_CMD_FOR_ME";
            return "RESFM";
        case GTSEvent::NOTIFY_CMD_FOR_ME:
            // return "NOTIFY_CMD_FOR_ME";
            return "NO";
        case GTSEvent::CFP_STARTED:
            // return "CFP_STARTED";
            return "CFP";
        case GTSEvent::SEND_COMPLETE:
            // return "SEND_COMPLETE";
            return "CO";
        default:
            DSME_ASSERT(false);
            return nullptr;
    }
}

const char* GTSManager::stateToString(GTSManagerFSM_t::state_t state) {
    if(state == &GTSManager::stateBusy) {
        return "BUSY";
    } else if(state == &GTSManager::stateIdle) {
        return "IDLE";
    } else if(state == &GTSManager::stateSending) {
        // return "SENDING";
        return "SEND";
    } else if(state == &GTSManager::stateWaitForResponse) {
        // return "WAITFORRESPONSE";
        return "WAITR";
    } else if(state == &GTSManager::stateWaitForNotify) {
        // return "WAITFORNOTIFY";
        return "WAITN";
    } else {
        DSME_ASSERT(false);
        return nullptr;
    }
}

void GTSManager::actionSendImmediateNegativeResponse(GTSEvent& event) {
    int8_t fsmId = event.getFsmId();
    DSME_ASSERT(event.signal == GTSEvent::MLME_RESPONSE_ISSUED);

    IDSMEMessage* msg = dsme.getPlatform().getEmptyMessage();
    event.replyNotifyCmd.prependTo(msg);

    //add gackGTS parameters to Header IEList
    if(event.replyNotifyCmd.getGackGTS()!=GTS::UNDEFINED){
        gackResponseIE *gackRspIE = new gackResponseIE();
        DSME_ASSERT(gackRspIE != nullptr);
        gackRspIE->superframeID = event.replyNotifyCmd.getGackGTS().superframeID;
        gackRspIE->slotID = event.replyNotifyCmd.getGackGTS().slotID;
        gackRspIE->channelIndex = event.replyNotifyCmd.getGackGTS().channel;
        msg->getHeader().getIEList().insert(gackRspIE);
    }

    LOG_INFO("Negative GTS response " << event.replyNotifyCmd.getDestinationAddress() << " TRANSACTION_OVERFLOW");
    uint16_t destinationShortAddress = event.replyNotifyCmd.getDestinationAddress();
    event.management.status = GTSStatus::NO_DATA; // misuse NO_DATA to signal that the destination was busy
    if(!sendGTSCommand(fsmId, msg, event.management, CommandFrameIdentifier::DSME_GTS_REPLY, destinationShortAddress, false)) {
        LOG_INFO("Could not send REPLY");
        dsme.getPlatform().releaseMessage(msg);
    }
}

void GTSManager::actionReportBusyNotify(GTSEvent& event) {
    // LOG_INFO("BusyNotify on event '" << signalToString(event.signal) << "-" << stateToString(this->getState()) );

    mlme_sap::DSME_GTS_confirm_parameters busyConfirm;
    busyConfirm.deviceAddress = event.deviceAddr;
    busyConfirm.managementType = event.management.type;
    busyConfirm.direction = event.management.direction;
    busyConfirm.prioritizedChannelAccess = event.management.prioritizedChannelAccess;
    busyConfirm.dsmeSabSpecification = event.requestCmd.getSABSpec();
    busyConfirm.status = GTSStatus::TRANSACTION_OVERFLOW;
    this->dsme.getMLME_SAP().getDSME_GTS().notify_confirm(busyConfirm);
}

void GTSManager::actionReportBusyCommStatus(GTSEvent& event) {
    // LOG_INFO("BusyCommstatus on event '" << signalToString(event.signal) << "-" << stateToString(this->getState()) );

    mlme_sap::COMM_STATUS_indication_parameters params;
    // TODO also fill other fields
    params.status = CommStatus::Comm_Status::TRANSACTION_OVERFLOW;
    this->dsme.getMLME_SAP().getCOMM_STATUS().notify_indication(params);
    return;
}

/*****************************
 * External interfaces
 *****************************/

bool GTSManager::handleMLMERequest(uint16_t deviceAddr, GTSManagement& man, GTSRequestCmd& cmd) {
    int8_t fsmId = getFsmIdForRequest();
    return dispatch(fsmId, GTSEvent::MLME_REQUEST_ISSUED, deviceAddr, man, cmd);
}

bool GTSManager::handleMLMEResponse(GTSManagement& man, GTSReplyNotifyCmd& reply) {
    uint16_t destinationAddress = reply.getDestinationAddress();
    int8_t fsmId = getFsmIdForResponse(destinationAddress);
    return dispatch(fsmId, GTSEvent::MLME_RESPONSE_ISSUED, destinationAddress, man, reply);
}

bool GTSManager::handleGTSRequest(IDSMEMessage* msg) {
    // This can be directly passed to the upper layer.
    // There is no need to go over the state machine!
    uint16_t sourceAddr = msg->getHeader().getSrcAddr().getShortAddress();
    GTSManagement man;
    man.decapsulateFrom(msg);
    GTSRequestCmd req;
    req.decapsulateFrom(msg);

    mlme_sap::DSME_GTS_indication_parameters params;
    params.deviceAddress = sourceAddr;
    params.managementType = man.type;
    params.direction = man.direction;
    params.prioritizedChannelAccess = man.prioritizedChannelAccess;
    params.numSlot = req.getNumSlots();
    params.preferredSuperframeId = req.getPreferredSuperframeID();
    params.preferredSlotId = req.getPreferredSlotID();
    params.dsmeSabSpecification = req.getSABSpec();
    params.gackGTS = false;

    //Checks if message contains gackEnabled flag
    if(msg->getHeader().getIEListPresent()){
        gackEnabledIE *gackIE = (gackEnabledIE*) msg->getHeader().getIEList().getIEByID(InformationElement::ID_gackEnabled);
        if(gackIE != nullptr){
            params.gackGTS = gackIE->gackEnabled;
        }
    }

    if(man.type == ManagementType::DUPLICATED_ALLOCATION_NOTIFICATION) {
        dsme.getMAC_PIB().macDSMESAB.addOccupiedSlots(req.getSABSpec());
        actUpdater.duplicateAllocation(req.getSABSpec(), 0);
    }

    this->dsme.getMLME_SAP().getDSME_GTS().notify_indication(params);
    return true;
}

bool GTSManager::handleGTSResponse(IDSMEMessage* msg) {
    GTSManagement management;
    GTSReplyNotifyCmd replyNotifyCmd;
    management.decapsulateFrom(msg);
    replyNotifyCmd.decapsulateFrom(msg);

    //Checks if message contains gackRspIE
    if(msg->getHeader().getIEListPresent()){
        gackResponseIE *gackRspIE = (gackResponseIE*) msg->getHeader().getIEList().getIEByID(InformationElement::ID_gackResponse);
        if(gackRspIE != nullptr){
            replyNotifyCmd.setGackGTS(GTS(gackRspIE->superframeID,gackRspIE->slotID, gackRspIE->channelIndex));
        }
    }

    if(replyNotifyCmd.getDestinationAddress() == dsme.getMAC_PIB().macShortAddress) {
        int8_t fsmId = getFsmIdFromResponseForMe(msg);
        data[fsmId].responsePartnerAddress = IEEE802154MacAddress::NO_SHORT_ADDRESS;
        return dispatch(fsmId, GTSEvent::RESPONSE_CMD_FOR_ME, msg, management, replyNotifyCmd);
    } else if(management.status == GTSStatus::SUCCESS) {
        // Response overheared -> Add to the SAB regardless of the current state
        if(management.type == ManagementType::ALLOCATION) {
            if(!checkAndHandleGTSDuplicateAllocation(replyNotifyCmd.getSABSpec(), msg->getHeader().getSrcAddr().getShortAddress(), false)) {
                // If there is no conflict, the device shall update macDSMESAB according to the DSMESABSpecification in this
                // command frame to reflect the neighbor's newly allocated DSME-GTSs
                this->dsme.getMAC_PIB().macDSMESAB.addOccupiedSlots(replyNotifyCmd.getSABSpec());
            }
        } else if(management.type == ManagementType::DEALLOCATION) {
            this->dsme.getMAC_PIB().macDSMESAB.removeOccupiedSlots(replyNotifyCmd.getSABSpec());
        }
    } else {
        // A denied request should not be sent via broadcast!
        LOG_ERROR(management.status << " " << replyNotifyCmd.getDestinationAddress() << " " << dsme.getMAC_PIB().macShortAddress);
        DSME_ASSERT(false);
    }

    return true;
}

bool GTSManager::handleGTSNotify(IDSMEMessage* msg) {
    GTSManagement management;

    management.decapsulateFrom(msg);

    if(management.type != ManagementType::ALLOCATION && management.type != ManagementType::DEALLOCATION) {
        return true;
    }

    GTSReplyNotifyCmd replyNotifyCmd;
    replyNotifyCmd.decapsulateFrom(msg);

    //Checks if message contains gackRspIE
    if(msg->getHeader().getIEListPresent()){
        gackResponseIE *gackRspIE = (gackResponseIE*) msg->getHeader().getIEList().getIEByID(InformationElement::ID_gackResponse);
        if(gackRspIE != nullptr){
            replyNotifyCmd.setGackGTS(GTS(gackRspIE->superframeID,gackRspIE->slotID, gackRspIE->channelIndex));
        }
    }

    if(replyNotifyCmd.getDestinationAddress() == dsme.getMAC_PIB().macShortAddress) {
        int8_t fsmId = getFsmIdFromNotifyForMe(msg);
        data[fsmId].notifyPartnerAddress = IEEE802154MacAddress::NO_SHORT_ADDRESS;
        return dispatch(fsmId, GTSEvent::NOTIFY_CMD_FOR_ME, msg, management, replyNotifyCmd);
    } else {
        // Notify overheared -> Add to the SAB regardless of the current state
        if(management.type == ManagementType::ALLOCATION) {
            if(!checkAndHandleGTSDuplicateAllocation(replyNotifyCmd.getSABSpec(), msg->getHeader().getSrcAddr().getShortAddress(), false)) {
                // If there is no conflict, the device shall update macDSMESAB according to the DSMESABSpecification in this
                // command frame to reflect the neighbor's newly allocated DSME-GTSs
                this->dsme.getMAC_PIB().macDSMESAB.addOccupiedSlots(replyNotifyCmd.getSABSpec());
            }
        } else if(management.type == ManagementType::DEALLOCATION) {
            this->dsme.getMAC_PIB().macDSMESAB.removeOccupiedSlots(replyNotifyCmd.getSABSpec());
        }
    }
    return true;
}

bool GTSManager::handleStartOfCFP(uint8_t superframe) {
    for(uint8_t i = 0; i < GTS_STATE_MULTIPLICITY; ++i) {
        data[i].superframesInCurrentState++;
    }

    // also execute this during non-idle phases
    if(superframe == 0) {
        for(DSMEAllocationCounterTable::iterator it = dsme.getMAC_PIB().macDSMEACT.begin(); it != dsme.getMAC_PIB().macDSMEACT.end(); ++it) {
            if(it->getDirection() == Direction::RX) {
                // New multi-superframe started, so increment the idle counter according to 5.1.10.5.3
                it->incrementIdleCounter(); // gets reset to zero on RX
            }
        }
    }

    for(uint8_t i = 0; i < GTS_STATE_MULTIPLICITY; ++i) {
        if(getState(i) == &GTSManager::stateWaitForNotify || getState(i) == &GTSManager::stateWaitForResponse) {
            dispatch(i, GTSEvent::CFP_STARTED);
        }
    }

    int8_t fsmId = getFsmIdIdle();
    if(fsmId >= 0) {
        return dispatch(fsmId, GTSEvent::CFP_STARTED);
    } else {
        return true;
    }
}

bool GTSManager::onCSMASent(IDSMEMessage* msg, CommandFrameIdentifier cmdId, DataStatus::Data_Status status, uint8_t numBackoffs) {
    GTSManagement management;
    management.decapsulateFrom(msg);

    bool returnStatus;
    if(management.type == ManagementType::DUPLICATED_ALLOCATION_NOTIFICATION) {
        // Sending the duplicate allocation is not handled by the state machine, since
        // it is just a state-less notification (at least for our interpretation of the standard)
        LOG_INFO("DUPLICATED_ALLOCATION_NOTIFICATION sent");
        returnStatus = true;
    } else {
        int8_t validFsmId = -1;

        for(uint8_t i = 0; i < GTS_STATE_MULTIPLICITY; ++i) {
            // Check which statemachine waits for this msg
            if(data[i].msgToSend == msg) {
                data[i].msgToSend = nullptr;
                DSME_ASSERT(getState(i) == &GTSManager::stateSending ||
                            getState(i) == &GTSManager::stateIdle); // The FSM might still execute the sendGTSCommand in the IDLE state
                validFsmId = i;
                break;
            }
        }

        if(validFsmId >= 0 && validFsmId < GTS_STATE_MULTIPLICITY) {
            if(status != DataStatus::SUCCESS) {
                LOG_DEBUG("GTSManager::onCSMASent transmission failure: " << (int16_t)status);
            }
            returnStatus = dispatch(validFsmId, GTSEvent::SEND_COMPLETE, msg, management, cmdId, status);
        } else {
            // If the ACK was lost, but the message itself was delivered successfully,
            // the RESPONSE or NOTIFY might already have been handled properly.
            // Same holds if the current state is not sending (see there)
            // TODO What about the states of the receiver and the neighbours?
            LOG_DEBUG("Outdated message");
            returnStatus = true;
        }
    }

    dsme.getPlatform().releaseMessage(msg);
    return returnStatus;
}

/*****************************
 * Internal helpers
 *****************************/

bool GTSManager::checkAndHandleGTSDuplicateAllocation(DSMESABSpecification& sabSpec, uint16_t addr, bool allChannels) {
    DSMEAllocationCounterTable& macDSMEACT = this->dsme.getMAC_PIB().macDSMEACT;

    bool duplicateFound = false;

    GTSRequestCmd dupReq;
    dupReq.getSABSpec().setSubBlockLengthBytes(sabSpec.getSubBlockLengthBytes());
    dupReq.getSABSpec().setSubBlockIndex(sabSpec.getSubBlockIndex());

    for(DSMESABSpecification::SABSubBlock::iterator it = sabSpec.getSubBlock().beginSetBits(); it != sabSpec.getSubBlock().endSetBits(); ++it) {
        DSMEAllocationCounterTable::iterator actElement = macDSMEACT.find(sabSpec.getSubBlockIndex(), (*it) / dsme.getMAC_PIB().helper.getNumChannels());
        if(actElement != macDSMEACT.end() && (allChannels || actElement->getChannel() == (*it) % dsme.getMAC_PIB().helper.getNumChannels())) {
            LOG_INFO("Duplicate allocation " << (uint16_t)(actElement->getGTSlotID() + 9) << " " << (uint16_t)sabSpec.getSubBlockIndex() << " "
                                             << (uint16_t)actElement->getChannel());

            duplicateFound = true;
            dupReq.getSABSpec().getSubBlock().set(*it, true);

            // clear bit so the sabSpec can be used in a notification
            sabSpec.getSubBlock().set(*it, false);
        }
    }

    if(duplicateFound) {
        LOG_INFO("Duplicate found");
        IDSMEMessage* msg = dsme.getPlatform().getEmptyMessage();
        dupReq.prependTo(msg);
        GTSManagement man;
        man.type = ManagementType::DUPLICATED_ALLOCATION_NOTIFICATION;
        man.status = GTSStatus::SUCCESS;

        // this request expects no reply, so do not use usual request command
        // also do not handle this via the state machine
        uint8_t UNUSED_ANYWAY = 0;
        if(!sendGTSCommand(UNUSED_ANYWAY, msg, man, CommandFrameIdentifier::DSME_GTS_REQUEST, addr)) {
            // TODO should this be signaled to the upper layer?
            LOG_INFO("Could not send DUPLICATED_ALLOCATION_NOTIFICATION");
            dsme.getPlatform().releaseMessage(msg);
        }
    }

    return duplicateFound;
}

bool GTSManager::isTimeoutPending(uint8_t fsmId) {
    // According to the IEEE 802.15.4e standard, the macMaxFrameTotalWaitTime should be used for the timeout.
    // This is not enough, for example due to queuing of the reply and not considering the GTS times.
    // It was changed in the IEEE 802.15.4-2015 standard to macResponseWaitTime (see e.g. Figure 6-57).
    // macResponseWaitTime is given in aBaseSuperframeDurations (that do not include the superframe order)
    LOG_INFO("superframesInCurrentState: " << data[fsmId].superframesInCurrentState << "("
                                           << data[fsmId].superframesInCurrentState * (1 << dsme.getMAC_PIB().macSuperframeOrder) << "/"
                                           << dsme.getMAC_PIB().macResponseWaitTime);

    return (data[fsmId].superframesInCurrentState * (1 << dsme.getMAC_PIB().macSuperframeOrder) > dsme.getMAC_PIB().macResponseWaitTime);
}

bool GTSManager::sendGTSCommand(uint8_t fsmId, IDSMEMessage* msg, GTSManagement& man, CommandFrameIdentifier commandId, uint16_t dst, bool reportOnSent) {
    man.prependTo(msg);

    MACCommand cmd;
    cmd.setCmdId(commandId);
    cmd.prependTo(msg);

    msg->getHeader().setDstAddr(IEEE802154MacAddress(dst));
    msg->getHeader().setSrcAddrMode(AddrMode::SHORT_ADDRESS);
    msg->getHeader().setSrcAddr(IEEE802154MacAddress(dsme.getMAC_PIB().macShortAddress));
    msg->getHeader().setDstAddrMode(AddrMode::SHORT_ADDRESS);

    msg->getHeader().setSrcPANId(this->dsme.getMAC_PIB().macPANId);
    msg->getHeader().setDstPANId(this->dsme.getMAC_PIB().macPANId);

    msg->getHeader().setAckRequest(true);
    msg->getHeader().setFrameType(IEEE802154eMACHeader::FrameType::COMMAND);

    /* STATISTICS (START) */
    msg->getHeader().setCreationTime(dsme.getPlatform().getSymbolCounter());
    /* STATISTICS (END) */

    // The DUPLICATED_ALLOCATION_NOTIFICATION will be sent regardless of the current state and expects no response.
    // For example a DISALLOW REPSPONE will only be sent during BUSY, but the fact that it do not expect a notify
    // is handled inside of the onCSMASent.
    if(reportOnSent && (man.type != ManagementType::DUPLICATED_ALLOCATION_NOTIFICATION)) {
        DSME_ASSERT(fsmId < GTS_STATE_MULTIPLICITY);
        data[fsmId].cmdToSend = commandId;
        data[fsmId].msgToSend = msg;
    }

    numGTSMessages++;
    return dsme.getMessageDispatcher().sendInCAP(msg);
}

void GTSManager::preparePendingConfirm(GTSEvent& event) {
    int8_t fsmId = event.getFsmId();

    data[fsmId].pendingManagement = event.management;
    data[fsmId].pendingConfirm.deviceAddress = event.deviceAddr;
    data[fsmId].pendingConfirm.managementType = event.management.type;
    data[fsmId].pendingConfirm.direction = event.management.direction;
    data[fsmId].pendingConfirm.prioritizedChannelAccess = event.management.prioritizedChannelAccess;
    if(event.signal == GTSEvent::MLME_REQUEST_ISSUED) {
        data[fsmId].pendingConfirm.dsmeSabSpecification = event.requestCmd.getSABSpec();
    } else if(event.signal == GTSEvent::MLME_RESPONSE_ISSUED) {
        data[fsmId].pendingConfirm.dsmeSabSpecification = event.replyNotifyCmd.getSABSpec();
        data[fsmId].pendingConfirm.channelOffset = event.replyNotifyCmd.getChannelOffset();
    } else {
        DSME_ASSERT(false);
    }
}

/*****************************
 * FSM identification helpers
 *****************************/

int8_t GTSManager::getFsmIdIdle() {
    for(uint8_t i = 0; i < GTS_STATE_MULTIPLICITY; ++i) {
        if(getState(i) == &GTSManager::stateIdle) {
            return i;
        }
    }
    return -1;
}

int8_t GTSManager::getFsmIdForRequest() {
    int8_t fsmId = getFsmIdIdle();
    if(fsmId < 0) {
        return GTS_STATE_MULTIPLICITY;
    } else {
        return fsmId;
    }
}

int8_t GTSManager::getFsmIdForResponse(uint16_t destinationAddress) {
    int8_t fsmId = getFsmIdIdle();
    if(fsmId < 0) {
        return GTS_STATE_MULTIPLICITY;
    } else {
        return fsmId;
    }
}

int8_t GTSManager::getFsmIdFromResponseForMe(IDSMEMessage* msg) {
    uint16_t srcAddress = msg->getHeader().getSrcAddr().getShortAddress();
    for(uint8_t i = 0; i < GTS_STATE_MULTIPLICITY; ++i) {
        if(getState(i) == &GTSManager::stateWaitForResponse && data[i].responsePartnerAddress == srcAddress) {
            return i;
        }
    }
    return GTS_STATE_MULTIPLICITY;
}

int8_t GTSManager::getFsmIdFromNotifyForMe(IDSMEMessage* msg) {
    uint16_t srcAddress = msg->getHeader().getSrcAddr().getShortAddress();
    for(uint8_t i = 0; i < GTS_STATE_MULTIPLICITY; ++i) {
        if(getState(i) == &GTSManager::stateWaitForNotify && data[i].notifyPartnerAddress == srcAddress) {
            return i;
        }
    }
    return GTS_STATE_MULTIPLICITY;
}

bool GTSManager::hasBusyFsm() {
    bool busyFsm = false;
    for(uint8_t i = 0; i < GTS_STATE_MULTIPLICITY; ++i) {
        if(getState(i) != &GTSManager::stateIdle) {
            busyFsm = true;
        }
    }
    return busyFsm;
}

} /* namespace dsme */
