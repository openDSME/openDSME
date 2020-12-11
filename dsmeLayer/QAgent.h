#ifndef QAGENT_H
#define QAGENT_H

#include "../mac_services/DSME_Common.h"
#include "../../dsme_platform.h"
#include "FeatureManager.h"


namespace dsme {

/* FORWARD DECLARATION */
class DSMELayer;

/* FEATURES (TABLE) */
using TimeFeature = Feature<uint32_t, 'A', 0, 60*9*(1<<3), REPLACE, UPDATE_RULE::ON_STATE_COLLECTION, true, 54>;
using MSFFeature = Feature<uint16_t, 'B', 0, 8, REPLACE, UPDATE_RULE::ON_STATE_COLLECTION, false>;
using AckReceivedFeature = Feature<bool, 'Y', true, false, REPLACE, UPDATE_RULE::ON_STATE_COLLECTION, false, false, UPDATE_RULE::ON_MSF_EVENT>;
using MsgReceivedFeature = Feature<bool, 'Z', true, false, REPLACE, UPDATE_RULE::ON_STATE_COLLECTION, false, false, UPDATE_RULE::ON_MSF_EVENT>;
using QueueFullFeature2 = Feature<uint16_t, 'D', 0, CAP_QUEUE_SIZE, REPLACE, UPDATE_RULE::ON_STATE_COLLECTION, false>;
using SuccessFeature = Feature<bool, 'G', false, true, REPLACE, UPDATE_RULE::ON_USER_EVENT, false, 0, UPDATE_RULE::ON_MSF_EVENT>;
using CCASuccessFeature = Feature<bool, 'H', false, true, REPLACE, UPDATE_RULE::ON_USER_EVENT, false, 0, UPDATE_RULE::ON_MSF_EVENT>;


using OtherQueueFullFeature = Feature<uint16_t, 'E', 0, CAP_QUEUE_SIZE, MAXIMUM, UPDATE_RULE::ON_USER_EVENT, false, 0, UPDATE_RULE::ON_MSF_EVENT>;
using OverheardPacketsFeature = Feature<uint8_t, 'F', 0, 255, ACCUMULATE, UPDATE_RULE::ON_USER_EVENT, false, 0, UPDATE_RULE::ON_MSF_EVENT>;
using SentPacketsFeature = Feature<uint16_t, 'I', 0, 254, ACCUMULATE, UPDATE_RULE::ON_USER_EVENT, false, 0, UPDATE_RULE::ON_MSF_EVENT>;
using TxSuccessFeature = Feature<uint8_t, 'J', 0, 253, ACCUMULATE, UPDATE_RULE::ON_USER_EVENT, false, 0, UPDATE_RULE::ON_MSF_EVENT>;
using TxFailedFeature = Feature<uint8_t, 'K', 0, 252, ACCUMULATE, UPDATE_RULE::ON_USER_EVENT, false, 0, UPDATE_RULE::ON_MSF_EVENT>;
using QState = State<MsgReceivedFeature, AckReceivedFeature, MSFFeature, QueueFullFeature2, TimeFeature, OtherQueueFullFeature, SuccessFeature, CCASuccessFeature, OverheardPacketsFeature, SentPacketsFeature, TxSuccessFeature, TxFailedFeature>;
using QFeatureManager = FeatureManager<MsgReceivedFeature, AckReceivedFeature, MSFFeature, QueueFullFeature2, TimeFeature, OtherQueueFullFeature, SuccessFeature, CCASuccessFeature, OverheardPacketsFeature, SentPacketsFeature, TxSuccessFeature, TxFailedFeature>;

using action_t = uint8_t;
using reward_t = float;


enum class QAction : action_t {
    BACKOFF,
    CCA,
    SEND,
    NUM_ACTIONS
};

class QAgent {
public:
    QAgent(DSMELayer &dsme, uint16_t eps=0.001, uint16_t eps_min=0.001, float eps_decay=0.9999, float gamma=0.9, float lr=0.5);

    auto initialize() -> void;

    auto getFeatureManager() -> QFeatureManager& {
        return featureManager;
    }

    auto selectAction(bool deterministic=false) -> QAction;

    auto update() -> void;

    auto resetTx() -> void;

    auto printQTable() const -> void;

    auto printTxTimes() const -> void;

    auto printPolicy() const -> void;

    auto signalOverheardMsg() -> void;

    auto handleStartOfCFP() -> void;

    /* Feature helper */
    auto timeFeatureUpdateFunc() -> uint32_t;

    uint32_t accReward;

    auto calcMaxQ() -> uint32_t {
        uint32_t q = 0;
        for(uint32_t id=0; id<QState::getMaxId(); id++) {
            q += maxQ(id);
        }
        return q;
    }

private:
    auto maxQ(state_t const id) const -> float;

    auto maxAction(state_t const id) -> action_t;

    auto updateQTable(state_t const id, state_t const nextId, QAction const &action, reward_t reward) -> void;

private:
    DSMELayer &dsme;
    uint16_t eps;
    uint16_t eps_min;
    float eps_decay;
    float gamma;
    float lr;
    float qTable[QState::getMaxId()][(action_t)QAction::NUM_ACTIONS];
    action_t policy[QState::getMaxId()];
    uint16_t sfCounter;
    bool sfCounterStarted;

    QAction lastAction;
    QState lastState;

    QFeatureManager featureManager;
};

};
#endif /* QAGENT_H */
