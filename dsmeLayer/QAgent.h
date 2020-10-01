#ifndef QAGENT_H
#define QAGENT_H

#include "../mac_services/DSME_Common.h"
#include "../../dsme_platform.h"
#include "FeatureManager.h"


namespace dsme {

/* FORWARD DECLARATION */
class DSMELayer;

/* FEATURES (TABLE) */
using TimeFeature = Feature<uint32_t, 'T', 0, 60*8*(1<<6), REPLACE, UPDATE_RULE::ON_STATE_COLLECTION, true, 216>;
using QueueFullFeature = Feature<uint16_t, 'Q', 0, CAP_QUEUE_SIZE, REPLACE, UPDATE_RULE::ON_STATE_COLLECTION, false, 2>;
using QueueFullFeature2 = Feature<uint16_t, 'Z', 0, CAP_QUEUE_SIZE, REPLACE, UPDATE_RULE::ON_STATE_COLLECTION, false>;
using OtherQueueFullFeature = Feature<uint16_t, 'O', 0, CAP_QUEUE_SIZE, MAXIMUM, UPDATE_RULE::ON_USER_EVENT, false, 0, UPDATE_RULE::ON_MSF_EVENT>;
using OverheardPacketsFeature = Feature<uint8_t, 'P', 0, 255, ACCUMULATE, UPDATE_RULE::ON_USER_EVENT, false, 0, UPDATE_RULE::ON_MSF_EVENT>;
/* FEATURES (REWARD) */
using SuccessFeature = Feature<bool, 'S', false, true, REPLACE, UPDATE_RULE::ON_USER_EVENT, false, 0, UPDATE_RULE::ON_MSF_EVENT>;
using CCASuccessFeature = Feature<bool, 'C', false, true, REPLACE, UPDATE_RULE::ON_USER_EVENT, false, 0, UPDATE_RULE::ON_MSF_EVENT>;
using SentPacketsFeature = Feature<uint16_t, 'R', 0, 254, ACCUMULATE, UPDATE_RULE::ON_USER_EVENT, false, 0, UPDATE_RULE::ON_MSF_EVENT>;

using TxSuccessFeature = Feature<uint8_t, 'E', 0, 253, ACCUMULATE, UPDATE_RULE::ON_USER_EVENT, false, 0, UPDATE_RULE::ON_MSF_EVENT>;
using TxFailedFeature = Feature<uint8_t, 'F', 0, 252, ACCUMULATE, UPDATE_RULE::ON_USER_EVENT, false, 0, UPDATE_RULE::ON_MSF_EVENT>;

using DwellTimeFeature = Feature<uint16_t, 'D', 0, 16000, ACCUMULATE, UPDATE_RULE::ON_USER_EVENT, false, 0, UPDATE_RULE::ON_USER_EVENT>;

using QState = State<DwellTimeFeature, QueueFullFeature2, TimeFeature, QueueFullFeature, OtherQueueFullFeature, SuccessFeature, CCASuccessFeature, OverheardPacketsFeature, SentPacketsFeature, TxSuccessFeature, TxFailedFeature>;
using QFeatureManager = FeatureManager<DwellTimeFeature, QueueFullFeature2, TimeFeature, QueueFullFeature, OtherQueueFullFeature, SuccessFeature, CCASuccessFeature, OverheardPacketsFeature, SentPacketsFeature, TxSuccessFeature, TxFailedFeature>;

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
    QAgent(DSMELayer &dsme, float eps=1.0, float eps_min=0.01, float eps_decay=0.999, float gamma=0.9, float lr=0.5);

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

    auto age() -> void;

    /* Feature helper */
    auto timeFeatureUpdateFunc() -> uint32_t;

private:
    auto maxQ(state_t const id) const -> float;

    auto maxAction(state_t const id) -> action_t;

    auto updateQTable(state_t const id, state_t const nextId, QAction const &action, reward_t reward) -> void;

private:
    DSMELayer &dsme;
    float eps;
    float eps_min;
    float eps_decay;
    float gamma;
    float lr;
    float qTable[QState::getMaxId()][(action_t)QAction::NUM_ACTIONS];
    action_t policy[QState::getMaxId()];

    QAction lastAction;
    QState lastState;

    QFeatureManager featureManager;
};

};
#endif /* QAGENT_H */
