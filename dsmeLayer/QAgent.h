#ifndef QAGENT_H
#define QAGENT_H

#include "../mac_services/DSME_Common.h"
#include "../../dsme_platform.h"
#include "FeatureManager.h"


namespace dsme {

/* FORWARD DECLARATION */
class DSMELayer;

/* FEATURES (TABLE) */
using TimeFeature = Feature<uint32_t, 0, 60*8*(2<<6), REPLACE, UPDATE_RULE::ON_STATE_COLLECTION, false, 1563>;
using QueueFullFeature = Feature<uint16_t, 0, CAP_QUEUE_SIZE, REPLACE, UPDATE_RULE::ON_STATE_COLLECTION, true>;
using OtherQueueFullFeature = Feature<uint16_t, 0, CAP_QUEUE_SIZE, REPLACE, UPDATE_RULE::ON_USER_EVENT>;
using OverheardPacketsFeature = Feature<uint8_t, 0, 255, ACCUMULATE, UPDATE_RULE::ON_USER_EVENT, false, 0, UPDATE_RULE::ON_MSF_EVENT>;
/* FEATURES (REWARD) */
using SuccessFeature = Feature<bool, false, true, REPLACE, UPDATE_RULE::ON_USER_EVENT>;
using SentPacketsFeature = Feature<uint16_t, 0, 254, ACCUMULATE, UPDATE_RULE::ON_USER_EVENT, false, 0, UPDATE_RULE::ON_MSF_EVENT>;

using TxSuccessFeature = Feature<uint8_t, 0, 253, ACCUMULATE, UPDATE_RULE::ON_USER_EVENT, false, 0, UPDATE_RULE::ON_MSF_EVENT>;
using TxFailedFeature = Feature<uint8_t, 0, 252, ACCUMULATE, UPDATE_RULE::ON_USER_EVENT, false, 0, UPDATE_RULE::ON_MSF_EVENT>;

using QState = State<TimeFeature, QueueFullFeature, OtherQueueFullFeature, SuccessFeature, OverheardPacketsFeature, SentPacketsFeature, TxSuccessFeature, TxFailedFeature>;
using QFeatureManager = FeatureManager<TimeFeature, QueueFullFeature, OtherQueueFullFeature, SuccessFeature, OverheardPacketsFeature, SentPacketsFeature, TxSuccessFeature, TxFailedFeature>;

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
    QAgent(DSMELayer &dsme, float eps=1.0, float eps_min=0.00, float eps_decay=0.999, float gamma=0.9, float lr=0.1);

    auto getFeatureManager() -> QFeatureManager& {
        return featureManager;
    }

    auto selectAction(bool deterministic=false) -> QAction;

    auto update() -> void;

    auto printQTable() const -> void;

    auto printTxTimes() const -> void;

    /* Feature helper */
    auto timeFeatureUpdateFunc() -> uint32_t;

private:
    auto maxQ(uint8_t const id) const -> float;

    auto maxAction(uint8_t const id) const -> action_t;

    auto updateQTable(uint8_t const id, uint8_t const nextId, QAction const &action, reward_t reward) -> void;

private:
    DSMELayer &dsme;
    float eps;
    float eps_min;
    float eps_decay;
    float gamma;
    float lr;
    float qTable[QState::getMaxId()][(action_t)QAction::NUM_ACTIONS];

    QAction lastAction;
    QState lastState;

    QFeatureManager featureManager;
};

};
#endif /* QAGENT_H */
