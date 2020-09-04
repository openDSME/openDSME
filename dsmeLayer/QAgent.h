#ifndef QAGENT_H
#define QAGENT_H

#include "../mac_services/DSME_Common.h"
#include "../../dsme_platform.h"
#include "FeatureManager.h"


namespace dsme {

/* FORWARD DECLARATION */
class DSMELayer;

/* FEATURES (TABLE) */
using TimeFeature = Feature<uint32_t, 0, 60*8*(2<<6), UPDATE_RULE::ON_STATE_COLLECTION, true, 80>;
using OtherQueueFullFeature = Feature<uint16_t, 0, CAP_QUEUE_SIZE, UPDATE_RULE::ON_USER_EVENT>;
using QueueFullFeature = Feature<uint16_t, 0, CAP_QUEUE_SIZE, UPDATE_RULE::ON_STATE_COLLECTION>;
/* FEATURES (REWARD) */
using QueueFeature = Feature<uint16_t, 0, CAP_QUEUE_SIZE, UPDATE_RULE::ON_STATE_COLLECTION>;
using SuccessFeature = Feature<bool, false, true, UPDATE_RULE::ON_USER_EVENT>;

using QState = State<QueueFeature, TimeFeature, QueueFullFeature, OtherQueueFullFeature, SuccessFeature>;
using QFeatureManager = FeatureManager<QueueFeature, TimeFeature, QueueFullFeature, OtherQueueFullFeature, SuccessFeature>;

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
