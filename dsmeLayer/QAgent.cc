#include "./QAgent.h"
#include "./DSMELayer.h"
#include "../interfaces/IDSMEPlatform.h"
#include "../mac_services/pib/MAC_PIB.h"
#include "../mac_services/pib/dsme_mac_constants.h"
#include "capLayer/CAPLayer.h"

namespace dsme {

QAgent::QAgent(DSMELayer &dsme, float eps, float eps_min, float eps_decay, float gamma, float lr) : dsme(dsme), eps{eps}, eps_min{eps_min}, eps_decay{eps_decay}, gamma{gamma}, lr{lr}, lastAction{QAction::NUM_ACTIONS} {
        /* INIT FEATURES */
        featureManager.getState().getFeature<QueueFeature>().setUpdateFunc(DELEGATE(&CAPLayer::getQueueLevel, dsme.getCapLayer()));
        featureManager.getState().getFeature<QueueFullFeature>().setUpdateFunc(DELEGATE(&CAPLayer::getQueueLevel, dsme.getCapLayer()));
        featureManager.getState().getFeature<TimeFeature>().setUpdateFunc(DELEGATE(&QAgent::timeFeatureUpdateFunc, *this));

        /* INIT QTABLE */
        for(uint32_t i=0; i<QState::getMaxId(); i++) {
                for(action_t action=0; action<(action_t)QAction::NUM_ACTIONS; action++) {
                        if(action == (action_t)QAction::SEND) {
                            qTable[i][action] = 0.1;
                        } else {
                            qTable[i][action] = 0;
                        }

                }
        }
}

void QAgent::updateQTable(uint8_t const id, uint8_t const nextId, QAction const &action, reward_t reward) {
        float q = qTable[id][(int)action] + lr * (reward + gamma * maxQ(nextId) - qTable[id][(int)action]);
        dsme.getPlatform().signalQ(q);
        qTable[id][(int)action] = q;
}

void QAgent::printQTable() const {
        LOG_INFO("Q-TABLE:");
        for(uint32_t i=0; i<QState::getMaxId(); i++) {
                for(action_t action=0; action<(action_t)QAction::NUM_ACTIONS; action++) {
                        LOG_INFO_PURE(qTable[i][action] << " ");
                }
                LOG_INFO_PURE(std::endl);
        }
}

float QAgent::maxQ(uint8_t id) const {
        auto maxQ = qTable[id][0];
        for(auto& q : qTable[id]) {
                if(q > maxQ) {
                        maxQ = q;
                }
        }
        return maxQ;
}

action_t QAgent::maxAction(uint8_t id) const {
        auto maxQ = qTable[id][0];
        action_t a = 0;
        for(uint8_t action=0; action<(action_t)QAction::NUM_ACTIONS; action++) {
                if(qTable[id][action] >= maxQ) {
                        maxQ = qTable[id][action];
                        a = action;
                }
        }
        return a;
}

QAction QAgent::selectAction(bool deterministic) {
        QState currentState = featureManager.getState();
        lastState = currentState;

        // update and signal eps
        eps = eps * eps_decay > eps_min ? eps * eps_decay : eps_min;
        LOG_INFO("EPS: " << eps);
        dsme.getPlatform().signalEPS(eps);

        // select a next action and save it q update
        uint16_t rnd = dsme.getPlatform().getRandom() % 10000;
        if((rnd < 10000*eps) && !deterministic) {
                lastAction =  static_cast<QAction>(dsme.getPlatform().getRandom() % static_cast<action_t>(QAction::NUM_ACTIONS));
        } else {
                lastAction =  static_cast<QAction>(maxAction(currentState.getId()));
        }
        LOG_INFO("ACTION: " << (int)lastAction);

        return lastAction;
}

void QAgent::update() {
        QState currentState = featureManager.getState();

        // REWARD CALUCLATION
        reward_t reward = 0;
        switch(lastAction) {
            case QAction::BACKOFF:
                reward = -1 * (currentState.getFeature<QueueFeature>().getValue() >= 7);
                break;
            case QAction::CCA:
            case QAction::SEND:
                reward = currentState.getFeature<SuccessFeature>().getValue() ? 1 : -1;
                reward -= (currentState.getFeature<OverheardPacketsFeature>().getValue() <= currentState.getFeature<SentPacketsFeature>().getValue());
                break;
            default:
                DSME_ASSERT(false);
        }
        dsme.getPlatform().signalReward(reward);


        LOG_INFO("QA: Got reward " << reward);
        updateQTable(lastState.getId(), currentState.getId(), lastAction, reward);
}

void QAgent::printTxTimes() const {
    LOG_INFO_PREFIX;
    for(uint32_t id=0; id<QState::getMaxId(); id++) {
        if(qTable[id][(int)QAction::BACKOFF] < qTable[id][(int)QAction::SEND] || qTable[id][(int)QAction::BACKOFF] < qTable[id][(int)QAction::CCA] ) {
            LOG_INFO_PURE(" " << id);
        }
    }
    LOG_INFO_PURE(std::endl);
}

/* Feature helpers */
auto QAgent::timeFeatureUpdateFunc() -> uint32_t {
    return dsme.getSymbolsSinceCapFrameStart(dsme.getPlatform().getSymbolCounter());
}

}; /* DSME */
