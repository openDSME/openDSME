#include "./QAgent.h"
#include "./DSMELayer.h"
#include "../interfaces/IDSMEPlatform.h"
#include "../mac_services/pib/MAC_PIB.h"
#include "../mac_services/pib/dsme_mac_constants.h"
#include "capLayer/CAPLayer.h"

#include<iostream>

namespace dsme {

QAgent::QAgent(DSMELayer &dsme, float eps, float eps_min, float eps_decay, float gamma, float lr) : dsme(dsme), eps{eps}, eps_min{eps_min}, eps_decay{eps_decay}, gamma{gamma}, lr{lr}, lastAction{QAction::BACKOFF} {
        /* INIT FEATURES */
        featureManager.getState().getFeature<QueueFullFeature>().setUpdateFunc(DELEGATE(&CAPLayer::getQueueLevel, dsme.getCapLayer()));
        featureManager.getState().getFeature<QueueFullFeature2>().setUpdateFunc(DELEGATE(&CAPLayer::getQueueLevel, dsme.getCapLayer()));
        featureManager.getState().getFeature<TimeFeature>().setUpdateFunc(DELEGATE(&QAgent::timeFeatureUpdateFunc, *this));

        /* INIT QTABLE */
        for(uint32_t i=0; i<QState::getMaxId(); i++) {
                for(action_t action=0; action<(action_t)QAction::NUM_ACTIONS; action++) {
                        if(action == (action_t)QAction::SEND) {
                            qTable[i][action] = -0.1;
                        } else {
                            qTable[i][action] = 0;
                        }
                }
        }
}

void QAgent::updateQTable(state_t const id, state_t const nextId, QAction const &action, reward_t reward) {
        //std::cout << "id: " << (int)id << " next: " << (int)nextId << " a: " << (int)action << std::endl;
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

float QAgent::maxQ(state_t id) const {
        auto maxQ = qTable[id][0];
        for(auto& q : qTable[id]) {
                if(q > maxQ) {
                        maxQ = q;
                }
        }
        return maxQ;
}

action_t QAgent::maxAction(state_t id) {
        auto maxQ = qTable[id][0];
        action_t a = 0;
        for(uint8_t action=0; action<(action_t)QAction::NUM_ACTIONS; action++) {
                auto currentQ = qTable[id][action];
                /*if(action == (action_t)QAction::SEND) {
                    currentQ += featureManager.getState().getFeature<QueueFullFeature>().getValue();
                }*/
                if(currentQ >= maxQ) {
                        maxQ = currentQ;//qTable[id][action];
                        a = action;
                }
        }
        return a;
}

QAction QAgent::selectAction(bool deterministic) {
        QState currentState = featureManager.getState();
        lastState = currentState;

        LOG_DEBUG("QA: time -> " << currentState.getFeature<TimeFeature>().getValue());

        // update and signal eps
        eps = eps * eps_decay > eps_min ? eps * eps_decay : eps_min;
        LOG_INFO("EPS: " << eps);
        dsme.getPlatform().signalEPS(eps);

        // select a next action and save it q update
        uint16_t rnd = dsme.getPlatform().getRandom() % 10000;
        if((rnd < 10000*eps) && !deterministic) {
                LOG_INFO("RANDOM");
                lastAction =  static_cast<QAction>(dsme.getPlatform().getRandom() % static_cast<action_t>(QAction::NUM_ACTIONS));
        } else {
                lastAction =  static_cast<QAction>(maxAction(currentState.getId()));
        }
        LOG_INFO("ACTION: " << (int)lastAction);

        return lastAction;
}

auto QAgent::resetTx() -> void {
    QState currentState = featureManager.getState();
    LOG_INFO("QA: Someone sent in this slot -> negative reinforcement for sending");
    updateQTable(currentState.getId(), currentState.getId(), QAction::SEND, -2);
}

void QAgent::update() {
        QState currentState = featureManager.getState();

        // REWARD CALUCLATION
        reward_t reward = 0;
        switch(lastAction) {
            case QAction::BACKOFF:
                reward = currentState.getFeature<QueueFullFeature2>().getValue() >= 5 ? -2 : 0;
                break;
            case QAction::CCA:
                reward = currentState.getFeature<CCASuccessFeature>().getValue() && !currentState.getFeature<SuccessFeature>().getValue() ? -2 : 0;
                reward += currentState.getFeature<CCASuccessFeature>().getValue() && currentState.getFeature<SuccessFeature>().getValue() ? 4 : 0;
            case QAction::SEND:
                reward = currentState.getFeature<SuccessFeature>().getValue() ? 4 : -2;
                reward -= currentState.getFeature<OtherQueueFullFeature>().getValue() > currentState.getFeature<QueueFullFeature2>().getValue() ? 2 : 0;
                break;
            default:
                DSME_ASSERT(false);
        }
        dsme.getPlatform().signalReward(reward);

        LOG_DEBUG("QA: time -> " << currentState.getFeature<TimeFeature>().getValue());

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
