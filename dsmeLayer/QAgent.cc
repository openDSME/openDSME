#include "./QAgent.h"
#include "./DSMELayer.h"
#include "../interfaces/IDSMEPlatform.h"
#include "../mac_services/pib/MAC_PIB.h"
#include "../mac_services/pib/dsme_mac_constants.h"

namespace dsme {

QAgent::QAgent(DSMELayer &dsme, float eps, float eps_min, float eps_decay, float gamma, float lr) : dsme(dsme), eps{eps}, eps_min{eps_min}, eps_decay{eps_decay}, gamma{gamma}, lr{lr}, lastState{0,0,0,0,false,false,0}, lastAction{0}, otherQueue{0} {
        for(uint32_t i=0; i<QState::getMaxId(); i++) {
                for(action_t action=0; action<(action_t)QAction::NUM_ACTIONS; action++) {
                        if(action == 0) {
                            qTable[i][action] = 0;
                        } else {
                            qTable[i][action] = 0;
                        }

                }
        }
}

void QAgent::updateQTable(QState const &state, QState const &nextState, action_t const &action, reward_t reward) {
        float q;
        //if(nextState.getRetransmissions() == 0 && nextState.getBackoffs() == 0) {
        //-> final state
        //    q = (1-lr) * qTable[state.getId()][action] + lr * reward;
        //} else {
        q = qTable[state.getId()][action] + lr * (reward + gamma * maxQ(nextState) - qTable[state.getId()][action]);
        //}
        dsme.getPlatform().signalQ(q);
        qTable[state.getId()][action] = q;
}

void QAgent::printQTable() const {
        LOG_INFO("Q-TABLE:");
        for(uint32_t i=0; i<QState::getMaxId(); i++) {
                for(action_t action=0; action<(action_t)QAction::NUM_ACTIONS; action++) {
                        LOG_INFO_PURE(qTable[i][action]);
                }
                LOG_INFO_PURE(std::endl);
        }
}

float QAgent::maxQ(QState const &state) const {
        auto maxQ = qTable[state.getId()][0];
        for(auto& q : qTable[state.getId()]) {
                if(q > maxQ) {
                        maxQ = q;
                }
        }
        return maxQ;
}

action_t QAgent::maxAction(QState const &state) const {
        auto maxQ = qTable[state.getId()][0];
        action_t a = 0;
        for(uint8_t action=0; action<(action_t)QAction::NUM_ACTIONS; action++) {
                if(qTable[state.getId()][action] > maxQ) {
                        maxQ = qTable[state.getId()][action];
                        a = action;
                }
        }
        return a;
}

void QAgent::signalQueueLevelCAP(uint8_t queueLevel) {
        otherQueue = queueLevel;
}

action_t QAgent::selectAction(bool deterministic) {
        // calculate current state (ts, queue, otherQueue) and save it for q update
        uint32_t time = dsme.getSymbolsSinceCapFrameStart(dsme.getPlatform().getSymbolCounter()) / (aBaseSlotDuration * 2<<dsme.getMAC_PIB().macSuperframeOrder);
        QState currentState = QState(0, 0, time, dsme.getCapLayer().getQueueLevel(), false, false, otherQueue);
        lastState = currentState;

        // update and signal eps
        eps = eps * eps_decay > eps_min ? eps * eps_decay : eps_min;
        dsme.getPlatform().signalEPS(eps);

        // select a next action and save it q update
        uint16_t rnd = dsme.getPlatform().getRandom() % 10000;
        if((rnd < 10000*eps) && !deterministic) {
                lastAction =  dsme.getPlatform().getRandom() % (action_t)QAction::NUM_ACTIONS;
        } else {
                lastAction =  maxAction(currentState);
        }
        LOG_INFO("ACTION: " << (int)lastAction);
        return lastAction;
}

void QAgent::update(bool ccaSuccess, bool txSuccess, bool queueFull, uint8_t NR, uint8_t NB, uint32_t dwellTime) {
        // GET ADDITIONAL INFORMATION
        uint16_t queueLevel = dsme.getCapLayer().getQueueLevel();
        uint32_t time = dsme.getSymbolsSinceCapFrameStart(dsme.getPlatform().getSymbolCounter()) / (aBaseSlotDuration * 2<<dsme.getMAC_PIB().macSuperframeOrder);


        // REWARD CALUCLATION
        reward_t reward = -1;
        //reward -= (queueLevel > otherQueue+1) || (queueLevel < otherQueue-1);
        //if(NR == 0 && NB == 0 && !txSuccess) reward -= 10;
        switch((QAction)lastAction) {
            case QAction::BACKOFF:
                reward -= queueFull;
                break;
            case QAction::CCA:
                reward = reward - !txSuccess - (otherQueue > 5);
                break;
            default:
                DSME_ASSERT(false);
        }
        dsme.getPlatform().signalReward(reward);

        // Q-TABLE UPDATE
        QState state = QState(NR, NB, lastState.getTime()+1, queueLevel, txSuccess, ccaSuccess, otherQueue);
        state.print();
        updateQTable(lastState, state, lastAction, reward);
        printQTable();
}

}; /* DSME */
