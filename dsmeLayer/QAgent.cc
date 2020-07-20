#include "./QAgent.h"
#include "./DSMELayer.h"
#include "../interfaces/IDSMEPlatform.h"
#include "../mac_services/pib/MAC_PIB.h"

namespace dsme {

QAgent::QAgent(DSMELayer &dsme, float eps, float eps_min, float eps_decay, float gamma, float lr) : dsme(dsme), eps{eps}, eps_min{eps_min}, eps_decay{eps_decay}, gamma{gamma}, lr{lr}, lastState{3,0,0,0,0,false,false,0}, lastAction{0}, otherQueue{0} {
        for(uint32_t i=0; i<QState::getMaxId(); i++) {
            for(action_t action=0; action<ACTION_STATES; action++) {
                qTable[i][action] = 0;
            }
        }
    }

    action_t QAgent::selectAction(QState const &state, bool deterministic) {
        if(eps*eps_decay > eps_min) {
            eps *= eps_decay;
        } else {
            eps = eps_min;
        }
        LOG_INFO("EPS: " << eps);

        uint16_t rnd = dsme.getPlatform().getRandom() % 10000;
        if((rnd < 10000*eps) && !deterministic) {
            return dsme.getPlatform().getRandom() % ACTION_STATES;
        } else {
            return maxAction(state);
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
            for(action_t action=0; action<ACTION_STATES; action++) {
                LOG_INFO(qTable[i][action]);
            }
            LOG_INFO("ROW");
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
        for(uint8_t action=0; action<ACTION_STATES; action++) {
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

void QAgent::update(bool ccaFailed, bool txFailed, bool txSuccess, uint8_t NR, uint8_t NB, uint32_t lastWaitTime) {
    // GET ADDITIONAL INFORMATION
    uint8_t minBE = dsme.getMAC_PIB().macMinBE;
    uint16_t queueLevel = dsme.getCapLayer().getQueueLevel();
    uint16_t currentSlot = dsme.getCurrentSlot();

    // REWARD CALUCLATION
    reward_t reward = -1;
    if(NR >= 2) reward -= 1;
    if(otherQueue >= 6) reward -= 1;
    if(queueLevel >= 6) reward -= 1;
    if(NR == 0 && NB == 0 && txSuccess) reward = 0;

    lastState.print();
    LOG_INFO("LAST STATE ACTION: " << (int)lastAction);
    LOG_INFO("LAST STATE REWARD: " << (int)reward);
    dsme.getPlatform().signalReward(reward);

    // Q-TABLE UPDATE
    QState state = QState(minBE, NR, NB, currentSlot, queueLevel, txFailed, ccaFailed, otherQueue);
    state.print();
    updateQTable(lastState, state, lastAction, reward);

    // EXECUTE NEXT ACTION
    action_t action = selectAction(lastState);
    dsme.getPlatform().signalEPS(eps);
    LOG_INFO("ACTION: " << (int)action);
    dsme.getMAC_PIB().macMinBE = action;
    dsme.getMAC_PIB().macMaxBE = action;
    dsme.getPlatform().signalBE(dsme.getMAC_PIB().macMinBE);

    lastState = state;
    lastAction = action;
}

}; /* DSME */
