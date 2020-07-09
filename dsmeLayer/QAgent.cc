#include "./QAgent.h"
#include "./DSMELayer.h"
#include "../interfaces/IDSMEPlatform.h"
#include "../mac_services/pib/MAC_PIB.h"

namespace dsme {

QAgent::QAgent(DSMELayer &dsme, float eps, float eps_min, float eps_decay, float gamma, float lr) : dsme(dsme), eps{eps}, eps_min{eps_min}, eps_decay{eps_decay}, gamma{gamma}, lr{lr}, lastState{3,5,false,0,0,0}, lastAction{0}, success{false}, dwellTime{0} {
        for(uint16_t i=0; i<QState::getMaxId(); i++) {
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

        uint16_t rnd = dsme.getPlatform().getRandom() % 100;
        if((rnd < 100*eps) && !deterministic) {
            LOG_INFO("Random action");
            return dsme.getPlatform().getRandom() % ACTION_STATES;
        } else {
            return maxAction(state);
        }
    }

    void QAgent::updateQTable(QState const &state, QState const &nextState, action_t const &action, reward_t reward) {
        float q = qTable[state.getId()][action] + lr * (reward + gamma * maxQ(nextState) - qTable[state.getId()][action]);
        dsme.getPlatform().signalQ(q);
        qTable[state.getId()][action] = q;
    }

    void QAgent::printQTable() const {
        LOG_DEBUG("Q-TABLE:");
        for(uint16_t i=0; i<QState::getMaxId(); i++) {
            for(action_t action=0; action<ACTION_STATES; action++) {
                LOG_DEBUG(qTable[i][action]);
            }
            LOG_DEBUG("ROW");
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

void QAgent::handleStartOfCFP(uint8_t NR, uint8_t NB, uint32_t lastWaitTime) {
    static uint32_t SFC = 0;

    uint8_t minBE = dsme.getMAC_PIB().macMinBE;
    uint16_t queueLevel = dsme.getCapLayer().getQueueLevel();

    // REWARD CALUCLATION
    reward_t reward = 100 * success - lastWaitTime; //- lastWaitTime/ 10; //(1<<minBE); // - minBE * (lastState.getRetransmissions() * dsme.getMAC_PIB().macMaxCSMABackoffs + (lastState.getBackoffs()+1)) - queueLevel;
    if(NR >= (dsme.getMAC_PIB().macMaxFrameRetries-1)) reward = -10000;
    LOG_INFO("REWARD: " << (int)reward);
    dsme.getPlatform().signalReward(reward);

    // Q-TABLE UPDATEN
    QState nextState = QState(minBE, queueLevel, success, NR, NB, dsme.getCurrentSlot());
    updateQTable(lastState, nextState, lastAction, reward);
    LOG_INFO("STATE: " << lastState.getId());
    lastState.print();
    lastState = nextState;
    lastState.print();

    lastAction = selectAction(lastState);
    lastAction = 0;
    LOG_INFO("ACT: " << (int)lastAction);
    dsme.getMAC_PIB().macMinBE = lastAction;
    dsme.getMAC_PIB().macMaxBE = lastAction;
    success = false;
}

void QAgent::onCSMASent(DataStatus::Data_Status status, uint8_t numBackoffs, uint32_t dwellTime) {
    success = status == DataStatus::SUCCESS;
    this->dwellTime = dwellTime;
}

}; /* DSME */
