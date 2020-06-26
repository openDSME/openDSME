#include "./QAgent.h"
#include "./DSMELayer.h"
#include "../interfaces/IDSMEPlatform.h"
#include "../mac_services/pib/MAC_PIB.h"

#include <iostream>

namespace dsme {

QAgent::QAgent(DSMELayer &dsme, uint8_t eps, float gamma, float lr) : dsme(dsme), eps{eps}, gamma{gamma}, lr{lr}, lastState{3,5,0}, lastAction{5} {
        for(uint16_t i=0; i<QState::getMaxId(); i++) {
            for(action_t action=0; action<ACTION_STATES; action++) {
                qTable[i][action] = 0;
            }
        }
    }


    action_t QAgent::selectAction(QState const &state, bool deterministic) const {
        uint16_t rnd = dsme.getPlatform().getRandom() % 100;
        if((rnd < eps) && !deterministic) {
            std::cout << "RANDOM" << std::endl;
            return dsme.getPlatform().getRandom() % 5;
        } else {
            return maxAction(state);
        }
    }

    void QAgent::updateQTable(QState const &state, action_t const &action, reward_t reward) {
        qTable[state.getId()][action] = qTable[state.getId()][action] + lr * (reward + gamma * maxQ(state) - qTable[state.getId()][action]);
    }

    void QAgent::printQTable() const {
        LOG_DEBUG("Q-TABLE:");
        for(uint16_t i=0; i<QState::getMaxId(); i++) {
            for(action_t action=0; action<ACTION_STATES; action++) {
                LOG_DEBUG(qTable[i][action]);
            }
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

void QAgent::handleStartOfCFP(uint16_t currentSuperframe, uint16_t currentMultiSuperframe) { /* DSME SPECIFIC */
    static uint32_t SFC = 0;
    if(currentSuperframe != 0) return;

    uint8_t minBE = dsme.getMAC_PIB().macMinBE;
    uint8_t maxBE = dsme.getMAC_PIB().macMaxBE;
    std::cout << "minBE: " << (int)minBE << "  maxBE: " << (int)maxBE << std::endl;
    uint16_t queueLevel = dsme.getCapLayer().getQueueLevel();

    //reward_t reward = dsme::CAP_QUEUE_SIZE - queueLevel;
    reward_t reward = 2*queueLevel;
    std::cout << "Q: " << (int)queueLevel << std::endl;
    if(minBE == 0 && lastAction == 0) reward = -100;
    if(minBE == 9 && lastAction == 1) reward = -100;
    if(maxBE == 0 && lastAction == 2) reward = -100;
    if(maxBE == 9 && lastAction == 3) reward = -100;
    if(maxBE == minBE && (lastAction == 1 || lastAction == 2)) reward = -100;
    if(lastAction < 5) {
        updateQTable(lastState, lastAction, reward);
    }

    lastState = QState(minBE, maxBE, queueLevel);

    std::cout << "C: " << SFC << std::endl;
    lastAction = selectAction(lastState, SFC++ > 4000);
    switch(lastAction) {
         case 0: // inc minBE
         {
            std::cout << "minDEC" << std::endl;
            if(minBE > 0) dsme.getMAC_PIB().macMinBE--;
         }
            break;
         case 1: // dec minBE
         {
             std::cout << "minINC" << std::endl;
            if(minBE < 9 && minBE < maxBE) dsme.getMAC_PIB().macMinBE++;
         }
            break;
         case 2: // inc maxBE
         {
            std::cout << "maxDEC" << std::endl;
            if(maxBE > 0 && maxBE > minBE) dsme.getMAC_PIB().macMaxBE--;
         }
            break;
         case 3: // dec maxBE
         {
            std::cout << "maxINC" << std::endl;
            if(maxBE < 9) dsme.getMAC_PIB().macMaxBE++;
         }
            break;
         case 4: // stay
            std::cout << "nothing" << std::endl;
            break;
         default:
            DSME_ASSERT(false);
     }
}

}; /* DSME */
