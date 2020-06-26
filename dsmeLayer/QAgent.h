#ifndef QAGENT_H
#define QAGENT_H

#include <stdint.h>

namespace dsme {

using action_t = uint8_t;
using reward_t = int8_t;

class DSMELayer;

class QState {
public:
    QState(uint8_t minBE, uint8_t maxBE, uint8_t queueLevel) : minBE(minBE), maxBE(maxBE), queueLevel(queueLevel) {
    }

    uint16_t getId() const {
        return minBE * MAX_BE_STATES * QUEUE_LEVEL_STATES + maxBE * QUEUE_LEVEL_STATES + queueLevel;
    }

    static constexpr int getMaxId() {
        return MIN_BE_STATES * MAX_BE_STATES * QUEUE_LEVEL_STATES;
    }


private:
    static uint8_t const MIN_BE_STATES = 10;
    static uint8_t const MAX_BE_STATES = 10;
    static uint8_t const QUEUE_LEVEL_STATES = 21;

    uint8_t minBE;
    uint8_t maxBE;
    uint8_t queueLevel;
};


class QAgent {
public:
    QAgent(DSMELayer &dsme, uint8_t eps=30, float gamma=0.99, float lr=0.6);

    void handleStartOfCFP(uint16_t currentSuperframe, uint16_t currentMultiSuperframe);

    action_t selectAction(QState const &state, bool deterministic=false) const;

    void updateQTable(QState const &state, action_t const &action, reward_t reward);

    void printQTable() const;

private:
    float maxQ(QState const &state) const;

    action_t maxAction(QState const &state) const;

private:
    static uint8_t const ACTION_STATES = 5;
    DSMELayer &dsme;
    uint8_t eps;
    float gamma;
    float lr;
    QState lastState;
    action_t lastAction;
    float qTable[QState::getMaxId()][ACTION_STATES];
};

};
#endif /* QAGENT_H */
