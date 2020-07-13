#ifndef QAGENT_H
#define QAGENT_H

#include "../mac_services/DSME_Common.h"
#include "../../dsme_platform.h"

namespace dsme {

using action_t = uint8_t;
using reward_t = int32_t;

class DSMELayer;


class QState {
public: //TODO INCLUDE CURRENT CAP SLOT, compress "success, retransmissions, backoffs" to toal
    QState(uint8_t be, uint8_t queueLevel, bool success, uint8_t retransmissions, uint8_t backoffs, uint8_t slot) : be{be}, queueLevel(queueLevel), success{success}, retransmissions{retransmissions}, backoffs{backoffs}, slot{slot} {
    }

    uint16_t getId() const {
        return retransmissions * BACKOFF_STATES * SLOT_STATES + backoffs * SLOT_STATES + slot;
        // be * QUEUE_LEVEL_STATES * SUCCESS_STATES * RETRANSMISSION_STATES * BACKOFF_STATES + (int)success * RETRANSMISSION_STATES * BACKOFF_STATES * SLOT_STATES + queueLevel * RETRANSMISSION_STATES * BACKOFF_STATES * SLOT_STATES
    }

    void print() const {
        LOG_INFO("BE: " << (int)be << " Q: " << (int)queueLevel << " SUC: " << success << " RT: " << (int)retransmissions << " NB: " << (int)backoffs << " S: " << (int)slot);
    }

    static constexpr int getMaxId() {
        return RETRANSMISSION_STATES * BACKOFF_STATES * SLOT_STATES;
    }

    uint8_t getRetransmissions() const {
        return retransmissions;
    }

    uint8_t getBackoffs() const {
        return backoffs;
    }

private:
    static uint8_t const BE_STATES = 11;
    static uint8_t const QUEUE_LEVEL_STATES = 21;
    static uint8_t const SUCCESS_STATES = 2;
    static uint8_t const RETRANSMISSION_STATES = 5;
    static uint8_t const BACKOFF_STATES = 5;
    static uint8_t const SLOT_STATES = 16;

    uint8_t be;
    uint8_t queueLevel;
    bool success;
    uint8_t retransmissions;
    uint8_t backoffs;
    uint8_t slot;
};


class QAgent {
public:
    QAgent(DSMELayer &dsme, float eps=1.0, float eps_min=0.00, float eps_decay=1.0, float gamma=0.8, float lr=0.15);

    void handleStartOfCFP(uint8_t NR, uint8_t NB, uint32_t lastWaitTime);

    void onCSMASent(DataStatus::Data_Status status, uint8_t numBackoffs, uint32_t dwellTime);

    action_t selectAction(QState const &state, bool deterministic=false);

    void updateQTable(QState const &state, QState const &nextState, action_t const &action, reward_t reward);

    void printQTable() const;

private:
    float maxQ(QState const &state) const;

    action_t maxAction(QState const &state) const;

private:
    static uint8_t const ACTION_STATES = 10;
    DSMELayer &dsme;
    float eps;
    float eps_min;
    float eps_decay;
    float gamma;
    float lr;
    QState lastState;
    action_t lastAction;
    float qTable[QState::getMaxId()][ACTION_STATES];

    /* PRR */
    bool success;
    uint32_t dwellTime;
};

};
#endif /* QAGENT_H */
