#ifndef QAGENT_H
#define QAGENT_H

#include "../mac_services/DSME_Common.h"
#include "../../dsme_platform.h"

namespace dsme {

using action_t = uint8_t;
using reward_t = int32_t;

class DSMELayer;


class QState {
public:
    QState(uint8_t NR, uint8_t NB, uint32_t ts, uint8_t queue, bool txFailed, bool ccaSuccess, uint8_t otherQueue) : nr{NR}, nb{NB}, ts{ts}, queue{queue}, txFailed{txFailed}, ccaSuccess{ccaSuccess}, otherQueue{otherQueue} {
    }

    uint16_t getId() const {
        //return be * NR_STATES * NB_STATES * TS_STATES * QUEUE_STATES * TX_STATES * CCA_STATES * OTHER_QUEUE_STATES + nr * NB_STATES * TS_STATES * QUEUE_STATES * TX_STATES * CCA_STATES * OTHER_QUEUE_STATES + nb * TS_STATES * QUEUE_STATES * TX_STATES * CCA_STATES * OTHER_QUEUE_STATES + ts * QUEUE_STATES * TX_STATES * CCA_STATES * OTHER_QUEUE_STATES + queue  * TX_STATES * CCA_STATES * OTHER_QUEUE_STATES + txFailed * CCA_STATES * OTHER_QUEUE_STATES + ccaFailed * OTHER_QUEUE_STATES + otherQueue;
        //return nr * NB_STATES * QUEUE_STATES * OTHER_QUEUE_STATES * TS_STATES + nb * QUEUE_STATES * OTHER_QUEUE_STATES * TS_STATES + queue * OTHER_QUEUE_STATES * TS_STATES + otherQueue * TS_STATES + ts; //+ ccaSuccess;
        return ts; // * QUEUE_STATES * OTHER_QUEUE_STATES + queue * OTHER_QUEUE_STATES + otherQueue;
    }

    void print() const {
        LOG_INFO(" NR: " << (int)nr << " NB: " << (int)nb << " TS: " << (int)ts << " Q: " << (int)queue << " TX-FAILED: " << (int)txFailed << " CCA-SUCCESS: " << (int)ccaSuccess << " otherQueue: " << (int)otherQueue);
    }

    static constexpr int getMaxId() {
        //return BE_STATES * NR_STATES * NB_STATES * TS_STATES * QUEUE_STATES * TX_STATES * CCA_STATES * OTHER_QUEUE_STATES;
        //return NR_STATES * NB_STATES * QUEUE_STATES * OTHER_QUEUE_STATES * TS_STATES;
        return TS_STATES; // * QUEUE_STATES * OTHER_QUEUE_STATES;
    }

    uint8_t getRetransmissions() const {
        return nr;
    }

    uint8_t getBackoffs() const {
        return nb;
    }

    uint32_t getTime() const {
        return ts;
    }

private:
    static uint8_t const NR_STATES = 5;
    static uint8_t const NB_STATES = 5;
    static uint32_t const TS_STATES = 80;
    static uint8_t const QUEUE_STATES = 9;
    static uint8_t const TX_STATES = 2;
    static uint8_t const CCA_STATES = 2;
    static uint8_t const OTHER_QUEUE_STATES = 9;

    uint8_t nr;
    uint8_t nb;
    uint32_t ts;
    uint8_t queue;
    bool txFailed;
    bool ccaSuccess;
    uint8_t otherQueue;
};

enum class QAction : action_t {
    BACKOFF,
    CCA,
    //SEND,
    NUM_ACTIONS
};


class QAgent {
public:
    QAgent(DSMELayer &dsme, float eps=1.0, float eps_min=0.02, float eps_decay=0.999, float gamma=0.9, float lr=0.1);

    action_t selectAction(bool deterministic=false);
    void update(bool ccaSuccess, bool txSuccess, bool queueFull, uint8_t NR, uint8_t NB, uint32_t dwellTime);

    void printQTable() const;

    /* Notifies about parent's queue level */
    void signalQueueLevelCAP(uint8_t queueLevel);

    void printTxTimes() const;

private:
    float maxQ(QState const &state) const;
    action_t maxAction(QState const &state) const;
    void updateQTable(QState const &state, QState const &nextState, action_t const &action, reward_t reward);

private:
    DSMELayer &dsme;
    float eps;
    float eps_min;
    float eps_decay;
    float gamma;
    float lr;
    QState lastState;
    action_t lastAction;
    float qTable[QState::getMaxId()][(action_t)QAction::NUM_ACTIONS];
    uint8_t otherQueue;
};

};
#endif /* QAGENT_H */
