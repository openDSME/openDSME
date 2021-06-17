#ifndef EXPECTED_SARSA_H
#define EXPECTED_SARSA_H

#include <stdint.h>
#include <array>

#define NUM_STATES 16
#define NUM_ACTIONS 16

using Q_STATE_TYPE = uint8_t;
using Q_ACTION_TYPE = uint8_t;
using Q_VALUE_TYPE = float;
using Q_REWARD_TYPE = Q_VALUE_TYPE;


class ExpectedSarsaAgent {
public:
    ExpectedSarsaAgent(Q_VALUE_TYPE alpha = 0.5, Q_VALUE_TYPE gamma = 0.99) : qTable{0}, alpha{alpha}, gamma{gamma} {
    }
    ~ExpectedSarsaAgent() = default;

    /* Get q value for a state action pair. */
    auto getQValue(Q_STATE_TYPE const& state, Q_ACTION_TYPE const& action) -> Q_VALUE_TYPE;

    /* Expected Sarsa update. */
    auto update(Q_STATE_TYPE const& state, Q_ACTION_TYPE const& action, Q_REWARD_TYPE const& reward, Q_STATE_TYPE nextState, Q_VALUE_TYPE epsilon) -> void;

    /* Selects action with the highest Q-value for the given state. */
    auto greedyActionSelection(Q_STATE_TYPE const& state) -> Q_ACTION_TYPE;

    /* Returns the probability of taking each action in the next state. */
    auto calculateActionProbabilities(Q_STATE_TYPE const& state, Q_VALUE_TYPE const& epsilon) -> std::array<Q_VALUE_TYPE, NUM_ACTIONS>;

    /* Prints the current Q-table */ 
    auto printQTable() -> void;

    auto setAlpha(Q_VALUE_TYPE alpha) -> void {
        this->alpha = alpha;
    }

    auto setGamma(Q_VALUE_TYPE gamma) -> void {
        this->gamma = gamma;
    }

private:
    // state and actions are consecutively numbered -> 0,1,2,3,...
    Q_VALUE_TYPE qTable[NUM_STATES][NUM_ACTIONS];

    Q_VALUE_TYPE alpha;
    Q_VALUE_TYPE gamma;
};

#endif /* EXPECTED_SARSA_H */
