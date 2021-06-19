#include "ExpectedSarsa.h"
#include <stdlib.h>
#include <iostream>

/* Get q value for a state action pair. */
auto ExpectedSarsaAgent::getQValue(Q_STATE_TYPE const& state, Q_ACTION_TYPE const& action) -> Q_VALUE_TYPE {
    return this->qTable[state][action];
}

/* Expected Sarsa update. */
auto ExpectedSarsaAgent::update(Q_STATE_TYPE const& state, Q_ACTION_TYPE const& action, Q_REWARD_TYPE const& reward, Q_STATE_TYPE nextState, Q_VALUE_TYPE epsilon) -> void {
    std::array<Q_VALUE_TYPE, NUM_ACTIONS> nextStateProbabilities = this->calculateActionProbabilities(nextState, epsilon);  // Probability for taking each action in next state
    Q_VALUE_TYPE nextStateExpection = 0;
    for(Q_ACTION_TYPE action=0; action<NUM_ACTIONS; action++) { // Q-values for each action in next state
        nextStateExpection += nextStateProbabilities[action] * this->qTable[nextState][action];
    }

    this->qTable[state][action] = this->qTable[state][action] + this->alpha * (reward + this->gamma * nextStateExpection - this->qTable[state][action]);
}

/* Get next action */
auto ExpectedSarsaAgent::getAction(Q_STATE_TYPE const& state, Q_VALUE_TYPE epsilon) -> Q_ACTION_TYPE {
    if((rand()%1000) < (epsilon*1000)) {
        std::cout << "random" << std::endl;
        return rand() % NUM_ACTIONS;
    } else {
        return greedyActionSelection(state);
    }
}

/* Selects action with the highest Q-value for the given state. */
auto ExpectedSarsaAgent::greedyActionSelection(Q_STATE_TYPE const& state) -> Q_ACTION_TYPE {
    // set maximum Q-value to first action
    Q_ACTION_TYPE maxAction = 0;
    Q_VALUE_TYPE maxActionValue = this->qTable[state][maxAction];

    // find maximum Q-value
    for(Q_ACTION_TYPE action=0; action<NUM_ACTIONS; action++) {
        if(this->qTable[state][action] > maxActionValue) {
            maxActionValue = this->qTable[state][action];
            maxAction = action;
        } else if(this->qTable[state][action] == maxActionValue) {
            // with probability 0.5 select other action with same maximum value
            if(rand() % 2 == 1) {   // TODO: replace with openDSME / OMNET++ randomness
                maxAction = action;
            }
        }
    }

    return maxAction;
}

/* Returns the probability of taking each action in the next state. */
auto ExpectedSarsaAgent::calculateActionProbabilities(Q_STATE_TYPE const& state, Q_VALUE_TYPE const& epsilon) -> std::array<Q_VALUE_TYPE, NUM_ACTIONS> {
    std::array<Q_VALUE_TYPE, NUM_ACTIONS> nextStateProbabilities;

    nextStateProbabilities.fill(epsilon / NUM_ACTIONS);
    Q_ACTION_TYPE bestAction = this->greedyActionSelection(state);
    nextStateProbabilities[bestAction] += 1.0 - epsilon;

    return nextStateProbabilities;
}

/* Prints the content of the Q-table */
auto ExpectedSarsaAgent::printQTable() -> void {
    for(Q_STATE_TYPE state=0; state<NUM_STATES; state++) {
        for(Q_ACTION_TYPE action=0; action<NUM_ACTIONS; action++) {
            std::cout << this->qTable[state][action] << " ";
        }
        std::cout << std::endl;
    }
}
