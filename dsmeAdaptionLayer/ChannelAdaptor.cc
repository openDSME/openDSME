
#include "./ChannelAdaptor.h"
#include "../interfaces/IDSMEPlatform.h"
#include "../mac_services/pib/MAC_PIB.h"
#include "../../dsme_platform.h"

namespace dsme {

ChannelAdaptor::ChannelAdaptor(DSMEAdaptionLayer &dsmeAdaptionLayer) : dsmeAdaptionLayer{dsmeAdaptionLayer}, agent{ExpectedSarsaAgent()} {
}

uint8_t ChannelAdaptor::selectChannel(uint8_t slotId) {
    LOG_INFO("SELECTING CHANNEL:");
    printChannelStatusList();

    // state is given by the time slot to allocate
    Q_STATE_TYPE currentState = slotId;

    // Select the next action based on the current state
    Q_ACTION_TYPE_TYPE action = agent.greedyActionSelection(currentState);

    // Delay update to a later point in time

    // return a random channel
    //return dsmeAdaptionLayer.getRandom() % dsmeAdaptionLayer.getMAC_PIB().helper.getNumChannels();
    return action;
}

bool ChannelAdaptor::checkDeallocateGTS(uint8_t channel) {
    //TODO implement
    return false;
}

void ChannelAdaptor::signalTransmissionStatus(uint8_t channel, uint8_t attempts, bool success) {
    // calcualte reward for transmission
    Q_REWARD_TYPE reward = 10 * success;

    // get current state (one of the 16 time slots per superframe)
    Q_STATE_TYPE currentState = dsmeAdaptionLayer.getDSME().getCurrentSlot();

    // update Sarsa based on transmission where channel was the action
    agent.update(currentState, channel, reward, (currentState+1)%16, epsilon);

    // update epsilon
    epsilon *= 0.9999;


    // update prr of all channels -> not of interest
    auto it = std::find_if(channelStatusList.begin(), channelStatusList.end(), [channel](const std::tuple<uint8_t, uint32_t, uint32_t>& e) {return std::get<0>(e) == channel;});
    if(it != channelStatusList.end()) {
        std::get<1>(*it) += attempts;
        std::get<2>(*it) += success ? 1 : 0;
    } else {
        channelStatusList.push_back(std::tuple<uint8_t, uint32_t, uint32_t>(channel, attempts, success));
    }
}

void ChannelAdaptor::printChannelStatusList() {
    LOG_INFO("CHANNEL STATUS LIST");
    for(auto &channelStatus : channelStatusList) {
        LOG_INFO("ch: " << (int)std::get<0>(channelStatus) << "  tx: " << (long)std::get<1>(channelStatus) << "  success: " << (long)std::get<2>(channelStatus));
    }
}

} /* namespace dsme */
