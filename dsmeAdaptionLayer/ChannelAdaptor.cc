
#include "./ChannelAdaptor.h"
#include "../interfaces/IDSMEPlatform.h"
#include "../mac_services/pib/MAC_PIB.h"
#include "../../dsme_platform.h"

namespace dsme {

ChannelAdaptor::ChannelAdaptor(DSMEAdaptionLayer &dsmeAdaptionLayer) : dsmeAdaptionLayer(dsmeAdaptionLayer) {
}

uint8_t ChannelAdaptor::selectChannel() {
    LOG_INFO("SELECTING CHANNEL:");
    printChannelStatusList();

    // TODO create a blacklist and use it to choose a channel

    // return a random channel
    return dsmeAdaptionLayer.getRandom() % dsmeAdaptionLayer.getMAC_PIB().helper.getNumChannels();
}

void ChannelAdaptor::signalTransmissionStatus(uint8_t channel, uint8_t attempts, bool success) {
    auto it = std::find_if(channelStatusList.begin(), channelStatusList.end(), [channel](const std::tuple<uint8_t, uint32_t, uint32_t>& e) {return std::get<0>(e) == channel;});
    if(it != channelStatusList.end()) {
        std::get<1>(*it) += attempts;
        std::get<2>(*it) += success ? 1 : 0;
    }
}

void ChannelAdaptor::printChannelStatusList() {
    LOG_INFO("CHANNEL STATUS LIST");
    for(auto &channelStatus : channelStatusList) {
        LOG_INFO("ch: " << (int)std::get<0>(channelStatus) << "  tx: " << (long)std::get<1>(channelStatus) << "  success: " << (long)std::get<2>(channelStatus));
    }
}

} /* namespace dsme */
