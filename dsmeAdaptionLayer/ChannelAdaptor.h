#ifndef CHANNELADAPTOR_H_
#define CHANNELADAPTOR_H_

#include "../mac_services/DSME_Common.h"

#include <vector>
#include <tuple>
#include <algorithm>

namespace dsme {

class DSMEAdaptionLayer;

class ChannelAdaptor {
public:
    ChannelAdaptor(DSMEAdaptionLayer&);

    /** Selects a channel for the next GTS allocation.
     *  Returns an index to a channel in the channel
     *  map, i.e., idx=0->ch=11 / idx=2->ch=13.
     *
     *  TODO: TO BE IMPLEMENTED BY YOU!
     */
    uint8_t selectChannel();


    /** Collects information about transmissions and
     *  channel quality.
     */
    void signalTransmissionStatus(uint8_t channel, uint8_t attempts, bool success);

private:
    std::vector<std::tuple<uint8_t, uint32_t, uint32_t>> channelStatusList; // Vector of tuples (channel, transmissions, successful transmissions)
    DSMEAdaptionLayer &dsmeAdaptionLayer;

    void printChannelStatusList();
};

} /* namespace dsme */

#endif /* CHANNELADAPTOR_H_ */
