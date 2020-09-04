#ifndef FEATUREMANAGER_H
#define FEATUREMANAGER_H

#include "./State.h"

namespace dsme {

template<typename ...TS>
class FeatureManager {
public: /* MEMBER */
    auto handleSlotEvent(uint8_t slotId, uint8_t superframeId) -> void {
        features.update(UPDATE_RULE::ON_SLOT_EVENT);
        features.reset(UPDATE_RULE::ON_SLOT_EVENT);

        if(slotId == 0) {
            features.update(UPDATE_RULE::ON_MSF_EVENT);
            features.reset(UPDATE_RULE::ON_MSF_EVENT);
        }

        //TODO ON_CAP and ON_CFP require access to MAC_PIB
    }

    auto getState() -> State<TS...>& {
        /* Update state if neccessary */
        features.update(UPDATE_RULE::ON_STATE_COLLECTION);
        features.reset(UPDATE_RULE::ON_STATE_COLLECTION); // DOES THIS MAKE SENSE?
        return features;
    }

private:
    State<TS...> features;
};

}; /* NAMESPACE DMSE */

#endif /* QAGENT_H */
