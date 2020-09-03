#ifndef FEATUREMANAGER_H
#define FEATUREMANAGER_H

#include "./State.h"

namespace dsme {

template<typename ...TS>
class FeatureManager {
public: /* MEMBER */
    auto handleSlotEvent(uint8_t slotId, uint8_t superframeId) -> void {
    }

    auto getState() -> State<TS...>& {
        /* Update state if neccessary */
        features.update(UPDATE_RULE::ON_STATE_COLLECTION);
        return features;
    }

private:
    State<TS...> features;
};

}; /* NAMESPACE DMSE */

#endif /* QAGENT_H */
