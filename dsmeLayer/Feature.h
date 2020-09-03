#ifndef FEATURE_H
#define FEATURE_H

#include "../mac_services/DSME_Common.h"

namespace dsme {

enum class UPDATE_RULE {
    ON_SLOT_EVENT,
    ON_CAP_EVENT,
    ON_CFP_EVENT,
    ON_MSF_EVENT,
    ON_USER_EVENT,
    ON_STATE_COLLECTION
};

template<typename T, T MIN_VALUE, T MAX_VALUE, UPDATE_RULE UPDATE, bool ACTIVE=false, T DESIRED_VALUES = 0>      /* TODO: use min, max value correctly */
class Feature {
public: /* STATIC */
    constexpr static auto isActive() -> bool {
        return ACTIVE;
    }

    constexpr static auto getUpdateRule() -> UPDATE_RULE const {
        return UPDATE;
    }

    constexpr static auto getNumValues() -> T const {
        if(DESIRED_VALUES != 0 && MAX_VALUE - MIN_VALUE > DESIRED_VALUES) {
            return DESIRED_VALUES;
        }
        return  MAX_VALUE - MIN_VALUE;
    }

public: /* MEMBER */
    auto getValue() -> T const {
        return value;
    }

    auto getId() -> T const {
        return value - MIN_VALUE;
    }

    auto setUpdateFunc(Delegate<T()> updateFunc) {
        this->updateFunc = updateFunc;
    }

    auto update() -> void {
        if(updateFunc) {
            if(getNumValues() == DESIRED_VALUES) {
                value = updateFunc() * DESIRED_VALUES / (MAX_VALUE - MIN_VALUE);
            } else {
                value = updateFunc();
            }
            LOG_INFO("FM: Feature updated with rule " << (int)UPDATE << " -> value = " << value);
        }
    }

    auto update(T const& val) -> void {
        if(getNumValues() == DESIRED_VALUES) {
            value = val * DESIRED_VALUES / (MAX_VALUE - MIN_VALUE);
        } else {
            value = val;
        }
        LOG_INFO("FM: Feature updated manually -> value = " << value);
    }

private:
    Delegate<T()> updateFunc;
    T value;
};


}; /* NAMESPACE DSME */

#endif /* QAGENT_H */
