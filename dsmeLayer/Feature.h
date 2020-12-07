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


template<typename T>
struct ACCUMULATE {
    static auto call(T &value, T &newValue) -> void {
        value += newValue;
    }
};

template<typename T>
struct REPLACE {
    static auto call(T &value, T &newValue) -> void {
        value = newValue;
    }
};

template<typename T>
struct MAXIMUM {
    static auto call(T &value, T &newValue) -> void {
        value = value > newValue ? value : newValue;
    }
};

template<typename T, uint8_t FID, T MIN_VALUE, T MAX_VALUE, template<typename> typename FUNC=REPLACE, UPDATE_RULE UPDATE=UPDATE_RULE::ON_USER_EVENT,  bool ACTIVE=false, T DESIRED_VALUES = 0, UPDATE_RULE RESET=UPDATE_RULE::ON_USER_EVENT, T DEFAULT_VALUE=0>      /* TODO: use min, max value correctly */
class Feature {
public: /* STATIC */
    constexpr static auto isActive() -> bool {
        return ACTIVE;
    }

    constexpr static auto getUpdateRule() -> UPDATE_RULE const {
        return UPDATE;
    }

    constexpr static auto getResetRule() -> UPDATE_RULE const {
        return RESET;
    }

    constexpr static auto getNumValues() -> T const {
        if(DESIRED_VALUES != 0 && MAX_VALUE - MIN_VALUE > DESIRED_VALUES) {
            return DESIRED_VALUES;
        }
        return  MAX_VALUE - MIN_VALUE;
    }

public: /* MEMBER */
    Feature() : updateFunc{}, value{0} {
    }

    auto getValue() -> T const {
        return value;
    }

    auto getId() -> T const {
        return value - MIN_VALUE;
    }

    auto setUpdateFunc(Delegate<T()> updateFunc) -> void {
        this->updateFunc = updateFunc;
    }

    auto update() -> void {
        if(updateFunc) {
            if(getNumValues() == DESIRED_VALUES) {
                T tmp = updateFunc();
                if(tmp >= MAX_VALUE) {
                    return;
                    tmp = MAX_VALUE;
                }
                tmp = tmp * DESIRED_VALUES / (MAX_VALUE - MIN_VALUE);
                FUNC<T>::call(value, tmp);
            } else {
                T tmp = updateFunc();
                if(tmp >= MAX_VALUE) {
                    return;
                    tmp = MAX_VALUE;
                }
                FUNC<T>::call(value, tmp);
            }
            LOG_INFO("FM: Feature " << FID << " updated with rule " << (int)UPDATE << " -> value = " << value);
        }
    }

    auto update(T const& val) -> void {
        T tp = val;
        if(tp > MAX_VALUE) {
            return;
            tp = MAX_VALUE;
        }
        if(getNumValues() == DESIRED_VALUES) {
            T tmp = tp * DESIRED_VALUES / (MAX_VALUE - MIN_VALUE);
            FUNC<T>::call(value, tmp);
        } else {
            T tmp = tp;
            FUNC<T>::call(value, tmp);
        }
        LOG_INFO("FM: Feature " << FID << " updated manually -> value = " << value);
    }

    auto reset() -> void {
        LOG_INFO("FM: Feature " << FID << " reset with rule " << (int)RESET << " -> old value = " << value);
        value = DEFAULT_VALUE;
    }

private:
    Delegate<T()> updateFunc;
    T value;
};


}; /* NAMESPACE DSME */

#endif /* QAGENT_H */
