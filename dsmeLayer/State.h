#ifndef STATE_H
#define STATE_H

#include "./Feature.h"
#include <typeinfo> //TODO remove -> just for debugging

namespace dsme {

using state_t = uint16_t;

/* TODO: THIS UGLY AND OVERHEAD !!! */
struct same_type {
    constexpr static bool value = true;
};
struct different_type {
    constexpr static bool value = false;
};
template<typename T, typename U>
struct is_same : different_type {
};
template<typename T>
struct is_same<T, T> : same_type {
};
/* TODO: THIS UGLY AND OVERHEAD !!! */



/* Holding arbitrary number of features */
template<typename T, typename ...TS>
class State : State<TS...> {
public: /* STATIC */
    constexpr static auto getMaxId() -> state_t {
        if(T::isActive()) {
            return T::getNumValues() * State<TS...>::getMaxId();
        } else {
            return State<TS...>::getMaxId();
        }
    }

public: /* MEMBER */
    State() = default;
    virtual ~State() = default;

    auto getId() -> state_t {
        if(T::isActive()) {
            return feature.getId() * State<TS...>::getMaxId() + State<TS...>::getId();  // TODO: avoid recursive call
        } else {
            return State<TS...>::getId();
        }
    }

    template<typename F>
    auto getFeature() -> F& {
        if constexpr(is_same<T, F>::value) {
            return feature;
        } else {
            return State<TS...>::template getFeature<F>();
        }
    }

    auto update(UPDATE_RULE const& rule) -> void  {
        if(T::getUpdateRule() == rule) {
            feature.update();
        }
        State<TS...>::update(rule);
    }

    auto reset(UPDATE_RULE const& rule) -> void {
        if(T::getResetRule() == rule) {
            feature.reset();
        }
        State<TS...>::reset(rule);
    }

private:
    T feature;
};

/* Specialization for only one feature */
template<typename T>
class State<T> {
public: /* STATIC */
    constexpr static auto getMaxId() -> state_t {
        if(T::isActive()) {
            return T::getNumValues();
        } else {
            return 1;
        }
    }

public: /* MEMBER */
    State() = default;
    virtual ~State() = default;

    auto getId() -> state_t {
        if(T::isActive()) {
            return feature.getValue();
        } else {
            return 0;
        }
    }

    template<typename F>
    auto getFeature() -> T& {
        return feature;
    }

    auto update(UPDATE_RULE const& rule) -> void {
        if(T::getUpdateRule() == rule) {
            feature.update();
        }
        //LOG_INFO("FM: All features updated with rule " << (int)rule);
    }

    auto reset(UPDATE_RULE const& rule) -> void {
        if(T::getResetRule() == rule) {
            feature.reset();
        }
        //LOG_INFO("FM: All features reset with rule " << (int)rule);
    }

private:
    T feature;
};

}; /* NAMESPACE DSME */

#endif /* QAGENT_H */
