/**
 * C++ Delegates On Steroids
 * https://github.com/marcmo/delegates
 *
 * Modified for the needs in this framework.
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 oliver.mueller@gmail.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef DELEGATE_H
#define DELEGATE_H

/**
 * Main template for delgates
 *
 * \tparam return_type  return type of the function that gets captured
 * \tparam params       variadic template list for possible arguments
 *                      of the captured function
 */
template<typename return_type, typename... params>
class Delegate; //forward declaration..

template<typename return_type, typename... params>
class Delegate<return_type(params...)> {
    typedef return_type (*Pointer2Function)(void* callee, params...);
public:
    Delegate(void* callee, Pointer2Function function)
        : fpCallee(callee)
        , fpCallbackFunction(function) {
    }

    Delegate()
        : fpCallee(nullptr)
        , fpCallbackFunction(nullptr) {
    }

    operator bool() const {
        return (fpCallbackFunction != nullptr);
    }

    return_type operator()(params... xs) const {
        return (*fpCallbackFunction)(fpCallee, xs...);
    }

    bool operator==(const Delegate& other) const {
        return (fpCallee == other.fpCallee)
               && (fpCallbackFunction == other.fpCallbackFunction);
    }

private:

    void* fpCallee;
    Pointer2Function fpCallbackFunction;
};

/**
 * A DelegateFactory is used to create a Delegate for a certain method call.
 * It takes care of setting up the function that will cast the object that is stored
 * inside the Delegate back to the correct type.
 */
template<typename T, typename return_type, typename... params>
struct DelegateFactory {
    template<return_type (T::*Func)(params...)>
    static return_type MethodCaller(void* o, params... xs) {
        return (static_cast<T*>(o)->*Func)(xs...);
    }

    template<return_type (T::*Func)(params...)>
    inline static Delegate<return_type(params...)> Create(T* o) {
        return Delegate<return_type(params...)>(o, &DelegateFactory::MethodCaller<Func>);
    }
};

/**
 * helper function that is used to deduce the template arguments.
 * will return a DelegateFactory
 */
template<typename T, typename return_type, typename... params>
DelegateFactory<T, return_type, params...> MakeDelegate(return_type (T::*)(params...)) {
    return DelegateFactory<T, return_type, params...>();
}

#define DELEGATE(func, thisPrtRef) (MakeDelegate(func).Create<func>(&thisPrtRef))

#endif
