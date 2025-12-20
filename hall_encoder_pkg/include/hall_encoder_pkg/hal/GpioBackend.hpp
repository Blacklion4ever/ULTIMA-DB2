#pragma once
#include <functional>

class GpioBackend {
public:
    using Callback = std::function<void(bool)>;

    virtual void watch(int gpioA, int gpioB, Callback cb) = 0;
    virtual ~GpioBackend() = default;
};
