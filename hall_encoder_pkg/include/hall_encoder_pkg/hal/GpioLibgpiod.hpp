#pragma once
#include "GpioBackend.hpp"
#include <gpiod.h>
#include <thread>

class GpioLibgpiod : public GpioBackend {
public:
    explicit GpioLibgpiod(const char* chipname = "gpiochip0");
    ~GpioLibgpiod();

    void watch(int gpioA, int gpioB, Callback cb) override;

private:
    gpiod_chip* chip_;
};
