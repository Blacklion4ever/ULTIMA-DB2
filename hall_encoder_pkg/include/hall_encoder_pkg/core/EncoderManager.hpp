#pragma once
#include "Encoder.hpp"
#include "hal/GpioBackend.hpp"
#include <memory>
#include <vector>

class EncoderManager {
public:
    EncoderManager(std::unique_ptr<GpioBackend> gpio);

    void add(int gpioA, int gpioB);
    std::vector<Encoder*> encoders();

private:
    std::vector<std::unique_ptr<Encoder>> encs_;
    std::unique_ptr<GpioBackend> gpio_;
};
