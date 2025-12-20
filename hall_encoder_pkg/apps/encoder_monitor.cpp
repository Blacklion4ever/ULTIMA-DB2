#include "core/EncoderManager.hpp"
#include "hal/GpioLibgpiod.hpp"
#include <iostream>
#include <unistd.h>

int main() {
    auto gpio = std::make_unique<GpioLibgpiod>();
    EncoderManager mgr(std::move(gpio));

    mgr.add(20,21);
    mgr.add(24,23);

    while (true) {
        auto encs = mgr.encoders();
        for (size_t i=0;i<encs.size();i++) {
            std::cout << "Enc " << i
                      << " ticks=" << encs[i]->ticks()
                      << " vel=" << encs[i]->velocity()
                      << " | ";
        }
        std::cout << "\r" << std::flush;
        usleep(100000);
    }
}
