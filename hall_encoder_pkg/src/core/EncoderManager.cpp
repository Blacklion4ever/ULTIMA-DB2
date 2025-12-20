#include "EncoderManager.hpp"

EncoderManager::EncoderManager(std::unique_ptr<GpioBackend> gpio)
: gpio_(std::move(gpio)) {}

void EncoderManager::add(int a, int b) {
    auto enc = std::make_unique<Encoder>(a,b);
    Encoder* ptr = enc.get();

    gpio_->watch(a, b, [ptr](bool levelB){
        ptr->onEdgeA(levelB);
    });

    encs_.push_back(std::move(enc));
}

std::vector<Encoder*> EncoderManager::encoders() {
    std::vector<Encoder*> v;
    for (auto& e : encs_) v.push_back(e.get());
    return v;
}
