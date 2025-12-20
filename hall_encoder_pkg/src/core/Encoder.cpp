#include "Encoder.hpp"

Encoder::Encoder(int, int) {
    last_ = std::chrono::steady_clock::now();
}

void Encoder::onEdgeA(bool levelB) {
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_).count();

    ticks_ += (levelB ? -1 : +1);
    if (dt > 0) velocity_ = 1.0 / dt;

    last_ = now;
}
