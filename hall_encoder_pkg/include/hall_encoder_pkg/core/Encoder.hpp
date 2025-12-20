#pragma once
#include <atomic>
#include <chrono>

class Encoder {
public:
    Encoder(int gpioA, int gpioB);

    void onEdgeA(bool levelB);

    int64_t ticks() const { return ticks_.load(); }
    double velocity() const { return velocity_.load(); }

private:
    std::atomic<int64_t> ticks_{0};
    std::atomic<double> velocity_{0.0};
    std::chrono::steady_clock::time_point last_;
};
