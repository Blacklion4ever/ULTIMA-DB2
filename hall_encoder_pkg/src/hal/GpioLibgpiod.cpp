#include "GpioLibgpiod.hpp"
#include <stdexcept>
#include <iostream>

GpioLibgpiod::GpioLibgpiod(const char* chipname) {
    chip_ = gpiod_chip_open_by_name(chipname);
    if (!chip_)
        throw std::runtime_error("Cannot open gpio chip");
}

GpioLibgpiod::~GpioLibgpiod() {
    if (chip_) gpiod_chip_close(chip_);
}

void GpioLibgpiod::watch(int gpioA, int gpioB, Callback cb) {
    auto lineA = gpiod_chip_get_line(chip_, gpioA);
    auto lineB = gpiod_chip_get_line(chip_, gpioB);

    if (!lineA || !lineB)
        throw std::runtime_error("Invalid GPIO line");

    gpiod_line_request_both_edges_events(
        lineA, "encoderA"
    );

    gpiod_line_request_input(
        lineB, "encoderB"
    );

    std::thread([=]() {
        while (true) {
            gpiod_line_event ev;
            int ret = gpiod_line_event_wait(lineA, nullptr);
            if (ret <= 0) continue;

            gpiod_line_event_read(lineA, &ev);

            if (ev.event_type == GPIOD_LINE_EVENT_RISING_EDGE) {
                bool levelB = gpiod_line_get_value(lineB);
                cb(levelB);
            }
        }
    }).detach();
}
