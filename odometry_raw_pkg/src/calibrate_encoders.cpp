#include <gpiod.h>
#include <atomic>
#include <thread>
#include <iostream>
#include <chrono>
#include <cmath>

static const int8_t quad_table[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

class QuadEncoder
{
public:
    QuadEncoder(int chip, int a, int b)
    {
        chip_ = gpiod_chip_open_by_number(chip);
        lineA_ = gpiod_chip_get_line(chip_, a);
        lineB_ = gpiod_chip_get_line(chip_, b);

        gpiod_line_bulk_init(&bulk_);
        gpiod_line_bulk_add(&bulk_, lineA_);
        gpiod_line_bulk_add(&bulk_, lineB_);
        gpiod_line_request_bulk_both_edges_events(&bulk_, "calib");

        prev_state_ = read_state();
        thread_ = std::thread(&QuadEncoder::loop, this);
    }

    ~QuadEncoder()
    {
        stop_ = true;
        if (thread_.joinable()) thread_.join();
        gpiod_chip_close(chip_);
    }

    int64_t getTicks() const { return ticks_.load(); }
    void reset() { ticks_ = 0; }

private:
    uint8_t read_state()
    {
        return ((gpiod_line_get_value(lineA_) & 1) << 1) |
               (gpiod_line_get_value(lineB_) & 1);
    }

    void loop()
    {
        while (!stop_)
        {
            timespec timeout{1, 0};
            gpiod_line_bulk ev;
            gpiod_line_bulk_init(&ev);

            if (gpiod_line_event_wait_bulk(&bulk_, &timeout, &ev) <= 0)
                continue;

            for (unsigned i = 0; i < gpiod_line_bulk_num_lines(&ev); ++i)
            {
                gpiod_line_event e;
                gpiod_line_event_read(
                    gpiod_line_bulk_get_line(&ev, i), &e);

                uint8_t curr = read_state();
                uint8_t idx = (prev_state_ << 2) | curr;
                ticks_ += quad_table[idx];
                prev_state_ = curr;
            }
        }
    }

    gpiod_chip* chip_;
    gpiod_line* lineA_;
    gpiod_line* lineB_;
    gpiod_line_bulk bulk_;
    std::atomic<int64_t> ticks_{0};
    uint8_t prev_state_;
    std::thread thread_;
    std::atomic<bool> stop_{false};
};

int main()
{
    QuadEncoder motor(0, 216, 38);
    QuadEncoder wheel(0, 149, 200);

    std::cout << "Press ENTER to start calibration...";
    std::cin.get();

    motor.reset();
    wheel.reset();

    std::cout << "Rolling... Press ENTER to stop.\n";

    std::atomic<bool> stop{false};

    // Thread affichage live
    std::thread printer([&]() {
        while (!stop)
        {
            int64_t mt = motor.getTicks();
            int64_t wt = wheel.getTicks();

            std::cout << "\rMotor ticks: " << mt
                      << " | Wheel ticks: " << wt
                      << std::flush;

            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    });

    std::cin.get();   // STOP via ENTER
    stop = true;
    printer.join();

    int64_t motor_ticks = motor.getTicks();
    int64_t wheel_ticks = wheel.getTicks();

    std::cout << "\nEnter measured distance in meters: ";
    double distance;
    std::cin >> distance;

    std::cout << "\nFinal motor ticks: " << motor_ticks << std::endl;
    std::cout << "Final wheel ticks: " << wheel_ticks << std::endl;

    std::cout << "\nmeters_per_tick_motor = "
              << distance / motor_ticks << std::endl;

    std::cout << "meters_per_tick_wheel = "
              << distance / wheel_ticks << std::endl;

    return 0;
}