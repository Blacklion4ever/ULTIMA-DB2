#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <gpiod.h>
#include <atomic>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

// ================= QUADRATURE TABLE =================
static const int8_t quad_table[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

class WheelEncoderNode : public rclcpp::Node
{
public:
    WheelEncoderNode() : Node("wheel_encoder_node")
    {
        // -------- Parameters --------
        gpio_chip_ = declare_parameter("gpio_chip", 0);
        gpio_a_ = declare_parameter("gpio_a", 149);
        gpio_b_ = declare_parameter("gpio_b", 200);

        meters_per_tick_ = declare_parameter("meters_per_tick", 0.0285);

        period_ms_   = declare_parameter("period_ms", 150);   // 50 ms
        ema_alpha_   = declare_parameter("ema_alpha", 0.7);   // réactif
        log_console_ = declare_parameter("log_console", true);

        period_sec_ = period_ms_ / 1000.0;

        // -------- ROS --------
        pub_speed_ =
            create_publisher<std_msgs::msg::Float64>("/wheel/speed_mps", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms_),
            std::bind(&WheelEncoderNode::onTimer, this)
        );

        // -------- GPIO --------
        chip_ = gpiod_chip_open_by_number(gpio_chip_);
        if (!chip_)
            throw std::runtime_error("Failed to open gpiochip");

        lineA_ = gpiod_chip_get_line(chip_, gpio_a_);
        lineB_ = gpiod_chip_get_line(chip_, gpio_b_);
        if (!lineA_ || !lineB_)
            throw std::runtime_error("Failed to get GPIO lines");

        gpiod_line_bulk_init(&bulk_);
        gpiod_line_bulk_add(&bulk_, lineA_);
        gpiod_line_bulk_add(&bulk_, lineB_);

        if (gpiod_line_request_bulk_both_edges_events(&bulk_, "wheel_enc") < 0)
            throw std::runtime_error("Failed to request GPIO events");

        prev_state_ = read_state();

        // -------- GPIO thread --------
        gpio_thread_ = std::thread(&WheelEncoderNode::gpioLoop, this);

        RCLCPP_INFO(
            get_logger(),
            "Wheel encoder started | GPIO(%d,%d) | period=%d ms | alpha=%.2f",
            gpio_a_, gpio_b_, period_ms_, ema_alpha_);
    }

    ~WheelEncoderNode()
    {
        stop_ = true;
        if (gpio_thread_.joinable())
            gpio_thread_.join();

        gpiod_line_release(lineA_);
        gpiod_line_release(lineB_);
        gpiod_chip_close(chip_);
    }

private:
    // ================= GPIO =================
    uint8_t read_state()
    {
        return ((gpiod_line_get_value(lineA_) & 1) << 1) |
               (gpiod_line_get_value(lineB_) & 1);
    }

    void gpioLoop()
    {
        while (!stop_)
        {
            timespec timeout{1, 0};

            gpiod_line_bulk ev_bulk;
            gpiod_line_bulk_init(&ev_bulk);

            int ret = gpiod_line_event_wait_bulk(&bulk_, &timeout, &ev_bulk);
            if (ret <= 0)
                continue;

            for (unsigned i = 0; i < gpiod_line_bulk_num_lines(&ev_bulk); ++i)
            {
                gpiod_line_event ev;
                gpiod_line_event_read(
                    gpiod_line_bulk_get_line(&ev_bulk, i), &ev);

                uint8_t curr = read_state();
                uint8_t idx = (prev_state_ << 2) | curr;
                tick_acc_ += quad_table[idx];
                prev_state_ = curr;
            }
        }
    }

    // ================= TIMER =================
    void onTimer()
    {
        int64_t curr = tick_acc_.load();
        int64_t delta = curr - tick_acc_prev_;
        tick_acc_prev_ = curr;

        double tps_inst = delta / period_sec_;
        double speed_inst = tps_inst * meters_per_tick_;

        // EMA
        if (!ema_initialized_) {
            speed_ema_ = speed_inst;
            ema_initialized_ = true;
        } else {
            speed_ema_ = ema_alpha_ * speed_inst +
                         (1.0 - ema_alpha_) * speed_ema_;
        }

        std_msgs::msg::Float64 msg;
        msg.data = speed_ema_;
        pub_speed_->publish(msg);

        if (log_console_) {
            RCLCPP_INFO(
                get_logger(),
                "Δticks=%ld | inst=%.3f m/s | ema=%.3f m/s",
                delta, speed_inst, speed_ema_);
        }
    }

    // ================= ROS =================
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_speed_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ================= GPIO =================
    gpiod_chip* chip_{nullptr};
    gpiod_line* lineA_{nullptr};
    gpiod_line* lineB_{nullptr};
    gpiod_line_bulk bulk_;

    std::thread gpio_thread_;
    std::atomic<bool> stop_{false};

    std::atomic<int64_t> tick_acc_{0};
    int64_t tick_acc_prev_{0};
    uint8_t prev_state_{0};

    // ================= EMA =================
    double speed_ema_{0.0};
    bool ema_initialized_{false};

    // ================= PARAMS =================
    int gpio_chip_;
    int gpio_a_;
    int gpio_b_;
    int period_ms_;
    double period_sec_;
    double meters_per_tick_;
    double ema_alpha_;
    bool log_console_;
};

// ================= MAIN =================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelEncoderNode>());
    rclcpp::shutdown();
    return 0;
}
