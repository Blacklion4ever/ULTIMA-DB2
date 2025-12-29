#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <gpiod.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>
#include <fstream>
#include <sstream>
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

static const int8_t quad_table[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

struct LutPoint { double power; double tps; };

class MotorEncoderNode : public rclcpp::Node
{
public:
    MotorEncoderNode() : Node("motor_encoder_node")
    {
        gpio_chip_idx_ = this->declare_parameter("gpio_chip", 0);
        gpio_a_ = this->declare_parameter("gpio_a", 216);
        gpio_b_ = this->declare_parameter("gpio_b", 38);

        // 1) Déclare le paramètre (valeur par défaut = nom du fichier)
        std::string lut_param =
        this->declare_parameter<std::string>("lut_path", "motor_lut.csv");

        // 2) Résout le chemin final
        if (lut_param.empty()) {
        RCLCPP_ERROR(this->get_logger(), "lut_path parameter is empty");
        throw std::runtime_error("lut_path empty");
        }

        // 3) Si chemin absolu → on le prend tel quel
        if (lut_param.front() == '/') {
        lut_path_ = lut_param;
        }
        // 4) Sinon → relatif au share du package
        else {
        const std::string pkg_share =
            ament_index_cpp::get_package_share_directory("hall_encoder_pkg");
        lut_path_ = pkg_share + "/" + lut_param;
        }

        RCLCPP_INFO(this->get_logger(), "Using motor LUT: %s", lut_path_.c_str());

        log_console_ = this->declare_parameter("log_console", true);

        loadLut(lut_path_);

        pub_power_ = this->create_publisher<std_msgs::msg::Float64>("/motor/power_percent", 10);
        timer_ = this->create_wall_timer(50ms, std::bind(&MotorEncoderNode::onTimer, this));

        chip_ = gpiod_chip_open_by_number(gpio_chip_idx_);
        lineA_ = gpiod_chip_get_line(chip_, gpio_a_);
        lineB_ = gpiod_chip_get_line(chip_, gpio_b_);

        gpiod_line_bulk_init(&bulk_);
        gpiod_line_bulk_add(&bulk_, lineA_);
        gpiod_line_bulk_add(&bulk_, lineB_);
        gpiod_line_request_bulk_both_edges_events(&bulk_, "motor_enc");

        prev_state_ = read_state();
        thread_ = std::thread(&MotorEncoderNode::eventLoop, this);
    }

    ~MotorEncoderNode()
    {
        stop_ = true;
        if (thread_.joinable()) thread_.join();
        gpiod_line_release(lineA_);
        gpiod_line_release(lineB_);
        gpiod_chip_close(chip_);
    }

private:
    void loadLut(const std::string &path)
    {
        std::ifstream f(path);
        if (!f.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open LUT: %s", path.c_str());
            return;
        }
        std::string line;
        while (std::getline(f, line))
        {
            if (line.empty() || line[0] == '#') continue;
            std::stringstream ss(line);
            std::string field;
            LutPoint p{};
            std::getline(ss, field, ','); p.power = std::stod(field);
            std::getline(ss, field, ','); p.tps = std::stod(field);
            lut_.push_back(p);
        }
        f.close();
    }

    uint8_t read_state()
    {
        return ((gpiod_line_get_value(lineA_) & 1) << 1) |
               (gpiod_line_get_value(lineB_) & 1);
    }

    double interpPower(double tps)
    {
        if (lut_.empty()) return 0.0;
        for (size_t i = 1; i < lut_.size(); ++i)
        {
            if ((tps >= lut_[i-1].tps && tps <= lut_[i].tps) ||
                (tps <= lut_[i-1].tps && tps >= lut_[i].tps))
            {
                double ratio = (tps - lut_[i-1].tps) / (lut_[i].tps - lut_[i-1].tps);
                return lut_[i-1].power + ratio * (lut_[i].power - lut_[i-1].power);
            }
        }
        return (tps > 0) ? lut_.back().power : lut_.front().power;
    }

    void eventLoop()
    {
        while (!stop_)
        {
            timespec timeout{1, 0};
            gpiod_line_bulk ev;
            gpiod_line_bulk_init(&ev);
            if (gpiod_line_event_wait_bulk(&bulk_, &timeout, &ev) <= 0)
                continue;

            for (unsigned i = 0; i < gpiod_line_bulk_num_lines(&ev); i++)
            {
                gpiod_line_event e;
                gpiod_line_event_read(gpiod_line_bulk_get_line(&ev, i), &e);
                uint8_t curr = read_state();
                uint8_t idx = (prev_state_ << 2) | curr;
                ticks_ += quad_table[idx];
                prev_state_ = curr;
            }
        }
    }

    void onTimer()
    {
        int64_t delta = ticks_.exchange(0);
        double tps = delta / 0.05; // 50 ms window
        double power = interpPower(tps);

        std_msgs::msg::Float64 msg;
        msg.data = power;
        pub_power_->publish(msg);

        if (log_console_)
            RCLCPP_INFO(this->get_logger(), "ticks/s=%.1f -> power=%.1f%%", tps, power);
    }

    // ROS
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_power_;
    rclcpp::TimerBase::SharedPtr timer_;

    // GPIO
    gpiod_chip *chip_;
    gpiod_line *lineA_;
    gpiod_line *lineB_;
    gpiod_line_bulk bulk_;
    uint8_t prev_state_;
    std::atomic<int64_t> ticks_{0};
    std::thread thread_;
    std::atomic<bool> stop_{false};

    // Params
    int gpio_chip_idx_;
    int gpio_a_, gpio_b_;
    std::string lut_path_;
    bool log_console_;
    std::vector<LutPoint> lut_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorEncoderNode>());
    rclcpp::shutdown();
    return 0;
}
