#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
  #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
  #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#include <gpiod.h>
#include <atomic>
#include <thread>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

// ================= QUADRATURE TABLE =================
static const int8_t quad_table[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

// ====================================================
// ================= ENCODER CLASS ====================
// ====================================================

class QuadEncoder
{
public:
    QuadEncoder(int chip_idx, int gpio_a, int gpio_b)
    {
        chip_ = gpiod_chip_open_by_number(chip_idx);
        lineA_ = gpiod_chip_get_line(chip_, gpio_a);
        lineB_ = gpiod_chip_get_line(chip_, gpio_b);

        gpiod_line_bulk_init(&bulk_);
        gpiod_line_bulk_add(&bulk_, lineA_);
        gpiod_line_bulk_add(&bulk_, lineB_);
        gpiod_line_request_bulk_both_edges_events(&bulk_, "quad_enc");

        prev_state_ = read_state();
        thread_ = std::thread(&QuadEncoder::loop, this);
    }

    ~QuadEncoder()
    {
        stop_ = true;
        if (thread_.joinable()) thread_.join();
        gpiod_line_release(lineA_);
        gpiod_line_release(lineB_);
        gpiod_chip_close(chip_);
    }

    int64_t getTicks()
    {
        return ticks_.load();
    }

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

// ====================================================
// ================= ODOM NODE ========================
// ====================================================

class OdomNode : public rclcpp::Node
{
public:
    OdomNode() : Node("odom_node")
    {
        // -------- Parameters --------
        meters_per_tick_motor_ = declare_parameter("meters_per_tick_motor", 0.0001);
        meters_per_tick_wheel_ = declare_parameter("meters_per_tick_wheel", 0.0285);
        wheelbase_             = declare_parameter("wheelbase", 0.30);
        update_period_ms_      = declare_parameter("update_period_ms", 50);

        sigma_v_base_ = declare_parameter("sigma_v_base", 0.05);
        sigma_v_gain_ = declare_parameter("sigma_v_gain", 0.5);
        sigma_yaw_    = declare_parameter("sigma_yaw", 0.1);

        int chip = declare_parameter("gpio_chip", 0);

        int motor_a = declare_parameter("motor_gpio_a", 216);
        int motor_b = declare_parameter("motor_gpio_b", 38);

        int wheel_a = declare_parameter("wheel_gpio_a", 149);
        int wheel_b = declare_parameter("wheel_gpio_b", 200);

        // -------- Encoders --------
        motor_encoder_ = std::make_unique<QuadEncoder>(chip, motor_a, motor_b);
        wheel_encoder_ = std::make_unique<QuadEncoder>(chip, wheel_a, wheel_b);

        // -------- Steering --------
        steering_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/steering/angle",
            10,
            [this](std_msgs::msg::Float64::SharedPtr msg)
            {
                steering_angle_ = msg->data;
            });

        // -------- Odom pub --------
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom_raw", 10);
        //tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(update_period_ms_),
            std::bind(&OdomNode::update, this));

        last_time_ = now();
    }

private:
    void update()
    {
        auto now_time = now();
        double dt = (now_time - last_time_).seconds();
        last_time_ = now_time;

        if (dt <= 0.0)
            return;

        int64_t motor_ticks = motor_encoder_->getTicks();
        int64_t wheel_ticks = wheel_encoder_->getTicks();

        int64_t delta_motor = motor_ticks - motor_prev_;
        int64_t delta_wheel = wheel_ticks - wheel_prev_;

        motor_prev_ = motor_ticks;
        wheel_prev_ = wheel_ticks;

        double dist_motor = delta_motor * meters_per_tick_motor_;
        double dist_wheel = delta_wheel * meters_per_tick_wheel_;

        double v = dist_motor / dt;
        double delta_v = (dist_motor - dist_wheel) / dt;

        double yaw_rate = (v / wheelbase_) * std::tan(steering_angle_);

        yaw_ += yaw_rate * dt;
        x_ += v * std::cos(yaw_) * dt;
        y_ += v * std::sin(yaw_) * dt;

        publish(now_time, v, yaw_rate, delta_v);
    }

    void publish(rclcpp::Time stamp,
                 double v,
                 double yaw_rate,
                 double delta_v)
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_);
        odom.pose.pose.orientation = tf2::toMsg(q);

        odom.twist.twist.linear.x = v;
        odom.twist.twist.angular.z = yaw_rate;

        double sigma_v = sigma_v_base_ +
                         sigma_v_gain_ * std::abs(delta_v);
        double var_v = sigma_v * sigma_v;

        odom.twist.covariance[0] = var_v;
        odom.twist.covariance[35] = sigma_yaw_ * sigma_yaw_;

        odom_pub_->publish(odom);

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = x_;
        tf.transform.translation.y = y_;
        tf.transform.rotation = tf2::toMsg(q);

        // tf_broadcaster_->sendTransform(tf);
    }

    // Encoders
    std::unique_ptr<QuadEncoder> motor_encoder_;
    std::unique_ptr<QuadEncoder> wheel_encoder_;

    int64_t motor_prev_{0};
    int64_t wheel_prev_{0};

    // State
    double x_{0.0};
    double y_{0.0};
    double yaw_{0.0};
    double steering_angle_{0.0};

    rclcpp::Time last_time_;

    // Params
    double meters_per_tick_motor_;
    double meters_per_tick_wheel_;
    double wheelbase_;
    int update_period_ms_;

    double sigma_v_base_;
    double sigma_v_gain_;
    double sigma_yaw_;

    // ROS
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_sub_;
    //std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomNode>());
    rclcpp::shutdown();
    return 0;
}