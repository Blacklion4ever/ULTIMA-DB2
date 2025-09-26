#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <openhmd.h>
#include <cmath>
#include <cstring>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_srvs/srv/trigger.hpp>
#define _USE_MATH_DEFINES

struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};
class DK2Node : public rclcpp::Node {

    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_pan_tilt;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;


    ohmd_context* ctx = nullptr;
    ohmd_device* device = nullptr;

    tf2::Quaternion q0;       // orientation de référence
    rclcpp::TimerBase::SharedPtr timer_;


public:
    DK2Node() : Node("dk2_node") {
        // Init OpenHMD
        ohmd_require_version(0, 3, 0);
        ctx = ohmd_ctx_create();
        if (!ctx) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create OpenHMD context");
            return;
        }
        int num_devices = ohmd_ctx_probe(ctx);
        if (num_devices <= 0) {
            RCLCPP_ERROR(this->get_logger(), "No OpenHMD devices found");
            return;
        }

        int dk2_index = -1;
        for (int i = 0; i < num_devices; i++) {
            const char* vendor = ohmd_list_gets(ctx, i, OHMD_VENDOR);
            if (strstr(vendor, "Oculus") != NULL) {
                dk2_index = i;
                break;
            }
        }

        if (dk2_index == -1) {
            RCLCPP_ERROR(this->get_logger(), "No DK2 device found");
            return;
        }

        device = ohmd_list_open_device(ctx, dk2_index);
        if (!device) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open DK2 device");
            return;
        }

        pub_pan_tilt = this->create_publisher<std_msgs::msg::Int16MultiArray>("/command/pan_tilt", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&DK2Node::publish_pan_tilt, this)
        );

        //ros2 service call /reset_imu std_srvs/srv/Trigger
        reset_service_ = this->create_service<std_srvs::srv::Trigger>(
            "reset_imu",
            std::bind(&DK2Node::resetIMUCallback, this,
                      std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "DK2 node started, publishing pan/tilt");
    }

    ~DK2Node() {
        if (ctx) ohmd_ctx_destroy(ctx);
    }

private:
    void resetIMUCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        float q[4];
        if (ohmd_device_getf(device, OHMD_ROTATION_QUAT, q) != 0) {
            res->success = false;
            res->message = "Failed to read DK2 orientation";
            return;
        }
        q0 = tf2::Quaternion(q[0], q[1], q[2], q[3]);
        res->success = true;
        res->message = "DK2 IMU reset";
    }
    void publish_pan_tilt() {
        ohmd_ctx_update(ctx);

        float q[4]; // x, y, z, w
        if (ohmd_device_getf(device, OHMD_ROTATION_QUAT, q) != 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to read rotation from DK2");
            return;
        }

        printf("quat: % 4.4f, % 4.4f, % 4.4f, % 4.4f\n", q[0], q[1], q[2], q[3]);
        tf2::Quaternion qt(q[0],q[1],q[2],q[3]);
        tf2::Quaternion qt_rel = q0.inverse() * qt;
        tf2::Matrix3x3 m(qt_rel);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw); // radians

        // Convertir en degrés
        float pitch_deg = pitch * 180.0 / M_PI;  // yaw de la tête → Pan
        float yaw_deg   = yaw * 180.0 / M_PI;
        float roll_deg  = roll * 180.0 / M_PI;

        // Re-mapper pour Pan/Tilt selon ton repère
        double pan_deg  = -pitch * 180.0 / M_PI;  // yaw de la tête → Pan
        double tilt_deg = -roll * 180.0 / M_PI;  // pitch de la tête → Tilt

        // --- Publier ---
        auto msg = std_msgs::msg::Int16MultiArray();
        msg.data.resize(2);
        msg.data[0] = static_cast<int16_t>(pan_deg);   // Pan
        msg.data[1] = static_cast<int16_t>(tilt_deg); // Tilt

        pub_pan_tilt->publish(msg);

        RCLCPP_INFO(this->get_logger(), "yaw_deg=%.1f°, pitch_deg=%.1f°, roll_deg=%.1f°", yaw_deg, pitch_deg, roll_deg);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DK2Node>());
    rclcpp::shutdown();
    return 0;
}
