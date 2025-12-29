#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <openhmd.h>
#include <cmath>
#include <cstring>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_srvs/srv/trigger.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

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
    // dans la classe
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;

    // membre
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub_;
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

          // dans le constructeur
        tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("hmd/look", 10);
        pub_pan_tilt = this->create_publisher<std_msgs::msg::Int16MultiArray>("/command/pan_tilt", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
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
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /* unused */,
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

    float raw[4]; // x,y,z,w
    if (ohmd_device_getf(device, OHMD_ROTATION_QUAT, raw) != 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to read rotation from DK2");
        return;
    }

    tf2::Quaternion q_meas(raw[0], raw[1], raw[2], raw[3]); q_meas.normalize();
    tf2::Quaternion q_ref = q0; q_ref.normalize();

    // relatif DK2
    tf2::Quaternion q_rel = q_meas * q_ref.inverse();

    // DK2 (+X right, +Y up, +Z back) -> ENU (X fwd, Y left, Z up)
    const tf2::Quaternion S(0.5, -0.5, -0.5, 0.5);
    tf2::Quaternion q_rel_enu = S * q_rel * S.inverse();
    q_rel_enu.normalize();

    // TF: base_link -> hmd (orientation ENU)
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "base_link";
    t.child_frame_id  = "hmd";
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation = tf2::toMsg(q_rel_enu);
    tf_br_->sendTransform(t);

    // Pan/Tilt robustes (pas d’Euler)
    tf2::Matrix3x3 R_enu(q_rel_enu);
    tf2::Vector3 f = R_enu * tf2::Vector3(1,0,0); // forward (X local)
    tf2::Vector3 l = R_enu * tf2::Vector3(0,1,0); // left    (Y local)

    const double rxy = std::hypot(f.x(), f.y());
    static double pan_hold = 0.0;

    // pan: angle de l’axe left dans le plan XY (stable en pitch/roll)
    double pan  = (rxy > 1e-3) ? -std::atan2(l.x(), l.y()) : pan_hold;
    // tilt: élévation du forward
    double tilt = std::atan2(f.z(), rxy);

    if (rxy > 1e-3) pan_hold = pan;

    // (option) deadband anti-bruit
    // if (std::abs(pan)  < 0.5*M_PI/180.0) pan  = 0;
    // if (std::abs(tilt) < 0.5*M_PI/180.0) tilt = 0;

    constexpr double K = 180.0/M_PI * 10.0; // 0,1°
    int16_t pan_cmd  = (int16_t)std::lround(pan  * K);
    int16_t tilt_cmd = (int16_t)std::lround(-tilt * K);

    std_msgs::msg::Int16MultiArray msg;
    msg.data = {pan_cmd, tilt_cmd};
    pub_pan_tilt->publish(msg);

    // Marker "regard"
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "base_link";
    m.header.stamp = this->get_clock()->now();
    m.ns = "hmd"; m.id = 1;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.02; m.scale.y = 0.04; m.scale.z = 0.0;
    m.color.a = 1.0; m.color.r = 1.0;
    geometry_msgs::msg::Point p0, p1; p0.x=p0.y=p0.z=0.0;
    double L = 0.5; p1.x=f.x()*L; p1.y=f.y()*L; p1.z=f.z()*L;
    m.points = {p0, p1};
    viz_pub_->publish(m);

    RCLCPP_INFO(this->get_logger(), "pan=%.1f°, tilt=%.1f°",
                pan*(180.0/M_PI), tilt*(180.0/M_PI));
    }


};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DK2Node>());
    rclcpp::shutdown();
    return 0;
}
