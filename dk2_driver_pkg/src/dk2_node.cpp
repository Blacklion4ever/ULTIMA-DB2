#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <openhmd.h>
#include <cstring>

class DK2Node : public rclcpp::Node {

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
    ohmd_context* ctx = nullptr;
    ohmd_device* device = nullptr;

public:
    DK2Node() : Node("dk2_node") {

        ohmd_require_version(0, 3, 0);

        int major, minor, patch;
        ohmd_get_version(&major, &minor, &patch);
        RCLCPP_INFO(this->get_logger(), "OpenHMD version: %d.%d.%d", major, minor, patch);

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

        // Cherche le DK2
        int dk2_index = -1;
        for (int i = 0; i < num_devices; i++) {
            const char* vendor = ohmd_list_gets(ctx, i, OHMD_VENDOR);
            const char* product = ohmd_list_gets(ctx, i, OHMD_PRODUCT);
            RCLCPP_INFO(this->get_logger(), "Device %d: %s %s", i, vendor, product);

            if (strstr(vendor, "Oculus")!= NULL) {
            	RCLCPP_INFO(this->get_logger(), "DK2 device found");
                dk2_index = i;
                break;
            }
        }

        if (dk2_index == -1) {
            RCLCPP_ERROR(this->get_logger(), "No DK2 device found");
            return;
        }

	// Open specified device idx or 0 (default) if nothing specified
	printf("opening device: %d\n", dk2_index);
        device = ohmd_list_open_device(ctx, dk2_index);
        if (!device) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open DK2 device");
            return;
        }
        
        publisher = this->create_publisher<sensor_msgs::msg::Imu>("dk2/imu", 10);

        timer = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&DK2Node::publish_imu, this)
        );
    }

    ~DK2Node() {
        if (ctx) ohmd_ctx_destroy(ctx);
    }

private:
    void publish_imu() {
        ohmd_ctx_update(ctx);

        float rotation[4]; // x, y, z, w
        int ret = ohmd_device_getf(device, OHMD_ROTATION_QUAT, rotation);
        if (ret != 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to read rotation from DK2");
            return;
        }

        // Affiche pour debug
        printf("Rotation quaternion: x=%f y=%f z=%f w=%f\n",
               rotation[0], rotation[1], rotation[2], rotation[3]);

        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->now();
        msg.header.frame_id = "dk2_link";
        msg.orientation.x = rotation[0];
        msg.orientation.y = rotation[1];
        msg.orientation.z = rotation[2];
        msg.orientation.w = rotation[3];

        publisher->publish(msg);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DK2Node>());
    rclcpp::shutdown();
    return 0;
}

