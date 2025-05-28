#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

using std::placeholders::_1;

class OffboardControl : public rclcpp::Node {
public:
    OffboardControl() : Node("offb_node") {
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, std::bind(&OffboardControl::state_cb, this, _1));

        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "mavros/setpoint_position/local", 10);

        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
            "mavros/cmd/arming");

        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
            "mavros/set_mode");

        pose_.pose.position.x = 0.0;
        pose_.pose.position.y = 0.0;
        pose_.pose.position.z = 2.0;

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&OffboardControl::control_loop, this));
    }

private:
    void state_cb(const mavros_msgs::msg::State::SharedPtr msg) {
        current_state_ = *msg;
    }

    void control_loop() { 
        if (!current_state_.connected) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Waiting for FCU connection...");
            return;
        }

        // Send a few setpoints before starting
        if (!initialized_) {
            for (int i = 0; i < 100 && rclcpp::ok(); ++i) {
                local_pos_pub_->publish(pose_);
                rclcpp::spin_some(shared_from_this());
                rclcpp::sleep_for(std::chrono::milliseconds(50));
            }
            last_request_ = this->now();
            initialized_ = true;
        }

        if (current_state_.mode != "OFFBOARD" &&
            (this->now() - last_request_).seconds() > 5.0) {
            auto mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            mode_req->custom_mode = "OFFBOARD";

            if (!set_mode_client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_WARN(this->get_logger(), "SetMode service not available");
                return;
            }

            auto result = set_mode_client_->async_send_request(mode_req);
            rclcpp::spin_until_future_complete(this->get_node_base_interface(), result);

            if (result.get()->mode_sent) {
                RCLCPP_INFO(this->get_logger(), "Offboard mode enabled");
            }
            last_request_ = this->now();
        } else if (!current_state_.armed &&
                   (this->now() - last_request_).seconds() > 5.0) {
            auto arm_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            arm_req->value = true;

            if (!arming_client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_WARN(this->get_logger(), "Arming service not available");
                return;
            }

            auto result = arming_client_->async_send_request(arm_req);
            rclcpp::spin_until_future_complete(this->get_node_base_interface(), result);

            if (result.get()->success) {
                RCLCPP_INFO(this->get_logger(), "Vehicle armed");
            }
            last_request_ = this->now();
        }

        local_pos_pub_->publish(pose_);
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped pose_;
    rclcpp::Time last_request_;
    bool initialized_ = false;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}