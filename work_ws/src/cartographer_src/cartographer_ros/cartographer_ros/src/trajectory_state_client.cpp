#include "rclcpp/rclcpp.hpp"
#include "cartographer_ros_msgs/srv/get_trajectory_states.hpp"
#include <chrono>
#include <sstream>

using namespace std::chrono;

class TrajectoryStateClient : public rclcpp::Node {
public:
    TrajectoryStateClient() : Node("trajectory_state_client") {
        // Create the client for the GetTrajectoryStates service
        client_ = this->create_client<cartographer_ros_msgs::srv::GetTrajectoryStates>("/get_trajectory_states");

        // Wait for the service to be available
        while (!client_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
    }

    void send_request() {
        // Create a request
        auto request = std::make_shared<cartographer_ros_msgs::srv::GetTrajectoryStates::Request>();

        // Record the start time
        auto start_time = high_resolution_clock::now();

        // Send the request asynchronously
        auto future = client_->async_send_request(request);

        // Wait for the response
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS) {

            // Calculate and log the response time
            auto end_time = high_resolution_clock::now();
            auto duration = duration_cast<milliseconds>(end_time - start_time);
            RCLCPP_INFO(this->get_logger(), "Response received in %ld ms", duration.count());

            // Log the response
            auto response = future.get();

            const auto &trajectory_ids = response->trajectory_states.trajectory_id;
            const auto &trajectory_states = response->trajectory_states.trajectory_state;

            if (trajectory_ids.size() != trajectory_states.size()) {
                RCLCPP_ERROR(this->get_logger(), "Mismatch between trajectory IDs and states");
                return;
            }

            std::ostringstream oss;
            for (size_t i = 0; i < trajectory_ids.size(); ++i) {
                oss << "Trajectory ID: " << trajectory_ids[i] << ", State: ";
                switch (trajectory_states[i]) {
                    case cartographer_ros_msgs::msg::TrajectoryStates::ACTIVE:
                        oss << "ACTIVE";
                        break;
                    case cartographer_ros_msgs::msg::TrajectoryStates::FINISHED:
                        oss << "FINISHED";
                        break;
                    case cartographer_ros_msgs::msg::TrajectoryStates::FROZEN:
                        oss << "FROZEN";
                        break;
                    case cartographer_ros_msgs::msg::TrajectoryStates::DELETED:
                        oss << "DELETED";
                        break;
                    default:
                        oss << "UNKNOWN";
                }
                oss << "\n";
            }

            RCLCPP_INFO(this->get_logger(), "Trajectory States:\n%s", oss.str().c_str());

        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

private:
    rclcpp::Client<cartographer_ros_msgs::srv::GetTrajectoryStates>::SharedPtr client_;
};

int main(int argc, char **argv) {
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create the TrajectoryStateClient node
    auto node = std::make_shared<TrajectoryStateClient>();

    // Send the service request
    node->send_request();

    // Shut down the ROS 2 system
    rclcpp::shutdown();

    return 0;
}
