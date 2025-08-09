#include <rclcpp/rclcpp.hpp>
#include <my_bridge_msgs/msg/gates_positions.hpp>  // پیام سفارشی شما
#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>

class GatesPositionPublisher : public rclcpp::Node {
public:
  GatesPositionPublisher()
  : Node("gates_position_publisher")
  {
    publisher_ = this->create_publisher<my_bridge_msgs::msg::GatesPositions>(
      "gates_positions", 10);

    gz_node_.Subscribe("/world/default/pose/info",
      &GatesPositionPublisher::gzCallback, this);

    gz_node_.Subscribe("/world/default/dynamic_pose/info",
      &GatesPositionPublisher::gzDynamicCallback, this);

    RCLCPP_INFO(this->get_logger(), "Subscribed to Gazebo topics");
  }

private:
  my_bridge_msgs::msg::GatesPositions gates_msg_;

  void gzCallback(const gz::msgs::Pose_V &msg)
  {
    for (int gate_num = 1; gate_num <= 10; ++gate_num) {
      bool found = false;
      for (int i = 0; i < msg.pose_size(); ++i) {
        const auto &pose = msg.pose(i);
        if (pose.name() == ("gate" + std::to_string(gate_num))) {
          my_bridge_msgs::msg::Pose2D gate_pose;
          gate_pose.x = pose.position().x();
          gate_pose.y = pose.position().y();

          if (gates_msg_.gates_and_robots.size() < 10)
            gates_msg_.gates_and_robots.resize(10);

          gates_msg_.gates_and_robots[gate_num - 1] = gate_pose;
          found = true;
          break;
        }
      }
      if (!found) {
        my_bridge_msgs::msg::Pose2D empty_pose;
        empty_pose.x = 0.0;
        empty_pose.y = 0.0;
        if (gates_msg_.gates_and_robots.size() < 10)
          gates_msg_.gates_and_robots.resize(10);
        gates_msg_.gates_and_robots[gate_num - 1] = empty_pose;
      }
    }
    publishIfReady();
  }

  void gzDynamicCallback(const gz::msgs::Pose_V &msg)
  {
    const std::vector<std::string> robots = {"burger","robot_2", "robot_3", "robot_4"};
    if (gates_msg_.gates_and_robots.size() < 14)
      gates_msg_.gates_and_robots.resize(14);

    for (size_t idx = 0; idx < robots.size(); ++idx) {
      bool found = false;
      for (int i = 0; i < msg.pose_size(); ++i) {
        const auto &pose = msg.pose(i);
        if (pose.name() == robots[idx]) {
          my_bridge_msgs::msg::Pose2D robot_pose;
          robot_pose.x = pose.position().x();
          robot_pose.y = pose.position().y();

          gates_msg_.gates_and_robots[10 + idx] = robot_pose;
          found = true;
          break;
        }
      }
      if (!found) {
        my_bridge_msgs::msg::Pose2D empty_pose;
        empty_pose.x = 0.0;
        empty_pose.y = 0.0;
        gates_msg_.gates_and_robots[10 + idx] = empty_pose;
      }
    }
    publishIfReady();
  }

  void publishIfReady()
  {
    if (gates_msg_.gates_and_robots.size() >= 14) {
      publisher_->publish(gates_msg_);
      RCLCPP_INFO(this->get_logger(), "Published gates_and_robots positions array.");
    }
  }

  gz::transport::Node gz_node_;
  rclcpp::Publisher<my_bridge_msgs::msg::GatesPositions>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GatesPositionPublisher>());
  rclcpp::shutdown();
  return 0;
}
