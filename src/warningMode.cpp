#include <behaviortree_cpp/action_node.h>      // v4: Sync/Async 기본 헤더
#include "std_msgs/msg/int32.hpp"
#include <rclcpp/rclcpp.hpp>
#include <limits>

using namespace BT;

class WarningMode : public SyncActionNode
{
    public:

  WarningMode(const std::string& name, const NodeConfig& cfg)
      : SyncActionNode(name, cfg)
  {
      /* 블랙보드에서 ROS2 노드 가져오기 */
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

      pub_ = node_->create_publisher<std_msgs::msg::Int32>(
                 "/low_throttle_cmd", 1);

      sub_ = node_->create_subscription<std_msgs::msg::Int32>(
                 "/low_throttle_feedback", 1,
                 [this](const std_msgs::msg::Int32::SharedPtr msg)
                 {
                     feedback_ = msg->data;
                     received_ = true;
                 });
  }

  /* 포트가 없으면 빈 map 리턴 */
  static PortsList providedPorts() { return {}; }

  NodeStatus tick() override
  {
      /* 최초 1회 CMD 전송 */
      if (!sent_)
      {
          std_msgs::msg::Int32 m;  m.data = 1;
          pub_->publish(m);
          sent_ = true;
      }

      /* 콜백 처리 */
      rclcpp::spin_some(node_);

      if (!received_)                              return NodeStatus::RUNNING;
      return (feedback_ == 2) ? NodeStatus::SUCCESS
                              : NodeStatus::FAILURE;
  }

//   void halt() override
//   {
//       sent_ = false;
//       received_ = false;
//       feedback_ = 0;
//   }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr      pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr   sub_;

  int  feedback_{0};
  bool sent_{false};
  bool received_{false};
};
