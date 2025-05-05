#include <behaviortree_cpp/basic_types.h>           // BT::Blackboard
#include "behaviortree_cpp/bt_factory.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "std_msgs/msg/float64.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>

#include "checkRiskLevel.cpp"
#include "checkRiskLevelWarning.cpp"
#include "warningMode.cpp"

using namespace std::chrono_literals;
using namespace BT;
 
 
// class CheckRiskLevel : public BT::ConditionNode
// {
//     public:
//         CheckRiskLevel(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config)
//         {
//             node_ = rclcpp::Node::make_shared("bt_check_risk_level_node");

//         }

//     NodeStatus tick() override
//     {
//         std::cout<<"RiskCheck : " << " " << std::endl;
//         return BT::NodeStatus::FAILURE;
//     }
// };

class RobotTask1 : public BT::SyncActionNode
{
  public:
    RobotTask1(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }
 
    // You must override the virtual function tick()
    NodeStatus tick() override
    {
        std::cout << "RobotTask1: " << this->name() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
};
 
class RobotTask2 : public BT::SyncActionNode
{
  public:
    RobotTask2(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }
 
    // You must override the virtual function tick()
    NodeStatus tick() override
    {
        std::cout << "RobotTask2: " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};
 
class RobotTask3 : public BT::SyncActionNode
{
  public:
    RobotTask3(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }
 
    // You must override the virtual function tick()
    NodeStatus tick() override
    {
        std::cout << "RobotTask3: " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};
 
std::string tree_path =
    ament_index_cpp::get_package_share_directory("carla_behavior") +
    "/behavior_trees/monitor_tree.xml";

 
int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto shared_node = std::make_shared<rclcpp::Node>("bt_shared_node");
    // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;

    // factory.registerBuilder<WarningMode>(
    //     "slow_driving",
    //     [shared_node](const std::string& name, const BT::NodeConfiguration& cfg){
    //         return std::make_unique<WarningMode>(name,cfg,shared_node);
    //     }
    // );

    auto blackboard = BT::Blackboard::create();
    blackboard->set<rclcpp::Node::SharedPtr>("node", shared_node);
    
    factory.registerNodeType<CheckRiskLevel>("CheckRisk");
    factory.registerNodeType<CheckRiskLevel2>("CheckRiskWarning");
    factory.registerNodeType<WarningMode>("slow_driving");

    factory.registerNodeType<RobotTask1>("RobotTask1");
    factory.registerNodeType<RobotTask2>("RobotTask2");
    factory.registerNodeType<RobotTask3>("RobotTask3");
    
    // Trees are created at deployment-time (i.e. at run-time, but only once at the beginning).
    // The currently supported format is XML.
    // IMPORTANT: when the object "tree" goes out of scope, all the TreeNodes are destroyed

    auto tree = factory.createTreeFromFile(tree_path, blackboard);

    BT::Groot2Publisher groot_publisher(tree, 1666);//groot 퍼블리셔

    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.

    
    rclcpp::Rate loop_rate(10);
    while(rclcpp::ok()) {
  
      rclcpp::spin_some(shared_node);
      tree.tickWhileRunning(10ms);
      loop_rate.sleep();
    }
    
  
    rclcpp::shutdown();


    
    return 0;
}