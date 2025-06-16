#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "sanctuary-pkg/three_dof_arm.hpp"

using std::placeholders::_1;

class IKSubscriber : public rclcpp::Node
{
  public:
    IKSubscriber()
    : Node("ik_subscriber")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
      "pose_topic", 10, std::bind(&IKSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const geometry_msgs::msg::Pose2D & pose)
    {
        // call IK
        //TODO: change IK function signature to take Pose2D
        std::vector<std::vector<double>> result = robot.solve_ik({pose.x,pose.y,pose.theta});
        RCLCPP_INFO(this->get_logger(), "Solution 1: theta1 = %.2f, theta2 = %.2f, theta3 = %.2f", result[0][0], result[0][1], result[0][2]);    
        RCLCPP_INFO(this->get_logger(), "Solution 2: theta1 = %.2f, theta2 = %.2f, theta3 = %.2f", result[1][0], result[1][1], result[1][2]);    
    }
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subscription_;

    //instantiat a 3DOF arm object
    ThreeDofArm robot = ThreeDofArm(0.3,0.3,.1);
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IKSubscriber>());
  rclcpp::shutdown();
  return 0;
}