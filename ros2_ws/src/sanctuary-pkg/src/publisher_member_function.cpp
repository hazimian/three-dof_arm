#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "sanctuary-pkg/three_dof_arm.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
class FKPublisher : public rclcpp::Node
{
  public:
    FKPublisher(): Node("fk_publisher"), count_(0)
    {
        // Declare parameters with default values
        this->declare_parameter<double>("theta1", 0.0);
        this->declare_parameter<double>("theta2", 0.0);
        this->declare_parameter<double>("theta3", 0.0);

        // Get parameter values
        this->get_parameter("theta1", theta1);
        this->get_parameter("theta2", theta2);
        this->get_parameter("theta3", theta3);



        publisher_ = this->create_publisher<geometry_msgs::msg::Pose2D>("pose_topic", 10);
        timer_ = this->create_wall_timer(
        500ms, std::bind(&FKPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        // call FK
        //TODO: change FK function signature to return Pose2D
        std::vector<double> result = arm.solve_fk({theta1,theta2,theta3});
        auto pose = geometry_msgs::msg::Pose2D();
        pose.x = result[0];
        pose.y = result[1];
        pose.theta = result[2];
        publisher_->publish(pose);
        RCLCPP_INFO(this->get_logger(), "Sent pose: x = %.2f, y = %.2f, phi = %.2f", pose.x, pose.y, pose.theta);    



    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr publisher_;
    size_t count_;
    double theta1, theta2,theta3;
    //instantiat a 3DOF arm object
    ThreeDofArm arm = ThreeDofArm(0.3,0.3,.1);
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FKPublisher>());
  rclcpp::shutdown();
  return 0;
}