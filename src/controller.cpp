#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tutorial_interfaces/msg/ankles.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Controller : public rclcpp::Node
{
  public:
    Controller()
    : Node("controller")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(//KEEP AT 100ms THIS USED TO COMPUTE SPEED TEMPORARILY
        100ms, std::bind(&Controller::timer_callback, this));

        subscription_ = this->create_subscription<tutorial_interfaces::msg::Ankles>(    // CHANGE
        "ankles", 10, std::bind(&Controller::computeAnkleVel, this, _1));
    }

  private:
    void timer_callback()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = cmd_vel_x;
        // RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel X: %f", message.linear.x);
        publisher_->publish(message);
    }

    
    void computeAnkleVel(const tutorial_interfaces::msg::Ankles & msg)  // CHANGE
    {

        double right_speed = 0;

        if(msg.right_score>threshold)
        {
          double current_right_x =  msg.right.x;
          right_speed = - (current_right_x-prev_right_x)/0.1*tuning_constant;//negative because when leg moves away from body, x coordinate decreases. divide by 100ms
          if(right_speed<0) right_speed=0; //dont take reverse
          prev_right_x = current_right_x;
        }

        double left_speed = 0;

        if(msg.left_score>threshold)
        {
          double current_left_x =  msg.left.x;
          left_speed = - (current_left_x-prev_left_x)/0.1*tuning_constant;//negative because when leg moves away from body, x coordinate decreases. divide by 100ms
          if(left_speed<0) left_speed=0; //dont take reverse
          prev_left_x = current_left_x;
        }

        cmd_vel_x = std::max(left_speed,right_speed);

        RCLCPP_INFO(this->get_logger(), "Right Score: %f, Left Score: %f , CmdVel: %f", msg.right_score , msg.left_score, cmd_vel_x);     // CHANGE


    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<tutorial_interfaces::msg::Ankles>::SharedPtr subscription_;  // CHANGE
    double cmd_vel_x = 0;
    double prev_right_x = 0;
    double prev_left_x = 0;
    double tuning_constant =0.01;
    double threshold = 0.4;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}