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

        subscriptionX_ = this->create_subscription<tutorial_interfaces::msg::Ankles>(    // CHANGE
        "ankles", 10, std::bind(&Controller::computeAnkleVelX, this, _1));

        subscriptionZ_ = this->create_subscription<tutorial_interfaces::msg::Ankles>(    // CHANGE
        "ankles_front", 10, std::bind(&Controller::computeAnkleVelZ, this, _1));
    }

  private:
    void timer_callback()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = cmd_vel_x;
        message.angular.z = cmd_vel_z;
        RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel X,Z: %f, %f", cmd_vel_x, cmd_vel_z);
        publisher_->publish(message);
    }

    
    void computeAnkleVelX(const tutorial_interfaces::msg::Ankles & msg)  // CHANGE
    {

        double right_speed = 0;

        if(msg.right_score>score_threshold)
        {
          double current_right_x =  msg.right.x;
          right_speed = (current_right_x-prev_right_x)/0.1*tuning_constant;//postive because when leg moves away from body, x coordinate increases. divide by 100ms
          if(right_speed<0) right_speed=0; //dont take reverse
          prev_right_x = current_right_x;
        }

        double left_speed = 0;

        if(msg.left_score>score_threshold)
        {
          double current_left_x =  msg.left.x;
          left_speed = (current_left_x-prev_left_x)/0.1*tuning_constant;//postive because when leg moves away from body, x coordinate increases. divide by 100ms
          if(left_speed<0) left_speed=0; //dont take reverse
          prev_left_x = current_left_x;
        }

        cmd_vel_x = std::max(left_speed,right_speed);

        // RCLCPP_INFO(this->get_logger(), "Right Score: %f, Left Score: %f , CmdVel: %f", msg.right_score , msg.left_score, cmd_vel_x);     // CHANGE


    }

    void computeAnkleVelZ(const tutorial_interfaces::msg::Ankles & msg)  // CHANGE
    {

      if(msg.right_score>score_threshold && msg.left_score>score_threshold)
      {
        double current_right_x =  msg.right.x;
        double right_speed = -(current_right_x-prev_right_x_turn)/0.1*tuning_constant_z;//negative because when leg moves to right of body, x coordinate decreases. divide by 100ms
        if(right_speed<0) right_speed=0; //dont take reverse

        double current_left_x =  msg.left.x;
        double left_speed = (current_left_x-prev_left_x_turn)/0.1*tuning_constant_z;//postive because when leg moves to left of body, x coordinate increases. divide by 100ms

        if(left_speed<0) left_speed=0; //dont take reverse

        if(abs(current_right_x-current_left_x)<=neutral_threshold)
        {
          currentState=neutral;//ready for next side step
        }
        else if(currentState==neutral && right_speed>left_speed)//was previously neutral, now taking next step
        {
          currentState=right;
        }
        else if(currentState==neutral && left_speed>right_speed)//was previously neutral, now taking next step
        {
          currentState=left;
        }


        if(currentState==neutral)
        {
          cmd_vel_z=0;
        }
        else if(currentState==right)
        {
          cmd_vel_z = - right_speed;
        }
        else if(currentState==left)
        {
          cmd_vel_z = left_speed;
        }

        
        prev_right_x_turn = current_right_x;
        prev_left_x_turn = current_left_x;
      }
      else
      {
        cmd_vel_z=0;
      }


        // RCLCPP_INFO(this->get_logger(), "Right Score: %f, Left Score: %f , CmdVel: %f", msg.right_score , msg.left_score, cmd_vel_z);     // CHANGE


    }    

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<tutorial_interfaces::msg::Ankles>::SharedPtr subscriptionX_;  // CHANGE
    rclcpp::Subscription<tutorial_interfaces::msg::Ankles>::SharedPtr subscriptionZ_;  // CHANGE
    double cmd_vel_x = 0;
    double prev_right_x = 0;
    double prev_left_x = 0;
    double tuning_constant =0.001;
    double score_threshold = 0.4;

    double cmd_vel_z = 0;
    double prev_right_x_turn = 0;
    double prev_left_x_turn = 0;
    double tuning_constant_z =0.01;
    double neutral_threshold = 100;
    enum state {neutral,right,left};
    state currentState = neutral;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}