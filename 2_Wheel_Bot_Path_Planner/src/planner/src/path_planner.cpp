/*
    CS22B1090
    Shubh Khandelwal
*/

/*

    Map Visualization:

    |___ ___|
        |
    |___|___|
        |
    |___|___|
        |

    13      15
    12      14
        11
    08      10
    07      09
        06
    03      05
    02      04
        01
        00

*/

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"
#include <vector>

class PathPlanner : public rclcpp::Node
{

    private:

    geometry_msgs::msg::Twist velocity_msg;
    u_int location;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr milestone_subscriber;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr target_subscriber;
    rclcpp::TimerBase::SharedPtr timer;
    std::vector<std::vector<double>> path;

    void milestone_subscriber_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            location++;
        }
    }

    void target_subscriber_callback(const std_msgs::msg::Int8::SharedPtr msg)
    {
        if (msg->data == 11)
        {

            location = 0;
            path.clear();

            // 00 TO 13
            path.push_back({1.0, 0.0});
            path.push_back({1.0, 0.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});

            // 13 TO 03
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});

            // 03 TO 00
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});

        } else if (msg->data == 12)
        {

            location = 0;
            path.clear();

            // 00 TO 03
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});

            // 03 TO 15
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});

            // 15 TO 00
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});
            path.push_back({1.0, 0.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});

        } else if (msg->data == 21)
        {

            location = 0;
            path.clear();

            // 00 TO 13
            path.push_back({1.0, 0.0});
            path.push_back({1.0, 0.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});

            // 13 TO 05
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});

            // 05 TO 00
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});

        } else if (msg->data == 22)
        {

            location = 0;
            path.clear();

            // 00 TO 05
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});

            // 05 TO 15
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});

            // 15 TO 00
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});
            path.push_back({1.0, 0.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});

        } else if (msg->data == 31)
        {

            location = 0;
            path.clear();

            // 00 TO 13
            path.push_back({1.0, 0.0});
            path.push_back({1.0, 0.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});

            // 13 TO 08
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});

            // 08 TO 00
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});

        } else if (msg->data == 32)
        {

            location = 0;
            path.clear();

            // 00 TO 08
            path.push_back({1.0, 0.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});

            // 08 TO 15
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});

            // 15 TO 00
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});
            path.push_back({1.0, 0.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});

        } else if (msg->data == 41)
        {

            location = 0;
            path.clear();

            // 00 TO 13
            path.push_back({1.0, 0.0});
            path.push_back({1.0, 0.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});

            // 13 TO 10
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});

            // 10 TO 00
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});

        } else if (msg->data == 42)
        {

            location = 0;
            path.clear();

            // 00 TO 10
            path.push_back({1.0, 0.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});

            // 10 TO 15
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});

            // 15 TO 00
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, -1.0});
            path.push_back({1.0, 0.0});
            path.push_back({1.0, 0.0});
            path.push_back({1.0, 0.0});
            path.push_back({0.0, 1.0});

        }
    }

    void velocity_publisher_callback()
    {
        if (location < path.size())
        {
            velocity_msg.linear.x = path[location][0];
            velocity_msg.angular.z = path[location][1];
        } else
        {
            velocity_msg.linear.x = 0.0;
            velocity_msg.angular.z = 0.0;
        }
        velocity_publisher->publish(velocity_msg);
    }

    public:

    PathPlanner() : Node("path_planner")
    {
        milestone_subscriber = this->create_subscription<std_msgs::msg::Bool>("/milestone", 10, std::bind(&PathPlanner::milestone_subscriber_callback, this, std::placeholders::_1));
        location = 0;
        target_subscriber = this->create_subscription<std_msgs::msg::Int8>("/target", 10, std::bind(&PathPlanner::target_subscriber_callback, this, std::placeholders::_1));
        timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PathPlanner::velocity_publisher_callback, this));
        velocity_msg.linear.y = 0.0;
        velocity_msg.linear.z = 0.0;
        velocity_msg.angular.x = 0.0;
        velocity_msg.angular.y = 0.0;
        velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanner>());
    rclcpp::shutdown();
    return 0;
}