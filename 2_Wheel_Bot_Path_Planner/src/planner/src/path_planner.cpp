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
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr position_publisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr milestone_subscriber;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr target_subscriber;
    rclcpp::TimerBase::SharedPtr position_publisher_timer;
    rclcpp::TimerBase::SharedPtr velocity_publisher_timer;
    std::vector<int> milestones;
    std::vector<std::vector<double>> path;

    void milestone_subscriber_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            if (location < path.size() - 1)
            {
                location++;
            }
        }
    }

    void position_publisher_callback()
    {
        std_msgs::msg::Int8 position;
        if (location < milestones.size())
        {
            position.data = milestones[location];
        } else
        {
            position.data = 0;
        }
        position_publisher->publish(position);
    }

    void target_subscriber_callback(const std_msgs::msg::Int8::SharedPtr msg)
    {
        if (msg->data == 11)
        {

            location = 0;
            milestones.clear();
            path.clear();

            // 00 TO 13
            milestones.push_back(0);
            path.push_back({1.0, 0.0});
            milestones.push_back(1);
            path.push_back({1.0, 0.0});
            milestones.push_back(6);
            path.push_back({1.0, 0.0});
            milestones.push_back(11);
            path.push_back({0.0, -1.0});
            milestones.push_back(11);
            path.push_back({1.0, 0.0});
            milestones.push_back(12);
            path.push_back({0.0, 1.0});
            milestones.push_back(12);
            path.push_back({1.0, 0.0});
            milestones.push_back(13);

            // 13 TO 03
            path.push_back({0.0, 1.0});
            milestones.push_back(13);
            path.push_back({1.0, 0.0});
            milestones.push_back(12);
            path.push_back({0.0, -1.0});
            milestones.push_back(12);
            path.push_back({1.0, 0.0});
            milestones.push_back(11);
            path.push_back({0.0, 1.0});
            milestones.push_back(11);
            path.push_back({1.0, 0.0});
            milestones.push_back(6);
            path.push_back({1.0, 0.0});
            milestones.push_back(1);
            path.push_back({0.0, 1.0});
            milestones.push_back(1);
            path.push_back({1.0, 0.0});
            milestones.push_back(2);
            path.push_back({0.0, 1.0});
            milestones.push_back(2);
            path.push_back({1.0, 0.0});
            milestones.push_back(3);

            // 03 TO 00
            path.push_back({0.0, 1.0});
            milestones.push_back(3);
            path.push_back({1.0, 0.0});
            milestones.push_back(2);
            path.push_back({0.0, -1.0});
            milestones.push_back(2);
            path.push_back({1.0, 0.0});
            milestones.push_back(1);
            path.push_back({0.0, 1.0});
            milestones.push_back(1);
            path.push_back({1.0, 0.0});
            milestones.push_back(0);
            path.push_back({0.0, 1.0});
            milestones.push_back(0);
            path.push_back({0.0, 0.0});

        } else if (msg->data == 12)
        {

            location = 0;
            milestones.clear();
            path.clear();

            // 00 TO 03
            milestones.push_back(0);
            path.push_back({1.0, 0.0});
            milestones.push_back(1);
            path.push_back({0.0, -1.0});
            milestones.push_back(1);
            path.push_back({1.0, 0.0});
            milestones.push_back(2);
            path.push_back({0.0, 1.0});
            milestones.push_back(2);
            path.push_back({1.0, 0.0});
            milestones.push_back(3);

            // 03 TO 15
            path.push_back({0.0, 1.0});
            milestones.push_back(3);
            path.push_back({1.0, 0.0});
            milestones.push_back(2);
            path.push_back({0.0, -1.0});
            milestones.push_back(2);
            path.push_back({1.0, 0.0});
            milestones.push_back(1);
            path.push_back({0.0, -1.0});
            milestones.push_back(1);
            path.push_back({1.0, 0.0});
            milestones.push_back(6);
            path.push_back({1.0, 0.0});
            milestones.push_back(11);
            path.push_back({0.0, 1.0});
            milestones.push_back(11);
            path.push_back({1.0, 0.0});
            milestones.push_back(14);
            path.push_back({0.0, -1.0});
            milestones.push_back(14);
            path.push_back({1.0, 0.0});
            milestones.push_back(15);

            // 15 TO 00
            path.push_back({0.0, 1.0});
            milestones.push_back(15);
            path.push_back({1.0, 0.0});
            milestones.push_back(14);
            path.push_back({0.0, 1.0});
            milestones.push_back(14);
            path.push_back({1.0, 0.0});
            milestones.push_back(11);
            path.push_back({0.0, -1.0});
            milestones.push_back(11);
            path.push_back({1.0, 0.0});
            milestones.push_back(6);
            path.push_back({1.0, 0.0});
            milestones.push_back(1);
            path.push_back({1.0, 0.0});
            milestones.push_back(0);
            path.push_back({0.0, 1.0});
            milestones.push_back(0);
            path.push_back({0.0, 0.0});

        } else if (msg->data == 21)
        {

            location = 0;
            milestones.clear();
            path.clear();

            // 00 TO 13
            milestones.push_back(0);
            path.push_back({1.0, 0.0});
            milestones.push_back(1);
            path.push_back({1.0, 0.0});
            milestones.push_back(6);
            path.push_back({1.0, 0.0});
            milestones.push_back(11);
            path.push_back({0.0, -1.0});
            milestones.push_back(11);
            path.push_back({1.0, 0.0});
            milestones.push_back(12);
            path.push_back({0.0, 1.0});
            milestones.push_back(12);
            path.push_back({1.0, 0.0});
            milestones.push_back(13);

            // 13 TO 05
            path.push_back({0.0, 1.0});
            milestones.push_back(13);
            path.push_back({1.0, 0.0});
            milestones.push_back(12);
            path.push_back({0.0, -1.0});
            milestones.push_back(12);
            path.push_back({1.0, 0.0});
            milestones.push_back(11);
            path.push_back({0.0, 1.0});
            milestones.push_back(11);
            path.push_back({1.0, 0.0});
            milestones.push_back(6);
            path.push_back({1.0, 0.0});
            milestones.push_back(1);
            path.push_back({0.0, -1.0});
            milestones.push_back(1);
            path.push_back({1.0, 0.0});
            milestones.push_back(4);
            path.push_back({0.0, -1.0});
            milestones.push_back(4);
            path.push_back({1.0, 0.0});
            milestones.push_back(5);

            // 05 TO 00
            path.push_back({0.0, 1.0});
            milestones.push_back(5);
            path.push_back({1.0, 0.0});
            milestones.push_back(4);
            path.push_back({0.0, 1.0});
            milestones.push_back(4);
            path.push_back({1.0, 0.0});
            milestones.push_back(1);
            path.push_back({0.0, -1.0});
            milestones.push_back(1);
            path.push_back({1.0, 0.0});
            milestones.push_back(0);
            path.push_back({0.0, 1.0});
            milestones.push_back(0);
            path.push_back({0.0, 0.0});

        } else if (msg->data == 22)
        {

            location = 0;
            milestones.clear();
            path.clear();

            // 00 TO 05
            milestones.push_back(0);
            path.push_back({1.0, 0.0});
            milestones.push_back(1);
            path.push_back({0.0, 1.0});
            milestones.push_back(1);
            path.push_back({1.0, 0.0});
            milestones.push_back(4);
            path.push_back({0.0, -1.0});
            milestones.push_back(4);
            path.push_back({1.0, 0.0});
            milestones.push_back(5);

            // 05 TO 15
            path.push_back({0.0, 1.0});
            milestones.push_back(5);
            path.push_back({1.0, 0.0});
            milestones.push_back(4);
            path.push_back({0.0, 1.0});
            milestones.push_back(4);
            path.push_back({1.0, 0.0});
            milestones.push_back(1);
            path.push_back({0.0, 1.0});
            milestones.push_back(1);
            path.push_back({1.0, 0.0});
            milestones.push_back(6);
            path.push_back({1.0, 0.0});
            milestones.push_back(11);
            path.push_back({0.0, 1.0});
            milestones.push_back(11);
            path.push_back({1.0, 0.0});
            milestones.push_back(14);
            path.push_back({0.0, -1.0});
            milestones.push_back(14);
            path.push_back({1.0, 0.0});
            milestones.push_back(15);

            // 15 TO 00
            path.push_back({0.0, 1.0});
            milestones.push_back(15);
            path.push_back({1.0, 0.0});
            milestones.push_back(14);
            path.push_back({0.0, 1.0});
            milestones.push_back(14);
            path.push_back({1.0, 0.0});
            milestones.push_back(11);
            path.push_back({0.0, -1.0});
            milestones.push_back(11);
            path.push_back({1.0, 0.0});
            milestones.push_back(6);
            path.push_back({1.0, 0.0});
            milestones.push_back(1);
            path.push_back({1.0, 0.0});
            milestones.push_back(0);
            path.push_back({0.0, 1.0});
            milestones.push_back(0);
            path.push_back({0.0, 0.0});

        } else if (msg->data == 31)
        {

            location = 0;
            milestones.clear();
            path.clear();

            // 00 TO 13
            milestones.push_back(0);
            path.push_back({1.0, 0.0});
            milestones.push_back(1);
            path.push_back({1.0, 0.0});
            milestones.push_back(6);
            path.push_back({1.0, 0.0});
            milestones.push_back(11);
            path.push_back({0.0, -1.0});
            milestones.push_back(11);
            path.push_back({1.0, 0.0});
            milestones.push_back(12);
            path.push_back({0.0, 1.0});
            milestones.push_back(12);
            path.push_back({1.0, 0.0});
            milestones.push_back(13);

            // 13 TO 08
            path.push_back({0.0, 1.0});
            milestones.push_back(13);
            path.push_back({1.0, 0.0});
            milestones.push_back(12);
            path.push_back({0.0, -1.0});
            milestones.push_back(12);
            path.push_back({1.0, 0.0});
            milestones.push_back(11);
            path.push_back({0.0, 1.0});
            milestones.push_back(11);
            path.push_back({1.0, 0.0});
            milestones.push_back(6);
            path.push_back({0.0, 1.0});
            milestones.push_back(6);
            path.push_back({1.0, 0.0});
            milestones.push_back(7);
            path.push_back({0.0, 1.0});
            milestones.push_back(7);
            path.push_back({1.0, 0.0});
            milestones.push_back(8);

            // 08 TO 00
            path.push_back({0.0, 1.0});
            milestones.push_back(8);
            path.push_back({1.0, 0.0});
            milestones.push_back(7);
            path.push_back({0.0, -1.0});
            milestones.push_back(7);
            path.push_back({1.0, 0.0});
            milestones.push_back(6);
            path.push_back({0.0, 1.0});
            milestones.push_back(6);
            path.push_back({1.0, 0.0});
            milestones.push_back(1);
            path.push_back({1.0, 0.0});
            milestones.push_back(0);
            path.push_back({0.0, 1.0});
            milestones.push_back(0);
            path.push_back({0.0, 0.0});

        } else if (msg->data == 32)
        {

            location = 0;
            milestones.clear();
            path.clear();

            // 00 TO 08
            milestones.push_back(0);
            path.push_back({1.0, 0.0});
            milestones.push_back(1);
            path.push_back({1.0, 0.0});
            milestones.push_back(6);
            path.push_back({0.0, -1.0});
            milestones.push_back(6);
            path.push_back({1.0, 0.0});
            milestones.push_back(7);
            path.push_back({0.0, 1.0});
            milestones.push_back(7);
            path.push_back({1.0, 0.0});
            milestones.push_back(8);

            // 08 TO 15
            path.push_back({0.0, 1.0});
            milestones.push_back(8);
            path.push_back({1.0, 0.0});
            milestones.push_back(7);
            path.push_back({0.0, -1.0});
            milestones.push_back(7);
            path.push_back({1.0, 0.0});
            milestones.push_back(6);
            path.push_back({0.0, -1.0});
            milestones.push_back(6);
            path.push_back({1.0, 0.0});
            milestones.push_back(11);
            path.push_back({0.0, 1.0});
            milestones.push_back(11);
            path.push_back({1.0, 0.0});
            milestones.push_back(14);
            path.push_back({0.0, -1.0});
            milestones.push_back(14);
            path.push_back({1.0, 0.0});
            milestones.push_back(15);

            // 15 TO 00
            path.push_back({0.0, 1.0});
            milestones.push_back(15);
            path.push_back({1.0, 0.0});
            milestones.push_back(14);
            path.push_back({0.0, 1.0});
            milestones.push_back(14);
            path.push_back({1.0, 0.0});
            milestones.push_back(11);
            path.push_back({0.0, -1.0});
            milestones.push_back(11);
            path.push_back({1.0, 0.0});
            milestones.push_back(6);
            path.push_back({1.0, 0.0});
            milestones.push_back(1);
            path.push_back({1.0, 0.0});
            milestones.push_back(0);
            path.push_back({0.0, 1.0});
            milestones.push_back(0);
            path.push_back({0.0, 0.0});

        } else if (msg->data == 41)
        {

            location = 0;
            milestones.clear();
            path.clear();

            // 00 TO 13
            milestones.push_back(0);
            path.push_back({1.0, 0.0});
            milestones.push_back(1);
            path.push_back({1.0, 0.0});
            milestones.push_back(6);
            path.push_back({1.0, 0.0});
            milestones.push_back(11);
            path.push_back({0.0, -1.0});
            milestones.push_back(11);
            path.push_back({1.0, 0.0});
            milestones.push_back(12);
            path.push_back({0.0, 1.0});
            milestones.push_back(12);
            path.push_back({1.0, 0.0});
            milestones.push_back(13);

            // 13 TO 10
            path.push_back({0.0, 1.0});
            milestones.push_back(13);
            path.push_back({1.0, 0.0});
            milestones.push_back(12);
            path.push_back({0.0, -1.0});
            milestones.push_back(12);
            path.push_back({1.0, 0.0});
            milestones.push_back(11);
            path.push_back({0.0, 1.0});
            milestones.push_back(11);
            path.push_back({1.0, 0.0});
            milestones.push_back(6);
            path.push_back({0.0, -1.0});
            milestones.push_back(6);
            path.push_back({1.0, 0.0});
            milestones.push_back(9);
            path.push_back({0.0, -1.0});
            milestones.push_back(9);
            path.push_back({1.0, 0.0});
            milestones.push_back(10);

            // 10 TO 00
            path.push_back({0.0, 1.0});
            milestones.push_back(10);
            path.push_back({1.0, 0.0});
            milestones.push_back(9);
            path.push_back({0.0, 1.0});
            milestones.push_back(9);
            path.push_back({1.0, 0.0});
            milestones.push_back(6);
            path.push_back({0.0, -1.0});
            milestones.push_back(6);
            path.push_back({1.0, 0.0});
            milestones.push_back(1);
            path.push_back({1.0, 0.0});
            milestones.push_back(0);
            path.push_back({0.0, 1.0});
            milestones.push_back(0);
            path.push_back({0.0, 0.0});

        } else if (msg->data == 42)
        {

            location = 0;
            milestones.clear();
            path.clear();

            // 00 TO 10
            milestones.push_back(0);
            path.push_back({1.0, 0.0});
            milestones.push_back(1);
            path.push_back({1.0, 0.0});
            milestones.push_back(6);
            path.push_back({0.0, 1.0});
            milestones.push_back(6);
            path.push_back({1.0, 0.0});
            milestones.push_back(9);
            path.push_back({0.0, -1.0});
            milestones.push_back(9);
            path.push_back({1.0, 0.0});
            milestones.push_back(10);

            // 10 TO 15
            path.push_back({0.0, 1.0});
            milestones.push_back(10);
            path.push_back({1.0, 0.0});
            milestones.push_back(9);
            path.push_back({0.0, 1.0});
            milestones.push_back(9);
            path.push_back({1.0, 0.0});
            milestones.push_back(6);
            path.push_back({0.0, 1.0});
            milestones.push_back(6);
            path.push_back({1.0, 0.0});
            milestones.push_back(11);
            path.push_back({0.0, 1.0});
            milestones.push_back(11);
            path.push_back({1.0, 0.0});
            milestones.push_back(14);
            path.push_back({0.0, -1.0});
            milestones.push_back(14);
            path.push_back({1.0, 0.0});
            milestones.push_back(15);

            // 15 TO 00
            path.push_back({0.0, 1.0});
            milestones.push_back(15);
            path.push_back({1.0, 0.0});
            milestones.push_back(14);
            path.push_back({0.0, 1.0});
            milestones.push_back(14);
            path.push_back({1.0, 0.0});
            milestones.push_back(11);
            path.push_back({0.0, -1.0});
            milestones.push_back(11);
            path.push_back({1.0, 0.0});
            milestones.push_back(6);
            path.push_back({1.0, 0.0});
            milestones.push_back(1);
            path.push_back({1.0, 0.0});
            milestones.push_back(0);
            path.push_back({0.0, 1.0});
            milestones.push_back(0);
            path.push_back({0.0, 0.0});

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
        location = 0;
        milestone_subscriber = this->create_subscription<std_msgs::msg::Bool>("/milestone", 10, std::bind(&PathPlanner::milestone_subscriber_callback, this, std::placeholders::_1));
        position_publisher_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PathPlanner::position_publisher_callback, this));
        position_publisher = this->create_publisher<std_msgs::msg::Int8>("/bot_pos", 10);
        target_subscriber = this->create_subscription<std_msgs::msg::Int8>("/target", 10, std::bind(&PathPlanner::target_subscriber_callback, this, std::placeholders::_1));
        velocity_msg.linear.y = 0.0;
        velocity_msg.linear.z = 0.0;
        velocity_msg.angular.x = 0.0;
        velocity_msg.angular.y = 0.0;
        velocity_publisher_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PathPlanner::velocity_publisher_callback, this));
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