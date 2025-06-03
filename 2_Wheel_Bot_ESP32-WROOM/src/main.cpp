/*
  CS22B1090
  Shubh Khandelwal
*/

#include <geometry_msgs/msg/twist.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
#include <WiFi.h>

#define WIFI_SSID "B5"
#define WIFI_PASSWORD "pendrive"
#define AGENT_PORT 8888

#define DIR_L GPIO_NUM_27
#define PWM_L GPIO_NUM_33
#define DIR_R GPIO_NUM_25
#define PWM_R GPIO_NUM_32
#define SPEED 51

#define IR_L GPIO_NUM_34
#define IR_M GPIO_NUM_26
#define IR_R GPIO_NUM_35

IPAddress AGENT_IP(172, 16, 205, 249);

rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_ret_t ret;

rclc_executor_t executor_publisher;
rcl_publisher_t publisher;
std_msgs__msg__Bool msg_publisher;
rcl_timer_t timer;

rclc_executor_t executor_subscriber;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg_subscriber;

bool rotation = false;

void move(char c)
{
    switch(c)
    {
        case 'F':
            rotation = false;
            digitalWrite(DIR_L, HIGH);
            digitalWrite(PWM_L, HIGH);
            digitalWrite(DIR_R, LOW);
            digitalWrite(PWM_R, HIGH);
            break;
        case 'B':
            rotation = false;
            digitalWrite(DIR_L, LOW);
            digitalWrite(PWM_L, HIGH);
            digitalWrite(DIR_R, HIGH);
            digitalWrite(PWM_R, HIGH);
            break;
        case 'L':
            rotation = true;
            digitalWrite(DIR_L, LOW);
            digitalWrite(PWM_L, HIGH);
            digitalWrite(DIR_R, LOW);
            digitalWrite(PWM_R, HIGH);
            break;
        case 'R':
            rotation = true;
            digitalWrite(DIR_L, HIGH);
            digitalWrite(PWM_L, HIGH);
            digitalWrite(DIR_R, HIGH);
            digitalWrite(PWM_R, HIGH);
            break;
        default:
            digitalWrite(PWM_L, LOW);
            digitalWrite(PWM_R, LOW);
    }
}

void callback_publisher(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        byte l = digitalRead(IR_L);
        byte m = digitalRead(IR_M);
        byte r = digitalRead(IR_R);

        Serial.print(l);
        Serial.print(m);
        Serial.print(r);
        Serial.println(rotation);

        if (rotation)
        {
            if (m && !l && !r)
            {
                msg_publisher.data = true;
            } else
            {
                msg_publisher.data = false;
            }
        } else
        {
            if (!m || l || r)
            {
                msg_publisher.data = true;
            } else
            {
                msg_publisher.data = false;
            }
        }
        rcl_publish(&publisher, &msg_publisher, NULL);
    }
}

void callback_subscriber(const void *msgin)
{

    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *) msgin;

    char m = 'S';
    if (msg->linear.x > 0)
    {
        m = 'F';
    } else if (msg->linear.x < 0)
    {
        m = 'B';
    } else if (msg->angular.z > 0)
    {
        m = 'R';
    } else if (msg->angular.z < 0)
    {
        m = 'L';
    }

    move(m);

}

void initialize_wifi_connection()
{

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(1000);
    }
    
    Serial.println("WiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    
}

void create_ros_entities()
{

    allocator = rcl_get_default_allocator();

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "esp32_node", "", &support);

    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/milestone"
    );

    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"
    );

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        callback_publisher
    );

    ret = rclc_executor_init(&executor_publisher, &support.context, 1, &allocator);
    if (ret != RCL_RET_OK)
    {
        Serial.println("Error in executor_publisher.");
    }
    rclc_executor_add_timer(&executor_publisher, &timer);

    ret = rclc_executor_init(&executor_subscriber, &support.context, 1, &allocator);
    if (ret != RCL_RET_OK)
    {
        Serial.println("Error in executor_subscriber.");
    }
    rclc_executor_add_subscription(&executor_subscriber, &subscriber, &msg_subscriber, &callback_subscriber, ON_NEW_DATA);

}

void setup()
{

    Serial.begin(115200);
    
    initialize_wifi_connection();

    if (!rmw_uros_ping_agent(10, 1))
    {
        Serial.println("micro_ros_agent not found!");
    }
    else
    {
        Serial.println("micro_ros_agent found!");
    }

    set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);

    create_ros_entities();

    pinMode(IR_L, INPUT);
    pinMode(IR_M, INPUT);
    pinMode(IR_R, INPUT);
    pinMode(DIR_L, 0x02);
    pinMode(DIR_R, 0x02);
    pinMode(PWM_L, 0x02);
    pinMode(PWM_R, 0x02);

    move('S');

}

void loop()
{

    rclc_executor_spin_some(&executor_publisher, RCL_MS_TO_NS(100));
    rclc_executor_spin_some(&executor_subscriber, RCL_MS_TO_NS(10));
    delay(10);

}