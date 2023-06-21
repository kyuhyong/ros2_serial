#include <chrono>
#include <serial/serial.h> 
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/range.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class Ros2SerialNode : public rclcpp::Node
{

public:
    Ros2SerialNode()
    : Node("ros2_serial_node")
    {
        this->configure();
        pub_sensor = this->create_publisher<sensor_msgs::msg::Range>(sensor_topic, 1000);
        tf_broadcaster =std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        try
        {
            ser.setPort(port);
            ser.setBaudrate(baudrate);
            serial::Timeout to = serial::Timeout::simpleTimeout(100);
            ser.setTimeout(to);
            ser.open();
        }
        catch (serial::IOException &e)
        {
            RCLCPP_INFO(this->get_logger(), "Unable to open port ");
            return;
        }
        if (ser.isOpen())
        {
            RCLCPP_INFO(this->get_logger(), "Serial Port initialized");
        }
        else
        {
            return;
        }
        // ser.flush();
        timer_ = this->create_wall_timer(
        timer_ms, std::bind(&Ros2SerialNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        int count = ser.available();
        if (count != 0)
        {
            //ROS_INFO_ONCE("Data received from serial port.");
            int num;
            rclcpp::Time now = this->get_clock()->now();
            char read_buf[count];
            num = ser.read((unsigned char*)read_buf, count);
            char* ch = strtok(read_buf, ",");
            if(ch!=NULL && read_buf[0] == '$') {
              ch = strtok(NULL, ",");
              int dataNum = atoi(ch);
              //printf("%s,", ch);
              int i = 0;
              int sonarData[dataNum] = {0,};
              for(i=0; i<dataNum; i++) {
                ch = strtok(NULL,",");
                sonarData[i] = atoi(ch);
                //printf("%d,", sonarData[i]);
              }
              //printf(" EOF\r\n");
            }

            //printf("%s\r\n", read_buf);
        }
    }
    void configure()
    {
        this->declare_parameter<std::string>("port.name",       "/dev/ttyUSB0");
        this->declare_parameter<int>        ("port.baudrate",   115200);
        this->declare_parameter<std::string>("sensor.topic",    "sensor_data");
        this->declare_parameter<int>        ("sensor.output_hz", 20);
        this->declare_parameter<bool>       ("tf.enable",       false);
        this->declare_parameter<std::string>("tf.base_frame",   "base_link");
        this->declare_parameter<std::string>("tf.sensor_frame", "sensor_link");

        this->get_parameter<std::string>    ("port.name",       port);
        this->get_parameter<int>            ("port.baudrate",   baudrate);
        this->get_parameter<int>            ("sensor.output_hz", output_hz);
        this->get_parameter<std::string>    ("sensor.topic",     sensor_topic);
        this->get_parameter<bool>           ("tf.enable",       is_pub_tf);
        this->get_parameter<std::string>    ("tf.base_frame",   base_frame);
        this->get_parameter<std::string>    ("tf.sensor_frame",  sensor_frame);
        RCLCPP_INFO(this->get_logger(), "Params:");
        RCLCPP_INFO(this->get_logger(), "\tport.Name: %s",      port.c_str());
        RCLCPP_INFO(this->get_logger(), "\tport.Baudrate: %d",  baudrate);
        RCLCPP_INFO(this->get_logger(), "\tSENSOR.Ouput_Hz: %d",   output_hz);
        timer_ms = std::chrono::milliseconds {static_cast<long int>(1000 / output_hz)};
        RCLCPP_INFO(this->get_logger(), "\tMilliseconds: %d",     timer_ms.count());
        RCLCPP_INFO(this->get_logger(), "\tSENSOR.Topic: %s",     sensor_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "\tTF.Base_link: %s",     base_frame.c_str());
        RCLCPP_INFO(this->get_logger(), "\tTF.Sensor_link: %s",   sensor_frame.c_str());

    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr       pub_sensor;
    std::string port;
    int baudrate;
    int pub_rate;
    int output_hz;
    serial::Serial ser;
    bool is_pub_tf;
    std::chrono::milliseconds timer_ms;
    std::string sensor_frame;
    std::string base_frame;
    std::string sensor_topic;
    std::unique_ptr<tf2_ros::TransformBroadcaster>              tf_broadcaster;
};

int main(int argc, char * argv[])
{
  printf("hello world ros2_serial package\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ros2SerialNode>());
  rclcpp::shutdown();
  return 0;
}
