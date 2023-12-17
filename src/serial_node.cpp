#include <chrono>
#include <iostream>
#include <fmt/core.h>

#include "AsioSerial.hpp"
#include "AsioSerial.cpp"

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/range.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"

#include "rclcpp/rclcpp.hpp"

#define SERIAL_PORT_READ_BUF_SIZE 256

using namespace std::chrono_literals;

class SerialCom : public rclcpp::Node
{
public:
    SerialCom() : Node("serial_com_node")
    {
        this->configure();
        pub_sensor = this->create_publisher<sensor_msgs::msg::Range>(sensor_topic, 1000);
        tf_broadcaster =std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        serial.set_callback_read_until(
            boost::bind(&SerialCom::on_serial_rx, this, _1, _2));
        if(!serial.open(port_name_, baudrate_)) {
            RCLCPP_INFO(this->get_logger(), "Error: cannot open port %s", port_name_.c_str());
            return;
        }
        serial.start_async_receive();

        std::cout<<"Initialize timer"<<std::endl;
        timer_ = this->create_wall_timer(
            timer_ms_, std::bind(&SerialCom::timer_callback, this));
    }

private:
    void timer_callback()
    {
        int count = 0;
        rclcpp::Time now = this->get_clock()->now();
        std::string msg = fmt::format("Hello {}!\n\r", this->count_);
        //this->write_some(msg);
        //serial.start_async_write(msg);
        //std::cout<<msg<<std::endl;
        serial.write_some(msg);
        this->count_++;
        if (count != 0)
        {
            //ROS_INFO_ONCE("Data received from serial port.");
            
            //char read_buf[count];
            //num = ser.read((unsigned char*)read_buf, count);
            // char* ch = strtok(read_buf, ",");
            // if(ch!=NULL && read_buf[0] == '$') {
            //   ch = strtok(NULL, ",");
            //   int dataNum = atoi(ch);
            //   //printf("%s,", ch);
            //   int i = 0;
            //   int sonarData[dataNum] = {0,};
            //   for(i=0; i<dataNum; i++) {
            //     ch = strtok(NULL,",");
            //     sonarData[i] = atoi(ch);
            //     //printf("%d,", sonarData[i]);
            //   }
            //   //printf(" EOF\r\n");
            // }

            //printf("%s\r\n", read_buf);
        }
    }

    void on_serial_rx(std::string packet, int bytes_transferred)
    {
        std::cout<<"Read Line:"<<packet<<std::endl;
        (void)bytes_transferred;
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

        this->get_parameter<std::string>    ("port.name",       port_name_);
        this->get_parameter<int>            ("port.baudrate",   baudrate_);
        this->get_parameter<int>            ("sensor.output_hz", output_hz_);
        this->get_parameter<std::string>    ("sensor.topic",     sensor_topic);
        this->get_parameter<bool>           ("tf.enable",       is_pub_tf);
        this->get_parameter<std::string>    ("tf.base_frame",   base_frame);
        this->get_parameter<std::string>    ("tf.sensor_frame",  sensor_frame);
        RCLCPP_INFO(this->get_logger(), "Params:");
        RCLCPP_INFO(this->get_logger(), "\tport.Name: %s",      port_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "\tport.Baudrate: %d",  baudrate_);
        RCLCPP_INFO(this->get_logger(), "\tSENSOR.Ouput_Hz: %d",   output_hz_);
        timer_ms_ = std::chrono::milliseconds {static_cast<long int>(1000 / output_hz_)};
        RCLCPP_INFO(this->get_logger(), "\tMilliseconds: %d",     timer_ms_.count());
        RCLCPP_INFO(this->get_logger(), "\tSENSOR.Topic: %s",     sensor_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "\tTF.Base_link: %s",     base_frame.c_str());
        RCLCPP_INFO(this->get_logger(), "\tTF.Sensor_link: %s",   sensor_frame.c_str());
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr       pub_sensor;
    std::string port_name_;
    int baudrate_;
    int pub_rate_;
    int output_hz_;
    AsioSerial serial;
    int count_;
    bool is_pub_tf;
    std::chrono::milliseconds timer_ms_;
    std::string sensor_frame;
    std::string base_frame;
    std::string sensor_topic;
    std::unique_ptr<tf2_ros::TransformBroadcaster>              tf_broadcaster;
};

int main(int argc, char * argv[])
{
    std::cout << "Starting serial com node..." << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialCom>());
    rclcpp::shutdown();
    return 0;
}
