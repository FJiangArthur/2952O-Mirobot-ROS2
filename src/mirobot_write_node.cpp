
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

//#include <string>
//#include <ros/ros.h>
#include <serial/serial.h>
//#include <std_msgs/String.h>
#include <std_msgs/msg/empty.hpp>//Why change to CAPITAL!!!
#include <std_msgs/msg/u_int32.hpp> //WHY??
#include <std_msgs/msg/float32.hpp> //WHY changing naming convension???

//https://docs.ros2.org/foxy/api/sensor_msgs/msg/JointState.html
#include <sensor_msgs/msg/joint_state.hpp>


serial::Serial _serial;				// serial object



class MirobotWriteNode : public rclcpp::Node
{
  public:
    MirobotWriteNode(std::string name) : Node(name)
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 1, std::bind(&MirobotWriteNode::angle_write_callback, this, _1));
    }

  private:
  	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
	  
	void angle_write_callback(const sensor_msgs::msg::JointState& msg)
	{
		
		std::string Gcode = "";
		std_msgs::msg::String result;
		char angle0[10];
		char angle1[10];
		char angle2[10];
		char angle3[10];
		char angle4[10];
		char angle5[10];

		sprintf(angle0, "%.2f", msg.position[0]*57.296);
		sprintf(angle1, "%.2f", msg.position[1]*57.296);
		sprintf(angle2, "%.2f", msg.position[2]*57.296);
		sprintf(angle3, "%.2f", msg.position[3]*57.296);
		sprintf(angle4, "%.2f", msg.position[4]*57.296);
		sprintf(angle5, "%.2f", msg.position[5]*57.296);
		Gcode = (std::string)"M50 G0 X" + angle0 + " Y" + angle1 + " Z" + angle2 + " A" + angle3 + "B" + angle4 + "C" + angle5 + " F3000" + "\r\n";
		RCLCPP_INFO(this->get_logger(),"Art Jiang: %s", Gcode.c_str());
		_serial.write(Gcode.c_str());
		result.data = _serial.read(_serial.available());
	}
	/*
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
	*/
	
};



int main(int argc, char** argv)
{	
	rclcpp::init(argc, argv);//初始化，节点名称 "Mirobot_write_node"
	//ros::NodeHandle nh;
	auto node = std::make_shared<MirobotWriteNode>("mirobot_write_node");
	//ros::Subscriber sub_angle = nh.subscribe("/joint_states", 1, angle_write_callback);//指定节点订阅的话题，并指定回调函数
	//auto sub_angle = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 1, std::bind(&MirobotWriteNode::angle_write_callback, this, _1));
	rclcpp::spin(node);
	rclcpp::Rate loop_rate(20);//指定了频率为20Hz
	rclcpp::Rate sleep_rate(1.0);		
	try//尝试连接机械臂的串口
	{
		_serial.setPort("/dev/ttyUSB0");
		_serial.setBaudrate(115200);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		_serial.setTimeout(to);
		_serial.open();
		_serial.write("M50\r\n");
		RCLCPP_INFO(node->get_logger(), "Port has been open successfully");
	}
	catch (serial::IOException& e)
	{
		RCLCPP_INFO(node->get_logger(), "Unable to open port");
		return -1;
	}
	
	if (_serial.isOpen())
	{
		sleep_rate.sleep();	
		RCLCPP_INFO(node->get_logger(), "Attach and wait for commands");
	}

	while (rclcpp::ok())
	{
		rclcpp::spin_some(node);
		loop_rate.sleep();
	}
	
	return 0;
}


