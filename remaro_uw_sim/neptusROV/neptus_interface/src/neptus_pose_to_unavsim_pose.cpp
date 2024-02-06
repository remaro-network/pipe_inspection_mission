#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <chrono>

#include <iostream>

constexpr int NWIDTH = 7;
static constexpr int MESSAGE_THROTTLE = 100;

using namespace msr::airlib;
msr::airlib::MultirotorRpcLibClient client;

class NeptusInterface : public rclcpp::Node
{
    public:
        NeptusInterface() 
        : Node("neptus_interface")
        {
            std::string namespace_str = get_namespace();
            local_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            namespace_str + "/from_imc/base_link",  // Replace with the topic name you want to subscribe to
            10,std::bind(&NeptusInterface::local_pose_callback, this, std::placeholders::_1));    
            RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", local_pose_sub->get_topic_name());
        }
    private:
        void local_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            // Get pose
            auto x = msg->pose.position.x;
            auto y = msg->pose.position.y;
            auto z = msg->pose.position.z;
            auto ow = msg->pose.orientation.w;
            auto ox = msg->pose.orientation.x;
            auto oy = msg->pose.orientation.y;
            auto oz = msg->pose.orientation.z;
            // Set pose
            msr::airlib::Vector3r p(x, -y, -z);
            msr::airlib::Quaternionr o(ow, ox, -oy, -oz);
            client.simSetVehiclePose(Pose(p, o), true);
        }
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub;
};

int main(int _argc, char** _argv)
{
    // Connect with UnavSim
    client.confirmConnection();
    // Create ROS node
    rclcpp::init(_argc, _argv);
    rclcpp::spin(std::make_shared<NeptusInterface>());
    rclcpp::shutdown();
    return 0;
}