#include "circle.hpp"
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(circle, CircleNode, CircleNode, nodelet::Nodelet);

CircleNode::CircleNode()
{
    setName("CircleNode");
    ROS_WARN("Construct circle");
}

CircleNode::~CircleNode()
{
}

void CircleNode::run(double dt)
{
    Eigen::Vector3d position, velocity, acceleration;
    position(0) = r * std::sin(w * dt) + initialPosition(0);
    position(1) = r * std::cos(w * dt) - r + initialPosition(1);
    position(2) = initialPosition(2);

    velocity(0) = w * r * std::cos(w * dt);
    velocity(1) = -w * r * std::sin(w * dt);
    velocity(2) = 0;

    acceleration(0) = -w * w * r * std::sin(w * dt);
    acceleration(1) = -w * w * r * std::cos(w * dt);
    acceleration(2) = 0;

    publishCommand(position, velocity, acceleration);
}

void CircleNode::clean()
{
    ROS_WARN("circle clean");
    MissionTemplateNode::clean();
}

bool CircleNode::start()
{
    ROS_WARN("circle start");
    initialPosition = readAveragePosition(1);
    MissionTemplateNode::start();
    return true;
}

Eigen::Vector3d CircleNode::getInitialPosition() const
{
    return initialPosition;
}

void CircleNode::onInit()
{
    ros::NodeHandle nh(getPrivateNodeHandle());
    nh.param("w", w, 0.5);
    nh.param("r", r, 1.0);
    MissionTemplateNode::onInit();
}
