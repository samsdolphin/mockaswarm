#include "vicsek.hpp"
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(vicsek, VicsekNode, VicsekNode, nodelet::Nodelet);

Eigen::Vector3d pre_relative_p(0, 0, 0);
double pre_t;
double delta = 0.0;
double H = 1.0;
double beta = 0.25;
double theta = 2;
double d0 = 1;
double d1 = 6;

VicsekNode::VicsekNode()
{
    setName("VicsekNode");
    ROS_WARN("Construct vicsek");
}

VicsekNode::~VicsekNode()
{
}

void VicsekNode::tar_odom_callback(const nav_msgs::Odometry::ConstPtr& data)
{
    tar_position(0) = data->pose.pose.position.x;
    tar_position(1) = data->pose.pose.position.y;
    tar_position(2) = data->pose.pose.position.z;
}

void VicsekNode::run(double dt)
{
    Eigen::Vector3d position, velocity, acceleration, relative_p, f, lambda;
    
    if(pre_relative_p.norm() > 0)
    {
        relative_p = tar_position - readAveragePosition(1);
        double aij = H / pow((1 + pow(relative_p.norm(), 2)), beta);
        double f = 1 / pow(pow(relative_p.norm(), 2) - d0, theta);
        double g = 1 / pow(pow(relative_p.norm(), 2) - d1, theta);
        
        delta = dt - pre_t;
        relative_v = (relative_p - pre_relative_p)/delta;
        double lambda = pow(0.5 * pow(relative_v.norm(), 2), 0.5);
        pre_relative_p = relative_p;
        pre_t = dt;

        acceleration(0) = aij * relative_v(0) - lambda * f * relative_p(0) + lambda * g *relative_p(0);
        acceleration(1) = aij * relative_v(1) - lambda * f * relative_p(1) + lambda * g *relative_p(1);
        
        //acceleration(0) = relative_v(0);
        //acceleration(1) = relative_v(1);
        acceleration(2) = 0;
        
        velocity(0) = readAverageVelocity(1)(0) + acceleration(0) * delta;
        velocity(1) = readAverageVelocity(1)(1) + acceleration(1) * delta;
        velocity(2) = 0;

        position(0) = readAveragePosition(1)(0) + readAverageVelocity(1)(0) * delta + 0.5 * acceleration(0) * pow(delta, 2);
        position(1) = readAveragePosition(1)(1) + readAverageVelocity(1)(1) * delta + 0.5 * acceleration(1) * pow(delta, 2);
        position(2) = readAveragePosition(1)(2);

        publishCommand(position, velocity, acceleration);
    }
    else
    {
        pre_relative_p = tar_position - readAveragePosition(1);
        pre_t = dt;
    }
}

void VicsekNode::clean()
{
    ROS_WARN("vicsek clean");
    MissionTemplateNode::clean();
}

bool VicsekNode::start()
{
    ROS_WARN("vicsek start");
    initialPosition = readAveragePosition(1);
    MissionTemplateNode::start();
    return true;
}

Eigen::Vector3d VicsekNode::getInitialPosition() const
{
    return initialPosition;
}

void VicsekNode::onInit()
{
    ros::NodeHandle nh(getPrivateNodeHandle());
    tar_odom_sub = nh.subscribe("tar_odom", 10, &VicsekNode::tar_odom_callback, this, ros::TransportHints().tcpNoDelay());
    pre_t = 0;
    nh.param("w", w, 0.5);
    nh.param("r", r, 1.0);
    MissionTemplateNode::onInit();
}
