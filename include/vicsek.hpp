#ifndef CIRCLE_HPP
#define CIRCLE_HPP

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include "mission_template.hpp"

class CircleNode : public MissionTemplateNode
{
public:
    CircleNode();
    virtual ~CircleNode();

public:
    Eigen::Vector3d getInitialPosition() const;

public:
    void onInit();

protected:
    virtual void run(double dt);
    virtual void clean();
    virtual bool start();

private:
    Eigen::Vector3d initialPosition;

    double w;
    double r;
};

#endif // CIRCLE_HPP
