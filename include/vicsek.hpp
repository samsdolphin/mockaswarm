#ifndef VICSEK_HPP
#define VICSEK_HPP

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include "mission_template.hpp"

class VicsekNode : public MissionTemplateNode
{
public:
    VicsekNode();
    virtual ~VicsekNode();

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

#endif
