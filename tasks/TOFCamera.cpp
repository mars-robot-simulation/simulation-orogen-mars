/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "TOFCamera.hpp"
#include <mars_utils/misc.h>

using namespace mars;

TOFCamera::TOFCamera(std::string const& name)
    : TOFCameraBase(name)
{
}

TOFCamera::~TOFCamera()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See TOFCamera.hpp for more detailed
// documentation about them.

bool TOFCamera::configureHook()
{
    if (! TOFCameraBase::configureHook())
        return false;
    return true;
}
bool TOFCamera::startHook()
{
    if (! TOFCameraBase::startHook())
        return false;
    return true;
}
void TOFCamera::updateHook()
{
    TOFCameraBase::updateHook();
}
void TOFCamera::errorHook()
{
    TOFCameraBase::errorHook();
}
void TOFCamera::stopHook()
{
    TOFCameraBase::stopHook();
}
void TOFCamera::cleanupHook()
{
    TOFCameraBase::cleanupHook();
}

void TOFCamera::getData()
{
    if (camera == nullptr)
    {
        exception(EXCEPTION);
        return;
    }
    if(camera->haveNewData())
    {
        base::samples::Pointcloud pcl;
        base::samples::RigidBodyState rbs;

        std::vector<mars::utils::Vector> colors;

        //pcl.time = base::Time::fromMilliseconds(camera->getImageTime());
        pcl.time = getTime();
        rbs.time = pcl.time;
        rbs.sourceFrame = _camera_frame.value();
        rbs.targetFrame = _world_frame.value();
        camera->getPose(rbs.position, rbs.orientation);
        rbs.cov_orientation = base::Matrix3d::Identity();
        camera->getColoredPointcloud(&(pcl.points), &(pcl.colors));

        // pcl.colors.resize(colors.size());
        // for (uint i = 0; i < pcl.colors.size(); ++i)
        // {
        //     pcl.colors.at(i) = colors.at(i).homogeneous();
        // }

        //utils::msleep(1000);
        _orientation_samples.write( rbs );
        _pointcloud.write(pcl);
    }
}
